"""Fuse HSV contours with aligned D435 depth and tf2."""

from __future__ import annotations

import math
from typing import Optional

import cv2
import message_filters
import numpy as np
import rclpy
from builtin_interfaces.msg import Duration as DurationMessage
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from visual_grasp_calibration.geometry import transform_message_to_matrix
from visual_grasp_interfaces.msg import BlockDetection, BlockDetectionArray

from .orientation import box_axis_yaw_in_output
from .segmentation import default_hsv_ranges, detect_color_regions


class ColorCubeDetector(Node):
    def __init__(self) -> None:
        super().__init__('color_cube_detector')
        self.bridge = CvBridge()
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.rgb_topic = self.declare_parameter(
            'rgb_topic', '/camera/camera/color/image_raw').value
        self.depth_topic = self.declare_parameter(
            'depth_topic', '/camera/camera/aligned_depth_to_color/image_raw').value
        self.camera_info_topic = self.declare_parameter(
            'camera_info_topic', '/camera/camera/color/camera_info').value
        self.output_frame = self.declare_parameter(
            'output_frame', 'piper_base_link').value
        self.cube_size_m = float(self.declare_parameter('cube_size_m', 0.030).value)
        self.top_to_center_m = float(self.declare_parameter(
            'top_to_center_m', 0.015).value)
        self.depth_scale = float(self.declare_parameter('depth_scale', 0.001).value)
        self.min_depth_m = float(self.declare_parameter('min_depth_m', 0.12).value)
        self.max_depth_m = float(self.declare_parameter('max_depth_m', 1.20).value)
        self.min_area_px = float(self.declare_parameter('min_area_px', 100.0).value)
        self.max_area_px = float(self.declare_parameter('max_area_px', 30000.0).value)
        self.min_square_ratio = float(self.declare_parameter(
            'min_square_ratio', 0.55).value)
        self.morphology_kernel = int(self.declare_parameter(
            'morphology_kernel', 5).value)
        self.red_hue_upper = int(self.declare_parameter(
            'red_hue_upper', 6).value)
        self.red_hue_wrap_lower = int(self.declare_parameter(
            'red_hue_wrap_lower', 170).value)
        if not 0 <= self.red_hue_upper < self.red_hue_wrap_lower <= 179:
            raise ValueError(
                'Expected 0 <= red_hue_upper < red_hue_wrap_lower <= 179')
        self.hsv_ranges = default_hsv_ranges()
        self.hsv_ranges['red'] = [
            (
                np.array([0, 90, 55]),
                np.array([self.red_hue_upper, 255, 255]),
            ),
            (
                np.array([self.red_hue_wrap_lower, 90, 55]),
                np.array([179, 255, 255]),
            ),
        ]
        self.physical_side_min_m = float(self.declare_parameter(
            'physical_side_min_m', 0.015).value)
        self.physical_side_max_m = float(self.declare_parameter(
            'physical_side_max_m', 0.055).value)
        self.roi = [int(value) for value in self.declare_parameter(
            'roi_xywh', [0, 0, 640, 480]).value]
        self.sync_slop_s = float(self.declare_parameter('sync_slop_s', 0.08).value)
        self.max_latest_tf_age_s = float(self.declare_parameter(
            'max_latest_tf_age_s', 0.7).value)

        self.camera_matrix: Optional[np.ndarray] = None
        self.camera_frame = ''
        self.last_tf_warning_ns = 0

        self.detections_publisher = self.create_publisher(
            BlockDetectionArray, '/visual_grasp/blocks', 10)
        self.debug_publisher = self.create_publisher(
            Image, '/visual_grasp/debug_image', qos_profile_sensor_data)
        self.marker_publisher = self.create_publisher(
            MarkerArray, '/visual_grasp/block_markers', 10)
        self.create_subscription(
            CameraInfo, self.camera_info_topic, self._camera_info_callback,
            qos_profile_sensor_data)

        self.rgb_subscriber = message_filters.Subscriber(
            self, Image, self.rgb_topic, qos_profile=qos_profile_sensor_data)
        self.depth_subscriber = message_filters.Subscriber(
            self, Image, self.depth_topic, qos_profile=qos_profile_sensor_data)
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_subscriber, self.depth_subscriber],
            queue_size=10,
            slop=self.sync_slop_s,
        )
        self.synchronizer.registerCallback(self._synchronized_callback)
        self.get_logger().info(
            f'Detecting {self.cube_size_m * 1000.0:.0f} mm red/green/blue cubes; '
            f'output frame={self.output_frame}')

    def _camera_info_callback(self, message: CameraInfo) -> None:
        self.camera_matrix = np.asarray(message.k, dtype=np.float64).reshape(3, 3)
        self.camera_frame = message.header.frame_id

    def _synchronized_callback(self, rgb_message: Image, depth_message: Image) -> None:
        if self.camera_matrix is None:
            return
        bgr = self.bridge.imgmsg_to_cv2(rgb_message, desired_encoding='bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_message, desired_encoding='passthrough')
        depth = np.asarray(depth)
        if depth.shape[:2] != bgr.shape[:2]:
            self.get_logger().error('Aligned depth resolution does not match RGB')
            return

        height, width = bgr.shape[:2]
        roi_x, roi_y, roi_width, roi_height = self.roi
        roi_x = int(np.clip(roi_x, 0, width - 1))
        roi_y = int(np.clip(roi_y, 0, height - 1))
        roi_width = int(np.clip(roi_width, 1, width - roi_x))
        roi_height = int(np.clip(roi_height, 1, height - roi_y))
        cropped = bgr[roi_y:roi_y + roi_height, roi_x:roi_x + roi_width]
        regions, _ = detect_color_regions(
            cropped,
            min_area_px=self.min_area_px,
            max_area_px=self.max_area_px,
            min_square_ratio=self.min_square_ratio,
            morphology_kernel=self.morphology_kernel,
            hsv_ranges=self.hsv_ranges,
        )

        camera_frame = rgb_message.header.frame_id or self.camera_frame
        transform_matrix = self._lookup_transform(
            camera_frame, rgb_message.header.stamp)
        output = BlockDetectionArray()
        output.header = rgb_message.header
        output.header.frame_id = self.output_frame
        debug = bgr.copy()

        if transform_matrix is not None:
            for region in regions:
                region.contour[:, 0, 0] += roi_x
                region.contour[:, 0, 1] += roi_y
                region.box[:, 0] += roi_x
                region.box[:, 1] += roi_y
                center_u = region.center_uv[0] + roi_x
                center_v = region.center_uv[1] + roi_y
                detection = self._build_detection(
                    region, center_u, center_v, depth, transform_matrix,
                    output.header)
                if detection is None:
                    continue
                output.detections.append(detection)
                self._draw_detection(debug, region, detection, center_u, center_v)

        self.detections_publisher.publish(output)
        self._publish_markers(output)
        debug_message = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
        debug_message.header = rgb_message.header
        self.debug_publisher.publish(debug_message)

    def _lookup_transform(self, camera_frame: str, stamp_message) -> Optional[np.ndarray]:
        image_time = Time.from_msg(stamp_message)
        try:
            transform = self.tf_buffer.lookup_transform(
                self.output_frame,
                camera_frame,
                image_time,
                timeout=Duration(seconds=0.08),
            )
            return transform_message_to_matrix(transform)
        except TransformException as error:
            # PiPER's CAN feedback stamp can trail the D435 image stamp by a
            # few hundred milliseconds.  During eye-in-hand detection the arm
            # must be stationary, so a recent latest transform is preferable
            # to dropping every frame on future extrapolation.
            try:
                latest = self.tf_buffer.lookup_transform(
                    self.output_frame,
                    camera_frame,
                    Time(),
                    timeout=Duration(seconds=0.02),
                )
                latest_time = Time.from_msg(latest.header.stamp)
                age_s = abs(
                    image_time.nanoseconds - latest_time.nanoseconds) / 1e9
                if age_s <= self.max_latest_tf_age_s:
                    return transform_message_to_matrix(latest)
            except TransformException:
                pass
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self.last_tf_warning_ns > 2_000_000_000:
                self.get_logger().warning(
                    f'Cannot transform {camera_frame} -> {self.output_frame} '
                    f'at image time, and no latest TF newer than '
                    f'{self.max_latest_tf_age_s:.2f} s is available: {error}')
                self.last_tf_warning_ns = now_ns
            return None

    def _build_detection(
        self, region, center_u: float, center_v: float, depth: np.ndarray,
        output_T_camera: np.ndarray, header,
    ) -> Optional[BlockDetection]:
        mask = np.zeros(depth.shape[:2], dtype=np.uint8)
        cv2.drawContours(mask, [region.contour], -1, 255, thickness=cv2.FILLED)
        mask = cv2.erode(mask, np.ones((5, 5), dtype=np.uint8), iterations=1)
        values = depth[mask > 0].astype(np.float64) * self.depth_scale
        values = values[(values >= self.min_depth_m) & (values <= self.max_depth_m)]
        if values.size < 20:
            return None
        depth_m = float(np.median(values))

        fx = float(self.camera_matrix[0, 0])
        fy = float(self.camera_matrix[1, 1])
        cx = float(self.camera_matrix[0, 2])
        cy = float(self.camera_matrix[1, 2])
        camera_point = np.array([
            (center_u - cx) * depth_m / fx,
            (center_v - cy) * depth_m / fy,
            depth_m,
            1.0,
        ])
        output_top = output_T_camera @ camera_point
        output_center = output_top[:3].copy()
        # piper_base_link Z is assumed vertical and the table is horizontal.
        output_center[2] -= self.top_to_center_m

        edges = [
            float(np.linalg.norm(region.box[(index + 1) % 4] - region.box[index]))
            for index in range(4)
        ]
        side_px = float(np.median(edges))
        estimated_side_m = side_px * depth_m / ((fx + fy) * 0.5)
        if not self.physical_side_min_m <= estimated_side_m <= self.physical_side_max_m:
            return None
        size_error = abs(estimated_side_m - self.cube_size_m) / self.cube_size_m
        physical_score = float(math.exp(-1.5 * size_error))
        try:
            object_axis_yaw = box_axis_yaw_in_output(
                region.box,
                fx,
                fy,
                output_T_camera[:3, :3],
            )
        except ValueError as error:
            self.get_logger().warning(
                f'Cannot recover block orientation: {error}')
            return None

        detection = BlockDetection()
        detection.header = header
        detection.color = region.color
        detection.pose = Pose()
        detection.pose.position.x = float(output_center[0])
        detection.pose.position.y = float(output_center[1])
        detection.pose.position.z = float(output_center[2])
        detection.pose.orientation.z = math.sin(0.5 * object_axis_yaw)
        detection.pose.orientation.w = math.cos(0.5 * object_axis_yaw)
        detection.size.x = self.cube_size_m
        detection.size.y = self.cube_size_m
        detection.size.z = self.cube_size_m
        detection.confidence = float(0.65 * region.score + 0.35 * physical_score)
        detection.depth_m = depth_m
        detection.image_angle_rad = math.radians(region.angle_deg)
        detection.contour_area_px = region.area_px
        return detection

    @staticmethod
    def _draw_detection(debug, region, detection, center_u, center_v) -> None:
        colors = {
            'red': (0, 0, 255),
            'green': (0, 255, 0),
            'blue': (255, 0, 0),
        }
        color = colors.get(region.color, (255, 255, 255))
        box = np.int32(np.round(region.box))
        cv2.polylines(debug, [box], True, color, 2, cv2.LINE_AA)
        cv2.circle(debug, (int(round(center_u)), int(round(center_v))), 4, color, -1)
        base_yaw_deg = math.degrees(2.0 * math.atan2(
            detection.pose.orientation.z,
            detection.pose.orientation.w,
        ))
        text = (
            f'{region.color} {detection.depth_m:.3f}m '
            f'conf={detection.confidence:.2f} yaw={base_yaw_deg:.1f}deg')
        cv2.putText(
            debug, text, (box[0, 0], max(18, box[0, 1] - 8)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.48, color, 2, cv2.LINE_AA)

    def _publish_markers(self, detections: BlockDetectionArray) -> None:
        marker_array = MarkerArray()
        colors = {
            'red': (1.0, 0.05, 0.05),
            'green': (0.05, 1.0, 0.05),
            'blue': (0.05, 0.05, 1.0),
        }
        for index, detection in enumerate(detections.detections):
            marker = Marker()
            marker.header = detections.header
            marker.ns = 'detected_cubes'
            marker.id = index
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose = detection.pose
            marker.scale = detection.size
            red, green, blue = colors.get(detection.color, (1.0, 1.0, 1.0))
            marker.color.r = red
            marker.color.g = green
            marker.color.b = blue
            marker.color.a = 0.75
            marker.lifetime = DurationMessage(sec=0, nanosec=300_000_000)
            marker_array.markers.append(marker)
        self.marker_publisher.publish(marker_array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ColorCubeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
