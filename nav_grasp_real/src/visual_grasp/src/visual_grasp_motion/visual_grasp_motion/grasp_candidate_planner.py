"""Generate inspectable, table-referenced poses without commanding the arm."""

from __future__ import annotations

import copy
import math

import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from visual_grasp_interfaces.msg import BlockDetectionArray

from .motion_math import (
    horizontal_axis_yaw,
    normalize_quaternion,
    plane_height_at_xy,
    rotate_orientation_about_base_z,
    square_alignment_delta,
)


class GraspCandidatePlanner(Node):
    def __init__(self) -> None:
        super().__init__('grasp_candidate_planner')
        self.target_color = self.declare_parameter('target_color', 'any').value
        self.pregrasp_height_m = float(self.declare_parameter(
            'pregrasp_height_m', 0.080).value)
        self.lift_height_m = float(self.declare_parameter(
            'lift_height_m', 0.100).value)
        self.max_detection_age_s = float(self.declare_parameter(
            'max_detection_age_s', 0.5).value)
        self.min_confidence = float(self.declare_parameter(
            'min_confidence', 0.75).value)
        self.height_source = str(self.declare_parameter(
            'height_source', 'depth_center').value).strip().lower()
        if self.height_source not in {'depth_center', 'fixed_plane'}:
            raise ValueError(
                "height_source must be 'depth_center' or 'fixed_plane'")
        self.support_plane_z_m = float(self.declare_parameter(
            'support_plane_z_m', -0.004454).value)
        self.support_plane_reference_x_m = float(self.declare_parameter(
            'support_plane_reference_x_m', 0.0).value)
        self.support_plane_reference_y_m = float(self.declare_parameter(
            'support_plane_reference_y_m', 0.0).value)
        self.support_plane_dz_dx = float(self.declare_parameter(
            'support_plane_dz_dx', 0.0).value)
        self.support_plane_dz_dy = float(self.declare_parameter(
            'support_plane_dz_dy', 0.0).value)
        self.cube_height_m = float(self.declare_parameter(
            'cube_height_m', 0.030).value)
        self.grasp_center_z_offset_m = float(self.declare_parameter(
            'grasp_center_z_offset_m', 0.0).value)
        self.target_offset_x_m = float(self.declare_parameter(
            'target_offset_x_m', 0.0).value)
        self.target_offset_y_m = float(self.declare_parameter(
            'target_offset_y_m', 0.0).value)
        self.workspace_min = list(self.declare_parameter(
            'workspace_min_xyz_m', [0.10, -0.45, -0.12]).value)
        self.workspace_max = list(self.declare_parameter(
            'workspace_max_xyz_m', [0.65, 0.45, 0.60]).value)
        self.tool_down_quaternion = normalize_quaternion(self.declare_parameter(
            'tool_down_quaternion_xyzw', [0.0, 1.0, 0.0, 0.0]).value)
        self.align_grasp_to_block = bool(self.declare_parameter(
            'align_grasp_to_block', True).value)
        self.grasp_yaw_offset_rad = math.radians(float(self.declare_parameter(
            'grasp_yaw_offset_deg', 0.0).value))
        self.straighten_after_lift = bool(self.declare_parameter(
            'straighten_after_lift', True).value)
        self.nominal_tool_axis_yaw = horizontal_axis_yaw(
            self.tool_down_quaternion)

        self.latest: BlockDetectionArray | None = None
        self.create_subscription(
            BlockDetectionArray, '/visual_grasp/blocks', self._detections_callback, 10)
        self.sequence_publisher = self.create_publisher(
            PoseArray, '/visual_grasp/pick_sequence', 10)
        self.selected_publisher = self.create_publisher(
            PoseStamped, '/visual_grasp/selected_block', 10)
        self.marker_publisher = self.create_publisher(
            MarkerArray, '/visual_grasp/pick_markers', 10)
        self.create_service(Trigger, '/visual_grasp/plan_pick', self._plan_pick)
        self.get_logger().info(
            'Planning only: execution is handled by the separately guarded '
            'piper_pick_executor node')

    def _detections_callback(self, message: BlockDetectionArray) -> None:
        self.latest = message

    def _plan_pick(self, _request, response):
        if self.latest is None or not self.latest.detections:
            response.success = False
            response.message = 'No cube detections available'
            return response
        stamp = Time.from_msg(self.latest.header.stamp)
        age_s = abs(self.get_clock().now().nanoseconds - stamp.nanoseconds) / 1e9
        if age_s > self.max_detection_age_s:
            response.success = False
            response.message = f'Detections are stale ({age_s:.2f} s)'
            return response

        candidates = [
            detection for detection in self.latest.detections
            if (self.target_color == 'any' or detection.color == self.target_color)
            and detection.confidence >= self.min_confidence
            and self._detection_inside_workspace(detection.pose)
        ]
        if not candidates:
            response.success = False
            response.message = (
                f'No {self.target_color} detection inside configured workspace')
            return response
        selected = max(candidates, key=lambda item: item.confidence)

        grasp = copy.deepcopy(selected.pose)
        # Correct a measured, repeatable planar hand-eye/TCP bias in the base
        # frame.  Apply it to the grasp before deriving pregrasp and lift so
        # all three poses remain on one vertical line.
        grasp.position.x += self.target_offset_x_m
        grasp.position.y += self.target_offset_y_m
        if self.height_source == 'fixed_plane':
            detected_height = float(selected.size.z)
            cube_height = detected_height if detected_height > 0.0 else self.cube_height_m
            support_z = plane_height_at_xy(
                grasp.position.x,
                grasp.position.y,
                (
                    self.support_plane_reference_x_m,
                    self.support_plane_reference_y_m,
                    self.support_plane_z_m,
                ),
                (self.support_plane_dz_dx, self.support_plane_dz_dy),
            )
            grasp.position.z = (
                support_z + 0.5 * cube_height
                + self.grasp_center_z_offset_m)
        else:
            # The RGB-D detector publishes the cube centre, not the observed
            # top surface: it transforms the measured top point into the base
            # frame and subtracts top_to_center_m along base Z.  Use that
            # per-detection height so the trolley can work on changing support
            # surfaces without inheriting the calibration board/box height.
            grasp.position.z += self.grasp_center_z_offset_m
        yaw_adjustment = 0.0
        grasp_quaternion = self.tool_down_quaternion
        if self.align_grasp_to_block:
            object_axis_yaw = horizontal_axis_yaw((
                selected.pose.orientation.x,
                selected.pose.orientation.y,
                selected.pose.orientation.z,
                selected.pose.orientation.w,
            ))
            yaw_adjustment = square_alignment_delta(
                object_axis_yaw,
                self.nominal_tool_axis_yaw,
                self.grasp_yaw_offset_rad,
            )
            grasp_quaternion = rotate_orientation_about_base_z(
                self.tool_down_quaternion, yaw_adjustment)
        self._set_orientation(grasp, grasp_quaternion)
        if not self._inside_workspace(grasp):
            response.success = False
            response.message = (
                'Corrected grasp target is outside configured workspace: '
                f'({grasp.position.x:.4f}, {grasp.position.y:.4f}, '
                f'{grasp.position.z:.4f}) m')
            return response
        pregrasp = copy.deepcopy(grasp)
        pregrasp.position.z += self.pregrasp_height_m
        lift = copy.deepcopy(grasp)
        lift.position.z += self.lift_height_m
        straighten = copy.deepcopy(lift)
        if self.straighten_after_lift:
            self._set_orientation(straighten, self.tool_down_quaternion)

        sequence = PoseArray()
        sequence.header = self.latest.header
        sequence.poses = [pregrasp, grasp, lift, straighten]
        self.sequence_publisher.publish(sequence)

        selected_message = PoseStamped()
        selected_message.header = self.latest.header
        selected_message.pose = selected.pose
        self.selected_publisher.publish(selected_message)
        self._publish_markers(sequence)

        response.success = True
        response.message = (
            f'Selected {selected.color}, confidence={selected.confidence:.2f}; '
            f'grasp TCP=({grasp.position.x:.4f}, {grasp.position.y:.4f}, '
            f'{grasp.position.z:.4f}) m; base XY correction='
            f'({self.target_offset_x_m:+.4f}, '
            f'{self.target_offset_y_m:+.4f}) m; '
            f'grasp yaw adjustment={math.degrees(yaw_adjustment):+.1f} deg; '
            f'height_source={self.height_source}; '
            'published pregrasp/grasp/lift/straighten only')
        return response

    @staticmethod
    def _set_orientation(pose: Pose, quaternion) -> None:
        (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ) = (float(value) for value in quaternion)

    def _inside_workspace(self, pose: Pose) -> bool:
        coordinates = [pose.position.x, pose.position.y, pose.position.z]
        return all(
            float(minimum) <= value <= float(maximum)
            for value, minimum, maximum in zip(
                coordinates, self.workspace_min, self.workspace_max)
        )

    def _detection_inside_workspace(self, pose: Pose) -> bool:
        # A fixed plane deliberately replaces RGB-D Z, so only X/Y is checked
        # before the corrected grasp pose gets a full XYZ check.  Dynamic
        # depth mode validates the measured centre Z immediately.
        if self.height_source == 'fixed_plane':
            return all(
                float(minimum) <= value <= float(maximum)
                for value, minimum, maximum in zip(
                    (pose.position.x, pose.position.y),
                    self.workspace_min[:2], self.workspace_max[:2])
            )
        return self._inside_workspace(pose)

    def _publish_markers(self, sequence: PoseArray) -> None:
        marker_array = MarkerArray()
        labels = ['pregrasp', 'grasp', 'lift', 'straighten']
        colors = [
            (1.0, 0.8, 0.0),
            (1.0, 0.1, 1.0),
            (0.0, 1.0, 1.0),
            (0.2, 1.0, 0.2),
        ]
        for index, (pose, label, color) in enumerate(zip(
                sequence.poses, labels, colors)):
            marker = Marker()
            marker.header = sequence.header
            marker.ns = 'pick_sequence'
            marker.id = index
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose = pose
            marker.scale.x = 0.08
            marker.scale.y = 0.012
            marker.scale.z = 0.012
            marker.color.r, marker.color.g, marker.color.b = color
            marker.color.a = 0.9
            marker.text = label
            marker_array.markers.append(marker)
        self.marker_publisher.publish(marker_array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GraspCandidatePlanner()
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
