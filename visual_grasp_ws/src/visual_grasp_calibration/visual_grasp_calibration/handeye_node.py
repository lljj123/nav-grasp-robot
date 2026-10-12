"""Detect TAG36H11-3 and collect/solve eye-in-hand calibration samples."""

from __future__ import annotations

from collections import deque
from pathlib import Path
from typing import Dict, Optional

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .geometry import (
    matrix_to_xyz_quaternion,
    make_transform,
    transform_message_to_matrix,
)
from .solver import best_solution, motion_diversity
from .stability import pose_motion_span


class HandEyeCalibrator(Node):
    def __init__(self) -> None:
        super().__init__('handeye_calibrator')
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.image_topic = self.declare_parameter(
            'image_topic', '/camera/camera/color/image_raw').value
        self.camera_info_topic = self.declare_parameter(
            'camera_info_topic', '/camera/camera/color/camera_info').value
        self.base_frame = self.declare_parameter(
            'base_frame', 'piper_base_link').value
        self.flange_frame = self.declare_parameter(
            'flange_frame', 'piper_link6').value
        self.camera_link_frame = self.declare_parameter(
            'camera_link_frame', 'camera_link').value
        self.camera_optical_frame = self.declare_parameter(
            'camera_optical_frame', 'camera_color_optical_frame').value
        self.tag_id = int(self.declare_parameter('tag_id', 3).value)
        self.tag_size_m = float(self.declare_parameter('tag_size_m', 0.035).value)
        self.min_samples = int(self.declare_parameter('min_samples', 15).value)
        self.max_detection_age_s = float(self.declare_parameter(
            'max_detection_age_s', 1.0).value)
        self.max_reprojection_error_px = float(self.declare_parameter(
            'max_reprojection_error_px', 2.0).value)
        self.stability_duration_s = float(self.declare_parameter(
            'stability_duration_s', 1.0).value)
        self.stability_translation_m = float(self.declare_parameter(
            'stability_translation_m', 0.001).value)
        self.stability_rotation_deg = float(self.declare_parameter(
            'stability_rotation_deg', 0.5).value)
        self.max_flange_pose_age_s = float(self.declare_parameter(
            'max_flange_pose_age_s', 0.5).value)
        self.dataset_path = Path(self.declare_parameter(
            'dataset_path',
            '~/.config/nav_grasp/calibration/handeye_samples.yaml',
        ).value).expanduser()
        self.result_path = Path(self.declare_parameter(
            'result_path',
            '~/.config/nav_grasp/calibration/handeye.yaml',
        ).value).expanduser()

        self.dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)
        self.detector_parameters = cv2.aruco.DetectorParameters_create()
        self.detector_parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        self.camera_matrix: Optional[np.ndarray] = None
        self.distortion: Optional[np.ndarray] = None
        self.latest_observation: Optional[Dict] = None
        self.samples = []
        self.flange_pose_history = deque()
        self._load_existing_samples()

        self.pose_publisher = self.create_publisher(
            PoseStamped, '/handeye/tag_pose', 10)
        self.debug_publisher = self.create_publisher(
            Image, '/handeye/debug_image', qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, self.camera_info_topic, self._camera_info_callback,
            qos_profile_sensor_data)
        self.create_subscription(
            Image, self.image_topic, self._image_callback, qos_profile_sensor_data)
        self.create_service(
            Trigger, '/handeye/capture_sample', self._capture_sample)
        self.create_service(Trigger, '/handeye/solve', self._solve)
        self.create_service(Trigger, '/handeye/reset', self._reset)
        self.stability_timer = self.create_timer(0.05, self._track_flange_pose)

        self.get_logger().info(
            f'Waiting for tag36h11 id={self.tag_id}, detection size={self.tag_size_m:.3f} m; '
            f'loaded {len(self.samples)} samples; capture requires '
            f'{self.stability_duration_s:.1f} s stable pose within '
            f'{self.stability_translation_m * 1000.0:.1f} mm / '
            f'{self.stability_rotation_deg:.2f} deg')

    def _camera_info_callback(self, message: CameraInfo) -> None:
        self.camera_matrix = np.asarray(message.k, dtype=np.float64).reshape(3, 3)
        self.distortion = np.asarray(message.d, dtype=np.float64)

    def _image_callback(self, message: Image) -> None:
        if self.camera_matrix is None or self.distortion is None:
            return
        bgr = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.dictionary, parameters=self.detector_parameters)

        if ids is None or self.tag_id not in ids.reshape(-1).tolist():
            self.latest_observation = None
            self._publish_debug(message, bgr)
            return

        index = ids.reshape(-1).tolist().index(self.tag_id)
        image_points = np.asarray(corners[index], dtype=np.float64).reshape(4, 2)
        half = self.tag_size_m * 0.5
        object_points = np.asarray([
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ], dtype=np.float64)

        result = cv2.solvePnPGeneric(
            object_points,
            image_points,
            self.camera_matrix,
            self.distortion,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        valid, rvecs, tvecs = result[:3]
        errors = result[3] if len(result) > 3 else None
        if not valid or len(rvecs) == 0:
            self.latest_observation = None
            self._publish_debug(message, bgr)
            return

        error_values = (
            np.asarray(errors, dtype=np.float64).reshape(-1)
            if errors is not None else np.full(len(rvecs), np.inf))
        candidates = []
        for candidate_index, (rvec, tvec) in enumerate(zip(rvecs, tvecs)):
            translation = np.asarray(tvec, dtype=np.float64).reshape(3)
            if translation[2] <= 0.0:
                continue
            rotation, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64))
            candidates.append((
                float(error_values[candidate_index]),
                make_transform(rotation, translation),
                rvec,
                tvec,
            ))
        if not candidates:
            self.latest_observation = None
            self._publish_debug(message, bgr)
            return

        reprojection_error, camera_T_tag, rvec, tvec = min(
            candidates, key=lambda item: item[0])
        if reprojection_error > self.max_reprojection_error_px:
            self.latest_observation = None
        else:
            self.latest_observation = {
                'stamp': message.header.stamp,
                'stamp_ns': Time.from_msg(message.header.stamp).nanoseconds,
                'camera_T_tag': camera_T_tag,
                'reprojection_error_px': reprojection_error,
                'image_frame': message.header.frame_id,
            }
            self._publish_tag_pose(message, camera_T_tag)

        cv2.aruco.drawDetectedMarkers(bgr, [corners[index]], np.asarray([[self.tag_id]]))
        cv2.drawFrameAxes(
            bgr, self.camera_matrix, self.distortion, rvec, tvec,
            self.tag_size_m * 0.5)
        cv2.putText(
            bgr, f'id={self.tag_id} err={reprojection_error:.2f}px', (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2, cv2.LINE_AA)
        self._publish_debug(message, bgr)

    def _publish_tag_pose(self, image: Image, camera_T_tag: np.ndarray) -> None:
        translation, quaternion = matrix_to_xyz_quaternion(camera_T_tag)
        pose = PoseStamped()
        pose.header.stamp = image.header.stamp
        pose.header.frame_id = image.header.frame_id or self.camera_optical_frame
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = translation
        pose.pose.orientation.x, pose.pose.orientation.y = quaternion[:2]
        pose.pose.orientation.z, pose.pose.orientation.w = quaternion[2:]
        self.pose_publisher.publish(pose)

    def _publish_debug(self, source: Image, bgr: np.ndarray) -> None:
        message = self.bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
        message.header = source.header
        self.debug_publisher.publish(message)

    def _track_flange_pose(self) -> None:
        """Keep recent, uniquely stamped flange poses for a capture guard."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.flange_frame,
                Time(),
                timeout=Duration(seconds=0.02),
            )
        except TransformException:
            return

        source_stamp_ns = Time.from_msg(transform.header.stamp).nanoseconds
        if (self.flange_pose_history
                and source_stamp_ns == self.flange_pose_history[-1][1]):
            return

        now_ns = self.get_clock().now().nanoseconds
        self.flange_pose_history.append((
            now_ns,
            source_stamp_ns,
            transform_message_to_matrix(transform),
        ))
        keep_after_ns = now_ns - int((self.stability_duration_s + 1.0) * 1e9)
        while (self.flange_pose_history
               and self.flange_pose_history[0][0] < keep_after_ns):
            self.flange_pose_history.popleft()

    def _stable_flange_pose(self):
        """Return stability status and a user-facing diagnostic."""
        if not self.flange_pose_history:
            return False, (
                f'No live TF for {self.base_frame} -> {self.flange_frame}')

        now_ns = self.get_clock().now().nanoseconds
        newest_age_s = (now_ns - self.flange_pose_history[-1][0]) / 1e9
        if newest_age_s > self.max_flange_pose_age_s:
            return False, f'Flange pose feedback is stale ({newest_age_s:.2f} s)'

        cutoff_ns = now_ns - int(self.stability_duration_s * 1e9)
        if self.flange_pose_history[0][0] > cutoff_ns:
            covered_s = (
                self.flange_pose_history[-1][0]
                - self.flange_pose_history[0][0]) / 1e9
            return False, (
                f'Waiting for stability history '
                f'({covered_s:.2f}/{self.stability_duration_s:.2f} s)')

        window = [
            item[2] for item in self.flange_pose_history
            if item[0] >= cutoff_ns
        ]
        if len(window) < 3:
            return False, 'Too few live flange poses to verify stability'

        translation_span_m, rotation_span_deg = pose_motion_span(window)
        if (translation_span_m > self.stability_translation_m
                or rotation_span_deg > self.stability_rotation_deg):
            return False, (
                f'Robot is not stable over {self.stability_duration_s:.1f} s: '
                f'motion={translation_span_m * 1000.0:.2f} mm / '
                f'{rotation_span_deg:.3f} deg; wait, then press Enter again')
        return True, (
            f'stable {self.stability_duration_s:.1f} s, '
            f'motion={translation_span_m * 1000.0:.2f} mm / '
            f'{rotation_span_deg:.3f} deg')

    def _capture_sample(self, _request, response):
        observation = self.latest_observation
        if observation is None:
            response.success = False
            response.message = 'No valid tag detection; inspect /handeye/debug_image'
            return response

        age_s = abs(self.get_clock().now().nanoseconds - observation['stamp_ns']) / 1e9
        if age_s > self.max_detection_age_s:
            response.success = False
            response.message = f'Tag detection is stale ({age_s:.2f} s)'
            return response

        stable, stability_message = self._stable_flange_pose()
        if not stable:
            response.success = False
            response.message = stability_message
            return response

        image_time = Time.from_msg(observation['stamp'])
        used_latest_tf = False
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.flange_frame,
                image_time,
                timeout=Duration(seconds=0.3),
            )
        except TransformException:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    self.flange_frame,
                    Time(),
                    timeout=Duration(seconds=0.3),
                )
                used_latest_tf = True
            except TransformException as error:
                response.success = False
                response.message = (
                    f'Cannot query {self.base_frame} -> {self.flange_frame}: {error}')
                return response

        self.samples.append({
            'stamp_ns': int(observation['stamp_ns']),
            'base_T_flange': transform_message_to_matrix(transform),
            'camera_T_tag': observation['camera_T_tag'].copy(),
            'reprojection_error_px': float(observation['reprojection_error_px']),
            'used_latest_tf': used_latest_tf,
        })
        self._write_samples()
        response.success = True
        response.message = (
            f'Captured sample {len(self.samples)}; reprojection '
            f'{observation["reprojection_error_px"]:.3f} px; '
            f'{stability_message}'
            + ('; used latest TF because exact timestamp was unavailable'
               if used_latest_tf else ''))
        return response

    def _solve(self, _request, response):
        if len(self.samples) < self.min_samples:
            response.success = False
            response.message = (
                f'Need at least {self.min_samples} samples; have {len(self.samples)}')
            return response

        base_T_flange = [item['base_T_flange'] for item in self.samples]
        camera_T_tag = [item['camera_T_tag'] for item in self.samples]
        try:
            selected, solutions = best_solution(base_T_flange, camera_T_tag)
        except (ValueError, RuntimeError, cv2.error) as error:
            response.success = False
            response.message = f'Hand-eye solve failed: {error}'
            return response

        try:
            internal = self.tf_buffer.lookup_transform(
                self.camera_link_frame,
                self.camera_optical_frame,
                Time(),
                timeout=Duration(seconds=1.0),
            )
        except TransformException as error:
            response.success = False
            response.message = (
                f'Solved optical transform, but D435 internal TF '
                f'{self.camera_link_frame} -> {self.camera_optical_frame} is unavailable: {error}')
            return response

        camera_link_T_optical = transform_message_to_matrix(internal)
        flange_T_camera_link = (
            selected.flange_T_camera @ np.linalg.inv(camera_link_T_optical))
        link_translation, link_quaternion = matrix_to_xyz_quaternion(
            flange_T_camera_link)
        optical_translation, optical_quaternion = matrix_to_xyz_quaternion(
            selected.flange_T_camera)
        translation_span_m, rotation_span_deg = motion_diversity(base_T_flange)

        result = {
            'handeye': {
                'valid': True,
                'mode': 'eye_in_hand',
                'sample_count': len(self.samples),
                'tag': {
                    'family': 'tag36h11',
                    'id': self.tag_id,
                    'size_m': self.tag_size_m,
                },
                'parent_frame': self.flange_frame,
                'child_frame': self.camera_link_frame,
                'camera_optical_frame': self.camera_optical_frame,
                'selected_method': selected.method,
                'transform_flange_to_camera_link': {
                    'translation_m': link_translation.tolist(),
                    'rotation_xyzw': link_quaternion.tolist(),
                },
                'transform_flange_to_camera_optical': {
                    'translation_m': optical_translation.tolist(),
                    'rotation_xyzw': optical_quaternion.tolist(),
                },
                'quality': {
                    'translation_rms_m': selected.translation_rms_m,
                    'rotation_rms_deg': selected.rotation_rms_deg,
                    'robot_translation_span_m': translation_span_m,
                    'robot_rotation_span_deg': rotation_span_deg,
                    'mean_reprojection_error_px': float(np.mean([
                        item['reprojection_error_px'] for item in self.samples
                    ])),
                },
                'candidate_methods': {
                    name: {
                        'translation_rms_m': value.translation_rms_m,
                        'rotation_rms_deg': value.rotation_rms_deg,
                        'score': value.score,
                    }
                    for name, value in solutions.items()
                },
            },
        }
        self.result_path.parent.mkdir(parents=True, exist_ok=True)
        self.result_path.write_text(
            yaml.safe_dump(result, sort_keys=False, allow_unicode=True),
            encoding='utf-8')

        response.success = True
        response.message = (
            f'Saved {self.result_path}; method={selected.method}, '
            f'translation RMS={selected.translation_rms_m * 1000.0:.2f} mm, '
            f'rotation RMS={selected.rotation_rms_deg:.3f} deg')
        return response

    def _reset(self, _request, response):
        self.samples.clear()
        if self.dataset_path.exists():
            self.dataset_path.unlink()
        response.success = True
        response.message = 'All in-memory and saved hand-eye samples were removed'
        return response

    def _write_samples(self) -> None:
        serializable = {
            'format_version': 1,
            'base_frame': self.base_frame,
            'flange_frame': self.flange_frame,
            'camera_optical_frame': self.camera_optical_frame,
            'tag_family': 'tag36h11',
            'tag_id': self.tag_id,
            'tag_size_m': self.tag_size_m,
            'samples': [{
                'stamp_ns': item['stamp_ns'],
                'base_T_flange': item['base_T_flange'].tolist(),
                'camera_T_tag': item['camera_T_tag'].tolist(),
                'reprojection_error_px': item['reprojection_error_px'],
                'used_latest_tf': item['used_latest_tf'],
            } for item in self.samples],
        }
        self.dataset_path.parent.mkdir(parents=True, exist_ok=True)
        self.dataset_path.write_text(
            yaml.safe_dump(serializable, sort_keys=False, allow_unicode=True),
            encoding='utf-8')

    def _load_existing_samples(self) -> None:
        if not self.dataset_path.exists():
            return
        try:
            data = yaml.safe_load(self.dataset_path.read_text(encoding='utf-8')) or {}
            if (data.get('tag_id') != self.tag_id or
                    abs(float(data.get('tag_size_m', 0.0)) - self.tag_size_m) > 1e-9 or
                    data.get('base_frame') != self.base_frame or
                    data.get('flange_frame') != self.flange_frame):
                self.get_logger().warning(
                    'Existing hand-eye dataset metadata does not match current '
                    'parameters; ignored')
                return
            for item in data.get('samples', []):
                self.samples.append({
                    'stamp_ns': int(item['stamp_ns']),
                    'base_T_flange': np.asarray(item['base_T_flange'], dtype=np.float64),
                    'camera_T_tag': np.asarray(item['camera_T_tag'], dtype=np.float64),
                    'reprojection_error_px': float(item['reprojection_error_px']),
                    'used_latest_tf': bool(item.get('used_latest_tf', False)),
                })
        except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as error:
            self.get_logger().error(f'Cannot load existing samples: {error}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HandEyeCalibrator()
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
