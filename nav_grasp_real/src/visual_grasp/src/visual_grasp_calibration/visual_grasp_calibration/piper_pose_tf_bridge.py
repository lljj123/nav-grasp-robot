"""Broadcast the PiPER base-to-flange TF from a read-only pose topic."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster


class PiperPoseTfBridge(Node):
    """Convert a measured PiPER flange pose into a dynamic TF."""

    def __init__(self) -> None:
        super().__init__('piper_pose_tf_bridge')
        self.pose_topic = self.declare_parameter(
            'pose_topic', '/arm/feedback/end_pose').value
        self.parent_frame = self.declare_parameter(
            'parent_frame', 'piper_base_link').value
        self.child_frame = self.declare_parameter(
            'child_frame', 'piper_link6').value
        self.reject_frame_mismatch = bool(self.declare_parameter(
            'reject_frame_mismatch', True).value)
        self.message_timeout_s = float(self.declare_parameter(
            'message_timeout_s', 2.0).value)
        self.use_pose_header_stamp = bool(self.declare_parameter(
            'use_pose_header_stamp', False).value)

        self.broadcaster = TransformBroadcaster(self)
        self.last_message_ns = 0
        self.received_once = False
        self.last_frame_warning_ns = 0
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.subscription = self.create_subscription(
            PoseStamped, self.pose_topic, self._pose_callback, qos)
        self.watchdog = self.create_timer(1.0, self._watchdog_callback)
        self.get_logger().info(
            f'Waiting for measured flange pose on {self.pose_topic}; will publish '
            f'{self.parent_frame} -> {self.child_frame}')
        self.get_logger().warning(
            'If the source is /arm/feedback/tcp_pose, the AGX driver tcp_offset '
            'must remain all zeros so the message still represents the flange')
        if not self.use_pose_header_stamp:
            self.get_logger().info(
                'Dynamic TF will use the ROS receipt time. This avoids stale TF '
                'when the PiPER device timestamp lags the camera clock.')

    def _pose_callback(self, message: PoseStamped) -> None:
        source_frame = message.header.frame_id.strip()
        if source_frame and source_frame != self.parent_frame:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self.last_frame_warning_ns > 2_000_000_000:
                self.get_logger().error(
                    f'Pose frame is {source_frame}, expected {self.parent_frame}')
                self.last_frame_warning_ns = now_ns
            if self.reject_frame_mismatch:
                return

        position = message.pose.position
        orientation = message.pose.orientation
        values = [
            position.x, position.y, position.z,
            orientation.x, orientation.y, orientation.z, orientation.w,
        ]
        if not all(math.isfinite(value) for value in values):
            self.get_logger().error('Rejected non-finite PiPER pose feedback')
            return
        quaternion_norm = math.sqrt(
            orientation.x ** 2 + orientation.y ** 2
            + orientation.z ** 2 + orientation.w ** 2)
        if quaternion_norm < 1e-9:
            self.get_logger().error('Rejected PiPER pose with zero quaternion')
            return

        now = self.get_clock().now()
        transform = TransformStamped()
        if (self.use_pose_header_stamp
                and (message.header.stamp.sec != 0
                     or message.header.stamp.nanosec != 0)):
            transform.header.stamp = message.header.stamp
        else:
            # The AGX/PiPER feedback is delivered at high rate, but its device
            # stamp can trail the ROS camera clock by several seconds.  Copying
            # that stamp makes otherwise-current base->flange TF unusable for
            # image transforms.  Receipt time expresses when this measured pose
            # became available in the ROS graph.
            transform.header.stamp = now.to_msg()
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = self.child_frame
        transform.transform.translation.x = position.x
        transform.transform.translation.y = position.y
        transform.transform.translation.z = position.z
        transform.transform.rotation.x = orientation.x / quaternion_norm
        transform.transform.rotation.y = orientation.y / quaternion_norm
        transform.transform.rotation.z = orientation.z / quaternion_norm
        transform.transform.rotation.w = orientation.w / quaternion_norm
        self.broadcaster.sendTransform(transform)

        self.last_message_ns = now.nanoseconds
        if not self.received_once:
            self.received_once = True
            self.get_logger().info(
                f'Received flange feedback and started publishing '
                f'{self.parent_frame} -> {self.child_frame}')

    def _watchdog_callback(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if self.last_message_ns == 0:
            return
        age_s = (now_ns - self.last_message_ns) / 1e9
        if age_s > self.message_timeout_s:
            self.get_logger().warning(
                f'PiPER pose feedback is stale ({age_s:.1f} s); TF is no longer '
                'being updated')
            self.last_message_ns = 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PiperPoseTfBridge()
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
