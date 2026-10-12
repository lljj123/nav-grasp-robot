#!/usr/bin/env python3
"""Print a concise safety-oriented snapshot of base and arm feedback."""

from __future__ import annotations

import argparse
import math
import time

from agx_arm_msgs.msg import AgxArmStatus, GripperStatus
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


ARM_FAULT_FIELDS = (
    'voltage_too_low',
    'motor_overheating',
    'driver_overcurrent',
    'driver_overheating',
    'sensor_status',
    'driver_error_status',
)
ACTIVE_TEACH_STATES = {1, 3, 4, 5, 7}


def vector_norm(vector) -> float:
    """Return the Euclidean norm of a geometry vector."""
    return math.sqrt(vector.x ** 2 + vector.y ** 2 + vector.z ** 2)


def true_indexes(values) -> list[int]:
    """Return one-based indexes whose boolean value is true."""
    return [index + 1 for index, value in enumerate(values) if value]


class StatusSnapshot(Node):
    """Collect one recent message from each hardware feedback stream."""

    def __init__(self) -> None:
        """Create subscriptions without publishing any robot command."""
        super().__init__('nav_grasp_status_snapshot')
        self.odom = None
        self.arm = None
        self.gripper = None
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(
            AgxArmStatus, '/arm/feedback/arm_status', self._on_arm, 10)
        self.create_subscription(
            GripperStatus,
            '/arm/feedback/gripper_status',
            self._on_gripper,
            10,
        )

    def _on_odom(self, message: Odometry) -> None:
        self.odom = message

    def _on_arm(self, message: AgxArmStatus) -> None:
        self.arm = message

    def _on_gripper(self, message: GripperStatus) -> None:
        self.gripper = message

    def complete(self) -> bool:
        """Return whether all required feedback has arrived."""
        return all((self.odom, self.arm, self.gripper))


def print_snapshot(node: StatusSnapshot) -> bool:
    """Print the collected feedback and return whether it is safe to start."""
    odom = node.odom
    arm = node.arm
    gripper = node.gripper
    if odom is None or arm is None or gripper is None:
        missing = []
        if odom is None:
            missing.append('/odom')
        if arm is None:
            missing.append('/arm/feedback/arm_status')
        if gripper is None:
            missing.append('/arm/feedback/gripper_status')
        print('状态检查失败：缺少 ' + ', '.join(missing))
        return False

    linear_speed = vector_norm(odom.twist.twist.linear)
    angular_speed = vector_norm(odom.twist.twist.angular)
    print(
        '底盘状态：反馈正常 | '
        f'线速度={linear_speed:.3f} m/s，'
        f'角速度={angular_speed:.3f} rad/s'
    )

    limit_faults = true_indexes(arm.joint_angle_limit)
    communication_faults = true_indexes(arm.communication_status_joint)
    arm_safe = (
        arm.ctrl_mode == 1
        and arm.arm_status == 0
        and arm.err_status == 0
        and not limit_faults
        and not communication_faults
        and arm.teach_status not in ACTIVE_TEACH_STATES
    )
    arm_label = '正常' if arm_safe else '异常，禁止启动任务'
    motion_label = '到位' if arm.motion_status == 0 else '运动中/未到位'
    print(
        f'机械臂状态：{arm_label} | ctrl_mode={arm.ctrl_mode}，'
        f'arm_status={arm.arm_status}，err_status={arm.err_status}，'
        f'teach_status={arm.teach_status}，运动={motion_label}'
    )
    if limit_faults:
        print(f'  关节限位报警：{limit_faults}')
    if communication_faults:
        print(f'  关节通信报警：{communication_faults}')

    gripper_faults = [
        name for name in ARM_FAULT_FIELDS if bool(getattr(gripper, name))
    ]
    gripper_safe = not gripper_faults
    if gripper_faults:
        gripper_label = '异常，禁止启动任务'
    elif gripper.driver_enable_status:
        gripper_label = '正常'
    else:
        # visual_grasp_ws deliberately permits this initial state: its first
        # opening command enables the gripper and then verifies the feedback.
        gripper_label = '待使能（抓取开始时自动使能）'
    print(
        f'夹爪状态：{gripper_label} | 宽度={gripper.width:.4f} m，'
        f'使能={gripper.driver_enable_status}，'
        f'回零={gripper.homing_status}，力={gripper.force:.2f} N'
    )
    if gripper_faults:
        print('  夹爪故障：' + ', '.join(gripper_faults))

    return arm_safe and gripper_safe


def main() -> int:
    """Wait briefly for feedback, print it, and return a health exit code."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout', type=float, default=5.0)
    arguments = parser.parse_args()

    rclpy.init()
    node = StatusSnapshot()
    deadline = time.monotonic() + max(arguments.timeout, 0.1)
    try:
        while not node.complete() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        return 0 if print_snapshot(node) else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
