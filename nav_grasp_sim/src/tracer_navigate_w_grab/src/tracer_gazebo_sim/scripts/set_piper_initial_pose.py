#!/usr/bin/env python3

import sys
import time
from pathlib import Path

import rclpy
import yaml
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


ARM_JOINTS = [f"piper_joint{index}" for index in range(1, 7)]
GRIPPER_JOINTS = ["piper_gripper_joint1", "piper_gripper_joint2"]
ALL_JOINTS = ARM_JOINTS + GRIPPER_JOINTS


class PiperInitialPoseCommander(Node):
    def __init__(self):
        super().__init__("piper_mobile_initial_pose_commander")

        self.declare_parameter("pose_file", "")
        self.declare_parameter("move_duration", 2.5)
        self.declare_parameter("server_timeout", 90.0)
        self.declare_parameter("result_timeout", 30.0)
        self.declare_parameter("verification_timeout", 10.0)
        self.declare_parameter("position_tolerance", 0.02)

        self.move_duration = float(self.get_parameter("move_duration").value)
        self.server_timeout = float(self.get_parameter("server_timeout").value)
        self.result_timeout = float(self.get_parameter("result_timeout").value)
        self.verification_timeout = float(
            self.get_parameter("verification_timeout").value
        )
        self.position_tolerance = float(
            self.get_parameter("position_tolerance").value
        )
        self.positions = self._load_positions(
            self.get_parameter("pose_file").value
        )

        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/piper_arm_controller/follow_joint_trajectory",
        )
        self.gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/piper_gripper_controller/follow_joint_trajectory",
        )
        self.measured_positions = {}
        self.joint_state_subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_callback,
            qos_profile_sensor_data,
        )

    def _load_positions(self, pose_file):
        path = Path(pose_file)
        if not path.is_file():
            raise FileNotFoundError(f"Piper pose file not found: {path}")
        with path.open("r", encoding="utf-8") as stream:
            content = yaml.safe_load(stream)
        values = content.get("initial_positions", {}) if content else {}
        missing = [joint for joint in ALL_JOINTS if joint not in values]
        if missing:
            raise ValueError("Pose file is missing: " + ", ".join(missing))
        return {joint: float(values[joint]) for joint in ALL_JOINTS}

    def _joint_state_callback(self, message):
        for name, position in zip(message.name, message.position):
            self.measured_positions[name] = float(position)

    def _wait_for_server(self, client, action_name):
        self.get_logger().info(f"Waiting for controller: {action_name}")
        deadline = time.monotonic() + self.server_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if client.wait_for_server(timeout_sec=1.0):
                return
        raise TimeoutError(f"Controller action unavailable: {action_name}")

    def _send_goal(self, client, action_name, joints, duration):
        self._wait_for_server(client, action_name)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joints

        point = JointTrajectoryPoint()
        point.positions = [self.positions[joint] for joint in joints]
        point.time_from_start = Duration(
            nanoseconds=int(duration * 1_000_000_000)
        ).to_msg()
        goal.trajectory.points = [point]

        self.get_logger().info(
            "Sending Piper pose: "
            + ", ".join(
                f"{joint}={value:.4f}"
                for joint, value in zip(joints, point.positions)
            )
        )
        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(
            self, send_future, timeout_sec=self.server_timeout
        )
        if not send_future.done():
            raise TimeoutError(f"Timed out sending goal: {action_name}")
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f"Controller rejected goal: {action_name}")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=self.result_timeout
        )
        if not result_future.done():
            raise TimeoutError(f"Timed out executing goal: {action_name}")
        result = result_future.result().result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(
                f"Trajectory failed on {action_name}: "
                f"code={result.error_code}, message={result.error_string}"
            )

    def _verify_positions(self):
        deadline = time.monotonic() + self.verification_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if not all(joint in self.measured_positions for joint in ALL_JOINTS):
                continue
            errors = [
                abs(self.measured_positions[joint] - self.positions[joint])
                for joint in ALL_JOINTS
            ]
            if max(errors) <= self.position_tolerance:
                self.get_logger().info(
                    "Verified Piper pose from /joint_states"
                )
                return

        missing = [
            joint for joint in ALL_JOINTS if joint not in self.measured_positions
        ]
        if missing:
            raise RuntimeError(
                "/joint_states is missing Piper joints: " + ", ".join(missing)
            )
        raise RuntimeError("Piper did not reach the requested initial pose")

    def move(self):
        self._send_goal(
            self.arm_client,
            "/piper_arm_controller/follow_joint_trajectory",
            ARM_JOINTS,
            self.move_duration,
        )
        self._send_goal(
            self.gripper_client,
            "/piper_gripper_controller/follow_joint_trajectory",
            GRIPPER_JOINTS,
            min(self.move_duration, 1.5),
        )
        self._verify_positions()
        self.get_logger().info("Mobile Piper initial pose applied and verified")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PiperInitialPoseCommander()
        node.move()
    except Exception as error:
        if node is not None:
            node.get_logger().error(str(error))
        else:
            print(f"Failed to start Piper pose commander: {error}", file=sys.stderr)
        raise SystemExit(1) from error
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
