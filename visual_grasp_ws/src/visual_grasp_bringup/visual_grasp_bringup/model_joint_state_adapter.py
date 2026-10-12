"""Adapt AGX PiPER joint feedback for the visualization-only URDF."""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class ModelJointStateAdapter(Node):
    """Replace the driver's single jaw width with two URDF finger joints."""

    def __init__(self) -> None:
        super().__init__('model_joint_state_adapter')
        input_topic = str(self.declare_parameter(
            'input_topic', '/arm/feedback/joint_states').value)
        output_topic = str(self.declare_parameter(
            'output_topic', '/visual_grasp/model_joint_states').value)
        self.gripper_joint = str(self.declare_parameter(
            'gripper_input_joint', 'gripper').value)
        self.left_finger_joint = str(self.declare_parameter(
            'left_finger_joint', 'gripper_joint1').value)
        self.right_finger_joint = str(self.declare_parameter(
            'right_finger_joint', 'gripper_joint2').value)
        self.max_width_m = float(self.declare_parameter(
            'max_gripper_width_m', 0.080).value)

        self.publisher = self.create_publisher(JointState, output_topic, 10)
        self.subscription = self.create_subscription(
            JointState, input_topic, self._callback, 10)
        self.get_logger().info(
            f'Visual model joint feedback: {input_topic} -> {output_topic}')

    @staticmethod
    def _value_at(values, index: int) -> float:
        if index < len(values) and math.isfinite(float(values[index])):
            return float(values[index])
        return 0.0

    def _callback(self, message: JointState) -> None:
        output = JointState()
        output.header = message.header

        gripper_index = None
        for index, name in enumerate(message.name):
            if name == self.gripper_joint:
                gripper_index = index
                continue
            output.name.append(name)
            output.position.append(self._value_at(message.position, index))
            if message.velocity:
                output.velocity.append(self._value_at(message.velocity, index))
            if message.effort:
                output.effort.append(self._value_at(message.effort, index))

        if gripper_index is not None:
            width = self._value_at(message.position, gripper_index)
            width = min(max(width, 0.0), self.max_width_m)
            half_width = 0.5 * width
            output.name.extend([
                self.left_finger_joint,
                self.right_finger_joint,
            ])
            output.position.extend([half_width, -half_width])
            if message.velocity:
                half_velocity = 0.5 * self._value_at(
                    message.velocity, gripper_index)
                output.velocity.extend([half_velocity, -half_velocity])
            if message.effort:
                half_effort = 0.5 * self._value_at(
                    message.effort, gripper_index)
                output.effort.extend([half_effort, -half_effort])

        self.publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ModelJointStateAdapter()
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
