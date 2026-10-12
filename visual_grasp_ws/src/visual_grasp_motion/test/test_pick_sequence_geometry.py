import copy
import math

import pytest
from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import JointState

from visual_grasp_motion.motion_math import rotate_orientation_about_base_z
from visual_grasp_motion.piper_pick_executor import PiperPickExecutor


def make_executor_without_ros_init():
    executor = PiperPickExecutor.__new__(PiperPickExecutor)
    executor.base_frame = 'piper_base_link'
    executor.tcp_workspace_min = [0.15, -0.35, -0.12]
    executor.tcp_workspace_max = [0.60, 0.35, 0.35]
    executor.flange_workspace_min = [0.05, -0.45, 0.02]
    executor.flange_workspace_max = [0.70, 0.45, 0.70]
    executor.grasp_z_min_m = -0.12
    executor.grasp_z_max_m = 0.20
    executor.min_approach_height_m = 0.06
    executor.max_tool_tilt_rad = math.radians(12.0)
    executor.orientation_tolerance_rad = math.radians(5.0)
    executor.tcp_offset = [0.0, 0.0, 0.142]
    return executor


def make_pose(z, quaternion):
    pose = Pose()
    pose.position.x = 0.30
    pose.position.y = 0.02
    pose.position.z = z
    (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ) = quaternion
    return pose


def test_four_pose_sequence_lifts_before_straightening():
    executor = make_executor_without_ros_init()
    nominal = (0.0, 1.0, 0.0, 0.0)
    aligned = rotate_orientation_about_base_z(nominal, math.radians(25.0))
    grasp = make_pose(-0.05, aligned)
    pregrasp = make_pose(0.04, aligned)
    lift = make_pose(0.05, aligned)
    straighten = make_pose(0.05, nominal)
    sequence = PoseArray()
    sequence.header.frame_id = 'piper_base_link'
    sequence.poses = [pregrasp, grasp, lift, straighten]

    tcp_poses, flange_poses = executor._validated_sequence(sequence)

    assert len(tcp_poses) == 4
    assert len(flange_poses) == 4
    assert tcp_poses[2].position.z == pytest.approx(tcp_poses[3].position.z)


def test_three_pose_sequence_is_rejected():
    executor = make_executor_without_ros_init()
    nominal = (0.0, 1.0, 0.0, 0.0)
    pose = make_pose(0.05, nominal)
    sequence = PoseArray()
    sequence.poses = [copy.deepcopy(pose) for _ in range(3)]

    with pytest.raises(ValueError, match='straighten'):
        executor._validated_sequence(sequence)


def test_approximate_carry_posture_accepts_configured_tolerance():
    executor = make_executor_without_ros_init()
    executor.carry_joint_names = [f'joint{index}' for index in range(1, 7)]
    executor.carry_joint_positions = [0.0, 0.46, -0.74, 0.0, 1.05, 0.0]
    executor.carry_joint_tolerance_rad = math.radians(6.0)
    executor.target_reached_since = None
    feedback = JointState()
    feedback.name = list(reversed(executor.carry_joint_names))
    feedback.position = list(reversed([
        0.03, 0.42, -0.69, -0.02, 1.10, 0.04,
    ]))
    executor.joint_feedback = feedback

    assert executor._carry_posture_reached(settle=False)


def test_carry_posture_rejects_large_error_or_missing_joint():
    executor = make_executor_without_ros_init()
    executor.carry_joint_names = [f'joint{index}' for index in range(1, 7)]
    executor.carry_joint_positions = [0.0, 0.46, -0.74, 0.0, 1.05, 0.0]
    executor.carry_joint_tolerance_rad = math.radians(6.0)
    executor.target_reached_since = None
    feedback = JointState()
    feedback.name = list(executor.carry_joint_names)
    feedback.position = [0.0, 0.46, -0.74, 0.0, 0.80, 0.0]
    executor.joint_feedback = feedback
    assert not executor._carry_posture_reached(settle=False)

    feedback.name.pop()
    feedback.position.pop()
    assert not executor._carry_posture_reached(settle=False)
