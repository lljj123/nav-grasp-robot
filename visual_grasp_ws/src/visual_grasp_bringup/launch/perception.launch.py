import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    perception_share = Path(get_package_share_directory(
        'visual_grasp_perception'))
    motion_share = Path(get_package_share_directory('visual_grasp_motion'))
    calibration_dir_default = os.environ.get(
        'NAV_GRASP_CALIBRATION_DIR',
        str(Path.home() / '.config' / 'nav_grasp' / 'calibration'),
    )
    gripper_report_default = os.environ.get(
        'NAV_GRASP_GRIPPER_REPORT',
        str(Path.home() / '.config' / 'nav_grasp' / 'gripper'
            / 'manual_gripper_range.json'),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'calibration_dir',
            default_value=calibration_dir_default,
            description='Directory containing the robot-specific hand-eye result.'),
        DeclareLaunchArgument(
            'gripper_range_report_path',
            default_value=gripper_report_default,
            description='Robot-specific gripper endpoint validation report.'),
        DeclareLaunchArgument(
            'use_pose_tf_bridge', default_value='true',
            description='Bridge measured PiPER flange pose into tf2.'),
        DeclareLaunchArgument(
            'arm_pose_topic', default_value='/arm/feedback/tcp_pose',
            description='AGX pose feedback; driver tcp_offset must be all zeros.'),
        DeclareLaunchArgument(
            'allow_robot_commands', default_value='false',
            description=(
                'Enable the guarded executor publishers. Keep false for '
                'perception and dry-run validation.')),
        Node(
            package='visual_grasp_calibration',
            executable='piper_pose_tf_bridge',
            name='piper_pose_tf_bridge',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_pose_tf_bridge')),
            parameters=[{
                'pose_topic': LaunchConfiguration('arm_pose_topic'),
                'parent_frame': 'piper_base_link',
                'child_frame': 'piper_link6',
                # PiPER device stamps can lag the D435/system clock.  Feedback
                # arrives continuously, so publish the measured pose at its ROS
                # receipt time to keep the dynamic TF usable by perception.
                'use_pose_header_stamp': False,
            }],
        ),
        # Nominal TCP at the center between the gripper fingers. Do not add this
        # offset again in perception or grasp planning.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='piper_tcp_static_tf',
            output='screen',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.142',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                '--frame-id', 'piper_link6',
                '--child-frame-id', 'piper_tcp_link',
            ],
        ),
        Node(
            package='visual_grasp_calibration',
            executable='handeye_tf_publisher',
            name='handeye_tf_publisher',
            output='screen',
            parameters=[{
                'result_path': PathJoinSubstitution([
                    LaunchConfiguration('calibration_dir'),
                    'handeye.yaml',
                ]),
            }],
        ),
        Node(
            package='visual_grasp_perception',
            executable='color_cube_detector',
            name='color_cube_detector',
            output='screen',
            parameters=[str(
                perception_share / 'config' / 'color_cubes.yaml')],
        ),
        Node(
            package='visual_grasp_motion',
            executable='grasp_candidate_planner',
            name='grasp_candidate_planner',
            output='screen',
            parameters=[str(
                motion_share / 'config' / 'grasp_planner.yaml')],
        ),
        Node(
            package='visual_grasp_motion',
            executable='piper_pick_executor',
            name='piper_pick_executor',
            output='screen',
            parameters=[
                str(motion_share / 'config' / 'piper_executor.yaml'),
                {
                    'allow_robot_commands': ParameterValue(
                        LaunchConfiguration('allow_robot_commands'),
                        value_type=bool),
                    'flange_feedback_topic': LaunchConfiguration(
                        'arm_pose_topic'),
                    'gripper_range_report_path': LaunchConfiguration(
                        'gripper_range_report_path'),
                },
            ],
        ),
    ])
