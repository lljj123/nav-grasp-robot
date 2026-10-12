import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_share = Path(get_package_share_directory('visual_grasp_calibration'))
    calibration_dir_default = os.environ.get(
        'NAV_GRASP_CALIBRATION_DIR',
        str(Path.home() / '.config' / 'nav_grasp' / 'calibration'),
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'calibration_dir',
            default_value=calibration_dir_default,
            description='Directory for robot-specific samples and results.'),
        DeclareLaunchArgument(
            'arm_pose_topic',
            default_value='/arm/feedback/end_pose',
            description='Measured flange pose; use /arm/feedback/tcp_pose only '
                        'when the AGX driver tcp_offset is all zeros.',
        ),
        DeclareLaunchArgument('base_frame', default_value='piper_base_link'),
        DeclareLaunchArgument('flange_frame', default_value='piper_link6'),
        Node(
            package='visual_grasp_calibration',
            executable='piper_pose_tf_bridge',
            name='piper_pose_tf_bridge',
            output='screen',
            parameters=[{
                'pose_topic': LaunchConfiguration('arm_pose_topic'),
                'parent_frame': LaunchConfiguration('base_frame'),
                'child_frame': LaunchConfiguration('flange_frame'),
            }],
        ),
        Node(
            package='visual_grasp_calibration',
            executable='handeye_calibrator',
            name='handeye_calibrator',
            output='screen',
            parameters=[
                str(package_share / 'config' / 'handeye.yaml'),
                {
                    'base_frame': LaunchConfiguration('base_frame'),
                    'flange_frame': LaunchConfiguration('flange_frame'),
                    'dataset_path': PathJoinSubstitution([
                        LaunchConfiguration('calibration_dir'),
                        'handeye_samples.yaml',
                    ]),
                    'result_path': PathJoinSubstitution([
                        LaunchConfiguration('calibration_dir'),
                        'handeye.yaml',
                    ]),
                },
            ],
        ),
    ])
