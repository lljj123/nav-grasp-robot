from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'arm_pose_topic', default_value='/arm/feedback/end_pose'),
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
                'use_pose_header_stamp': False,
            }],
        ),
    ])
