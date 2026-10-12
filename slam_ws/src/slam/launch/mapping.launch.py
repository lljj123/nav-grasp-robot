import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('slam')
    description_share = get_package_share_directory('robot_description')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_rviz = LaunchConfiguration('start_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    start_robot_description = LaunchConfiguration('start_robot_description')
    start_joint_state_publisher = LaunchConfiguration(
        'start_joint_state_publisher')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(package_share, 'config', 'slam_toolbox.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('start_robot_description', default_value='true'),
        DeclareLaunchArgument(
            'start_joint_state_publisher', default_value='false'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(package_share, 'rviz', 'mapping.rviz')),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                description_share, 'launch', 'robot_description.launch.py')),
            condition=IfCondition(start_robot_description),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'start_joint_state_publisher': start_joint_state_publisher,
            }.items(),
        ),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),
        Node(
            condition=IfCondition(start_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
