import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = get_package_share_directory('lidar')
    params_file = LaunchConfiguration('params_file')
    start_lidar = LaunchConfiguration('start_lidar')
    publish_lidar_tf = LaunchConfiguration('publish_lidar_tf')
    cloud_topic = LaunchConfiguration('cloud_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    base_frame = LaunchConfiguration('base_frame')
    lidar_frame = LaunchConfiguration('lidar_frame')
    driver_package = LaunchConfiguration('lidar_driver_package')
    driver_executable = LaunchConfiguration('lidar_driver_executable')
    driver_config_relative_path = LaunchConfiguration(
        'lidar_driver_config_relative_path')

    lidar_x = LaunchConfiguration('lidar_x')
    lidar_y = LaunchConfiguration('lidar_y')
    lidar_z = LaunchConfiguration('lidar_z')
    lidar_roll = LaunchConfiguration('lidar_roll')
    lidar_pitch = LaunchConfiguration('lidar_pitch')
    lidar_yaw = LaunchConfiguration('lidar_yaw')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(package_share, 'config', 'converter.yaml')),
        DeclareLaunchArgument('start_lidar', default_value='true'),
        DeclareLaunchArgument(
            'lidar_driver_package', default_value='livox_ros_driver2'),
        DeclareLaunchArgument(
            'lidar_driver_executable', default_value='livox_ros_driver2_node'),
        DeclareLaunchArgument(
            'lidar_driver_config_relative_path',
            default_value='config/MID360_config.json'),
        DeclareLaunchArgument('publish_lidar_tf', default_value='true'),
        DeclareLaunchArgument('cloud_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('lidar_frame', default_value='livox_frame'),
        DeclareLaunchArgument('lidar_x', default_value='0.15'),
        DeclareLaunchArgument('lidar_y', default_value='0.0'),
        DeclareLaunchArgument('lidar_z', default_value='0.10'),
        DeclareLaunchArgument('lidar_roll', default_value='0.0'),
        DeclareLaunchArgument('lidar_pitch', default_value='0.0'),
        DeclareLaunchArgument('lidar_yaw', default_value='0.0'),

        Node(
            condition=IfCondition(start_lidar),
            package=driver_package,
            executable=driver_executable,
            name='lidar_driver',
            output='screen',
            parameters=[{
                'xfer_format': 0,  # sensor_msgs/msg/PointCloud2
                'multi_topic': 0,
                'data_src': 0,
                'publish_freq': 10.0,
                'output_data_type': 0,
                'frame_id': lidar_frame,
                'user_config_path': PathJoinSubstitution([
                    FindPackageShare(driver_package),
                    driver_config_relative_path,
                ]),
            }],
        ),

        Node(
            condition=IfCondition(publish_lidar_tf),
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_lidar_tf',
            arguments=[
                '--x', lidar_x, '--y', lidar_y, '--z', lidar_z,
                '--roll', lidar_roll, '--pitch', lidar_pitch, '--yaw', lidar_yaw,
                '--frame-id', base_frame, '--child-frame-id', lidar_frame,
            ],
        ),

        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            parameters=[params_file],
            remappings=[('cloud_in', cloud_topic), ('scan', scan_topic)],
        ),
    ])
