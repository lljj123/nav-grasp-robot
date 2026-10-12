"""Combined base + lidar launch using the repository's default hardware profile.

Run with an absolute path from scripts/ (not installed into a package):

    ros2 launch /absolute/path/slam_ws/scripts/base_lidar.launch.py port_name:=can1

Starts the configured chassis (/odom, odom -> base_footprint, /cmd_vel) and
lidar driver + pointcloud-to-laserscan conversion (/scan) in one process
so the interactive start.sh can keep the terminal count low.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    tracer_share = get_package_share_directory('tracer_base')
    converter_share = get_package_share_directory('lidar')

    port_name = LaunchConfiguration('port_name')

    return LaunchDescription([
        DeclareLaunchArgument('port_name', default_value='can1'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tracer_share, 'launch', 'tracer_base.launch.py')),
            launch_arguments={
                'port_name': port_name,
                'base_frame': 'base_footprint',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(converter_share, 'launch', 'converter.launch.py')),
            # robot_state_publisher gets this transform from the shared URDF.
            launch_arguments={'publish_lidar_tf': 'false'}.items(),
        ),
    ])
