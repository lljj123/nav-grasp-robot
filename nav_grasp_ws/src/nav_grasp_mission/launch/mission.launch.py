from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory('nav_grasp_mission'))
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=str(package_share / 'config' / 'mission.yaml'),
            description=(
                'Mission-only parameters; existing subsystem configs are '
                'untouched.'),
        ),
        Node(
            package='nav_grasp_mission',
            executable='mission_node',
            name='nav_grasp_mission',
            output='screen',
            parameters=[params_file],
        ),
    ])
