from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    package_share = Path(get_package_share_directory('visual_grasp_perception'))
    return LaunchDescription([
        Node(
            package='visual_grasp_perception',
            executable='color_cube_detector',
            name='color_cube_detector',
            output='screen',
            parameters=[str(package_share / 'config' / 'color_cubes.yaml')],
        ),
    ])

