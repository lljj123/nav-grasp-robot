#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("tracer_pick_place_mission")
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=f"{package_share}/config/mission_params.yaml",
                description="Two-cycle pick/place mission parameters.",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="tracer_pick_place_mission",
                executable="pick_place_mission",
                name="pick_place_mission",
                output="screen",
                parameters=[params_file, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
            ),
        ]
    )
