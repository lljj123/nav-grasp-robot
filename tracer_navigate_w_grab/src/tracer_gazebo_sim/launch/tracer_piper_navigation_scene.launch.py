#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("tracer_gazebo_sim")

    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    rviz = LaunchConfiguration("rviz")
    start_ekf = LaunchConfiguration("start_ekf")
    apply_initial_positions = LaunchConfiguration("apply_initial_positions")

    tracer_piper_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [package_share, "launch", "tracer_piper_sim.launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "world": world,
            "gazebo_gui": gazebo_gui,
            "rviz": rviz,
            "start_ekf": start_ekf,
            "apply_initial_positions": apply_initial_positions,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use the Gazebo simulation clock.",
            ),
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution(
                    [package_share, "worlds", "tracer_navigation_world.sdf"]
                ),
                description="Navigation obstacle world used by the combined robot.",
            ),
            DeclareLaunchArgument(
                "gazebo_gui",
                default_value="true",
                description="Start the Gazebo GUI.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Start RViz with the combined robot configuration.",
            ),
            DeclareLaunchArgument(
                "start_ekf",
                default_value="true",
                description="Start the EKF and publish odom to base_link TF.",
            ),
            DeclareLaunchArgument(
                "apply_initial_positions",
                default_value="true",
                description="Move Piper to its configured initial pose.",
            ),
            tracer_piper_sim,
        ]
    )
