#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    sim_share = get_package_share_directory("tracer_gazebo_sim")
    config_share = get_package_share_directory("tracer_piper_moveit_config")

    initial_positions_file = LaunchConfiguration("initial_positions_file")
    controllers_file = LaunchConfiguration("controllers_file")

    moveit_config = (
        MoveItConfigsBuilder(
            "tracer_piper",
            package_name="tracer_piper_moveit_config",
        )
        .robot_description(
            file_path=f"{sim_share}/urdf/tracer_piper.urdf.xacro",
            mappings={
                "initial_positions_file": initial_positions_file,
                "controllers_file": controllers_file,
            },
        )
        .robot_description_semantic(
            file_path=f"{config_share}/config/tracer_piper.srdf"
        )
        .robot_description_kinematics(
            file_path=f"{config_share}/config/kinematics.yaml"
        )
        .joint_limits(
            file_path=f"{config_share}/config/joint_limits.yaml"
        )
        .trajectory_execution(
            file_path=f"{config_share}/config/moveit_controllers.yaml"
        )
        .planning_pipelines("ompl", ["ompl"])
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": use_sim_time,
                "allow_trajectory_execution": True,
                "publish_planning_scene": True,
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use the Gazebo simulation clock.",
            ),
            DeclareLaunchArgument(
                "initial_positions_file",
                default_value=f"{sim_share}/config/piper_initial_positions.yaml",
                description="Piper initial-position YAML used by the robot xacro.",
            ),
            DeclareLaunchArgument(
                "controllers_file",
                default_value=f"{sim_share}/config/piper_controllers.yaml",
                description="ros2_control YAML used by the robot xacro.",
            ),
            move_group,
        ]
    )
