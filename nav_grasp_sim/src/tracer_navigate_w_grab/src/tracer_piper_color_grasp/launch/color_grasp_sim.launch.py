#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_stack = LaunchConfiguration("start_stack")
    start_debug_view = LaunchConfiguration("start_debug_view")
    world = LaunchConfiguration("world")

    sim_share = get_package_share_directory("tracer_gazebo_sim")
    moveit_share = get_package_share_directory("tracer_piper_moveit_config")
    grasp_share = get_package_share_directory("tracer_piper_color_grasp")

    initial_positions_file = f"{sim_share}/config/piper_initial_positions.yaml"
    controllers_file = f"{sim_share}/config/piper_controllers.yaml"

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f"{sim_share}/launch/tracer_piper_sim.launch.py"
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "world": world,
            "gazebo_gui": "true",
            "rviz": "false",
        }.items(),
        condition=IfCondition(start_stack),
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f"{moveit_share}/launch/move_group.launch.py"
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "initial_positions_file": initial_positions_file,
            "controllers_file": controllers_file,
        }.items(),
        condition=IfCondition(start_stack),
    )

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
            file_path=f"{moveit_share}/config/tracer_piper.srdf"
        )
        .robot_description_kinematics(
            file_path=f"{moveit_share}/config/kinematics.yaml"
        )
        .joint_limits(
            file_path=f"{moveit_share}/config/joint_limits.yaml"
        )
        .trajectory_execution(
            file_path=f"{moveit_share}/config/moveit_controllers.yaml"
        )
        .planning_pipelines("ompl", ["ompl"])
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    grasp_server = Node(
        package="tracer_piper_color_grasp",
        executable="color_grasp_server",
        name="color_grasp_server",
        output="screen",
        parameters=[
            f"{grasp_share}/config/grasp_task.yaml",
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            moveit_config.planning_pipelines,
            {"use_sim_time": use_sim_time},
        ],
    )


    debug_image_view = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        name="wrist_camera_image_view",
        output="screen",
        # Show the uninterrupted camera stream.  The processed debug image is
        # only published while the detector is active and therefore is not a
        # suitable default monitoring window.
        arguments=["/wrist_camera/image"],
        condition=IfCondition(start_debug_view),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use the Gazebo simulation clock.",
            ),
            DeclareLaunchArgument(
                "start_stack",
                default_value="true",
                description=(
                    "Start standalone Gazebo GUI, RViz2 and MoveIt. "
                    "Set false when that stack is already running."
                ),
            ),
            DeclareLaunchArgument(
                "start_debug_view",
                default_value="false",
                description="Open rqt_image_view for the live wrist camera image.",
            ),
            DeclareLaunchArgument(
                "world",
                default_value=f"{sim_share}/worlds/tracer_grasp.sdf",
                description="Existing visual-grasp Gazebo world.",
            ),
            simulator,
            move_group,
            grasp_server,
            debug_image_view,
        ]
    )
