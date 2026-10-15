#!/usr/bin/env python3

import os
import shutil

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    Shutdown,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


def _gazebo_process(context):
    # Start the simulator directly: the upstream shell wrapper can orphan its
    # Ruby child when launch escalates SIGINT to SIGTERM.
    version = LaunchConfiguration("gz_version").perform(context)
    classic_cli = int(version) < 7
    cli = shutil.which("ign" if classic_cli else "gz")
    ruby = shutil.which("ruby")
    if not cli or not ruby:
        raise RuntimeError("Gazebo CLI or Ruby executable is missing")
    cmd = [ruby, cli, "gazebo" if classic_cli else "sim"]
    if LaunchConfiguration("gazebo_gui").perform(context).lower() != "true":
        cmd.append("-s")
    cmd += ["-r", "-v", "3", LaunchConfiguration("world").perform(context),
            "--force-version", version]
    env = context.environment
    plugin_path = os.pathsep.join(filter(None, [
        env.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""),
        env.get("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", ""), env.get("LD_LIBRARY_PATH", "")]))
    resource_path = os.pathsep.join(filter(None, [
        env.get("GZ_SIM_RESOURCE_PATH", ""), env.get("IGN_GAZEBO_RESOURCE_PATH", "")]))
    return [ExecuteProcess(
        cmd=cmd, shell=False, output="screen",
        additional_env={"GZ_SIM_SYSTEM_PLUGIN_PATH": plugin_path,
                        "IGN_GAZEBO_SYSTEM_PLUGIN_PATH": plugin_path,
                        "GZ_SIM_RESOURCE_PATH": resource_path,
                        "IGN_GAZEBO_RESOURCE_PATH": resource_path},
        on_exit=Shutdown(reason="Gazebo simulator exited"))]


def generate_launch_description():
    package_share = FindPackageShare("tracer_gazebo_sim")

    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")
    use_rviz = LaunchConfiguration("rviz")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    start_ekf = LaunchConfiguration("start_ekf")
    apply_initial_positions = LaunchConfiguration("apply_initial_positions")
    initial_positions_file = LaunchConfiguration("initial_positions_file")
    rviz_config = LaunchConfiguration("rviz_config")

    robot_xacro = PathJoinSubstitution(
        [package_share, "urdf", "tracer_piper.urdf.xacro"]
    )
    controllers_file = PathJoinSubstitution(
        [package_share, "config", "piper_controllers.yaml"]
    )
    bridge_file = PathJoinSubstitution(
        [package_share, "config", "tracer_piper_bridge.yaml"]
    )
    ekf_file = PathJoinSubstitution([package_share, "config", "ekf.yaml"])

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            robot_xacro,
            " initial_positions_file:=",
            initial_positions_file,
            " controllers_file:=",
            controllers_file,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(
            robot_description_content,
            value_type=str,
        )
    }

    # Gazebo resolves package:// resources from each install prefix's share folder.
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            PathJoinSubstitution(
                [FindPackagePrefix("tracer_gazebo_sim"), "share"]
            ),
            ":",
            PathJoinSubstitution(
                [FindPackagePrefix("agx_arm_description"), "share"]
            ),
            ":",
            EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
        ],
    )

    gazebo = OpaqueFunction(function=_gazebo_process)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_tracer_piper",
        output="screen",
        arguments=[
            "-topic",
            "/robot_description",
            "-name",
            "tracer_piper",
            "-allow_renaming",
            "false",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.02",
        ],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        name="spawn_piper_joint_state_broadcaster",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
            "--switch-timeout",
            "60",
        ],
    )

    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        name="spawn_piper_arm_controller",
        output="screen",
        arguments=[
            "piper_arm_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
    )

    gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        name="spawn_piper_gripper_controller",
        output="screen",
        arguments=[
            "piper_gripper_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
    )

    initial_pose_commander = Node(
        package="tracer_gazebo_sim",
        executable="set_piper_initial_pose.py",
        name="piper_mobile_initial_pose_commander",
        output="screen",
        condition=IfCondition(apply_initial_positions),
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "pose_file": initial_positions_file,
                "move_duration": 2.5,
            }
        ],
    )

    start_joint_state_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster],
        )
    )
    start_arm_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[arm_controller],
        )
    )
    start_gripper_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=arm_controller,
            on_exit=[gripper_controller],
        )
    )
    command_initial_pose = RegisterEventHandler(
        OnProcessExit(
            target_action=gripper_controller,
            on_exit=[initial_pose_commander],
        )
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="tracer_piper_gz_bridge",
        output="screen",
        parameters=[
            {
                "config_file": bridge_file,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="simulation_image_bridge",
        output="screen",
        arguments=[
            "/wrist_camera/image",
            "/wrist_camera/depth_image",
            "/gazebo_overview/image",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    world_control_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gazebo_world_control_bridge",
        output="screen",
        arguments=[
            "/world/tracer_world/control@ros_gz_interfaces/srv/ControlWorld",
            "/world/tracer_world/set_pose@ros_gz_interfaces/srv/SetEntityPose",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        condition=IfCondition(start_ekf),
        parameters=[ekf_file, {"use_sim_time": use_sim_time}],
    )

    rviz = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(use_rviz),
            )
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
                "world",
                default_value=PathJoinSubstitution(
                    [package_share, "worlds", "tracer_navigation_world.sdf"]
                ),
                description="Absolute path to the Gazebo Harmonic world.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Start RViz2.",
            ),
            DeclareLaunchArgument("gz_version", default_value="6"),
            DeclareLaunchArgument(
                "gazebo_gui",
                default_value="true",
                description="Start the standalone Gazebo GUI. Set false when using the integrated UI.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [package_share, "rviz", "tracer_piper_nav2.rviz"]
                ),
                description="RViz configuration file.",
            ),
            DeclareLaunchArgument(
                "start_ekf",
                default_value="true",
                description="Fuse wheel odometry and IMU and publish odom TF.",
            ),
            DeclareLaunchArgument(
                "initial_positions_file",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "piper_initial_positions.yaml"]
                ),
                description="YAML file containing prefixed Piper joint positions.",
            ),
            DeclareLaunchArgument(
                "apply_initial_positions",
                default_value="true",
                description="Command the Piper pose after its controllers are active.",
            ),
            gazebo_resource_path,
            # Register the controller chain before spawning so a fast create process
            # cannot exit before its OnProcessExit handler is ready.
            start_joint_state_broadcaster,
            start_arm_controller,
            start_gripper_controller,
            command_initial_pose,
            gazebo,
            robot_state_publisher,
            spawn_robot,
            bridge,
            image_bridge,
            world_control_bridge,
            ekf_node,
            rviz,
        ]
    )
