#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration


def generate_launch_description():
    sim = get_package_share_directory('tracer_gazebo_sim')
    nav = get_package_share_directory('nav2_bringup')
    moveit = get_package_share_directory('tracer_piper_moveit_config')
    grasp = get_package_share_directory('tracer_piper_color_grasp')
    mission = get_package_share_directory('tracer_pick_place_mission')
    clock = LaunchConfiguration('use_sim_time')

    def include(share, file, arguments, **kwargs):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(f'{share}/launch/{file}'),
            launch_arguments=arguments.items(), **kwargs)

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gazebo_gui', default_value=EnvironmentVariable('TRACER_GAZEBO_GUI', default_value='true')),
        DeclareLaunchArgument('rviz', default_value=EnvironmentVariable('TRACER_START_RVIZ', default_value='true')),
        DeclareLaunchArgument('camera_view', default_value=EnvironmentVariable('TRACER_START_CAMERA_VIEW', default_value='true')),
        DeclareLaunchArgument('mission_console', default_value=EnvironmentVariable('TRACER_START_MISSION_CONSOLE', default_value='true')),
        DeclareLaunchArgument('world', default_value=f'{sim}/worlds/tracer_navigation_world.sdf'),
        DeclareLaunchArgument('map', default_value=f'{sim}/maps/tracer_map.yaml'),
        DeclareLaunchArgument('nav2_params', default_value=f'{sim}/config/nav2_params.yaml'),
        DeclareLaunchArgument('mission_params', default_value=f'{mission}/config/mission_params.yaml'),
        # Preserve the requested Nav2 RViz setting before the simulator include
        # overrides its own rviz argument to false. Deferred controller actions
        # in the simulator need the include's launch configurations to persist.
        SetLaunchConfiguration('pick_place_start_rviz', LaunchConfiguration('rviz')),
        include(sim, 'tracer_piper_navigation_scene.launch.py', {
            'use_sim_time': clock, 'rviz': 'false',
            'gazebo_gui': LaunchConfiguration('gazebo_gui'), 'world': LaunchConfiguration('world')}),
        include(nav, 'bringup_launch.py', {
            'map': LaunchConfiguration('map'), 'params_file': LaunchConfiguration('nav2_params'),
            'use_sim_time': clock, 'autostart': 'true', 'use_composition': 'False'}),
        include(nav, 'rviz_launch.py', {
            'use_sim_time': clock, 'rviz_config': f'{sim}/rviz/tracer_piper_nav2.rviz'},
            condition=IfCondition(LaunchConfiguration('pick_place_start_rviz'))),
        include(moveit, 'move_group.launch.py', {'use_sim_time': clock}),
        include(grasp, 'color_grasp_sim.launch.py', {
            'use_sim_time': clock, 'start_stack': 'false',
            'start_debug_view': LaunchConfiguration('camera_view')}),
        include(mission, 'tracer_pick_place_mission.launch.py', {
            'params_file': LaunchConfiguration('mission_params'), 'use_sim_time': clock}),
        ExecuteProcess(
            cmd=[
                'gnome-terminal', '--wait', '--title=TRACER 任务流程日志', '--',
                'bash', '-c',
                'exec ros2 run tracer_pick_place_mission mission_console',
            ],
            output='screen',
            condition=IfCondition(LaunchConfiguration('mission_console')),
        ),
    ])
