import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    package_share = get_package_share_directory('nav')
    nav2_share = get_package_share_directory('nav2_bringup')
    description_share = get_package_share_directory('robot_description')

    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_rviz = LaunchConfiguration('start_rviz')
    start_robot_description = LaunchConfiguration('start_robot_description')
    start_joint_state_publisher = LaunchConfiguration(
        'start_joint_state_publisher')
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites={
            'default_nav_to_pose_bt_xml': os.path.join(
                package_share,
                'behavior_trees',
                'navigate_to_pose_no_backup.xml',
            ),
            'default_nav_through_poses_bt_xml': os.path.join(
                package_share,
                'behavior_trees',
                'navigate_through_poses_no_backup.xml',
            ),
        },
        convert_types=True,
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_file,
            'params_file': configured_params,
            'slam': 'False',
            'use_sim_time': use_sim_time,
            'autostart': 'True',
            'use_composition': 'False',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=os.environ.get('NAV_GRASP_MAP', ''),
            description=(
                'Absolute path to a map YAML. Set map:=... or NAV_GRASP_MAP.')),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(package_share, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('start_robot_description', default_value='true'),
        DeclareLaunchArgument(
            'start_joint_state_publisher', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                description_share, 'launch', 'robot_description.launch.py')),
            condition=IfCondition(start_robot_description),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'start_joint_state_publisher': start_joint_state_publisher,
            }.items(),
        ),
        nav2_launch,
        Node(
            condition=IfCondition(start_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(package_share, 'rviz', 'navigation.rviz')],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
