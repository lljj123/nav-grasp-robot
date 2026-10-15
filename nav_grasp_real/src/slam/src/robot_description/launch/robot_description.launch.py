from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    default_xacro = PathJoinSubstitution([
        FindPackageShare('robot_description'),
        'urdf',
        'mobile_manipulator.urdf.xacro',
    ])
    model_xacro = LaunchConfiguration('model_xacro')
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_joint_state_publisher = LaunchConfiguration(
        'start_joint_state_publisher')

    robot_description = {
        'robot_description': ParameterValue(
            Command([
                FindExecutable(name='xacro'), ' ', model_xacro,
                ' base_frame:=', LaunchConfiguration('base_frame'),
                ' lidar_frame:=', LaunchConfiguration('lidar_frame'),
                ' manipulator_base_frame:=',
                LaunchConfiguration('manipulator_base_frame'),
                ' lidar_x:=', LaunchConfiguration('lidar_x'),
                ' lidar_y:=', LaunchConfiguration('lidar_y'),
                ' lidar_z:=', LaunchConfiguration('lidar_z'),
                ' lidar_roll:=', LaunchConfiguration('lidar_roll'),
                ' lidar_pitch:=', LaunchConfiguration('lidar_pitch'),
                ' lidar_yaw:=', LaunchConfiguration('lidar_yaw'),
                ' manipulator_x:=', LaunchConfiguration('manipulator_x'),
                ' manipulator_y:=', LaunchConfiguration('manipulator_y'),
                ' manipulator_z:=', LaunchConfiguration('manipulator_z'),
                ' manipulator_yaw:=', LaunchConfiguration('manipulator_yaw'),
            ]),
            value_type=str,
        )
    }

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'model_xacro',
            default_value=default_xacro,
            description=(
                'Xacro accepting the frame and mount arguments declared here.')),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('lidar_frame', default_value='livox_frame'),
        DeclareLaunchArgument(
            'manipulator_base_frame', default_value='piper_base_link'),
        DeclareLaunchArgument('lidar_x', default_value='0.15'),
        DeclareLaunchArgument('lidar_y', default_value='0.0'),
        DeclareLaunchArgument('lidar_z', default_value='0.10'),
        DeclareLaunchArgument('lidar_roll', default_value='0.0'),
        DeclareLaunchArgument('lidar_pitch', default_value='0.0'),
        DeclareLaunchArgument('lidar_yaw', default_value='0.0'),
        DeclareLaunchArgument('manipulator_x', default_value='0.0'),
        DeclareLaunchArgument('manipulator_y', default_value='0.0'),
        DeclareLaunchArgument('manipulator_z', default_value='0.15'),
        DeclareLaunchArgument('manipulator_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'start_joint_state_publisher',
            default_value='false',
            description='Enable only when the selected model has movable joints.'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description, {'use_sim_time': use_sim_time}],
        ),
        Node(
            condition=IfCondition(start_joint_state_publisher),
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[robot_description, {'use_sim_time': use_sim_time}],
        ),
    ])

