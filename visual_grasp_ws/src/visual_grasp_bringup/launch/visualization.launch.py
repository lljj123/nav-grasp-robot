"""Start the read-only RViz view for the PiPER D435 grasp stack."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def _launch_setup(context):
    description_share = Path(get_package_share_directory(
        'agx_arm_description'))
    xacro_file = description_share / 'urdf' / 'agx_arm_description.urdf.xacro'
    if not xacro_file.is_file():
        raise FileNotFoundError(f'PiPER model xacro not found: {xacro_file}')

    robot_description = xacro.process_file(
        str(xacro_file),
        mappings={
            'arm_type': 'piper_h',
            'end_effector': 'gripper',
            'revo2_side': 'right',
            # The calibrated D435 TF remains authoritative.  Do not introduce
            # a second nominal camera transform through the display URDF.
            'with_camera_stand': 'false',
            'with_camera': 'false',
        },
    ).toxml()

    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    joint_states_topic = LaunchConfiguration(
        'joint_states_topic').perform(context)

    return [
        Node(
            package='visual_grasp_bringup',
            executable='model_joint_state_adapter',
            name='model_joint_state_adapter',
            output='screen',
            parameters=[{
                'input_topic': joint_states_topic,
                'output_topic': '/visual_grasp/model_joint_states',
                'max_gripper_width_m': 0.080,
            }],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='visual_grasp',
            name='model_robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                # The live grasp TF already contains piper_base_link ->
                # piper_link6.  A separate prefix prevents a second TF tree
                # from giving the same child frame two parents.
                'frame_prefix': 'model_',
            }],
            remappings=[
                ('joint_states', '/visual_grasp/model_joint_states'),
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='visual_model_root_tf',
            output='screen',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.0',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                '--frame-id', 'piper_base_link',
                '--child-frame-id', 'model_world',
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='visual_grasp_rviz',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            arguments=['-d', rviz_config],
        ),
    ]


def generate_launch_description():
    bringup_share = Path(get_package_share_directory('visual_grasp_bringup'))
    default_rviz = bringup_share / 'rviz' / 'visual_grasp.rviz'
    return LaunchDescription([
        DeclareLaunchArgument(
            'joint_states_topic',
            default_value='/arm/feedback/joint_states',
            description='Live PiPER joint feedback topic.'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=str(default_rviz),
            description='RViz2 configuration file.'),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start the RViz2 GUI.'),
        OpaqueFunction(function=_launch_setup),
    ])
