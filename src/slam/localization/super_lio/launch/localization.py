import os
import launch.logging
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_super_lio = get_package_share_directory('super_lio')
    relocation_yaml = os.path.join(pkg_super_lio, 'config', 'localization.yaml')
    rviz_config_file = os.path.join(pkg_super_lio, 'rviz', 'localization.rviz')

    declare_config_arg = DeclareLaunchArgument(
        'config',
        default_value='livox_360.yaml',
        description='Robot config yaml that provides the LIO / sensor / extrinsic block'
    )
    config_yaml = PathJoinSubstitution([pkg_super_lio, 'config', LaunchConfiguration('config')])

    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to start RVIZ2'
    )
    rviz_flag = LaunchConfiguration('rviz')

    super_lio_node = Node(
        package='super_lio',
        executable='relocation_node',
        name='relocation_node',
        output='screen',
        parameters=[config_yaml, relocation_yaml],
        arguments=['--ros-args', '--log-level', 'info']
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='super_lio',
        arguments=['-d', rviz_config_file, '--ros-args', '--log-level', 'warn'],
        condition=IfCondition(rviz_flag)
    )

    # static_tf = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_tf_publisher',
    #     arguments=[
    #         '--x', '0.0',
    #         '--y', '0.0',
    #         '--z', '0.0',
    #         '--roll', '0.0',
    #         '--pitch', '0.0',
    #         '--yaw', '0.0',
    #         '--frame-id', 'base_link',
    #         '--child-frame-id', 'imu'
    #     ]
    # )

    ld = LaunchDescription()
    ld.add_action(declare_config_arg)
    ld.add_action(declare_rviz_arg)
    ld.add_action(super_lio_node)
    ld.add_action(rviz2_node)
    # ld.add_action(static_tf)

    return ld