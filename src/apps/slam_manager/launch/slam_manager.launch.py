#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('slam_manager')
    default_params_file = os.path.join(
        package_share,
        'config',
        'slam_manager.yaml',
    )

    params_file = LaunchConfiguration('params_file')
    robot_param_file = LaunchConfiguration('robot_param_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='slam_manager ROS 2 parameter YAML file',
        ),
        DeclareLaunchArgument(
            'robot_param_file',
            default_value=os.path.join(
                package_share, 'config', 'robots', 'P2.5_mid360s/P2.5_mid360s.yaml'),
            description='Robot YAML containing the LiDAR mount, IMU topic and LiDAR-to-IMU rotation',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use the ROS simulation clock',
        ),
        Node(
            package='slam_manager',
            executable='slam_manager_node',
            name='slam_manager',
            output='screen',
            parameters=[
                params_file,
                robot_param_file,
                {'use_sim_time': use_sim_time},
            ],
        ),
    ])
