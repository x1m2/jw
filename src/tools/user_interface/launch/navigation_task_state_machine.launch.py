#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    user_interface_share = FindPackageShare('user_interface')

    default_params_file = PathJoinSubstitution([
        user_interface_share,
        'config',
        'navigation_task_state_machine.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='导航任务状态机参数 YAML 文件路径'
        ),

        DeclareLaunchArgument(
            'data_dir',
            default_value='/robotfs/data/slam_data',
            description='SLAM data root directory'
        ),

        DeclareLaunchArgument(
            'map_id',
            default_value='0',
            description='Map ID; defaults to map_0'
        ),

        Node(
            package='user_interface',
            executable='navigation_task_state_machine_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'runtime_profile': 'test',
                    'task_folder_path': PathJoinSubstitution([
                        LaunchConfiguration('data_dir'),
                        PythonExpression([
                            "'map_' + '",
                            LaunchConfiguration('map_id'),
                            "'",
                        ]),
                        'task',
                    ]),
                    'target_points_file': PathJoinSubstitution([
                        LaunchConfiguration('data_dir'),
                        PythonExpression([
                            "'map_' + '",
                            LaunchConfiguration('map_id'),
                            "'",
                        ]),
                        'topology_map',
                        'generated_map_target_points.txt',
                    ]),
                },
            ]
        )
    ])
