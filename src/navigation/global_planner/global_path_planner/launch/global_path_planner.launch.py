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
    pkg_share = FindPackageShare('global_path_planner')

    default_config_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'global_path_planner_params.yaml'
    ])

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Path to global path planner parameter yaml file'
    )
    runtime_profile = LaunchConfiguration('runtime_profile')
    runtime_profile_arg = DeclareLaunchArgument(
        'runtime_profile',
        default_value='test',
        description='test autostarts; platform waits for slam_manager'
    )

    data_dir_arg = DeclareLaunchArgument(
        'data_dir',
        default_value='/robotfs/data/slam_data',
        description='SLAM data root directory'
    )
    map_id_arg = DeclareLaunchArgument(
        'map_id',
        default_value='0',
        description='Map data directory ID used in test profile'
    )
    map_directory = PythonExpression([
        "'map_' + '", LaunchConfiguration('map_id'), "'"
    ])

    global_path_planner_node = Node(
        package='global_path_planner',
        executable='global_path_planner_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'runtime_profile': runtime_profile,
                'topology_map_file': PathJoinSubstitution([
                    LaunchConfiguration('data_dir'),
                    map_directory,
                    'task',
                    'topology_map.json',
                ]),
                'free_space_pcd_file': PathJoinSubstitution([
                    LaunchConfiguration('data_dir'),
                    map_directory,
                    'map',
                    'result_free_space.pcd',
                ]),
            },
        ]
    )

    return LaunchDescription([
        config_file_arg,
        runtime_profile_arg,
        data_dir_arg,
        map_id_arg,
        global_path_planner_node,
    ])
