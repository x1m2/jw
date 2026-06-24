#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_data_dir = os.environ["ROBOT_DATA_DIR"]
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

    global_path_planner_node = Node(
        package='global_path_planner',
        executable='global_path_planner_node',
        name='global_path_planner_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'topology_map_file': os.path.join(
                    robot_data_dir,
                    'topology_map',
                    'connected_topology_map.txt',
                )
            },
        ]
    )

    return LaunchDescription([
        config_file_arg,
        global_path_planner_node,
    ])
