#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    topology_share = get_package_share_directory('topology_rviz_plugin')

    config_file = os.path.join(
        topology_share,
        'config',
        'target_points_visualizer.yaml'
    )

    rviz_config_file = os.path.join(
        topology_share,
        'rviz',
        'target_points_visualization.rviz'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Whether to start RViz2'
    )

    data_dir_arg = DeclareLaunchArgument(
        'data_dir',
        default_value='/robotfs/data/slam_data',
        description='SLAM data root directory'
    )

    target_points_visualizer_node = Node(
        package='topology_rviz_plugin',
        executable='target_points_visualizer_node',
        output='screen',
        parameters=[
            config_file,
            {
                'target_points_file': PathJoinSubstitution([
                    LaunchConfiguration('data_dir'),
                    'topology_map',
                    'generated_map_target_points.txt',
                ]),
            },
        ]
    )

    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        rviz_arg,
        data_dir_arg,
        target_points_visualizer_node,
        # rviz_node,
    ])
