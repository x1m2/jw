#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    topology_share = get_package_share_directory('topology_rviz_plugin')

    target_points_visualizer_launch = os.path.join(
        topology_share,
        'launch',
        'target_points_visualizer.launch.py'
    )

    rviz_config_file = os.path.join(
        topology_share,
        'rviz',
        'topology_demo.rviz'
    )

    topology_tool_config = os.path.join(
        topology_share,
        'config',
        'topology_tool.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'data_dir',
            default_value='/robotfs/data/slam_data',
            description='SLAM data root directory'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(target_points_visualizer_launch),
            launch_arguments={
                'data_dir': LaunchConfiguration('data_dir'),
            }.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            parameters=[
                topology_tool_config,
                {
                    'topology_tool.save_file': PathJoinSubstitution([
                        LaunchConfiguration('data_dir'),
                        'topology_map',
                        'connected_topology_map.txt',
                    ]),
                    'topology_tool.pcd_file': PathJoinSubstitution([
                        LaunchConfiguration('data_dir'),
                        'map',
                        'result_free_space.pcd',
                    ]),
                },
            ]
        ),
    ])
