#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pcd_publisher_node = Node(
        package='pcd_publisher',
        executable='pcd_publisher',
        name='pcd_publisher',
        output='screen',
        parameters=[
            {
                'pcd_file_path': '/robotfs/data/robot_data/map/result.pcd',
                'frame_id': 'map',
                'topic_name': '/pointcloud',
                'publish_rate': 1.0,
            }
        ]
    )

    return LaunchDescription([
        pcd_publisher_node,
    ])