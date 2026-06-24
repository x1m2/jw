#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    robot_data_dir = os.environ["ROBOT_DATA_DIR"]
    pcd_publisher_node = Node(
        package='pcd_publisher',
        executable='pcd_publisher',
        name='pcd_publisher',
        output='screen',
        parameters=[
            {
                'pcd_file_path': os.path.join(
                    robot_data_dir,
                    'map',
                    'result.pcd',
                ),
                'frame_id': 'map',
                'topic_name': '/pointcloud',
                'publish_rate': 1.0,
            }
        ]
    )

    return LaunchDescription([
        pcd_publisher_node,
    ])
