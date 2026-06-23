#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    topology_share = get_package_share_directory("topology_rviz_plugin")

    config_file = os.path.join(
        topology_share,
        "config",
        "topology_connect.yaml"
    )

    run_once = LaunchConfiguration("run_once")

    return LaunchDescription([
        DeclareLaunchArgument(
            "run_once",
            default_value="true",
            description="true: 连边后退出；false: 连边后持续发布 RViz Marker"
        ),

        Node(
            package="topology_rviz_plugin",
            executable="topology_connect.py",
            name="topology_connect_node",
            output="screen",
            parameters=[
                config_file,
                {
                    "run_once": run_once,
                    "publish_markers": True,
                    "marker_topic": "/topology_markers",
                    "frame_id": "map",
                    "publish_period": 1.0,
                }
            ],
        )
    ])