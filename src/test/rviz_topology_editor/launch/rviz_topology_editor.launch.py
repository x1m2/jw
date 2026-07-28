#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("rviz_topology_editor")

    params_file = LaunchConfiguration("params_file")
    pcd_file = LaunchConfiguration("pcd_file")
    output_file = LaunchConfiguration("output_file")
    connected_topology_file = LaunchConfiguration("connected_topology_file")
    task_file = LaunchConfiguration("task_file")
    use_rviz = LaunchConfiguration("rviz")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(
                package_share, "config", "rviz_topology_editor.yaml"
            ),
        ),
        DeclareLaunchArgument(
            "pcd_file",
            default_value="/robotfs/data/slam_data/map/globalmap.pcd",
        ),
        DeclareLaunchArgument(
            "output_file",
            default_value=(
                "/robotfs/data/slam_data/topology_map/generated_map.txt"
            ),
        ),
        DeclareLaunchArgument(
            "connected_topology_file",
            default_value=(
                "/robotfs/data/slam_data/topology_map/"
                "connected_topology_map.txt"
            ),
        ),
        DeclareLaunchArgument(
            "task_file",
            default_value="/robotfs/data/slam_data/task/test.txt",
        ),
        DeclareLaunchArgument("rviz", default_value="true"),
        Node(
            package="rviz_topology_editor",
            executable="rviz_topology_editor_node",
            name="rviz_topology_editor",
            output="screen",
            parameters=[
                params_file,
                {
                    "pcd_file": pcd_file,
                    "output_file": output_file,
                    "connected_topology_file": connected_topology_file,
                    "task_file": task_file,
                },
            ],
        ),
        Node(
            condition=IfCondition(use_rviz),
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=[
                "-d",
                os.path.join(
                    package_share, "rviz", "rviz_topology_editor.rviz"
                ),
            ],
        ),
    ])
