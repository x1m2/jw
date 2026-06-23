#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


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
            'node_name',
            default_value='navigation_task_state_machine_node',
            description='节点名称'
        ),

        Node(
            package='user_interface',
            executable='navigation_task_state_machine_node',
            name=LaunchConfiguration('node_name'),
            output='screen',
            parameters=[LaunchConfiguration('params_file')]
        )
    ])
