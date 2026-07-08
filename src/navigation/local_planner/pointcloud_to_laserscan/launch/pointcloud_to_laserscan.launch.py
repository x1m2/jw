#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    cloud_in_topic = LaunchConfiguration('cloud_in')
    scan_topic = LaunchConfiguration('scan_topic')
    lidar_frame = LaunchConfiguration('lidar_frame')
    target_frame = LaunchConfiguration('target_frame')

    return LaunchDescription([

        DeclareLaunchArgument(
            'cloud_in',
            default_value='/livox/pointcloud2',
            description='Input PointCloud2 topic for pointcloud_to_laserscan'
        ),

        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan',
            description='Output LaserScan topic'
        ),

        DeclareLaunchArgument(
            'lidar_frame',
            default_value='livox_frame',
            description='LiDAR frame id'
        ),

        DeclareLaunchArgument(
            'target_frame',
            default_value='base_link',
            description='Target frame for generated LaserScan'
        ),

        # 1. 发布静态 TF：base_link -> livox_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_livox_frame',
            arguments=[
                '0.28', '0.0', '0.70',      # x y z: 雷达在 base_link 前方0.28m，高0.70m
                '0.0', '0.4119', '0.0',     # roll pitch yaw: 约24.5deg
                'base_link',            # parent frame
                lidar_frame             # child frame
            ],
            output='screen'
        ),

        # 2. PointCloud2 -> LaserScan
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',

            remappings=[
                ('cloud_in', cloud_in_topic),
                ('scan', scan_topic),
            ],

            parameters=[{
                # 将点云转换到 base_link 坐标系下再生成 LaserScan
                'target_frame': target_frame,
                'transform_tolerance': 0.05,

                # 按 base_link 坐标系下的 z 高度裁剪点云
                'min_height': 0.2,
                'max_height': 1.3,

                # 360 度输出 LaserScan
                'angle_min': -3.1415926,
                'angle_max': 3.1415926,
                'angle_increment': 0.0087,

                # MID-360 常见 10Hz 左右
                'scan_time': 0.1,

                'range_min': 0.4,
                'range_max': 8.0,

                'use_inf': True,
                'inf_epsilon': 1.0,
            }]
        ),
    ])