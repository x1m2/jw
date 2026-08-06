#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def package_file(package, *parts):
    return os.path.join(get_package_share_directory(package), *parts)


def find_robot_param(package_share):
    current = os.path.abspath(package_share)
    while True:
        candidate = os.path.join(current, 'src', 'config', 'robot_param.yaml')
        if os.path.isfile(candidate):
            return candidate
        parent = os.path.dirname(current)
        if parent == current:
            raise FileNotFoundError('Cannot find src/config/robot_param.yaml')
        current = parent


def include_launch(package, launch_file, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(package_file(package, 'launch', launch_file)),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    data_root = LaunchConfiguration('data_root')
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_param = find_robot_param(get_package_share_directory('fast_lio'))

    # 与run_slam.sh保持一致：启动MID360s驱动，为FastLIO提供点云和IMU。
    livox_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            package_file(
                'livox_ros_driver2', 'launch_ROS2', 'msg_MID360s_launch.py'
            )
        )
    )

    runtime = Node(
        package='robot_runtime',
        executable='robot_runtime_node',
        output='screen',
        parameters=[
            package_file('robot_runtime', 'config', 'robot_runtime.yaml'),
            {
                'map_storage.root_directory': data_root,
                'use_sim_time': use_sim_time,
            },
        ],
    )

    # 这三个 launch 原本固定为 test 模式，因此在这里直接启动 Worker 外壳。
    pgo_worker = Node(
        package='pgo_sc',
        executable='pgo_sc_node',
        output='screen',
        parameters=[
            package_file('pgo_sc', 'config', 'config.yaml'),
            {'runtime_profile': 'platform', 'use_sim_time': use_sim_time},
        ],
    )
    fast_lio_worker = Node(
        package='fast_lio',
        executable='fastlio_mapping_node',
        output='screen',
        parameters=[
            package_file('fast_lio', 'config', 'mid360.yaml'),
            robot_param,
            {'runtime_profile': 'platform', 'use_sim_time': use_sim_time},
        ],
    )
    localization_worker = Node(
        package='fastlio_localization',
        executable='fastlio_localization_ros2',
        output='screen',
        prefix=['stdbuf -o L'],
        parameters=[
            package_file('fastlio_localization', 'config', 'ros2_param.yaml'),
            robot_param,
            {'runtime_profile': 'platform', 'use_sim_time': use_sim_time},
        ],
    )

    platform_args = {'runtime_profile': 'platform'}
    navigation_workers = [
        include_launch('obstacle_stop', 'obstacle_stop.launch.py', platform_args),
        include_launch('costmap_2d', 'costmap_2d.launch.py', platform_args),
        include_launch(
            'global_path_planner',
            'global_path_planner.launch.py',
            {**platform_args, 'data_dir': data_root},
        ),
        include_launch('path_smoother', 'path_smoother.launch.py', platform_args),
        include_launch(
            'pure_pursuit_local_planner',
            'pure_pursuit_local_planner.launch.py',
            {**platform_args, 'launch_rviz': 'false'},
        ),
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'data_root',
            default_value='/robotfs/data/slam_data',
            description='Map storage root used by robot_runtime',
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use /clock when testing with a rosbag',
        ),
        livox_driver,
        runtime,
        pgo_worker,
        fast_lio_worker,
        localization_worker,
        *navigation_workers,
    ])
