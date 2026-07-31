from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def find_lidar_config(package_share):
    current_path = os.path.abspath(package_share)
    while True:
        candidate = os.path.join(current_path, 'src', 'config', 'lidar.yaml')
        if os.path.isfile(candidate):
            return candidate

        parent_path = os.path.dirname(current_path)
        if parent_path == current_path:
            raise FileNotFoundError(
                'Cannot find src/config/lidar.yaml from package share: '
                + package_share
            )
        current_path = parent_path


def generate_launch_description():

    config_file = LaunchConfiguration('config_file')
    lidar_config_file = LaunchConfiguration('lidar_config_file')
    runtime_profile = LaunchConfiguration('runtime_profile')

    package_share = get_package_share_directory('costmap_2d')
    default_config_file = os.path.join(
        package_share,
        'config',
        'costmap_2d.yaml'
    )
    default_lidar_config_file = find_lidar_config(package_share)

    return LaunchDescription([

        DeclareLaunchArgument(
            'config_file',
            default_value=default_config_file,
            description='costmap_2d_node 参数配置文件'
        ),
        DeclareLaunchArgument(
            'lidar_config_file',
            default_value=default_lidar_config_file,
            description='雷达到 base_link 的公共外参配置文件'
        ),
        DeclareLaunchArgument(
            'runtime_profile',
            default_value='test',
            description='test自动启动算法；platform等待robot_runtime启动'
        ),

        Node(
            package='costmap_2d',
            executable='costmap_2d_node',
            output='screen',
            parameters=[
                config_file,
                lidar_config_file,
                {'runtime_profile': runtime_profile}
            ],
            remappings=[
                ('/costmap', '/local_costmap/costmap'),
            ]
        ),
    ])
