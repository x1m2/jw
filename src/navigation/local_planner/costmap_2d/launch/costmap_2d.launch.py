from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def find_robot_param(package_share):
    current_path = os.path.abspath(package_share)
    while True:
        candidate = os.path.join(current_path, 'src', 'config', 'robot_param.yaml')
        if os.path.isfile(candidate):
            return candidate

        parent_path = os.path.dirname(current_path)
        if parent_path == current_path:
            raise FileNotFoundError(
                'Cannot find src/config/robot_param.yaml from package share: '
                + package_share
            )
        current_path = parent_path


def generate_launch_description():

    config_file = LaunchConfiguration('config_file')
    robot_param_file = LaunchConfiguration('robot_param_file')
    runtime_profile = LaunchConfiguration('runtime_profile')
    global_frame = LaunchConfiguration('global_frame')

    package_share = get_package_share_directory('costmap_2d')
    default_config_file = os.path.join(
        package_share,
        'config',
        'costmap_2d.yaml'
    )
    default_robot_param_file = find_robot_param(package_share)

    return LaunchDescription([

        DeclareLaunchArgument(
            'config_file',
            default_value=default_config_file,
            description='costmap_2d_node 参数配置文件'
        ),
        DeclareLaunchArgument(
            'robot_param_file',
            default_value=default_robot_param_file,
            description='机器人尺寸和雷达外参公共配置文件'
        ),
        DeclareLaunchArgument(
            'runtime_profile',
            default_value='test',
            description='test自动启动算法；platform等待robot_runtime启动'
        ),
        DeclareLaunchArgument(
            'global_frame',
            default_value='map',
            description='Costmap global frame; use odom for FAST-LIO UWB follow'
        ),

        Node(
            package='costmap_2d',
            executable='costmap_2d_node',
            output='screen',
            parameters=[
                config_file,
                robot_param_file,
                {
                    'runtime_profile': runtime_profile,
                    'global_frame': global_frame,
                }
            ],
            remappings=[
                ('/costmap', '/local_costmap/costmap'),
            ]
        ),
    ])
