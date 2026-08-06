import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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


def generate_launch_description():
    package_share = get_package_share_directory('obstacle_stop')
    config_file = LaunchConfiguration('config_file')
    robot_param_file = LaunchConfiguration('robot_param_file')
    runtime_profile = LaunchConfiguration('runtime_profile')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(
                package_share, 'config', 'obstacle_stop.yaml'
            ),
        ),
        DeclareLaunchArgument(
            'robot_param_file',
            default_value=find_robot_param(package_share),
        ),
        DeclareLaunchArgument(
            'runtime_profile',
            default_value='test',
            description='test autostarts; platform waits for robot_runtime',
        ),
        Node(
            package='obstacle_stop',
            executable='obstacle_stop_node',
            output='screen',
            parameters=[
                config_file,
                robot_param_file,
                {'runtime_profile': runtime_profile},
            ],
        ),
    ])
