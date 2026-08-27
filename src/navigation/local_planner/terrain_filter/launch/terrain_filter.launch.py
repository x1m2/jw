from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_name = 'terrain_filter'
    default_config = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'terrain_filter.yaml'
    )

    config_file = LaunchConfiguration('config_file')
    use_dual_radar = LaunchConfiguration('use_dual_radar')
    runtime_profile = LaunchConfiguration('runtime_profile')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Path to terrain filter parameter file'
        ),
        DeclareLaunchArgument(
            'use_dual_radar',
            default_value='false',
            description='Whether to enable dual radar mode'
        ),
        DeclareLaunchArgument(
            'runtime_profile',
            default_value='test',
            description='test autostarts; platform waits for a Worker command'
        ),
        Node(
            package=package_name,
            executable='terrain_filter_node',
            output='screen',
            parameters=[
                config_file,
                {'runtime_profile': runtime_profile},
                {'use_dual_radar': ParameterValue(use_dual_radar, value_type=bool)},
            ],
        ),
    ])
