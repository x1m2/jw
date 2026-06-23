from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_name = 'dual_radar_traversable_extractor'
    default_config = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'dual_radar_config.yaml'
    )

    config_file = LaunchConfiguration('config_file')
    use_dual_radar = LaunchConfiguration('use_dual_radar')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Path to dual radar traversable extractor parameter file'
        ),
        DeclareLaunchArgument(
            'use_dual_radar',
            default_value='false',
            description='Whether to enable dual radar mode'
        ),
        Node(
            package=package_name,
            executable='dual_radar_traversable_extractor_node',
            name='dual_radar_traversable_extractor',
            output='screen',
            parameters=[
                config_file,
                {'use_dual_radar': ParameterValue(use_dual_radar, value_type=bool)},
            ],
        ),
    ])
