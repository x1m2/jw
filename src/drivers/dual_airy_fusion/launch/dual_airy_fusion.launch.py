import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('dual_airy_fusion')
    config_file = os.path.join(package_share, 'config', 'dual_airy_fusion.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'extrinsics_file', default_value=config_file,
            description='Spatial calibration YAML read by the fusion node.'),
        DeclareLaunchArgument(
            'auto_time', default_value='true',
            description='Check startup time offset before starting cloud fusion.'),
        DeclareLaunchArgument(
            'startup_timeout_s', default_value='60.0',
            description='Startup time-check timeout in seconds (at least 12).'),
        DeclareLaunchArgument(
            'session_file', default_value='',
            description='Optional output path for session calibration and timing report.'),
        Node(
            package='dual_airy_fusion',
            executable='dual_airy_fusion_node',
            name='dual_airy_fusion_node',
            output='screen',
            parameters=[{
                # This YAML is a calibration document, not a ROS parameter file.
                'extrinsics_file': ParameterValue(
                    LaunchConfiguration('extrinsics_file'), value_type=str),
                'auto_time': ParameterValue(
                    LaunchConfiguration('auto_time'), value_type=bool),
                'startup_timeout_s': ParameterValue(
                    LaunchConfiguration('startup_timeout_s'), value_type=float),
                'session_file': ParameterValue(
                    LaunchConfiguration('session_file'), value_type=str),
            }],
            additional_env={
                'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
                'ROS_LOCALHOST_ONLY': '0',
            },
        ),
    ])
