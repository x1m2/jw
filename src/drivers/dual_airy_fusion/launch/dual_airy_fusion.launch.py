import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_node(context):
    overrides = {}
    for name, value_type in [('auto_time', bool), ('startup_timeout_s', float),
                             ('session_file', str)]:
        value = LaunchConfiguration(name).perform(context)
        if value != '':
            overrides[name] = ParameterValue(
                LaunchConfiguration(name), value_type=value_type)
    return [Node(
        package='dual_airy_fusion',
        executable='dual_airy_fusion_node',
        name='dual_airy_fusion_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file').perform(context), overrides],
        additional_env={
            'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
            'ROS_LOCALHOST_ONLY': '0',
        },
    )]


def generate_launch_description():
    package_share = get_package_share_directory('dual_airy_fusion')
    config_file = os.path.join(package_share, 'config', 'dual_airy_fusion.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file', default_value=config_file,
            description='ROS 2 parameter YAML, including calibration and runtime options.'),
        DeclareLaunchArgument(
            'auto_time', default_value='',
            description='Optional override of auto_time from the parameter file.'),
        DeclareLaunchArgument(
            'startup_timeout_s', default_value='',
            description='Optional startup timeout override in seconds (at least 12).'),
        DeclareLaunchArgument(
            'session_file', default_value='',
            description='Optional session output path override.'),
        OpaqueFunction(function=launch_node),
    ])
