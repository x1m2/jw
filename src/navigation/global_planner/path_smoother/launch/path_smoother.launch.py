from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')

    default_config_file = PathJoinSubstitution([
        FindPackageShare('path_smoother'),
        'config',
        'path_smoother.yaml'
    ])

    return LaunchDescription([

        DeclareLaunchArgument(
            'config_file',
            default_value=default_config_file,
            description='Path smoother parameter yaml file'
        ),

        Node(
            package='path_smoother',
            executable='path_smoother_node',
            name='path_smoother',
            output='screen',
            parameters=[config_file],

            remappings=[
                ('global_path', '/global_path'),
                ('smoothed_global_path', '/smoothed_global_path'),
                ('path_visualization', '/path_visualization'),
            ],
        ),
    ])