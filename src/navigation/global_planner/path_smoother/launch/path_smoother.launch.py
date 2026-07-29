from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')
    runtime_profile = LaunchConfiguration('runtime_profile')

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
        DeclareLaunchArgument(
            'runtime_profile',
            default_value='test',
            description='test autostarts; platform waits for robot_runtime'
        ),

        Node(
            package='path_smoother',
            executable='path_smoother_node',
            output='screen',
            parameters=[config_file, {'runtime_profile': runtime_profile}],
        ),
    ])
