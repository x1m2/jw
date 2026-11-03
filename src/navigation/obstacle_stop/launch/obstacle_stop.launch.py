import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


MODEL_FILES = {
    'p2.5_mid360s': (
        'obstacle_stop_p2.5_mid360s.yaml',
        'p2.5_mid360s.yaml',
    ),
    'p3_airy96': (
        'obstacle_stop_p3_airy96.yaml',
        'p3_airy96.yaml',
    ),
}


def find_robot_param(package_share, file_name):
    current = os.path.abspath(package_share)
    while True:
        candidate = os.path.join(current, 'src', 'config', file_name)
        if os.path.isfile(candidate):
            return candidate
        parent = os.path.dirname(current)
        if parent == current:
            raise FileNotFoundError(
                f'Cannot find src/config/{file_name}'
            )
        current = parent


def launch_setup(context, package_share):
    robot_model = LaunchConfiguration('robot_model').perform(context)
    if robot_model not in MODEL_FILES:
        supported = ', '.join(MODEL_FILES)
        raise RuntimeError(
            f'Unsupported robot_model "{robot_model}"; choose: {supported}'
        )

    obstacle_file, robot_file = MODEL_FILES[robot_model]
    config_file = os.path.join(package_share, 'config', obstacle_file)
    robot_param_file = find_robot_param(package_share, robot_file)

    overrides = {
        'runtime_profile': LaunchConfiguration(
            'runtime_profile'
        ).perform(context),
    }

    return [Node(
        package='obstacle_stop',
        executable='obstacle_stop_node',
        output='screen',
        parameters=[config_file, robot_param_file, overrides],
    )]


def generate_launch_description():
    package_share = get_package_share_directory('obstacle_stop')

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_model',
            default_value='p2.5_mid360s',
            description='Robot model: p2.5_mid360s or p3_airy96',
        ),
        DeclareLaunchArgument(
            'runtime_profile',
            default_value='test',
            description='test autostarts; platform waits for slam_manager',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz2 with the obstacle stop visualization',
        ),
        OpaqueFunction(function=launch_setup, args=[package_share]),
        Node(
            package='rviz2',
            executable='rviz2',
            name='obstacle_stop_rviz',
            output='screen',
            arguments=['-d', os.path.join(
                package_share, 'config', 'obstacle_stop.rviz')],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
