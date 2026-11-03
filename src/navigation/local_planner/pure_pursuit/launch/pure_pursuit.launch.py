import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def find_robot_param(package_share):
    current = os.path.abspath(package_share)
    while True:
        candidate = os.path.join(
            current, 'src', 'config', 'p2.5_mid360s.yaml'
        )
        if os.path.isfile(candidate):
            return candidate
        parent = os.path.dirname(current)
        if parent == current:
            raise FileNotFoundError(
                'Cannot find src/config/p2.5_mid360s.yaml'
            )
        current = parent


def generate_launch_description():
    pkg_share = FindPackageShare('pure_pursuit')
    package_share = get_package_share_directory('pure_pursuit')

    config_file = LaunchConfiguration('config_file')
    robot_param_file = LaunchConfiguration('robot_param_file')
    rviz_config = LaunchConfiguration('rviz_config')
    launch_rviz = LaunchConfiguration('launch_rviz')
    runtime_profile = LaunchConfiguration('runtime_profile')
    use_sim_time = LaunchConfiguration('use_sim_time')
    global_frame = LaunchConfiguration('global_frame')

    return LaunchDescription([

        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                pkg_share,
                'config',
                'pure_pursuit_params.yaml'
            ]),
            description='Pure pursuit and path smoother parameter yaml file'
        ),
        DeclareLaunchArgument(
            'robot_param_file',
            default_value=find_robot_param(package_share),
            description='机器人尺寸和雷达外参公共配置文件'
        ),

        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                pkg_share,
                'rviz',
                'pure_pursuit.rviz'
            ]),
            description='RViz config file'
        ),

        DeclareLaunchArgument(
            'launch_rviz',
            default_value='false',
            description='Whether to launch RViz2'
        ),
        DeclareLaunchArgument(
            'runtime_profile',
            default_value='test',
            description='test autostarts; platform waits for slam_manager'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock from /clock'
        ),
        DeclareLaunchArgument(
            'global_frame',
            default_value='map',
            description='Tracking frame; use odom for FAST-LIO UWB follow'
        ),

        Node(
            package='pure_pursuit',
            executable='pure_pursuit_node',
            output='screen',
            parameters=[
                config_file,
                robot_param_file,
                {
                    'runtime_profile': runtime_profile,
                    'use_sim_time': use_sim_time,
                    'global_frame': global_frame,
                }
            ],
        ),

        # Node(
        #     condition=IfCondition(launch_rviz),
        #     package='rviz2',
        #     executable='rviz2',
        #     arguments=['-d', rviz_config],
        #     output='screen',
        # ),
    ])
