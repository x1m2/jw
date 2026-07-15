from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('pure_pursuit_local_planner')

    config_file = LaunchConfiguration('config_file')
    rviz_config = LaunchConfiguration('rviz_config')
    launch_rviz = LaunchConfiguration('launch_rviz')

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

        Node(
            package='pure_pursuit_local_planner',
            executable='pure_pursuit_local_planner_node',
            output='screen',
            parameters=[config_file],
            remappings=[
                ('/global_path', '/smoothed_global_path'),
                ('/local_costmap', '/local_costmap/costmap'),

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
