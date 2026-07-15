import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    config_file_path = os.path.join(
        get_package_share_directory('localization_sc'),
        'config',
        'config.yaml'
    )

    rviz_config_file = os.path.join(
        get_package_share_directory('localization_sc'),
        'config',
        'localization_sc.rviz'
    )

    localization_use_rviz_arg = DeclareLaunchArgument(
        'localization_use_rviz',
        default_value='true',
        description='Whether to launch RViz2'
    )

    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/Odometry'
    )

    lidar_topic_arg = DeclareLaunchArgument(
        'lidar_topic',
        default_value='/cloud_registered'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    data_dir_arg = DeclareLaunchArgument(
        'data_dir',
        default_value='/robotfs/data/slam_data',
        description='SLAM data root directory'
    )

    return LaunchDescription([
        localization_use_rviz_arg,
        odom_topic_arg,
        lidar_topic_arg,
        use_sim_time_arg,
        data_dir_arg,

        Node(
            condition=IfCondition(LaunchConfiguration('localization_use_rviz')),
            package='rviz2',
            executable='rviz2',
            name='localization_rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        ),

        Node(
            package='localization_sc',
            executable='localization_sc_node',
            name='localization_sc_node',
            output='screen',
            parameters=[
                config_file_path,
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'basic.saved_map': PathJoinSubstitution([
                        LaunchConfiguration('data_dir'),
                        'map',
                        'result_bag',
                    ]),
                }
            ],
            remappings=[
                ('/Odometry', LaunchConfiguration('odom_topic')),
                ('/cloud_registered', LaunchConfiguration('lidar_topic')),
            ]
        ),
    ])
