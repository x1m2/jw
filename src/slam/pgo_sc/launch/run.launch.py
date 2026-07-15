import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config_file = os.path.join(
        get_package_share_directory('pgo_sc'),
        'config',
        'pgo.rviz'
    )

    config_file_path = os.path.join(
        get_package_share_directory('pgo_sc'),
        'config',
        'config.yaml'
    )

    pgo_use_rviz_arg = DeclareLaunchArgument(
        "pgo_use_rviz",
        default_value="true",
        description="Whether to launch RViz2"
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time /clock"
    )

    data_dir_arg = DeclareLaunchArgument(
        'data_dir',
        default_value='/robotfs/data/slam_data',
        description='SLAM data root directory'
    )

    return LaunchDescription([
        pgo_use_rviz_arg,
        use_sim_time_arg,
        data_dir_arg,

        Node(
            package='pgo_sc',
            executable='pgo_sc_node',
            parameters=[
                config_file_path,
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'basic.package_path': PathJoinSubstitution([
                        LaunchConfiguration('data_dir'),
                        'map',
                    ]),
                }
            ],
            output='screen'
        ),

        Node(
            condition=IfCondition(LaunchConfiguration("pgo_use_rviz")),
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config_file],
            parameters=[
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time')
                }
            ],
            output="screen"
        ),
    ])
