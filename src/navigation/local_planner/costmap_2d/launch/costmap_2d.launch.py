from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    config_file = LaunchConfiguration('config_file')

    default_config_file = os.path.join(
        get_package_share_directory('costmap_2d'),
        'config',
        'costmap_2d.yaml'
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'config_file',
            default_value=default_config_file,
            description='costmap_2d_node 参数配置文件'
        ),

        Node(
            package='costmap_2d',
            executable='costmap_2d_node',
            output='screen',
            parameters=[config_file],
            remappings=[
                ('/costmap', '/local_costmap/costmap'),
            ]
        ),
    ])
