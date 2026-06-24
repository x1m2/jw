from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    robot_data_dir = os.environ["ROBOT_DATA_DIR"]

    config_file = LaunchConfiguration('config_file')

    default_config_file = os.path.join(
        get_package_share_directory('topology_map_generator'),
        'config',
        'topology_map_generator.yaml'
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'config_file',
            default_value=default_config_file,
            description='topology_map_generator 参数配置文件'
        ),

        Node(
            package='topology_map_generator',
            executable='topology_map_generator_node',
            name='topology_map_generator',
            output='screen',
            parameters=[
                config_file,
                {
                    'output_file': os.path.join(
                        robot_data_dir,
                        'topology_map',
                        'generated_map.txt',
                    )
                },
            ]
        )
    ])
