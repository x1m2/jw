import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def find_lidar_config(package_share):
    current_path = os.path.abspath(package_share)
    while True:
        candidate = os.path.join(current_path, 'src', 'config', 'lidar.yaml')
        if os.path.isfile(candidate):
            return candidate

        parent_path = os.path.dirname(current_path)
        if parent_path == current_path:
            raise FileNotFoundError(
                'Cannot find src/config/lidar.yaml from package share: '
                + package_share
            )
        current_path = parent_path


def generate_launch_description():
    package_share = get_package_share_directory('fastlio_localization')
    config = os.path.join(package_share, 'config', 'ros2_param.yaml')
    lidar_config = find_lidar_config(package_share)
    rviz_config = os.path.join(
        package_share, 'rviz_cfg', 'localization_ros2.rviz')

    localization_use_rviz_arg = DeclareLaunchArgument(
        'localization_use_rviz',
        default_value='true',
        description='是否启动定位 RViz 界面'
    )

    data_dir_arg = DeclareLaunchArgument(
        'data_dir',
        default_value='/robotfs/data/slam_data',
        description='SLAM data root directory'
    )

    map_id_arg = DeclareLaunchArgument(
        'map_id',
        default_value='0',
        description='Map ID; defaults to map_0'
    )

    map_directory = PythonExpression([
        "'map_' + '", LaunchConfiguration('map_id'), "'"
    ])

    fastlio_localization = Node(
        package="fastlio_localization",
        executable="fastlio_localization_ros2",
        prefix=['stdbuf -o L'],
        output='screen',
        parameters=[
            config,
            lidar_config,
            {
                'runtime_profile': 'test',
                'map_path': PathJoinSubstitution([
                    LaunchConfiguration('data_dir'),
                    map_directory,
                    'map',
                ]),
            },
        ])
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('localization_use_rviz')),
    )
    return LaunchDescription([
        localization_use_rviz_arg,
        data_dir_arg,
        map_id_arg,
        fastlio_localization,
        rviz2,
    ])
