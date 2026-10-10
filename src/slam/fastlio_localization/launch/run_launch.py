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


def find_robot_param(package_share):
    current_path = os.path.abspath(package_share)
    while True:
        candidate = os.path.join(
            current_path, 'src', 'config', 'robot_param.yaml')
        if os.path.isfile(candidate):
            return candidate

        parent_path = os.path.dirname(current_path)
        if parent_path == current_path:
            raise FileNotFoundError(
                'Cannot find src/config/robot_param.yaml from package share: '
                + package_share
            )
        current_path = parent_path


def generate_launch_description():
    package_share = get_package_share_directory('fastlio_localization')
    config = PathJoinSubstitution([
        package_share, 'config', LaunchConfiguration('config_file')
    ])
    default_robot_param = find_robot_param(package_share)
    rviz_config = os.path.join(
        package_share, 'rviz_cfg', 'localization_ros2.rviz')

    localization_use_rviz_arg = DeclareLaunchArgument(
        'localization_use_rviz',
        default_value='true',
        description='是否启动定位 RViz 界面'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='ros2_param.yaml',
        description='Localization ROS 2 parameter file in the config directory'
    )

    robot_param_file_arg = DeclareLaunchArgument(
        'robot_param_file',
        default_value=default_robot_param,
        description='Robot dimensions and LiDAR pose parameter file'
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

    map_path_arg = DeclareLaunchArgument(
        'map_path',
        default_value=PathJoinSubstitution([
            LaunchConfiguration('data_dir'), map_directory, 'map'
        ]),
        description='Directory containing globalmap.pcd and BBS3D voxel maps'
    )

    fastlio_localization = Node(
        package="fastlio_localization",
        executable="fastlio_localization_ros2",
        prefix=['stdbuf -o L'],
        output='screen',
        parameters=[
            config,
            LaunchConfiguration('robot_param_file'),
            {
                'runtime_profile': 'test',
                'map_path': LaunchConfiguration('map_path'),
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
        config_file_arg,
        robot_param_file_arg,
        data_dir_arg,
        map_id_arg,
        map_path_arg,
        fastlio_localization,
        rviz2,
    ])
