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

def generate_launch_description():
    config = os.path.join(get_package_share_directory('fastlio_localization'), 'config', 'ros2_param.yaml')
    rviz_config = os.path.join(get_package_share_directory('fastlio_localization'), 'rviz_cfg', 'localization_ros2.rviz')

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
            {
                'runtime_profile': 'test',
                'map_path': PathJoinSubstitution([
                    LaunchConfiguration('data_dir'),
                    map_directory,
                    'map',
                ]),
            },
        ])
    # fastlio_localization = Node(package="fastlio_localization", executable="fastlio_localization_ros2", prefix=['gdb -ex run --args'], output='screen', parameters=[config])
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('localization_use_rviz')),
    )
    map_publisher = Node(package='pcl_ros', executable='pointcloud_to_pcd', output='screen',
                         parameters=[{'file_name': '/path/to/your/pcd/file.pcd', 'frame_id': 'map'}],
                         remappings=[('/cloud_pcd', '/your/pointcloud_topic')])

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_publisher',
        arguments=[
            '--x', '-0.28',
            '--y', '0.0',
            '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'lidar'
        ]
    )
    return LaunchDescription([
        localization_use_rviz_arg,
        data_dir_arg,
        map_id_arg,
        fastlio_localization,
        rviz2,
        static_tf,
    ])
