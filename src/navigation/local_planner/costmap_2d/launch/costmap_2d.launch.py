from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

import os


def find_robot_param(package_share):
    current_path = os.path.abspath(package_share)
    while True:
        candidate = os.path.join(current_path, 'src', 'config', 'p2.5_mid360s.yaml')
        if os.path.isfile(candidate):
            return candidate

        parent_path = os.path.dirname(current_path)
        if parent_path == current_path:
            raise FileNotFoundError(
                'Cannot find src/config/p2.5_mid360s.yaml from package share: '
                + package_share
            )
        current_path = parent_path


def generate_launch_description():

    config_file = LaunchConfiguration('config_file')
    robot_param_file = LaunchConfiguration('robot_param_file')
    runtime_profile = LaunchConfiguration('runtime_profile')
    global_frame = LaunchConfiguration('global_frame')
    lidar_type = LaunchConfiguration('lidar_type')
    livox_topic = LaunchConfiguration('livox_topic')
    lidar_frame = LaunchConfiguration('lidar_frame')
    pointcloud2_in_base_frame = LaunchConfiguration(
        'pointcloud2_in_base_frame'
    )
    pointcloud2_apply_height_filter = LaunchConfiguration(
        'pointcloud2_apply_height_filter'
    )
    clear_unobserved_rays = LaunchConfiguration('clear_unobserved_rays')

    package_share = get_package_share_directory('costmap_2d')
    default_config_file = os.path.join(
        package_share,
        'config',
        'costmap_2d.yaml'
    )
    default_robot_param_file = find_robot_param(package_share)

    return LaunchDescription([

        DeclareLaunchArgument(
            'config_file',
            default_value=default_config_file,
            description='costmap_2d_node 参数配置文件'
        ),
        DeclareLaunchArgument(
            'robot_param_file',
            default_value=default_robot_param_file,
            description='机器人尺寸和雷达外参公共配置文件'
        ),
        DeclareLaunchArgument(
            'runtime_profile',
            default_value='test',
            description='test自动启动算法；platform等待slam_manager启动'
        ),
        DeclareLaunchArgument(
            'global_frame',
            default_value='map',
            description='Costmap global frame; use odom for FAST-LIO UWB follow'
        ),
        DeclareLaunchArgument(
            'lidar_type',
            default_value='2',
            description='1 uses Livox CustomMsg; other values use PointCloud2'
        ),
        DeclareLaunchArgument(
            'livox_topic',
            default_value='/obstacle_cloud',
            description='Input Livox or PointCloud2 topic'
        ),
        DeclareLaunchArgument(
            'lidar_frame',
            default_value='base_link',
            description='Configured input point-cloud frame'
        ),
        DeclareLaunchArgument(
            'pointcloud2_in_base_frame',
            default_value='true',
            description='PointCloud2 input is already expressed in base_link'
        ),
        DeclareLaunchArgument(
            'pointcloud2_apply_height_filter',
            default_value='true',
            description='Apply min/max height clipping to PointCloud2 input'
        ),
        DeclareLaunchArgument(
            'clear_unobserved_rays',
            default_value='false',
            description='Treat empty projected angle bins as confirmed free rays'
        ),

        Node(
            package='costmap_2d',
            executable='costmap_2d_node',
            output='screen',
            parameters=[
                config_file,
                robot_param_file,
                {
                    'runtime_profile': runtime_profile,
                    'global_frame': global_frame,
                    'lidar_type': ParameterValue(lidar_type, value_type=int),
                    'livox_topic': livox_topic,
                    'lidar_frame': lidar_frame,
                    'pointcloud2_in_base_frame': ParameterValue(
                        pointcloud2_in_base_frame, value_type=bool
                    ),
                    'pointcloud2_apply_height_filter': ParameterValue(
                        pointcloud2_apply_height_filter, value_type=bool
                    ),
                    'clear_unobserved_rays': ParameterValue(
                        clear_unobserved_rays, value_type=bool
                    ),
                }
            ]
        ),
    ])
