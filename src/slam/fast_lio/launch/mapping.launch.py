import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node


def find_robot_param(package_share):
    current_path = os.path.abspath(package_share)
    while True:
        candidate = os.path.join(current_path, 'src', 'config', 'robot_param.yaml')
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
    package_path = get_package_share_directory('fast_lio')
    default_robot_param_path = find_robot_param(package_path)
    default_config_path = os.path.join(package_path, 'config')
    default_rviz_config_path = os.path.join(
        package_path, 'rviz', 'fastlio.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    robot_param_file = LaunchConfiguration('robot_param_file')
    rviz_cfg = LaunchConfiguration('rviz_cfg')

    use_rviz_arg = DeclareLaunchArgument(
        "fast_lio_use_rviz",
        default_value="false",
        description="Whether to launch Rviz2"

    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='mapping_mid360s.yaml',
        description='Config file'
    )
    declare_robot_param_file_cmd = DeclareLaunchArgument(
        'robot_param_file',
        default_value=default_robot_param_path,
        description='Robot dimensions and LiDAR pose shared by navigation'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping_node',
        parameters=[PathJoinSubstitution([config_path, config_file]),
                    robot_param_file,
                    {
                        'use_sim_time': use_sim_time,
                        'runtime_profile': 'test',
                    }],
        output='screen'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(LaunchConfiguration('fast_lio_use_rviz'))
    )

    ld = LaunchDescription()
    ld.add_action(use_rviz_arg)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_robot_param_file_cmd)
    ld.add_action(declare_rviz_config_path_cmd)

    ld.add_action(fast_lio_node)
    ld.add_action(rviz_node)

    return ld
