import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_super_lio = get_package_share_directory('super_lio')
    rviz_config_file = os.path.join(pkg_super_lio, 'rviz', 'localization.rviz')

    declare_config_arg = DeclareLaunchArgument(
        'config',
        default_value='reloc_superlio_mid360s.yaml',
        description='Complete sensor and relocalization config yaml'
    )
    config_file_arg = DeclareLaunchArgument('config_file', default_value=LaunchConfiguration('config'))
    config_yaml = PathJoinSubstitution([pkg_super_lio, 'config', LaunchConfiguration('config_file')])
    robot_param_arg = DeclareLaunchArgument('robot_param_file', default_value='',
        description='Optional robot YAML providing lidar_to_base_T/R')
    map_path_arg = DeclareLaunchArgument('map_path', default_value='',
        description='Prior map directory; empty uses the selected YAML')
    runtime_arg = DeclareLaunchArgument('runtime_profile', default_value='test',
        description='test autostarts; platform waits for Worker START')

    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to start RVIZ2'
    )
    rviz_flag = LaunchConfiguration('rviz')

    def localization_node(context):
        parameters = [config_yaml]
        robot_file = LaunchConfiguration('robot_param_file').perform(context)
        if robot_file:
            parameters.append(robot_file)
        overrides = {'runtime_profile': LaunchConfiguration('runtime_profile').perform(context)}
        map_path = LaunchConfiguration('map_path').perform(context)
        if map_path:
            overrides['map_path'] = map_path
        parameters.append(overrides)
        return [Node(package='super_lio', executable='super_lio_localization_node',
                     output='screen', parameters=parameters)]

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='super_lio',
        arguments=['-d', rviz_config_file, '--ros-args', '--log-level', 'warn'],
        condition=IfCondition(rviz_flag)
    )

    ld = LaunchDescription()
    ld.add_action(declare_config_arg)
    ld.add_action(config_file_arg)
    ld.add_action(robot_param_arg)
    ld.add_action(map_path_arg)
    ld.add_action(runtime_arg)
    ld.add_action(declare_rviz_arg)
    ld.add_action(OpaqueFunction(function=localization_node))
    ld.add_action(rviz2_node)

    return ld
