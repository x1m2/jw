from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution([
        FindPackageShare('uwb_follow_planner'),
        'config',
        'uwb_follow_planner.yaml',
    ])
    default_rviz_config = PathJoinSubstitution([
        FindPackageShare('uwb_follow_planner'),
        'rviz',
        'uwb_follow.rviz',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='UWB follow planner parameter file',
        ),
        DeclareLaunchArgument(
            'runtime_profile',
            default_value='test',
            description='test autostarts; platform waits for slam_manager',
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Whether to launch the UWB follow RViz view',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_config,
            description='UWB follow RViz config file',
        ),
        Node(
            package='uwb_follow_planner',
            executable='uwb_follow_planner_node',
            name='uwb_follow_planner_node',
            output='screen',
            parameters=[
                LaunchConfiguration('config_file'),
                {'runtime_profile': LaunchConfiguration('runtime_profile')},
            ],
        ),
        Node(
            condition=IfCondition(LaunchConfiguration('launch_rviz')),
            package='rviz2',
            executable='rviz2',
            name='uwb_follow_rviz',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            output='screen',
        ),
    ])
