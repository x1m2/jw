"""Start the planner with the real-robot parameter file."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory("pure_pursuit"),
        "config",
        "pure_pursuit_params.yaml",
    )

    return LaunchDescription([
        Node(
            package="pure_pursuit",
            executable="pure_pursuit_node",
            output="screen",
            parameters=[params_file],
        ),
    ])
