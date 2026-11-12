"""Start the planner with the real-robot parameter file."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory("scan_planner"),
        "config",
        "scan_planner_params.yaml",
    )

    return LaunchDescription([
        Node(
            package="scan_planner",
            executable="scan_planner_node",
            output="screen",
            parameters=[params_file],
        ),
    ])
