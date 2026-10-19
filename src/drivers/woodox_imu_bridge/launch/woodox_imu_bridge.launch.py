# Copyright 2026 Bucar
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

import os


def generate_launch_description():
    package_share = get_package_share_directory('woodox_imu_bridge')
    parameters = os.path.join(package_share, 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='woodox_imu_bridge',
            executable='woodox_imu_bridge_node',
            name='woodox_imu_bridge',
            output='screen',
            parameters=[parameters],
        ),
    ])
