#!/usr/bin/env bash
set -e

cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
source ../../install/setup.bash
exec ros2 run uwb_aoa_pkg libAoa_robot_example /dev/ttyFT232
