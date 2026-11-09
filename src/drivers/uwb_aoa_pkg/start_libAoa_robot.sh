#!/usr/bin/env bash
set -e

serial_device=""
for device in /dev/ttyUSB0 /dev/ttyUSB1; do
  if [[ -c "$device" ]]; then
    serial_device="$device"
    break
  fi
done
if [[ -z "$serial_device" ]]; then
  echo "未找到 UWB 串口：/dev/ttyUSB0 或 /dev/ttyUSB1" >&2
  exit 1
fi

cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
source ../../install/setup.bash
exec ros2 run uwb_aoa_pkg libAoa_robot_example "$serial_device"
