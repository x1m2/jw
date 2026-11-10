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

# DDS 配置路径由 systemd service 的 Environment 提供。
if [[ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]]; then
  echo "未设置 FASTRTPS_DEFAULT_PROFILES_FILE，请检查 service 环境配置" >&2
  exit 1
fi

if [[ ! -r "$FASTRTPS_DEFAULT_PROFILES_FILE" ]]; then
  echo "无法读取 DDS 配置：$FASTRTPS_DEFAULT_PROFILES_FILE" >&2
  exit 1
fi

echo "UWB 串口：$serial_device"
echo "DDS 配置：$FASTRTPS_DEFAULT_PROFILES_FILE"

exec ros2 run uwb_aoa_pkg libAoa_robot_example "$serial_device"