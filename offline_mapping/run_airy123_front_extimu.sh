#!/usr/bin/env bash
set -euo pipefail

offline_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-93}"
exec bash "$offline_dir/run_offline_mapping.sh" "$@" --lidar airy --body-imu --bag /data/datasets/data/Airy/123
