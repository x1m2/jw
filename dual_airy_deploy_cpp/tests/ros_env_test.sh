#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd -P)
source "$ROOT/ros_env.sh"
fixture=$(mktemp -d)
trap 'rm -rf "$fixture"' EXIT
export DUAL_AIRY_PROC_ROOT="$fixture"
mkdir -p "$fixture/123"
ln -s /fake/install/rslidar_sdk/rslidar_sdk_node "$fixture/123/exe"
printf 'ROS_DOMAIN_ID=194\0ROS_LOCALHOST_ONLY=0\0RMW_IMPLEMENTATION=rmw_fastrtps_cpp\0UNRELATED_SECRET=do_not_print\0' > "$fixture/123/environ"
(
  export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1 FASTRTPS_DEFAULT_PROFILES_FILE=/wrong.xml
  dual_airy_configure_ros_env auto
  [[ "$ROS_DOMAIN_ID" == 194 && "$ROS_LOCALHOST_ONLY" == 0 ]]
  [[ "$FASTRTPS_DEFAULT_PROFILES_FILE" == "$ROOT/config/fastdds_fusion.xml" ]]
)
printf 'ROS_LOCALHOST_ONLY=1\0' > "$fixture/123/environ"
if (dual_airy_configure_ros_env auto); then echo 'FAIL: local-only driver must require restart' >&2; exit 1; fi
printf 'ROS_DOMAIN_ID=194\0' > "$fixture/123/environ"
cp -a "$fixture/123" "$fixture/124"
if (dual_airy_configure_ros_env auto); then echo 'FAIL: duplicate drivers must be rejected' >&2; exit 1; fi
rm -rf "$fixture/123" "$fixture/124"
(
  export ROS_DOMAIN_ID=194 ROS_LOCALHOST_ONLY=1
  dual_airy_configure_ros_env auto
  [[ "$ROS_DOMAIN_ID" == 194 && "$ROS_LOCALHOST_ONLY" == 0 ]]
)
(
  export ROS_DOMAIN_ID=194 ROS_LOCALHOST_ONLY=1
  dual_airy_configure_ros_env inherit
  [[ "$ROS_LOCALHOST_ONLY" == 0 ]]
)
if (export ROS_DOMAIN_ID=999; dual_airy_configure_ros_env inherit); then exit 1; fi
echo 'PASS: driver environment, isolation cancellation, duplicate rejection, invalid domain'
