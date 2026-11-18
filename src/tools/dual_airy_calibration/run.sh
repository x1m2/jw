#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
# ROS is only needed for recording/reading a bag. Prepared datasets work without it.
if [[ -f "${DUAL_AIRY_ROS_SETUP:-/opt/ros/humble/setup.bash}" ]]; then
  set +u
  source "${DUAL_AIRY_ROS_SETUP:-/opt/ros/humble/setup.bash}"
  set -u
fi
export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}"
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-1}"
exec /usr/bin/python3 "$ROOT/tool.py" "$@"
