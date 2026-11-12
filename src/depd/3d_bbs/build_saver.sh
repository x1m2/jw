#!/usr/bin/env bash
set -euo pipefail

root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
build_dir="${BBS_BUILD_DIR:-$root/build}"
jobs="${BUILD_JOBS:-2}"

if [[ ! "$jobs" =~ ^[1-9][0-9]*$ ]]; then
  echo "BUILD_JOBS must be a positive integer" >&2
  exit 1
fi

# The map converter uses CPU; its maps also serve the OpenCL backend.
cmake -S "$root" -B "$build_dir" \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_OPENCL=ON \
  -DBUILD_BBS_TEST_TOOLS=ON "$@"
cmake --build "$build_dir" --target voxelmaps_saver opencl_bbs3d --parallel "$jobs"

echo "voxelmaps_saver: $build_dir/voxelmaps_saver"
echo "Usage: $build_dir/voxelmaps_saver PCD_DIRECTORY MIN_LEVEL_RES MAX_LEVEL"
