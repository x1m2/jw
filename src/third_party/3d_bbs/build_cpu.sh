#!/usr/bin/env bash
set -euo pipefail

root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace="$(cd -- "$root/../../.." && pwd)"
build_dir="${BBS_BUILD_DIR:-$root/build_cpu}"
install_dir="${BBS_INSTALL_DIR:-$workspace/install/bbs3d}"
jobs="${BUILD_JOBS:-2}"
if [[ ! "$jobs" =~ ^[1-9][0-9]*$ ]]; then
  echo "BUILD_JOBS must be a positive integer" >&2
  exit 1
fi

cmake -S "$root" -B "$build_dir" "$@" \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_OPENCL=OFF \
  -DBUILD_BBS_TEST_TOOLS=ON -DBUILD_BBS_OPENCL_TEST=OFF \
  -DCMAKE_INSTALL_PREFIX="$install_dir"
cmake --build "$build_dir" --target voxelmaps_saver --parallel "$jobs"
cmake --install "$build_dir"
echo "CPU library: $install_dir/lib/libcpu_bbs3d.so"
echo "Map converter: $install_dir/bin/voxelmaps_saver"
echo "Localization: colcon build --packages-select fastlio_localization --cmake-args -DENABLE_OPENCL_BBS=OFF -DBBS3D_ROOT=$install_dir"
