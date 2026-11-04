#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
set +u
source "${DUAL_AIRY_ROS_SETUP:-/opt/ros/humble/setup.bash}"
set -u
# 在当前 CPU 架构编译 Release；包内不带电脑 build/，机器狗 ARM64 应本地编译。
# Python 路径仅供 ROS/ament 构建工具使用，两个运行节点都是 C++。
cmake -S "$ROOT" -B "$ROOT/build" -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3 -DBUILD_TESTING=ON
cmake --build "$ROOT/build" --parallel "${DUAL_AIRY_BUILD_JOBS:-2}"
# 验证矩阵方向、字段、窗口和时间检查逻辑；不替代实机外参精度验收。
ctest --test-dir "$ROOT/build" --output-on-failure
"$ROOT/build/dual_airy_fusion" --check "$ROOT/config/extrinsics.yaml"
