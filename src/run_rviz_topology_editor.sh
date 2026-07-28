#!/bin/bash
# ROS 2 Humble 的 setup.bash 会读取部分尚未定义的环境变量，不能在
# source 期间启用 nounset（set -u）。
set -eo pipefail

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "${SCRIPT_PATH}")"
WORKSPACE_DIR="$(dirname "${SCRIPT_DIR}")"

DATA_DIR="${1:-/robotfs/data/slam_data}"
PCD_FILE="${2:-}"
USE_RVIZ="${3:-true}"
TOPOLOGY_DIR="${DATA_DIR}/topology_map"
TASK_DIR="${DATA_DIR}/task"

OUTPUT_FILE="${TOPOLOGY_DIR}/generated_map.txt"
CONNECTED_FILE="${TOPOLOGY_DIR}/connected_topology_map.txt"
TASK_FILE="${TASK_DIR}/test.txt"

if [ -z "${PCD_FILE}" ]; then
    PCD_FILE="${DATA_DIR}/map/globalmap.pcd"
fi

if [ ! -f "/opt/ros/humble/setup.bash" ]; then
    echo "[ERROR] 找不到 /opt/ros/humble/setup.bash"
    exit 1
fi

if [ ! -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
    echo "[ERROR] 找不到工作空间环境: ${WORKSPACE_DIR}/install/setup.bash"
    echo "[ERROR] 请先编译 rviz_topology_editor"
    exit 1
fi

if [ ! -f "${PCD_FILE}" ]; then
    echo "[ERROR] 找不到可用于地图打点的PCD: ${PCD_FILE}"
    echo "[ERROR] 也可以通过第二个参数显式指定PCD路径"
    exit 1
fi

mkdir -p "${TOPOLOGY_DIR}" "${TASK_DIR}"

source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"

echo "[INFO] 启动 rviz_topology_editor"
echo "[INFO] PCD: ${PCD_FILE}"
echo "[INFO] 原始拓扑: ${OUTPUT_FILE}"
echo "[INFO] 连边拓扑: ${CONNECTED_FILE}"
echo "[INFO] 状态机任务: ${TASK_FILE}"
echo "[INFO] 打点完成后，在RViz左侧点击“保存并生成”"

exec ros2 launch rviz_topology_editor \
    rviz_topology_editor.launch.py \
    rviz:="${USE_RVIZ}" \
    pcd_file:="${PCD_FILE}" \
    output_file:="${OUTPUT_FILE}" \
    connected_topology_file:="${CONNECTED_FILE}" \
    task_file:="${TASK_FILE}"
