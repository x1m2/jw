#!/bin/bash
set -e
set -o pipefail

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "${SCRIPT_PATH}")"
WORKSPACE_DIR="$(dirname "${SCRIPT_DIR}")"

info() { echo "[INFO] $*"; }
warn() { echo "[WARN] $*"; }
err()  { echo "[ERROR] $*"; }

readonly MAP_ID_INPUT="${MAP_ID:-0}"
if ! [[ "${MAP_ID_INPUT}" =~ ^[0-9]+$ ]] ||
   [ "${#MAP_ID_INPUT}" -gt 10 ] ||
   { [ "${#MAP_ID_INPUT}" -eq 10 ] && [[ "${MAP_ID_INPUT}" > "4294967295" ]]; }; then
    err "MAP_ID 必须是 uint32：${MAP_ID_INPUT}"
    exit 1
fi
readonly MAP_ID="$((10#${MAP_ID_INPUT}))"
readonly DATA_ROOT="${SLAM_DATA_ROOT:-/robotfs/data/slam_data}"
readonly DATA_DIR="${DATA_DIR:-${DATA_ROOT}/map_${MAP_ID}}"

if [ ! -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
    err "找不到工作空间环境: ${WORKSPACE_DIR}/install/setup.bash"
    err "请先编译 FAST-LIO、UWB 和导航依赖包"
    exit 1
fi

cd "${WORKSPACE_DIR}"
source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}
readonly UWB_USE_RVIZ="${UWB_USE_RVIZ:-true}"

if [ -n "${WORKFLOW_LOG_DIR:-}" ]; then
    LOG_ROOT="${WORKFLOW_LOG_DIR}"
else
    LOG_ROOT="${DATA_DIR}/log/$(date +%Y%m%d_%H%M%S)"
fi
readonly FOLLOW_LOG_DIR="${LOG_ROOT}/uwb_follow"
readonly RUN_DIR="${DATA_DIR}/run"
readonly PID_FILE="${RUN_DIR}/uwb_follow.pid"
mkdir -p "${DATA_DIR}/map" "${DATA_DIR}/task" "${FOLLOW_LOG_DIR}" "${RUN_DIR}"

if [ -f "${PID_FILE}" ]; then
    EXISTING_PID="$(tr -cd '0-9' < "${PID_FILE}")"
    if [ -n "${EXISTING_PID}" ] && kill -0 "${EXISTING_PID}" 2>/dev/null; then
        err "UWB 跟随已经运行，PID=${EXISTING_PID}"
        exit 1
    fi
fi
echo "$$" > "${PID_FILE}"

log_time() { date +%Y%m%d_%H%M%S; }

declare -a PIDS=()
CLEANING=0

kill_process_group_safe() {
    local pid="$1"
    local name="$2"
    if [ -z "${pid}" ] || ! kill -0 "${pid}" 2>/dev/null; then
        return
    fi
    info "停止 ${name}"
    kill -SIGINT "-${pid}" 2>/dev/null || kill -SIGINT "${pid}" 2>/dev/null || true
    sleep 2
    if kill -0 "${pid}" 2>/dev/null; then
        kill -SIGTERM "-${pid}" 2>/dev/null || kill -SIGTERM "${pid}" 2>/dev/null || true
    fi
    sleep 1
    if kill -0 "${pid}" 2>/dev/null; then
        warn "${name} 未退出，强制结束"
        kill -SIGKILL "-${pid}" 2>/dev/null || kill -SIGKILL "${pid}" 2>/dev/null || true
    fi
}

cleanup() {
    if [ "${CLEANING}" -eq 1 ]; then
        return
    fi
    CLEANING=1
    trap - EXIT INT TERM
    rm -f "${PID_FILE}" 2>/dev/null || true
    info "退出 UWB 跟随，按反向顺序清理后台节点..."
    local index
    for ((index=${#PIDS[@]}-1; index>=0; index--)); do
        kill_process_group_safe "${PIDS[index]}" "UWB 跟随后台节点"
    done
    wait 2>/dev/null || true
    info "UWB 跟随流程已结束"
}
trap cleanup EXIT
trap 'cleanup; exit 0' INT TERM

start_bg() {
    local name="$1"
    local log_file="$2"
    local wait_sec="$3"
    shift 3
    info "启动 ${name}..."
    setsid "$@" > "${log_file}" 2>&1 &
    local pid=$!
    PIDS+=("${pid}")
    sleep "${wait_sec}"
    if ! kill -0 "${pid}" 2>/dev/null; then
        err "${name} 启动失败，日志: ${log_file}"
        tail -n 80 "${log_file}" || true
        exit 1
    fi
    info "${name} 启动成功，PID=${pid}"
}

info "开始 FAST-LIO2 里程计坐标系下的 UWB 跟随"
info "运行数据目录: ${DATA_DIR}"
info "跟踪坐标系: odom"
info "UWB 跟随 RViz: ${UWB_USE_RVIZ}"

start_bg "Livox ROS Driver2" "/dev/null" 4 \
    ros2 launch livox_ros_driver2 msg_MID360s_launch.py

start_bg "FAST-LIO2" "${FOLLOW_LOG_DIR}/fast_lio_$(log_time).log" 3 \
    ros2 launch fast_lio mapping.launch.py fast_lio_use_rviz:=false

start_bg "Obstacle Stop" "${FOLLOW_LOG_DIR}/obstacle_stop_$(log_time).log" 1 \
    ros2 launch obstacle_stop obstacle_stop.launch.py

start_bg "Local Costmap" "${FOLLOW_LOG_DIR}/costmap_2d_$(log_time).log" 1 \
    ros2 launch costmap_2d costmap_2d.launch.py global_frame:=odom

start_bg "Path Smoother" "${FOLLOW_LOG_DIR}/path_smoother_$(log_time).log" 1 \
    ros2 launch path_smoother path_smoother.launch.py

start_bg "Pure Pursuit" "${FOLLOW_LOG_DIR}/pure_pursuit_$(log_time).log" 1 \
    ros2 launch pure_pursuit_local_planner pure_pursuit_local_planner.launch.py \
    global_frame:=odom

start_bg "UWB Follow Planner" "${FOLLOW_LOG_DIR}/uwb_follow_planner_$(log_time).log" 1 \
    ros2 launch uwb_follow_planner uwb_follow_planner.launch.py \
    launch_rviz:="${UWB_USE_RVIZ}"

readonly UWB_TOPIC="/libAoa_robot_publisher"
readonly UWB_TOPIC_TYPE="uwb_aoa_pkg/msg/LibAoaRobotMsg"
if timeout 3s ros2 topic list 2>/dev/null | grep -qx "${UWB_TOPIC}"; then
    ACTUAL_UWB_TYPE="$(timeout 3s ros2 topic type "${UWB_TOPIC}" 2>/dev/null || true)"
    if [ "${ACTUAL_UWB_TYPE}" != "${UWB_TOPIC_TYPE}" ]; then
        err "${UWB_TOPIC} 类型错误: ${ACTUAL_UWB_TYPE:-未知}"
        err "期望类型: ${UWB_TOPIC_TYPE}"
        exit 1
    fi
    info "UWB 话题已连接: ${UWB_TOPIC} [${UWB_TOPIC_TYPE}]"
else
    warn "当前未发现 /libAoa_robot_publisher；UWB硬件发布程序可稍后接入"
    info "等待 ${UWB_TOPIC} [${UWB_TOPIC_TYPE}]；按 Ctrl+C 停止跟随"
fi

info "UWB 跟随后台节点已启动"

while true; do
    sleep 2
    for pid in "${PIDS[@]}"; do
        if ! kill -0 "${pid}" 2>/dev/null; then
            err "检测到后台节点异常退出，结束 UWB 跟随"
            exit 1
        fi
    done
done
