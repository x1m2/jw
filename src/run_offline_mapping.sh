#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly DATA_DIR="$(pwd -P)"
LOG_ROOT="${DATA_DIR}/log"
MAP_DIR="${DATA_DIR}/map"

info() { echo "[INFO] $*"; }
warn() { echo "[WARN] $*"; }
err()  { echo "[ERROR] $*"; }

if [ ! -d "${WORKSPACE_DIR}/src" ]; then
    err "找不到 src 目录: ${WORKSPACE_DIR}/src"
    exit 1
fi

if [ ! -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
    err "找不到工作空间环境: ${WORKSPACE_DIR}/install/setup.bash"
    err "请确认工作空间已经编译，并且路径正确"
    exit 1
fi

mkdir -p "${LOG_ROOT}" "${MAP_DIR}"

cd "${WORKSPACE_DIR}"

source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}

log_time() {
    date +%Y%m%d_%H%M%S
}

FASTLIO_LOG_FILE="${LOG_ROOT}/fast_lio_$(log_time).log"
PGO_LOG_FILE="${LOG_ROOT}/pgo_sc_$(log_time).log"

FASTLIO_PID=""
PGO_PID=""
CLEANING=0

kill_process_tree_safe() {
    local pid="$1"
    local name="$2"

    if [ -z "${pid}" ]; then
        return
    fi

    if ! kill -0 "${pid}" 2>/dev/null; then
        return
    fi

    info "停止 ${name}"

    local pgid
    pgid=$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d ' ' || true)

    if [ -n "${pgid}" ]; then
        kill -SIGINT "-${pgid}" 2>/dev/null || true
    else
        kill -SIGINT "${pid}" 2>/dev/null || true
    fi

    sleep 2

    local children
    children=$(pgrep -P "${pid}" || true)
    if [ -n "${children}" ]; then
        kill -SIGTERM ${children} 2>/dev/null || true
    fi

    if kill -0 "${pid}" 2>/dev/null; then
        if [ -n "${pgid}" ]; then
            kill -SIGTERM "-${pgid}" 2>/dev/null || true
        else
            kill -SIGTERM "${pid}" 2>/dev/null || true
        fi
    fi

    sleep 1

    if kill -0 "${pid}" 2>/dev/null; then
        warn "${name} 未正常退出，强制结束"
        if [ -n "${pgid}" ]; then
            kill -SIGKILL "-${pgid}" 2>/dev/null || true
        else
            kill -SIGKILL "${pid}" 2>/dev/null || true
        fi
    fi

    sleep 1
}

kill_all_related_processes() {
    info "清理 ROS2 建图相关残留进程..."

    local patterns=(
        "fastlio_mapping_node"
        "fast_lio"
        "imu_filter_madgwick"
        "imu_filter_madgwick_node"
        "mapping.launch.py"
        "pgo_sc_node"
        "run.launch.py"
        "rviz2"
        "rosbag2_player"
        "ros2 bag play"
        "component_container"
    )

    for p in "${patterns[@]}"; do
        pkill -SIGINT -f "${p}" 2>/dev/null || true
    done

    sleep 1

    for p in "${patterns[@]}"; do
        pkill -SIGTERM -f "${p}" 2>/dev/null || true
    done

    sleep 1

    for p in "${patterns[@]}"; do
        pkill -SIGKILL -f "${p}" 2>/dev/null || true
    done

    sleep 1

    info "残留进程清理完成"
}

cleanup() {
    if [ "${CLEANING}" -eq 1 ]; then
        return
    fi

    CLEANING=1
    trap - SIGINT SIGTERM EXIT

    echo ""
    info "正在关闭 FAST-LIO 和 PGO SC..."

    kill_process_tree_safe "${PGO_PID}" "PGO SC"
    kill_process_tree_safe "${FASTLIO_PID}" "FAST-LIO"

    wait 2>/dev/null || true

    kill_all_related_processes

    echo ""
    info "已退出"
    info "FAST-LIO 日志: ${FASTLIO_LOG_FILE}"
    info "PGO SC 日志: ${PGO_LOG_FILE}"

    exit 0
}

trap cleanup SIGINT SIGTERM EXIT

start_bg() {
    local pid_var="$1"
    local name="$2"
    local log_file="$3"
    local sleep_sec="$4"
    shift 4

    info "启动 ${name}..."

    setsid "$@" > "${log_file}" 2>&1 &
    local pid=$!

    printf -v "${pid_var}" "%s" "${pid}"

    sleep "${sleep_sec}"

    if ! kill -0 "${pid}" 2>/dev/null; then
        err "${name} 启动失败"
        err "日志: ${log_file}"
        tail -n 80 "${log_file}" || true
        exit 1
    fi

    info "${name} 启动成功，日志: ${log_file}"
}

# ==================================================
# 启动前清理残留
# ==================================================
info "启动前清理旧进程"
kill_all_related_processes

# ==================================================
# 启动 FAST-LIO 和 PGO
# ==================================================
info "开始启动离线建图节点"
info "工作空间: ${WORKSPACE_DIR}"
info "日志目录: ${LOG_ROOT}"
info "数据目录: ${DATA_DIR}"
info "地图目录: ${MAP_DIR}"

start_bg FASTLIO_PID "FAST-LIO" "${FASTLIO_LOG_FILE}" 3 \
    ros2 launch fast_lio mapping.launch.py fast_lio_use_rviz:=false use_sim_time:=true

start_bg PGO_PID "PGO SC" "${PGO_LOG_FILE}" 2 \
    ros2 launch pgo_sc run.launch.py pgo_use_rviz:=true use_sim_time:=true \
    "data_dir:=${DATA_DIR}"

info "建议检查 publisher 数量："
echo "ros2 topic info /Odometry -v"
echo "ros2 topic info /cloud_registered -v"
echo ""
info "按 Ctrl+C 退出"
echo ""

while true; do
    sleep 1

    if [ -n "${FASTLIO_PID}" ] && ! kill -0 "${FASTLIO_PID}" 2>/dev/null; then
        err "FAST-LIO 进程异常退出"
        err "日志: ${FASTLIO_LOG_FILE}"
        tail -n 80 "${FASTLIO_LOG_FILE}" || true
        cleanup
    fi

    if [ -n "${PGO_PID}" ] && ! kill -0 "${PGO_PID}" 2>/dev/null; then
        err "PGO SC 进程异常退出"
        err "日志: ${PGO_LOG_FILE}"
        tail -n 80 "${PGO_LOG_FILE}" || true
        cleanup
    fi
done
