#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
LOG_ROOT="${WORKSPACE_DIR}/log"

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

mkdir -p "${LOG_ROOT}"

cd "${WORKSPACE_DIR}"

source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"

# 指定当前要使用的地图根目录。
# result_bag 放在 src 同级目录，即 ${WORKSPACE_DIR}/result_bag
export ROBOT_DATA_DIR="${WORKSPACE_DIR}"

MAP_BAG_DIR="${ROBOT_DATA_DIR}/result_bag"

if [ ! -d "${MAP_BAG_DIR}" ]; then
    err "找不到已有地图目录: ${MAP_BAG_DIR}"
    err "请确认目录结构为："
    err "${ROBOT_DATA_DIR}/result_bag/result_bag_0.db3"
    exit 1
fi

if ! find "${MAP_BAG_DIR}" -maxdepth 1 -name "*.db3" -type f | grep -q .; then
    err "地图目录存在，但未找到 .db3 文件: ${MAP_BAG_DIR}"
    exit 1
fi

info "ROBOT_DATA_DIR: ${ROBOT_DATA_DIR}"
info "已有地图目录: ${MAP_BAG_DIR}"

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}

log_time() {
    date +%Y%m%d_%H%M%S
}

FASTLIO_LOG_FILE="${LOG_ROOT}/fast_lio_$(log_time).log"
LOCALIZATION_LOG_FILE="${LOG_ROOT}/localization_sc_$(log_time).log"

FASTLIO_PID=""
LOCALIZATION_PID=""
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
    info "清理 FAST-LIO / PGO / localization / rosbag / RViz 相关残留进程..."

    local patterns=(
        "laser_mapping"
        "fast_lio"
        "mapping.launch.py"

        "pgo_sc"
        "pgo_sc_node"

        "localization_sc"
        "localization_sc_node"
        "relocalization"
        "relocalization_node"

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
    info "正在关闭 FAST-LIO 和 localization_sc..."

    kill_process_tree_safe "${LOCALIZATION_PID}" "LOCALIZATION SC"
    kill_process_tree_safe "${FASTLIO_PID}" "FAST-LIO"

    wait 2>/dev/null || true

    kill_all_related_processes

    echo ""
    info "已退出"
    info "FAST-LIO 日志: ${FASTLIO_LOG_FILE}"
    info "LOCALIZATION SC 日志: ${LOCALIZATION_LOG_FILE}"

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
# 启动 FAST-LIO 和 localization_sc
# ==================================================
info "开始启动离线重定位节点"
info "工作空间: ${WORKSPACE_DIR}"
info "日志目录: ${LOG_ROOT}"

start_bg FASTLIO_PID "FAST-LIO" "${FASTLIO_LOG_FILE}" 3 \
    ros2 launch fast_lio mapping.launch.py fast_lio_use_rviz:=false use_sim_time:=true

start_bg LOCALIZATION_PID "LOCALIZATION SC" "${LOCALIZATION_LOG_FILE}" 2 \
    ros2 launch localization_sc run.launch.py localization_use_rviz:=true use_sim_time:=true

info "建议检查 topic："
echo "ros2 topic info /Odometry -v"
echo "ros2 topic info /cloud_registered -v"
echo "ros2 topic info /slam -v"
echo "ros2 topic info /corrected_current_pcd -v"
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

    if [ -n "${LOCALIZATION_PID}" ] && ! kill -0 "${LOCALIZATION_PID}" 2>/dev/null; then
        err "LOCALIZATION SC 进程异常退出"
        err "日志: ${LOCALIZATION_LOG_FILE}"
        tail -n 80 "${LOCALIZATION_LOG_FILE}" || true
        cleanup
    fi
done
