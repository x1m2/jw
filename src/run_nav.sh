#!/bin/bash
set -e
set -o pipefail

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "${SCRIPT_PATH}")"
WORKSPACE_DIR="$(dirname "${SCRIPT_DIR}")"

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

cd "${WORKSPACE_DIR}"

source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}
export ROBOT_DATA_DIR=${ROBOT_DATA_DIR:-/robotfs/data/robot_data}
mkdir -p "${ROBOT_DATA_DIR}"

if [ -n "${WORKFLOW_LOG_DIR}" ]; then
    LOG_ROOT="${WORKFLOW_LOG_DIR}"
else
    LOG_ROOT="${ROBOT_DATA_DIR}/log/$(date +%Y%m%d_%H%M%S)"
fi

NAV_LOG_DIR="${LOG_ROOT}/nav"
RELOCALIZATION_DIR="${LOG_ROOT}/relocalization"
mkdir -p "${NAV_LOG_DIR}"

log_time() {
    date +%Y%m%d_%H%M%S
}

PROJECT_DIR="${WORKSPACE_DIR}/src"
GUI_SCRIPT="${PROJECT_DIR}/tools/user_interface/scripts/navigation_control_gui.py"

info "开始自主导航流程"
info "数据目录: ${ROBOT_DATA_DIR}"

# ==================================================
# 进程记录
# ==================================================
declare -a PIDS=()

# run_relocalization.sh 保留的后台节点 PID
LIVOX_PID=""
FASTLIO_PID=""
PCD2PGM_RELOC_PID=""
POINTCLOUD_TO_LASERSCAN_RELOC_PID=""
LOCALIZATION_PID=""

# run_nav.sh 自己启动的节点 PID
LIVOX_CONVERT_PID=""
POINTCLOUD_TO_LASERSCAN_PID=""
COSTMAP_PID=""
GLOBAL_PLANNER_PID=""
PATH_SMOOTHER_PID=""
PURE_PURSUIT_PID=""
DUAL_RADAR_EXTRACTOR_PID=""
TARGET_POINTS_VISUALIZER_PID=""
NAVIGATION_STATE_MACHINE_PID=""

CLEANING=0

# ==================================================
# 安全关闭进程组
# ==================================================
kill_process_group_safe() {
    local pid="$1"
    local name="$2"

    if [ -z "${pid}" ]; then
        return
    fi

    if ! kill -0 "${pid}" 2>/dev/null; then
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

    info "退出自主导航流程，清理后台节点..."

    for pid in "${PIDS[@]}"; do
        kill_process_group_safe "${pid}" "navigation 后台节点"
    done

    wait 2>/dev/null || true

    info "自主导航流程已结束"
    info "数据目录: ${ROBOT_DATA_DIR}"
}

trap cleanup EXIT INT TERM

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
    PIDS+=("${pid}")

    sleep "${sleep_sec}"

    if ! kill -0 "${pid}" 2>/dev/null; then
        err "${name} 启动失败"
        err "日志: ${log_file}"
        tail -n 80 "${log_file}" || true
        exit 1
    fi

    info "${name} 启动成功"
}

# ==================================================
# 读取 run_relocalization.sh 保留的 PID
# add_existing_pid_from_file PID变量名 节点名 PID文件 required/optional
# ==================================================
add_existing_pid_from_file() {
    local pid_var="$1"
    local name="$2"
    local pid_file="$3"
    local required="$4"

    if [ ! -f "${pid_file}" ]; then
        if [ "${required}" = "required" ]; then
            err "找不到 ${name} PID 文件: ${pid_file}"
            err "请先通过 GUI 完成重定位，再启动导航"
            exit 1
        fi
        return
    fi

    local pid
    pid="$(cat "${pid_file}")"

    if [ -z "${pid}" ] || ! kill -0 "${pid}" 2>/dev/null; then
        if [ "${required}" = "required" ]; then
            err "${name} 进程不存在或已退出: ${pid}"
            err "请重新执行重定位"
            exit 1
        fi
        return
    fi

    printf -v "${pid_var}" "%s" "${pid}"
    PIDS+=("${pid}")
    info "${name} 已运行"
}

# ==================================================
# 检查重定位阶段保留的节点
# localization_sc 必须存在；其他节点可选存在。
# ==================================================
add_existing_pid_from_file \
    LIVOX_PID \
    "livox_ros_driver2" \
    "${RELOCALIZATION_DIR}/livox_ros_driver2.pid" \
    "optional"

add_existing_pid_from_file \
    FASTLIO_PID \
    "fast_lio" \
    "${RELOCALIZATION_DIR}/fast_lio.pid" \
    "optional"

add_existing_pid_from_file \
    PCD2PGM_RELOC_PID \
    "pcd2pgm" \
    "${RELOCALIZATION_DIR}/pcd2pgm.pid" \
    "optional"

add_existing_pid_from_file \
    POINTCLOUD_TO_LASERSCAN_RELOC_PID \
    "pointcloud_to_laserscan" \
    "${RELOCALIZATION_DIR}/pointcloud_to_laserscan.pid" \
    "optional"

add_existing_pid_from_file \
    LOCALIZATION_PID \
    "localization_sc" \
    "${RELOCALIZATION_DIR}/localization_sc.pid" \
    "required"

# ==================================================
# 启动导航节点
# ==================================================

start_bg LIVOX_CONVERT_PID "livox_custom_to_pointcloud2" \
    "${NAV_LOG_DIR}/livox_custom_to_pointcloud2_$(log_time).log" 1 \
    python3 "${WORKSPACE_DIR}/src/scripts/livox_custom_to_pointcloud2.py"

start_bg POINTCLOUD_TO_LASERSCAN_PID "pointcloud_to_laserscan" \
    "${NAV_LOG_DIR}/pointcloud_to_laserscan_$(log_time).log" 1 \
    ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan.launch.py

start_bg COSTMAP_PID "costmap_2d" \
    "${NAV_LOG_DIR}/costmap_2d_$(log_time).log" 1 \
    ros2 launch costmap_2d costmap_2d.launch.py

start_bg GLOBAL_PLANNER_PID "global_path_planner" \
    "${NAV_LOG_DIR}/global_path_planner_$(log_time).log" 1 \
    ros2 launch global_path_planner global_path_planner.launch.py

start_bg PATH_SMOOTHER_PID "path_smoother" \
    "${NAV_LOG_DIR}/path_smoother_$(log_time).log" 1 \
    ros2 launch path_smoother path_smoother.launch.py

start_bg PURE_PURSUIT_PID "pure_pursuit" \
    "${NAV_LOG_DIR}/pure_pursuit_$(log_time).log" 1 \
    ros2 launch pure_pursuit_local_planner pure_pursuit_local_planner.launch.py

start_bg DUAL_RADAR_EXTRACTOR_PID "dual_radar_extractor" \
    "${NAV_LOG_DIR}/dual_radar_extractor_$(log_time).log" 3 \
    ros2 launch dual_radar_traversable_extractor dual_radar_extractor.launch.py

start_bg TARGET_POINTS_VISUALIZER_PID "target_points_visualizer" \
    "${NAV_LOG_DIR}/target_points_visualizer_$(log_time).log" 1 \
    ros2 launch topology_rviz_plugin target_points_visualizer.launch.py

start_bg NAVIGATION_STATE_MACHINE_PID "navigation_state_machine" \
    "${NAV_LOG_DIR}/navigation_state_machine_$(log_time).log" 1 \
    ros2 launch user_interface navigation_task_state_machine.launch.py

# ==================================================
# 导航保持运行：
# 手工启动时默认打开 GUI；
# systemd / 开机自启动时 NAV_HEADLESS=1，不打开 GUI。
# ==================================================
info "导航后台节点已启动"

if [ "${NAV_HEADLESS:-0}" = "1" ]; then
    info "检测到 NAV_HEADLESS=1：无界面自动导航模式，不启动导航控制 GUI"
    info "等待导航状态机运行；收到 SIGINT / SIGTERM 时再退出导航流程"

    # 保持 run_nav.sh 前台运行，避免脚本退出后 trap cleanup 清理所有节点。
    while true; do
        sleep 2

        # 状态机都不在了，说明导航核心已异常退出。
        if ! ros2 node list 2>/dev/null | grep -qx "/navigation_task_state_machine_node"; then
            warn "navigation_task_state_machine_node 已退出，结束导航流程"
            break
        fi
    done
else
    if [ ! -f "${GUI_SCRIPT}" ]; then
        err "找不到导航 GUI 脚本: ${GUI_SCRIPT}"
        exit 1
    fi

    chmod +x "${GUI_SCRIPT}"

    NAV_GUI_LOG_FILE="${NAV_LOG_DIR}/navigation_control_gui_$(log_time).log"

    info "启动导航控制 GUI，关闭 GUI 后自动结束导航流程"

    python3 "${GUI_SCRIPT}" > "${NAV_GUI_LOG_FILE}" 2>&1

    info "导航控制 GUI 已退出"
fi

cleanup