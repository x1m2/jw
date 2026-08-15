#!/bin/bash
set -e
set -o pipefail

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "${SCRIPT_PATH}")"
WORKSPACE_DIR="$(dirname "${SCRIPT_DIR}")"

info() { echo "[INFO] $*"; }
warn() { echo "[WARN] $*"; }
err()  { echo "[ERROR] $*"; }

# GUI 会传入 MAP_ID；独立启动脚本时统一使用 map_0。
readonly MAP_ID_INPUT="${MAP_ID:-0}"
if ! [[ "${MAP_ID_INPUT}" =~ ^[0-9]+$ ]] ||
   [ "${#MAP_ID_INPUT}" -gt 10 ] ||
   { [ "${#MAP_ID_INPUT}" -eq 10 ] && [[ "${MAP_ID_INPUT}" > "4294967295" ]]; }; then
    err "MAP_ID 必须是 uint32：${MAP_ID_INPUT}"
    exit 1
fi
readonly MAP_ID="$((10#${MAP_ID_INPUT}))"

# DATA_DIR 只用于兼容 GUI 传入的 map_<id> 完整目录；所有子 launch 都继续
# 使用 DATA_ROOT + MAP_ID 自行构造 map_<id>，避免不同节点落到不同地图。
if [ -n "${DATA_DIR:-}" ]; then
    DATA_ROOT="$(dirname "${DATA_DIR}")"
else
    DATA_ROOT="${SLAM_DATA_ROOT:-/robotfs/data/slam_data}"
fi
readonly DATA_ROOT
readonly DATA_DIR="${DATA_ROOT}/map_${MAP_ID}"

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
mkdir -p "${DATA_DIR}"

if [ -n "${WORKFLOW_LOG_DIR}" ]; then
    LOG_ROOT="${WORKFLOW_LOG_DIR}"
else
    LOG_ROOT="${DATA_DIR}/log/$(date +%Y%m%d_%H%M%S)"
fi

NAV_LOG_DIR="${LOG_ROOT}/nav"
RELOCALIZATION_DIR="${LOG_ROOT}/relocalization"
mkdir -p "${NAV_LOG_DIR}" "${RELOCALIZATION_DIR}"

RELOCALIZATION_EXIT_CODE_FILE="${RELOCALIZATION_DIR}/relocalization_exit_code.txt"
RELOCALIZATION_SUCCESS_FILE="${RELOCALIZATION_DIR}/relocalization_success.txt"
NAVIGATION_CONFIRM_FILE="${RELOCALIZATION_DIR}/navigation_confirm.txt"
SLAM_MANAGER_DIR="$(dirname "${DATA_DIR}")"
WAIT_SCRIPT="${SLAM_MANAGER_DIR}/wait_reloc_success_$$.py"

rm -f \
    "${RELOCALIZATION_EXIT_CODE_FILE}" \
    "${RELOCALIZATION_SUCCESS_FILE}" \
    "${NAVIGATION_CONFIRM_FILE}" \
    "${WAIT_SCRIPT}"

log_time() {
    date +%Y%m%d_%H%M%S
}

info "开始重定位与自主导航流程"
info "地图ID: ${MAP_ID}"
info "数据目录: ${DATA_DIR}"

# ==================================================
# 进程记录
# ==================================================
declare -a PIDS=()

# 重定位阶段节点 PID
LIVOX_PID=""
LOCALIZATION_PID=""
WAIT_PID=""

# 导航阶段节点 PID
COSTMAP_PID=""
GLOBAL_PLANNER_PID=""
PATH_SMOOTHER_PID=""
PURE_PURSUIT_PID=""
OBSTACLE_STOP_PID=""
# DUAL_RADAR_EXTRACTOR_PID=""
TARGET_POINTS_VISUALIZER_PID=""
NAVIGATION_STATE_MACHINE_PID=""

CLEANING=0

write_relocalization_result() {
    local exit_code="$1"
    local success_flag="$2"

    echo "${exit_code}" > "${RELOCALIZATION_EXIT_CODE_FILE}"
    echo "${success_flag}" > "${RELOCALIZATION_SUCCESS_FILE}"
    info "写入重定位结果: exit_code=${exit_code}, success=${success_flag}"
}

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
    rm -f "${WAIT_SCRIPT}" 2>/dev/null || true

    if [ -n "${WAIT_PID}" ] && kill -0 "${WAIT_PID}" 2>/dev/null; then
        kill -SIGTERM "${WAIT_PID}" 2>/dev/null || true
    fi

    for pid in "${PIDS[@]}"; do
        kill_process_group_safe "${pid}" "navigation 后台节点"
    done

    wait 2>/dev/null || true

    info "自主导航流程已结束"
    info "数据目录: ${DATA_DIR}"
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
# 重定位阶段
# ==================================================
info "启动重定位依赖节点..."

start_bg LIVOX_PID "livox_ros_driver2" "/dev/null" 4 \
    ros2 launch livox_ros_driver2 msg_MID360s_launch.py

# 在 localization 启动前先订阅，避免错过 /reloc_success=true。
cat > "${WAIT_SCRIPT}" <<'PY'
#!/usr/bin/env python3
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class WaitRelocalizationSuccess(Node):
    def __init__(self, timeout_sec):
        super().__init__("wait_reloc_success")
        self.timeout_sec = float(timeout_sec)
        self.start_time = time.time()
        self.success = False
        self.sub = self.create_subscription(
            Bool, "/reloc_success", self.callback, 10
        )

    def callback(self, msg):
        if msg.data:
            self.success = True

    def timeout_reached(self):
        return (time.time() - self.start_time) > self.timeout_sec


def main():
    timeout_sec = float(sys.argv[1]) if len(sys.argv) >= 2 else 180.0
    rclpy.init()
    node = WaitRelocalizationSuccess(timeout_sec)

    try:
        while rclpy.ok() and not node.success:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.timeout_reached():
                break
    except KeyboardInterrupt:
        pass

    success = node.success
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
PY

chmod +x "${WAIT_SCRIPT}"
python3 "${WAIT_SCRIPT}" 180 > /dev/null 2>&1 &
WAIT_PID=$!
sleep 0.5

start_bg LOCALIZATION_PID "fastlio_localization" \
    "${RELOCALIZATION_DIR}/fastlio_localization_$(log_time).log" 0 \
    ros2 launch fastlio_localization run_launch.py \
    data_dir:="${DATA_ROOT}" map_id:="${MAP_ID}"

info "等待重定位成功，超时时间 180s..."
if wait "${WAIT_PID}"; then
    WAIT_PID=""
    rm -f "${WAIT_SCRIPT}"
    write_relocalization_result 0 1
    info "重定位成功，等待 GUI 确认当前位置和朝向"
else
    WAIT_PID=""
    rm -f "${WAIT_SCRIPT}"
    write_relocalization_result 1 0
    err "重定位失败或超时"
    err "日志目录: ${RELOCALIZATION_DIR}"
    exit 1
fi

# GUI 读取上面的结果后弹出人工确认框，再写入 navigation_confirm.txt。
CONFIRM_TIMEOUT_SEC=300
CONFIRM_WAITED_SEC=0
while [ ! -f "${NAVIGATION_CONFIRM_FILE}" ]; do
    sleep 1
    CONFIRM_WAITED_SEC=$((CONFIRM_WAITED_SEC + 1))
    if [ "${CONFIRM_WAITED_SEC}" -ge "${CONFIRM_TIMEOUT_SEC}" ]; then
        err "等待 GUI 确认重定位结果超时"
        exit 1
    fi
done

CONFIRM_VALUE="$(tr -d '[:space:]' < "${NAVIGATION_CONFIRM_FILE}")"
if [ "${CONFIRM_VALUE}" != "1" ]; then
    warn "用户未确认重定位结果，取消启动导航"
    exit 1
fi

info "用户已确认重定位结果，开始启动导航节点"

# ==================================================
# 启动导航节点
# ==================================================

start_bg OBSTACLE_STOP_PID "obstacle_stop" \
    "${NAV_LOG_DIR}/obstacle_stop_$(log_time).log" 1 \
    ros2 launch obstacle_stop obstacle_stop.launch.py

start_bg COSTMAP_PID "costmap_2d" \
    "${NAV_LOG_DIR}/costmap_2d_$(log_time).log" 1 \
    ros2 launch costmap_2d costmap_2d.launch.py

start_bg GLOBAL_PLANNER_PID "global_path_planner" \
    "${NAV_LOG_DIR}/global_path_planner_$(log_time).log" 1 \
    ros2 launch global_path_planner global_path_planner.launch.py \
    data_dir:="${DATA_ROOT}" map_id:="${MAP_ID}"

start_bg PATH_SMOOTHER_PID "path_smoother" \
    "${NAV_LOG_DIR}/path_smoother_$(log_time).log" 1 \
    ros2 launch path_smoother path_smoother.launch.py

start_bg PURE_PURSUIT_PID "pure_pursuit" \
    "${NAV_LOG_DIR}/pure_pursuit_$(log_time).log" 1 \
    ros2 launch pure_pursuit_local_planner pure_pursuit_local_planner.launch.py

# start_bg DUAL_RADAR_EXTRACTOR_PID "dual_radar_extractor" \
#     "${NAV_LOG_DIR}/dual_radar_extractor_$(log_time).log" 3 \
#     ros2 launch dual_radar_traversable_extractor dual_radar_extractor.launch.py

start_bg TARGET_POINTS_VISUALIZER_PID "target_points_visualizer" \
    "${NAV_LOG_DIR}/target_points_visualizer_$(log_time).log" 1 \
    ros2 launch topology_rviz_plugin target_points_visualizer.launch.py \
    data_dir:="${DATA_ROOT}" map_id:="${MAP_ID}"

start_bg NAVIGATION_STATE_MACHINE_PID "navigation_state_machine" \
    "${NAV_LOG_DIR}/navigation_state_machine_$(log_time).log" 1 \
    ros2 launch user_interface navigation_task_state_machine.launch.py \
    data_dir:="${DATA_ROOT}" map_id:="${MAP_ID}"

# ==================================================
# 导航保持运行：
# run_nav.sh 统一负责重定位、人工确认后的导航启动和后台节点清理。
# 该脚本保留旧的独立导航状态机流程；正式APP流程由SlamManager Action控制。
# ==================================================
info "导航后台节点已启动"
info "请在主 GUI 的导航任务控制区选择任务、急停、回家、停止或继续任务"
info "等待导航状态机运行；收到 SIGINT / SIGTERM 时再退出导航流程"

# 保持 run_nav.sh 前台运行，避免脚本退出后 trap cleanup 清理所有节点。
# ros2 node list 依赖 ROS graph/daemon，节点刚启动或 DDS 抖动时可能短暂查不到。
# 因此只用启动时记录的 PID 判断状态机是否真正退出；ROS graph 仅用于提示。
NAV_STATE_MACHINE_MISSING_COUNT=0
while true; do
    sleep 2

    if ! kill -0 "${NAVIGATION_STATE_MACHINE_PID}" 2>/dev/null; then
        warn "navigation_state_machine 进程已退出，结束导航流程"
        break
    fi

    if ros2 node list 2>/dev/null | grep -qx "/navigation_task_state_machine_node"; then
        NAV_STATE_MACHINE_MISSING_COUNT=0
        continue
    fi

    NAV_STATE_MACHINE_MISSING_COUNT=$((NAV_STATE_MACHINE_MISSING_COUNT + 1))

    if [ "${NAV_STATE_MACHINE_MISSING_COUNT}" -ge 5 ]; then
        warn "navigation_task_state_machine_node 连续多次不可见，但进程仍在运行，继续等待"
        NAV_STATE_MACHINE_MISSING_COUNT=0
    fi
done

cleanup
