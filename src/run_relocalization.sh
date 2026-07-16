#!/bin/bash
set -e
set -o pipefail

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "${SCRIPT_PATH}")"
WORKSPACE_DIR="$(dirname "${SCRIPT_DIR}")"

info() { echo "[INFO] $*"; }
warn() { echo "[WARN] $*"; }
err()  { echo "[ERROR] $*"; }

readonly DATA_DIR="/robotfs/data/slam_data"

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

RELOCALIZATION_DIR="${LOG_ROOT}/relocalization"
mkdir -p "${RELOCALIZATION_DIR}"

log_time() {
    date +%Y%m%d_%H%M%S
}

# 关键日志统一放到 /robotfs/data/slam_data/log/本次时间/relocalization
FASTLIO_LOG_FILE="${RELOCALIZATION_DIR}/fast_lio_$(log_time).log"
LOCALIZATION_LOG_FILE="${RELOCALIZATION_DIR}/localization_sc_$(log_time).log"

# PID 文件保留给 run_nav.sh 读取，导航退出时统一清理
LIVOX_PID_FILE="${RELOCALIZATION_DIR}/livox_ros_driver2.pid"
FASTLIO_PID_FILE="${RELOCALIZATION_DIR}/fast_lio.pid"
PCD2PGM_PID_FILE="${RELOCALIZATION_DIR}/pcd2pgm.pid"
LOCALIZATION_PID_FILE="${RELOCALIZATION_DIR}/localization_sc.pid"

RELOCALIZATION_EXIT_CODE_FILE="${RELOCALIZATION_DIR}/relocalization_exit_code.txt"
RELOCALIZATION_SUCCESS_FILE="${RELOCALIZATION_DIR}/relocalization_success.txt"

rm -f "${RELOCALIZATION_EXIT_CODE_FILE}" "${RELOCALIZATION_SUCCESS_FILE}"

write_relocalization_result() {
    local exit_code="$1"
    local success_flag="$2"

    mkdir -p "${RELOCALIZATION_DIR}"
    echo "${exit_code}" > "${RELOCALIZATION_EXIT_CODE_FILE}"
    echo "${success_flag}" > "${RELOCALIZATION_SUCCESS_FILE}"

    info "写入重定位结果: exit_code=${exit_code}, success=${success_flag}"
    info "结果文件: ${RELOCALIZATION_EXIT_CODE_FILE}, ${RELOCALIZATION_SUCCESS_FILE}"
}

info "开始重定位流程"
info "数据目录: ${DATA_DIR}"
info "日志目录: ${RELOCALIZATION_DIR}"

PIDS=()
PID_FILES=()
CLEANING=0

LIVOX_PID=""
FASTLIO_PID=""
PCD2PGM_PID=""
LOCALIZATION_PID=""

add_pid() {
    local pid="$1"
    local pid_file="$2"

    PIDS+=("${pid}")
    PID_FILES+=("${pid_file}")
}

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

cleanup_failed() {
    if [ "${CLEANING}" -eq 1 ]; then
        return
    fi

    CLEANING=1
    trap - SIGINT SIGTERM EXIT

    warn "重定位失败或被中断，正在清理后台节点..."
    rm -f "${WAIT_SCRIPT:-}" 2>/dev/null || true

    write_relocalization_result 1 0 || true

    for pid in "${PIDS[@]}"; do
        kill_process_group_safe "${pid}" "relocalization 后台节点"
    done

    for pid_file in "${PID_FILES[@]}"; do
        rm -f "${pid_file}"
    done

    wait 2>/dev/null || true

    info "清理完成"
}

trap cleanup_failed SIGINT SIGTERM EXIT

start_bg() {
    local pid_var="$1"
    local name="$2"
    local log_file="$3"
    local pid_file="$4"
    local sleep_sec="$5"
    shift 5

    info "启动 ${name}..."

    setsid "$@" > "${log_file}" 2>&1 &
    local pid=$!

    printf -v "${pid_var}" "%s" "${pid}"
    echo "${pid}" > "${pid_file}"
    add_pid "${pid}" "${pid_file}"

    sleep "${sleep_sec}"

    if ! kill -0 "${pid}" 2>/dev/null; then
        err "${name} 启动失败"
        rm -f "${pid_file}"

        write_relocalization_result 1 0 || true

        if [ "${log_file}" != "/dev/null" ]; then
            err "日志: ${log_file}"
            tail -n 80 "${log_file}" || true
        else
            err "该节点日志已关闭，请单独执行启动命令排查"
        fi

        exit 1
    fi

    info "${name} 启动成功"
}

# ==================================================
# 启动重定位依赖节点
# ==================================================
start_bg LIVOX_PID "livox_ros_driver2" "/dev/null" "${LIVOX_PID_FILE}" 4 \
    ros2 launch livox_ros_driver2 msg_MID360s_launch.py

# start_bg FASTLIO_PID "fast_lio" "${FASTLIO_LOG_FILE}" "${FASTLIO_PID_FILE}" 3 \
#     ros2 launch fast_lio mapping.launch.py fast_lio_use_rviz:=false

start_bg PCD2PGM_PID "pcd2pgm" "/dev/null" "${PCD2PGM_PID_FILE}" 2 \
    ros2 launch pcd2pgm pcd2pgm_launch.py pgm_use_rviz:=false

# ==================================================
# 先启动等待节点，避免错过 /reloc_success=true
# 依赖 localization_sc 发布：
# /reloc_success std_msgs/msg/Bool true
# ==================================================
ROBOT_RUNTIME_DIR="$(dirname "${DATA_DIR}")"
WAIT_SCRIPT="${ROBOT_RUNTIME_DIR}/wait_reloc_success_$$.py"

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
            Bool,
            "/reloc_success",
            self.callback,
            10,
        )

    def callback(self, msg):
        if msg.data:
            self.success = True

    def timeout_reached(self):
        return (time.time() - self.start_time) > self.timeout_sec


def main():
    timeout_sec = 180.0

    if len(sys.argv) >= 2:
        timeout_sec = float(sys.argv[1])

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

info "提前启动 /reloc_success 等待节点..."
python3 "${WAIT_SCRIPT}" 180 > /dev/null 2>&1 &
WAIT_PID=$!

# 给等待节点一点时间完成 rclpy init 和订阅创建
sleep 0.5

# ==================================================
# 再启动 localization_sc
# ==================================================
# start_bg LOCALIZATION_PID "localization_sc" "${LOCALIZATION_LOG_FILE}" "${LOCALIZATION_PID_FILE}" 0 \
#     ros2 launch localization_sc run.launch.py localization_use_rviz:=true
start_bg LOCALIZATION_PID "fastlio_localization" "${LOCALIZATION_LOG_FILE}" "${LOCALIZATION_PID_FILE}" 0 \
    ros2 launch fastlio_localization run_launch.py

info "等待重定位成功，超时时间 180s..."

if wait "${WAIT_PID}"; then
    rm -f "${WAIT_SCRIPT}"

    write_relocalization_result 0 1

    info "重定位成功"
    info "后台节点保持运行，后续由 run_nav.sh 统一清理"

    # 成功后取消 trap，不清理后台节点
    # 后续 run_nav.sh 需要读取这些 pid 文件，并在导航退出时统一清理
    trap - SIGINT SIGTERM EXIT

    exit 0
else
    rm -f "${WAIT_SCRIPT}"

    write_relocalization_result 1 0

    err "重定位失败或超时"
    err "日志目录: ${RELOCALIZATION_DIR}"
    exit 1
fi
