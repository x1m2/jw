#!/bin/bash
set -e
set -o pipefail

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "${SCRIPT_PATH}")"
WORKSPACE_DIR="$(dirname "${SCRIPT_DIR}")"

info() { echo "[INFO] $*"; }
warn() { echo "[WARN] $*"; }
err()  { echo "[ERROR] $*"; }

source "${SCRIPT_DIR}/load_robot_env.sh"

if [ ! -d "${WORKSPACE_DIR}/src" ]; then
    err "找不到 src 目录: ${WORKSPACE_DIR}/src"
    exit 1
fi

if [ ! -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
    err "找不到工作空间环境: ${WORKSPACE_DIR}/install/setup.bash"
    exit 1
fi

cd "${WORKSPACE_DIR}"

source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}

RUN_KILL_SCRIPT="${SCRIPT_DIR}/run_kill_node.sh"
RUN_RELOCALIZATION_SCRIPT="${SCRIPT_DIR}/run_relocalization.sh"
RUN_NAV_SCRIPT="${SCRIPT_DIR}/run_nav.sh"

AUTO_NAV_ATTEMPTS=${AUTO_NAV_ATTEMPTS:-2}
AUTO_NAV_RETRY_DELAY=${AUTO_NAV_RETRY_DELAY:-10}
AUTO_NAV_TASK_FOLDER=${AUTO_NAV_TASK_FOLDER:-"${ROBOT_DATA_DIR}/task"}
AUTO_NAV_LOOP=${AUTO_NAV_LOOP:-0}
AUTO_NAV_SIGNAL=${AUTO_NAV_SIGNAL:-}
AUTO_NAV_STARTUP_WAIT=${AUTO_NAV_STARTUP_WAIT:-60}

# ==================================================
# 开机启动前清理历史运行日志
# 只删除：${ROBOT_DATA_DIR}/log/* 
# 不删除地图、任务、拓扑点等数据。
# ==================================================
LOG_BASE_DIR="${ROBOT_DATA_DIR}/log"
ROBOT_DATA_PARENT="$(dirname "${ROBOT_DATA_DIR}")"
ROBOT_HOME_PARENT="$(dirname "${ROBOT_HOME}")"

# 防止目录变量异常时误删上层目录。
if [ -z "${ROBOT_DATA_DIR}" ] || \
   [ "${ROBOT_DATA_DIR}" = "/" ] || \
   [ "${ROBOT_DATA_DIR}" = "${ROBOT_HOME_PARENT}" ] || \
   [ "${ROBOT_DATA_DIR}" = "${ROBOT_HOME}" ] || \
   [ "${ROBOT_DATA_DIR}" = "${ROBOT_DATA_PARENT}" ]; then
    err "ROBOT_DATA_DIR 异常，拒绝清理日志: ${ROBOT_DATA_DIR}"
    exit 1
fi

clean_old_logs() {
    if [ -d "${LOG_BASE_DIR}" ]; then
        info "清理之前所有运行日志: ${LOG_BASE_DIR}"

        # 删除 log 目录下面所有历史文件、目录和隐藏文件。
        find "${LOG_BASE_DIR}" -mindepth 1 -maxdepth 1 -exec rm -rf -- {} +

        info "历史运行日志清理完成"
    else
        info "历史日志目录不存在，创建: ${LOG_BASE_DIR}"
    fi

    mkdir -p "${LOG_BASE_DIR}"
}

clean_old_logs

# 清理完旧日志后，再创建本次自动启动日志目录。
LOG_ROOT="${WORKFLOW_LOG_DIR:-${LOG_BASE_DIR}/autostart_$(date +%Y%m%d_%H%M%S)}"
AUTOSTART_LOG_DIR="${LOG_ROOT}/autostart"

mkdir -p "${AUTOSTART_LOG_DIR}"
export WORKFLOW_LOG_DIR="${LOG_ROOT}"

NAV_PID=""
CLEANING=0

cleanup() {
    if [ "${CLEANING}" -eq 1 ]; then
        return
    fi
    CLEANING=1

    if [ -n "${NAV_PID}" ] && kill -0 "${NAV_PID}" 2>/dev/null; then
        info "自启动脚本退出，停止 run_nav.sh..."
        kill -SIGINT "-${NAV_PID}" 2>/dev/null || kill -SIGINT "${NAV_PID}" 2>/dev/null || true
        wait "${NAV_PID}" 2>/dev/null || true
    fi
}

trap cleanup INT TERM

require_script() {
    local script="$1"
    if [ ! -x "${script}" ]; then
        err "脚本不存在或不可执行: ${script}"
        exit 1
    fi
}

run_cleanup_nodes() {
    info "清理残留 ROS2 节点..."
    KILL_NAV_GUI=1 "${RUN_KILL_SCRIPT}" > "${AUTOSTART_LOG_DIR}/kill_$(date +%Y%m%d_%H%M%S).log" 2>&1 || true
}

run_relocalization_once() {
    local attempt="$1"
    local log_file="${AUTOSTART_LOG_DIR}/relocalization_attempt_${attempt}_$(date +%Y%m%d_%H%M%S).log"

    info "启动重定位，第 ${attempt}/${AUTO_NAV_ATTEMPTS} 次..."
    if "${RUN_RELOCALIZATION_SCRIPT}" > "${log_file}" 2>&1; then
        local success_file="${LOG_ROOT}/relocalization/relocalization_success.txt"
        if [ -f "${success_file}" ] && [ "$(cat "${success_file}" 2>/dev/null)" = "1" ]; then
            info "重定位成功"
            return 0
        fi

        warn "run_relocalization.sh 返回 0，但未读取到成功标志: ${success_file}"
        return 1
    fi

    warn "重定位失败，日志: ${log_file}"
    return 1
}

resolve_nav_signal() {
    if [ -n "${AUTO_NAV_SIGNAL}" ]; then
        echo "${AUTO_NAV_SIGNAL}"
        return
    fi

    if [ -n "${AUTO_NAV_TASK}" ]; then
        if [ "${AUTO_NAV_LOOP}" = "1" ]; then
            echo "LOOP:${AUTO_NAV_TASK}"
        else
            echo "${AUTO_NAV_TASK}"
        fi
        return
    fi

    local first_task=""
    if [ -d "${AUTO_NAV_TASK_FOLDER}" ]; then
        first_task="$(find "${AUTO_NAV_TASK_FOLDER}" -maxdepth 1 -type f -name "*.txt" 2>/dev/null | sort | head -n 1)"
    fi

    if [ -z "${first_task}" ]; then
        err "未指定 AUTO_NAV_TASK/AUTO_NAV_SIGNAL，且任务目录没有 .txt: ${AUTO_NAV_TASK_FOLDER}"
        return 1
    fi

    local task_name
    task_name="$(basename "${first_task}" .txt)"

    if [ "${AUTO_NAV_LOOP}" = "1" ]; then
        echo "LOOP:${task_name}"
    else
        echo "${task_name}"
    fi
}

wait_for_navigation_state_machine() {
    local deadline=$((SECONDS + AUTO_NAV_STARTUP_WAIT))

    info "等待导航状态机启动，最多 ${AUTO_NAV_STARTUP_WAIT}s..."
    while [ "${SECONDS}" -lt "${deadline}" ]; do
        if ros2 node list 2>/dev/null | grep -qx "/navigation_task_state_machine_node"; then
            info "导航状态机已启动"
            return 0
        fi
        sleep 1
    done

    err "等待导航状态机超时"
    return 1
}

publish_nav_start_signal() {
    local signal="$1"

    info "自动发送导航开始信号: ${signal}"

    # 连续发布几次，避免刚启动时订阅端尚未完全 ready。
    for _ in 1 2 3 4 5; do
        ros2 topic pub --once /nav_task_signal std_msgs/msg/String "{data: '${signal}'}" >/dev/null 2>&1 || true
        sleep 1
    done
}

main() {
    require_script "${RUN_KILL_SCRIPT}"
    require_script "${RUN_RELOCALIZATION_SCRIPT}"
    require_script "${RUN_NAV_SCRIPT}"

    info "开机自启动导航流程"
    info "工作空间: ${WORKSPACE_DIR}"
    info "日志目录: ${LOG_ROOT}"

    run_cleanup_nodes

    local success=0
    for attempt in $(seq 1 "${AUTO_NAV_ATTEMPTS}"); do
        if run_relocalization_once "${attempt}"; then
            success=1
            break
        fi

        if [ "${attempt}" -lt "${AUTO_NAV_ATTEMPTS}" ]; then
            warn "等待 ${AUTO_NAV_RETRY_DELAY}s 后重新清理并重试重定位"
            sleep "${AUTO_NAV_RETRY_DELAY}"
            run_cleanup_nodes
        fi
    done

    if [ "${success}" -ne 1 ]; then
        err "重定位连续失败，停止自启动流程"
        exit 1
    fi

    local nav_signal
    nav_signal="$(resolve_nav_signal)"

    info "启动 run_nav.sh..."
    setsid "${RUN_NAV_SCRIPT}" > "${AUTOSTART_LOG_DIR}/run_nav_$(date +%Y%m%d_%H%M%S).log" 2>&1 &
    NAV_PID=$!

    if ! kill -0 "${NAV_PID}" 2>/dev/null; then
        err "run_nav.sh 启动失败"
        exit 1
    fi

    wait_for_navigation_state_machine
    publish_nav_start_signal "${nav_signal}"

    info "自启动流程完成，run_nav.sh 正在前台流程中运行，PID=${NAV_PID}"
    wait "${NAV_PID}"
}

main "$@"
