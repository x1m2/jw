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
    err "请确认工作空间已经编译，并且路径正确"
    exit 1
fi

cd "${WORKSPACE_DIR}"

source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}
mkdir -p "${ROBOT_DATA_DIR}"

if [ -n "${WORKFLOW_LOG_DIR}" ]; then
    LOG_ROOT="${WORKFLOW_LOG_DIR}"
else
    LOG_ROOT="${ROBOT_DATA_DIR}/log/$(date +%Y%m%d_%H%M%S)"
fi

EDIT_MAP_DIR="${LOG_ROOT}/edit_map"
mkdir -p "${EDIT_MAP_DIR}"


log_time() {
    date +%Y%m%d_%H%M%S
}

TOPOLOGY_CONNECT_LOG_FILE="${EDIT_MAP_DIR}/topology_connect_$(log_time).log"
TOPOLOGY_TOOL_LOG_FILE="${EDIT_MAP_DIR}/topology_tool_$(log_time).log"

TOPOLOGY_MAP_DIR="${ROBOT_DATA_DIR}/topology_map"
TARGET_POINTS_FILE="${TOPOLOGY_MAP_DIR}/generated_map.txt"
FALLBACK_TARGET_POINTS_FILE="${TOPOLOGY_MAP_DIR}/generated_map_target_points.txt"

TASK_DIR="${ROBOT_DATA_DIR}/task"
TASK_FILE="${TASK_DIR}/test.txt"

info "开始地图编辑流程"
info "数据目录: ${ROBOT_DATA_DIR}"

# ==================================================
# 只关闭本脚本启动的进程组
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
    kill -INT "-${pid}" 2>/dev/null || kill -INT "${pid}" 2>/dev/null || true
    sleep 2

    if kill -0 "${pid}" 2>/dev/null; then
        kill -TERM "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
    fi

    sleep 1

    if kill -0 "${pid}" 2>/dev/null; then
        warn "${name} 未退出，强制结束"
        kill -KILL "-${pid}" 2>/dev/null || kill -KILL "${pid}" 2>/dev/null || true
    fi
}

# ==================================================
# 自动生成状态机任务文件
# ==================================================
generate_task_file() {
    local input_file=""

    if [ -f "${TARGET_POINTS_FILE}" ]; then
        input_file="${TARGET_POINTS_FILE}"
    elif [ -f "${FALLBACK_TARGET_POINTS_FILE}" ]; then
        input_file="${FALLBACK_TARGET_POINTS_FILE}"
        warn "未找到 generated_map.txt，使用备用目标点文件"
    else
        err "  ${TARGET_POINTS_FILE}"
        err "  ${FALLBACK_TARGET_POINTS_FILE}"
        err "请先运行建图打点，生成目标点文件"
        exit 1
    fi

    mkdir -p "${TASK_DIR}"

    local node_count
    node_count=$(grep -v '^#' "${input_file}" | grep -v '^$' | wc -l)

    if [ "${node_count}" -le 0 ]; then
        err "目标点文件为空: ${input_file}"
        exit 1
    fi

    cat > "${TASK_FILE}" <<EOF
# 任务列表 (UTF-8编码)
# 格式：目标点ID 等待延时 抵达后发布的数据
EOF

    awk '!/^#/ && NF {print $1 " 0 1"}' "${input_file}" >> "${TASK_FILE}"

    info "任务文件已生成: ${TASK_FILE}"
    info "目标点数量: ${node_count}"
}

generate_task_file

# ==================================================
# 后台进程清理
# ==================================================
TOPOLOGY_CONNECT_PID=""
PCD2PGM_PID=""
CLEANING=0

cleanup() {
    if [ "${CLEANING}" -eq 1 ]; then
        return
    fi

    CLEANING=1
    trap - EXIT INT TERM

    info "退出地图编辑流程，清理本脚本后台进程..."

    if [ -n "${TOPOLOGY_CONNECT_PID}" ]; then
        kill_process_group_safe "${TOPOLOGY_CONNECT_PID}" "topology_connect"
    fi

    if [ -n "${PCD2PGM_PID}" ]; then
        kill_process_group_safe "${PCD2PGM_PID}" "pcd2pgm"
    fi

    TOPOLOGY_CONNECT_PID=""
    PCD2PGM_PID=""
    wait 2>/dev/null || true

    info "地图编辑流程已结束"
    info "数据目录: ${ROBOT_DATA_DIR}"
}

trap cleanup EXIT INT TERM

# ==================================================
# 1. 后台启动 PCD -> PGM 地图发布
# ==================================================
info "启动 pcd2pgm..."

setsid ros2 launch pcd2pgm pcd2pgm_launch.py pgm_use_rviz:=false \
    > /dev/null 2>&1 &

PCD2PGM_PID=$!

sleep 2

if ! kill -0 "${PCD2PGM_PID}" 2>/dev/null; then
    err "pcd2pgm 启动失败"
    exit 1
fi

info "pcd2pgm 启动成功"

# ==================================================
# 2. 一次性生成拓扑顺序连边
# ==================================================
info "启动 topology_connect..."

setsid ros2 launch topology_rviz_plugin topology_connect.launch.py \
    > "${TOPOLOGY_CONNECT_LOG_FILE}" 2>&1 &

TOPOLOGY_CONNECT_PID=$!

TOPOLOGY_CONNECT_DONE=0
for _ in {1..100}; do
    if grep -q "成功写入连接后的拓扑地图" "${TOPOLOGY_CONNECT_LOG_FILE}" 2>/dev/null; then
        TOPOLOGY_CONNECT_DONE=1
        break
    fi

    if ! kill -0 "${TOPOLOGY_CONNECT_PID}" 2>/dev/null; then
        break
    fi

    sleep 0.1
done

if [ "${TOPOLOGY_CONNECT_DONE}" -ne 1 ]; then
    err "topology_connect 连边失败或超时"
    err "日志: ${TOPOLOGY_CONNECT_LOG_FILE}"
    tail -n 80 "${TOPOLOGY_CONNECT_LOG_FILE}" || true
    exit 1
fi

kill_process_group_safe "${TOPOLOGY_CONNECT_PID}" "topology_connect"
TOPOLOGY_CONNECT_PID=""

info "topology_connect 连边完成"

# ==================================================
# 3. 启动 RViz 拓扑编辑工具
# ==================================================
info "启动 topology_tool，关闭 RViz 后自动结束地图编辑..."

ros2 launch topology_rviz_plugin topology_tool.launch.py \
    > "${TOPOLOGY_TOOL_LOG_FILE}" 2>&1

info "topology_tool 已退出"

cleanup
