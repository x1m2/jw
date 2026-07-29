#!/bin/bash
set -e

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "${SCRIPT_PATH}")"
WORKSPACE_DIR="$(dirname "${SCRIPT_DIR}")"

info() { echo "[INFO] $*"; }
warn() { echo "[WARN] $*"; }
err()  { echo "[ERROR] $*"; }

readonly SLAM_DATA_ROOT="/robotfs/data/slam_data"
readonly MAP_ID_INPUT="${MAP_ID:-0}"
if ! [[ "${MAP_ID_INPUT}" =~ ^[0-9]+$ ]] ||
   [ "${#MAP_ID_INPUT}" -gt 10 ] ||
   { [ "${#MAP_ID_INPUT}" -eq 10 ] && [[ "${MAP_ID_INPUT}" > "4294967295" ]]; }; then
    err "MAP_ID 必须是 uint32：${MAP_ID_INPUT}"
    exit 1
fi
readonly MAP_ID="$((10#${MAP_ID_INPUT}))"
# GUI 会显式传入 DATA_DIR；直接执行脚本时默认保存到 map_0。
readonly DATA_DIR="${DATA_DIR:-${SLAM_DATA_ROOT}/map_${MAP_ID}}"
readonly BBS_BUILD_DIR="${WORKSPACE_DIR}/env/3d_bbs/test/build"
readonly MAP_DIR="${DATA_DIR}/map"

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
# 每张地图的数据均收敛到 map_<id>，并预先建立统一的子目录。
mkdir -p "${MAP_DIR}" "${DATA_DIR}/log" "${DATA_DIR}/task"

if [ -n "${WORKFLOW_LOG_DIR}" ]; then
    LOG_ROOT="${WORKFLOW_LOG_DIR}"
else
    LOG_ROOT="${DATA_DIR}/log/$(date +%Y%m%d_%H%M%S)"
fi

SLAM_LOG_DIR="${LOG_ROOT}/slam"
mkdir -p "${SLAM_LOG_DIR}"

log_time() {
    date +%Y%m%d_%H%M%S
}

FASTLIO_LOG_FILE="${SLAM_LOG_DIR}/fast_lio_$(log_time).log"
PGO_LOG_FILE="${SLAM_LOG_DIR}/pgo_sc_$(log_time).log"
TOPOLOGY_LOG_FILE="${SLAM_LOG_DIR}/topology_map_generator_$(log_time).log"


info "开始建图流程"
info "数据目录: ${DATA_DIR}"

# ==================================================
# 进程记录
# ==================================================
declare -a PIDS=()

LIVOX_PID=""
FASTLIO_PID=""
PGO_PID=""
TOPOLOGY_PID=""
CLEANING=0

SAVE_POPUP_PID=""
ROBOT_RUNTIME_DIR="$(dirname "${DATA_DIR}")"
SAVE_POPUP_SIGNAL="${ROBOT_RUNTIME_DIR}/run_slam_save_popup_done_$$"
SAVE_POPUP_STATUS="${ROBOT_RUNTIME_DIR}/run_slam_save_popup_status_$$"

show_save_popup_with_done_button() {
    rm -f "${SAVE_POPUP_SIGNAL}" "${SAVE_POPUP_STATUS}"

    python3 - <<PYEOF &
import os
import tkinter as tk

signal_file = "${SAVE_POPUP_SIGNAL}"
status_file = "${SAVE_POPUP_STATUS}"
save_dir = "${DATA_DIR}"

root = tk.Tk()
root.title("地图保存")
root.geometry("680x300")
root.resizable(False, False)
root.attributes("-topmost", True)

label = tk.Label(
    root,
    text="检测到 Ctrl+C\n\n正在保存地图、轨迹并生成 3d_bbs 数据\n请不要关闭终端...",
    font=("WenQuanYi Micro Hei", 15),
    justify="center",
)
label.pack(expand=True, padx=25, pady=20)

button = tk.Button(
    root,
    text="保存中...",
    font=("WenQuanYi Micro Hei", 13, "bold"),
    width=14,
    state=tk.DISABLED,
)
button.pack(pady=(0, 25))

def finish_and_close():
    root.destroy()

def disable_close():
    pass

root.protocol("WM_DELETE_WINDOW", disable_close)

def check_done():
    if os.path.exists(signal_file):
        status = "success"
        try:
            if os.path.exists(status_file):
                with open(status_file, "r") as f:
                    status = f.read().strip()
        except Exception:
            status = "unknown"

        if status == "success":
            label.config(
                text="地图、轨迹和 3d_bbs 数据已生成完成\n\n"
                     "保存目录：\n" + save_dir + "\n\n"
                     "请点击“完成”退出建图流程"
            )
        elif status == "bbs_failed":
            label.config(
                text="地图和轨迹已保存，但 3d_bbs 数据生成失败\n\n"
                     "请查看终端错误信息。\n"
                     "请点击“完成”退出建图流程"
            )
        else:
            label.config(
                text="未收到 pgo_sc 的 /save_done 完成信号\n\n"
                     "可能保存失败或保存超时。\n"
                     "请点击“完成”退出建图流程"
            )

        button.config(
            text="完成",
            state=tk.NORMAL,
            command=finish_and_close,
        )
        root.protocol("WM_DELETE_WINDOW", finish_and_close)
        root.lift()
        root.attributes("-topmost", True)
    else:
        root.after(500, check_done)

root.after(500, check_done)
root.mainloop()
PYEOF

    SAVE_POPUP_PID=$!
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
    kill -SIGINT "-${pid}" 2>/dev/null || true
    sleep 2

    if kill -0 "${pid}" 2>/dev/null; then
        kill -SIGTERM "-${pid}" 2>/dev/null || true
    fi

    sleep 1

    if kill -0 "${pid}" 2>/dev/null; then
        warn "${name} 未退出，强制结束"
        kill -SIGKILL "-${pid}" 2>/dev/null || true
    fi
}

generate_3d_bbs_data() {
    # pgo_sc 的 /save_done 到达后再运行，避免 saver 读取未写完的地图。
    if [ ! -x "${BBS_BUILD_DIR}/saver" ]; then
        err "找不到 3d_bbs saver 可执行文件: ${BBS_BUILD_DIR}/saver"
        err "请先运行 robot_rk3588_env.sh 完成 3d_bbs/test 编译"
        return 1
    fi

    if [ ! -d "${MAP_DIR}" ]; then
        err "地图目录不存在: ${MAP_DIR}"
        return 1
    fi

    info "运行 3d_bbs saver..."
    if (
        cd "${BBS_BUILD_DIR}"
        ./saver "${MAP_DIR}/" 0.5 6
    ); then
        info "3d_bbs saver 执行完成"
        return 0
    fi

    err "3d_bbs saver 执行失败"
    return 1
}

cleanup() {
    if [ "${CLEANING}" -eq 1 ]; then
        return
    fi

    CLEANING=1
    trap - SIGINT SIGTERM EXIT

    echo ""
    info "结束建图，开始保存地图和轨迹..."

    show_save_popup_with_done_button

    SAVE_DONE_FLAG="${ROBOT_RUNTIME_DIR}/pgo_sc_save_done_$$"
    rm -f "${SAVE_DONE_FLAG}"

    timeout 120s bash -c "
source /opt/ros/humble/setup.bash
source '${WORKSPACE_DIR}/install/setup.bash'

ros2 topic echo --once /save_done std_msgs/msg/Empty \
  --qos-reliability reliable \
  --qos-durability transient_local \
  >/dev/null 2>&1

touch '${SAVE_DONE_FLAG}'
" &
    SAVE_DONE_WAIT_PID=$!

    sleep 1

    info "触发 pgo_sc 保存..."
    timeout 30s ros2 topic pub --once -w 1 /save_dir std_msgs/msg/Empty "{}" \
        >"${SLAM_LOG_DIR}/pgo_save_trigger.log" 2>&1 || true

    info "等待保存完成信号..."
    wait "${SAVE_DONE_WAIT_PID}" 2>/dev/null || true

    if [ ! -f "${SAVE_DONE_FLAG}" ]; then
        err "保存未确认完成，未收到 /save_done"
        echo "failed" > "${SAVE_POPUP_STATUS}"
    else
        info "地图和轨迹保存完成"
        if generate_3d_bbs_data; then
            echo "success" > "${SAVE_POPUP_STATUS}"
        else
            echo "bbs_failed" > "${SAVE_POPUP_STATUS}"
        fi
    fi

    rm -f "${SAVE_DONE_FLAG}" 2>/dev/null || true
    touch "${SAVE_POPUP_SIGNAL}"

    if [ -n "${SAVE_POPUP_PID}" ]; then
        wait "${SAVE_POPUP_PID}" 2>/dev/null || true
    fi

    info "关闭后台建图节点..."

    kill_process_group_safe "${PGO_PID}" "pgo_sc"

    for pid in "${PIDS[@]}"; do
        [ -z "${pid}" ] && continue
        [ "${pid}" = "${PGO_PID}" ] && continue

        if [ "${pid}" = "${TOPOLOGY_PID}" ]; then
            kill_process_group_safe "${pid}" "topology_map_generator"
        else
            kill_process_group_safe "${pid}" "后台进程"
        fi
    done

    wait 2>/dev/null || true
    rm -f "${SAVE_POPUP_SIGNAL}" "${SAVE_POPUP_STATUS}" 2>/dev/null || true

    echo ""
    info "建图流程已结束"

    sleep 1
    exit 0
}

trap cleanup SIGINT SIGTERM

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
        if [ "${log_file}" != "/dev/null" ]; then
            err "日志: ${log_file}"
            tail -n 80 "${log_file}" || true
        fi
        exit 1
    fi

    info "${name} 启动成功"
}

# ==================================================
# 启动流程
# ==================================================
start_bg LIVOX_PID "Livox ROS Driver2" "/dev/null" 4 \
    ros2 launch livox_ros_driver2 msg_MID360s_launch.py

start_bg FASTLIO_PID "FAST-LIO" "${FASTLIO_LOG_FILE}" 3 \
    ros2 launch fast_lio mapping.launch.py fast_lio_use_rviz:=false

start_bg PGO_PID "PGO SC" "${PGO_LOG_FILE}" 1 \
    ros2 launch pgo_sc run.launch.py pgo_use_rviz:=true data_dir:="${DATA_DIR}"

# 建图过程中按 K 保存拓扑点的旧流程已停用。
# 建图完成后统一使用 rviz_topology_editor 在优化后的 PCD 地图上打点。
# start_bg TOPOLOGY_PID "topology_map_generator" "${TOPOLOGY_LOG_FILE}" 1 \
#     ros2 launch topology_map_generator topology_map_generator.launch.py

# ==================================================
# 前台键盘控制
# ==================================================
echo ""
info "键盘控制已启动，开始建图"
info "结束建图：在当前终端按 Ctrl+C"
echo ""

python3 -u "${WORKSPACE_DIR}/src/scripts/keyboard_control.py" || true

cleanup
