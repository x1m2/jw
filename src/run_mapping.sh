#!/bin/bash
set -e

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "${SCRIPT_PATH}")"
WORKSPACE_DIR="$(dirname "${SCRIPT_DIR}")"

info() { echo "[INFO] $*"; }
warn() { echo "[WARN] $*"; }
err()  { echo "[ERROR] $*"; }

usage() {
    cat <<EOF
用法: $(basename "$0") [--lidar mid360|airy] [--ota-root OTA目录] [--check]
      $(basename "$0") [mid360|airy]

可通过命令行参数或 LIDAR_TYPE 环境变量选择雷达，默认使用 mid360。
不传 --ota-root 时使用源码工作空间；传入后直接运行 OTA 程序，无需 install/setup.bash。
--check 只检查运行文件，不创建数据目录或启动节点。
DATA_DIR 可指定本次建图的数据目录，地图写入其 map 子目录。
示例:
  $(basename "$0") --lidar airy
  $(basename "$0") mid360
  DATA_DIR=/tmp/airy_map $(basename "$0") --lidar airy --ota-root /home/jw/ota
EOF
}

LIDAR_TYPE="${LIDAR_TYPE:-mid360}"
MAPPING_OTA_ROOT=""
CHECK_ONLY=false
while [ "$#" -gt 0 ]; do
    case "$1" in
        --lidar)
            if [ "$#" -lt 2 ]; then
                err "--lidar 后需要填写 airy 或 mid360"
                usage >&2
                exit 1
            fi
            LIDAR_TYPE="$2"
            shift 2
            ;;
        airy|mid360)
            LIDAR_TYPE="$1"
            shift
            ;;
        --ota-root)
            if [ "$#" -lt 2 ] || [ -z "$2" ]; then
                err "--ota-root 后需要填写 OTA 目录"
                exit 1
            fi
            MAPPING_OTA_ROOT="$2"
            shift 2
            ;;
        --check)
            CHECK_ONLY=true
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            err "未知参数: $1"
            usage >&2
            exit 1
            ;;
    esac
done

readonly LIDAR_TYPE
readonly AIRY_SDK_DIR="${SCRIPT_DIR}/drivers/airy96_ros_driver2/rslidar_sdk"
AIRY_DRIVER_CONFIG="${AIRY_SDK_DIR}/config/airy96_driver.yaml"
AIRY_FASTDDS_PROFILE="${AIRY_SDK_DIR}/config/fastshm.xml"

case "${LIDAR_TYPE}" in
    mid360)
        readonly FASTLIO_CONFIG="mapping_mid360s.yaml"
        ROBOT_PARAM_FILE="${SCRIPT_DIR}/config/P2.5_mid360s/P2.5_mid360s.yaml"
        readonly PGO_CONFIG="pgo_mid360s.yaml"
        readonly LIDAR_DRIVER_NAME="Livox ROS Driver2"
        ;;
    airy)
        readonly FASTLIO_CONFIG="mapping_airy96.yaml"
        ROBOT_PARAM_FILE="${SCRIPT_DIR}/config/P3_airy96/P3_airy96.yaml"
        readonly PGO_CONFIG="pgo_airy96.yaml"
        readonly LIDAR_DRIVER_NAME="Airy96 ROS Driver"
        ;;
    *)
        err "不支持的雷达类型: ${LIDAR_TYPE}，请选择 airy 或 mid360"
        usage >&2
        exit 1
        ;;
esac

readonly MAP_ID_INPUT="${MAP_ID:-0}"
if ! [[ "${MAP_ID_INPUT}" =~ ^[0-9]+$ ]] ||
   [ "${#MAP_ID_INPUT}" -gt 10 ] ||
   { [ "${#MAP_ID_INPUT}" -eq 10 ] && [[ "${MAP_ID_INPUT}" > "4294967295" ]]; }; then
    err "MAP_ID 必须是 uint32：${MAP_ID_INPUT}"
    exit 1
fi
readonly MAP_ID="$((10#${MAP_ID_INPUT}))"
# GUI 会显式传入 DATA_DIR；直接执行脚本时默认保存到该目录。
if [ -n "${MAPPING_OTA_ROOT}" ]; then
    if [ ! -d "${MAPPING_OTA_ROOT}" ]; then
        err "OTA 目录不存在: ${MAPPING_OTA_ROOT}"
        exit 1
    fi
    MAPPING_OTA_ROOT="$(cd -- "${MAPPING_OTA_ROOT}" && pwd)"
    SLAM_DIR="${MAPPING_OTA_ROOT}/slam/pack/content"
    MANAGER_DIR="${MAPPING_OTA_ROOT}/slam_manager/pack/content"
    AIRY_DRIVER_CONFIG="${SLAM_DIR}/config/airy96_driver.yaml"
    AIRY_FASTDDS_PROFILE="${SLAM_DIR}/config/fastshm.xml"
    if [ "${LIDAR_TYPE}" = "airy" ]; then
        ROBOT_PARAM_FILE="${MANAGER_DIR}/config/p3_airy96.yaml"
        DRIVER_BINARY="${SLAM_DIR}/lidar/rslidar_sdk_node"
    else
        ROBOT_PARAM_FILE="${MANAGER_DIR}/config/p2.5_mid360s.yaml"
        DRIVER_BINARY="${SLAM_DIR}/lidar/livox_ros_driver2_node"
    fi
    FASTLIO_PARAM_FILE="${SLAM_DIR}/config/${FASTLIO_CONFIG}"
    PGO_PARAM_FILE="${SLAM_DIR}/config/${PGO_CONFIG}"
    BBS_SAVER="${SLAM_DIR}/localization/voxelmaps_saver"
    MAPPING_DEFAULT_DATA_DIR="${MAPPING_OTA_ROOT}/data/slam_data/standalone_$(date +%Y%m%d_%H%M%S)_$$"
else
    BBS_SAVER="/home/jw/3d_bbs/test/build/saver"
    MAPPING_DEFAULT_DATA_DIR="/home/jw/slam_data"
fi
DATA_DIR="${DATA_DIR:-${MAPPING_DEFAULT_DATA_DIR}}"
# 参数和子进程均使用绝对路径，避免切换工作目录后保存到错误位置。
readonly DATA_DIR="$(realpath -m -- "${DATA_DIR}")"
readonly MAP_DIR="${DATA_DIR}/map"

if [ -z "${MAPPING_OTA_ROOT}" ] && [ ! -d "${WORKSPACE_DIR}/src" ]; then
    err "找不到 src 目录: ${WORKSPACE_DIR}/src"
    exit 1
fi

if [ -z "${MAPPING_OTA_ROOT}" ] && [ ! -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
    err "找不到工作空间环境: ${WORKSPACE_DIR}/install/setup.bash"
    err "请确认工作空间已经编译，并且路径正确"
    exit 1
fi

REQUIRED_FILES=(/opt/ros/humble/setup.bash "${ROBOT_PARAM_FILE}")
if [ -n "${MAPPING_OTA_ROOT}" ]; then
    REQUIRED_FILES+=("${FASTLIO_PARAM_FILE}" "${PGO_PARAM_FILE}")
    for binary in "${DRIVER_BINARY}" "${SLAM_DIR}/mapping/fastlio_mapping_node" \
        "${SLAM_DIR}/pgo_sc/pgo_sc_node" "${BBS_SAVER}"; do
        if [ ! -x "${binary}" ]; then
            err "程序不存在或不可执行: ${binary}"
            exit 1
        fi
    done
    if [ "${LIDAR_TYPE}" = "mid360" ]; then
        REQUIRED_FILES+=("${SLAM_DIR}/config/MID360s_driver.yaml" "${SLAM_DIR}/config/MID360s_config.json")
    fi
fi
if [ "${LIDAR_TYPE}" = "airy" ]; then
    REQUIRED_FILES+=("${AIRY_DRIVER_CONFIG}" "${AIRY_FASTDDS_PROFILE}")
fi
for config in "${REQUIRED_FILES[@]}"; do
    if [ ! -r "${config}" ]; then
        err "文件不存在或不可读: ${config}"
        exit 1
    fi
done
if "${CHECK_ONLY}"; then
    info "运行文件检查通过；未启动节点"
    info "机体配置: ${ROBOT_PARAM_FILE}"
    exit 0
fi

source /opt/ros/humble/setup.bash
if [ -n "${MAPPING_OTA_ROOT}" ]; then
    cd "${MAPPING_OTA_ROOT}"
    export LD_LIBRARY_PATH="${SLAM_DIR}/lib:${MANAGER_DIR}/lib:${MAPPING_OTA_ROOT}/nav/pack/content/lib:/opt/ros/humble/lib:${LD_LIBRARY_PATH:-}"
    # 不停止已有服务，避免与正在运行的 OTA 驱动/建图进程重复发布或保存。
    if pgrep -f '(^|/)(rslidar_sdk_node|livox_ros_driver2_node|fastlio_mapping_node|pgo_sc_node)([[:space:]]|$)' >/dev/null; then
        err "已有雷达或建图程序运行，请先停止对应服务，再启动独立建图"
        exit 1
    fi
else
    cd "${WORKSPACE_DIR}"
    source "${WORKSPACE_DIR}/install/setup.bash"
fi

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}

if [ "${LIDAR_TYPE}" = "airy" ]; then
    if [ ! -f "${AIRY_DRIVER_CONFIG}" ]; then
        err "找不到 Airy96 驱动配置: ${AIRY_DRIVER_CONFIG}"
        exit 1
    fi
    if [ ! -f "${AIRY_FASTDDS_PROFILE}" ]; then
        err "找不到 Airy96 FastDDS 配置: ${AIRY_FASTDDS_PROFILE}"
        exit 1
    fi

    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export FASTRTPS_DEFAULT_PROFILES_FILE="${AIRY_FASTDDS_PROFILE}"
    export RMW_FASTRTPS_USE_QOS_FROM_XML=1
fi
if [ -n "${MAPPING_OTA_ROOT}" ] && [ "${LIDAR_TYPE}" = "mid360" ]; then
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE RMW_FASTRTPS_USE_QOS_FROM_XML
fi

# 地图、日志和任务数据保存在本次 DATA_DIR 下。
mkdir -p "${MAP_DIR}" "${DATA_DIR}/log" "${DATA_DIR}/task"

if [ -n "${WORKFLOW_LOG_DIR:-}" ]; then
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
LIDAR_DRIVER_LOG_FILE="${SLAM_LOG_DIR}/${LIDAR_TYPE}_driver_$(log_time).log"
PGO_LOG_FILE="${SLAM_LOG_DIR}/pgo_sc_$(log_time).log"
TOPOLOGY_LOG_FILE="${SLAM_LOG_DIR}/topology_map_generator_$(log_time).log"


info "开始建图流程"
info "雷达类型: ${LIDAR_TYPE}"
info "FAST-LIO 配置: ${FASTLIO_CONFIG}"
info "PGO 配置: ${PGO_CONFIG}"
info "机体配置: ${ROBOT_PARAM_FILE}"
if [ -n "${MAPPING_OTA_ROOT}" ]; then
    info "OTA 目录: ${MAPPING_OTA_ROOT}（独立建图，runtime_profile=test）"
fi
info "数据目录: ${DATA_DIR}"

# ==================================================
# 进程记录
# ==================================================
declare -a PIDS=()

LIDAR_DRIVER_PID=""
FASTLIO_PID=""
PGO_PID=""
TOPOLOGY_PID=""
CLEANING=0

SAVE_POPUP_PID=""
SLAM_MANAGER_DIR="$(dirname "${DATA_DIR}")"
SAVE_POPUP_SIGNAL="${SLAM_MANAGER_DIR}/run_mapping_save_popup_done_$$"
SAVE_POPUP_STATUS="${SLAM_MANAGER_DIR}/run_mapping_save_popup_status_$$"

show_save_popup_with_done_button() {
    rm -f "${SAVE_POPUP_SIGNAL}" "${SAVE_POPUP_STATUS}"

    python3 - "${SAVE_POPUP_SIGNAL}" "${SAVE_POPUP_STATUS}" "${DATA_DIR}" <<'PYEOF' &
import os
import sys
import tkinter as tk

signal_file, status_file, save_dir = sys.argv[1:]

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
        return 0
    fi

    if ! kill -0 -- "-${pid}" 2>/dev/null; then
        return 0
    fi

    info "停止 ${name}"
    kill -SIGINT "-${pid}" 2>/dev/null || true
    sleep 2

    if kill -0 -- "-${pid}" 2>/dev/null; then
        kill -SIGTERM "-${pid}" 2>/dev/null || true
    fi

    sleep 1

    if kill -0 -- "-${pid}" 2>/dev/null; then
        warn "${name} 未退出，强制结束"
        kill -SIGKILL "-${pid}" 2>/dev/null || true
    fi
}

generate_3d_bbs_data() {
    # pgo_sc 的 /save_done 到达后再运行，避免 saver 读取未写完的地图。
    if [ ! -x "${BBS_SAVER}" ]; then
        err "找不到 3d_bbs saver 可执行文件: ${BBS_SAVER}"
        return 1
    fi

    if [ ! -d "${MAP_DIR}" ]; then
        err "地图目录不存在: ${MAP_DIR}"
        return 1
    fi

    info "运行 3d_bbs saver..."
    if "${BBS_SAVER}" "${MAP_DIR}/" 0.5 6; then
        info "3d_bbs saver 执行完成"
        return 0
    fi

    err "3d_bbs saver 执行失败"
    return 1
}

cleanup() {
    local exit_status="${1:-0}"
    if [ "${CLEANING}" -eq 1 ]; then
        return
    fi

    CLEANING=1
    trap - SIGINT SIGTERM EXIT
    # 保存或单个节点退出失败时，仍须清理其余已启动的进程。
    set +e

    echo ""
    info "结束建图，开始保存地图和轨迹..."

    if [ -z "${PGO_PID}" ] || ! kill -0 "${PGO_PID}" 2>/dev/null; then
        warn "PGO 未运行，跳过保存"
    else
        if [ -z "${MAPPING_OTA_ROOT}" ] && [ -n "${DISPLAY:-}" ]; then
            show_save_popup_with_done_button
        fi

        # 继承当前 ROS/DDS 环境，不依赖源码工作空间的 setup.bash。
        timeout 120s ros2 topic echo --once /save_done std_msgs/msg/Empty \
            --qos-reliability reliable --qos-durability transient_local \
            >"${SLAM_LOG_DIR}/pgo_save_done.log" 2>&1 &
        SAVE_DONE_WAIT_PID=$!

        sleep 1

        info "触发 pgo_sc 保存..."
        timeout 30s ros2 topic pub --once -w 1 /save_dir std_msgs/msg/Empty "{}" \
            >"${SLAM_LOG_DIR}/pgo_save_trigger.log" 2>&1 || true

        info "等待保存完成信号..."
        if ! wait "${SAVE_DONE_WAIT_PID}" 2>/dev/null; then
            err "保存未确认完成，未收到 /save_done"
            exit_status=1
            echo "failed" > "${SAVE_POPUP_STATUS}"
        else
            info "地图和轨迹保存完成"
            if generate_3d_bbs_data; then
                echo "success" > "${SAVE_POPUP_STATUS}"
            else
                exit_status=1
                echo "bbs_failed" > "${SAVE_POPUP_STATUS}"
            fi
        fi

        touch "${SAVE_POPUP_SIGNAL}"

        if [ -n "${SAVE_POPUP_PID}" ]; then
            wait "${SAVE_POPUP_PID}" 2>/dev/null || true
        fi
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
    exit "${exit_status}"
}

trap 'cleanup 0' SIGINT SIGTERM
trap 'cleanup $?' EXIT

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
if [ -n "${MAPPING_OTA_ROOT}" ]; then
    if [ "${LIDAR_TYPE}" = "airy" ]; then
        start_bg LIDAR_DRIVER_PID "${LIDAR_DRIVER_NAME}" "${LIDAR_DRIVER_LOG_FILE}" 4 \
            "${DRIVER_BINARY}" --ros-args -p "config_path:=${AIRY_DRIVER_CONFIG}"
    else
        start_bg LIDAR_DRIVER_PID "${LIDAR_DRIVER_NAME}" "${LIDAR_DRIVER_LOG_FILE}" 4 \
            "${DRIVER_BINARY}" --ros-args \
            --params-file "${SLAM_DIR}/config/MID360s_driver.yaml" \
            -p "user_config_path:=${SLAM_DIR}/config/MID360s_config.json"
    fi
    start_bg FASTLIO_PID "FAST-LIO" "${FASTLIO_LOG_FILE}" 3 \
        "${SLAM_DIR}/mapping/fastlio_mapping_node" --ros-args \
        --params-file "${FASTLIO_PARAM_FILE}" --params-file "${ROBOT_PARAM_FILE}" \
        -p runtime_profile:=test
    start_bg PGO_PID "PGO SC" "${PGO_LOG_FILE}" 1 \
        "${SLAM_DIR}/pgo_sc/pgo_sc_node" --ros-args \
        --params-file "${PGO_PARAM_FILE}" -p runtime_profile:=test \
        -p "basic.package_path:=${MAP_DIR}"
else
    if [ "${LIDAR_TYPE}" = "airy" ]; then
        info "Airy96 FastDDS 配置: ${FASTRTPS_DEFAULT_PROFILES_FILE}"
        start_bg LIDAR_DRIVER_PID "${LIDAR_DRIVER_NAME}" "${LIDAR_DRIVER_LOG_FILE}" 4 \
            ros2 run rslidar_sdk rslidar_sdk_node --ros-args \
            -p "config_path:=${AIRY_DRIVER_CONFIG}"
    else
        start_bg LIDAR_DRIVER_PID "${LIDAR_DRIVER_NAME}" "${LIDAR_DRIVER_LOG_FILE}" 4 \
            ros2 launch livox_ros_driver2 msg_MID360s_launch.py
    fi

    start_bg FASTLIO_PID "FAST-LIO" "${FASTLIO_LOG_FILE}" 3 \
        ros2 launch fast_lio mapping.launch.py fast_lio_use_rviz:=false \
        config_file:="${FASTLIO_CONFIG}" robot_param_file:="${ROBOT_PARAM_FILE}"

    start_bg PGO_PID "PGO SC" "${PGO_LOG_FILE}" 1 \
        ros2 launch pgo_sc run.launch.py pgo_use_rviz:=true data_dir:="${DATA_DIR}" \
        config_file:="${PGO_CONFIG}"
fi

# 建图过程中按 K 保存拓扑点的旧流程已停用。
# 建图完成后统一使用 rviz_topology_editor 在优化后的 PCD 地图上打点。
# start_bg TOPOLOGY_PID "topology_map_generator" "${TOPOLOGY_LOG_FILE}" 1 \
#     ros2 launch topology_map_generator topology_map_generator.launch.py

# ==================================================
# 前台键盘控制
# ==================================================
echo ""
info "独立建图已启动"
info "结束建图：在当前终端按 Ctrl+C"
echo ""

if [ -z "${MAPPING_OTA_ROOT}" ] && [ -f "${WORKSPACE_DIR}/src/scripts/keyboard_control.py" ]; then
    python3 -u "${WORKSPACE_DIR}/src/scripts/keyboard_control.py" || true
else
    info "RViz查看车体系点云：Fixed Frame=base_link，Topic=/cloud_body_downsampled"
    while true; do
        for pid in "${PIDS[@]}"; do
            if ! kill -0 "${pid}" 2>/dev/null; then
                err "建图进程退出，查看日志: ${SLAM_LOG_DIR}"
                exit 1
            fi
        done
        sleep 1
    done
fi

cleanup
