#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly DATA_DIR="$(pwd -P)"
LOG_ROOT="${LOG_ROOT:-${DATA_DIR}/log}"
MAP_DIR="${DATA_DIR}/map"
BAG_DIR=""
LIDAR_TYPE="${LIDAR_TYPE:-airy}"
PLAY_RATE="${PLAY_RATE:-1.0}"
AIRY_CONFIG="${AIRY_CONFIG:-${WORKSPACE_DIR}/src/drivers/airy96_ros_driver2/rslidar_sdk/config/airy96_driver.yaml}"
AIRY_FASTRTPS_PROFILE="${AIRY_FASTRTPS_PROFILE:-${WORKSPACE_DIR}/src/drivers/airy96_ros_driver2/rslidar_sdk/config/fastshm.xml}"
PGO_USE_RVIZ="${PGO_USE_RVIZ:-true}"

info() { printf '[INFO] %s\n' "$*"; }
warn() { printf '[WARN] %s\n' "$*"; }
err()  { printf '[ERROR] %s\n' "$*" >&2; }
fail() { err "$*"; exit 1; }

usage() {
    cat <<EOF
用法: $0 --bag <ROS 2 bag目录> [选项]

选项:
  --lidar airy|mid360    雷达类型，默认 airy
  --rate <倍速>          bag 回放倍速，默认 1.0
  --airy-config <文件>   Airy 离线解包配置文件
  --no-rviz              不启动 PGO RViz
  -h, --help             显示帮助

运行期间按键:
  空格                   暂停/继续 bag 回放
  Ctrl+C                 保存地图并退出

示例:
  $0 --lidar airy --bag /data/airy_bag --rate 1.0
  $0 --lidar mid360 --bag /data/mid360_bag --rate 0.5
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --bag)
            [[ $# -ge 2 ]] || fail "--bag 后需要填写 ROS 2 bag 目录"
            BAG_DIR="$2"
            shift 2
            ;;
        --lidar)
            [[ $# -ge 2 ]] || fail "--lidar 后需要填写 airy 或 mid360"
            LIDAR_TYPE="$2"
            shift 2
            ;;
        --rate)
            [[ $# -ge 2 ]] || fail "--rate 后需要填写回放倍速"
            PLAY_RATE="$2"
            shift 2
            ;;
        --airy-config)
            [[ $# -ge 2 ]] || fail "--airy-config 后需要填写 YAML 文件路径"
            AIRY_CONFIG="$2"
            shift 2
            ;;
        --no-rviz)
            PGO_USE_RVIZ=false
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            usage >&2
            fail "未知参数: $1"
            ;;
    esac
done

[[ -d "${WORKSPACE_DIR}/src" ]] || fail "找不到 src 目录: ${WORKSPACE_DIR}/src"
[[ -f "${WORKSPACE_DIR}/install/setup.bash" ]] || \
    fail "找不到工作空间环境: ${WORKSPACE_DIR}/install/setup.bash，请先编译工作空间"
[[ -n "${BAG_DIR}" ]] || { usage >&2; fail "必须通过 --bag 指定数据包目录"; }
[[ -f "${BAG_DIR}/metadata.yaml" ]] || fail "未找到 ROS 2 bag: ${BAG_DIR}/metadata.yaml"
BAG_DIR="$(cd "${BAG_DIR}" && pwd -P)"

if ! python3 - "${PLAY_RATE}" <<'PY'
import math
import sys

try:
    rate = float(sys.argv[1])
except ValueError:
    raise SystemExit(1)
raise SystemExit(0 if math.isfinite(rate) and rate > 0 else 1)
PY
then
    fail "回放倍速必须是大于 0 的数字: ${PLAY_RATE}"
fi

case "${LIDAR_TYPE}" in
    airy)
        FASTLIO_CONFIG="mapping_airy96.yaml"
        ROBOT_PARAM_FILE="${WORKSPACE_DIR}/src/config/P3_airy96.yaml"
        PGO_CONFIG="pgo_airy96.yaml"
        # Packet 解码为点云；Airy 的 IMU 已作为独立话题录入 bag，
        # rslidar_sdk 从 packet 回放时不会重新发布这一路 IMU。
        BAG_TOPICS=(/rslidar_packets /rslidar_imu_data /imu/data)
        [[ -f "${AIRY_CONFIG}" ]] || fail "未找到 Airy 配置文件: ${AIRY_CONFIG}"
        [[ -f "${AIRY_FASTRTPS_PROFILE}" ]] || \
            fail "未找到 Airy Fast DDS 配置文件: ${AIRY_FASTRTPS_PROFILE}"
        python3 - "${AIRY_CONFIG}" <<'PY'
import sys
import yaml

path = sys.argv[1]
with open(path, encoding='utf-8') as stream:
    config = yaml.safe_load(stream)
if config.get('common', {}).get('msg_source') != 2:
    raise SystemExit(f'{path}: common.msg_source 必须设为 2（从 ROS 2 packet 解包）')
if not config.get('common', {}).get('send_point_cloud_ros'):
    raise SystemExit(f'{path}: common.send_point_cloud_ros 必须设为 true')
lidars = config.get('lidar', [])
if not lidars or lidars[0].get('ros', {}).get('ros_recv_packet_topic') != '/rslidar_packets':
    raise SystemExit(f'{path}: ros_recv_packet_topic 必须为 /rslidar_packets')
PY
        ;;
    mid360)
        FASTLIO_CONFIG="mapping_mid360s.yaml"
        ROBOT_PARAM_FILE="${WORKSPACE_DIR}/src/config/P2.5_mid360s.yaml"
        PGO_CONFIG="pgo_mid360s.yaml"
        BAG_TOPICS=(/livox/lidar /livox/imu)
        ;;
    *)
        fail "不支持的雷达类型: ${LIDAR_TYPE}，请选择 airy 或 mid360"
        ;;
esac

python3 - "${BAG_DIR}/metadata.yaml" "${BAG_TOPICS[@]}" <<'PY'
import sys
import yaml

with open(sys.argv[1], encoding='utf-8') as stream:
    info = yaml.safe_load(stream)['rosbag2_bagfile_information']
available = {
    item['topic_metadata']['name']
    for item in info.get('topics_with_message_count', [])
}
missing = [topic for topic in sys.argv[2:] if topic not in available]
if missing:
    raise SystemExit('bag 中缺少话题: ' + ', '.join(missing))
PY

mkdir -p "${LOG_ROOT}" "${MAP_DIR}"

set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

if [[ "${LIDAR_TYPE}" == "airy" ]]; then
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export FASTRTPS_DEFAULT_PROFILES_FILE="${AIRY_FASTRTPS_PROFILE}"
    export RMW_FASTRTPS_USE_QOS_FROM_XML=1
fi

log_time() {
    date +%Y%m%d_%H%M%S
}

RUN_TIME="$(log_time)"
FASTLIO_LOG_FILE="${LOG_ROOT}/fast_lio_${RUN_TIME}.log"
PGO_LOG_FILE="${LOG_ROOT}/pgo_sc_${RUN_TIME}.log"
DRIVER_LOG_FILE="${LOG_ROOT}/rslidar_driver_${RUN_TIME}.log"
BAG_LOG_FILE="${LOG_ROOT}/bag_play_${RUN_TIME}.log"
SAVE_LOG_FILE="${LOG_ROOT}/map_save_${RUN_TIME}.log"

FASTLIO_PID=""
PGO_PID=""
DRIVER_PID=""
BAG_PID=""
SAVE_WAIT_PID=""
MAP_SAVED=0
CLEANING=0
BAG_PAUSED=0
KEYBOARD_CONTROL=0

kill_process_tree_safe() {
    local pid="$1"
    local name="$2"

    [[ -n "${pid}" ]] || return
    kill -0 "${pid}" 2>/dev/null || return

    info "停止 ${name}"
    local pgid
    pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d ' ' || true)"

    if [[ -n "${pgid}" ]]; then
        kill -SIGINT -- "-${pgid}" 2>/dev/null || true
    else
        kill -SIGINT "${pid}" 2>/dev/null || true
    fi

    for _ in 1 2 3 4 5; do
        kill -0 "${pid}" 2>/dev/null || return
        sleep 1
    done

    warn "${name} 未正常退出，发送 SIGTERM"
    if [[ -n "${pgid}" ]]; then
        kill -SIGTERM -- "-${pgid}" 2>/dev/null || true
    else
        kill -SIGTERM "${pid}" 2>/dev/null || true
    fi
}

kill_all_related_processes() {
    info "清理 ROS 2 建图相关残留进程..."
    local patterns=(
        "fastlio_mapping_node"
        "imu_filter_madgwick_node"
        "pgo_sc_node"
        "rosbag2_player"
        "ros2 bag play"
        "rslidar_sdk_node"
        "rviz2"
    )

    local pattern
    for pattern in "${patterns[@]}"; do
        pkill -SIGINT -f "${pattern}" 2>/dev/null || true
    done
    sleep 1
    for pattern in "${patterns[@]}"; do
        pkill -SIGTERM -f "${pattern}" 2>/dev/null || true
    done
    info "残留进程清理完成"
}

save_mapping_result() {
    [[ "${MAP_SAVED}" -eq 0 ]] || return 0
    [[ -n "${PGO_PID}" ]] && kill -0 "${PGO_PID}" 2>/dev/null || return 1

    info "保存 PGO 地图到 ${MAP_DIR} ..."
    setsid ros2 topic echo /save_done std_msgs/msg/Empty --once \
        --qos-durability transient_local >"${SAVE_LOG_FILE}" 2>&1 &
    SAVE_WAIT_PID=$!
    sleep 1

    if ! timeout 10s ros2 topic pub --once /save_dir std_msgs/msg/Empty '{}' \
        >>"${SAVE_LOG_FILE}" 2>&1; then
        kill -TERM -- "-${SAVE_WAIT_PID}" 2>/dev/null || true
        wait "${SAVE_WAIT_PID}" 2>/dev/null || true
        SAVE_WAIT_PID=""
        err "地图保存命令发送失败，日志: ${SAVE_LOG_FILE}"
        return 1
    fi

    local waited=0
    while kill -0 "${SAVE_WAIT_PID}" 2>/dev/null; do
        if [[ "${waited}" -ge 300 ]]; then
            kill -TERM -- "-${SAVE_WAIT_PID}" 2>/dev/null || true
            wait "${SAVE_WAIT_PID}" 2>/dev/null || true
            SAVE_WAIT_PID=""
            err "等待地图保存完成超时，日志: ${SAVE_LOG_FILE}"
            return 1
        fi
        sleep 1
        waited=$((waited + 1))
    done

    if ! wait "${SAVE_WAIT_PID}"; then
        SAVE_WAIT_PID=""
        err "地图保存失败，日志: ${SAVE_LOG_FILE}"
        return 1
    fi
    SAVE_WAIT_PID=""
    [[ -s "${MAP_DIR}/globalmap.pcd" ]] || {
        err "PGO 已返回保存完成，但未生成 ${MAP_DIR}/globalmap.pcd"
        return 1
    }
    MAP_SAVED=1
    info "地图保存完成: ${MAP_DIR}/globalmap.pcd"
}

cleanup() {
    local status=$?
    if [[ "${CLEANING}" -eq 1 ]]; then
        return
    fi

    CLEANING=1
    trap - SIGINT SIGTERM EXIT

    echo
    info "正在关闭 bag、雷达驱动和建图节点..."
    kill_process_tree_safe "${BAG_PID}" "rosbag2 player"
    if [[ "${MAP_SAVED}" -eq 0 && -n "${PGO_PID}" ]] && \
       kill -0 "${PGO_PID}" 2>/dev/null; then
        save_mapping_result || warn "未能保存建图结果"
    fi
    kill_process_tree_safe "${PGO_PID}" "PGO SC"
    kill_process_tree_safe "${FASTLIO_PID}" "FAST-LIO"
    kill_process_tree_safe "${DRIVER_PID}" "Airy 驱动"
    wait 2>/dev/null || true

    echo
    info "已退出"
    info "地图目录: ${MAP_DIR}"
    info "FAST-LIO 日志: ${FASTLIO_LOG_FILE}"
    info "PGO SC 日志: ${PGO_LOG_FILE}"
    info "地图保存日志: ${SAVE_LOG_FILE}"
    [[ "${LIDAR_TYPE}" != "airy" ]] || info "Airy 驱动日志: ${DRIVER_LOG_FILE}"
    info "bag 回放日志: ${BAG_LOG_FILE}"
    exit "${status}"
}

trap cleanup SIGINT SIGTERM EXIT

start_bg() {
    local pid_var="$1"
    local name="$2"
    local log_file="$3"
    local wait_sec="$4"
    shift 4

    info "启动 ${name}..."
    setsid "$@" >"${log_file}" 2>&1 &
    local pid=$!
    printf -v "${pid_var}" '%s' "${pid}"

    sleep "${wait_sec}"
    if ! kill -0 "${pid}" 2>/dev/null; then
        err "${name} 启动失败，日志: ${log_file}"
        tail -n 80 "${log_file}" || true
        exit 1
    fi
    info "${name} 启动成功，日志: ${log_file}"
}

toggle_bag_pause() {
    if [[ -z "${BAG_PID}" ]] || ! kill -0 "${BAG_PID}" 2>/dev/null; then
        warn "bag 已结束，无法暂停或继续"
        return
    fi

    if timeout 5s ros2 service call \
        /rosbag2_player/toggle_paused \
        rosbag2_interfaces/srv/TogglePaused '{}' \
        >>"${BAG_LOG_FILE}" 2>&1; then
        BAG_PAUSED=$((1 - BAG_PAUSED))
        if [[ "${BAG_PAUSED}" -eq 1 ]]; then
            info "bag 回放已暂停；再按空格继续"
        else
            info "bag 回放已继续"
        fi
    else
        warn "bag 暂停/继续命令失败，日志: ${BAG_LOG_FILE}"
    fi
}

handle_keyboard_input() {
    local key=""

    if [[ "${KEYBOARD_CONTROL}" -eq 1 ]]; then
        if IFS= read -r -s -n 1 -t 0.2 key; then
            if [[ "${key}" == " " ]]; then
                toggle_bag_pause
            fi
        fi
    else
        sleep 0.2
    fi
}

kill_all_related_processes

info "开始启动离线建图"
info "雷达类型: ${LIDAR_TYPE}"
info "FAST-LIO 配置: ${FASTLIO_CONFIG}"
info "PGO 配置: ${PGO_CONFIG}"
info "数据包: ${BAG_DIR}"
info "回放倍速: ${PLAY_RATE}"
info "数据目录: ${DATA_DIR}"
info "地图目录: ${MAP_DIR}"

if [[ "${LIDAR_TYPE}" == "airy" ]]; then
    info "Airy 配置: ${AIRY_CONFIG}"
    info "RMW 实现: ${RMW_IMPLEMENTATION}"
    info "Fast DDS 配置: ${FASTRTPS_DEFAULT_PROFILES_FILE}"
    info "使用 XML QoS: ${RMW_FASTRTPS_USE_QOS_FROM_XML}"
    start_bg DRIVER_PID "Airy packet 解包驱动" "${DRIVER_LOG_FILE}" 2 \
        ros2 run rslidar_sdk rslidar_sdk_node --ros-args \
        -p "config_path:=${AIRY_CONFIG}"
fi

start_bg FASTLIO_PID "FAST-LIO" "${FASTLIO_LOG_FILE}" 3 \
    ros2 launch fast_lio mapping.launch.py \
    "config_file:=${FASTLIO_CONFIG}" \
    "robot_param_file:=${ROBOT_PARAM_FILE}" \
    fast_lio_use_rviz:=false use_sim_time:=true

start_bg PGO_PID "PGO SC" "${PGO_LOG_FILE}" 2 \
    ros2 launch pgo_sc run.launch.py "pgo_use_rviz:=${PGO_USE_RVIZ}" use_sim_time:=true \
    "data_dir:=${DATA_DIR}" "config_file:=${PGO_CONFIG}"

start_bg BAG_PID "ROS 2 bag 回放" "${BAG_LOG_FILE}" 1 \
    ros2 bag play "${BAG_DIR}" --rate "${PLAY_RATE}" --clock \
    --disable-keyboard-controls --topics "${BAG_TOPICS[@]}"

if [[ -t 0 ]]; then
    KEYBOARD_CONTROL=1
    info "按空格可暂停/继续 bag 回放"
else
    warn "当前不是交互式终端，空格暂停功能不可用"
fi
info "离线建图已启动；bag 播放完毕后自动保存地图，按 Ctrl+C 退出"
info "可在另一终端检查: ros2 topic hz /cloud_registered"

while true; do
    handle_keyboard_input

    if [[ -n "${BAG_PID}" ]] && ! kill -0 "${BAG_PID}" 2>/dev/null; then
        set +e
        wait "${BAG_PID}"
        bag_status=$?
        set -e
        BAG_PID=""
        if [[ "${bag_status}" -eq 0 ]]; then
            if [[ -n "${PGO_PID}" ]]; then
                info "bag 回放完成，开始保存建图结果"
                save_mapping_result
            else
                info "bag 回放完成；PGO SC 已关闭，跳过 PGO 地图保存"
            fi
            info "bag 回放已结束，建图节点和 RViz 继续运行；按 Ctrl+C 退出"
            continue
        fi
        err "bag 回放异常退出，日志: ${BAG_LOG_FILE}"
        tail -n 80 "${BAG_LOG_FILE}" || true
        exit "${bag_status}"
    fi

    if [[ -n "${FASTLIO_PID}" ]] && ! kill -0 "${FASTLIO_PID}" 2>/dev/null; then
        err "FAST-LIO 进程异常退出，日志: ${FASTLIO_LOG_FILE}"
        tail -n 80 "${FASTLIO_LOG_FILE}" || true
        exit 1
    fi

    if [[ -n "${PGO_PID}" ]] && ! kill -0 "${PGO_PID}" 2>/dev/null; then
        err "PGO SC 进程异常退出，日志: ${PGO_LOG_FILE}"
        tail -n 80 "${PGO_LOG_FILE}" || true
        exit 1
    fi

    if [[ -n "${DRIVER_PID}" ]] && ! kill -0 "${DRIVER_PID}" 2>/dev/null; then
        err "Airy 驱动异常退出，日志: ${DRIVER_LOG_FILE}"
        tail -n 80 "${DRIVER_LOG_FILE}" || true
        exit 1
    fi
done
