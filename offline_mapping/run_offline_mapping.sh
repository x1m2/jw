#!/usr/bin/env bash
set -euo pipefail

OFFLINE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
WORKSPACE_DIR="$(cd "$OFFLINE_DIR/.." && pwd -P)"
source "$OFFLINE_DIR/scripts/runtime.sh"
FASTLIO_CONFIG_DIR="$WORKSPACE_DIR/src/slam/mapping/fast_lio/config"
BAG_DIR=""
LIDAR_TYPE="${LIDAR_TYPE:-airy}"
PLAY_RATE="${PLAY_RATE:-1.0}"
AIRY_CONFIG="${AIRY_CONFIG:-${WORKSPACE_DIR}/src/drivers/airy96_ros_driver2/rslidar_sdk/config/airy96_driver.yaml}"
AIRY_FASTRTPS_PROFILE="${AIRY_FASTRTPS_PROFILE:-${WORKSPACE_DIR}/src/drivers/airy96_ros_driver2/rslidar_sdk/config/fastshm.xml}"
PGO_USE_RVIZ="${PGO_USE_RVIZ:-true}"
PREPARE_ONLY=0
BODY_IMU=0
BAG_REMAPS=()

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
  --body-imu             使用同一安装的 Woodox 本体 IMU /imu/data
  --no-rviz              不启动 PGO RViz
  --prepare-only         只准备并检查离线配置，不启动 ROS 节点
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
        --body-imu)
            BODY_IMU=1
            shift
            ;;
        --no-rviz)
            PGO_USE_RVIZ=false
            shift
            ;;
        --prepare-only)
            PREPARE_ONLY=1
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

[[ "$LIDAR_TYPE" == airy || "$LIDAR_TYPE" == mid360 ]] || fail "不支持的雷达类型: $LIDAR_TYPE"
run_prefix="$LIDAR_TYPE"
if [[ "$BODY_IMU" -eq 1 ]]; then
    [[ "$LIDAR_TYPE" == airy ]] || fail "--body-imu 仅用于 Airy"
    run_prefix=airy_body_imu
fi
if [[ "$LIDAR_TYPE" == airy ]]; then
    [[ -f "$AIRY_CONFIG" ]] || fail "未找到 Airy 配置文件: $AIRY_CONFIG"
    AIRY_CONFIG=$(realpath -- "$AIRY_CONFIG")
    [[ -f "$AIRY_FASTRTPS_PROFILE" ]] || fail "未找到 Airy Fast DDS 配置文件: $AIRY_FASTRTPS_PROFILE"
    AIRY_FASTRTPS_PROFILE=$(realpath -- "$AIRY_FASTRTPS_PROFILE")
fi
offline_new_run "$run_prefix"
MAP_DIR="$DATA_DIR/map"

case "${LIDAR_TYPE}" in
    airy)
        FASTLIO_CONFIG="mapping_airy96.yaml"
        ROBOT_PARAM_FILE="${WORKSPACE_DIR}/src/config/p3_airy96.yaml"
        [[ -f "${ROBOT_PARAM_FILE}" ]] || \
            ROBOT_PARAM_FILE="${WORKSPACE_DIR}/src/config/P3_airy96.yaml"
        PGO_CONFIG="pgo_airy96.yaml"
        # Packet 解码为点云；Airy 的 IMU 已作为独立话题录入 bag，
        # rslidar_sdk 从 packet 回放时不会重新发布这一路 IMU。
        replay_args=()
        if [[ "$BODY_IMU" -eq 1 ]]; then
            python3 - "$FASTLIO_CONFIG_DIR/$FASTLIO_CONFIG" "$OFFLINE_DIR/config/airy_woodox.yaml" \
                "$DATA_DIR/$FASTLIO_CONFIG" "${AIRY_IMU_TIME_OFFSET:-0.003}" <<'PY'
import math
from pathlib import Path
import sys

import yaml

source, calibration, output = map(Path, sys.argv[1:4])
offset = float(sys.argv[4])
if not math.isfinite(offset):
    raise SystemExit("AIRY_IMU_TIME_OFFSET 必须是有限的秒数")
config = yaml.safe_load(source.read_text())
external = yaml.safe_load(calibration.read_text())
params = config["/**"]["ros__parameters"]
params["common"].update(imu_topic="/imu/data", time_sync_en=False, time_offset_lidar_to_imu=offset)
params["mapping"].update(external)
output.write_text(yaml.safe_dump(config, sort_keys=False))
print(f"[INFO] 本体 IMU 时间补偿 {offset:+.3f} s")
PY
            FASTLIO_CONFIG_DIR="$DATA_DIR"
            replay_args+=(--filter-imu-clock)
        fi
        # 生成专用的离线配置和 bag 副本，不修改在线驱动配置或原始数据。
        mkdir -p "${LOG_ROOT}/airy_replay"
        REPLAY_ENV="${LOG_ROOT}/airy_replay/replay.env"
        python3 "${OFFLINE_DIR}/scripts/prepare_airy_replay.py" \
            --bag "${BAG_DIR}" --driver-config "${AIRY_CONFIG}" \
            --fastlio-config "${FASTLIO_CONFIG_DIR}/${FASTLIO_CONFIG}" \
            --output-dir "${LOG_ROOT}/airy_replay" --env-file "${REPLAY_ENV}" "${replay_args[@]}"
        source "${REPLAY_ENV}"
        ;;
    mid360)
        FASTLIO_CONFIG="mapping_mid360s.yaml"
        ROBOT_PARAM_FILE="${WORKSPACE_DIR}/src/config/p2.5_mid360s.yaml"
        [[ -f "${ROBOT_PARAM_FILE}" ]] || \
            ROBOT_PARAM_FILE="${WORKSPACE_DIR}/src/config/P2.5_mid360s.yaml"
        PGO_CONFIG="pgo_mid360s.yaml"
        BAG_TOPICS=(/livox/lidar /livox/imu)
        ;;
    *)
        fail "不支持的雷达类型: ${LIDAR_TYPE}，请选择 airy 或 mid360"
        ;;
esac

[[ -f "${ROBOT_PARAM_FILE}" ]] || fail "未找到机器人配置: ${ROBOT_PARAM_FILE}"

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

if [[ "${PREPARE_ONLY}" -eq 1 ]]; then
    info "离线准备检查通过，未启动 ROS 节点"
    info "回放数据: ${BAG_DIR}"
    info "回放话题: ${BAG_TOPICS[*]}"
    [[ "${LIDAR_TYPE}" != "airy" ]] || info "离线驱动配置: ${AIRY_CONFIG}"
    exit 0
fi

mkdir -p "${LOG_ROOT}" "${MAP_DIR}"
offline_redirect_logs

set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

# GTSAM 的私有依赖（例如 libmetis-gtsam）需要进入动态库搜索路径。
# 使用本工作空间实际编译时的前缀，也允许通过环境变量显式指定。
GTSAM_LIBRARY_DIR="${GTSAM_LIBRARY_DIR:-}"
if [[ -z "${GTSAM_LIBRARY_DIR}" && -f "${WORKSPACE_DIR}/build/pgo_sc/CMakeCache.txt" ]]; then
    GTSAM_CONFIG_DIR="$(sed -n 's/^GTSAM_DIR:[^=]*=//p' "${WORKSPACE_DIR}/build/pgo_sc/CMakeCache.txt")"
    if [[ -n "${GTSAM_CONFIG_DIR}" && -d "${GTSAM_CONFIG_DIR}/../.." ]]; then
        GTSAM_LIBRARY_DIR="$(cd "${GTSAM_CONFIG_DIR}/../.." && pwd -P)"
    fi
fi
if [[ -n "${GTSAM_LIBRARY_DIR}" ]]; then
    export LD_LIBRARY_PATH="${GTSAM_LIBRARY_DIR}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi

for package_executable in \
    "rslidar_sdk:rslidar_sdk_node" "fast_lio:fastlio_mapping_node" "pgo_sc:pgo_sc_node"; do
    [[ "${LIDAR_TYPE}" == "airy" || "${package_executable}" != rslidar_sdk:* ]] || continue
    package="${package_executable%%:*}"
    executable="${package_executable#*:}"
    package_prefix="$(ros2 pkg prefix "${package}" 2>/dev/null)" || \
        fail "未安装 ${package}；请在 ${WORKSPACE_DIR} 执行 colcon build --symlink-install --packages-up-to rslidar_sdk fast_lio pgo_sc"
    [[ -x "${package_prefix}/lib/${package}/${executable}" ]] || \
        fail "未找到可执行程序 ${package}/${executable}，请先编译工作空间"
done

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-91}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"

if [[ "${LIDAR_TYPE}" == "airy" ]]; then
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    unset FASTDDS_DEFAULT_PROFILES_FILE
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

    [[ -n "${pid}" ]] || return 0
    kill -0 "${pid}" 2>/dev/null || return 0

    info "停止 ${name}"
    local pgid
    pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d ' ' || true)"

    if [[ -n "${pgid}" ]]; then
        kill -SIGINT -- "-${pgid}" 2>/dev/null || true
    else
        kill -SIGINT "${pid}" 2>/dev/null || true
    fi

    for _ in 1 2 3 4 5; do
        kill -0 "${pid}" 2>/dev/null || return 0
        sleep 1
    done

    warn "${name} 未正常退出，发送 SIGTERM"
    if [[ -n "${pgid}" ]]; then
        kill -SIGTERM -- "-${pgid}" 2>/dev/null || true
    else
        kill -SIGTERM "${pid}" 2>/dev/null || true
    fi
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
    env LD_PRELOAD="${AIRY_FILE_REDIRECT}${LD_PRELOAD:+:${LD_PRELOAD}}" ros2 launch fast_lio mapping.launch.py \
    "config_path:=${FASTLIO_CONFIG_DIR}" \
    "config_file:=${FASTLIO_CONFIG}" \
    "robot_param_file:=${ROBOT_PARAM_FILE}" \
    fast_lio_use_rviz:=false use_sim_time:=true

start_bg PGO_PID "PGO SC" "${PGO_LOG_FILE}" 2 \
    env LD_PRELOAD="${AIRY_FILE_REDIRECT}${LD_PRELOAD:+:${LD_PRELOAD}}" ros2 launch pgo_sc run.launch.py \
    "pgo_use_rviz:=${PGO_USE_RVIZ}" use_sim_time:=true \
    "data_dir:=${DATA_DIR}" "config_file:=${PGO_CONFIG}"

start_bg BAG_PID "ROS 2 bag 回放" "${BAG_LOG_FILE}" 1 \
    ros2 bag play "${BAG_DIR}" --rate "${PLAY_RATE}" --clock \
    --disable-keyboard-controls --topics "${BAG_TOPICS[@]}" "${BAG_REMAPS[@]}"

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
