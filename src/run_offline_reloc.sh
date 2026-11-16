#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
MAP_DIR="${MAP_DIR:-${WORKSPACE_DIR}/map}"
BAG_DIR=''
SAVER="${SAVER:-${WORKSPACE_DIR}/install/bbs3d/bin/voxelmaps_saver}"
ROBOT_PARAM_FILE="${ROBOT_PARAM_FILE:-}"
LOG_ROOT="${LOG_ROOT:-${WORKSPACE_DIR}/log/offline_reloc}"
SCAN_INDEX="${SCAN_INDEX:-100}"
PLAY_RATE="${PLAY_RATE:-0.5}"
RELOC_TIMEOUT_SEC="${RELOC_TIMEOUT_SEC:-300}"
LIDAR_TYPE="${LIDAR_TYPE:-airy}"
PREPARE_ONLY=false

info() { printf '[INFO] %s\n' "$*"; }
fail() { printf '[ERROR] %s\n' "$*" >&2; exit 1; }
usage() { printf '用法: %s --bag 数据包目录 [--lidar airy|mid360] [--rate 倍速] [--scan-index 帧号] [--prepare-only]\n' "$0"; }

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
            [[ $# -ge 2 ]] || fail "--rate 后需要填写倍速"
            PLAY_RATE="$2"
            shift 2
            ;;
        --scan-index)
            [[ $# -ge 2 ]] || fail "--scan-index 后需要填写点云帧号"
            SCAN_INDEX="$2"
            shift 2
            ;;
        --prepare-only)
            PREPARE_ONLY=true
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

case "${LIDAR_TYPE}" in
    airy)
        CONFIG_FILE=reloc_airy96.yaml
        ROBOT_PARAM_FILE="${ROBOT_PARAM_FILE:-${WORKSPACE_DIR}/src/config/P3_airy96.yaml}"
        LIDAR_TOPIC=/rslidar_points
        IMU_TOPIC=/rslidar_imu_data
        CLOUD_MSG_TYPE=sensor_msgs/msg/PointCloud2
        ;;
    mid360)
        CONFIG_FILE=reloc_mid360s.yaml
        ROBOT_PARAM_FILE="${ROBOT_PARAM_FILE:-${WORKSPACE_DIR}/src/config/P2.5_mid360s.yaml}"
        LIDAR_TOPIC=/livox/lidar
        IMU_TOPIC=/livox/imu
        CLOUD_MSG_TYPE=livox_ros_driver2/msg/CustomMsg
        ;;
    *) fail "不支持的雷达类型: ${LIDAR_TYPE}，请选择 airy 或 mid360" ;;
esac

[[ -n "${BAG_DIR}" ]] || { usage >&2; fail "必须通过 --bag 指定数据包目录"; }

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

[[ -f "${WORKSPACE_DIR}/install/setup.bash" ]] || fail "未找到 ROS 2 工作空间环境"
[[ -s "${MAP_DIR}/globalmap.pcd" ]] || fail "未找到地图: ${MAP_DIR}/globalmap.pcd"
[[ -f "${BAG_DIR}/metadata.yaml" ]] || fail "未找到 ROS 2 bag: ${BAG_DIR}/metadata.yaml"
BAG_DIR="$(cd "${BAG_DIR}" && pwd -P)"
[[ -x "${SAVER}" ]] || fail "未找到可执行的 3D-BBS saver: ${SAVER}"
[[ -f "${ROBOT_PARAM_FILE}" ]] || fail "未找到机器人标定文件: ${ROBOT_PARAM_FILE}"
[[ "${SCAN_INDEX}" =~ ^[0-9]+$ ]] || fail "SCAN_INDEX 必须是非负整数"

mkdir -p "${LOG_ROOT}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-${LOG_ROOT}/ros}"
mkdir -p "${ROS_LOG_DIR}"
info "离线重定位日志目录: ${LOG_ROOT}"
info "saver 日志: ${LOG_ROOT}/saver.log"
if [[ "${PREPARE_ONLY}" != true ]]; then
    info "定位日志: ${LOG_ROOT}/localization.log"
    info "bag 回放日志: ${LOG_ROOT}/bag_play.log"
fi

set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

# Use the same voxel resolution and levels as both Airy and MID360 ROS 2 configurations.
info "根据 ${MAP_DIR}/globalmap.pcd 生成 BBS3D 体素地图"
"${SAVER}" "${MAP_DIR}" 0.5 6 2>&1 | tee "${LOG_ROOT}/saver.log"
VOXEL_DIR="${MAP_DIR}/voxelmaps_coords"
[[ -s "${VOXEL_DIR}/voxel_params.txt" ]] || fail "saver 未生成 voxel_params.txt"
for level in 0 1 2 3 4 5 6; do
    [[ -s "${VOXEL_DIR}/${level}.pcd" ]] || fail "saver 未生成体素层 ${level}.pcd"
done
info "体素地图已生成: ${VOXEL_DIR}"

# Export one real scan for inspecting the bag input. Localization below uses the
# original LiDAR and IMU messages, preserving per-point timestamps.
SCAN_FILE="${LOG_ROOT}/${LIDAR_TYPE}_scan_${SCAN_INDEX}.pcd"
SCAN_OFFSET_FILE="${LOG_ROOT}/${LIDAR_TYPE}_scan_${SCAN_INDEX}.offset"
python3 - "${BAG_DIR}" "${SCAN_INDEX}" "${SCAN_FILE}" "${SCAN_OFFSET_FILE}" "${LIDAR_TOPIC}" "${CLOUD_MSG_TYPE}" <<'PY'
import math
import pathlib
import sqlite3
import struct
import sys

import yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointField
from rosidl_runtime_py.utilities import get_message

bag_dir = pathlib.Path(sys.argv[1])
index = int(sys.argv[2])
output = pathlib.Path(sys.argv[3])
offset_file = pathlib.Path(sys.argv[4])
lidar_topic = sys.argv[5]
cloud_type = sys.argv[6]
message_class = get_message(cloud_type)
metadata = yaml.safe_load((bag_dir / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
if metadata['storage_identifier'] != 'sqlite3':
    raise SystemExit('只支持 sqlite3 格式的 ROS 2 bag')

cloud = None
recorded_time = None
previous_scan_time = None
for relative_path in metadata['relative_file_paths']:
    db_path = bag_dir / relative_path
    with sqlite3.connect(f'file:{db_path}?mode=ro', uri=True) as db:
        topic = db.execute('SELECT id FROM topics WHERE name=? AND type=?',
                           (lidar_topic, cloud_type)).fetchone()
        if topic is None:
            continue
        topic_id = topic[0]
        count = db.execute('SELECT COUNT(*) FROM messages WHERE topic_id=?',
                           (topic_id,)).fetchone()[0]
        if index >= count:
            index -= count
            if count:
                previous_scan_time = db.execute(
                    'SELECT timestamp FROM messages WHERE topic_id=? '
                    'ORDER BY timestamp DESC LIMIT 1',
                    (topic_id,)).fetchone()[0]
            continue
        if index:
            previous_scan_time = db.execute(
                'SELECT timestamp FROM messages WHERE topic_id=? '
                'ORDER BY timestamp LIMIT 1 OFFSET ?',
                (topic_id, index - 1)).fetchone()[0]
        row = db.execute('SELECT timestamp, data FROM messages WHERE topic_id=? '
                         'ORDER BY timestamp LIMIT 1 OFFSET ?',
                         (topic_id, index)).fetchone()
        recorded_time = row[0]
        cloud = deserialize_message(row[1], message_class)
        break
if cloud is None:
    raise SystemExit('SCAN_INDEX 超出 bag 中的点云帧数')

points = bytearray()
if cloud_type == "livox_ros_driver2/msg/CustomMsg":
    for point in cloud.points:
        if all(math.isfinite(value) for value in (point.x, point.y, point.z)):
            points.extend(struct.pack("<ffff", point.x, point.y, point.z,
                                      float(point.reflectivity)))
else:
    required = {'x': (0, PointField.FLOAT32), 'y': (4, PointField.FLOAT32),
                'z': (8, PointField.FLOAT32), 'intensity': (12, PointField.FLOAT32),
                'ring': (16, PointField.UINT16), 'timestamp': (18, PointField.FLOAT64)}
    fields = {field.name: field for field in cloud.fields}
    for name, (offset, datatype) in required.items():
        field = fields.get(name)
        if field is None or field.offset != offset or field.datatype != datatype:
            raise SystemExit(f'Airy 点云字段不匹配: {name}')
    if cloud.is_bigendian or cloud.point_step < 26:
        raise SystemExit('Airy 点云的字节序或点长不匹配')

    for row in range(cloud.height):
        for column in range(cloud.width):
            offset = row * cloud.row_step + column * cloud.point_step
            xyz = struct.unpack_from('<fff', cloud.data, offset)
            if all(math.isfinite(value) for value in xyz):
                points.extend(cloud.data[offset:offset + 16])
point_count = len(points) // 16
header = (f'# .PCD v0.7 - Point Cloud Data file format\n'
          f'VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\n'
          f'TYPE F F F F\nCOUNT 1 1 1 1\nWIDTH {point_count}\n'
          f'HEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
          f'POINTS {point_count}\nDATA binary\n')
with output.open('wb') as stream:
    stream.write(header.encode('ascii'))
    stream.write(points)
bag_start = metadata['starting_time']['nanoseconds_since_epoch']
# Begin just after the preceding LiDAR scan. This keeps the IMU lead-in
# needed for synchronization while making this the first played LiDAR frame.
if previous_scan_time is None:
    start_time = bag_start
else:
    gap = recorded_time - previous_scan_time
    if gap <= 1_000_000:
        raise SystemExit('相邻点云记录时间间隔过小，无法安全选择首帧')
    start_time = previous_scan_time + min(1_000_000, gap // 4)
start_offset = max(0.0, (start_time - bag_start) / 1e9)
offset_file.write_text(f'{start_offset:.9f}\n')
print(f'已导出点云: {output} ({point_count} 个有效点，frame={cloud.header.frame_id})')
PY
SCAN_START_OFFSET="$(<"${SCAN_OFFSET_FILE}")"

if [[ "${PREPARE_ONLY}" == true ]]; then
    info "准备完成；未启动定位回放"
    exit 0
fi

LOCALIZATION_PID=''
BAG_PID=''
cleanup() {
    local status=$?
    trap - EXIT INT TERM
    for pid in "${BAG_PID}" "${LOCALIZATION_PID}"; do
        if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
            kill -INT -- "-${pid}" 2>/dev/null || true
            wait "${pid}" 2>/dev/null || true
        fi
    done
    exit "${status}"
}
trap cleanup EXIT INT TERM

info "启动 ${LIDAR_TYPE} 全局重定位和连续定位"
setsid ros2 launch fastlio_localization run_launch.py \
    "config_file:=${CONFIG_FILE}" \
    localization_use_rviz:=true \
    "map_path:=${MAP_DIR}" \
    "robot_param_file:=${ROBOT_PARAM_FILE}" \
    > "${LOG_ROOT}/localization.log" 2>&1 &
LOCALIZATION_PID=$!

# The algorithm node appears only after the worker has loaded its map.
ready=false
for _ in $(seq 1 60); do
    kill -0 "${LOCALIZATION_PID}" 2>/dev/null || fail "定位节点已退出，查看 ${LOG_ROOT}/localization.log"
    if ros2 node list 2>/dev/null | grep -qx '/fastlio_localization'; then
        ready=true
        break
    fi
    sleep 1
done
[[ "${ready}" == true ]] || fail "定位节点未就绪，查看 ${LOG_ROOT}/localization.log"

info "暂停启动 bag，选取第 ${SCAN_INDEX} 帧附近的数据触发重定位"
setsid ros2 bag play "${BAG_DIR}" --rate "${PLAY_RATE}" \
    --start-paused --start-offset "${SCAN_START_OFFSET}" \
    --disable-keyboard-controls \
    --topics "${LIDAR_TOPIC}" "${IMU_TOPIC}" \
    > "${LOG_ROOT}/bag_play.log" 2>&1 &
BAG_PID=$!

python3 - "${RELOC_TIMEOUT_SEC}" <<'PY'
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rosbag2_interfaces.srv import Burst, Resume
from std_msgs.msg import Bool

rclpy.init()
node = rclpy.create_node('offline_reloc_playback_controller')
succeeded = False
started = False
manual_pose_version = 0
manual_pose_received_at = 0.0

def on_initial_pose(message):
    global manual_pose_version, manual_pose_received_at
    manual_pose_version += 1
    manual_pose_received_at = time.monotonic()
    print('收到手动初始位姿，准备补送点云和 IMU 触发配准', flush=True)

def on_status(message):
    global succeeded, started
    if message.data:
        succeeded = True
    else:
        started = True

subscription = node.create_subscription(Bool, '/reloc_success', on_status, 10)
initial_pose_subscription = node.create_subscription(
    PoseWithCovarianceStamped, '/initialpose', on_initial_pose, 10)
burst_client = node.create_client(Burst, '/rosbag2_player/burst')
resume_client = node.create_client(Resume, '/rosbag2_player/resume')
deadline = time.monotonic() + float(sys.argv[1])

def await_result(future):
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not future.done() or future.exception() is not None:
        raise RuntimeError('bag 播放控制服务无响应')
    return future.result()

try:
    while not burst_client.wait_for_service(timeout_sec=0.5):
        if time.monotonic() >= deadline:
            raise RuntimeError('bag 播放器未启动')

    # Feed a few synchronized LiDAR/IMU messages, then hold the bag still
    # while the first global matching attempt runs.
    for _ in range(20):
        if started or succeeded:
            break
        response = await_result(burst_client.call_async(Burst.Request(num_messages=100)))
        if response.actually_burst == 0:
            raise RuntimeError('bag 中没有可用于重定位的后续消息')
        until = min(deadline, time.monotonic() + 2.0)
        while not started and not succeeded and time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.2)
    if not started and not succeeded:
        raise RuntimeError('已送入点云和 IMU，但定位节点没有开始重定位')

    print('等待重定位成功，bag 保持暂停；可在 RViz 设置初始位姿', flush=True)
    attempted_pose_version = 0
    manual_bursts_remaining = 0
    next_manual_burst_at = 0.0
    while rclpy.ok() and not succeeded and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if succeeded:
            break
        if manual_pose_version != attempted_pose_version:
            attempted_pose_version = manual_pose_version
            manual_bursts_remaining = 3
            # Allow the localization node to receive the same /initialpose
            # before new sensor data triggers its matching callback.
            next_manual_burst_at = manual_pose_received_at + 0.3
        if manual_bursts_remaining and time.monotonic() >= next_manual_burst_at:
            response = await_result(burst_client.call_async(
                Burst.Request(num_messages=100)))
            if response.actually_burst == 0:
                raise RuntimeError('bag 已结束，无法继续手动重定位；请重新启动脚本')
            manual_bursts_remaining -= 1
            next_manual_burst_at = time.monotonic() + 2.0
            if not manual_bursts_remaining:
                print('已补送配准数据；若仍未成功，可重新设置初始位姿重试', flush=True)
    if not succeeded:
        raise RuntimeError('重定位超时；检查手动初始位姿、SCAN_INDEX 和定位日志')

    while not resume_client.wait_for_service(timeout_sec=0.5):
        if time.monotonic() >= deadline:
            raise RuntimeError('无法连接 bag 恢复服务')
    await_result(resume_client.call_async(Resume.Request()))
    print('全局重定位成功，恢复 bag 回放并继续定位', flush=True)
finally:
    node.destroy_node()
    rclpy.shutdown()
PY

info "已重定位；等待 bag 回放结束。按 Ctrl+C 可提前退出。"
wait "${BAG_PID}"
info "回放完成；日志在 ${LOG_ROOT}"
