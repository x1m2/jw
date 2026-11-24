#!/usr/bin/env bash
set -eo pipefail

OFFLINE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
WORKSPACE_DIR="$(cd "$OFFLINE_DIR/.." && pwd -P)"
source "$OFFLINE_DIR/scripts/runtime.sh"
src="$WORKSPACE_DIR/src"
bag=""
delta=""
rviz=true
prepare_only=false

usage() {
    printf '用法: bash %s --bag <双 Airy bag 目录> [--delta 时差秒数] [--no-rviz] [--prepare-only]\n' "$0"
}

fail() { printf '[ERROR] %s\n' "$*" >&2; exit 1; }

while (( $# )); do
    case "$1" in
        --bag)
            [[ $# -ge 2 ]] || fail "--bag 后需要填写 bag 目录"
            bag="$2"
            shift 2
            ;;
        --delta)
            [[ $# -ge 2 && -n "$2" ]] || fail "--delta 后需要填写已测得的时差（秒）"
            delta="$2"
            shift 2
            ;;
        --no-rviz) rviz=false; shift ;;
        --prepare-only) prepare_only=true; shift ;;
        -h|--help) usage; exit 0 ;;
        *) usage >&2; fail "未知参数 $1" ;;
    esac
done

[[ -n "$bag" && -f "$bag/metadata.yaml" ]] || fail "请指定含有 metadata.yaml 的双 Airy bag"
bag=$(realpath -- "$bag")
fusion_node="$WORKSPACE_DIR/install/dual_airy_fusion/lib/dual_airy_fusion/dual_airy_fusion_node"
fusion_args=(-p startup_timeout_s:=120.0)
if [[ -n "$delta" ]]; then
    if ! delta=$(python3 - "$delta" <<'PY'
import math
import sys

try:
    value = float(sys.argv[1])
except ValueError:
    sys.exit(1)
if not math.isfinite(value):
    sys.exit(1)
print(repr(value))
PY
    ); then
        fail "--delta 必须是有限的秒数"
    fi
    fusion_args=(-p auto_time:=false -p "delta_s:=$delta")
fi
if ! $prepare_only; then
    [[ -x "$fusion_node" ]] || fail "找不到支持 ROS 参数文件的融合程序 $fusion_node"
    [[ -r "$WORKSPACE_DIR/install/setup.bash" ]] || fail "找不到 $WORKSPACE_DIR/install/setup.bash"
fi

offline_new_run airy_dual
run="$DATA_DIR"

PYTHONPATH="$OFFLINE_DIR/scripts${PYTHONPATH:+:$PYTHONPATH}" \
    python3 - "$bag" "$src" "$run" <<'PY'
from contextlib import closing
from pathlib import Path
import sqlite3
import sys
import tempfile

import yaml

from prepare_airy_replay import header_nanoseconds, rewrite_msop_timestamp

bag, src, run = (Path(arg).resolve() for arg in sys.argv[1:])
metadata = yaml.safe_load((bag / "metadata.yaml").read_text())
info = metadata["rosbag2_bagfile_information"]
if info["storage_identifier"] != "sqlite3" or info.get("compression_format"):
    raise SystemExit("双 Airy 回放需要未压缩的 SQLite ROS 2 bag")
packet_topics = ("/rslidar_front/packets", "/rslidar_rear/packets")
imu_topics = ("/rslidar_front/imu", "/rslidar_rear/imu")
available = {entry["topic_metadata"]["name"]: entry for entry in info["topics_with_message_count"]}
for topics, message_type in ((packet_topics, "rslidar_msg/msg/RslidarPacket"), (imu_topics, "sensor_msgs/msg/Imu")):
    for name in topics:
        entry = available.get(name)
        if not entry or not entry["message_count"] or entry["topic_metadata"]["type"] != message_type:
            raise SystemExit(f"bag 中缺少有效的 {name} ({message_type})")

driver = yaml.safe_load((src / "drivers/airy96_ros_driver2/rslidar_sdk/config/airy96_driver.yaml").read_text())
required_streams = {(f"/rslidar_{side}/packets", f"/rslidar_{side}/points", "RSAIRY") for side in ("front", "rear")}
streams = [(lidar["ros"]["ros_recv_packet_topic"], lidar["ros"]["ros_send_point_cloud_topic"],
            lidar["driver"]["lidar_type"]) for lidar in driver["lidar"]]
if len(streams) != 2 or set(streams) != required_streams:
    raise SystemExit("驱动模板需要包含与融合节点对应的前后 RSAIRY 话题")
driver["common"].update(msg_source=2, send_packet_ros=False, send_point_cloud_ros=True)
for lidar in driver["lidar"]:
    lidar["driver"]["use_lidar_clock"] = True

mapping = yaml.safe_load((src / "slam/mapping/fast_lio/config/mapping_airy96.yaml").read_text())
mapping["/**"]["ros__parameters"]["common"].update(
    lid_topic="/dual_airy/points_merged", imu_topic=imu_topics[0]
)

rewritten = dict.fromkeys(packet_topics, 0)
files = []
print("[INFO] 准备双 Airy 回放副本，统一使用录制的 Header.stamp", flush=True)
with tempfile.TemporaryDirectory(dir=run, prefix=".replay-") as staging:
    staged = Path(staging)
    for filename in info["relative_file_paths"]:
        relative = Path(filename)
        if relative.is_absolute() or ".." in relative.parts:
            raise SystemExit(f"无效 bag 数据文件路径 {filename}")
        destination = staged / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        source_uri = (bag / relative).as_uri() + "?mode=ro"
        with closing(sqlite3.connect(source_uri, uri=True)) as original, closing(sqlite3.connect(destination)) as replica:
            original.backup(replica)
            for topic_id, name in replica.execute("SELECT id,name FROM topics WHERE name IN (?,?,?,?)", packet_topics + imu_topics):
                cursor = replica.execute("SELECT id,timestamp,data FROM messages WHERE topic_id=? ORDER BY id", (topic_id,))
                while rows := cursor.fetchmany(1000):
                    updates = []
                    for message_id, record_stamp, data in rows:
                        stamp = header_nanoseconds(data) or record_stamp
                        if name in rewritten:
                            data, changed = rewrite_msop_timestamp(data)
                            rewritten[name] += changed
                        updates.append((stamp, data, message_id))
                    replica.executemany("UPDATE messages SET timestamp=?,data=? WHERE id=?", updates)
            replica.commit()
            first, last, count = replica.execute("SELECT MIN(timestamp),MAX(timestamp),COUNT(*) FROM messages").fetchone()
            if not count:
                raise SystemExit(f"bag 数据库为空 {filename}")
            files.append({"path": filename, "starting_time": {"nanoseconds_since_epoch": first},
                          "duration": {"nanoseconds": last - first}, "message_count": count})
    if not all(rewritten.values()):
        raise SystemExit(f"前后雷达都需要可解码的 MSOP 包，实际数量 {rewritten}")
    start = min(item["starting_time"]["nanoseconds_since_epoch"] for item in files)
    end = max(item["starting_time"]["nanoseconds_since_epoch"] + item["duration"]["nanoseconds"] for item in files)
    info.update(files=files, starting_time={"nanoseconds_since_epoch": start}, duration={"nanoseconds": end - start})
    (staged / "metadata.yaml").write_text(yaml.safe_dump(metadata, sort_keys=False))
    staged.rename(run / "bag")

(run / "dual_decoder.yaml").write_text(yaml.safe_dump(driver, sort_keys=False))
(run / "mapping_dual.yaml").write_text(yaml.safe_dump(mapping, sort_keys=False))
print(f"[INFO] 已准备 MSOP 包 {rewritten}", flush=True)
PY

if $prepare_only; then
    printf '[INFO] 双路回放准备完成 %s/bag\n' "$run"
    exit 0
fi

source /opt/ros/humble/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"
offline_redirect_logs
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-90}" ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp RMW_FASTRTPS_USE_QOS_FROM_XML=1
unset FASTDDS_DEFAULT_PROFILES_FILE
export FASTRTPS_DEFAULT_PROFILES_FILE="$src/drivers/airy96_ros_driver2/rslidar_sdk/config/fastshm.xml"
export LD_LIBRARY_PATH="/home/jw/slam_ws/dependencies/install/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export ROS_LOG_DIR="$run/log/ros"
mkdir -p "$run/map" "$run/log"

pids=()
cleanup() {
    local pid attempt alive
    for pid in "${pids[@]}"; do
        kill -INT -- "-$pid" 2>/dev/null || true
    done
    for attempt in 1 2 3; do
        alive=false
        for pid in "${pids[@]}"; do
            if kill -0 -- "-$pid" 2>/dev/null; then
                alive=true
            fi
        done
        $alive || return 0
        sleep 1
    done
    for pid in "${pids[@]}"; do
        kill -TERM -- "-$pid" 2>/dev/null || true
    done
}
trap cleanup EXIT
trap 'exit 130' INT TERM

start() {
    local log=$1
    shift
    setsid "$@" >"$run/log/$log.log" 2>&1 &
    pids+=("$!")
}

start fusion "$fusion_node" --ros-args \
    --params-file "$src/drivers/dual_airy_fusion/config/dual_airy_fusion.yaml" \
    "${fusion_args[@]}" --log-level dual_airy_fusion_node:=debug
timeout 10s ros2 topic echo /dual_airy/status std_msgs/msg/String --once >/dev/null || \
    fail "融合节点启动失败，查看 $run/log/fusion.log"

start decoder ros2 run rslidar_sdk rslidar_sdk_node --ros-args -p "config_path:=$run/dual_decoder.yaml"
start fast_lio env LD_PRELOAD="${AIRY_FILE_REDIRECT}${LD_PRELOAD:+:${LD_PRELOAD}}" ros2 launch fast_lio mapping.launch.py \
    "config_path:=$run" config_file:=mapping_dual.yaml \
    "robot_param_file:=$src/config/P3_airy96.yaml" fast_lio_use_rviz:=false use_sim_time:=true
start pgo env LD_PRELOAD="${AIRY_FILE_REDIRECT}${LD_PRELOAD:+:${LD_PRELOAD}}" ros2 launch pgo_sc run.launch.py \
    "data_dir:=$run" config_file:=pgo_airy96.yaml \
    "pgo_use_rviz:=$rviz" use_sim_time:=true

sleep 5
for pid in "${pids[@]}"; do
    kill -0 "$pid" 2>/dev/null || fail "节点启动期间退出，查看 $run/log"
done
if [[ -n "$delta" ]]; then
    printf '[INFO] 开始双雷达回放，使用固定时差 %s s。\n' "$delta"
else
    printf '[INFO] 开始双雷达回放。融合节点检查时差后才会发布建图点云。\n'
fi
start bag ros2 bag play "$run/bag" --rate 1.0 --clock --disable-keyboard-controls \
    --topics /rslidar_front/packets /rslidar_rear/packets /rslidar_front/imu /rslidar_rear/imu
bag_pid=${pids[-1]}
timeout 125s ros2 topic echo /dual_airy/points_merged sensor_msgs/msg/PointCloud2 --field header --once >/dev/null || \
    fail "未收到融合点云，查看 $run/log/fusion.log 和 $run/log/decoder.log"
printf '[INFO] 已收到双雷达融合点云\n'
wait "$bag_pid" || fail "bag 回放异常，查看 $run/log/bag.log"
unset 'pids[-1]'

sleep 2
timeout 10s ros2 topic pub --once /save_dir std_msgs/msg/Empty '{}' >/dev/null || \
    fail "发送保存指令失败，查看 $run/log/pgo.log"
timeout 300s ros2 topic echo /save_done std_msgs/msg/Empty --once --qos-durability transient_local >/dev/null || \
    fail "地图保存未完成，查看 $run/log/pgo.log"
[[ -s "$run/map/globalmap.pcd" ]] || fail "未生成地图 $run/map/globalmap.pcd"
printf '[INFO] 地图已保存到 %s/map/globalmap.pcd，按 Ctrl+C 退出\n' "$run"
wait
