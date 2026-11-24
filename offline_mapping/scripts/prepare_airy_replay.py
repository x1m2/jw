#!/usr/bin/env python3
"""Prepare a private RSAIRY packet replay using the recorded ROS header clock.

The SDK's use_lidar_clock=False uses the *current* host clock, even in ROS
packet mode. Its True mode uses the timestamp embedded in each MSOP packet.
For offline replay, copy the SQLite bag and replace only that embedded timestamp
with the recorded packet Header.stamp, then use True. This avoids inventing a
constant LiDAR/IMU offset and leaves the source bag and online YAML untouched.
Replay the selected packet/IMU messages at their sensor header time as well:
recorder delivery delays can otherwise overflow the LIO synchronization buffers.
"""

import argparse
import copy
import hashlib
import json
from pathlib import Path
import shlex
import shutil
import sqlite3
import struct
import tempfile

import yaml


def rewrite_msop_timestamp(message):
    """Return (CDR message, changed), without changing its ROS header/payload size."""
    encoding = message[:2]
    if encoding not in (b"\x00\x01", b"\x00\x00"):
        raise ValueError("只支持 ROS 2 CDR1 格式的 RslidarPacket")
    endian = "<" if encoding == b"\x00\x01" else ">"
    sec, nsec, frame_length = struct.unpack_from(endian + "iII", message, 4)
    if sec < 0 or nsec >= 1_000_000_000 or frame_length < 1:
        raise ValueError("packet Header.stamp/frame_id 无效")
    # CDR alignment is relative to the payload after the 4-byte encapsulation.
    flags = 4 + 12 + frame_length
    if flags + 2 > len(message) or message[flags - 1] != 0:
        raise ValueError("packet frame_id 长度无效")
    size_offset = 4 + ((flags + 2 - 4 + 3) // 4) * 4
    size = struct.unpack_from(endian + "I", message, size_offset)[0]
    start = size_offset + 4
    if start + size != len(message):
        raise ValueError("packet data 长度无效")
    if message[flags] or message[start:start + 4] != b"\x55\xaa\x05\x5a":
        return message, False  # DIFOP and other packet types must stay intact.
    if size != 1248:
        raise ValueError(f"不支持的 RSAIRY MSOP 长度: {size}")
    result = bytearray(message)
    # RSAIRYHeader.timestamp is at byte 20: 6-byte UTC seconds + 4-byte us, BE.
    result[start + 20:start + 30] = (
        sec.to_bytes(6, "big") + (nsec // 1000).to_bytes(4, "big")
    )
    return bytes(result), True


def header_nanoseconds(message):
    if message[:2] not in (b"\x00\x01", b"\x00\x00"):
        raise ValueError("只支持 CDR1 消息")
    endian = "<" if message[:2] == b"\x00\x01" else ">"
    sec, nsec = struct.unpack_from(endian + "iI", message, 4)
    if sec < 0 or nsec >= 1_000_000_000:
        raise ValueError("消息 Header.stamp 无效")
    return sec * 1_000_000_000 + nsec


def choose_topics(info, driver, fastlio):
    types = {
        row["topic_metadata"]["name"]: row["topic_metadata"]["type"]
        for row in info["topics_with_message_count"]
    }
    common = fastlio["/**"]["ros__parameters"]["common"]
    candidates = [
        lidar for lidar in driver["lidar"]
        if lidar["ros"].get("ros_send_point_cloud_topic") == common["lid_topic"]
    ]
    if len(candidates) != 1:
        raise ValueError("驱动配置中无法唯一匹配 FastLIO 的点云话题")
    lidar = copy.deepcopy(candidates[0])
    if lidar["driver"]["lidar_type"] != "RSAIRY":
        raise ValueError("此离线准备工具仅支持 RSAIRY")
    packet = lidar["ros"].get("ros_recv_packet_topic")
    if packet not in types:
        packet = "/rslidar_packets"
    if types.get(packet) != "rslidar_msg/msg/RslidarPacket":
        raise ValueError("bag 中缺少配置的 Airy packet 话题或旧版 /rslidar_packets")
    imu = common["imu_topic"]
    if imu not in types:
        imu = "/rslidar_imu_data"
    if types.get(imu) != "sensor_msgs/msg/Imu":
        raise ValueError("bag 中缺少配置的 IMU 话题或旧版 /rslidar_imu_data")
    lidar["ros"]["ros_recv_packet_topic"] = packet
    lidar["driver"]["use_lidar_clock"] = True
    topics = [packet, imu]
    if "/imu/data" in types and "/imu/data" not in topics:
        topics.append("/imu/data")
    remaps = [] if imu == common["imu_topic"] else ["--remap", f"{imu}:={common['imu_topic']}"]
    return packet, lidar, topics, remaps


def prepare(bag, driver_path, fastlio_path, output, *, filter_imu_clock=False):
    metadata_path = bag / "metadata.yaml"
    metadata = yaml.safe_load(metadata_path.read_text())
    info = metadata["rosbag2_bagfile_information"]
    if info["storage_identifier"] != "sqlite3" or info.get("compression_format"):
        raise ValueError("Airy packet 时间准备目前只支持未压缩的 sqlite3 ROS 2 bag")
    driver = yaml.safe_load(driver_path.read_text())
    fastlio = yaml.safe_load(fastlio_path.read_text())
    packet, lidar, topics, remaps = choose_topics(info, driver, fastlio)
    if filter_imu_clock and topics[1] != "/imu/data":
        raise ValueError("本体 IMU 模式要求 bag 中有 /imu/data")
    paths = []
    for name in info["relative_file_paths"]:
        relative = Path(name)
        if relative.is_absolute() or ".." in relative.parts:
            raise ValueError("bag 数据库路径必须位于 bag 目录内")
        paths.append(bag / relative)
    fingerprint = {
        "version": 2, "bag": str(bag), "metadata": metadata_path.read_text(),
        "driver": driver_path.read_text(), "fastlio": fastlio_path.read_text(),
        "files": [(str(p.relative_to(bag)), p.stat().st_size, p.stat().st_mtime_ns) for p in paths],
    }
    if filter_imu_clock:
        fingerprint["filter_imu_clock_ns"] = 100_000_000
    digest = hashlib.sha256(json.dumps(fingerprint, sort_keys=True).encode()).hexdigest()[:20]
    output.mkdir(parents=True, exist_ok=True)
    target = output / digest
    if not (target / "ready.json").is_file():
        staging = Path(tempfile.mkdtemp(prefix=".preparing-", dir=output))
        rewritten = 0
        retimed = 0
        dropped_imu = 0
        file_info = []
        try:
            print("[INFO] 准备独立的离线 bag 副本，使用录制的 ROS Header.stamp 解码并排序回放", flush=True)
            for source in paths:
                destination = staging / "bag" / source.relative_to(bag)
                destination.parent.mkdir(parents=True, exist_ok=True)
                with sqlite3.connect(source.as_uri() + "?mode=ro", uri=True) as original:
                    with sqlite3.connect(destination) as replica:
                        original.backup(replica)
                        selected = [row for row in replica.execute("SELECT id,name FROM topics WHERE name IN (?,?)", (packet, topics[1]))]
                        for topic_id, topic_name in selected:
                            cursor = replica.execute("SELECT id,timestamp,data FROM messages WHERE topic_id=? ORDER BY id", (topic_id,))
                            while rows := cursor.fetchmany(1000):
                                updates = []
                                excluded_ids = []
                                for message_id, record_stamp, message in rows:
                                    sensor_stamp = header_nanoseconds(message)
                                    if filter_imu_clock and topic_name == "/imu/data" and abs(sensor_stamp - record_stamp) > 100_000_000:
                                        excluded_ids.append((message_id,))
                                        continue
                                    data, changed = rewrite_msop_timestamp(message) if topic_name == packet else (message, False)
                                    # DIFOP packets may legitimately have an empty header stamp.
                                    replay_stamp = sensor_stamp or record_stamp
                                    updates.append((data, replay_stamp, message_id))
                                    rewritten += int(changed)
                                    retimed += int(replay_stamp != record_stamp)
                                replica.executemany("UPDATE messages SET data=?,timestamp=? WHERE id=?", updates)
                                if excluded_ids:
                                    replica.executemany("DELETE FROM messages WHERE id=?", excluded_ids)
                                    dropped_imu += len(excluded_ids)
                        replica.commit()
                        first, last, count = replica.execute("SELECT MIN(timestamp),MAX(timestamp),COUNT(*) FROM messages").fetchone()
                        if not count:
                            raise ValueError("不支持空的 bag 数据库")
                        file_info.append({"path": str(source.relative_to(bag)), "starting_time": {"nanoseconds_since_epoch": first},
                                          "duration": {"nanoseconds": last - first}, "message_count": count})
            if rewritten == 0:
                raise ValueError("所选 packet 话题中没有可解码的 RSAIRY MSOP 包")
            if filter_imu_clock:
                imu_info = next(row for row in info["topics_with_message_count"] if row["topic_metadata"]["name"] == "/imu/data")
                imu_info["message_count"] -= dropped_imu
                if imu_info["message_count"] <= 0:
                    raise ValueError("过滤后没有与雷达同一时间基准的 /imu/data")
                info["message_count"] = sum(item["message_count"] for item in file_info)
                print(f"[INFO] /imu/data 保留 {imu_info['message_count']} 条，剔除 {dropped_imu} 条", flush=True)
            first = min(row["starting_time"]["nanoseconds_since_epoch"] for row in file_info)
            last = max(row["starting_time"]["nanoseconds_since_epoch"] + row["duration"]["nanoseconds"] for row in file_info)
            info.update(starting_time={"nanoseconds_since_epoch": first}, duration={"nanoseconds": last - first}, files=file_info)
            (staging / "bag" / "metadata.yaml").write_text(yaml.safe_dump(metadata, sort_keys=False))
            offline = copy.deepcopy(driver)
            offline["common"].update(msg_source=2, send_packet_ros=False, send_point_cloud_ros=True)
            offline["lidar"] = [lidar]
            (staging / "driver.yaml").write_text(yaml.safe_dump(offline, sort_keys=False))
            (staging / "ready.json").write_text(json.dumps({"rewritten_msop": rewritten, "retimed_messages": retimed, "fingerprint": fingerprint}, indent=2))
            staging.rename(target)
        except BaseException:
            shutil.rmtree(staging, ignore_errors=True)
            raise
    summary = json.loads((target / "ready.json").read_text())
    print(f"[INFO] 离线 MSOP 包数: {summary['rewritten_msop']}；按消息时间回放: {summary['retimed_messages']} 条；点云: {lidar['ros']['ros_send_point_cloud_topic']}", flush=True)
    return target, topics, remaps


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("bag", "driver-config", "fastlio-config", "output-dir", "env-file"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--filter-imu-clock", action="store_true")
    args = parser.parse_args()
    try:
        target, topics, remaps = prepare(
            args.bag.resolve(), args.driver_config.resolve(), args.fastlio_config.resolve(), args.output_dir.resolve(),
            filter_imu_clock=args.filter_imu_clock,
        )
        env = (
            f"BAG_DIR={shlex.quote(str(target / 'bag'))}\n"
            f"AIRY_CONFIG={shlex.quote(str(target / 'driver.yaml'))}\n"
            "BAG_TOPICS=(" + " ".join(map(shlex.quote, topics)) + ")\n"
            "BAG_REMAPS=(" + " ".join(map(shlex.quote, remaps)) + ")\n"
        )
        args.env_file.write_text(env)
    except (ValueError, KeyError, OSError, sqlite3.Error, struct.error, yaml.YAMLError) as exc:
        parser.exit(1, f"[ERROR] Airy 离线准备失败: {exc}\n")


if __name__ == "__main__":
    main()
