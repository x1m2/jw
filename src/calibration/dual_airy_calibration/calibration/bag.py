"""Read native PointCloud2/Imu bags offline; never starts drivers or bag playback."""
from pathlib import Path
import numpy as np
import yaml

from .common import json_write, write_pcd


def pointcloud(message):
    """Respect field offsets, endian, organized-cloud row padding and invalid XYZ."""
    fields = {f.name: f for f in message.fields}
    if not all(k in fields for k in ("x", "y", "z")):
        raise ValueError("输入 PointCloud2 缺少 x/y/z；请记录驱动原生点云")
    endian = ">" if message.is_bigendian else "<"
    formats = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}
    names = ["x", "y", "z"] + (["intensity"] if "intensity" in fields else [])
    if message.width <= 0 or message.height <= 0 or message.point_step <= 0:
        raise ValueError("收到空 PointCloud2")
    dtypes, offsets = [], []
    for name in names:
        f = fields[name]
        if f.count != 1 or f.datatype not in formats:
            raise ValueError(f"不支持 PointCloud2 字段 {name} 的类型/COUNT")
        dtype = np.dtype(endian + formats[f.datatype])
        if f.offset < 0 or f.offset + dtype.itemsize > message.point_step:
            raise ValueError("PointCloud2 字段越过 point_step")
        dtypes.append(dtype)
        offsets.append(f.offset)
    if message.row_step < message.width * message.point_step or len(message.data) < message.row_step * message.height:
        raise ValueError("PointCloud2 数据长度/row_step 不合法")
    dtype = np.dtype({"names": names, "formats": dtypes, "offsets": offsets, "itemsize": message.point_step})
    a = np.ndarray((message.height, message.width), dtype=dtype, buffer=message.data,
                   strides=(message.row_step, message.point_step)).reshape(-1)
    xyz = np.column_stack([a[k] for k in ("x", "y", "z")]).astype(float)
    intensity = a["intensity"].astype(float) if "intensity" in names else np.ones(len(xyz))
    good = np.isfinite(xyz).all(axis=1)
    return np.column_stack([xyz[good], np.nan_to_num(intensity[good])])


def quiet_segments(streams, cfg):
    """Use one recorder clock to intersect static intervals; do not assume synced headers."""
    lo = max(s[0, 0] for s in streams.values())
    hi = min(s[-1, 0] for s in streams.values())
    width = cfg["imu_bin_s"]
    quiet = []
    for start in np.arange(lo, hi - width, width):
        okay = True
        for rows in streams.values():
            part = rows[(rows[:, 0] >= start) & (rows[:, 0] < start + width)]
            if len(part) < cfg["minimum_imu_samples_per_bin"] or np.max(np.diff(part[:, 0]), initial=0) > 0.1:
                okay = False
                break
            gravity = float(np.median(np.linalg.norm(part[:, 5:8], axis=1)))
            if gravity < 0.1 or not np.isfinite(gravity):
                raise ValueError("IMU 加速度无效，不能判断静止")
            gyro = np.percentile(np.linalg.norm(part[:, 2:5], axis=1), 95)
            accel = np.linalg.norm(np.std(part[:, 5:8], axis=0)) / gravity
            if gyro > cfg["gyro_p95_rad_s"] or accel > cfg["accel_std_gravity"]:
                okay = False
                break
        if okay:
            if quiet and start - quiet[-1][1] < width * 0.1:
                quiet[-1][1] = float(start + width)
            else:
                quiet.append([float(start), float(start + width)])
    return [s for s in quiet if s[1] - s[0] >= cfg["minimum_quiet_s"]]


def reader_for(path):
    import rosbag2_py
    path = Path(path).resolve()
    metadata = path / "metadata.yaml"
    if not metadata.is_file():
        raise ValueError("--bag 须指向含 metadata.yaml 的 ROS2 bag 目录（先解压；不是 zip 或单个 db3）")
    information = yaml.safe_load(metadata.read_text())["rosbag2_bagfile_information"]
    if information.get("compression_format"):
        raise ValueError("请先用 ros2 bag convert 将压缩 bag 转为未压缩 sqlite3/mcap")
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=information["storage_identifier"]),
                rosbag2_py.ConverterOptions("", ""))
    return reader


def extract(path, out, cfg):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Imu, PointCloud2
    topics = cfg["topics"]
    reader = reader_for(path)
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    for key, topic in topics.items():
        expected = "sensor_msgs/msg/Imu" if key.endswith("imu") else "sensor_msgs/msg/PointCloud2"
        if types.get(topic) != expected:
            raise ValueError(f"bag 需要 {topic} ({expected})；packet 或已融合点云不能代替原生前后点云")
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topics[k + "_imu"] for k in ("front", "rear")]))
    streams = {"front": [], "rear": []}
    while reader.has_next():
        topic, data, received = reader.read_next()
        key = "front" if topic == topics["front_imu"] else "rear"
        msg = deserialize_message(data, Imu)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        row = [received * 1e-9, stamp, msg.angular_velocity.x, msg.angular_velocity.y,
               msg.angular_velocity.z, msg.linear_acceleration.x, msg.linear_acceleration.y,
               msg.linear_acceleration.z]
        if stamp <= 0 or not np.isfinite(row).all():
            raise ValueError(f"{topic} 时间戳或 IMU 数据非法")
        if streams[key] and stamp <= streams[key][-1][1]:
            raise ValueError(f"{topic} header 时间未严格递增；检查传感器重启/时钟跳变并重新录包")
        streams[key].append(row)
    for key, rows in streams.items():
        if len(rows) < 100:
            raise ValueError(f"{key} IMU 样本不足")
        streams[key] = np.asarray(rows)
    ext = cfg["extraction"]
    segments = quiet_segments(streams, ext)
    if not segments:
        raise ValueError("未找到双 IMU 同时静止的片段；每个位置停稳至少 4 秒，检查 IMU 单位及噪声")
    if len(segments) > ext["max_positions"]:
        raise ValueError(f"静止片段 {len(segments)} 超过 max_positions；分段录制或在配置中明确增大上限")
    # Three samples per stop prove that apparently quiet IMUs do not hide steady motion.
    targets = np.array([[lo + ext["quiet_margin_s"], (lo + hi) / 2,
                         hi - ext["quiet_margin_s"] - ext["scan_duration_s"]] for lo, hi in segments])
    best = {k: {} for k in streams}
    distance = {k: np.full(targets.shape, np.inf) for k in streams}
    last_stamp, frame_ids = {}, {}
    offsets = {k: [] for k in streams}
    reader = reader_for(path)
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topics[k + "_points"] for k in streams]))
    count = 0
    while reader.has_next():
        topic, data, received = reader.read_next()
        t = received * 1e-9
        if t < targets.min() - ext["max_pair_gap_s"] or t > targets.max() + ext["max_pair_gap_s"]:
            continue
        key = "front" if topic == topics["front_points"] else "rear"
        delta = np.abs(targets - t)
        if not np.any((delta < distance[key]) & (delta <= ext["max_pair_gap_s"])):
            continue
        msg = deserialize_message(data, PointCloud2)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp <= 0 or stamp <= last_stamp.get(key, -np.inf):
            raise ValueError(f"{topic} 选中点云时间戳不递增")
        last_stamp[key] = stamp
        frame = msg.header.frame_id
        if not frame or (key in frame_ids and frame != frame_ids[key]):
            raise ValueError(f"{topic} frame_id 缺失或录包中改变")
        frame_ids[key] = frame
        points = pointcloud(msg)
        radius = np.linalg.norm(points[:, :3], axis=1)
        points = points[(radius >= ext["range_min_m"]) & (radius <= ext["range_max_m"])]
        if len(points) < ext["minimum_points"]:
            continue
        for i, j in np.argwhere((delta < distance[key]) & (delta <= ext["max_pair_gap_s"])):
            best[key][(int(i), int(j))] = (points, stamp, t)
            distance[key][i, j] = delta[i, j]
        offsets[key].append(t - stamp)
        count += 1
    del reader
    if frame_ids.get("front") == frame_ids.get("rear"):
        raise ValueError("前后点云 frame_id 相同；请记录各自原生坐标系，关闭驱动外参预变换")
    clouds = out / "clouds"
    clouds.mkdir()
    records, rejected = [], []
    for i, interval in enumerate(segments):
        if any((i, j) not in best[k] for k in streams for j in range(3)):
            rejected.append({"segment": i, "reason": "静止片段缺少首/中/末完整前后扫描"})
            continue
        record = {"id": i, "recorder_interval_s": interval}
        for key in streams:
            for j, label in enumerate(("start", "middle", "end")):
                points, stamp, received = best[key][(i, j)]
                name = f"clouds/{i:03}_{key}_{label}.pcd"
                write_pcd(out / name, points)
                record[key if j == 1 else key + "_" + label] = name
                if j == 1:
                    record[key + "_stamp_s"] = stamp
                    record[key + "_receive_s"] = received
        if abs(record["front_receive_s"] - record["rear_receive_s"]) > ext["max_pair_gap_s"]:
            rejected.append({"segment": i, "reason": "前后扫描接收时刻差过大"})
            continue
        records.append(record)
    del best
    delta_imu = float(np.median(streams["rear"][:, 0] - streams["rear"][:, 1])
                      - np.median(streams["front"][:, 0] - streams["front"][:, 1]))
    result = {"schema_version": 1, "source": str(Path(path).resolve()), "records": records,
              "delta_s": delta_imu, "frame_ids": frame_ids,
              "timing_note": "Recorder receive-offset estimate only; MLCC solves spatial extrinsics, not clock sync. Recheck time on each fusion startup.",
              "quiet_segments": segments, "rejected": rejected, "selected_cloud_messages": count}
    json_write(out / "dataset.json", result)
    print(f"提取 {len(records)} 个静止片段；缺帧排除 {len(rejected)} 个", flush=True)
    return out / "dataset.json"
