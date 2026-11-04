#!/usr/bin/env python3
"""Generate a real ROS2 sqlite3 bag to exercise the ordinary no-trajectory workflow."""
import copy
import json
from pathlib import Path
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Imu, PointCloud2, PointField
from scipy.spatial.transform import Rotation
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from calibration.common import DEFAULTS, apply
from test_workflow import room


def make(out):
    out.mkdir(parents=True, exist_ok=False)
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(out / "bag"), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    cfg = copy.deepcopy(DEFAULTS)
    true = np.eye(4)
    true[:3, :3] = Rotation.from_euler("xyz", [178, 0.7, 3], degrees=True).as_matrix()
    true[:3, 3] = [-0.01, 0.02, -0.92]
    initial = true.copy()
    initial[:3, :3] = Rotation.from_rotvec(np.deg2rad([0.8, -0.5, 0.6])).as_matrix() @ true[:3, :3]
    initial[:3, 3] += [0.025, -0.02, 0.03]
    cfg["robot"] = "synthetic_bag"
    cfg["initial_T_front_rear"] = initial.tolist()
    for key, topic in cfg["topics"].items():
        kind = "Imu" if key.endswith("imu") else "PointCloud2"
        writer.create_topic(rosbag2_py.TopicMetadata(name=topic, type="sensor_msgs/msg/" + kind, serialization_format="cdr"))
    world = room(n=1800)
    epoch_ns = 1700000000000000000
    def stamp(message, seconds):
        ns = int(round(seconds * 1e9))
        message.header.stamp.sec = ns // 1000000000
        message.header.stamp.nanosec = ns % 1000000000
    for i in range(8):
        pose = np.eye(4)
        pose[:3, :3] = Rotation.from_euler("z", i * 8, degrees=True).as_matrix()
        pose[:3, 3] = [0.16 * i, 0.04 * np.sin(i), 0.0]
        front = apply(world, np.linalg.inv(pose))
        clouds = {"front": front, "rear": apply(front, np.linalg.inv(true))}
        for j in range(1000):
            t = i * 5 + j / 200
            moving = j >= 800
            received = epoch_ns + int(round(t * 1e9))
            for side in ("front", "rear"):
                sensor = 1000 + t + (0.01 if side == "rear" else 0)
                imu = Imu()
                stamp(imu, sensor)
                imu.header.frame_id = "rslidar_" + side
                imu.linear_acceleration.z = 9.81
                imu.angular_velocity.z = 0.15 if moving else 0.0
                writer.write(cfg["topics"][side + "_imu"], serialize_message(imu), received)
                if j % 20 == 0 and not moving:
                    points = np.column_stack([clouds[side], np.ones(len(world))]).astype("<f4")
                    msg = PointCloud2()
                    stamp(msg, sensor)
                    msg.header.frame_id = "rslidar_" + side
                    msg.width, msg.height = len(points), 1
                    msg.point_step, msg.row_step = 16, len(points) * 16
                    msg.fields = [PointField(name=k, offset=index * 4, datatype=7, count=1)
                                  for index, k in enumerate(("x", "y", "z", "intensity"))]
                    msg.data = points.tobytes()
                    writer.write(cfg["topics"][side + "_points"], serialize_message(msg), received)
    del writer
    (out / "robot.yaml").write_text(yaml.safe_dump(cfg, sort_keys=False))
    (out / "truth.json").write_text(json.dumps(true.tolist()))


if __name__ == "__main__":
    make(Path(sys.argv[1]).resolve())
