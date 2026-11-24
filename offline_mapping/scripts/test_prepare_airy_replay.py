import copy
from pathlib import Path
import sqlite3
import struct
import tempfile
import unittest

import yaml

from prepare_airy_replay import choose_topics, prepare, rewrite_msop_timestamp


def packet(sec=1791798902, nsec=747963200, frame="rslidar_front", endian="<", difop=False):
    frame = frame.encode() + b"\0"
    body = bytearray(struct.pack(endian + "iII", sec, nsec, len(frame)) + frame + bytes([difop, 1]))
    body.extend(b"\0" * (-len(body) % 4))
    raw = bytearray(1248)
    raw[:4] = b"\x55\xaa\x05\x5a"
    raw[20:30] = (sec + 37).to_bytes(6, "big") + (123456).to_bytes(4, "big")
    start = 4 + len(body) + 4
    body.extend(struct.pack(endian + "I", len(raw)) + raw)
    return (b"\0\1\0\0" if endian == "<" else b"\0\0\0\0") + body, start


class ReplayTest(unittest.TestCase):
    def test_timestamp_and_alignment(self):
        for endian in ("<", ">"):
            for frame in ("", "a", "abcd", "rslidar_front"):
                original, start = packet(endian=endian, frame=frame)
                result, changed = rewrite_msop_timestamp(original)
                self.assertTrue(changed)
                self.assertEqual(result[:start + 20], original[:start + 20])
                self.assertEqual(result[start + 30:], original[start + 30:])
                self.assertEqual(int.from_bytes(result[start + 20:start + 26], "big"), 1791798902)
                self.assertEqual(int.from_bytes(result[start + 26:start + 30], "big"), 747963)

    def test_difop_intact_and_invalid_input_rejected(self):
        original, _ = packet(difop=True)
        self.assertEqual(rewrite_msop_timestamp(original), (original, False))
        original, _ = packet(nsec=1_000_000_000)
        with self.assertRaises(ValueError):
            rewrite_msop_timestamp(original)
        original, _ = packet()
        with self.assertRaises(ValueError):
            rewrite_msop_timestamp(original[:-1])

    def test_prepare_copy_and_legacy_remap(self):
        with tempfile.TemporaryDirectory(prefix="airy-test-") as directory:
            root = Path(directory)
            bag = root / "original"
            bag.mkdir()
            driver = {"common": {"msg_source": 1}, "lidar": [
                {"driver": {"lidar_type": "RSAIRY", "use_lidar_clock": False},
                 "ros": {"ros_recv_packet_topic": "/rslidar_front/packets", "ros_send_point_cloud_topic": "/rslidar_front/points"}}
            ]}
            fastlio = {"/**": {"ros__parameters": {"common": {
                "lid_topic": "/rslidar_front/points", "imu_topic": "/rslidar_front/imu"}}}}
            info = {"storage_identifier": "sqlite3", "relative_file_paths": ["data.db3"],
                    "topics_with_message_count": [
                        {"topic_metadata": {"name": "/rslidar_front/packets", "type": "rslidar_msg/msg/RslidarPacket"}},
                        {"topic_metadata": {"name": "/rslidar_front/imu", "type": "sensor_msgs/msg/Imu"}}]}
            (bag / "metadata.yaml").write_text(yaml.safe_dump({"rosbag2_bagfile_information": info}))
            driver_path, fastlio_path = root / "driver.yaml", root / "fastlio.yaml"
            driver_path.write_text(yaml.safe_dump(driver))
            fastlio_path.write_text(yaml.safe_dump(fastlio))
            original, _ = packet()
            difop, _ = packet(difop=True)
            with sqlite3.connect(bag / "data.db3") as db:
                db.executescript("CREATE TABLE topics(id INTEGER,name TEXT); CREATE TABLE messages(id INTEGER,topic_id INTEGER,timestamp INTEGER,data BLOB);")
                db.execute("INSERT INTO topics VALUES(1,'/rslidar_front/packets')")
                db.executemany("INSERT INTO messages VALUES(?,1,100,?)", [(1, original), (2, difop)])
            before = (bag / "data.db3").read_bytes()
            target, topics, remaps = prepare(bag, driver_path, fastlio_path, root / "output")
            self.assertEqual(topics, ["/rslidar_front/packets", "/rslidar_front/imu"])
            self.assertEqual(remaps, [])
            self.assertEqual((bag / "data.db3").read_bytes(), before)
            self.assertEqual(yaml.safe_load(driver_path.read_text()), driver)
            replay_driver = yaml.safe_load((target / "driver.yaml").read_text())
            self.assertEqual(replay_driver["common"]["msg_source"], 2)
            self.assertTrue(replay_driver["lidar"][0]["driver"]["use_lidar_clock"])
            with sqlite3.connect(target / "bag/data.db3") as db:
                rows = db.execute("SELECT timestamp,data FROM messages ORDER BY id").fetchall()
            sensor_stamp = 1791798902_747963200
            self.assertEqual(rows, [(sensor_stamp, rewrite_msop_timestamp(original)[0]), (sensor_stamp, difop)])
            self.assertEqual(prepare(bag, driver_path, fastlio_path, root / "output")[0], target)
            legacy = copy.deepcopy(info)
            legacy["topics_with_message_count"][0]["topic_metadata"]["name"] = "/rslidar_packets"
            legacy["topics_with_message_count"][1]["topic_metadata"]["name"] = "/rslidar_imu_data"
            selected, _, _, remaps = choose_topics(legacy, driver, fastlio)
            self.assertEqual(selected, "/rslidar_packets")
            self.assertEqual(remaps, ["--remap", "/rslidar_imu_data:=/rslidar_front/imu"])


if __name__ == "__main__":
    unittest.main()
