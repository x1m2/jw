"""Behavior tests: portable bag layout, static gating, registration and data leakage."""
import copy
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from calibration.bag import pointcloud, quiet_segments
from calibration.common import DEFAULTS, apply, difference, load_config, pose_line, pose_matrix, read_pcd, rigid, write_pcd
from calibration.geometry import register
from calibration.pipeline import prepare, validate, export


def room(seed=54, n=1200):
    random = np.random.default_rng(seed)
    points = []
    for axis in range(3):
        for position in (-3.0, 3.0):
            a = random.uniform(-3, 3, (n, 3))
            a[:, axis] = position
            points.append(a)
    return np.concatenate(points)


class WorkflowTest(unittest.TestCase):
    def test_organized_big_endian_cloud_and_invalid_points(self):
        dtype = np.dtype({"names": ["x", "y", "z"], "formats": [">f4"] * 3,
                          "offsets": [0, 4, 8], "itemsize": 16})
        data = bytearray(80)  # 2 rows × (2 points × 16 bytes + 8 padding)
        a = np.ndarray((2, 2), dtype=dtype, buffer=data, strides=(40, 16))
        a["x"] = [[1, 2], [3, np.nan]]
        a["y"] = 5
        a["z"] = 6
        msg = NS(fields=[NS(name=k, offset=i * 4, datatype=7, count=1) for i, k in enumerate(("x", "y", "z"))],
                 width=2, height=2, row_step=40, point_step=16, data=data, is_bigendian=True)
        points = pointcloud(msg)
        np.testing.assert_allclose(points[:, 0], [1, 2, 3])
        self.assertEqual(points.shape, (3, 4))
        msg.row_step = 20
        with self.assertRaises(ValueError):
            pointcloud(msg)

    def test_quiet_intersection_excludes_motion_and_gaps(self):
        t = np.arange(0, 10, 0.005)
        a = np.zeros((len(t), 8))
        a[:, 0] = t
        a[:, 1] = t + 20
        a[:, 7] = 9.81
        b = a.copy()
        b[(t > 4) & (t < 6), 2] = 0.1
        segments = quiet_segments({"front": a, "rear": b}, DEFAULTS["extraction"])
        self.assertEqual(len(segments), 2)
        self.assertTrue(all(end <= 4 or start >= 6 for start, end in segments))
        missing = b[(t < 1) | (t > 9)]
        self.assertEqual(quiet_segments({"front": a, "rear": missing}, DEFAULTS["extraction"]), [])

    def test_small_step_registration_direction(self):
        target = room(n=2000)
        true = np.eye(4)
        true[:3, :3] = Rotation.from_euler("xyz", [2, -1, 7], degrees=True).as_matrix()
        true[:3, 3] = [0.25, -0.12, 0.04]
        source = apply(target, np.linalg.inv(true))
        estimated, _ = register(source, target, DEFAULTS["registration"])
        delta = difference(estimated, true)
        self.assertLess(delta["rotation_deg"], 0.05)
        self.assertLess(delta["translation_m"], 0.002)

    def test_single_plane_registration_rejected(self):
        a = room()[:1200]
        with self.assertRaisesRegex(ValueError, "退化|覆盖"):
            register(a, a, DEFAULTS["registration"])

    def test_config_does_not_guess_initial_mounting(self):
        with tempfile.TemporaryDirectory() as temp:
            p = Path(temp) / "robot.yaml"
            cfg = copy.deepcopy(DEFAULTS)
            cfg["robot"] = "test"
            p.write_text(yaml.safe_dump(cfg))
            with self.assertRaises(ValueError):
                load_config(p)
            cfg["initial_T_front_rear"] = np.eye(4).tolist()
            cfg["optimizer"]["global_iterations"] = 1.5
            p.write_text(yaml.safe_dump(cfg))
            with self.assertRaisesRegex(ValueError, "整数"):
                load_config(p)

    def test_transform_and_pcd_roundtrip(self):
        m = np.eye(4)
        m[:3, :3] = Rotation.from_euler("xyz", [178, 0.7, 3], degrees=True).as_matrix()
        m[:3, 3] = [0.01, 0.02, -0.92]
        np.testing.assert_allclose(pose_matrix(pose_line(m)), m, atol=1e-14)
        bad = m.copy()
        bad[:3, :3] *= -1
        with self.assertRaises(ValueError):
            rigid(bad)
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "test.pcd"
            points = room()[:100]
            write_pcd(path, points)
            np.testing.assert_allclose(read_pcd(path)[:, :3], points, atol=1e-6)

    def test_same_pose_cannot_leak_into_holdout(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)
            write_pcd(path / "cloud.pcd", room())
            record = {"front": "cloud.pcd", "rear": "cloud.pcd", "static_verified": True,
                      "T_world_front": np.eye(4).tolist()}
            data = {"schema_version": 1, "poses_source": "front-only synthetic test",
                    "static_verification": "synthetic stationary scans", "records":
                    [dict(record, split="train"), dict(record, split="holdout")]}
            (path / "dataset.json").write_text(json.dumps(data))
            cfg = copy.deepcopy(DEFAULTS)
            cfg["initial_T_front_rear"] = np.eye(4).tolist()
            with self.assertRaisesRegex(ValueError, "数据泄漏"):
                prepare(path / "dataset.json", path, cfg)

    def test_failed_validation_never_exports_deploy_filename(self):
        with tempfile.TemporaryDirectory() as temp:
            out = Path(temp)
            points = room(n=500)
            records = [{"id": i, "split": "holdout", "front": points, "rear": points, "pose": np.eye(4)} for i in range(2)]
            cfg = copy.deepcopy(DEFAULTS)
            cfg["initial_T_front_rear"] = np.eye(4).tolist()
            cfg["robot"] = "synthetic"
            wrong = np.eye(4)
            wrong[:3, 3] = [1, 1, 1]
            matrices = {"main": wrong, "perturb_plus": wrong, "perturb_minus": wrong}
            result = validate(records, matrices, cfg, out)
            self.assertFalse(result["automated_checks_passed"])
            export(out, cfg, {"delta_s": 0}, matrices, result)
            self.assertTrue((out / "candidate_extrinsics.yaml").exists())
            self.assertFalse((out / "extrinsics.yaml").exists())


if __name__ == "__main__":
    unittest.main()
