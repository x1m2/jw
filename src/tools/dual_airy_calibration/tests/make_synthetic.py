#!/usr/bin/env python3
"""Generate a known-transform, multi-pose dataset without ROS or private data."""
import copy
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from calibration.common import DEFAULTS, apply, write_pcd
from test_workflow import room


def make(out, seed=54):
    out.mkdir(parents=True, exist_ok=False)
    points = room(seed=seed, n=1800)
    true = np.eye(4)
    true[:3, :3] = Rotation.from_euler("xyz", [178, 0.7, 3], degrees=True).as_matrix()
    true[:3, 3] = [-0.01, 0.02, -0.92]
    initial = true.copy()
    initial[:3, :3] = Rotation.from_rotvec(np.deg2rad([0.8, -0.5, 0.6])).as_matrix() @ true[:3, :3]
    initial[:3, 3] += [0.025, -0.02, 0.03]
    records = []
    for i in range(8):
        pose = np.eye(4)
        pose[:3, :3] = Rotation.from_euler("xyz", [i * 0.7, i * 0.3, i * 32], degrees=True).as_matrix()
        pose[:3, 3] = [0.5 * np.cos(i), 0.4 * np.sin(i), 0.1 * np.sin(2 * i)]
        front = apply(points, np.linalg.inv(pose))
        rear = apply(front, np.linalg.inv(true))
        write_pcd(out / f"{i}_front.pcd", front)
        write_pcd(out / f"{i}_rear.pcd", rear)
        records.append({"id": i, "front": f"{i}_front.pcd", "rear": f"{i}_rear.pcd",
                        "T_world_front": pose.tolist(), "static_verified": True})
    manifest = {"schema_version": 1, "poses_source": "known synthetic front poses",
                "static_verification": "generated static plane scenes", "records": records, "delta_s": 0.0}
    (out / "dataset.json").write_text(json.dumps(manifest, indent=2))
    cfg = copy.deepcopy(DEFAULTS)
    cfg["robot"] = "synthetic"
    cfg["initial_T_front_rear"] = initial.tolist()
    (out / "robot.yaml").write_text(yaml.safe_dump(cfg, sort_keys=False))
    (out / "truth.json").write_text(json.dumps(true.tolist()))


if __name__ == "__main__":
    make(Path(sys.argv[1]).resolve(), int(sys.argv[2]) if len(sys.argv) > 2 else 54)
