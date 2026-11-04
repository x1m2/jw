#!/usr/bin/env python3
"""Repeat the public bag workflow and fixed-candidate verification with known truth."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from calibration.common import difference


def main():
    out = Path(tempfile.mkdtemp(prefix="dual_airy_calibration_test."))
    print(f"测试产物：{out}", flush=True)
    def command(*args, expected=0):
        result = subprocess.run(["/usr/bin/python3", *map(str, args)], cwd=ROOT)
        if result.returncode != expected:
            raise RuntimeError(f"expected exit {expected}, got {result.returncode}: {args}")
    command(ROOT / "tests/make_bag.py", out / "bag_input")
    command(ROOT / "tool.py", "calibrate", "--config", out / "bag_input/robot.yaml",
            "--bag", out / "bag_input/bag", "--output", out / "bag_run")
    truth = np.array(json.loads((out / "bag_input/truth.json").read_text()))
    estimated = np.array(yaml.safe_load((out / "bag_run/extrinsics.yaml").read_text())["T_front_rear"])
    error = difference(estimated, truth)
    if error["rotation_deg"] >= 0.05 or error["translation_m"] >= 0.002:
        raise RuntimeError(f"synthetic truth check failed: {error}")
    command(ROOT / "tests/make_synthetic.py", out / "independent", 91)
    command(ROOT / "tool.py", "verify", "--config", out / "bag_input/robot.yaml",
            "--extrinsics", out / "bag_run/extrinsics.yaml", "--dataset", out / "independent/dataset.json",
            "--output", out / "independent_verify")
    command(ROOT / "tool.py", "verify", "--config", out / "bag_input/robot.yaml",
            "--extrinsics", out / "bag_run/extrinsics.yaml", "--dataset", out / "bag_run/input/dataset.json",
            "--output", out / "same_input_rejected", expected=2)
    if (out / "same_input_rejected/extrinsics.yaml").exists():
        raise RuntimeError("same-input rejection incorrectly exported extrinsics")
    print("PASS: bag extraction, static checks, front-only poses, MLCC, truth error, verification, duplicate-input rejection")
    print(json.dumps({"truth_error": error, "artifacts": str(out)}, indent=2))


if __name__ == "__main__":
    main()
