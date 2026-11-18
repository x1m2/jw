"""Configuration, rigid transforms and portable point-cloud IO."""
import copy
import json
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation


DEFAULTS = {
    "schema_version": 1,
    "robot": "",
    "initial_T_front_rear": None,
    "topics": {
        "front_points": "/rslidar_front/points", "rear_points": "/rslidar_rear/points",
        "front_imu": "/rslidar_front/imu", "rear_imu": "/rslidar_rear/imu",
    },
    "extraction": {
        "range_min_m": 1.2, "range_max_m": 25.0, "scan_duration_s": 0.1,
        "imu_bin_s": 0.5, "gyro_p95_rad_s": 0.025, "accel_std_gravity": 0.015,
        "minimum_imu_samples_per_bin": 20, "minimum_quiet_s": 2.0,
        "quiet_margin_s": 0.4, "max_pair_gap_s": 0.15,
        "minimum_points": 500, "max_positions": 24,
        "static_translation_m": 0.02, "static_rotation_deg": 0.3,
    },
    "registration": {
        "voxel_m": 0.12, "max_correspondence_m": 0.6, "iterations": 45,
        "min_support": 0.5, "max_rmse_m": 0.10,
        "max_step_translation_m": 0.8, "max_step_rotation_deg": 20.0,
    },
    "dataset": {
        "min_train_positions": 3, "min_holdout_positions": 2,
        "max_scans_per_position": 2,
        "same_position_m": 0.3, "same_position_deg": 5.0,
    },
    "optimizer": {
        "pose_iterations": 8, "extrinsic_iterations": 10, "global_iterations": 15,
        "voxel_m": 1.0, "eigen_ratio": 20.0, "downsample_m": 0.08,
        "perturb_rotation_deg": 0.5, "perturb_translation_m": 0.02,
        "stage_timeout_s": 900,
    },
    "validation": {
        "reference_voxel_m": 0.06, "plane_radius_m": 0.3,
        "minimum_common_points": 200, "minimum_common_fraction": 0.2,
        "max_rmse_m": 0.05, "max_p95_m": 0.10,
        "max_regression_ratio": 1.05, "regression_tolerance_m": 0.002,
        "min_normal_eigenvalue": 0.01,
        "max_repeat_rotation_deg": 0.3, "max_repeat_translation_m": 0.03,
        "max_change_rotation_deg": 10.0, "max_change_translation_m": 0.2,
    },
}


def merge_config(default, supplied, prefix=""):
    if not isinstance(supplied, dict):
        raise ValueError(f"{prefix or '配置'} 必须为字典")
    out = copy.deepcopy(default)
    unknown = set(supplied) - set(default)
    if unknown:
        raise ValueError(f"未知配置项 {prefix}: {sorted(unknown)}")
    for key, value in supplied.items():
        out[key] = merge_config(default[key], value, prefix + key + ".") if isinstance(default[key], dict) else value
    return out


def rigid(value, label="变换矩阵"):
    m = np.asarray(value, dtype=float)
    if m.shape != (4, 4) or not np.isfinite(m).all():
        raise ValueError(f"{label} 必须为有限数值的 4×4 矩阵")
    if not np.allclose(m[3], [0, 0, 0, 1], atol=1e-12, rtol=0):
        raise ValueError(f"{label} 最后一行必须为 [0,0,0,1]")
    r = m[:3, :3]
    if not np.allclose(r.T @ r, np.eye(3), atol=1e-5, rtol=0) or abs(np.linalg.det(r) - 1) > 1e-5:
        raise ValueError(f"{label} 旋转必须正交且行列式为 +1")
    return m


def load_config(path):
    cfg = merge_config(DEFAULTS, yaml.safe_load(Path(path).read_text()))
    if cfg["schema_version"] != 1 or not isinstance(cfg["robot"], str) or not cfg["robot"].strip():
        raise ValueError("请填写 schema_version: 1 和非空 robot 名称")
    rigid(cfg["initial_T_front_rear"], "initial_T_front_rear；请使用 init 填写本机安装初值")
    topics = list(cfg["topics"].values())
    if any(not isinstance(t, str) or not t.startswith("/") for t in topics) or len(set(topics)) != 4:
        raise ValueError("四个 topics 必须是不同的绝对 ROS 话题名")
    for section in ("extraction", "registration", "dataset", "optimizer", "validation"):
        for key, value in cfg[section].items():
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not np.isfinite(value) or value <= 0:
                raise ValueError(f"{section}.{key} 必须为正有限数值")
    integer_keys = {
        "extraction": ["minimum_imu_samples_per_bin", "minimum_points", "max_positions"],
        "registration": ["iterations"], "dataset": ["min_train_positions", "min_holdout_positions", "max_scans_per_position"],
        "optimizer": ["pose_iterations", "extrinsic_iterations", "global_iterations"],
        "validation": ["minimum_common_points"],
    }
    for section, keys in integer_keys.items():
        for key in keys:
            if not isinstance(cfg[section][key], int):
                raise ValueError(f"{section}.{key} 必须为整数")
    if cfg["dataset"]["min_train_positions"] < 3 or cfg["dataset"]["min_holdout_positions"] < 2:
        raise ValueError("至少需要 3 个训练位置、2 个留出位置")
    e = cfg["extraction"]
    if e["range_min_m"] >= e["range_max_m"] or e["minimum_quiet_s"] <= 2 * e["quiet_margin_s"] + e["scan_duration_s"]:
        raise ValueError("距离范围或静止时间设置不合法")
    for section, key in (("registration", "min_support"), ("validation", "minimum_common_fraction")):
        if cfg[section][key] > 1:
            raise ValueError(f"{section}.{key} 必须在 (0,1] 内")
    if e["max_positions"] < cfg["dataset"]["min_train_positions"] + cfg["dataset"]["min_holdout_positions"]:
        raise ValueError("max_positions 不足以划分训练和留出位置")
    return cfg


def json_write(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n")


def apply(points, matrix):
    return points[:, :3] @ matrix[:3, :3].T + matrix[:3, 3]


def pose_line(matrix):
    q = Rotation.from_matrix(matrix[:3, :3]).as_quat()
    return [*matrix[:3, 3], q[3], *q[:3]]


def pose_matrix(line):
    line = np.asarray(line, dtype=float)
    if line.shape != (7,) or not np.isfinite(line).all() or abs(np.linalg.norm(line[3:]) - 1) > 0.01:
        raise ValueError("位姿必须为 tx ty tz qw qx qy qz，四元数须归一化")
    m = np.eye(4)
    m[:3, :3] = Rotation.from_quat([*line[4:], line[3]]).as_matrix()
    m[:3, 3] = line[:3]
    return m


def difference(a, b):
    return {
        "rotation_deg": float(np.rad2deg(Rotation.from_matrix(a[:3, :3] @ b[:3, :3].T).magnitude())),
        "translation_m": float(np.linalg.norm(a[:3, 3] - b[:3, 3])),
    }


def voxel(points, size):
    _, indices = np.unique(np.floor(points[:, :3] / size).astype(np.int64), axis=0, return_index=True)
    return points[np.sort(indices)]


def write_pcd(path, points):
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] < 3 or not np.isfinite(points[:, :3]).all():
        raise ValueError("PCD 输入必须为有限 XYZ")
    intensity = np.nan_to_num(points[:, 3]) if points.shape[1] > 3 else np.ones(len(points))
    a = np.column_stack([points[:, :3], intensity]).astype("<f4")
    n = len(a)
    with Path(path).open("wb") as f:
        f.write((f"VERSION .7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
                 f"WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary\n").encode())
        f.write(a.tobytes())


def read_pcd(path):
    """Accept the portable uncompressed XYZ/XYZI PCD contract, not arbitrary PCD encodings."""
    with Path(path).open("rb") as f:
        header = {}
        while True:
            raw = f.readline()
            if not raw:
                raise ValueError(f"PCD 头不完整: {path}")
            line = raw.decode("ascii").strip().split()
            if not line or line[0].startswith("#"):
                continue
            header[line[0].upper()] = line[1:]
            if line[0].upper() == "DATA":
                break
        fields = header.get("FIELDS", [])
        if fields not in (["x", "y", "z"], ["x", "y", "z", "intensity"]):
            raise ValueError(f"只支持 XYZ/XYZI PCD: {path}")
        if header.get("SIZE") != ["4"] * len(fields) or header.get("TYPE") != ["F"] * len(fields):
            raise ValueError(f"PCD 字段须为 float32: {path}")
        if header.get("COUNT", ["1"] * len(fields)) != ["1"] * len(fields):
            raise ValueError(f"PCD COUNT 不支持: {path}")
        n = int(header["POINTS"][0])
        mode = header["DATA"][0]
        if mode == "binary":
            a = np.frombuffer(f.read(), dtype="<f4").copy()
        elif mode == "ascii":
            a = np.loadtxt(f, dtype=np.float32).reshape(-1)
        else:
            raise ValueError(f"请将 PCD 转为 binary 或 ascii（不支持 compressed）: {path}")
        if a.size != n * len(fields) or n < 100:
            raise ValueError(f"PCD 点数不匹配或不足 100: {path}")
        a = a.reshape(n, len(fields))
        return a[np.isfinite(a[:, :3]).all(axis=1)]
