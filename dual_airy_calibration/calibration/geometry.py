"""Front-only scan registration and independent static plane residuals."""
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from .common import apply, difference, voxel


class Reference:
    def __init__(self, points, size=0.06, radius=0.3):
        self.points = voxel(points[:, :3], size)
        if len(self.points) < 100:
            raise ValueError("参考点云不足 100 点")
        self.tree = cKDTree(self.points)
        self.normals = np.zeros_like(self.points)
        self.centers = np.zeros_like(self.points)
        self.valid = np.zeros(len(self.points), bool)
        for lo in range(0, len(self.points), 10000):
            hi = min(lo + 10000, len(self.points))
            dist, ix = self.tree.query(self.points[lo:hi], k=12, workers=2)
            neighbours = self.points[ix]
            centers = neighbours.mean(axis=1)
            centered = neighbours - centers[:, None, :]
            cov = np.einsum("nki,nkj->nij", centered, centered) / 12
            values, vectors = np.linalg.eigh(cov)
            self.centers[lo:hi] = centers
            self.normals[lo:hi] = vectors[:, :, 0]
            self.valid[lo:hi] = ((values[:, 0] / np.maximum(values[:, 1], 1e-12) < 0.1)
                                    & (values[:, 1] > 0.0001) & (dist[:, -1] < radius))

    def measure(self, points, max_distance=0.3):
        dist, ix = self.tree.query(points, workers=2)
        normals = self.normals[ix]
        signed = np.einsum("ij,ij->i", points - self.centers[ix], normals)
        return signed, self.valid[ix] & (dist < max_distance), normals


def register(source, target, cfg, initial=None):
    """Small-step point-to-plane ICP. Reject low support/degenerate geometry."""
    matrix = np.eye(4) if initial is None else initial.copy()
    src = voxel(source[:, :3], cfg["voxel_m"])
    if len(src) > 15000:
        src = src[np.linspace(0, len(src) - 1, 15000).astype(int)]
    ref = Reference(target[:, :3], cfg["voxel_m"], max(0.3, 3 * cfg["voxel_m"]))
    for _ in range(cfg["iterations"]):
        points = apply(src, matrix)
        residual, mask, normals = ref.measure(points, cfg["max_correspondence_m"])
        if mask.sum() < 100 or mask.mean() < cfg["min_support"]:
            raise ValueError(f"前雷达配准覆盖不足 ({mask.mean():.1%})；缩小两次停靠间移动/转角，或换结构丰富的场地")
        p, n, r = points[mask], normals[mask], residual[mask]
        a = np.column_stack([np.cross(p, n), n])
        weight = np.sqrt(np.minimum(1.0, 0.05 / np.maximum(np.abs(r), 1e-12)))
        step, _, rank, singular = np.linalg.lstsq(a * weight[:, None], -r * weight, rcond=1e-5)
        if rank < 6 or singular[-1] / singular[0] < 1e-5:
            raise ValueError("前雷达配准几何退化；场地需有多个方向的墙面/地面及立体结构")
        scale = max(1.0, np.linalg.norm(step[:3]) / 0.1, np.linalg.norm(step[3:]) / 0.15)
        step /= scale
        update = np.eye(4)
        update[:3, :3] = Rotation.from_rotvec(step[:3]).as_matrix()
        update[:3, 3] = step[3:]
        matrix = update @ matrix
        if np.linalg.norm(step) < 1e-6:
            break
    residual, mask, _ = ref.measure(apply(src, matrix), cfg["max_correspondence_m"])
    rmse = float(np.sqrt(np.mean(residual[mask] ** 2))) if mask.any() else float("inf")
    if mask.mean() < cfg["min_support"] or not np.isfinite(rmse) or rmse > cfg["max_rmse_m"]:
        raise ValueError(f"前雷达配准未通过：support={mask.mean():.1%}, RMSE={rmse:.3f}m")
    return matrix, {"support": float(mask.mean()), "plane_rmse_m": rmse, **difference(matrix, np.eye(4))}


def statistics(values):
    if not len(values):
        return None
    return {"points": len(values), "median_m": float(np.median(values)),
            "p95_m": float(np.percentile(values, 95)),
            "rmse_m": float(np.sqrt(np.mean(values ** 2)))}
