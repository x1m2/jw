"""Prepare independent stops, run vendored MLCC, validate and export fusion YAML."""
import hashlib
import itertools
import json
import os
from pathlib import Path
import shutil
import subprocess

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from .common import (apply, difference, json_write, pose_line, pose_matrix,
                     read_pcd, rigid, voxel, write_pcd)
from .geometry import Reference, register, statistics


def build(root, jobs=2):
    if jobs < 1:
        raise ValueError("--jobs 必须大于 0")
    dest = root / "build"
    dest.mkdir(exist_ok=True)
    with (dest / "build.log").open("w") as log:
        for cmd in (["cmake", "-S", str(root / "backend"), "-B", str(dest), "-DCMAKE_BUILD_TYPE=Release"],
                    ["cmake", "--build", str(dest), "--parallel", str(jobs)]):
            result = subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT)
            if result.returncode:
                raise ValueError(f"MLCC 编译失败，详见 {dest / 'build.log'}")
    return dest


def prepare(manifest, out, cfg):
    manifest = Path(manifest).resolve()
    data = json.loads(manifest.read_text())
    if data.get("schema_version") != 1 or not isinstance(data.get("records"), list):
        raise ValueError("dataset.json 需要 schema_version: 1 和 records 列表")
    records = data["records"]
    if len(records) > cfg["extraction"]["max_positions"] * cfg["dataset"]["max_scans_per_position"]:
        raise ValueError("输入位置过多；请明确筛选采集片段或增大 max_positions")
    delta = float(data.get("delta_s", 0.0))
    if not np.isfinite(delta):
        raise ValueError("dataset.delta_s 必须为有限数值")
    poses_supplied = ["T_world_front" in r for r in records]
    if any(poses_supplied) and not all(poses_supplied):
        raise ValueError("已有前雷达位姿须提供给全部位置，不能部分混用")
    if any(poses_supplied) and not data.get("poses_source"):
        raise ValueError("提供 T_world_front 时须填写 poses_source，确认它来自原生前雷达，未使用后雷达或融合结果")
    collected, excluded, fingerprints = [], [], {}
    previous, world = None, np.eye(4)
    ext, reg = cfg["extraction"], cfg["registration"]
    for index, item in enumerate(records):
        def cloud(key):
            path = (manifest.parent / item[key]).resolve()
            if not path.is_file():
                raise ValueError(f"找不到 {key}: {path}")
            fingerprints[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
            a = read_pcd(path)
            radius = np.linalg.norm(a[:, :3], axis=1)
            a = a[(radius >= ext["range_min_m"]) & (radius <= ext["range_max_m"])]
            if len(a) < ext["minimum_points"]:
                raise ValueError(f"{path} 距离过滤后有效点不足")
            return a
        front, rear = cloud("front"), cloud("rear")
        checks = {}
        # Imported datasets may carry a documented external static-motion check.
        external_check = item.get("static_verified") is True and bool(data.get("static_verification"))
        if not external_check:
            for side in ("front", "rear"):
                if side + "_start" not in item or side + "_end" not in item:
                    raise ValueError("每个静止位置需要首/末扫描；外部数据须显式声明 static_verified 和 static_verification")
                try:
                    relative, check = register(cloud(side + "_end"), cloud(side + "_start"), reg)
                    movement = difference(relative, np.eye(4))
                    if movement["translation_m"] > ext["static_translation_m"] or movement["rotation_deg"] > ext["static_rotation_deg"]:
                        raise ValueError(f"{side} 静止片段仍在移动: {movement}")
                    checks[side] = check
                except ValueError as error:
                    checks["rejected"] = str(error)
                    break
        if "rejected" in checks:
            excluded.append({"id": item.get("id", index), "reason": checks["rejected"]})
            continue
        if all(poses_supplied):
            world = rigid(item["T_world_front"], "T_world_front")
        elif previous is not None:
            relative, check = register(front, previous, reg)
            if check["translation_m"] > reg["max_step_translation_m"] or check["rotation_deg"] > reg["max_step_rotation_deg"]:
                raise ValueError(f"位置 {index} 与前一个位置跨度过大；按说明缩小移动步长重新录包，或提供前雷达独立轨迹")
            world = world @ relative
            checks["front_pose_registration"] = check
        previous = front
        collected.append({"id": item.get("id", index), "front": front, "rear": rear,
                          "pose": world.copy(), "requested_split": item.get("split"), "static_checks": checks})
        print(f"准备位置 {index + 1}/{len(records)}", flush=True)
    if not collected:
        raise ValueError("所有片段均未通过静止检查；请重新采集并查看静止诊断")
    supplied_split = [r["requested_split"] is not None for r in collected]
    if any(supplied_split) and not all(supplied_split):
        raise ValueError("split 必须全部提供或全部省略")
    origin_inverse = np.linalg.inv(collected[0]["pose"])
    groups = []
    for item in collected:
        item["pose"] = origin_inverse @ item["pose"]
        matched = None
        for group in groups:
            old = group[0]
            d = difference(item["pose"], old["pose"])
            if d["translation_m"] < cfg["dataset"]["same_position_m"] and d["rotation_deg"] < cfg["dataset"]["same_position_deg"]:
                matched = group
                break
        if matched is not None:
            if item["requested_split"] and matched[0]["requested_split"] and item["requested_split"] != matched[0]["requested_split"]:
                raise ValueError("相同停靠位置同时出现在 train 和 holdout，存在数据泄漏")
            if len(matched) < cfg["dataset"]["max_scans_per_position"]:
                matched.append(item)
            else:
                excluded.append({"id": item["id"], "reason": "该停靠位置已达到扫描数上限，不增加独立位置计数"})
        else:
            groups.append([item])
    if len(groups) > ext["max_positions"]:
        raise ValueError("独立停靠位置超过 max_positions")
    for i, group in enumerate(groups):
        split = group[0]["requested_split"] or ("holdout" if i % 3 == 1 else "train")
        if split not in ("train", "holdout"):
            raise ValueError("split 只支持 train/holdout")
        for item in group:
            item["split"], item["stop_cluster"] = split, i
    for split in ("train", "holdout"):
        n = sum(group[0]["split"] == split for group in groups)
        if n < cfg["dataset"][f"min_{split}_positions"]:
            raise ValueError(f"独立 {split} 位置不足：{n}；需要至少 {cfg['dataset'][f'min_{split}_positions']}。请采集至少 6 个不同位姿")
    unique = [r for group in groups for r in group]
    # Persist diagnostics before solving; rear holdout clouds never enter MLCC inputs.
    report = {"source_manifest": str(manifest), "input_source": data.get("source", str(manifest)), "robot": cfg["robot"], "delta_s": delta,
              "pose_source": data.get("poses_source", "front-only small-step point-to-plane ICP"),
              "static_verification": data.get("static_verification", "both IMUs and front/rear first-to-last cloud registration"),
              "independent_positions": len(groups),
              "positions": [{"id": r["id"], "split": r["split"], "stop_cluster": r["stop_cluster"], "T_world_front": r["pose"].tolist(),
                             "static_checks": r["static_checks"]} for r in unique],
              "excluded": excluded, "source_sha256": fingerprints,
              "timing_note": data.get("timing_note", "Imported historical offset; recheck on every live fusion startup.")}
    json_write(out / "selection.json", report)
    for split in ("train", "holdout"):
        dest = out / split
        for side in ("0", "1"):
            (dest / side).mkdir(parents=True)
        chosen = [r for r in unique if r["split"] == split]
        for i, record in enumerate(chosen):
            write_pcd(dest / "0" / f"{i}.pcd", record["front"])
            write_pcd(dest / "1" / f"{i}.pcd", record["rear"])
        np.savetxt(dest / "pose.json", [pose_line(r["pose"]) for r in chosen], fmt="%.17g")
        np.savetxt(dest / "ref.json", [pose_line(rigid(cfg["initial_T_front_rear"]))], fmt="%.17g")
    return unique, report


def solve(root, out, cfg):
    binaries = root / "build"
    if not all((binaries / ("mlcc_" + s)).is_file() for s in ("pose", "extrinsic", "global")):
        print("首次使用，编译内置 MLCC 后端……", flush=True)
        build(root)
    train, opt = out / "train", cfg["optimizer"]
    model = out / "optimization"
    model.mkdir()
    logs = out / "logs"
    logs.mkdir()
    shutil.copyfile(train / "pose.json", train / "initial_poses.txt")
    shutil.copyfile(train / "ref.json", train / "initial_ref.txt")
    def stage(name, work, prefix):
        cmd = [str(binaries / ("mlcc_" + name)), str(work), str(prefix),
               str(opt[name + "_iterations"]), str(opt["voxel_m"]),
               str(opt["eigen_ratio"]), str(opt["downsample_m"])]
        print(f"MLCC {prefix.parent.name}/{name}", flush=True)
        with (logs / (prefix.parent.name + "_" + name + ".log")).open("w") as log:
            result = subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT,
                                    timeout=opt["stage_timeout_s"], env=dict(os.environ, OMP_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1"))
        if result.returncode:
            raise ValueError(f"MLCC {name} 失败；查看 {logs}，检查初值、共视结构和场地几何")
    main = model / "main"
    main.mkdir()
    stage("pose", train, main / "pose")
    shutil.copyfile(main / "pose.poses.txt", train / "pose.json")
    stage("extrinsic", train, main / "extrinsic")
    shutil.copyfile(main / "extrinsic.poses.txt", train / "pose.json")
    shutil.copyfile(main / "extrinsic.ref.txt", train / "ref.json")
    stage("global", train, main / "global")
    result = {"main": pose_matrix(np.loadtxt(main / "global.ref.txt"))}
    initial = rigid(cfg["initial_T_front_rear"])
    for name, sign in (("perturb_plus", 1), ("perturb_minus", -1)):
        dest = model / name
        dest.mkdir()
        # Each restart has a local copy of training PCDs, separate poses and outputs.
        for sensor in ("0", "1"):
            shutil.copytree(train / sensor, dest / sensor)
        shutil.copyfile(main / "pose.poses.txt", dest / "pose.json")
        perturbed = initial.copy()
        perturbed[:3, :3] = Rotation.from_rotvec(np.deg2rad(sign * opt["perturb_rotation_deg"] * np.array([1, -0.8, 0.6]))).as_matrix() @ initial[:3, :3]
        perturbed[:3, 3] += sign * opt["perturb_translation_m"] * np.array([1, -0.8, 0.6])
        np.savetxt(dest / "ref.json", [pose_line(perturbed)], fmt="%.17g")
        stage("extrinsic", dest, dest / "extrinsic")
        shutil.copyfile(dest / "extrinsic.poses.txt", dest / "pose.json")
        shutil.copyfile(dest / "extrinsic.ref.txt", dest / "ref.json")
        stage("global", dest, dest / "global")
        result[name] = pose_matrix(np.loadtxt(dest / "global.ref.txt"))
    return result


def validate(records, matrices, cfg, out):
    settings = cfg["validation"]
    initial = rigid(cfg["initial_T_front_rear"])
    candidate = matrices["main"]
    # Reference uses front clouds/poses only. No optimized rear cloud contributes.
    world_front = voxel(np.concatenate([apply(r["front"], r["pose"]) for r in records]), settings["reference_voxel_m"])
    reference = Reference(world_front, settings["reference_voxel_m"], settings["plane_radius_m"])
    write_pcd(out / "front_reference.pcd", world_front)
    pooled = {"initial": [], "candidate": []}
    frames, normals_all, failures, preview = [], [], [], []
    for record in records:
        if record["split"] != "holdout":
            continue
        rear = voxel(record["rear"], 0.10)
        measured = {}
        for name, matrix in (("initial", initial), ("candidate", candidate)):
            world = apply(apply(rear, matrix), record["pose"])
            measured[name] = reference.measure(world, settings["plane_radius_m"])
            if name == "candidate":
                preview.append(world)
        common = measured["initial"][1] & measured["candidate"][1]
        result = {"id": record["id"], "stop_cluster": record.get("stop_cluster"), "points": len(rear), "common_points": int(common.sum()),
                  "common_fraction": float(common.mean())}
        for name in pooled:
            values = np.abs(measured[name][0][common])
            result[name] = statistics(values)
            pooled[name].append(values)
        normals_all.append(measured["candidate"][2][common])
        if common.sum() < settings["minimum_common_points"] or common.mean() < settings["minimum_common_fraction"]:
            failures.append(f"留出位置 {record['id']} 共同平面覆盖不足；增加前后雷达可关联的环境结构和采集位姿")
        if result["candidate"] is not None:
            a, b = result["initial"], result["candidate"]
            if b["rmse_m"] > settings["max_rmse_m"] or b["p95_m"] > settings["max_p95_m"]:
                failures.append(f"留出位置 {record['id']} 几何残差超过配置门限")
            if b["rmse_m"] > a["rmse_m"] * settings["max_regression_ratio"] + settings["regression_tolerance_m"]:
                failures.append(f"留出位置 {record['id']} 相比安装初值变差")
        frames.append(result)
    pooled = {key: statistics(np.concatenate(values)) for key, values in pooled.items()}
    normals = np.concatenate(normals_all)
    eigenvalues = np.linalg.eigvalsh(normals.T @ normals / len(normals)) if len(normals) else np.zeros(3)
    if eigenvalues[0] < settings["min_normal_eigenvalue"]:
        failures.append("留出平面法向方向覆盖不足；单一地面/墙面不足以验证空间外参")
    repeats = {a + "_vs_" + b: difference(matrices[a], matrices[b]) for a, b in itertools.combinations(matrices, 2)}
    for pair, d in repeats.items():
        if d["rotation_deg"] > settings["max_repeat_rotation_deg"] or d["translation_m"] > settings["max_repeat_translation_m"]:
            failures.append(f"多初值结果不稳定：{pair}")
    change = difference(candidate, initial)
    if change["rotation_deg"] > settings["max_change_rotation_deg"] or change["translation_m"] > settings["max_change_translation_m"]:
        failures.append("结果相对安装初值变化过大；核对坐标方向/安装初值，避免错误局部解")
    result = {"automated_checks_passed": not failures, "failures": failures, "per_position": frames,
              "pooled_common_support": pooled, "repeated_start_differences": repeats,
              "change_vs_initial": change, "plane_normal_eigenvalues": eigenvalues.tolist(),
              "reference": "All native front clouds at front-only initial poses; held-out rear positions never enter MLCC optimization. Residuals are not ground-truth extrinsic errors.",
              "scope": ("Same-recording holdout and small initial perturbations; independent recording and physical verification still required."
                        if len(matrices) > 1 else "Fixed candidate geometry check; recording independence must be confirmed by the operator. No new optimization or repeated-start test performed. Physical verification still required.")}
    write_pcd(out / "holdout_rear_candidate.pcd", voxel(np.concatenate(preview), 0.08))
    json_write(out / "validation.json", result)
    return result


def export(out, cfg, selection, matrices, validation):
    matrix = matrices["main"]
    config = {"transform": "front <- rear", "time_convention": "t_front = t_rear + delta_s",
              "T_front_rear": matrix.tolist(), "delta_s": selection["delta_s"],
              "status": "EXPERIMENTAL_NOT_ACCEPTED",
              "valid_for": f"Robot {cfg['robot']}; this rigid mounting only. Spatial candidate; recheck clock offset on each live startup."}
    text = ("# 后雷达 -> 前雷达；米制平移；逐次启动融合时重新检查时差。\n"
            "# 自动检查通过不等于独立真值精度验收；安装变化后重新标定。\n" + yaml.safe_dump(config, sort_keys=False, allow_unicode=True))
    (out / "candidate_extrinsics.yaml").write_text(text)
    if validation["automated_checks_passed"]:
        (out / "extrinsics.yaml").write_text(text)
    lines = [f"# 双雷达标定：{cfg['robot']}", "", f"自动检查：{'通过' if validation['automated_checks_passed'] else '未通过'}。", "",
             "空间变换：`p_front = T_front_rear * p_rear`；平移单位为米。", "",
             "## 留出位置验证", "", "| 位置 | 共同覆盖 | 初值 RMSE/m | 标定 RMSE/m | 标定 P95/m |",
             "|---|---:|---:|---:|---:|"]
    for row in validation["per_position"]:
        initial, final = row["initial"], row["candidate"]
        values = (f"{initial['rmse_m']:.5f}", f"{final['rmse_m']:.5f}", f"{final['p95_m']:.5f}") if final else ("无有效点",) * 3
        lines.append(f"| {row['id']} | {row['common_fraction']:.1%} | {' | '.join(values)} |")
    lines += ["", "## 结果", "", f"- 相对初值变化：{json.dumps(validation['change_vs_initial'], ensure_ascii=False)}",
              f"- 多初值对比：{json.dumps(validation['repeated_start_differences'], ensure_ascii=False) if validation['repeated_start_differences'] else '本次为固定外参复核，不重新优化/复算初值。'}",
              "- 完整指标：validation.json；原始选择及来源指纹：selection.json。",
              "- 空间矩阵候选：candidate_extrinsics.yaml。只有自动检查通过才生成 extrinsics.yaml。",
              "- 可视化：front_reference.pcd 与 holdout_rear_candidate.pcd，均在同一前雷达参考世界系。",
              "", "## 使用范围", "",
              "本次只求解空间外参。delta_s 是录包会话的接收时差估计，不是 MLCC 求得的硬件同步值。",
              "融合仍使用默认启动时间检查；不要跨会话使用 --fixed-time。",
              "同一录包的留出残差和多初值稳定性不能代替独立新录包/实际尺寸的验收。",
              f"本次验证范围：{validation['scope']}",
              "初始前雷达配准、环境退化和场景变化仍可能造成错误局部解。", ""]
    if validation["failures"]:
        lines += ["## 未通过原因", ""] + ["- " + reason for reason in validation["failures"]] + [""]
    else:
        lines += ["## 下一步", "", "用独立录包复核，并在实机融合包中运行：", "", "```bash",
                  f"bash run.sh --extrinsics {out / 'extrinsics.yaml'}", "```", "",
                  "上述命令在融合包目录执行。标定工具不会覆盖已有外参。", ""]
    (out / "REPORT.md").write_text("\n".join(lines))
