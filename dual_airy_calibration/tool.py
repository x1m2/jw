#!/usr/bin/env python3
"""One entry point for recording, building and calibrating a new robot."""
import argparse
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parent


def parser():
    p = argparse.ArgumentParser(description="双雷达离线空间标定；独立输入/输出，不覆盖已有外参")
    sub = p.add_subparsers(dest="command", required=True)
    init = sub.add_parser("init", help="按本机安装初值生成唯一配置文件")
    init.add_argument("--robot", required=True)
    initial = init.add_mutually_exclusive_group()
    initial.add_argument("--xyz", type=float, nargs=3, metavar=("TX", "TY", "TZ"), help="后雷达原点在前雷达坐标系的位置，单位米")
    initial.add_argument("--initial", type=Path, help="已有 YAML，含 T_front_rear；仅作为初值")
    init.add_argument("--rpy-deg", type=float, nargs=3, metavar=("ROLL", "PITCH", "YAW"))
    init.add_argument("--output", type=Path, required=True)
    sub.add_parser("doctor", help="检查 Python/ROS/CMake 和内置后端来源")
    b = sub.add_parser("build", help="在当前机器编译内置 MLCC")
    b.add_argument("--jobs", type=int, default=2)
    for command in ("record", "prepare", "calibrate", "verify"):
        s = sub.add_parser(command, help={"record": "在驱动已经运行时录制四路原生数据；Ctrl+C 结束",
                                         "prepare": "只提取 bag 的静止扫描，作为可复用检查点",
                                         "calibrate": "提取、标定、验证并输出报告",
                                         "verify": "在新录包上复核固定候选，不重新优化"}[command])
        s.add_argument("--config", type=Path, required=True)
        s.add_argument("--output", type=Path, required=True, help="必须是尚不存在的新目录")
        if command == "prepare":
            s.add_argument("--bag", type=Path, required=True)
        if command in ("calibrate", "verify"):
            source = s.add_mutually_exclusive_group(required=True)
            source.add_argument("--bag", type=Path)
            source.add_argument("--dataset", type=Path, help="prepare 生成的 dataset.json，或符合输入契约的外部数据")
        if command == "verify":
            s.add_argument("--extrinsics", type=Path, required=True)
    return p


def verify_backend():
    manifest = json.loads((ROOT / "backend/UPSTREAM.json").read_text())
    for relative, expected in manifest["sha256"].items():
        file = ROOT / relative
        if not file.is_file() or hashlib.sha256(file.read_bytes()).hexdigest() != expected:
            raise ValueError(f"内置算法源码缺失或与记录不符: {relative}")


def main():
    args = parser().parse_args()
    output = None
    try:
        import numpy as np
        import yaml
        from scipy.spatial.transform import Rotation
        from calibration.common import DEFAULTS, json_write, load_config, rigid
        from calibration.pipeline import build, prepare, solve, validate, export
        if args.command == "init":
            if args.output.exists():
                raise ValueError(f"配置已存在，拒绝覆盖：{args.output}")
            cfg = copy.deepcopy(DEFAULTS)
            cfg["robot"] = args.robot
            if args.initial:
                if args.rpy_deg is not None:
                    raise ValueError("--rpy-deg 只能与 --xyz 一起使用")
                cfg["initial_T_front_rear"] = rigid(yaml.safe_load(args.initial.read_text())["T_front_rear"]).tolist()
            else:
                if args.xyz is None:
                    if args.rpy_deg is not None:
                        raise ValueError("--rpy-deg 需要同时提供 --xyz")
                    print("填写本机粗略安装初值；坐标均指前雷达原生坐标系，不是机身坐标系。")
                    args.xyz = [float(v) for v in input("后雷达原点位置 tx ty tz（米，空格分隔）：").split()]
                    args.rpy_deg = [float(v) for v in input("后坐标轴 -> 前坐标轴的 roll pitch yaw（度，空格分隔）：").split()]
                    if len(args.xyz) != 3 or len(args.rpy_deg) != 3:
                        raise ValueError("位置和角度均须各输入 3 个数值")
                if args.rpy_deg is None:
                    raise ValueError("使用 --xyz 时必须同时指定 --rpy-deg，不能猜测安装方向")
                m = np.eye(4)
                m[:3, :3] = Rotation.from_euler("xyz", args.rpy_deg, degrees=True).as_matrix()
                m[:3, 3] = args.xyz
                cfg["initial_T_front_rear"] = rigid(m).tolist()
            if not args.robot.strip():
                raise ValueError("--robot 不得为空")
            with args.output.open("x") as f:
                f.write("# 本机粗略安装初值：后 -> 前；平移米；其余默认门限通常不需修改。\n")
                yaml.safe_dump(cfg, f, sort_keys=False, allow_unicode=True)
            print(f"配置已生成：{args.output.resolve()}")
            return 0
        verify_backend()
        if args.command == "doctor":
            missing = []
            for name in ("cmake", "c++", "ros2"):
                found = shutil.which(name)
                print(f"{name}: {found or '缺少'}")
                if not found:
                    missing.append(name)
            for name in ("numpy", "scipy", "yaml", "rosbag2_py", "rclpy", "sensor_msgs"):
                found = importlib.util.find_spec(name) is not None
                print(f"{name}: {'OK' if found else '缺少'}")
                if not found:
                    missing.append(name)
            print("内置 MLCC 来源和 SHA256：OK")
            print("C++ 开发依赖由 build 的 CMake 检查；说明见 README.md。")
            return 2 if missing else 0
        if args.command == "build":
            build(ROOT, args.jobs)
            print("MLCC 编译完成")
            return 0
        cfg = load_config(args.config)
        target = args.output.resolve()
        if target.exists():
            raise ValueError(f"输出已存在，拒绝覆盖；换一个新目录：{target}")
        if args.command == "record":
            if not shutil.which("ros2"):
                raise ValueError("找不到 ros2；请在安装 ROS2 Humble 的机器运行")
            print("确认两台驱动已运行、话题为原生点云、驱动未预应用外参。沿用本终端 ROS_DOMAIN_ID/RMW。", flush=True)
            print("按 README 的小步移动/停稳流程采集至少 6 个不同位姿，Ctrl+C 正常结束录包。", flush=True)
            process = subprocess.Popen(["ros2", "bag", "record", "-s", "sqlite3", "-o", str(target), *cfg["topics"].values()])
            try:
                return process.wait()
            except KeyboardInterrupt:
                # Terminal SIGINT reaches ros2 too. Do not kill it before metadata is finalized.
                result = process.wait(timeout=30)
                return 0 if result in (0, -2, 130) else result
        if getattr(args, "bag", None):
            bag = args.bag.resolve()
            if not (bag / "metadata.yaml").is_file():
                raise ValueError("--bag 须指向含 metadata.yaml 的录包目录")
            if target == bag or bag in target.parents:
                raise ValueError("输出不得放在输入 bag 内，以保持原始录包不变")
        target.mkdir(parents=True, exist_ok=False)
        output = target
        (target / "config.snapshot.yaml").write_text(yaml.safe_dump(cfg, sort_keys=False, allow_unicode=True))
        if args.command == "prepare" or args.bag:
            from calibration.bag import extract
            location = target if args.command == "prepare" else target / "input"
            location.mkdir(exist_ok=True)
            dataset = extract(args.bag, location, cfg)
            if args.command == "prepare":
                print(f"静止扫描检查点：{dataset}")
                return 0
        else:
            dataset = args.dataset
        records, selection = prepare(dataset, target, cfg)
        if args.command == "verify":
            prior_selection = args.extrinsics.resolve().parent / "selection.json"
            if prior_selection.is_file():
                previous = json.loads(prior_selection.read_text())
                old_fingerprints = set(previous.get("source_sha256", {}).values())
                new_fingerprints = set(selection["source_sha256"].values())
                if previous.get("input_source") == selection["input_source"] or old_fingerprints & new_fingerprints:
                    raise ValueError("复核使用了原标定数据或重复扫描；请重新录制不同位置的数据作为独立复核")
            selected = yaml.safe_load(args.extrinsics.read_text())
            if selected.get("transform") != "front <- rear":
                raise ValueError("待复核外参方向必须为 front <- rear")
            matrices = {"main": rigid(selected["T_front_rear"])}
            (target / "extrinsics.input.yaml").write_text(args.extrinsics.read_text())
        else:
            matrices = solve(ROOT, target, cfg)
        validation = validate(records, matrices, cfg, target)
        export(target, cfg, selection, matrices, validation)
        print(f"报告：{target / 'REPORT.md'}")
        if not validation["automated_checks_passed"]:
            print("自动检查未通过，未生成可供部署的 extrinsics.yaml。原因：", file=sys.stderr)
            for reason in validation["failures"]:
                print("- " + reason, file=sys.stderr)
            return 2
        print(f"自动检查通过。外参候选：{target / 'extrinsics.yaml'}；用独立录包复核后用于融合。")
        return 0
    except (ValueError, KeyError, TypeError, OSError, ImportError, EOFError, RuntimeError, subprocess.SubprocessError) as error:
        message = f"{type(error).__name__}: {error}"
        print("标定停止：" + message, file=sys.stderr)
        if output:
            (output / "FAILED.md").write_text("# 本次流程未完成\n\n" + message + "\n\n未生成可供部署的外参；保留本目录供诊断，重试请使用新输出目录。\n")
        return 2


if __name__ == "__main__":
    sys.exit(main())
