# 双 Airy 雷达标定工具

以后换机器狗，使用本目录即可。它自带 MLCC C++ 空间标定后端、录包入口、静止筛选、前雷达初始位姿配准、三阶段优化、多初值复算、留出验证与报告。无需寻找旧实验脚本、旧轨迹或其他机器的外参。

正常使用只需关心 **一个机器配置文件、一个录包目录、一个结果目录**。此工具只做离线空间标定；实时融合使用同级 `dual_airy_deploy_cpp` 或 `dual_airy_deploy_cpp_v5` 包。

## 1. 第一次准备电脑

建议在 Ubuntu 22.04 / ROS2 Humble 的电脑上离线计算；机器狗负责运行前后雷达驱动和录包。录包也可复制到电脑后处理。当前验证平台是 x86_64；ARM64 如需使用，应在目标机器重新编译，不能复制电脑二进制。

在本目录执行：

```bash
# 仅首次缺少依赖时安装；不需要 ROS1 或旧 SLAM 工作空间。
sudo apt install build-essential cmake libpcl-dev libopencv-dev libeigen3-dev \
  nlohmann-json3-dev python3-numpy python3-scipy python3-yaml \
  ros-humble-rosbag2-py ros-humble-rosbag2-storage-default-plugins \
  ros-humble-rclpy ros-humble-sensor-msgs

bash run.sh doctor
bash run.sh build
```

编译默认使用 2 个并行任务；内存不足时用 `bash run.sh build --jobs 1`。构建日志在 `build/build.log`。`COLCON_IGNORE` 使本工具不参与整个机器人工作空间的日常 colcon 编译。

## 2. 为新机器生成配置

```bash
bash run.sh init --robot dog_02 --output dog_02.yaml
```

按提示输入本机粗略安装初值：

- `tx ty tz`：后雷达原点在**前雷达原生坐标系**中的位置，单位米。
- `roll pitch yaw`：将后雷达坐标轴转到前雷达坐标轴的旋转，单位度。
- 方向固定为 `p_front = R * p_rear + t`，`R = Rz(yaw) * Ry(pitch) * Rx(roll)`；不要直接填机身坐标系的安装位置。

先根据安装尺寸和轴向给出粗略初值，尽量控制在约 5° / 10 cm 内。这是局部精化工具，不是无需初值的全局配准器。不要把另一台机器的最终矩阵当成本机真值。

也可通过 `--xyz TX TY TZ --rpy-deg ROLL PITCH YAW` 显式提供六个数，或用 `--initial 安装初值.yaml` 导入含 `T_front_rear` 的 YAML。已有配置不会被覆盖。

生成的 `dog_02.yaml` 包含默认参数。一般只改 `robot`、安装初值和必要的话题名，其余参数先使用默认值。

## 3. 按流程采集

前后雷达驱动需已运行；同一终端的 `ROS_DOMAIN_ID`、RMW 应与驱动一致。输入必须是各自原生雷达坐标系中的数据，驱动不能提前应用前后雷达外参：

| 默认话题 | 类型 |
|---|---|
| `/rslidar_front/points` | `sensor_msgs/msg/PointCloud2` |
| `/rslidar_rear/points` | `sensor_msgs/msg/PointCloud2` |
| `/rslidar_front/imu` | `sensor_msgs/msg/Imu` |
| `/rslidar_rear/imu` | `sensor_msgs/msg/Imu` |

不需要启动 SLAM，不需要记录里程计，不使用 `/dual_airy/points_merged`。只含 packet 的旧录包需要先通过驱动解码；正常新录包直接使用下面的命令。

```bash
bash run.sh record --config dog_02.yaml --output /data/calibration/dog_02_bag
```

开始录制后：

1. 在静态、结构丰富的场地停稳 4～6 秒。场地应有不同方向的墙面、地面和立体结构，只有一片地面/一面墙不够。
2. 每次移动约 0.3～0.5 米，或转动约 8～12°，再次停稳 4～6 秒。相邻停靠位置的前雷达场景需大量重叠；不要突然转身或跳到另一个房间。
3. 采集 8～12 个不同位置/朝向，至少 6 个。增加方向和一定的高度/姿态变化；多次在同一位置等待不算多个独立位置。前、后雷达要在整段采集中看到可关联的静态结构，同一时刻不必完全共视。
4. `Ctrl+C` 正常结束录包，确认 bag 目录包含 `metadata.yaml`。录制过程中不要重启传感器或改变安装。

工具会检查两路 IMU 静止，以及前后点云在每个片段首末的移动量。大跨度采集、退化几何或无法配准时会停止；应按上述步长重采，而不是靠放宽门限强行出结果。

## 4. 一条命令标定

在已安装依赖的电脑、工具目录中执行：

```bash
bash run.sh calibrate \
  --config dog_02.yaml \
  --bag /data/calibration/dog_02_bag \
  --output runs/dog_02_01
```

输出目录必须尚不存在。工具会完成：读取录包 → 静止扫描提取 → 前雷达小步配准 → 重复位姿合并 → 按完整停靠位置划分训练/留出 → 位姿优化 → 外参优化 → 联合优化 → 两组扰动初值复算 → 验证和导出。原始录包、配置和已有融合外参不会被修改。

前雷达位姿仅由前点云估计；留出位置的后点云不进入优化。验证参考仅使用前点云及前雷达独立初始位姿。自动检查包含共同平面覆盖、逐位置残差、方向覆盖、相对初值变化和多初值一致性。

## 5. 只看结果目录

```text
runs/dog_02_01/
├── REPORT.md                    # 先看它：通过情况、逐位置指标、后续命令
├── extrinsics.yaml              # 仅自动检查通过时生成，融合直接读取
├── candidate_extrinsics.yaml    # 求解完成后的候选；检查未通过时也保留供诊断
├── validation.json              # 完整自动检查指标和失败原因
├── selection.json               # 位姿/训练划分/输入指纹
├── front_reference.pcd          # 前雷达参考点云
├── holdout_rear_candidate.pcd   # 同一参考系中的标定后后雷达留出点云
├── config.snapshot.yaml
└── logs/                        # 算法失败时检查这里
```

失败返回码为 2。处理失败时有 `FAILED.md`；求解完成但验证未通过时看 `REPORT.md` / `validation.json`。失败结果不会生成 `extrinsics.yaml`，也不会修改线上配置。重试换一个新输出目录，保留旧结果便于比较。

`extrinsics.yaml` 只包含融合实际读取的 6 个字段：`transform`、`time_convention`、`T_front_rear`、`delta_s`、`status`、`valid_for`。

自动检查通过仍是候选：按第 3 步重新录制一段不同位置/朝向的数据，然后复核固定外参（不重新优化）：

```bash
bash run.sh verify --config dog_02.yaml \
  --extrinsics runs/dog_02_01/extrinsics.yaml \
  --bag /data/calibration/dog_02_check_bag --output runs/dog_02_check
```

复核也查看新目录的 `REPORT.md`，并在实际融合中检查重合和安装尺寸。留出点到面残差是几何指标，不是真值外参精度；前雷达配准或场景结构仍可能造成错误局部解。导出的状态保持 `EXPERIMENTAL_NOT_ACCEPTED`。

## 6. 交给实时融合

独立数据复核后，在融合包目录执行，路径替换为本次结果的绝对路径：

```bash
bash run.sh --extrinsics /绝对路径/runs/dog_02_01/extrinsics.yaml
```

默认启动流程会重新检查本次时差。MLCC 只求解空间矩阵；导出的 `delta_s` 是录包接收时间与传感器时间的历史差分估计，包含传输延迟，不能跨会话直接使用 `--fixed-time`。改变安装后重新标定；只重启传感器而安装未变时，沿用空间矩阵、重新检查时间即可。

## 中断后复用已提取扫描（可选）

数据较大时可先单独提取，之后调整初值/参数不必重复读取 bag：

```bash
bash run.sh prepare --config dog_02.yaml \
  --bag /data/calibration/dog_02_bag --output runs/dog_02_scans
bash run.sh calibrate --config dog_02.yaml \
  --dataset runs/dog_02_scans/dataset.json --output runs/dog_02_02
```

静止筛选和距离过滤已冻结在检查点中；修改 extraction 参数需重新 prepare。复制检查点时同时复制 `dataset.json` 和 `clouds/`。

高级输入契约：`--dataset` 接受 `schema_version: 1`、`records` 列表，每项有 `id/front/rear`，路径相对 manifest，PCD 为 float32 XYZ/XYZI 的 binary 或 ascii。可提供全部 `T_world_front`（4×4 前雷达到同一世界系）及 `poses_source`，仅允许来自前雷达独立里程计/测量，不得直接填机身轨迹。未提供位姿时自动配准。静止检查需要 `front_start/front_end/rear_start/rear_end`；若外部流程已严格验证，须逐项写 `static_verified: true` 并提供全局 `static_verification` 说明。可全部指定 `split: train/holdout`，同一位姿不能跨集合。导入静止数据本身不估计时差，未给 `delta_s` 时仅填占位 0，实机融合必须使用默认启动时间检查。

## 源码与测试

本次软件验证范围与真实数据回归结果见 [VERIFICATION.md](VERIFICATION.md)。

- `backend/vendor/`：固定版本 MLCC 核心，`backend/src/`：三阶段独立适配入口。
- 来源、版本、SHA256 见 `backend/UPSTREAM.json`；许可证随包保留在 `LICENSE`（GPLv2）。去除 ROS1 include，保持原优化方程，外层体素迭代保持世界坐标规范不变。
- `calibration/`：通用录包、配准、数据划分、验证与报告；正常操作不用修改。
- 不依赖其他实验目录，不下载源码、不读取旧机器配置，不启动驱动或回放进程。

```bash
/usr/bin/python3 -m unittest discover -s tests -p 'test_*.py' -v
# 完整普通用户入口回归（自动生成 ROS2 bag、标定并检查已知真值）
source /opt/ros/humble/setup.bash
/usr/bin/python3 tests/check_integration.py

# 也可单独生成测试输入：
/usr/bin/python3 tests/make_synthetic.py /tmp/dual_airy_synthetic_input
bash run.sh calibrate --config /tmp/dual_airy_synthetic_input/robot.yaml \
  --dataset /tmp/dual_airy_synthetic_input/dataset.json --output /tmp/dual_airy_synthetic_result

# 标准 ROS2 bag 入口回归；只写本地测试录包，不启动 ROS 节点。
source /opt/ros/humble/setup.bash
/usr/bin/python3 tests/make_bag.py /tmp/dual_airy_bag_input
bash run.sh calibrate --config /tmp/dual_airy_bag_input/robot.yaml \
  --bag /tmp/dual_airy_bag_input/bag --output /tmp/dual_airy_bag_result
```

测试路径同样必须为新目录。上传 Git 只需本工具目录中的源码、配置模板、文档和许可证；`build/`、`runs/`、录包和地图不上传。直接复制整个目录到新电脑即可，复制时排除 `build/`、`runs/` 和 `__pycache__/`。
