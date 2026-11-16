# Super-LIO 重定位与纯定位

前端保留 Super-LIO 的 ESKF 和 OctVoxMap，只匹配已有地图，不插入新地图点，
不执行建图、回环或图优化。基础库位于 `src/depd/basic`。

## BBS3D 后端与编译

与 `fastlio_localization` 共用 `src/depd/3d_bbs` 安装的 CPU/OpenCL 库，
不再内置另一套 CPU/CUDA 实现。

```bash
# 仅 CPU
bash src/depd/3d_bbs/build_cpu.sh
colcon build --packages-up-to super_lio --cmake-args -DENABLE_OPENCL_BBS=OFF

# OpenCL（需要对应的开发环境及运行设备）
bash src/depd/3d_bbs/build_opencl.sh
colcon build --packages-up-to super_lio --cmake-args -DENABLE_OPENCL_BBS=ON
source install/setup.bash
```

非默认安装位置可通过 `-DBBS3D_ROOT=/path/to/bbs3d` 指定。
YAML 的 `bbs3d.backend` 选择 `cpu` 或 `opencl`；`opencl_batch_size`、
`opencl_fallback_cpu`、`opencl_verify` 与 FAST-LIO 使用同一实现。
默认选择 CPU。明确请求 OpenCL 且禁止 CPU 回退时，初始化失败会报告 Worker 错误。

## 启动与配置

两份配置 `reloc_superlio_mid360s.yaml`、`reloc_superlio_airy96.yaml` 均包含传感器、前端和重定位参数。
地图目录使用 `map_path`，其中必须包含 `globalmap.pcd`。

```bash
ros2 launch super_lio localization.py \
  config_file:=reloc_superlio_mid360s.yaml \
  robot_param_file:=$PWD/src/config/P2.5_mid360s.yaml \
  map_path:=/robotfs/data/slam_data/map_0/map rviz:=false
```

Airy 使用 `config_file:=reloc_superlio_airy96.yaml`，并传入对应机器人配置，例如
`robot_param_file:=$PWD/src/config/P3_airy96.yaml`。
Airy 的 `lio.sensor.lidar_type` 为 `8`（RSAIRY）。输入校验与 FAST-LIO 一致：
`x/y/z/intensity` 为 float32、`ring` 为 uint16、`timestamp` 为 float64，
拒绝空点云及首尾非有限时间戳或无效尾时间戳。帧开始采用消息头时间，
逐点偏移采用秒制的 `timestamp - 原始首点timestamp`；同步时按偏移排序，
以最大偏移计算帧结束时间并等待 IMU 覆盖。
`config:=...` 仍可作为配置文件参数使用。
机器人配置提供弧度制 `lidar_to_base_R` 和米制 `lidar_to_base_T`，即雷达在 base_link 中的位姿。
`lio.extrinsic_T` 为三维平移，`lio.extrinsic_R` 为按行展开的九元素旋转矩阵，
表示 `p_imu = R * p_lidar + t`；数值格式与 FAST-LIO 的 `extrinsic_T`、`extrinsic_R` 一致。
未提供时采用零平移、零旋转；实际机器人应传入正确的外参文件。
`lio.timedelay_lidar2imu` 以秒为单位加到 IMU 时间戳上，用于将 IMU 时间对齐到雷达时钟；
默认 `0.0`，符号约定与 FAST-LIO 的 `timedelay_lidar2imu` 相同。

## 重力与重定位

`bbs3d.gravity_align: true` 时，用 IMU 平均加速度构造重力对齐旋转，
通过 `T_gravity_imu * T_imu_lidar` 变换扫描点云，再在零附近搜索 roll/pitch 残差，
并搜索 yaw。结果通过 `T_map_imu = T_map_gravity * T_gravity_imu` 转回滤波器位姿。
手动初值也使用重力确定 roll/pitch，并保留输入 yaw。
关闭该开关时搜索原始雷达点云，需要配置覆盖真实安装姿态的角度范围。

`relocalization.algorithm_type` 支持 `bbs3d`、`scan_context`、`manually_set`。
Scan Context 需要至少 10 个轨迹点和对应描述子；其他模式只需要先验点云。
`voxelmaps_coords/` 缺失时可从已有地图生成搜索索引，不是在线建图。
`relocalization_profile` 支持 `initial`、`map_connection`，分别读取 `bnb3d` 中对应配置。
当前两个 profile 沿用原有调参值，可分别调整。

保留的 GNSS 先验输入为地图坐标系下的 `nav_msgs/Odometry`，由
`relocalization.use_gnss` 和 `relocalization.gnss_topic` 控制；
不将原始经纬度直接作为地图坐标。

## 工程接口

- `map_frame`、`lidar_frame`、`baselink_frame` 可配置，默认 `map`、`lidar`、`base_link`。
- `/initialpose` 输入地图坐标系下的 **base_link 位姿**，按外参转换到 IMU。
- `/lidar_localization`、`/imu_localization` 输出地图到 **base_link** 的里程计。
- `/pose_stamped_localization`、`/app/localization_pose` 输出机器人位姿。
- 应用匹配分数采用与 FAST-LIO 相同的有效点比例和平均残差加权公式；
  两个前端的点匹配结果不同，不能保证分数数值相同。
- `/reloc_success` 报告重定位/定位状态；无有效观测或非有限状态时重新初始化。
- 雷达或 IMU 超过 `sensor_timeout_sec`（默认 1 秒）没有收到数据时，
  `/reloc_success` 变为 false；两路数据恢复且前端再次有效定位后才恢复 true。
  超时使用单调时钟，暂停回放也会触发，检测周期为 100 ms。
- `/cloud_registered`、`/cloud_registered_body`、`/global_map`、`/imu_path` 输出可视化数据。
- `path_en`、`scan_publish_en`、`dense_publish_en`、`lidar_tf_broadcast`、`imu_tf_broadcast` 控制输出。
- `lio.output.pub_step` 控制世界系点云每隔多少帧发布一次，必须为正整数。
- `publish.print_pose: true` 时，每个有效定位帧打印底盘在地图中的 XYZ（米）、
  roll/pitch/yaw（度）、降采样点数、匹配分数和耗时，格式与 FAST-LIO 一致。
  `cost time` 使用单调时钟统计 IMU 传播、去畸变、降采样和观测更新耗时；
  `input_delay` 为开始处理时 ROS 时间与扫描结束时间的差值；`pose_delay` 为完成输出时
  ROS 时间与发布位姿时间戳的差值，单位均为毫秒。
  离线回放应使用匹配的仿真时钟，否则延迟包含录制时间与当前时间的差值。
  无效定位帧始终打印错误和诊断数据，不受 `publish.print_pose` 开关影响。
  点云发布开关只使用根层 `scan_publish_en`、`dense_publish_en`；
  重定位重力对齐由 `bbs3d.gravity_align` 控制，精配准 GICP 由 `gicp.use_gicp` 控制。
- `/lio/odom` 保留 IMU 位姿输出，供调试使用。

可执行文件 `superlio_localization_node` 使用与 FAST-LIO 相同的 `ManagedWorker` 协议。
launch 默认 `runtime_profile:=test`，自动启动算法；`platform` 模式等待 START。
服务为 `/superlio_localization_worker/set_state`，支持地图路径、手动初值模式、重定位 profile 和 session 启停，
不提供地图保存操作。停止时等待后台任务结束。

`slam_manager` 的 `localization.worker_service` 默认仍选择 FAST-LIO。
使用 Super-LIO 时，改为 `/superlio_localization_worker/set_state`，并启动 Super-LIO Worker 的 platform 模式。

上游项目：https://github.com/Liansheng-Wang/Super-LIO （GPLv3，见 LICENSE）。
