# Super-LIO 重定位与纯定位

本包基于 diwei 分支的 Super-LIO，仅保留已有地图上的重定位和连续定位。
前端使用 IMU 传播、点云去畸变、ESKF 和 OctVoxMap 地图匹配。
定位过程中不会插入新地图点，不生成关键帧，不执行回环检测或位姿图优化，
也不保存或覆盖建图产物。

基础库位于 `src/depd/basic`，以独立 ROS 包提供。

## 编译

在工作区根目录执行：

```bash
colcon build --packages-up-to super_lio
source install/setup.bash
```

默认使用 CPU BBS3D，无需 CUDA 或 GTSAM。需要 GPU BBS3D 时，
可在已安装 CUDA 的环境下传入 `--cmake-args -DBUILD_CUDA=ON`。

## 启动

在 `config/localization.yaml` 中将 `official.map_path` 设置为先验地图目录，
该目录必须包含 `globalmap.pcd`。

```bash
ros2 launch super_lio localization.py config:=livox_360.yaml
# Airy 雷达
ros2 launch super_lio localization.py config:=airy.yaml rviz:=false
```

机器人配置提供传感器、外参和前端参数，`localization.yaml` 提供重定位参数。
唯一的节点入口为 `relocation_node`，唯一的启动文件为 `localization.py`。

## 重定位

`relocalization.algorithm_type` 支持：

- `bbs3d`：在先验地图中搜索初始位姿；缺少 `voxelmaps_coords/` 时可自动构建搜索索引。
- `scan_context`：检索已有描述子，需要 `trajectory.pcd` 和 `scancontext/`；仅用于重定位。
- `manually_set`：通过 `/initialpose` 接收初始位姿。

可通过 `relocalization.use_gnss` 和 `relocalization.gnss_topic` 启用地图坐标系下的 GNSS 位姿先验。
初始位姿经过配准验证后用于初始化滤波器，随后对固定地图持续定位。
BBS3D 搜索索引由已有地图生成，不是在线建图。

主要输出包括 `/lidar_localization`、`/lio/odom`、`/lio/path`、
`/lio/cloud_world` 和 `/lio/global_map`。

上游项目：https://github.com/Liansheng-Wang/Super-LIO （GPLv3，见 LICENSE）。
