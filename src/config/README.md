# 机体雷达外参

建图、FAST-LIO 定位、Super-LIO 定位和避障通过公共包 `robot_geometry`
中的 `robot_geometry/lidar_extrinsics.hpp` 读取机体 YAML。该包仅提供头文件，
不启动节点；启动脚本仍通过 `robot_param_file` 或 `--params-file` 传入机体配置。
外参方向固定为 `p_base = R * p_lidar + t`：`translation` 是雷达原点在
base_link 中的位置，单位为米。前雷达配置为 `lidar_to_base`，后雷达为
`rear_lidar_to_base`。

MID360 使用弧度制 RPY：

```yaml
lidar_to_base:
  translation: [0.28, 0.0, 0.10]
  rotation:
    type: rpy_rad
    value: [0.0, 0.363, 0.0]  # roll, pitch, yaw，单位弧度
```

RPY 按 `R = Rz(yaw) * Ry(pitch) * Rx(roll)` 转换为旋转矩阵。
机体文件沿用原有弧度值，几何外参保持不变。

Airy 直接保存旋转矩阵，不经过 RPY。下面仅展示纯 pitch = 90° 的格式；
实际矩阵使用相应车型文件中的值：

```yaml
lidar_to_base:
  translation: [0.47, 0.0, -0.05]
  rotation:
    type: matrix
    value: [0.0, 0.0, 1.0,
            0.0, 1.0, 0.0,
           -1.0, 0.0, 0.0]
```

矩阵的九个元素按行排列。每份外参只选一种旋转表示；程序检查值的数量、
有限性以及矩阵正交性和行列式 +1，误差容许值为 `1e-5`，不会自动修正错误矩阵。
现有 Airy 前后雷达矩阵已按本次各自的 IMU 实测修正结果更新，须在对应机器上验收。

旧的 `lidar_to_base_T/R` 和 `rear_lidar_to_base_T/R` 不再使用，加载非空旧参数
会明确报错。完全未提供某份外参时，保留原有可选启动行为并打印警告；
建图和定位使用单位变换，避障的后雷达保留原默认位置。正式运行应指定完整机体配置。
仅提供部分新字段会报错。

`imu_calibration.extrinsic_R` 和算法配置里的 LiDAR/IMU 外参是独立参数。
`/robot/calibrate_lidar_tilt` 读取当前 `lidar_to_base` 作为朝向参考，用静止 IMU
修正倾斜，返回弧度制 roll、pitch 和按行排列的 `rotation_matrix`。
MID360 将 roll、pitch 填入 `rpy_rad`，保留原 yaw；Airy 直接填写返回矩阵。
日志中的 YAML 保留 6 位小数，内部与服务矩阵仍使用 double 精度。
标定要求底盘调平且静止，不独立标定 yaw 或平移，不自动修改 YAML。
Manager 配置中的 `imu_calibration.ground_fitting.enabled` 默认开启前雷达
2～5 m 地面拟合修正；MID360 与 Airy 共用流程，采集格式由机体配置的
`imu_calibration.pointcloud_type` 指定（`livox` / `pointcloud2`）。
地面模式需按实际 base_link 原点离地高度设置 `base_height_m`（正数，米），
`height_tolerance_m` 表示预期地面上下的取点容差（米）；验收失败不会退回纯 IMU 结果。
写回后须重启 Manager 以及使用该机体文件的建图、定位和避障节点。

修改后重新构建相关包并重启节点。部署时须同步更新机体 YAML 和节点程序，
不要混用旧程序与新配置。
