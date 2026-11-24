# ROS 2 离线建图

在 `robot_slam_ws` 下运行。入口使用该工作区的 `install/` 和 `src/` 配置，前端和后端节点都在独立结果目录中启动。

```bash
# 前 Airy + 内置 IMU
bash offline_mapping/run_offline_mapping.sh --lidar airy --bag /data/datasets/data/Airy/123

# 前 Airy + Woodox 本体 IMU，默认时间补偿 +0.003 s
bash offline_mapping/run_airy123_front_extimu.sh

# 前后 Airy 融合 + 前雷达 IMU，固定 Airy/123 已测时差
bash offline_mapping/run_offline_dual_mapping.sh --bag /data/datasets/data/Airy/123 --delta -0.000001431

# MID360
bash offline_mapping/run_offline_mapping.sh --lidar mid360 --bag /data/datasets/data/Mid360s/20261123
```

默认以 1 倍速回放并打开 RViz。带 `--no-rviz` 可关闭显示，带 `--prepare-only` 可检查数据和配置。双雷达省略 `--delta` 时在回放开始后自动估计时差。结果目录由终端打印，位于 `offline_mapping/runs/`。地图保存为该次结果的 `map/globalmap.pcd`。

`config/airy_woodox.yaml` 保存本体 IMU 模式的空间外参。`AIRY_IMU_TIME_OFFSET` 可以覆盖该模式的时间补偿。Airy 的 SQLite 回放副本和生成的 YAML 随结果目录保存。
