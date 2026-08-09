# robot_ws src 工作空间说明

本目录是一个基于 ROS2 的机器人建图、定位、导航与任务控制工作空间，整体链路围绕 Livox 雷达、FAST-LIO 前端里程计、ScanContext 回环/定位、拓扑导航、局部规划和任务状态机组成。

## 目录结构

```text
src
├── depd/                  # 算法依赖库
│   ├── nano_gicp           # 高效 GICP 点云精配准库
│   └── Quatro              # 全局粗配准库，用于回环和重定位初值
├── drivers/               # 传感器驱动
│   └── livox_ros_driver2   # Livox 雷达 ROS2 驱动
├── slam/                  # SLAM、后端优化、定位
│   ├── fast_lio            # 激光惯性里程计/建图前端
│   ├── pgo_sc              # ScanContext 回环检测 + GTSAM 位姿图优化
│   └── localization_sc     # 基于已有地图的 ScanContext 定位/重定位
├── navigation/            # 导航规划
│   ├── global_planner      # 拓扑地图、全局路径规划、路径平滑、RViz 工具
│   └── local_planner       # 局部代价地图、Pure Pursuit、避障、点云转激光
├── test/                  # 独立测试节点与工具
│   ├── rviz_topology_editor
│   └── user_interface      # 独立导航任务状态机
├── tools/                 # 通用工具
└── scripts/               # 辅助脚本
```

## 编译方式

建议先编译基础依赖包：

```bash
colcon build --packages-select quatro --cmake-args -DCMAKE_BUILD_TYPE=Release -DQUATRO_TBB=ON
```

再编译整个工作空间：

```bash
colcon build --symlink-install --parallel-workers 2 \
  --packages-skip quatro bbs3d fast_lio pgo_sc  fastlio_localization localization_sc \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DROS_EDITION=ROS2 \
    -DHUMBLE_ROS=humble

colcon build --symlink-install --parallel-workers 1 \
  --packages-select fast_lio fastlio_localization pgo_sc  robot_runtime_test \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

## PCD
LIBGL_ALWAYS_SOFTWARE=1 pcl_viewer globalmap.pcd --no-vbo

## 手动保存地图：
ros2 service call /map_save std_srvs/srv/Trigger
ros2 topic pub --once /save_dir std_msgs/msg/Empty "{}" 

### 运控板 192.168.50.101 感归控板192.168.50.103
