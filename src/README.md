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
├── tools/                 # 工具与用户界面
│   ├── pcd2pgm             # PCD 点云地图转 PGM 栅格地图
│   └── user_interface      # 导航任务状态机与控制界面
└── scripts/               # 辅助脚本
```

## 主要功能

### 1. 雷达驱动

`drivers/livox_ros_driver2` 用于接入 Livox 系列 3D LiDAR，主要为 FAST-LIO、PGO、定位和导航模块提供实时点云输入。

```bash
ros2 launch livox_ros_driver2 msg_MID360s_launch.py
```

### 2. FAST-LIO 前端建图

`slam/fast_lio` 负责激光惯性里程计估计和实时点云建图。

主要输出：

- `/Odometry`：前端里程计
- `/cloud_registered`：校正后的点云

启动示例：

```bash
ros2 launch fast_lio mapping.launch.py 
```

### 3. PGO + ScanContext 后端优化

`slam/pgo_sc` 在 FAST-LIO 前端基础上进行回环检测和位姿图优化。

主要功能：

- 使用 ScanContext 做回环候选检测
- 使用 Quatro 做全局粗配准
- 使用 Nano-GICP 做精配准
- 使用 GTSAM 做位姿图优化
- 支持保存 PCD 地图、bag 地图和 KITTI 格式轨迹

默认输入：

- `/Odometry`
- `/cloud_registered`

启动示例：

```bash
ros2 launch pgo_sc run.launch.py
```


### 4. 地图定位与重定位

`slam/localization_sc` 用于加载已有地图，并根据当前点云和里程计进行在线定位。

主要功能：

- 加载保存好的地图 bag
- 使用 ScanContext 做候选匹配
- 使用 Quatro 提供粗配准初值
- 使用 Nano-GICP 做精配准
- 输出机器人在 `map` 坐标系下的定位结果
- 支持重定位稳定性判断

默认地图路径配置在：

```text
slam/localization_sc/config/config.yaml
```

启动示例：

```bash
ros2 launch localization_sc run.launch.py
```

### 5. 全局路径规划

`navigation/global_planner/global_path_planner` 是基于拓扑地图的全局路径规划器。

主要订阅：

- `/pose_stamped_localization`：当前定位位姿
- `/clicked_point`：RViz 点击目标点
- `/target_points`：任务状态机发布的目标点
- `/free_space_cloud`：可行域点云

主要发布：

- `/global_path`：全局路径，供局部规划器跟踪

启动示例：

```bash
ros2 launch global_path_planner global_path_planner.launch.py
```

### 6. 拓扑地图生成

`navigation/global_planner/topology_map_generator` 用于手动记录拓扑点。

工作方式：

1. 订阅路径话题，例如 `/pgo/optimized_path`
2. 保存路径中的最新位姿
3. 收到 `/topology_node_set` 为 `true` 时记录一个拓扑点
4. 输出拓扑点文件和目标点文件

手动打点：

```bash
ros2 topic pub --once /topology_node_set std_msgs/msg/Bool "{data: true}"
```

默认输出：

- `generated_map.txt`
- `generated_map_target_points.txt`

### 7. 路径平滑

`navigation/global_planner/path_smoother` 用于对全局路径进行平滑处理，使路径更适合局部跟踪控制。

```bash
ros2 launch path_smoother path_smoother.launch.py
```

### 8. 局部规划与避障

`navigation/local_planner` 包含局部代价地图、Pure Pursuit 路径跟踪、可行区域提取和点云转激光等模块。

#### costmap_2d

生成机器人周围局部代价地图。

主要功能：

- 订阅 `/scan`
- 生成局部 `nav_msgs/OccupancyGrid`
- 支持障碍物膨胀
- 发布 `/costmap` 和 `/costmap_vis`

```bash
ros2 launch costmap_2d costmap_2d.launch.py
```

#### pure_pursuit_local_planner

Pure Pursuit 局部路径跟踪器，包含路径跟踪、局部规划和避障逻辑。

```bash
ros2 launch pure_pursuit_local_planner pure_pursuit_local_planner.launch.py
```

#### dual_radar_traversable_extractor

双雷达可行区域提取模块，用于从点云中提取可通行区域，供全局或局部规划使用。

#### pointcloud_to_laserscan

将 3D 点云转换为 2D LaserScan，方便接入基于 `/scan` 的代价地图或导航模块。

### 9. 地图转换工具

`tools/pcd2pgm` 用于将 `.pcd` 点云地图转换为 `.pgm` 栅格地图，便于导航模块使用。

```bash
ros2 launch pcd2pgm pcd2pgm_launch.py
```

### 10. 导航任务状态机

`tools/user_interface` 提供导航任务状态机和控制界面。

主要功能：

- 按任务列表依次发布目标点
- 监听导航完成状态
- 支持开始、停止、继续、回家、急停
- 发布任务动作信号
- 提供 GUI 控制脚本

主要订阅：

- `/nav_state`：是否到达目标点
- `/nav_task_signal`：任务控制信号

主要发布：

- `/task_action`：到点后发布的任务动作
- `/target_points`：当前目标点
- `/emergency_stop`：急停信号

启动状态机：

```bash
ros2 launch user_interface navigation_task_state_machine.launch.py
```

GUI 控制：

```bash
ros2 run user_interface navigation_control_gui.py
```

## 推荐运行流程

### 建图流程

```bash
# 1. 启动 Livox 雷达
ros2 launch livox_ros_driver2 msg_MID360s_launch.py

# 2. 启动 FAST-LIO 前端
ros2 launch fast_lio mapping.launch.py 

# 3. 启动 PGO + ScanContext 后端
ros2 launch pgo_sc run.launch.py
```

### 定位导航流程

```bash
# 1. 启动雷达
ros2 launch livox_ros_driver2 msg_MID360s_launch.py

# 2. 启动 FAST-LIO，提供实时里程计和点云
ros2 launch fast_lio mapping.launch.py 

# 3. 启动地图定位
ros2 launch localization_sc run.launch.py

# 4. 启动全局路径规划
ros2 launch global_path_planner global_path_planner.launch.py

# 5. 启动局部规划/路径跟踪
ros2 launch pure_pursuit_local_planner pure_pursuit_local_planner.launch.py

# 6. 启动任务状态机
ros2 launch user_interface navigation_task_state_machine.launch.py
```

## 主要依赖

- ROS2 Humble
- C++17
- PCL
- Eigen
- OpenMP
- Boost
- GTSAM
- TEASER++
- TBB
- Sophus
- Livox SDK / livox_ros_driver2
- RViz2

## 编译方式

建议先编译基础依赖包：

```bash
colcon build --packages-select quatro --cmake-args -DCMAKE_BUILD_TYPE=Release -DQUATRO_TBB=ON
```

再编译整个工作空间：

```bash
colcon build --symlink-install --parallel-workers 2 \
  --packages-skip quatro fast_lio pgo_sc localization_sc \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DROS_EDITION=ROS2 \
    -DHUMBLE_ROS=humble

colcon build --symlink-install --parallel-workers 1 \
  --packages-select fast_lio localization_sc pgo_sc \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

## 系统数据流

```text
Livox LiDAR / IMU
        |
        v
livox_ros_driver2
        |
        v
FAST-LIO
  |           |
  |           +--> /cloud_registered
  +--------------> /Odometry
        |
        v
pgo_sc 建图优化 或 localization_sc 在线定位
        |
        v
定位位姿 / 优化路径 / 地图
        |
        v
拓扑地图 + global_path_planner
        |
        v
/global_path
        |
        v
pure_pursuit_local_planner / costmap_2d / 避障模块
        |
        v
机器人速度控制
```

## PCD
LIBGL_ALWAYS_SOFTWARE=1 pcl_viewer result.pcd --no-vbo

## 手动保存地图：
ros2 service call /map_save std_srvs/srv/Trigger
ros2 topic pub --once /save_dir std_msgs/msg/Empty "{}" 

### 运控板 192.168.50.101 感归控板192.168.50.103
