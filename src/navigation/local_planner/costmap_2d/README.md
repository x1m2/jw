# Local Planner Package

这是一个独立的局部路径规划器包，不依赖于move_base框架。

## 功能模块

### 1. CostMap2D节点
- 订阅激光扫描数据 (`/scan`)
- 生成以机器人为中心的局部代价地图
- 支持障碍物膨胀
- 发布代价地图供路径规划使用

## 包结构

```
local_planner/
├── include/local_planner/
│   └── costmap_2d.h          # 代价地图类头文件
├── src/
│   ├── costmap_2d.cpp        # 代价地图实现
│   └── costmap_2d_node.cpp   # 节点主程序
├── launch/
│   └── costmap_2d.launch     # 启动文件
├── config/
│   └── costmap_params.yaml   # 参数配置
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 使用方法

### 1. 编译包
```bash
cd ~/WS_ROS1/ws_go2_real
catkin_make
source devel/setup.bash
```

### 2. 启动costmap_2d节点
```bash
roslaunch local_planner costmap_2d.launch
```

### 3. 查看话题
```bash
# 查看发布的话题
rostopic list | grep costmap

# 查看代价地图
rostopic echo /costmap
```

## 主要话题

### 订阅的话题
- `/scan` (sensor_msgs/LaserScan) - 激光扫描数据

### 发布的话题
- `/costmap` (nav_msgs/OccupancyGrid) - 代价地图
- `/costmap_vis` (nav_msgs/OccupancyGrid) - 可视化用代价地图

## 参数说明

### 坐标系参数
- `global_frame`: 全局坐标系 (默认: "map")
- `robot_base_frame`: 机器人基座坐标系 (默认: "base_link")
- `scan_topic`: 激光话题名 (默认: "/scan")

### 地图参数
- `resolution`: 分辨率 m/pixel (默认: 0.05)
- `width`: 宽度 pixels (默认: 400)
- `height`: 高度 pixels (默认: 400)

### 障碍物参数
- `max_obstacle_range`: 最大检测距离 m (默认: 10.0)
- `min_obstacle_range`: 最小检测距离 m (默认: 0.1)
- `inflation_radius`: 膨胀半径 m (默认: 0.5)

## 代价值定义

- `0`: 自由空间 (FREE_SPACE)
- `80`: 膨胀障碍物 (INSCRIBED_INFLATED_OBSTACLE)
- `100`: 致命障碍物 (LETHAL_OBSTACLE)
- `-1`: 未知区域 (NO_INFORMATION)

## 可视化

在RViz中添加:
1. Map display，话题设置为 `/costmap`
2. 坐标系设置为 `map`

## 依赖

- ROS Noetic
- sensor_msgs
- nav_msgs
- geometry_msgs
- tf2
- PCL

## 注意事项

1. 确保TF树正确设置 (map -> base_link)
2. 激光扫描数据正常发布
3. 代价地图会自动以机器人为中心更新
4. 地图尺寸为 20m x 20m (可在参数中调整)

## 扩展计划

- [ ] 路径规划器节点
- [ ] 速度控制器节点
- [ ] 动态窗口算法 (DWA)
- [ ] 轨迹跟踪
