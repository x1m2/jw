# terrain_filter

单/双雷达地形过滤节点。节点先保守分割普通地面，再跨越普通连通组件搜索
连续立面段，或通过踏面高度层验证级高一致且宽度足够的楼梯结构；无法确认
的点默认保留为障碍。

立面检测不要求与地面连接。只要存在至少三级连续立面，并通过级高、相邻
间距、方向和平行度的基本验证，即可建立楼梯模型；这使机器人靠近后
看不到地面或最低一级时，仍能识别剩余楼梯。
模型通过后会按拟合出的平均级高和进深向上继续匹配有限级数，并将短时
保存方向、宽度、级高和进深约三秒；当前帧三级立面不完整时，仍使用该
模型匹配踏面和立面，减少点在 `stair_cloud` 和 `obstacle_cloud` 间闪烁。
输入点云来自 `map` 或 `odom` 时，历史模型会通过逐帧 TF 更新到当前
`base_link`，补偿机器人靠近楼梯时的模型位置变化。
栅格模型通过后，还会在已识别楼梯的一格邻域内逐点复核踏面高度和立面
位置，用于补回被同格离群点影响而残留在 `obstacle_cloud` 中的楼梯点。

## 编译

```bash
cd ~/robotics_platform/robot_ws
colcon build --packages-select terrain_filter --symlink-install
source install/setup.bash
```

## 启动

单雷达模式：

```bash
ros2 launch terrain_filter terrain_filter.launch.py
```

双雷达模式：

```bash
ros2 launch terrain_filter terrain_filter.launch.py use_dual_radar:=true
```

## 参数文件

参数文件：

```bash
config/terrain_filter.yaml
```

ROS2 参数格式为：

```yaml
terrain_filter_node:
  ros__parameters:
    lidar_topic: "/livox/pointcloud2"
    base_frame: "base_link"
```

## 主要发布话题

- `ground_cloud`
- `stair_cloud`
- `obstacle_cloud`
- `combined_cloud`

`stair_cloud` 包含通过完整结构验证的可通行楼梯，以及在位置、踏面高度和
表面跨度上与其连续匹配的短时保持点。普通地面、楼梯和障碍物分别发布，
便于在 RViz 中检查误判。使用 `obstacle_cloud` 的下游节点不应再次按高度
裁剪，否则低矮障碍仍会丢失。

机器人靠近楼梯时，已确认表面还会在输入配准坐标系（如 `map`、`odom`）中
短期保存，并使用采集时刻的完整 TF 匹配后续观测，减少模型切换和扫描空隙
造成的楼梯点回到障碍点云。输入坐标系必须是固定参考系；输入已在
`base_link` 下、缺少运动参照时不启用这部分记忆。

- `stair_surface_history_duration`：记忆时长，默认 3 秒，设为 0 关闭。
- `stair_surface_history_distance`：到历史表面点的最大三维距离，默认 0.04 米。

仅当前结构或有效模型确认的点可以续期，单纯历史匹配不会续期，也不触发
整格扩张。时间倒退和坐标系改变会清理表面记忆。日志中的 `表面记忆` 是
本帧通过这条路径补回的点数。距离阈值内的微小凸起仍可能与表面噪声混淆，
不能用扩大此阈值的方式追求楼梯区域内障碍点完全清零。

## 说明

原 ROS1 版本中的 `ros::NodeHandle`、`ros::Subscriber`、`ros::Publisher`、`ROS_INFO`、`roslaunch` 等已替换为 ROS2 写法。PatchWork++ 头文件中的 ROS1 参数读取和时间 API 也已改为 ROS2 参数系统和 `rclcpp::Time`。
