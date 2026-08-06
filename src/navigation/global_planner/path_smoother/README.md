# path_smoother ROS2

这是迁移后的 ROS2 版本路径平滑节点。

## 编译

```bash
cd ~/robotics_platform/robot_ws
colcon build --packages-select path_smoother
source install/setup.bash
```

## 启动

```bash
ros2 launch path_smoother path_smoother.launch.py
```

## 默认话题

- 订阅：`/global_path`，类型 `nav_msgs/msg/Path`
- 发布：`/smoothed_global_path`，类型 `nav_msgs/msg/Path`
- 发布：`/path_visualization`，类型 `visualization_msgs/msg/MarkerArray`

## 常用参数

- `global_frame`: 默认 `map`
- `smoothing_factor`: 默认 `1.0`
- `smoothing_iterations`: 默认 `3`
- `min_point_distance`: 默认 `0.5`
- `max_deviation`: 默认 `1.5`
- `enable_visualization`: 默认 `true`
- `enable_point_filtering`: 默认 `true`
