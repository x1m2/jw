# global_path_planner ROS2

基于拓扑地图的 ROS2 全局路径规划器。

## 主要订阅

- `/pose_stamped_localization` (`geometry_msgs/msg/PoseStamped`)：机器人实时定位位姿，参数 `pose_topic` 可配置。
- `clicked_point` (`geometry_msgs/msg/PointStamped`)：RViz 点击目标点。
- `/target_points` (`geometry_msgs/msg/Pose`)：任务状态机发布的目标点位姿。

可行域点云不再通过话题订阅，节点启动时直接读取参数 `free_space_pcd_file` 指向的 PCD 文件。

## 主要发布

- `/global_path` (`nav_msgs/msg/Path`)：标准全局路径，用于平滑和可视化。
- `/global_path_with_policy` (`navigation_interfaces/msg/PathWithPolicy`)：路径以及与路径点对齐的停障策略。

## 拓扑边停障属性

边格式支持两个可选策略字段：

```text
edge_id from_id to_id weight type bidirectional obstacle_stop_enabled obstacle_avoidance_enabled
0 0 1 5.0 auto 1 1 0
1 1 2 6.0 auto 1 0 0
```

- `obstacle_stop_enabled=1`：该拓扑边开启雷达停障。
- `obstacle_avoidance_enabled=1`：该拓扑边开启PurePursuit主动绕障。
- 默认为`1 0`，即开启停障、关闭主动绕障。
- 文件缺少两个字段时仍按停障开启、主动绕障关闭处理。

## 编译

```bash
cd /root/robot_rk3588_ws
colcon build --packages-select global_path_planner
source install/setup.bash
```

## 启动

```bash
ros2 launch global_path_planner global_path_planner.launch.py
```

默认拓扑地图路径：

```bash
/root/slam_data/topology_map/generated_map.txt
```

如需修改，请编辑：

```bash
config/global_path_planner_params.yaml
```
