# global_path_planner ROS2

基于拓扑地图的 ROS2 全局路径规划器。

## 主要订阅

- `/pose_stamped_localization` (`geometry_msgs/msg/PoseStamped`)：机器人实时定位位姿，参数 `pose_topic` 可配置。
- `clicked_point` (`geometry_msgs/msg/PointStamped`)：RViz 点击目标点。
- `/target_points` (`geometry_msgs/msg/Pose`)：任务状态机发布的目标点位姿。
- `/free_space_cloud` (`sensor_msgs/msg/PointCloud2`)：可行域点云，参数 `free_space_topic` 可配置。

## 主要发布

- `/global_path` (`nav_msgs/msg/Path`)：给 pure_pursuit_local_planner 跟踪的全局路径。

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
