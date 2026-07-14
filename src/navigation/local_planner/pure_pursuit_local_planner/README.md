# pure_pursuit_local_planner

ROS 2 Humble 局部跟踪/绕障控制节点。当前导航里它负责接收路径和局部 costmap，输出 `/cmd_vel`，并通过 `/nav_state` 告诉状态机当前目标点是否完成。

## 当前流程

```text
/livox/lidar
  -> costmap_2d -> /local_costmap/costmap

/pose_stamped_localization + /target_points
  -> global_path_planner -> /global_path
  -> path_smoother       -> /smoothed_global_path

pure_pursuit_local_planner
  <- /smoothed_global_path
  <- /local_costmap/costmap
  <- /target_points
  -> /cmd_vel
  -> /nav_state
```

`pure_pursuit_local_planner.launch.py` 里做了 remap：

```text
/global_path   -> /smoothed_global_path
/local_costmap -> /local_costmap/costmap
```

## 启动

完整导航：

```bash
./src/run_nav.sh
```

单独启动本节点：

```bash
ros2 launch pure_pursuit_local_planner pure_pursuit_local_planner.launch.py
```

## 主要话题

订阅：

```text
/global_path      nav_msgs/msg/Path
/local_costmap    nav_msgs/msg/OccupancyGrid
/target_points    geometry_msgs/msg/Pose
/emergency_stop   std_msgs/msg/Bool
```

发布：

```text
/cmd_vel          geometry_msgs/msg/Twist
/nav_state        std_msgs/msg/Bool
local_path        nav_msgs/msg/Path
local_path_astar  nav_msgs/msg/Path
local_goal_marker visualization_msgs/msg/Marker
```

## 控制逻辑

```text
正常情况：
  跟踪 /smoothed_global_path

大角度偏航：
  先原地转向对准路径

前方硬停区有障碍：
  立即停车

前方障碍可绕：
  生成侧方临时点 -> 局部 A* 绕障 -> 找回归点 -> 回到全局路径

前方障碍不可绕：
  停车等待

当前目标不可达：
  发布 /nav_state=true，让状态机下发下一个目标点
```
