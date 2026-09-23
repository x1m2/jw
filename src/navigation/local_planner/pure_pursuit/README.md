# pure_pursuit

ROS 2 Humble 局部跟踪/绕障控制节点。当前导航里它负责接收路径和局部 costmap，输出 `/cmd_vel_planned`，并通过 `/nav_state` 告诉状态机当前目标点是否完成。最终 `/cmd_vel` 由 `obstacle_stop` 做安全检查后发布。

## 当前流程

```text
/livox/lidar
  -> costmap_2d -> /costmap

/pose_stamped_localization + /navigation/task_target
  -> global_path_planner -> /global_path
  -> path_smoother       -> /smoothed_global_path

pure_pursuit
  <- /smoothed_global_path_with_policy
  <- /costmap
  <- /navigation/task_target
  -> /cmd_vel_planned -> obstacle_stop -> /cmd_vel
  -> /nav_state
```

## 重规划等待

到点任务偏离重规划时，控制器订阅 `/navigation/global_replan_request`
（`std_msgs/Header`），立即停止旧路径跟踪，并持续输出零速度。收到时间戳
不早于请求、非空且非动态的新全局路径后，重置跟踪进度和绕障状态，发布
`/navigation/global_path_accepted` 确认，再解除重规划等待。队列中的旧路径
不能解除等待；新路径先到、停止请求后到时也不会被迟到请求再次锁停。

该等待状态与 `/pure_pursuit/set_paused` 的人工暂停独立：新路径到达不会
解除人工暂停，人工恢复也不会越过尚未完成的重规划。人工暂停状态通过
`/pure_pursuit/paused` 发布，供全局规划器避免在暂停期间触发重规划。

## 启动方式

完整导航：

```bash
./src/run_nav.sh
```

单独启动本节点：

```bash
ros2 launch pure_pursuit pure_pursuit.launch.py
```

## 主要话题

订阅：

```text
/smoothed_global_path_with_policy navigation_interfaces/msg/PathWithPolicy
/costmap          nav_msgs/msg/OccupancyGrid
/navigation/task_target    slam_manager_interfaces/msg/TaskTarget
```

发布：

```text
/cmd_vel_planned  geometry_msgs/msg/Twist
/nav_state        std_msgs/msg/Bool
/obstacle_stop/enabled std_msgs/msg/Bool
local_path        nav_msgs/msg/Path
local_path_astar  nav_msgs/msg/Path
local_astar_raw_path nav_msgs/msg/Path
local_astar_bezier_path nav_msgs/msg/Path
local_goal_marker visualization_msgs/msg/Marker
```

`/obstacle_stop/enabled`采用正向语义：`true`开启停障，`false`关闭停障。
当前路径点的`EdgePolicy.motion_direction=1`时保持正向跟踪；为`-1`时车头
对准路径反方向并输出负的`linear.x`。速度绝对值仍受同一`EdgePolicy`的
边速度和配置最大速度限制。

## 控制逻辑

```text
正常情况：
  跟踪 /smoothed_global_path

大角度偏航：
  先原地转向对准路径

前方障碍可绕：
  生成侧方临时点 -> 局部 A* 绕障 -> 找回归点 -> 回到全局路径

前方障碍不可绕：
  停车等待

当前目标不可达：
  发布 /nav_state=true，让状态机下发下一个目标点
```
