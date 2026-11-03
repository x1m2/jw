# global_path_planner

`global_path_planner`读取统一拓扑JSON，根据机器人当前位置和目标点生成
全局路径及当前边的导航策略。

## 规划流程

```text
topology_map.json + globalmap.pcd
               + /pose_stamped_localization
               + /navigation/task_target
                         |
                         v
              加载节点、边和可行域点云
                         |
                         v
             连接当前位置、目标点与拓扑图
                         |
                         v
                   Dijkstra选边
                         |
                         v
             生成直线路径和当前边策略
                         |
              ┌----------┴----------------┐
              v                           v
        /global_path          /global_path_with_policy
```

默认拓扑文件：

```text
/robotfs/data/slam_data/<map_id>/task/topology_map.json
```

该文件由`slam_manager`或RViz拓扑编辑器生成，规划器只读取
`schema_version=1`、`format=navigation_topology`格式。

## 边处理规则

- 每条边的方向语义由`topology_map.json`中的`bidirectional`决定：`true`表示
  同一条edge可正反向通行，`false`表示只允许`from_id -> to_id`。旧JSON未包含
  该字段时按`false`处理，以保持历史兼容。
- APP导航协议使用相邻节点，因此一次正常导航只执行一条普通边。
- `straight`边按直线生成路径。
- `bezier`边使用起点、两个控制点和终点生成三次贝塞尔曲线；反向经过
  可双向通行的贝塞尔边时会自动交换两个控制点。
- 路径按 `interpolation_spacing_m` 采样，默认每 0.2 m 一个间隔；直线按
  端点距离计算，贝塞尔曲线按近似弧长计算。参数可在配置 YAML 中调整。
- 搜索完成后，按原始边的 `abs(end.z - start.z)` 判断高度插点：
  `path_interpolation.enable` 默认 `true`，`z_threshold` 默认 `0.5 m`，
  `insert_count` 默认 `2`（参数前缀均为 `path_interpolation.`）。高度差严格
  超过阈值且数量为 2 时，直线边按“平台—坡段—平台”生成高度：
  `start_offset`、`end_offset` 默认各 `1.0 m`，按 XY 水平距离从两端向内
  取点，第一个点保持起点 z，第二个点保持终点 z，再分三段密集插值。
  起终点 XY 连线不变，上下坡均适用。两个偏移之和必须小于边的水平长度；
  短边或竖直边回退到等比例插值并打印 `reason=short_edge`，不生成交叉点。
  偏移为 0 的点与端点重合，不重复添加。贝塞尔边保持原曲线，其他插点数量
  沿用原来的均匀采样规则。原始端点、拓扑节点及任务目标不变；每次均从原始边
  重建，不会因重复规划递归增点。关闭或数量设为 0 时沿用原有采样。
  `path_z_offset` 仍在最后统一应用一次。此功能假设偏移处对应实际楼梯入口、
  出口，不自动识别地形；下游平滑/降采样也不保证完整保留折点或原高度形状。
  可在 RViz2 中显示 `/global_path` 和 `/smoothed_global_path`
  对比，并通过 `[path_interpolation]` 日志查看新增数量。
- 起点连接第二层邻居时，如果该临时连接替代了一条真实`bezier`边，会继承
  该边的两个控制点，避免相邻节点逐段导航时退化为直线。
- `map_connection`不加入物理规划邻接表，也不生成路径；其两端允许在统一
  `map`坐标中相距较远，由`slam_manager`到达前一个节点后使用后一个节点
  位姿执行重定位。
- 普通边的速度、运动方向和停障/绕障参数统一封装为`EdgePolicy`，并通过
  `PathWithPolicy.edge_policies[]`与路径点逐点对齐。
- `linear_velocity`、`angular_velocity`最终不会超过Pure Pursuit配置的最大值。
- `motion_direction=1`输出正线速度，`motion_direction=-1`输出负线速度。

## 区域策略

运行时拓扑可选包含互不重叠的`regions[]`。规划器对每个插值路径点执行
多边形包含判断：区域外保留当前边`EdgePolicy`；区域内只用`param`中实际
提供的`linear_velocity`、`angular_velocity`和
`obstacle_avoidance_distance_m`覆盖边策略。`obstacle_avoidance`区域自动
开启主动绕障，`stop`区域开启停障并关闭主动绕障，`speed_limit`只做参数
覆盖。生成的策略仍与`PathWithPolicy.path.poses[]`逐点对齐。

生成的路径交给`path_smoother`平滑；平滑器保持当前边策略不变，最终由
Pure Pursuit执行。

## 到点任务偏离重规划

坐标到点任务（`TaskTarget.use_topology=false`）和 RViz `/clicked_point`
目标支持自动重规划。机器人离开当前路径后，以最新定位作为临时起点，
保留任务终点，重新连接拓扑图并搜索路线。拓扑巡检不启用此功能，仍执行
任务指定的拓扑节点/边；重规划路线可以复用旧路线的后半段，不保证绕开旧路。

偏离距离使用 XY 点到路径线段的最短距离。有同版本的平滑路径时使用平滑
结果，避免把正常弯道平滑视为偏离。默认条件如下：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `deviation_replan_enabled` | `true` | 自动偏离重规划开关 |
| `deviation_replan_distance` | `0.5` | 超过此距离开始确认偏离，米 |
| `deviation_replan_confirm_sec` | `0.5` | 持续偏离时间，秒 |
| `deviation_replan_interval_sec` | `1.0` | 连续规划尝试的最小间隔，秒 |
| `deviation_replan_goal_tolerance` | `0.2` | 终点附近不触发重规划，米；应与控制器到点容差一致 |
| `deviation_replan_pose_timeout_sec` | `0.5` | 定位时间戳最大允许偏差，秒 |


内部协调话题（不直接控制底盘）：

- `/navigation/global_replan_request`：`std_msgs/Header`，`stamp` 表示允许恢复
  跟踪的最早路径版本，`frame_id` 为全局坐标系。
- `/navigation/global_path_accepted`：`std_msgs/Header`，控制器实际接收的全局
  路径版本。全局规划器持续等待匹配确认；旧版本确认不能解除等待。
- `/scan_planner/paused`：`std_msgs/Bool`，控制器的人工/上层暂停状态。

普通全局路径仍使用 `dynamic_path=false`。平滑器必须保留源路径的 header
时间戳；控制器会拒绝等待期间排队的旧路径和空路径。


## 启动

```bash
ros2 launch global_path_planner global_path_planner.launch.py
```

参数文件：

```text
config/global_path_planner_params.yaml
```
