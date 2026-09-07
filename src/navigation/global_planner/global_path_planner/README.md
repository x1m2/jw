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

- 边是有向边；双向通行需要配置两条方向相反的边。
- APP导航协议使用相邻节点，因此一次正常导航只执行一条普通边。
- `straight`边按直线生成路径。
- `bezier`边使用起点、两个控制点和终点生成三次贝塞尔曲线；反向经过
  可双向通行的贝塞尔边时会自动交换两个控制点。
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

## 启动

```bash
ros2 launch global_path_planner global_path_planner.launch.py
```

参数文件：

```text
config/global_path_planner_params.yaml
```
