# global_path_planner

`global_path_planner`读取统一拓扑JSON，根据机器人当前位置和目标点生成
全局路径及当前边的导航策略。

## 规划流程

```text
topology_map.json + globalmap.pcd
               + /pose_stamped_localization
               + /target_points
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
- `straight`和当前尚未实现几何的其他普通类型统一按直线生成路径。
- `map_connection`不加入物理规划邻接表，也不生成路径；其两端允许在统一
  `map`坐标中相距较远，由`slam_manager`到达前一个节点后使用后一个节点
  位姿执行重定位。
- 普通边的速度、运动方向和停障/绕障参数统一封装为`EdgePolicy`，并通过
  `PathWithPolicy.edge_policies[]`与路径点逐点对齐。
- `linear_velocity`、`angular_velocity`最终不会超过Pure Pursuit配置的最大值。
- `motion_direction=1`输出正线速度，`motion_direction=-1`输出负线速度。

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
