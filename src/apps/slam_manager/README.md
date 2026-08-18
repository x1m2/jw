# slam_manager

`slam_manager`负责连接APP与建图、定位、导航模块，并管理各算法Worker的
启动、停止和任务状态。

## 地图流程

```text
APP /robot/start_mapping
  -> 启动PGO和FastLIO
  -> 开始建图

APP /robot/finish_mapping
  -> 停止FastLIO
  -> PGO保存globalmap.pcd
  -> 发布到 /robotfs/data/slam_data/<map_id>/map/

APP /robot/load_map
  -> 内部状态IDLE切换为LOADING_MAP
  -> 检查globalmap.pcd
  -> 删除旧voxelmaps_coords
  -> 根据当前PCD重新生成3D-BBS体素
  -> 记录当前加载地图
  -> 内部状态恢复为IDLE

APP /robot/start_localization
  -> 使用最近一次load_map成功的地图启动定位
  -> use_initial_pose=true时使用APP给出的x/y/z/theta
```

APP拼接地图时，只需将已经转换到统一`map`坐标系的PCD保存为
`globalmap.pcd`。机器人执行`load_map`后会重新生成对应的3D-BBS数据，
定位流程不需要区分原始地图和拼接地图。

## 巡检导航流程

```text
APP task/task.json -------------------┐
                                      ├-> slam_manager生成topology_map.json
StartNavigation.nodes[]巡检节点顺序 --┘
                                                |
                                                v
slam_manager逐个发布 /target_points
  -> global_path_planner规划
  -> path_smoother平滑
  -> Pure Pursuit执行
  -> /nav_state返回到达状态
  -> 节点停留
  -> 发布下一个节点
```

任务文件路径：

```text
/robotfs/data/slam_data/<map_id>/task/task.json
```

处理规则：

- `StartNavigation.nodes[]`决定节点执行顺序。
- 按`node_id`查询`task.json`；存在时使用JSON中的节点位姿和停留时间。
- JSON中没有该节点时，使用Action携带的`x/y/z/theta`和默认策略。
- `task.json`不存在时，按Action节点顺序生成默认直线边。
- 相邻的两个JSON节点必须存在唯一的直接有向边，不使用多边间接路径。
- 校验和合并完成后，原子写入`task/topology_map.json`。
- `map_connection`表示无物理运动的虚拟连接，后一个节点直接逻辑完成，
  不发布运动目标，也不执行停留。
- 普通节点到达后按`stop_duration_sec`等待，再执行下一个节点。

边策略随本次相邻节点导航下发：

- `linear_velocity`、`angular_velocity`：本次导航速度上限。
- `motion_direction=1`：前进；`motion_direction=-1`：倒车。
- `obstacle_stop_enabled`：是否启用停障。
- `obstacle_avoidance_enabled`：是否启用绕障。

单点导航`/robot/navigate_to_pose`也会生成`topology_map.json`，然后使用相同
的全局规划、平滑和控制流程。

## 运行控制

- 同一时刻只执行一个导航Action。
- 暂停只停止Pure Pursuit速度输出，不销毁导航Worker。
- 取消任务会中断等待、停止导航Worker并返回`CANCELED`。
- 正常完成返回`SUCCEEDED`，规划失败或超时返回`ABORTED`。

## 地图目录

```text
/robotfs/data/slam_data/<map_id>/
├── map/
│   ├── globalmap.pcd
│   └── manifest.yaml
├── log/
└── task/
    ├── task.json
    └── topology_map.json
```
