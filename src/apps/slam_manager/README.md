# slam_manager

`slam_manager`负责连接APP与建图、定位、导航模块，并管理各算法Worker的
启动、停止和任务状态。

## 地图流程

```text
APP /robot/start_mapping
  -> 记录map_id和未来保存路径（不创建目录、不覆盖同名地图）
  -> 启动PGO和FastLIO
  -> 开始建图

APP /robot/finish_mapping
  -> 创建map/task目录（同名地图在此时整体覆盖）
  -> 停止FastLIO
  -> 显式调用PGO保存globalmap.pcd
  -> 停止PGO
  -> 发布到 /robotfs/data/slam_data/<map_id>/map/

APP /robot/cancel_mapping
  -> 停止FastLIO和PGO
  -> 跳过PGO地图保存
  -> 不创建地图目录，回到IDLE

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
  -> Action执行期间每秒反馈elapsed_time和当前阶段message
  -> 60秒内成功则返回SUCCEEDED，定位算法继续运行
  -> 失败或超时返回ABORTED并停止定位
  -> APP取消则返回CANCELED、停止定位并回到IDLE
```

APP拼接地图时，只需将已经转换到统一`map`坐标系的PCD保存为
`globalmap.pcd`。机器人执行`load_map`后会重新生成对应的3D-BBS数据，
定位流程不需要区分原始地图和拼接地图。

## 巡检导航流程

开始巡检或单点导航前，`slam_manager`会调用
`/GetState`（`woodox_ros2_interface/srv/GetState`）。只有响应为
`state: Locomotion`且`code: 0`时才启动导航Worker。服务不可用、响应超时或
机器人处于其他状态时，Action将以`ABORTED`结束，`result.message`包含实际
`state`和`code`，同时节点日志输出`[NAVIGATION_GATE]`状态记录。

状态检查通过后，启动导航前调用一次`/SetMode`并发送`mode: 1`。导航正常结束、
启动失败、运行失败或取消时调用一次`/SetMode`并发送`mode: 0`；不进行周期调用。
服务响应通过`[NAVIGATION_MODE]`日志记录，设置`mode: 1`失败时不会启动导航Worker。
暂停导航成功后同样发送一次`mode: 0`；继续导航时先发送一次`mode: 1`，成功后
才恢复Pure Pursuit速度输出。继续失败时会回退发送`mode: 0`。配置
`navigation.enable_set_mode: false`可统一禁用上述`/SetMode`调用。

`navigation.default_edge_policy`提供全局默认边策略：
`obstacle_avoidance_enabled`和`obstacle_avoidance_distance_m`。
任务JSON边中已提供的字段覆盖这些默认值，
区域策略再覆盖当前边策略，因此优先级为“区域 > 边 > YAML”。其中
`obstacle_avoidance_distance_m`当前仅完成读取和路径策略传递，尚未接入具体
绕障算法。`obstacle_stop_enabled`不再写入任务JSON，也不从YAML或任务JSON
读取；导航始终将其设为`true`。

```text
APP task/task.json -------------------┐
                                      ├-> slam_manager生成topology_map.json
StartNavigation.nodes[]巡检节点顺序 --┘
                                                |
                                                v
slam_manager逐个发布 /navigation/task_target
  -> global_path_planner规划
  -> path_smoother平滑
  -> Pure Pursuit执行
  -> /nav_state返回到达状态
  -> 节点停留
  -> 发布下一个节点

A -> B为map_connection时：
  -> 到达A后保持导航Workers运行，不向B发布运动目标
  -> 以B节点x/y/z/theta重启当前地图的定位Worker
  -> 发布B位姿到/initialpose并等待/reloc_success=true
  -> 逻辑完成B
  -> 发布B之后的下一个普通节点
```

任务文件路径：

```text
/robotfs/data/slam_data/<map_id>/task/<task_id>.json
```

处理规则：

- `StartNavigation.nodes[]`决定节点执行顺序；它是“执行序列”，允许同一个
  `node_id`多次出现，例如`0,1,2,3,4,5,4,3`。任务JSON中的`nodes[]`仍是
  拓扑定义，`node_id`必须唯一，不会因为重复巡检而复制节点。
- Action结果中的`completed_count`按已完成的执行序列项计数，同一个`node_id`
  第二次经过会再次计数，Feedback中的`current_node_id`也允许重复出现。
- 首次执行任务时，若当前二维位置距离首节点不超过
  `navigation.skip_first_target_threshold`（默认`0.8 m`），且任务至少有
  两个节点，则跳过首节点并从第二个节点开始；循环任务的后续轮次仍从首节点
  开始。可通过`navigation.enable_skip_first_target`关闭该行为。
- 根据`StartNavigation` Goal中的`task_id`读取`<task_id>.json`，并按
  `node_id`查询节点位姿和停留时间。
- JSON中没有该节点时，使用Action携带的`x/y/z/theta`和默认策略。
- `NavigationNode.stop_duration_sec`表示到点后的停留秒数；JSON命中节点
  时使用JSON值，JSON中没有该节点时使用Action消息中的值。
- `<task_id>.json`不存在时，直接使用Action携带的`nodes[]`位姿和停留时间
  继续导航，并为相邻节点生成默认双向直线边；该模式不加载自定义边和区域设置，
  边参数使用`TaskEdgePolicy`默认值。
- 任务JSON中的edge支持`bidirectional`属性，未配置时slam_manager按当前业务约定
  默认`true`。生成runtime `topology_map.json`时会明确写出该字段。相邻执行节点
  优先使用与当前方向完全一致的edge；若只存在反方向定义，则仅当该edge的
  `bidirectional=true`时复用同一条edge，不生成新的runtime反向edge，也不修改原始
  任务JSON。例如JSON只有`4->5`且为双向时，Action可以直接执行`5->4`。实际遍历
  方向由`TaskTarget.from_node_id -> target.node_id`确定；Bezier反向遍历时由全局规划器
  反向使用曲线几何。反向复用双向边时，该次到点朝向使用目标节点JSON中的
  `theta + π`并归一化到`[-π, π]`；只调整本次Action中的节点实例，不修改JSON或
  runtime拓扑节点。因此例如JSON路径为`0,1,2,3`，Action执行`0,1,2,3,2,1`时，
  后两个目标节点`2,1`分别使用其JSON朝向加180度。显式配置`bidirectional=false`
  可保留单向边语义。
- 单点导航和巡检统一通过`/navigation/task_target`发送`slam_manager_interfaces/TaskTarget`。
  `use_topology=false`按坐标规划；`use_topology=true`按拓扑ID规划。巡检消息
  携带上一已完成节点ID，并复用`NavigationNode`传递当前目标。
  全局规划器复用指定边的直线/贝塞尔插值和边策略，不因其他节点更近而替换路段；
  首次接入保留原有起点搜索，目标按ID匹配。跳过首点或完成跨地图重定位后，
  使用对应节点作为下一段起点。局部规划器沿用原有目标朝向和Bool到达反馈。
- 校验和合并完成后，原子写入`task/topology_map.json`。
- `map_connection`表示物理位置相同、统一`map`坐标发生跳转的虚拟连接。
  后一个节点不作为运动目标，而是作为重定位初始位姿；只有重定位成功并重新
  启动导航Workers后才逻辑完成，并执行该节点的`stop_duration_sec`。
- `map_connection`重定位继续使用当前`map_id`的PCD和已生成BBS体素，不重新
  执行`load_map`或生成体素。
- 普通节点到达后按`stop_duration_sec`等待，再执行下一个节点。

边策略随本次相邻节点导航下发：

- `linear_velocity`、`angular_velocity`：本次导航速度上限。
- `motion_direction=1`：前进；`motion_direction=-1`：倒车。
- 停障固定启用，不再作为边策略字段保存。
- `obstacle_avoidance_enabled`：是否启用绕障。

### 导航区域

任务JSON可选包含`regions[]`。未提供时保持原导航行为；第一版不允许两个
启用区域相交或接触。每个区域包含唯一`id`、`region_type`、至少三个
`map`坐标系多边形点，以及可选`param`：

```json
{
  "id": 0,
  "region_type": "obstacle_avoidance",
  "polygon": [
    {"x": 1.0, "y": 1.0},
    {"x": 3.0, "y": 1.0},
    {"x": 3.0, "y": 3.0}
  ],
  "param": {
    "linear_velocity": 0.2,
    "angular_velocity": 0.3,
    "obstacle_avoidance_distance_m": 1.0
  }
}
```

`region_type`支持`obstacle_avoidance`、`speed_limit`和`stop`。区域参数只
覆盖已提供的字段，缺失字段继承当前边；离开区域后恢复边策略。

每个区域可在任务目录下提供狗本体停障配置：

```text
task/region_obstacle_stop/<task_id>/<region_id>.yaml
```

文件只允许覆盖`hard_stop_distance`、`slowdown_distance`、
`obstacle_ratio_threshold`、`linear_velocity_factor`和
`angular_velocity_factor`。文件不存在时使用`obstacle_stop.yaml`默认值。
`slam_manager`只校验区域ID、类型、几何和区域参数，不读取停障YAML；
`obstacle_stop`启动时根据运行时拓扑中的区域ID直接读取并校验对应YAML，
再根据机器人实时`map`位姿切换配置。已存在但内容非法的YAML会使停障Worker
启动失败并中止本次导航。

单点导航`/robot/navigate_to_pose`也会生成`topology_map.json`，然后使用相同
的全局规划、平滑和控制流程。
Action Goal 中的 `enable_avoidance` 控制本次单点导航是否开启绕障，
默认 `false`。该值写入生成拓扑的双向边策略，全局路径及平滑路径携带该策略，
由局部规划器执行；不改变原有遇障停车设置。

```bash
ros2 action send_goal /robot/navigate_to_pose slam_manager_interfaces/action/NavigateToPose \
  "{x: 1.0, y: 2.0, z: 0.0, theta: 0.0, enable_avoidance: true}"
```

## 运行控制

- 同一时刻只执行一个导航Action。
- 暂停只停止Pure Pursuit速度输出，不销毁导航Worker。
- 取消任务会中断到点等待或`map_connection`重定位，停止相关Worker并返回
  `CANCELED`。
- `load_map`、`start_localization`等生命周期命令会先停止正在执行的导航；
  旧导航Action返回`ABORTED`，不会被误报为客户端取消。
- 正常完成返回`SUCCEEDED`，规划失败或超时返回`ABORTED`。

## 地图目录

`map_id`长度为1～64，可使用字母、数字、`_`和`-`，不再强制
`map_`前缀。`start_mapping`只记录保存路径，不创建目录；
`finish_mapping`才创建`map`和`task`子目录，同名目录此时整体覆盖。
`cancel_mapping`不创建目录，也不保存任何建图结果。`load_map`和
`delete_map`使用相同的`map_id`规则。当前选中的地图在空闲状态可以删除，
删除后会清除地图选择、定位结果和当前位姿；重定位或导航期间不能删除。
`log`为运行日志保留目录，不能作为
`map_id`。

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
