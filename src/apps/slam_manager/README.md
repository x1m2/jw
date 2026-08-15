# slam_manager

`slam_manager` 是 APP 与建图、定位和导航算法之间的统一 ROS 2
接口层。platform 模式下算法进程常驻，但算法对象由 Runtime 按任务启动和
停止；test 模式保持原有 launch 自动启动行为。

## APP接口

### Service

| 接口 | 类型 | 功能 |
| --- | --- | --- |
| `/robot/start_mapping` | `slam_manager_interfaces/srv/StartMapping` | 按`map_id`启动建图 |
| `/robot/finish_mapping` | `std_srvs/srv/Trigger` | 停止建图并保存地图 |
| `/robot/delete_map` | `slam_manager_interfaces/srv/DeleteMap` | 删除整个`<map_id>`目录 |
| `/robot/load_map` | `slam_manager_interfaces/srv/LoadMap` | 检查并记录后续重定位使用的地图 |
| `/robot/start_localization` | `slam_manager_interfaces/srv/StartLocalization` | 启动重定位，可选`x/y/z/theta`初始位姿 |
| `/robot/pause_navigation` | `std_srvs/srv/Trigger` | 暂停当前导航Action |
| `/robot/resume_navigation` | `std_srvs/srv/Trigger` | 继续当前导航Action |

`load_map`只检查`globalmap.pcd`并记录地图，不读取或发布PCD，也不启动定位
算法。`start_localization`使用最近一次`load_map`成功记录的地图。

`map_id`和巡检`task_id`均为字符串。APP下发完整地图标识，例如
`map_0`；Runtime不再自动补`map_`前缀。`map_id`必须以`map_`开头，
两种ID都只允许字母、数字、下划线和短横线，最长64个字符。

`StartLocalization.srv`使用四自由度初始位姿：

```srv
bool use_initial_pose
float64 x
float64 y
float64 z
float64 theta
---
bool success
string message
```

`use_initial_pose=false`时忽略`x/y/z/theta`。Runtime使用tf2将`theta`
转换为内部算法所需的四元数。

### Action

| 接口 | 类型 | 功能 |
| --- | --- | --- |
| `/robot/start_navigation` | `slam_manager_interfaces/action/StartNavigation` | 按APP节点数组执行巡检导航 |
| `/robot/navigate_to_pose` | `slam_manager_interfaces/action/NavigateToPose` | 执行单点导航 |

两个Action共用同一套导航Worker，同一时刻只接受一个导航任务。APP必须保存
Action返回的GoalHandle，并通过对应GoalHandle取消任务。取消完成后Action
返回`CANCELED`；失败或超时返回`ABORTED`；正常到达返回`SUCCEEDED`。

巡检节点使用：

```msg
uint32 node_id
float64 x
float64 y
float64 z
float64 theta
```

Runtime按节点数组顺序生成相邻双向边，保存为：

```text
<map目录>/task/patrol_<task_id>.txt
```

单点导航使用当前`map`位姿作为起点、APP目标作为终点，生成：

```text
<map目录>/task/single_point_navigation.txt
```

随后两种Action都复用原有全局规划、局部规划和控制链路。`theta`在下发到
内部算法时由tf2转换成四元数。

### Topic

| 接口 | 类型 | QoS | 功能 |
| --- | --- | --- | --- |
| `/robot/current_pose` | `slam_manager_interfaces/msg/RobotPose` | Reliable，KeepLast 20 | 发布`map`坐标下的`x/y/z/theta`和定位匹配分数 |
| `/robot/cloud_body` | `sensor_msgs/msg/PointCloud2` | Best Effort，KeepLast 20 | 转发实时机体点云 |
| `/robot/keyframe_cloud` | `slam_manager_interfaces/msg/MapKeyframe` | Reliable，KeepLast 100 | 转发新增地图关键帧 |
| `/robot/runtime_state` | `slam_manager_interfaces/msg/RuntimeState` | Reliable，Transient Local，KeepLast 1 | 发布运行状态和错误 |

`RobotPose.msg`：

```msg
float64 x
float64 y
float64 z
float64 theta
float32 match_score
```

所有位姿字段使用`map`坐标系，位置单位为米，`theta`表示绕Z轴的yaw角，
单位为弧度。`match_score`取值范围为0～1，越大表示FAST-LIO当前帧与
全局地图的匹配越好；建图阶段没有该分数时为0。

PGO只在新增关键帧时发布一次，不在回环后重发旧关键帧。APP实时拼接结果
用于建图预览，最终准确地图以`finish_mapping`保存的`globalmap.pcd`为准。

## 地图目录

默认根目录为`/robotfs/data/slam_data`：

```text
/robotfs/data/slam_data/
├── map_0/
│   ├── map/
│   │   ├── globalmap.pcd
│   │   └── manifest.yaml
│   ├── log/
│   └── task/
│       ├── patrol_0.txt
│       └── single_point_navigation.txt
└── .staging/
```

建图时先写入`.staging`；停止建图时先由PGO保存
`globalmap.pcd`，再基于该地图生成`voxelmaps_coords`。两者都完整后，
Runtime在同一文件系统内原子发布完整`<map_id>`目录。

## Worker控制

建图启动顺序为PGO、FastLIO，停止时先停止FastLIO数据生产，再由PGO保存
地图，然后生成BBS体素地图。

导航Worker按依赖顺序启动，Pure Pursuit最后启动；停止时Pure Pursuit最先
停止并发布零速度。任一Worker启动失败时，Runtime按逆序回滚已经启动的
Worker。

暂停导航只暂停Pure Pursuit速度输出，不销毁导航Worker；继续后重新计算
控制指令。取消Action会退出目标等待、停止全部导航Worker，并通过Action
Result确认最终停止状态。
