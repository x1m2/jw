# robot_runtime

`robot_runtime` 是机器人运行模式和算法 Worker 的统一控制包。platform 模式下，各算法二进制进程可以随系统启动，但算法对象默认不创建；`robot_runtime_node` 根据上层指令按顺序启动或销毁算法对象。

## 运行模式

- `runtime_profile=platform`：默认模式。只运行 `ManagedWorker`，等待 `~/set_state`。
- `runtime_profile=test`：自动创建算法对象，用于原有 launch 调试。

算法参数 YAML 不需要拆成 test/platform 两份。现有 launch 文件会额外传入：

```yaml
runtime_profile: test
```

直接运行安装后的二进制时没有该参数，因此默认进入 `platform`。

## 主要组件

- `ManagedWorker`：管理单个算法实例的创建、executor 线程、停止钩子和析构。
- `WorkerClient`：异步调用 Worker 的 `SetWorkerState` 服务。
- `RobotStateMachine`：限制建图、定位就绪和导航之间的合法状态流转。
- `MappingController`：控制 PGO、FastLIO 的启动、停止和失败回滚。
- `LocalizationController`：控制 FastLIO Localization 的启动和停止。
- `NavigationController`：控制导航模块的有序启动、停止和失败回滚。
- `RobotRuntimeNode`：面向 MQTT 转换层或其他上层应用提供统一 ROS 2 服务。

## 管理服务

```text
/robot/start_mapping
/robot/stop_mapping
/robot/load_map
/robot/delete_map
/robot/start_localization
/robot/stop_localization
/robot/start_navigation
/robot/pause_navigation
/robot/resume_navigation
/robot/stop_navigation
/robot/reset_error
```

携带地图 ID 的服务：

- `/robot/start_mapping`：`robot_interfaces/srv/StartMapping`
- `/robot/load_map`：`robot_interfaces/srv/LoadMap`
- `/robot/delete_map`：`robot_interfaces/srv/DeleteMap`

其余服务（包括 `/robot/start_localization`）为 `std_srvs/srv/Trigger`。`/robot/runtime_state` 发布当前系统模式。

调用 `/robot/start_localization` 后，运行时会启动
`/fastlio_localization_worker/set_state`，并保持Service请求等待。定位模块发布
`/reloc_success=true` 后，系统从 `localizing` 进入 `ready`，向APP返回
`success=true`；在 `localization.relocalization_timeout_sec` 时间内未成功则停止定位Worker并返回 `success=false`。算法在尝试过程中发布的 `false` 仅表示尚未成功，不会提前结束等待：

```bash
ros2 service call /robot/start_mapping robot_interfaces/srv/StartMapping \
  "{map_id: 'map_1'}"
ros2 service call /robot/stop_mapping std_srvs/srv/Trigger '{}'
ros2 service call /robot/load_map robot_interfaces/srv/LoadMap \
  "{map_id: 'map_1'}"
ros2 service call /robot/delete_map robot_interfaces/srv/DeleteMap \
  "{map_id: 'map_1'}"
ros2 service call /robot/start_localization std_srvs/srv/Trigger '{}'
ros2 service call /robot/start_navigation std_srvs/srv/Trigger '{}'
ros2 service call /robot/pause_navigation std_srvs/srv/Trigger '{}'
ros2 service call /robot/resume_navigation std_srvs/srv/Trigger '{}'
ros2 service call /robot/stop_navigation std_srvs/srv/Trigger '{}'
ros2 service call /robot/stop_localization std_srvs/srv/Trigger '{}'
```

`/robot/stop_localization` 可以在 `/robot/start_localization` 等待重定位结果时并发调用。它会先取消并唤醒开始定位请求，再退出算法Executor、回收重定位线程并析构 `FastlioLocalizationNode`，完成后才向APP返回 `success=true`。等待超过 `localization.stop_timeout_sec` 则返回失败并进入 `error`。

## 多地图管理

APP 只传业务地图 ID，不传机器人绝对路径。地图 ID 长度为 1～64，只允许字母、数字、下划线和横线，并且必须以字母或数字开头。地图根目录通过 `map_storage.root_directory` 配置，`map_storage.max_maps` 默认限制最多保存 5 张完整地图。默认目录结构为：

```text
/robotfs/data/slam_data/map/
├── map_1/
│   ├── manifest.yaml
│   ├── globalmap.pcd
│   ├── result_free_space.pcd
│   ├── result_bag/
│   └── sequence/
└── .staging/
```

开始建图时如果存在同名 `map_id`，RobotRuntime 会复用删除 Service 的同一套目录删除函数，先递归删除旧地图整个文件夹，再创建本次临时建图目录。调用 `/robot/stop_mapping` 后，Service 会等待完整流程：先停止 FastLIO，再由 PGO 保存地图；确认 `globalmap.pcd` 存在后，临时目录重命名为正式地图目录，最后才向 APP 返回 `success=true`。等待时间由 `map_storage.finish_mapping_timeout_sec` 配置，默认 300 秒；超时或任一步失败返回 `success=false` 和具体原因。

地图数量只统计包含 `globalmap.pcd` 的正式地图目录，不统计 `.staging`。达到 5 张后，创建新 `map_id` 会被拒绝；再次使用已有 `map_id` 会直接覆盖对应地图，不增加地图数量。

调用 `/robot/load_map` 时 RobotRuntime 检查 `<root>/<map_id>/globalmap.pcd`，读取 `map_display.pcd_filename` 指定的PCD，并通过 `/robot/map/point_cloud` 发布一次静态地图；PCD读取和发布完成后才记录当前地图并返回成功。Publisher使用 `reliable + transient_local`，因此APP晚于加载操作订阅也能收到最近一张地图。该操作只负责地图展示，不会启动定位算法。

随后调用 `/robot/start_localization`，RobotRuntime 才创建 FastLIO Localization，将选中目录传给 `map_path` 参数，算法读取完整地图并开始重定位。该Service等待真正的重定位结果：成功返回 `relocalization succeeded`；默认60秒未成功则停止定位并返回超时失败，清理成功后状态回到 `idle`，可再次发起定位。等待和停止超时分别由 `localization.relocalization_timeout_sec`、`localization.stop_timeout_sec` 配置。FastLIO Localization不再重复发布APP静态地图Topic，只保留原有内部 `/global_map` 调试Topic。选中的地图被删除或开始新建图时会自动清空，需要重新调用 `/robot/load_map`。原有test launch仍使用YAML中的固定路径，不受该OTA选择逻辑影响。

当定位算法使用 BBS3D 时，会优先加载地图目录中已有的层级体素缓存；新地图尚无缓存时，定位模块会从该目录的 `globalmap.pcd` 构建内存中的层级体素地图，因此 PGO 新保存的地图可以直接用于首次定位。

APP 可以在机器人处于 `idle` 时请求删除对应正式地图目录下的全部文件：

```bash
ros2 service call /robot/delete_map robot_interfaces/srv/DeleteMap \
  "{map_id: 'map_1'}"
```

RobotRuntime 会重新校验地图 ID，然后递归删除 `<map_storage.root_directory>/map_1`，删除完成后 `map_1` 文件夹也不再存在。Service通过 `success` 和 `message` 返回删除结果；建图、定位或导航过程中会拒绝删除。删除操作没有备份，无法恢复。

## 导航 Worker

导航启动顺序：

1. `dual_radar_worker`
2. `costmap_worker`
3. `global_path_planner_worker`
4. `path_smoother_worker`
5. `target_points_visualizer_worker`
6. `navigation_task_worker`
7. `pure_pursuit_worker`

纯跟踪最后启动，避免其在地图、路径与任务模块未就绪时输出控制指令。任一模块启动失败，会按逆序停止已经启动的模块。

导航停止时首先停止 `pure_pursuit_worker`。其停止钩子会取消控制定时器并发布零速度，然后再销毁算法对象。其余模块随后按依赖顺序停止。

导航运行中调用 `/robot/pause_navigation` 后，运行状态从 `navigating`
切换为 `navigation_paused`。该操作不会暂停任务状态机，也不会析构导航
Worker；Pure Pursuit 保持运行，并在每个控制周期持续发布零速度。调用
`/robot/resume_navigation` 后恢复为 `navigating`，控制器从零速度重新计算
当前控制指令，不会重放暂停前的速度。暂停状态下仍可直接调用
`/robot/stop_navigation` 完整停止导航。内部速度控制服务为
`/pure_pursuit/set_paused`，仅供 RobotRuntime 使用，APP 不直接依赖它。

对应服务为：

```text
/dual_radar_worker/set_state
/costmap_worker/set_state
/global_path_planner_worker/set_state
/path_smoother_worker/set_state
/target_points_visualizer_worker/set_state
/navigation_task_worker/set_state
/pure_pursuit_worker/set_state
```

## 算法实例模型

`ManagedWorker::AlgorithmInstance` 同时保存：

```cpp
std::shared_ptr<rclcpp::Node> node;
std::shared_ptr<void> owner;
std::function<void()> before_stop;
```

继承 `rclcpp::Node` 的算法只需设置 `node`。普通算法类由 `owner` 保持生命周期，不必为了接入运行时控制而改成继承 `rclcpp::Node`。运动控制类可通过 `before_stop` 执行安全停车。

## 部署关系

platform 模式下开机自启动的是 Worker 二进制和 `robot_runtime_node`。雷达驱动等常驻基础模块可以一直运行；建图和导航算法只有收到相应上层命令后才创建。本地 ROS 2 APP 可以直接调用管理服务并订阅 `/robot/runtime_state`。算法继续使用原有内部Topic，不再额外发布APP标准实时位姿和点云；需要对外数据接口时再单独设计数据服务或协议适配器。

## APP 标准数据接口

`robot_runtime_node` 不订阅、转发算法实时大数据话题。它只在APP调用 `/robot/load_map` 时从本地读取一次静态PCD并发布；算法原有Topic保持不变，现有launch、RViz和算法间连接不受影响。稳定接口如下：

| 数据 | 数据来源 | APP Topic | 消息类型 | 发布方式 |
| --- | --- | --- | --- | --- |
| 运行状态 | RobotRuntime | `/robot/runtime_state` | `std_msgs/msg/String` | 状态变化时发布 |
| 加载的静态地图 | RobotRuntime 本地PCD | `/robot/map/point_cloud` | `sensor_msgs/msg/PointCloud2` | 每次加载发布一次 |

静态地图接口由 `robot_runtime.yaml` 中的 `map_display.*` 配置，使用 `reliable + transient_local` QoS。FastLIO、PGO和FastLIO Localization中的 `api.*` 参数及 `/robot/pose`、`/robot/cloud/registered` 实时Publisher已经移除；算法原有内部Topic保持不变。
