# obstacle_stop

原始点云停障与最终速度保护节点。点云按机型外参转换到`base_link`，构建每帧2.5D高度栅格，
分别检查前、后、左、右方向的停止区和减速区。输入规划速度`/cmd_vel_planned`，输出最终速度`/cmd_vel`。

## 机型与启动

| 配置 | 点云输入 | 默认启用方向 |
|---|---|---|
| `obstacle_stop_p2.5_mid360s.yaml` | `/livox/lidar`，Livox CustomMsg | 前、左、右；后方关闭 |
| `obstacle_stop_p3_airy96.yaml` | `/rslidar_front/points`、`/rslidar_rear/points`，PointCloud2 | 前、后、左、右 |

机器人外形尺寸、外参和`lidars`输入配置集中在`src/config/p2.5_mid360s.yaml`、
`src/config/p3_airy96.yaml`；stop专用YAML保留过滤、栅格和检测区域等算法参数。

```bash
source install/setup.bash
ros2 launch obstacle_stop obstacle_stop.launch.py robot_model:=p2.5_mid360s
# 或：
ros2 launch obstacle_stop obstacle_stop.launch.py robot_model:=p3_airy96
```

launch默认打开RViz2并加载`config/obstacle_stop.rviz`；不需要界面时添加`use_rviz:=false`。
默认`runtime_profile:=test`自动启动算法；`runtime_profile:=platform`等待slam_manager管理启动。

## 雷达输入配置

雷达输入在`src/config`对应机型文件的`lidars.front/rear`中定义，
stop launch同时加载算法配置和公共机型配置，zones通过`sources`引用已启用的输入。
例如P3：

```yaml
lidars:
  front:
    enabled: true
    topic: "/rslidar_front/points"
    frame: "rslidar_front"
  rear:
    enabled: true
    topic: "/rslidar_rear/points"
    frame: "rslidar_rear"
```

P2.5的front使用`/livox/lidar`和`livox_frame`，rear只需`enabled: false`。
`lidar_type`保留：1表示前输入为Livox CustomMsg，其它值表示PointCloud2；后输入为PointCloud2。
雷达外参继续使用公共机型文件的`lidar_to_base_T/R`、`rear_lidar_to_base_T/R`，与其它模块共享。
每个启用输入只订阅一次。关闭输入后，对应启用区域不能再引用它，否则初始化失败。
不写`sources`时，前区默认front、后区默认rear，左右区默认使用当前所有启用输入。
区域开关省略时，前/后区跟随对应雷达开关，左右区在至少一个雷达开启时默认开启。
旧的顶层`lidar_topic/lidar_frame/rear_lidar_enabled/rear_lidar_topic/rear_lidar_frame`
以及`zones.*.origin`已移除；旧配置会给出迁移错误，防止误用默认输入。

## 四方向检测范围

stop内部车体坐标系固定为`base_link`，不提供`base_frame`参数。
所有区域沿`base_link`轴摆放，不跟随雷达倾角旋转：

| 方向 | 基准位置 | 外推方向 | 覆盖宽度 |
|---|---|---|---|
| front | 前雷达外参原点X/Y | +X | robot_width |
| rear | 后雷达外参原点X/Y | -X | robot_width |
| left | 车体左侧Y=robot_width/2 | +Y | 自动补齐前后停止区外沿，至少覆盖车长 |
| right | 车体右侧Y=-robot_width/2 | -Y | 自动补齐前后停止区外沿，至少覆盖车长 |

每个方向的检测起点在代码中固定为0，`stop_distance`为从基准向外延伸的停止区距离，
`slowdown_distance`为接在停止区外侧的减速区厚度，单位m。
前后区宽度直接使用实际车宽；左右区厚度分别由对应的`stop_distance`配置，
左右检测起点从实际车体边缘计算。左右区前端取车头边界与已启用前停止区外沿
`front_lidar_x + zones.front.stop_distance`的较大值，后端取车尾边界与已启用后停止区外沿
`rear_lidar_x - zones.rear.stop_distance`的较小值，补齐四角。未启用的前/后区不参与
该端外沿计算，该端保留车身边界。减速区不参与长度计算。区域配置切换时自动重算，
检测与可视化使用相同边界。例如P3默认左右区X范围为[-0.77, 0.77]m，长度1.54m。
默认前后停止区从对应雷达原点延伸0~30cm，减速区接在外侧30~50cm；左右停止区距车侧0~10cm，在功能上仅有停止区，不支持减速区及其配置参数。
高度按`base_link`中的`min_height`到`max_height`判断，不叠加雷达安装高度。
栅格按格心归属区域，边界精度受栅格分辨率影响；停止/减速公共边界只归减速区。
不同方向的检测体积可在角落重叠，同一格允许分别出现在两个方向的统计中，整体占用显示只保留一份。

机型YAML中的示例：

```yaml
zones:
  left:
    enabled: true
    sources: [front, rear]  # P2.5使用[front]，P3侧面可合并前后雷达
    stop_distance: 0.10
    obstacle_cell_threshold: 1
```

四个方向分别使用`zones.front/rear/left/right`，各自可开关、设置来源、距离和占用格阈值。只有front/rear支持`slowdown_distance`；
left/right不声明该参数、不建立减速区，也不执行减速判断，启动配置或地图区域YAML中
若给侧面配置`slowdown_distance`（包括0）会明确报错。
区域基准由方向自动确定，不再配置`origin`；`start_distance`及旧的前后起始距离参数均已移除，
不允许通过启动参数或地图区域覆盖改变0起点。
启用方向引用不存在的后雷达、未知来源、非法距离或阈值时初始化失败，不会静默忽略配置。
关闭的方向不发布区域框，不参与速度判断或点云显示裁剪；没有被启用方向引用的雷达不参与数据时效门控。

## 过滤与2.5D占用

1. 原始点通过对应雷达外参变换到`base_link`。
2. 检查有限坐标、Z高度、XY水平距离`hypot(x,y)`、水平角度`atan2(y,x)`和栅格范围。
3. 每个默认5cm × 5cm的XY格记录当前帧有效点数、最低高度、最高高度。
4. 每路雷达独立达到`min_points_per_cell`才算占用，默认1，不要求最小高度跨度。
5. 按方向配置合并来源：占用取并集，高度取有效占用来源的包络，点数取来源最大值，不累加两路点数。
6. 各方向的停止/减速区分别统计占用格数，达到该方向的`obstacle_cell_threshold`即触发。

两路各有1点但每格要求2点时，不会通过相加产生占用；两路都占用同一个XY格时，该区域仍只计1格。
点数不跨帧累加，最新帧替换旧帧。无效点云、超时来源和地图区域切换不会保留旧高度柱。
提高每格点数要求可能漏掉稀疏障碍或细杆，应通过实测点密度调节。

Livox CustomMsg和PointCloud2均始终执行`base_link`高度过滤，UWB跟随模式也使用
相同规则，只有`min_height <= Z <= max_height`的点参与占用检测。
避免将以base_link为圆心的`range_min`调大而裁掉车侧区域；
修改`range_min/max`、角度范围或栅格尺寸时，应核对它们是否完整覆盖所有检测框。
`range_min`和`range_max`必须为有限值，并且严格满足`range_min < range_max`。
`linear_velocity_factor`和`angular_velocity_factor`必须为`[0, 1]`内的有限值（包含端点）；不符合要求时拒绝初始化。

## 速度策略与状态

第一版机型配置均关闭方向脱困：

- 任一启用方向停止区触发：所有最终速度分量归零，`/obstacle_stop/stopped=true`。
- 无停止区触发、任一减速区触发：linear.x/y乘`linear_velocity_factor`，angular.z乘`angular_velocity_factor`。
- 停止区清空后等待`hard_stop_clear_delay`再解除停止；只有前后方向保留减速及`slowdown_clear_delay`清障过渡，左右停止解除后直接恢复正常速度。
- 任一被引用的雷达尚未到达、超时、无效或原始空帧，以及规划速度超时：最终速度归零。
- 外部急停或姿态封锁：最终速度归零；姿态封锁收到true时立即补发零速度。
- `/obstacle_stop/enabled=false`透传规划速度，仍保留急停、姿态及指令超时保护。

`/obstacle_stop/active`表示当前是否处于保护状态；`/obstacle_stop/stopped`仅在障碍停止状态为true，
不把减速、传感器故障或姿态封锁当作障碍停止。
保留`directional_release_enabled`用于旧的前后区左/中/右脱困策略；开启后前后区按子区阈值判断，
左右方向仍按完整区域保守停车，不支持侧向区域放行。默认关闭。
终端`[OBSTACLE_ZONES]`日志每秒最多一次打印前后停止/减速占用格数以及左右停止占用格数。

## ROS接口

| 话题 | 类型 | 说明 |
|---|---|---|
| `/livox/lidar` 或 `/rslidar_front/points` | CustomMsg / PointCloud2 | 前输入，可配置 |
| `/rslidar_rear/points` | PointCloud2 | 后输入，可配置 |
| `/cmd_vel_planned` | Twist | 规划速度，可配置 |
| `/emergency_stop` | Bool | 急停 |
| `/navigation/posture_stop` | Bool | 姿态封锁 |
| `/obstacle_stop/enabled` | Bool | 停障开关 |
| `/cmd_vel` | Twist | 最终速度，可配置 |
| `/obstacle_stop/active`、`/obstacle_stop/stopped` | Bool | 保护状态、障碍停止状态 |
| `/obstacle_stop/detection_marker` | MarkerArray | 各启用方向的完整3D区域框、状态和占用格计数 |
| `/obstacle_stop/occupied_columns` | MarkerArray | 去重后的2.5D高度柱 |
| `/obstacle_stop/occupied_cells` | PointCloud2 | 占用格中心与统计字段 |
| `/obstacle_stop/front_cloud_base`、`/obstacle_stop/rear_cloud_base` | PointCloud2 | 前后裁剪彩色点云，base_link坐标；仅启用后雷达时发布后话题 |
| `/obstacle_stop/front_cloud_base_unfiltered`、`/obstacle_stop/rear_cloud_base_unfiltered` | PointCloud2 | 原始输入的有限XYZ点经外参转换到base_link，保留输入时间戳，不经过stop高度、距离、角度或区域裁剪；对应雷达启用且有订阅者时发布 |

## RViz显示

```bash
rviz2 -d src/navigation/obstacle_stop/config/obstacle_stop.rviz
```

Fixed Frame为`base_link`，默认斜视角。
前后两路点云分别按其参与的启用区域进行裁剪：各方向整体包围盒绕中心长宽高放大1.5倍，取并集。
因此单MID360的前输出也包含左右区域的点，不再只保留雷达前方。
显示点云始终按1.5倍包围盒的高度范围裁剪；检测仍严格使用配置的原始高度范围。
亮紫色RGB(255,80,255)表示通过检测过滤且位于对应检测栅格内的点，蓝色表示裁剪范围内的周边点。
颜色不以每格点数阈值为条件，实际占用由黄色半透明高度柱表示。
原输入时间戳保留，输出XYZ/RGB字段，不做降采样，仅有订阅者时构建与发布。
点云输出SensorDataQoS，RViz使用Best Effort、Volatile和RGB8；若出现RELIABILITY_QOS_POLICY告警，
检查手动添加的重复PointCloud2订阅者是否使用Reliable。

区域停止框深绿、减速框浅绿；有占用但未达到阈值为黄，触发停止为红、触发减速为橙。
标签显示方向、Stop/Slow、占用格数/阈值。各方向使用`obstacle_stop_front/rear/left/right`及对应`_counts`命名空间。
高度柱命名空间为`height_cells`，高度为实际z_min到z_max，单点/薄障碍显示至少2cm厚；
最小厚度只影响显示，柱体表示观测高度包络，不意味着上下点之间的空间实际全部被填满。
`occupied_cells`的Z为高度中点，附带`z_min`、`z_max`和`point_count`；多来源时point_count是来源最大值。
RViz默认关闭此点云，避免与高度柱重复显示。
两个MarkerArray话题使用Reliable、Transient Local和完整快照，先清旧Marker再发布当前内容。

## 地图区域覆盖与兼容

平台Worker传入当前`topology_map.json`。节点订阅`/pose_stamped_localization`，
根据`regions[].polygon`选择拓扑同级`region_obstacle_stop/<region_id>.yaml`。
缺少区域YAML时使用机型默认参数；禁用地图区域跳过加载；YAML非法则初始化失败。
离开区域后恢复机型默认值，切换时清空旧点云统计并等待新帧。

区域YAML可用以下格式覆盖四方向距离和阈值；源、开关、坐标基准保持机型配置：

```yaml
zones:
  left:
    stop_distance: 0.20
    obstacle_cell_threshold: 2
linear_velocity_factor: 0.40
angular_velocity_factor: 1.0
```

距离和占用格阈值统一在`zones.front/rear/left/right`下配置；启动参数及区域YAML均不再支持旧的顶层距离、阈值字段。

栅格统计仅在收到新点云、检测区域切换或雷达有效状态变化时更新；
单雷达区域直接读取源栅格，需要前后融合的区域共享一次融合结果。
控制定时器按`control_frequency`检查超时、解除延迟和速度放行；
区域框、占用柱和占用点由独立回调组中的10 Hz定时器根据显示快照发布，消息生成与发布不持有控制锁。
前后裁剪点云仍按对应输入帧发布。
状态变化时立即打印原因；仅当任一停止区存在有效占用格时，每秒打印一条`[STOP_STATUS]`摘要：当前状态、停障开关、指令及雷达状态、各方向占用格数、输出速度。没有停止区障碍（包括仅减速区有障碍、仅解除延迟仍在保持）时不打印周期摘要。
摘要中的`front(stop/slow)`和`rear(stop/slow)`分别表示停止区/减速区的占用格数，不是阈值比值。

RViz默认开启完整base_link点云（前黄色、后青色），裁剪彩色点云可按需勾选。完整点云沿用驱动已执行的过滤，仅输出XYZ字段。

## 源码职责

实现沿用同一个 `ObstacleStop` 类，按职责分为三个主要文件：

| 文件 | 职责 |
| --- | --- |
| `src/obstacle_stop.cpp` | 节点初始化、参数加载、地图区域切换、停障状态和速度控制 |
| `src/obstacle_stop_detection.cpp` | 雷达回调、点云变换与过滤、栅格构建、检测区占用统计 |
| `src/obstacle_stop_visualization.cpp` | 显示资源初始化、调试点云、检测框与占用栅格发布 |

`src/region_profile_loader.cpp` 专门解析地图区域的 YAML 配置覆盖；拓扑多边形加载、定位选区和配置切换保留在主文件。
检测统计与控制决策共用原有状态和锁；可视化定时器在配置锁内复制快照，再在锁外构造和发布消息。调试点云仍由雷达回调在配置锁内发布。

## IMU倾斜期间屏蔽减速

stop直接订阅`woodox_imu_bridge`发布的`/imu/data`，进程内调用`src/depd/imu_stair_detector`公共库，不启动额外检测节点，也不依赖`imu_tools`。bridge应使用其默认参数文件（`orientation_format: wxyz`），提供有效设备姿态；裸运行bridge时其代码默认禁用姿态。

两个机型配置均设置`imu_stairs.enabled: true`。使用有效车体pitch与固定平地基准的绝对偏差，默认持续超过/等于14°达0.15秒进入，持续低于/等于10°达0.40秒退出；滤波时间常数为0.15秒。默认阈值和时间需要按实际姿态噪声、上下楼动作验证。坡道也可能触发；该算法不能提前识别第一阶。

进入倾斜状态后，最终速度决策忽略减速区触发及已有`slowdown_clear_delay`保持；占用统计和可视化仍保留。停止区、急停、姿态封锁、雷达异常及指令超时继续按原逻辑生效。退出后正常减速判断立即恢复，区域覆盖不会重置IMU检测器。

IMU未到达、姿态不可用（`orientation_covariance[0] < 0`）、非法四元数、坐标系不符、消息过旧/超前、时间倒退或超过0.30秒未更新时，取消屏蔽并恢复正常减速。超时不会单独引起停车。`/obstacle_stop/stairs_active`（Bool，Transient Local）表示当前有效倾斜减速屏蔽状态，不表示当前一定在移动；进入、退出或失效解除屏蔽时打印日志。

参数集中在`imu_stairs`下：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `enabled` | true | 是否启用倾斜减速屏蔽 |
| `topic` / `frame` | `/imu/data` / `woodox_imu` | 输入及严格匹配的IMU坐标系 |
| `imu_to_base_rpy` | `[0, 0, 0]` | R_base_imu的roll/pitch/yaw，rad；将IMU轴旋转到车体轴 |
| `pitch_baseline_deg` | 0 | 固定平地车体pitch基准，度 |
| `enter_angle_deg` / `exit_angle_deg` | 14 / 10 | 相对基准的进入/退出角度，度 |
| `filter_time_constant_sec` | 0.15 | 低通时间常数，0为关闭滤波 |
| `enter_duration_sec` / `exit_duration_sec` | 0.15 / 0.40 | 持续确认时间 |
| `timeout_sec` | 0.30 | 消息新鲜度及连续采样超时 |

`imu_to_base_rpy`默认假定IMU轴与车体轴对齐，需按实际安装填写，不能拿雷达外参替代。设备四元数参考系应保持重力对齐；平地、抬头、低头时须核对转换后的pitch。角度不自动以启动姿态归零，避免在楼梯上启动时把倾斜当作平地。
