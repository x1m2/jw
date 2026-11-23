# obstacle_stop

原始点云停障与最终速度保护节点。点云按机型外参转换到`base_link`，构建每帧2.5D高度栅格，
分别检查前、后、左、右方向的停止区和减速区。输入规划速度`/cmd_vel_planned`，输出最终速度`/cmd_vel`。

## 机型与启动

| 配置 | 点云输入 | 默认启用方向 |
|---|---|---|
| `obstacle_stop_p2.5_mid360s.yaml` | `/livox/lidar`，Livox CustomMsg | 前、左、右；后方关闭 |
| `obstacle_stop_p3_airy96.yaml` | `/rslidar_front/points`、`/rslidar_rear/points`，PointCloud2 | 前、后、左、右 |

机器人外形尺寸、外参和`lidars`输入配置集中在`src/config/P2.5_mid360s.yaml`、
`src/config/P3_airy96.yaml`；stop专用YAML保留过滤、栅格和检测区域等算法参数。

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
雷达外参使用公共机型文件的 `lidar_to_base`、`rear_lidar_to_base`，与建图和定位共用解析器。
`translation` 单位为米；`rotation.type: rpy_rad` 表示弧度制 RPY，`matrix` 表示按行排列的九个矩阵元素。
变换方向统一为 `p_base = R * p_lidar + t`。详见 [机体配置](../../config/README.md)。
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
机型配置须确保启用方向引用有效雷达来源，并正确设置距离和占用格阈值。
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

普通停障与减速规则如下；规划器发布横移/后退模式时按后文审核速度：

- 默认任一启用方向停止区触发：所有最终速度分量归零（规划器横移/后退模式见下文）。
- 无停止区触发、任一减速区触发：linear.x/y乘`linear_velocity_factor`，angular.z乘`angular_velocity_factor`。
- 停止区清空后等待`hard_stop_clear_delay`再解除停止；只有前后方向保留减速及`slowdown_clear_delay`清障过渡，左右停止解除后直接恢复正常速度。
- 任一被引用的雷达尚未到达、超时、无效或原始空帧，以及规划速度超时：最终速度归零。
- 雷达、规划速度和模式的接收心跳均使用本地单调时钟；即使ROS `/clock`暂停或跳变，接收超时仍生效。`lidar_timeout_sec`、`command_timeout_sec`和`planner_motion_mode_timeout_sec`的含义及默认值不变。模式消息`stamp`的源时间检查仍使用ROS时钟。
- 外部急停或姿态封锁：最终速度归零；姿态封锁收到true时立即补发零速度。
- `/obstacle_stop/enabled=false`透传规划速度，仍保留急停、姿态及指令超时保护。

`planner_velocity_guard_enabled`用于**规划器横移/后退模式的最终速度校验**。STOP不自主选择方向、等待或生成横移速度。SCAN planner负责动作距离、导航暂停、超时、重试和动作结束后的局部重规划。本次只接入STOP；旧规划器未发布模式时只能正常行驶/停障，不会自动横移。

参数统一使用规划器速度保护和净空检测命名，横移及后退共用预检和安全限速参数。部署配置需使用下表中的新参数名。STOP不保存动作开始时间、方向锁或中断锁止，不决定启动、结束、持续时间及重试；这些流程全部由SCAN负责。

### 状态及模式接口

- `/obstacle_stop/status`：`navigation_interfaces/ObstacleStopStatus`，包含`bool[4] stop_triggered`、`bool[4] move_allowed`和`float32[4] available_distance`三个固定数组，每控制周期发布，可靠、volatile、队列1。数组顺序前、后、左、右；SCAN按接收时间判断超时，超时必须停车。
- `stop_triggered`是各已启用停止区的原始阈值结果。`move_allowed`是当前方向的环境安全许可，不代表动作请求或命令确认。雷达失效、急停、姿态保护或STOP关闭时全部许可为false；停止工作前也发布全部false。前向许可沿用普通四区停车及清空等待，不做前向预检；后退、左右许可共用实际速度门的开关、区域启用、目标停止区与净空判断。许可与动作历史无关，不要求原侧障碍触发；模式/速度的新鲜度和纯运动分量仍需单独审核。可用距离通过状态数组反馈，数量、阈值和拒绝原因留在日志中。
- `available_distance`与其他数组保持前/后/左/右对齐，但前向索引0仅占位，始终为0，即使前向`move_allowed=true`也不表示提供前向距离。只有后、左、右三个方向提供距离，单位为米，从停止区外侧的预检内边界起算，最大为`clearance_check_distance`。这三个方向`move_allowed=false`时距离严格为0，为true时给出正距离。有效空预检返回完整预检距离，达到占用阈值时按最近格近边界保守计算。距离是当前观测下的上限，SCAN须持续读取更新，不能按单帧距离直接执行到底。
- `/scan_planner/motion_mode`：`navigation_interfaces/PlannerMotionMode`，可靠、volatile、队列1。消息仅含`builtin_interfaces/Time stamp`和`uint8 mode`；四种模式为`NORMAL=0`、`LATERAL_LEFT=1`、`LATERAL_RIGHT=2`、`BACKWARD=3`，方向固定相对`base_link`，不传`frame_id`。模式持续发布，每次更新`stamp`并与STOP使用同一ROS时钟；源时间与本地单调时钟接收心跳分别受`planner_motion_mode_timeout_sec`约束。
- `/cmd_vel_planned`仍为Twist，由规划器持续发布。模式改变后，STOP须再收到一条新的速度消息才允许输出，避免把切换前缓存的速度用于新模式。没有申请编号或确认握手。
- 首次尚未收到模式消息时，兼容旧规划器的普通速度门。收到过模式后，模式超时、非法值或无效时间戳均停车，**不能自动退回NORMAL**；重新发布新鲜NORMAL恢复普通判断。

### 速度门规则

- NORMAL或尚未接入模式：沿用普通四向停障、解除延迟、减速和开关逻辑。
- LATERAL_RIGHT：只允许`linear.y >= 0`的纯横移；LATERAL_LEFT只允许`linear.y <= 0`。本底盘指令+y向右、−y向左；点云几何仍为ROS base_link的+y左、−y右。`linear.x/z`、`angular.x/y/z`须为0，所有分量必须有限。
- 横移模式要求停障开关和`planner_velocity_guard_enabled`开启、左右区同时启用、雷达有效且速度与模式都新鲜。目标侧停止区触发、预检无效或净空耗尽时归零。不要求原侧障碍触发；前后停止区及其解除延时不阻止纯横移。
- BACKWARD：前后及左右区均启用、后区和两侧均未触发；只允许`linear.x <= 0`且其他分量为0。不要求前区触发。后方预检从后停止区外边缘与车尾中更靠后的边界继续向后扩展，宽度为完整车宽；后区阈值用于预检，净空不再扣除停止距离，与横移共用固定安全限速。侧面停障仍阻止后退。
- 只限幅规划器发来的速度，绝不自行生成非零速度：环境许可时使用固定上限`planner_velocity_limit`，不按剩余净空动态减速；目标停止区触发、预检无效或剩余净空不足时停车。正常减速由SCAN负责。规划器发0就输出0。
- 不再检查首次启动的最小距离，不计动作持续时间，也不锁定恢复方向或锁止中断后的动作。SCAN持续发送新鲜且匹配的模式与速度，STOP每周期只依据当前环境及保护状态决定放行；模式切换仍须收到新的速度消息。保护解除后无需先回NORMAL复位。
- 急停、姿态封锁、指令/模式超时、传感器异常始终输出零速。退出横移时规划器应先发零速并发布NORMAL，再从实际位姿重新规划；新轨迹成功前保持零速。STOP不决定何时重规划。

### 运动方向净空预检

四个普通停止区始终实时检测。雷达有效且`planner_velocity_guard_enabled`开启时，仅计算后、左、右三个方向的净空，不计算前向净空。前向状态沿用NORMAL停障规则，距离固定为0；三个预检方向的许可和距离每控制周期向SCAN反馈。净空结果在点云未更新时复用；新点云、雷达可用性变化或区域配置切换均使缓存失效。雷达失效时清空预检结果。模式、速度、急停和姿态保护仍每控制周期重新审核，不缓存放行结论。区域关闭、预检超出有效覆盖或净空不足时，对应许可为false。

- 从目标方向停止区外边界再向外预检`clearance_check_distance`（默认0.50 m），不含停止区本身。侧向10 cm停止区加外侧50 cm预检，总覆盖车身外60 cm；横移预检覆盖侧区全长（包括前后停止区旁的角部），不额外前后扩展。四向普通停止区仍独立检测内侧障碍。
- 使用该侧的雷达来源、高度/范围/角度过滤及`min_points_per_cell`。两路看到同一XY格只计1格，预检占用格数达到目标方向`zones.left/right/rear.obstacle_cell_threshold`才用最近占用格限制距离，不足阈值按完整预检宽度计算。
- 预检格心必须落在停止区外的预检框内；位于共享内边界的格子归普通停止区，不重复计入预检。最近距离按有效格子的近边界保守计算。
- 可用距离从预检内边界起算，等于min(预检距离, 最近达到阈值的占用格近边界距离)，负值取0。有效空预检按完整0.50 m计算，不再扣除停止距离；许可方向的距离通过`available_distance`反馈给SCAN，也用于STOP内部许可判断和日志，不用于动态限速。SCAN须自行配置动作距离和时间上限。
- 预检框超出内部栅格、范围/角度过滤时标记无效。有效空预检只表示过滤点云未发现障碍，不识别遮挡或认证自由空间。

| STOP参数 | 代码默认值 | 含义 |
|---|---:|---|
| `planner_velocity_guard_enabled` | false | 允许规划器横移/后退速度校验，机型YAML开启 |
| `planner_velocity_limit` | 0.05 m/s | 横移/后退速度上限，机型YAML为0.25 |
| `clearance_check_distance` | 0.50 m | 后/左/右停止区外额外预检距离，不含停止区 |
| `planner_motion_mode_timeout_sec` | 0.30 s | 模式源时间和接收心跳超时 |

STOP不提供动作计时、最小启动距离、等待或位姿进度参数，部署配置中应移除旧的动作流程参数。位姿订阅仅用于区域配置切换；动作距离、计时、方向选择、重试及重规划全部由SCAN负责。

- `[STOP_EVENT]`：状态、拒绝原因或四向触发结果变化时立即打印；包含模式、原因和四向停止区占用格数/阈值。相同状态下原因改变也会打印。
- `[STOP_STATUS]`：有占用格、保护状态或横移/后退指令时，每秒最多一次；打印前/后/左/右停止格数/阈值、前后减速格数/阈值、方向许可、前后雷达状态，以及规划速度和最终速度。雷达状态区分`unused/waiting/timeout/invalid/ok`。
- `[STOP_CLEARANCE]`：与周期摘要一起按后/左/右(B/L/R)打印，不打印前向预检。`valid`为预检有效性，`cells`为预检占用格数/阈值，`measured`为停止区外侧测得的净空（m，无效时为-1），`available`与状态消息距离数组一致，不允许方向统一为0。

日志中的`cells`均为去重XY占用格数，不是原始点数；每格至少达到`min_points_per_cell`才算占用。正常无障碍且无横移/后退指令时不持续打印周期摘要。周期日志使用单调时钟限频。

部署时先构建/发布`navigation_interfaces`的新消息，再构建STOP。SCAN planner本次未接入；下一步需订阅status、发布带时间戳的motion_mode，并在横移前暂停旧轨迹、结束后重规划。

## ROS接口

| 话题 | 类型 | 说明 |
|---|---|---|
| `/livox/lidar` 或 `/rslidar_front/points` | CustomMsg / PointCloud2 | 前输入，可配置 |
| `/rslidar_rear/points` | PointCloud2 | 后输入，可配置 |
| `/cmd_vel_planned` | Twist | 规划速度，可配置 |
| `/emergency_stop` | Bool | 急停 |
| `/navigation/posture_stop` | Bool | 姿态封锁 |
| `/obstacle_stop/enabled` | Bool | 停障开关 |
| `/obstacle_stop/status` | ObstacleStopStatus | 每周期四向停障、移动许可和可用距离（两个bool[4]及一个float32[4]数组） |
| `/scan_planner/motion_mode` | PlannerMotionMode | 输入：NORMAL/左横移/右横移/后退模式及时间戳 |
| `/cmd_vel` | Twist | 最终速度，可配置 |
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

stop直接订阅`woodox_imu_bridge`发布的`/imu/data`，进程内调用`src/third_party/imu_stair_detector`公共库，不启动额外检测节点，也不依赖`imu_tools`。bridge应使用其默认参数文件（`orientation_format: wxyz`），提供有效设备姿态；裸运行bridge时其代码默认禁用姿态。

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
