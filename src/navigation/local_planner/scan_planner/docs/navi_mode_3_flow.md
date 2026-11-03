# navi_mode=3：原版算法与实机适配

当前版本以 `src/SCAN-Planner-Ros2` 为算法基线。
包名保持 pure_pursuit，唯一可执行程序为 pure_pursuit_node。
ManagedWorker 管理节点 pure_pursuit_worker 与算法节点 pure_pursuit_node 同处一个进程。
算法回调使用一个默认互斥组和一个执行线程，保持原版顺序访问地图，避免可变地图并发读写。
不使用此前的地图快照、多组调度、额外绕障引导、规划预算或 BLOCKED_WAIT 状态。

## 实机输入

- `/pose_stamped_localization`：geometry_msgs/PoseStamped，map -> base_link。
- `/cloud_registered`：sensor_msgs/PointCloud2，位于 map。
- `/smoothed_global_path`：nav_msgs/Path，位于 map。仍为普通 Path，不消费逐边策略。
- `/navigation/task_target`：slam_manager_interfaces/TaskTarget，指定真实终点及最终朝向。
- `/navigation/global_replan_request`：std_msgs/Header，停止旧路径并等待相应的新路径。
- `/pure_pursuit_worker/set_state`：平台 START/STOP。
- `/pure_pursuit/set_paused`：暂停与恢复。

topics.* 参数直接控制输入输出，无需 launch 重映射。
PoseStamped 在入口适配成原版内部 Odometry；规划速度由连续定位差分、低通和限幅得到。
frame_id、坐标有效性和四元数在入口校验，不重复应用原版硬编码雷达外参。
当前点云已经位于世界坐标系，cloud_is_world=true、need_extrinsic=false，不重复转换。
算法本身不做地面分割；地面和楼梯是否与轨迹碰撞，取决于高度参考和膨胀范围。

UWB 对 SCAN 的适配已撤销，当前暂不支持 UWB 跟随。
普通导航不再需要 costmap_2d/terrain_filter，停障仍使用原始雷达及自身高度过滤。

## 从点云到速度

1. 原版 GridMap 接收位姿和点云，原版滑动窗口随位姿移动。
2. 原版 50 ms 占据更新定时器执行 raycastProcess、命中/未命中计数、概率融合和膨胀更新。
3. 收到全局 Path 后，原版 prepareReferenceWaypoints 去重、按间距采样并保留终点。
4. 原版全局多项式生成、局部目标选择和重规划 FSM 决定何时调用 reboundReplan。
5. 原版 A*、rebound/L-BFGS、动态可行性及时间重分配生成 B 样条。
6. 原版 checkCollisionCallback 以 50 ms 定时器检查轨迹，执行原版重规划与紧急停止行为。
7. 轨迹内部直接交给同节点控制器，同时在 planning/bspline 发布 navigation_interfaces/Bspline 供调试。
8. 控制器每 10 ms 使用原版前视期望朝向，角度误差过大时原地转向并冻结轨迹时间。
9. 对齐后使用原版“轨迹速度前馈 + 位置比例反馈”，转换为车体系速度并限幅。
10. closed_loop.max_vy=0.0，使所有输出 Twist.linear.y 为 0；输出到 /cmd_vel_planned，继续交给现有 obstacle_stop。

冻结期间 FSM 使用控制器执行时间同步原版 local_data.start_time，避免暂停/调头时两个时钟分离。
暂停、定位接收/源时间超时和 STOP 属于实机接口门控；不引入地图年龄停车阈值。
原版地图、规划和控制仍串行；计算耗时可能推迟控制回调，本版本不宣称解决这类性能问题。

## 到点反馈适配

原版 FSM 的“轨迹时间结束”不能直接作为平台到点成功。
模式 3 的轨迹用完会继续请求规划，不会仅凭时间上报成功。

接近真实路径终点且剩余路径足够短时，FSM 检查终点走廊和当前/下一小段转向足迹后，将终点阶段交给控制器。
控制器先做低速位置修正；真实 XY 距离 <= closed_loop.finish_dist 后锁存位置到达，禁止平移。
末端未到达时使用 terminal_min_vx（默认 0.12 m/s）作为最低线速度幅值，仍受最大速度和朝向对齐限制。
XY 达标的控制周期先发布零速度，下一周期才开始最终调角。
终点接管的投影和剩余路径长度只计算 XY，位置捕获后不因距离波动回切位置修正。
之后只原地旋转，使任务朝向误差 <= closed_loop.yaw_tolerance，再锁存成功并持续发布 /nav_state=true。
真正的新任务清除完成状态；同一未完成目标的重复任务消息不重置进度。
XY 已捕获后，同目标的新路径只更新时间戳并确认接收，保留调角或完成状态。
重规划请求先停车等待匹配路径，保留已捕获的 XY；暂停、定位超时和碰撞停车也不清除该进度。
不同目标的路径先缓存并停车，等待匹配任务后再执行；真正的 STOP 销毁本次算法实例。
任务和路径跨话题乱序时，只有端点匹配当前任务的路径/轨迹才能产生运动。
路径接收确认发布 /navigation/global_path_accepted，并过滤重复、旧路径及迟到重规划请求。

TaskTarget 没有独立任务序号。同一未完成任务按导航类型、拓扑目标 ID（适用时）、XYZ 和归一化最终朝向识别重复消息。
任务完成后再次收到相同任务仍按新任务处理，避免循环任务无法重新启动；因此完成后的迟到重复任务仍需上游避免。

当前 YAML 位置容差 0.1 m、角度容差 0.26 rad，终点接管距离 0.5 m。
这部分是为平台保留的终点适配，普通行驶保留原版闭环跟踪和朝向冻结。
当横向速度上限为 0、实际输出前进速度与轨迹 XY 速度均大于 0.05 m/s，且轨迹尚未结束时，
将位置误差投影到轨迹速度的左法向，附加 `closed_loop.cross_track_gain * 横向误差` 的转向补偿。
增益默认 0.6 (rad/s)/m，补偿限幅 `closed_loop.cross_track_max_wz` 默认 0.15 rad/s，
最终角速度仍受原最大角速度限制。任一参数设为 0 可关闭补偿；不改变横向速度输出。
暂停、规划 hold、定位失效、原地对齐、倒退、过期轨迹、终点 XY/最终调角均不应用这项补偿。
该补偿并非楼梯识别功能，对普通前进轨迹同样生效；需实机低速验证，不能替代底盘爬楼稳定性或碰撞保护。

上一目标完成后切换新任务，首条轨迹生成前检查定位差分速度：仅当三维速度不超过 0.1 m/s，
且在新路线首个有效 XY 方向上的投影小于 -0.001 m/s 时，起始速度归零，并清除速度低通中的旋转残余。
定位时间戳保留，继续拒绝乱序消息。首条轨迹成功交给控制器后结束该保护；规划失败重试期间继续检查。
前向速度、较大的实际速度及正常行驶中的重规划仍保留原有处理，原版优化器和跟踪朝向公式不变。

## 与原版的差异边界

原样恢复：地图缓冲和融合、滑动窗口、膨胀内核、射线遍历、A*、优化器、原版规划管理器和基本 FSM。
保留适配：扁平包构建、OTA 名称、Worker 启停、实机位姿/点云/路径话题、消息命名空间、暂停、定位超时与任务到点反馈。
显示定时器仍保留 grid_map.visualization_interval，默认 1 秒；内部建图仍是原版 50 ms 定时器。
可视化继续采用 SensorDataQoS（Best Effort），这是显示接口，不参与控制计算。
机器人碰撞半径、偏移、地图尺寸和速度参数属于实机配置，未强制替换为原版示例机器人尺寸。

原版失败累计达到 fsm.max_replan_fail_count（恢复默认 1000）时会进入原版紧急停止及等待新目标流程。
原版动态障碍清除、首次带障碍规划和运行性能需重新实测；当前不保留此前对此的额外修复。
本版不自动读取配置，OTA 必须显式 --params-file 加载 config/pure_pursuit_params.yaml。

## 本次验证发现的原版限制

Release 编译通过。隔离输入验证了原版跟踪、零横向速度、暂停/恢复、定位超时、重规划请求顺序和真实位置到点后转角。
连续直线目标可以完成。额外短距离任务（约 0.6–0.8 m，包括斜向任务）出现原版动态可行性检查拒绝，
日志为 acceleration > 1.500，连续失败后进入原版等待新任务流程。
直接调用原版规划管理器、起终点速度/加速度均设为零也能复现，不依赖平台 FSM 或控制器。
本次按保留原版的要求保留该行为，未放宽阈值或修改优化算法，不能据此宣称所有任务均通过。
