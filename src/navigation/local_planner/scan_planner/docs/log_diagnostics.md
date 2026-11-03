# 原版实机基线日志

本次恢复原版地图、规划及闭环跟踪后，之前 scan_map_kernel、scan_map_snapshot、scan_obstacle_guide、
scan_safety_gate 和 BLOCKED_WAIT 日志及对应额外行为已撤出。
旧测试日志不能作为本版行为证明。

启动时检查 `[scan_baseline] mapping=original planner=original tracking=original max_vy=0.000`。
`terminal=platform_adapter` 表示实际到点和最终朝向反馈仍为平台适配。

- `[scan_controller]`：横向速度上限与位置容差。
- `[scan_task]`：任务真实终点、最终 yaw、路径是否匹配。
- `[scan_terminal_reset]`：到点状态重置来源；新任务带内部 seq，新路径带 path_stamp。
- `[scan_terminal_preserved]`：重复任务、同目标路径或重规划请求保留终点进度，source 标识触发来源。
- `[scan_terminal_path]`：同目标路径已确认接收，未重新进入轨迹跟踪。
- `[scan_path_waiting_task]`：路径终点与当前任务不匹配，停车缓存并等待任务。
- `[scan_path_rejected]`：路径坐标系或时间戳无效，打印 minimum/accepted 供检查乱序。
- `[scan_control_gate]`：停车条件变化及恢复，reason 区分暂停、规划 hold、定位接收/源时间超时、未来时间戳和任务路径不匹配；同时打印定位年龄与终点状态。
- `[scan_trajectory_wait]`：任务和路径已匹配，但没有可执行轨迹，最多每 2 秒打印。
- `[scan_heading_align]`：普通轨迹跟踪中的朝向对齐，最多每秒打印，可与最终调角区分。
- `[scan_start_state]`：上一目标完成后的下一任务起点，打印定位差分原始速度、采用速度、路线首段方向和处理原因，最多每秒打印；action=zero_reverse_residual 表示清除了低速反向残余。
- `[scan_startup_trajectory]`：任务切换后的首条轨迹已交给控制器，后续途中重规划恢复原有速度处理。
- `[scan_fsm]`：原版 FSM 状态切换和触发来源。
- `[scan_control]`：原版跟踪的轨迹 ID、执行时间、vx/vy/wz。
- `[scan_lateral_correction]`：零横向速度跟踪时的横向误差（轨迹在左为正）、原始转向、限幅纠偏转向及最终转向；最多每秒打印一次。
- `[scan_terminal_enter]`：FSM 开始终点接管。
- `[scan_terminal]`：phase=xy/xy_reached/yaw，包含真实距离、捕获状态、当前/目标 yaw、角度误差及速度；常规状态最多每秒打印。
- `[scan_yaw_direction_change]`：同一调角阶段旋转指令变号，打印当时角度误差，用来区分状态重置与实际过冲/定位跳变。
- `[scan_goal_completed]`：完成瞬间的真实位置距离和角度误差。
- `[scan_cloud_rejected]`：点云 frame_id 与实机配置不一致。
- `[scan_callback] name=occupancy_update`：地图更新回调耗时与调度间隔（仅诊断，不改变算法）。

原版还会输出 A*、优化结果、重规划耗时和紧急停止信息。
显示周期由 grid_map.visualization_interval 决定，不代表内部建图频率。
Worker 会输出会话 START/STOP，区分同一进程中的不同任务。

排查“转过去又转回来”时：先对齐 seq 和 path_stamp，检查是否出现 terminal_reset；
若只有 terminal_preserved，再看 phase、target_yaw 和 yaw_direction_change，判断是否为航向误差本身变号。
seq 只在本次算法实例内递增，Worker STOP/START 后重新计数，不能作为跨会话任务 ID。
