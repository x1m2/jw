# user_interface

独立导航流程的测试状态机。源码放在 `src/test`，与 RobotRuntime 正式任务链路隔离；
ROS 包名、节点名和话题保持不变。

## 启动

```bash
ros2 launch user_interface navigation_task_state_machine.launch.py
```

可指定地图：

```bash
ros2 launch user_interface navigation_task_state_machine.launch.py map_id:=1
```

测试 GUI 使用统一脚本：

```bash
python3 src/apps/test/navigation_workflow_gui.py
```

## 接口

订阅：

- `/nav_state` (`std_msgs/Bool`)：导航是否到达目标点。
- `/nav_task_signal` (`std_msgs/String`)：任务名、`LOOP:<任务名>`、`STOP`、
  `CONTINUE` 或 `GO_HOME`。
- `/pose_stamped_localization` (`geometry_msgs/PoseStamped`)：当前定位位姿，
  话题可通过参数覆盖。

发布：

- `/target_points` (`geometry_msgs/Pose`)：当前导航目标点。
- `/task_action` (`std_msgs/Int32`)：到点后的任务动作。

任务文件默认从
`/robotfs/data/slam_data/map_<map_id>/task` 读取，目标点默认从
`/robotfs/data/slam_data/map_<map_id>/topology_map/generated_map_target_points.txt`
读取。launch 参数 `data_dir` 和 `map_id` 可覆盖这些路径。
