# slam_manager_test

该目录保存SlamManager内部集成测试工具：

- `slam_manager_test_gui.py`：调用正式的`/robot/*` Service、Action和
  Topic测试SlamManager，Worker使用`platform`模式；GUI也可以针对当前
  `map_id`启动RViz地图打点工具、启动键盘控制，并清理platform launch启动的
  全部项目ROS节点。日志统一带时间戳打印到启动终端并写入
  `/robotfs/data/slam_data/log/<map_id>/<时间>/gui.log`。
- `slam_manager_platform_test.launch.py`：启动SlamManager及全部platform
  Worker外壳。

## 构建与启动

```bash
colcon build --packages-select slam_manager_test
source install/setup.bash
ros2 launch slam_manager_test slam_manager_platform_test.launch.py
```

启动测试GUI：

```bash
source install/setup.bash

ros2 run slam_manager_test slam_manager_test_gui.py
```

`slam_manager_platform_test.launch.py`只服务于SlamManager接口测试。

SlamManager测试launch会启动与 `run_slam.sh` 一致的 Livox MID360s 驱动、
`slam_manager` 以及 PGO、FastLIO、定位和导航的所有 Worker 外壳，
全部使用 `runtime_profile=platform`。Worker 启动后算法对象仍为 STOPPED；
点击 GUI 中的功能按钮后才由 Runtime 创建算法对象。

## 分功能检查

1. **连接检查**：确认 Runtime Service/Action 和八个 Worker 状态全部可见。
2. **建图**：开始建图、观察位姿/关键帧/点云计数、完成建图并检查返回值。
3. **地图**：加载或删除指定字符串 `map_id`（例如 `map_0`）；“离线打点”使用
   RViz 2D Goal Pose，“在线打点”复用重定位后的持续纯定位位姿。
4. **定位**：加载地图后使用可选初始位姿启动重定位。
5. **导航**：测试单点导航、巡检导航、暂停、恢复和取消。

建图、重定位和导航的成功路径依赖真实传感器或对应 rosbag。没有数据时，GUI
仍可验证服务发现、状态机拒绝、Worker 启停顺序、超时和错误返回是否符合 APP
协议。

在线打点操作顺序：加载地图、启动重定位并等待成功、点击“在线打点”输入
`task_id`、启动现有键盘控制，然后按`K`。键盘节点只发布
`/topology_node_set`触发，编辑器抓取最新`/pose_stamped_localization`并立即写入
与离线打点相同的原始点、目标点、任务JSON和`test.txt`。


## 目录职责

- GUI 创建新地图目录：`<data_root>/<map_id>/map`、`<data_root>/<map_id>/task`。
- GUI 创建会话日志目录：`<data_root>/log/<map_id>/<时间>/`。
- `platform.log` 保存整个平台 launch 的 stdout/stderr。
- `ROS_LOG_DIR` 指向 `<data_root>/log/<map_id>/<时间>/ros/`。
- `slam_manager` 不创建 map/task/log 目录，只检查目录并把地图结果保存到已有的 `map/`。
