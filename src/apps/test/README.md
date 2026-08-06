# robot_runtime_test

该目录统一保存两套内部人工测试GUI：

- `navigation_workflow_gui.py`：通过`run_slam.sh`、`run_nav.sh`和
  `/nav_task_signal`测试独立算法流程，Worker使用`test`模式。
- `robot_runtime_test_gui.py`：调用正式的`/robot/*` Service、Action和
  Topic测试RobotRuntime，Worker使用`platform`模式。

两套流程会管理相同的算法节点，不能同时运行。独立流程启动前会清理旧节点。

## 构建与启动

```bash
colcon build --packages-select robot_runtime_test
source install/setup.bash
ros2 launch robot_runtime_test robot_runtime_platform_test.launch.py
```

直接启动需要的测试GUI：

```bash
source install/setup.bash

# 独立算法流程
ros2 run robot_runtime_test navigation_workflow_gui.py

# RobotRuntime接口流程
ros2 run robot_runtime_test robot_runtime_test_gui.py
```

`robot_runtime_platform_test.launch.py`只服务于RobotRuntime接口测试。
独立算法流程由自身GUI按步骤调用工作空间中的脚本。

RobotRuntime测试launch会启动与 `run_slam.sh` 一致的 Livox MID360s 驱动、
`robot_runtime` 以及 PGO、FastLIO、定位和导航的所有 Worker 外壳，
全部使用 `runtime_profile=platform`。Worker 启动后算法对象仍为 STOPPED；
点击 GUI 中的功能按钮后才由 Runtime 创建算法对象。

## 分功能检查

1. **连接检查**：确认 Runtime Service/Action 和八个 Worker 状态全部可见。
2. **建图**：开始建图、观察位姿/关键帧/点云计数、完成建图并检查返回值。
3. **地图**：加载或删除指定字符串 `map_id`（例如 `map_0`）。
4. **定位**：加载地图后使用可选初始位姿启动重定位。
5. **导航**：测试单点导航、巡检导航、暂停、恢复和取消。

建图、重定位和导航的成功路径依赖真实传感器或对应 rosbag。没有数据时，GUI
仍可验证服务发现、状态机拒绝、Worker 启停顺序、超时和错误返回是否符合 APP
协议。
