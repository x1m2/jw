# robot_runtime_test

该目录提供 APP 接入前的人工集成测试工具。所有控制都调用正式的
`/robot/*` Service、Action 和 Topic，不绕过 `robot_runtime`。

## 构建与启动

```bash
colcon build --packages-select robot_runtime_test
source install/setup.bash
ros2 launch robot_runtime_test robot_runtime_platform_test.launch.py
```

另开终端启动 GUI：

```bash
source install/setup.bash
ros2 run robot_runtime_test robot_runtime_test_gui.py
```

统一 launch 会启动与 `run_slam.sh` 一致的 Livox MID360s 驱动、
`robot_runtime` 以及 PGO、FastLIO、定位和导航的所有 Worker 外壳，
全部使用 `runtime_profile=platform`。Worker 启动后算法对象仍为 STOPPED；
点击 GUI 中的功能按钮后才由 Runtime 创建算法对象。

## 分功能检查

1. **连接检查**：确认 Runtime Service/Action 和七个 Worker 状态全部可见。
2. **建图**：开始建图、观察位姿/关键帧/点云计数、完成建图并检查返回值。
3. **地图**：加载或删除指定字符串 `map_id`（例如 `map_0`）。
4. **定位**：加载地图后使用可选初始位姿启动重定位。
5. **导航**：测试单点导航、巡检导航、暂停、恢复和取消。

建图、重定位和导航的成功路径依赖真实传感器或对应 rosbag。没有数据时，GUI
仍可验证服务发现、状态机拒绝、Worker 启停顺序、超时和错误返回是否符合 APP
协议。
