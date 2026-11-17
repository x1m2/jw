# imu_stair_detector

无ROS节点、消息或时钟依赖的C++17/Eigen公共库。根据设备提供的姿态四元数判断车体俯仰倾斜，供stop等模块复用，不依赖imu_tools。

`configure(Config)`设置安装旋转、平地俯仰基准、进入/退出角度、滤波时间常数、确认时间和超时；配置非法返回false。`update(q, timestamp)`输入IMU相对世界的姿态与秒时间戳，`status(now)`返回有效性、倾斜状态和滤波后的车体pitch。调用者负责线程同步，所有时间必须来自同一时钟。

安装旋转`imu_to_base`为R_base_imu，将IMU向量旋转到车体坐标系；姿态换算为R_world_base = R_world_imu × R_base_imu的转置。输入四元数自动归一化；设备应提供重力对齐、轴约定正确的姿态。安装补偿只使用旋转，不需要平移。

滤波使用随采样间隔变化的一阶低通系数。俯仰角偏差取绝对值，因此上下倾斜都可进入。进入角度必须高于退出角度，持续满足阈值才切换。数据超时、时间倒退/重复、非法四元数均清除历史，重新开始确认。新鲜度由调用者结合`status(now)`检查；额外的协方差、frame和消息有效性检查由ROS适配端负责。

这是倾斜状态判断，不识别台阶几何，坡道也可能触发；不能在车体开始倾斜前发现第一阶。默认角度与时间是初始配置，需要实机验证。

验证：

```bash
colcon build --packages-select imu_stair_detector
colcon test --packages-select imu_stair_detector
colcon test-result --test-result-base build/imu_stair_detector
```
