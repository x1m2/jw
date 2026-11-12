# Woodox IMU Bridge

标准 ROS 2 C++ 包，将机器狗发布的 Woodox 私有 IMU 消息转换为
`sensor_msgs/msg/Imu`。

## 输入和输出

- 输入：`/imu`，类型 `woodox_ros2_interface/msg/IMU`
- 输出：`/imu/data`，类型 `sensor_msgs/msg/Imu`
- 节点名：`woodox_imu_bridge`
- 输出坐标系：`woodox_imu`

当前默认配置保持原有行为：角速度单位为 rad/s，加速度单位为 m/s²并保留重力，
四元数从设备的 wxyz 顺序转换为 ROS 的 xyzw 顺序并归一化。输出消息使用节点收到
输入时的 ROS 时钟作为时间戳。

节点不会应用 IMU 标定偏置、LiDAR–IMU 外参、坐标轴旋转或时间补偿。三种协方差
默认未知（全零）；禁用姿态或四元数无效时，`orientation_covariance[0] = -1`。

## 编译

```bash
cd /home/jw-zkl/robotics_platform/robot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  woodox_ros2_interface woodox_imu_bridge
source install/setup.bash
```

## 启动

使用默认参数文件：

```bash
ros2 launch woodox_imu_bridge woodox_imu_bridge.launch.py
```

或者直接启动节点并指定参数文件：

```bash
ros2 run woodox_imu_bridge woodox_imu_bridge_node --ros-args \
  --params-file src/drivers/woodox_imu_bridge/config/woodox_imu_bridge.yaml
```

检查输出：

```bash
ros2 topic info /imu/data --verbose
ros2 topic hz /imu/data
ros2 topic echo /imu/data sensor_msgs/msg/Imu --once
```

## 参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `input_topic` | `/imu` | Woodox 私有 IMU 输入话题 |
| `output_topic` | `/imu/data` | 标准 IMU 输出话题 |
| `frame_id` | `woodox_imu` | 输出消息坐标系 |
| `gyro_unit` | `rad_s` | `rad_s` 或 `deg_s` |
| `acc_unit` | `m_s2` | `m_s2` 或 `g` |
| `orientation_format` | `wxyz` | `none`、`wxyz` 或 `xyzw` |
| `input_reliability` | `reliable` | `reliable` 或 `best_effort` |
| `input_timeout_s` | `5.0` | 输入超时告警周期 |

输出使用 Reliable、Volatile、Depth 100 QoS。输入的 Reliability 可配置，其他输入
QoS 为 Volatile、Depth 100。节点会拒绝同一主机、同一用户、同一 ROS domain 和同一
输出话题的重复实例。

本包不安装或管理 systemd 服务，也不包含开机自启动逻辑。
