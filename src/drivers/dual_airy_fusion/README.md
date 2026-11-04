# dual_airy_fusion

ROS 2 Humble 双 Airy 点云融合包。将后雷达点云转换到前雷达坐标系，校正时间差后按前雷达扫描时间窗合并。只启动一个节点 `dual_airy_fusion_node`。

## 编译与启动

先启动前后雷达驱动，在工作空间根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select dual_airy_fusion
source install/setup.bash
ros2 launch dual_airy_fusion dual_airy_fusion.launch.py
```

启动终端的 `ROS_DOMAIN_ID` 应与驱动一致。launch 使用 `rmw_fastrtps_cpp`，设置 `ROS_LOCALHOST_ONLY=0`；DDS 配置沿用运行环境。启动前请退出旧融合节点，避免重复发布。

默认先接收两路 IMU 和点云检查时间差，最早约 12 秒通过，60 秒未通过则退出。通过后释放检查订阅，使用本次固定时差进行两路点云融合。

## 配置与参数

外参文件为 `config/dual_airy_fusion.yaml`，由节点直接读取，不是 ROS 参数文件。

- `T_front_rear`：后雷达到前雷达的 4×4 变换，`p_front = T_front_rear * p_rear`，平移单位米。
- `delta_s`：时间关系为 `t_front = t_rear + delta_s`；默认启动检查会替换历史时差。
- 默认矩阵来自 packet_225732 的 MLCC 标定，状态为 `EXPERIMENTAL_NOT_ACCEPTED`，尚未完成独立精度验收，仅适用于对应的刚性安装。

launch 支持以下参数：

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `extrinsics_file` | 包内 YAML | 外参文件路径 |
| `auto_time` | `true` | 启动时检查时间差 |
| `startup_timeout_s` | `60.0` | 检查超时秒数，至少 12 |
| `session_file` | 空 | 可选输出路径，保存会话 YAML 和 `.timing.json`；默认仅在内存中交接 |

仅在当前时钟会话的时间差已确认时，使用固定时差：

```bash
ros2 launch dual_airy_fusion dual_airy_fusion.launch.py \
  auto_time:=false extrinsics_file:=/path/to/current_session/dual_airy_fusion.yaml
```

## 话题与检查

| 类型 | 默认话题 |
| --- | --- |
| 输入点云 | `/rslidar_front/points`、`/rslidar_rear/points` |
| 启动检查 IMU | `/rslidar_front/imu`、`/rslidar_rear/imu` |
| 输出点云 | `/dual_airy/points_merged`，frame 为 `rslidar_front` |
| 状态 | `/dual_airy/status`，JSON 字符串，`phase` 区分检查与融合 |

点云使用 26 字节 XYZIRT 格式，保留原始 ring 和 intensity，不降采样。节点同时发布 `rslidar_front` → `rslidar_rear` 静态 TF。

前后点云订阅深度（含启动检查阶段）和融合点云发布深度均为 10，默认 Reliable；启动检查 IMU 订阅深度为 200，状态发布深度为 5。

```bash
ros2 topic hz /dual_airy/points_merged
ros2 topic echo /dual_airy/status --field data --once
```

接收频率需在订阅端确认，状态中的 `publish_returned` 仅表示发布调用已返回，不代表实际交付。融合异常会停止输出并保持 `fault` 状态，故障后退出时返回非零状态码。时间差检查估计包含差分传输延迟；节点不执行运动去畸变或持续时钟漂移跟踪。雷达重启、时间源改变或 bag 回绕后重新启动节点；安装变化后重新标定。

默认仅在异常时打印：`[CLOUD_INPUT] front/rear` 只列超标项，时间戳间隔或回调间隔超过 150 ms、处理超过 50 ms、DDS 接收到回调超过 100 ms 时告警，每路最多 5 秒一条。输入停流或输出停滞超过约 1 秒时打印 `[CLOUD_STALL]`。正常运行和启动检查不定时打印状态，失败仍打印错误。

时间缺口仅提示疑似缺帧，不能直接归因于订阅队列；无效 DDS 接收时间不参与等待告警。原有状态统计仍可通过 `/dual_airy/status` 查看。
