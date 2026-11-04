# 双 Airy C++ 独立部署包

将后雷达点云变换到前雷达坐标系，按逐点时间合并为 `/dual_airy/points_merged`，frame=`rslidar_front`。启动时间检查与融合进程均为 C++，运行不依赖 Python 节点、NumPy、SciPy 或 PCL。只包含本部署所需源码、配置、脚本及测试；不包含 bag、日志或电脑二进制。

2026-11-03 已按用户选择，将默认 `config/extrinsics.yaml` 切换为 `dual_airy_packet_20250604_225732` 的本次 MLCC 矩阵；状态仍为 `EXPERIMENTAL_NOT_ACCEPTED`。它是读取标定结果的在线融合包，空间标定算法不在常驻节点中执行。旧默认配置保存在 `config/extrinsics_before_packet_mlcc_20261103.yaml`。

本次 v5 修订启动通信和接收诊断：取消 localhost-only，自动核对唯一的本机驱动 domain/RMW，两个阶段统一加载本包 DDS XML，IMU 和点云分组调度，增加逐路发现/累计接收/新鲜度诊断及重复启动锁。停流后发布频率显示为 0，避免误读历史 10 Hz。v5 初版的启动修订保留了空间矩阵；当前默认矩阵已按 2026-11-03 用户选择更新。核心融合算法、时差验收门限和 DDS 流控预算保持不变。

用户随后确认取消 ROS_LOCALHOST_ONLY=1 后当前版本已经恢复；当前能用的版本无需立即替换。v5 可独立保留备用，不覆盖原驱动或旧部署目录。

- [v5 启动行为、更新和验证说明](STARTUP_V5.md)

- [逐文件说明、矩阵和时间含义、日志字典](FILE_GUIDE.md)
- [Hz 问题的证据、具体修复和验收边界](HZ_FIX.md)

## 更新与启动

先在旧融合终端 Ctrl+C，保留两台雷达驱动运行。避免多个融合节点同时发布同一话题。新包放到独立目录，不覆盖旧包、驱动、FAST-LIO 或 robot_param.yaml。

在机器狗 `~/robot_rk3588_ws/src/drivers/` 下：

```bash
tar -xzf dual_airy_deploy_cpp_v5.tar.gz
cd dual_airy_deploy_cpp_v5
source /opt/ros/humble/setup.bash
bash build.sh
bash run.sh --check
bash run.sh
```

`build.sh` 在当前机器生成 Release 二进制，并运行核心测试；默认并行编译 2 个任务。内存不足可用 `DUAL_AIRY_BUILD_JOBS=1 bash build.sh`。电脑的 x86 二进制不能复制给 RK3588，因此压缩包只提供源码，在机器狗 ARM64 上编译。

需要 ROS2 Humble、C++17 编译器、CMake、ament_cmake、rclcpp、sensor_msgs、std_msgs、geometry_msgs、tf2、tf2_ros、rmw_fastrtps_cpp、yaml-cpp、nlohmann_json。缺少开发依赖时，在可访问软件源的机器狗安装：

```bash
sudo apt install build-essential cmake libyaml-cpp-dev nlohmann-json3-dev \
  ros-humble-ament-cmake ros-humble-rclcpp ros-humble-sensor-msgs \
  ros-humble-std-msgs ros-humble-geometry-msgs ros-humble-tf2 \
  ros-humble-tf2-ros ros-humble-rmw-fastrtps-cpp
```

CMake 的 ament 配置阶段仍使用 ROS 的系统 Python 工具；两个运行节点都是本地 C++ 可执行文件。

沿用驱动的 `ROS_DOMAIN_ID`，默认 0。脚本自动加载 `/opt/ros/humble/setup.bash`。默认流程：

1. 先核对唯一的本机驱动环境，ROS_LOCALHOST_ONLY 固定为 0；旧驱动仍以 1 运行或本机存在多份驱动时立即提示退出。检查器和融合共用 DDS XML。C++ 时间检查进程以三个执行线程分组订阅前后 points、imu 四路数据。优先采用有效的 DDS 接收时间，否则采用回调系统时间；IMU/点云接收抖动门限分别为 20/40 ms，并检查两类数据估计的一致性。
2. 最早约 12 秒检查通过，保存本次 `delta_s`、报告和完整矩阵到 `/tmp/dual_airy_cpp_session.XXXXXX/`，退出检查进程。60 秒未通过则退出，不启动融合、不沿用历史时差。
3. C++ 融合进程只订阅两路点云，使用本次固定时差。保留 v3 发布端 Fast DDS 异步分片流控，输出完整 XYZIRT 消息；不需要改变驱动或测频终端配置。

雷达重启、bag 回绕或时间源变化后重启脚本。融合阶段不持续跟踪时钟漂移。空间安装改变后必须重新标定。

仅当前时钟会话的时间关系已经验证时使用：

```bash
bash run.sh --fixed-time --extrinsics /path/to/current_session/extrinsics.yaml
```

自带 `delta_s=-0.006360054016113281` 仅对应 packet_225732 bag 的时钟会话，不能跨会话使用。默认启动检查会替换它。

## 矩阵外参

唯一空间外参是 `config/extrinsics.yaml` 中的 `T_front_rear`，为后雷达到前雷达的 4×4 齐次变换。采用列向量，平移单位米：

```
[x_F, y_F, z_F, 1]^T = T_front_rear * [x_R, y_R, z_R, 1]^T
```

矩阵右上列是平移，左上 3×3 是旋转，最后一行为 `[0,0,0,1]`。程序检查矩阵维度、有限性、旋转正交性及行列式 +1。它不是反方向的变换，也不是机身坐标系安装参数。

当前默认使用本次 packet_225732 MLCC 4×4 矩阵，安装须与该 bag 保持一致。旧地图法候选和上次 225738 MLCC 候选留作历史对照，未写入 robot_param.yaml。时间独立使用 `delta_s`，约定 `t_front = t_rear + delta_s`，每次启动重新检查。

## 话题与数据

- 输入：`/rslidar_front/points`、`/rslidar_rear/points`；启动时间检查还需 `/rslidar_front/imu`、`/rslidar_rear/imu`。
- 输出：`/dual_airy/points_merged`，Reliable，队列 5，frame=`rslidar_front`。
- 状态：`/dual_airy/status`，JSON，包含 `implementation=cpp`、`version=5`、计数及最近 100 次处理耗时。
- 静态 TF：`rslidar_front` → `rslidar_rear`，与同一个矩阵一致。若已有相同 TF，请避免重复发布。

输入和输出保持小端 26 字节 XYZIRT：x/y/z/intensity float32，ring uint16，timestamp float64 绝对秒。后雷达坐标和逐点时间经转换后，按前雷达时间窗合并并稳定排序；过滤非有限 XYZ/无效时间，不降采样。header 使用窗口内首点时间。ring 保留两台雷达各自的原始值。

节点等待下一个前雷达帧以确定窗口，通常引入约一帧的缓存等待；DDS 发送也有额外延迟。输出仍是原始扫描，每个点位于其采样时刻的前雷达轴系，不做运动去畸变、跨时刻运动补偿或 SLAM。仅凭此节点不能保证大幅运动时没有重影。

时间戳倒退、字段错误等会锁定 fault 并停止输出，需处理原因后重启。超时缺流不会伪造点云。没有启动驱动、RViz、bag、SLAM、系统调时或修改机器人配置的功能。

## 更新后检查

另开终端，使用与驱动相同的 ROS_DOMAIN_ID：

```bash
source /opt/ros/humble/setup.bash
ros2 topic hz /dual_airy/points_merged
ros2 topic echo /dual_airy/status --field data --once
```

观察至少 20～30 秒。目标约 10 Hz、没有数秒空洞；检查 `implementation=cpp`、`version=5`，且 `state=ready`。`generated` 是组装数量，`publish_returned`/`published` 是 publish() 返回数量，不能替代接收端统计。`performance` 给出各阶段 P50/P95/max、缓存长度和发布调用频率。

XML 流控 profile 精确匹配 `/dual_airy/points_merged`。更改输出话题时需同步修改 XML。流控预算是每 1 ms 131072 字节；实际吞吐仍受机器狗负载、传输和订阅者数量影响。保留当前 Fast DDS 设置是为控制大消息发送突发，不能用换 C++ 替代接收端验收。

## 已完成的本机验证

ROS2 Humble/x86_64，使用最新 `airy_dual_calib_20250604_225738` bag：

- Release 编译、核心测试通过：矩阵方向与错误配置拒绝、ARM 安全的非对齐字段读取、行填充、无效点过滤、窗口拆分、时间排序、启动时间检查及异常拒绝。
- 44 个真实输入点云与上一版 Python 实现离线对照：21 帧融合云、3467498 点；点数、逐点时间、ring、intensity 相同。最大 XYZ 数值差约 1.91 微米，来自浮点运算顺序。
- 完整默认流程从时间检查切换到融合成功。本次 bag 接收时差估计约 -7.395 ms，仅用于该回放会话。
- 普通 `ros2 topic hz` 与另一个 Best Effort 接收器同时运行：20.067 秒接收 200 帧，首尾频率 10.0022 Hz，header 无倒退、无超过 150 ms 的缺口。原生 hz 最后五次为 9.989、9.997、9.996、9.996、9.992 Hz；平均约 4.283 MB/帧。
- 本次回放期间计数无启动丢弃、未配对丢弃或缓存溢出。回调 P95 约 19.72 ms。数值仅为本机测试，不代表 RK3588 性能；机器狗仍需按上述命令复测。

压缩包不附带测试 bag、电脑 build 目录或测试日志。源码内 `tests/core_test.cpp` 可随 `build.sh` 在机器人重复验证核心行为。

## 用户提供的 RK3588 实机结果

随后用户提供的 C++ 版日志中，普通 `ros2 topic hz` 连续约 48 秒保持约 10 Hz，最后为 9.995 Hz，接收间隔 72～141 ms；状态 `ready`，所给日志的缓存溢出和未配对计数为 0，回调 P95 约 33～35 ms。这段实测未再出现之前数秒的接收空洞。

这些是当前机器狗发布与接收链路的证据，不是外参准确性或运动去畸变的验收。本次只补注释并重编译验证，没有重新连接机器狗测频；更多过程见 [Hz 修复说明](HZ_FIX.md)。

## 2026-11-03 默认矩阵更新

当前默认矩阵为 packet_225732 的本次 MLCC 结果。历史 Hz/启动回放记录保留为当时版本证据，不作为新矩阵的独立精度验收。矩阵切换不改变输入输出话题和启动时间检查；旧默认配置可用 `--extrinsics config/extrinsics_before_packet_mlcc_20261103.yaml` 显式选择。
