# 文件说明与关键参数

本包负责在线读取已标定外参，将后雷达点变换到前雷达坐标系，按前雷达扫描时间窗合并发布。空间标定已经在离线完成；运行本包不会重新求解空间外参。默认采用 packet_225732 的本次 MLCC 矩阵；其他候选通过 `--extrinsics` 选择。

## 文件对应的职责

| 文件 | 作用 | 优先查看的内容 |
| --- | --- | --- |
| `README.md` | 部署入口、依赖、运行与验收 | 按步骤在机器狗 ARM64 编译，先停旧融合进程 |
| `FILE_GUIDE.md` | 本说明，解释文件与参数 | 默认外参来源、时间符号、字段与日志 |
| `HZ_FIX.md` | 接收频率问题的证据与修复 | DDS 异步发布、分片流控及其限制 |
| `config/extrinsics.yaml` | 运行时空间矩阵、历史时差、来源元数据 | 中文注释解释每个关键字段；默认是本次 packet MLCC 矩阵 |
| `config/fastdds_fusion.xml` | 融合发布进程的 Fast DDS 配置 | 融合话题 writer 为异步，每 1 ms 最多发送 128 KiB |
| `run.sh` | 环境、参数检查、两阶段启动 | 时间检查通过后采用会话副本，再加载 XML 启动融合 |
| `build.sh` | 在当前机器编译 Release 并运行测试 | 默认并行 2 任务；构建使用系统 Python，运行节点均为 C++ |
| `CMakeLists.txt` | 构建目标和依赖 | 共享核心、融合程序、时间检查程序、核心测试 |
| `package.xml` | ROS 包名、版本、依赖元数据 | 不是雷达参数；不用于设置频率或外参 |
| `include/dual_airy/core.hpp` | 核心类型和接口 | Calibration、26 字节记录、FusionBuffer、HostTimeAlignment |
| `src/core.cpp` | 实际数据处理与时间检查算法 | 校验矩阵、解析字段、后雷达变换、时间窗选择、排序 |
| `src/fusion_node.cpp` | 常驻 ROS2 融合节点 | 两路点云订阅、融合发布、静态 TF、状态和性能计数 |
| `src/time_check.cpp` | 独立启动时间检查节点 | 四路 header/接收时间、会话 YAML、超时不启动融合 |
| `tests/core_test.cpp` | 可重复的核心行为测试 | 方向、坏矩阵、非对齐字段、行填充、窗口与时钟异常 |
| `.gitignore` | 排除本地生成目录 | build/install/log 不随源码包分发 |

## 空间矩阵表示什么

```text
             [ R00 R01 R02 tx ]
T_front_rear=[ R10 R11 R12 ty ]
             [ R20 R21 R22 tz ]
             [  0   0   0  1 ]

[p_front;1] = T_front_rear * [p_rear;1]
p_front = R * p_rear + t
```

`t` 单位为米，所有分量在前雷达原生坐标系表达。原生雷达轴系不等于机器狗的前/左/上坐标系，因此不能仅凭 `tz` 的符号判断雷达是在机身上方还是后方。输出 frame 保持 `rslidar_front`，不会自动变为重力对齐的坐标系。

本包默认 `status=EXPERIMENTAL_NOT_ACCEPTED`，表示本次 MLCC 候选尚未取得独立真值验收；节点打印警告并允许测试。`--check` 验证的是文件结构、矩阵合法性和有限性，不能验证物理安装是否正确、外参是否足够准确。

当前默认空间矩阵（截断展示）为：

```text
[ 0.9980606232, 0.0616782325, 0.0084135605, -0.0075300847 ]
[ 0.0616654999, -0.9980953121, 0.0017647011, -0.0062429679 ]
[ 0.0085063790, -0.0012424522, -0.9999630482, -0.9277915512 ]
[ 0.0000000000, 0.0000000000, 0.0000000000, 1.0000000000 ]
```

这里只展示截断值，程序使用 YAML 中的完整精度。本次已按用户选择更新默认矩阵；旧默认配置保存在 config/extrinsics_before_packet_mlcc_20261103.yaml。

## 时间补偿与默认启动流程

`delta_s` 的约定为 `t_front = t_rear + delta_s`。它同时用于后雷达的逐点 timestamp 和帧边界；前雷达时间不变。空间矩阵与时差分别应用，单纯加时差不会按运动轨迹修正点的位置。

默认 `bash run.sh`：

1. 独立检查四路 `/rslidar_front/imu`、`/rslidar_rear/imu`、`/rslidar_front/points`、`/rslidar_rear/points`。
2. 取共同接收窗口，估计每路 `接收时间-header时间` 的中位数。有效时差为后路中位数减前路中位数，包含两路差分传输延迟。
3. 抖动用 P95-P05；IMU 门限 20 ms，点云 40 ms；冻结前两类估计须相差不超过 6 ms，至少三个约每秒估计的极差不超过 2 ms。最早约 12 秒交接，60 秒未通过则退出。
4. 原矩阵与本次时差写到 `/tmp/dual_airy_cpp_session.XXXXXX/extrinsics.yaml`；原输入文件保持不变。会话副本由 YAML 库生成，原文件注释不一定随之保留，报告及元数据仍保留。
5. 检查进程退出，常驻融合进程只订阅两个点云输入，时差在本次会话固定。

这套检查没有设置主机/雷达时钟，也不是 PTP 或硬件触发。门限通过不代表已知真实时差误差小于 1～2 ms。传感器重启、时钟源改变或 bag 回绕后，应重启脚本重新检查；常驻融合阶段不持续跟踪漂移。

`--fixed-time` 会直接使用所选 YAML 的 `delta_s`。包内历史值 -9.277 ms 只属于历史会话，不适用于当前所有运行。实机曾测得 -27.615 ms，该数值也应随本次检查确定，不写死成永久值。

## 点如何合成一帧

融合窗口按前雷达扫描开始时间划分为 `[本帧开始, 下一帧开始)`。每个窗口可包含一帧前雷达数据和相邻后雷达帧的一部分，按校正后的逐点时间选择，不是简单把两份最近的完整扫描相加。

等待下一前雷达帧通常带来约一帧缓存；稳定后仍可每 0.1 秒输出一帧。前雷达收到 N 帧、发布 N-1 帧且缓存维持 1 帧，符合这一设计。200 ms 的定时器只处理状态与停流末帧，不负责限制融合频率为 5 Hz。

无效 XYZ/逐点时间会被过滤，输出按采样时间稳定排序，不做体素降采样。时间倒退、字段错误等会锁定 fault，修复原因后重启；覆盖不足的窗口会丢弃，不伪造点云。

## 话题、字段和 TF

| 内容 | 默认值 |
| --- | --- |
| 输入点云 | `/rslidar_front/points`、`/rslidar_rear/points` |
| 输出点云 | `/dual_airy/points_merged` |
| 输出 frame | `rslidar_front` |
| 输出 QoS | Reliable，KeepLast 5 |
| 输入 QoS | 默认 Reliable，KeepLast 2 |
| 状态 | `/dual_airy/status`，std_msgs/String JSON |
| 静态 TF | parent=`rslidar_front`，child=`rslidar_rear` |

| 字段 | 偏移/字节 | 类型 | 含义 |
| --- | ---: | --- | --- |
| x/y/z | 0/4/8 | float32 | 前雷达原生轴系中的坐标，单位米 |
| intensity | 12 | float32 | 原反射强度，保持原值 |
| ring | 16 | uint16 | 各雷达原始线号，保持原值；不能仅凭 ring 区分来源雷达 |
| timestamp | 18 | float64 | 前雷达时钟域中的绝对秒，后路已加本次 delta |

总 `point_step=26`。timestamp 未按 8 字节对齐，C++ 使用 memcpy 读取，避免 ARM64 的非对齐访问问题。内部 Point 结构带缓存时间和对齐填充，不能直接按其 sizeof 当作线上点格式。

header 是输出首点采样时间，不是主机当前时间。没有运动去畸变或跨时刻运动补偿：各点仍对应各自采样时刻的前雷达轴系，不能用一个静态 TF 消除扫描期间的运动。

## 日志怎么读

| 项目 | 解释 |
| --- | --- |
| `front_received/rear_received` | 成功解码并进入缓存的扫描数，不是驱动 UDP 包统计 |
| `generated` | 组装出的融合扫描数 |
| `publish_returned`、`published` | publish() 已返回次数；两者含义相同，不是 DDS 接收确认数 |
| `startup_dropped` | 缺少窗口开头后雷达覆盖而丢弃的前窗口 |
| `unmatched_dropped` | 无有效前/后点或结束时后雷达覆盖不足而丢弃的窗口 |
| `overflow_dropped/rear_overflow_dropped` | 前/后缓存达到上限时丢弃的扫描数 |
| `front_points/rear_points` | 实际纳入已组装输出的前/后点累计数量 |
| `stage_ms`、`callback_ms` | 最近 100 次耗时的 P50/P95/max，不是所有历史的最坏耗时 |
| `publish_call_hz` | 最近调用返回频率，不能替代接收端 Hz |
| `latest_front_minus_output_stamp_s` | 最新前扫描时间减输出首点时间，通常约 0.1 秒；不是端到端网络延迟 |
| `queued_front/queued_rear` | 当前扫描缓存长度，应避免持续增长 |
| `matched_subscribers` | 当前已匹配订阅者数量，0 可表示还没启动测频或下游 |

Hz 是发布/传输可用性的一项指标，与空间外参精度分别验收。
