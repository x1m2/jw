# 为什么原始点云 Hz 正常，融合点云曾出现数秒空洞

本包对大点云发布端增加了 Fast DDS 异步发送和分片流控；C++ 版还承担解码、坐标变换、分窗和排序。此前的同数据对照显示，仅换语言或增大 writer 队列不能稳定解决默认 Best Effort 接收缺帧。当前修复有接收端测量支持，不只是观察节点内部“已发布”计数。

## 1. 当时测到了什么

机器狗上的原输入约 10 Hz。旧融合节点内部计数也约 10 Hz，但普通 `ros2 topic hz /dual_airy/points_merged` 只有约 2～3 Hz，间隔可达数秒。独立 Reliable 接收测试又在约 20 秒收到 201 帧、约 10.01 Hz，header 没有缺口；消息平均约 3.11 MB，约 31.2 MB/s。

这些结果表明同一发布话题的可靠交付能够维持 10 Hz，但旧发布方式未满足默认 Best Effort 接收链路。所测试的 Humble `ros2 topic hz` 使用 SensorDataQoS/Best Effort；低 Hz 不能简单解释为文字显示错误，也不能据节点计数断言所有订阅者已收到。

原雷达驱动和融合节点的输出不是相同负载。融合帧通常更大，约 3～4 MB，实际大小随有效点数变化。两者的计算方式、发布时序和 DDS 配置也可能不同；驱动 C++ 使用默认 Reliable/队列 100，并不能保证任意更大消息都具有相同接收表现。

## 2. 如何区分计算问题与传输问题

此前用同一 bag 派生的“已经组装好的完整 PointCloud2”做对照，发布过程不再做坐标变换、配对或排序。电脑上的单次约 10 秒 Best Effort 接收示例：

| 发布方式 | 消息大小 | writer depth | 整段实收频率 |
| --- | ---: | ---: | ---: |
| C++ typed | 3.12 MB | 5 | 7.57 Hz |
| C++ typed | 3.12 MB | 100 | 4.78 Hz |
| Python typed | 3.12 MB | 5 | 2.30 Hz |

这说明问题能在没有融合计算的发布试验中复现，C++ 和大队列本身没有解决整条接收链路。上述单次电脑测试不是机器狗的性能上限，也不足以把语言差异归因于唯一原因。完整历史对照保存在工作空间 `tools/dual_airy_diagnostics/publisher_audit_result.md`。

## 3. 直接针对大消息发送突发的改动

`config/fastdds_fusion.xml` 中：

```xml
<flow_controller_descriptor>
  <name>airy_paced_flow</name>
  <scheduler>FIFO</scheduler>
  <max_bytes_per_period>131072</max_bytes_per_period>
  <period_ms>1</period_ms>
</flow_controller_descriptor>
```

FIFO 流控预算为每 1 ms 最多发送 128 KiB。约束的是 DDS 大消息分片的发送突发，不是把一个扫描拆成多个 ROS 点云消息，也没有降采样或降低传感器扫描频率。名义预算约 131 MB/s，实际吞吐受传输、订阅者数量与机器负载影响。

融合话题单独绑定该控制器：

```xml
<data_writer profile_name="/dual_airy/points_merged">
  <qos>
    <publishMode>
      <kind>ASYNCHRONOUS</kind>
      <flow_controller_name>airy_paced_flow</flow_controller_name>
    </publishMode>
    <data_sharing><kind>OFF</kind></data_sharing>
  </qos>
  <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>
</data_writer>
```

应用层 publish() 将消息交给 DDS，由异步发送线程按照控制器预算发送。点云话题仍是完整的一帧、约 10 Hz。`DYNAMIC` 使历史内存可按实际消息大小分配。`data_sharing=OFF` 是 Data Sharing 配置，不等于禁用了 Shared Memory Transport，更不能据此认定共享内存是原故障源。

Fast DDS 官方说明，大样本会被分片，Best Effort 模式下缺少分片会使完整消息无法交付；流控可用于减少发送突发。[Fast DDS 大数据率说明](https://fast-dds.docs.eprosima.com/en/v2.6.11/fastdds/use_cases/large_data/large_data.html)

## 4. XML 如何真正作用到融合节点

`run.sh` 在启动融合进程前设置：

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$ROOT/config/fastdds_fusion.xml"
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
```

这里是说明默认值，脚本会检查用户已选的 RMW。`RMW_FASTRTPS_USE_QOS_FROM_XML=1` 允许采用 XML 中的发布模式及历史内存策略。[rmw_fastrtps Humble 配置说明](https://github.com/ros2/rmw_fastrtps/blob/humble/README.md)

这几个变量只作用于启动脚本及后续进程，不会改变已经运行的驱动或其他终端。仅在另一个终端 export 不会追溯修改现有节点；直接运行二进制也不会自动加载本包 XML。应从 `bash run.sh` 启动。

`profile_name` 与 ROS 完整输出话题名称匹配，因此改名或重映射 `/dual_airy/points_merged` 时需同步核对 XML，避免节点仍正常启动但融合 writer 没有采用流控。

## 5. C++ 与两阶段启动分别解决什么

- 解码、后雷达矩阵变换、时间选择、稳定排序与序列化数据装配采用 C++，减少 Python 对象/数组处理开销；保持 26 字节 XYZIRT。unique_ptr 移交减少一次应用层消息复制，不代表整个 DDS 链路零复制。
- 两个点云回调在 SingleThreadedExecutor 中串行处理，缓存无并发修改。DDS 的接收和异步发送线程仍独立运行。
- 启动时间检查独立执行：先只检查四路时间，退出后再启动点云融合。它避免融合处理干扰 IMU 接收时间统计，针对的是早期“时间检查被大幅回调延迟污染”的问题。
- 日志区分 `generated` 与 `publish_returned`，并加入各阶段耗时和缓存长度，防止把内部计数当作接收端 Hz。

传输流控和这些运行组织改进一起保留在当前包内。没有在机器人上做逐项禁用的完整因果实验，不能声称每项改动分别贡献了多少 Hz。

## 6. 修复效果与边界

加流控后的电脑发布对照中，3.12 MB 和 4.34 MB 点云在两个 Best Effort 接收器并行时都恢复到约 10 Hz。Python v3 真实融合回放也已达到约 10 Hz，说明“必须换 C++ 才能解决传输问题”不成立。

C++ 默认启动完整回放验证：20.067 秒收到 200 帧，首尾接收频率 10.0022 Hz，header 无倒退、无 >150 ms 缺口。普通 ros2 topic hz 同时运行，最后五次为 9.989、9.997、9.996、9.996、9.992 Hz。

用户随后提供的 RK3588 实机结果：连续约 48 秒测到约 10 Hz，末次 9.995 Hz，接收间隔 72～141 ms；节点状态 ready，缓存没有持续增长，所给日志内未配对/缓存溢出计数为 0。回调 P95 约 33～35 ms。此前的数秒接收空洞在这段实测中没有再出现。

仍应分别理解：

- 正常的 Hz 和扫描计数不能证明雷达 UDP 包完全无丢失，也不能证明空间外参准确。
- 到达间隔存在调度/传输抖动；header 时间间隔与主机接收间隔是两种指标。
- 流控与扫描窗口等待会带来延迟；约一帧的缓存时间差不是端到端延迟测量。
- 当前没有抓包定位究竟在哪个内核/DDS/共享内存缓冲层丢失，不能把某个具体缓冲溢出当作已证实根因。
- 更高负载、更多订阅者或跨主机低带宽场景仍需重新验收，流控没有增加物理链路带宽。

本次更新只增加中文注释和说明，不改变以上运行参数，也没有重新测量机器狗；实机数字来自用户提供的日志。

## 7. 以后如何快速检查

先确认只有一个融合发布者，使用与驱动相同的 ROS_DOMAIN_ID，从 `run.sh` 启动，终端应显示 XML 的绝对路径。另开终端执行：

```bash
source /opt/ros/humble/setup.bash
ros2 topic hz /dual_airy/points_merged
ros2 topic echo /dual_airy/status --field data --once
```

持续观察至少 20～30 秒。若接收 Hz 低，先核对实际接收，再看 stage_ms、缓存增长、丢弃计数和订阅者数量；仅 `publish_call_hz=10` 不能排除传输缺帧。
