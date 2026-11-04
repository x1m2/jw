# v5 启动修订

当前机器狗在取消 ROS_LOCALHOST_ONLY=1 后已恢复，无需为了录包立即替换工作版本。此包用于独立更新和后续重复启动验证，不保证驱动或网络故障时仍能输出数据。

## 行为

- run.sh 默认扫描本机 rslidar_sdk_node 的 /proc 启动环境，只读取 domain、localhost-only、RMW 白名单，不输出其他环境变量。
- 唯一驱动存在时自动沿用其 domain/RMW。取消 localhost-only，脚本固定设置 ROS_LOCALHOST_ONLY=0；如果驱动仍以 1 运行则立即退出，提示 unset 后重启驱动。终端 export 不会改变已经运行的进程。
- 多份本机驱动立即拒绝。没有本机驱动时不猜测其他 domain，沿用当前 domain，默认 0。回放或远端驱动可显式使用 --ros-env inherit，仍取消 localhost-only。
- 检查和融合从开始即统一使用 config/fastdds_fusion.xml，避免第一阶段继承其他 XML。自定义 discovery server 或特殊传输配置不在本次自动适配范围，需单独核对。
- 通过 flock 防止同一用户、domain 的新版脚本重复运行。旧版脚本无此锁，更新前仍须退出旧融合节点。没有驱动启动、停止服务或改动驱动配置的动作。
- 独立时间检查使用三线程执行器，IMU、点云和定时器各有回调组；共享估计器和诊断以互斥锁保护。两路 IMU 订阅深度从 10 增加到 200，点云仍为 2，均为 Reliable。融合阶段仍仅订阅两路点云并串行处理。
- 保留 IMU 20 ms / 点云 40 ms 抖动门限、一致性和 12 s 最短观察要求；超时仍为 60 s，不回退到历史时差。
- 启动报告 streams 包含每路 callbacks_total、callback_age_s（单调时钟）、last_header_s、middleware_age_ms、发现和匹配的发布者、消息类型及可靠性；counts 仍是接收时间窗口计数，不能当累计回调计数。
- blocking_streams 列出缺失或回调超过 1 s 的输入。没有发布者、发现发布者但没有消息、回调新鲜但 DDS 接收时间过期分别报告原因。
- 融合停流超过 1 s 后，publish_call_hz=0，不再显示停流前的历史 10 Hz。计数仍不等于 DDS 交付证明。

## 机器狗更新

停止旧融合节点，驱动只运行一份。在驱动终端 unset ROS_LOCALHOST_ONLY 后重启驱动。保持安装和空间矩阵不变。

```bash
cd ~/robot_rk3588_ws/src/drivers
tar -xzf dual_airy_deploy_cpp_v5.tar.gz
cd dual_airy_deploy_cpp_v5
bash build.sh
bash run.sh
```

源码包不带 build/ 或电脑二进制。矩阵 config/extrinsics.yaml 与 v4 兼容包同步，默认已切换为 packet_225732 的本次 MLCC 候选，状态仍为 EXPERIMENTAL_NOT_ACCEPTED。新会话时差独立重测，不需重新标定空间外参。

每次运行的会话目录还保存 environment.txt 和详细 timing.json。若后续失败，保留整个目录，而非只截图 warming_up。

## 验证范围

本机 Release 构建和 CTest 验证核心融合及模拟 /proc 环境：错误终端 domain 自动对齐、终端残留 localhost=1 被取消、驱动 localhost=1 明确拒绝、多份驱动拒绝和非法 domain 拒绝。

使用 airy_dual_calib_20250604_225738 在隔离 domain 进行无输入诊断和三次独立启动回放，结果见 tests/v5_replay_result.json。测试会检查时间门限通过、实际接收融合点云、重复启动拒绝、停止输入后状态 stale 且发布频率归零。这是 x86 本机验证，不替代机器狗 ARM64 重复启动和完整录包验收。

原 FILE_GUIDE.md、HZ_FIX.md 中关于 v4 的运行组织和实测记录保留作历史说明；v5 的启动差异以本文为准。融合几何、26 字节布局、时差符号及流控预算保持不变。
