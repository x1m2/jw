# UWB 开机自启动

将源码包解压到 `/home/jw/uwb_aoa_pkg`，以 jw 用户执行：

```bash
bash /home/jw/uwb_aoa_pkg/install_uwb_service.sh
```

脚本自动创建 `/home/jw/uwb_ws/src`，将包复制到其中（保留解压目录，排除 build/install/log 等生成文件），
在 `/home/jw/uwb_ws` 下执行 colcon 编译，然后安装服务、重载 systemd、启用开机自启并重启服务。
每次执行均重新同步源码并增量编译；仅编译成功后更新服务。

```text
/home/jw/uwb_ws/
├── build/
├── install/
├── log/
└── src/
    └── uwb_aoa_pkg/
        ├── dds/fastdds.xml
        ├── dds/uwb-aoa.service
        ├── start_libAoa_robot.sh
        └── install_uwb_service.sh
```

目标机需已安装 ROS 2 Humble、Fast DDS RMW、colcon 和本包编译依赖，
并使用目标平台匹配的 lib/uwb_robot_algo.a。
安装前核对解压包的 dds/fastdds.xml，网卡 IP 白名单当前是 192.168.50.104，需匹配本机实际 IP。
服务不额外设置 ROS_DOMAIN_ID 和 ROS_LOCALHOST_ONLY，使用 ROS 默认值。

编译使用普通用户，服务安装通过 sudo 完成。
root 执行时默认运行用户 jw，也可在命令末尾指定用户名。
默认工作空间为运行用户家目录下的 uwb_ws，可用 UWB_WS 指定绝对路径。
解压目录及工作空间需允许运行用户读取/写入。

服务模板来自 dds/uwb-aoa.service，安装时按实际路径、运行用户和 ROS 配置生成。
模板保持原内容，仅更新 Fast DDS XML 路径；没有 WantedBy，安装脚本通过
`systemctl add-wants multi-user.target uwb-aoa.service` 建立开机启动关联。
启动脚本只加载 ROS 和 uwb_ws/install/setup.bash 并启动节点，开机不编译。
DDS XML 路径仅由 service 的 FASTRTPS_DEFAULT_PROFILES_FILE 环境变量指定，
指向 uwb_ws/src/uwb_aoa_pkg/dds/fastdds.xml；启动脚本仅检查该变量已设置、文件可读并打印路径，不设置 DDS 环境变量。
串口优先 /dev/ttyUSB0，其次 /dev/ttyUSB1；设置 dialout 组读写权限。
串口缺失或异常退出时每 5 秒重试。确认该串口确实为 UWB 设备。

```bash
# 预览最终服务内容，不复制、不编译、不安装
bash /home/jw/uwb_aoa_pkg/install_uwb_service.sh --print
systemctl status uwb-aoa.service
journalctl -u uwb-aoa.service -f
sudo systemctl restart uwb-aoa.service
sudo systemctl disable --now uwb-aoa.service
bash /home/jw/uwb_ws/src/uwb_aoa_pkg/uninstall_uwb_service.sh
```

修改解压包后可重新执行安装脚本；同名文件会覆盖到工作空间，不删除额外文件。
也可直接修改工作空间内的包，并从该目录执行安装脚本重新编译、更新服务。
卸载服务保留源码和编译产物。
