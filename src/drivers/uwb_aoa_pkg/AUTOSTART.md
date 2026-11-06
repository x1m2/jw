# UWB 开机自启动

板子目录结构如下：

```text
/home/jw/uwb_ws/
├── install/
└── src/
    └── uwb_aoa_pkg/
        ├── start_libAoa_robot.sh
        └── install_uwb_service.sh
```

启动脚本加载上两级的 `../../install/setup.bash`。
先在机器人上进入 `/home/jw/uwb_ws`，编译一次：

```bash
cd /home/jw/uwb_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select uwb_aoa_pkg
```

随后以运行 ROS 节点的普通用户执行（可在任意目录运行）：

```bash
bash /home/jw/uwb_ws/src/uwb_aoa_pkg/install_uwb_service.sh
```

安装脚本会通过 sudo 安装并立即启动 `uwb-aoa.service`，同时启用开机启动。
以 root 执行时，需在脚本后指定普通用户名。安装后移动工作区需重新安装服务。
可使用 `bash install_uwb_service.sh --print` 仅查看生成的服务配置。

服务调用 `start_libAoa_robot.sh`，固定连接 `/dev/ttyFT232`，默认加载 ROS Humble，
开机时不编译。安装前可用 `ROS_DISTRO` 指定其他已安装的 ROS 版本。
串口缺失或节点异常退出时，每 5 秒重试；串口出现后为其设置 dialout 组读写权限。
服务通过附加 dialout 组运行，不要求用户修改个人组成员关系。

```bash
systemctl status uwb-aoa.service
journalctl -u uwb-aoa.service -f
sudo systemctl restart uwb-aoa.service
sudo systemctl disable --now uwb-aoa.service
```

启动脚本只加载环境并启动节点，不执行编译。
手动启动时需事先具备串口读写权限。
