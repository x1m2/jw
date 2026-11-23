# UWB 部署与启动

板子上的工作区为 `/home/jw/uwb_ws`，目录结构如下：

```text
/home/jw/uwb_ws/
├── build/
├── install/
├── log/
└── src/
    └── uwb_aoa_pkg/
        ├── start_libAoa_robot.sh
        └── install_uwb_service.sh
```

## 编译

在工作区根目录执行：

```bash
cd /home/jw/uwb_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select uwb_aoa_pkg
```

## 手动启动

串口固定使用 `/dev/ttyFT232`，先检查设备：

```bash
ls -l /dev/ttyFT232
```

手动运行的用户需要串口读写权限。通常将用户加入 `dialout` 组后，重新登录生效：

```bash
sudo usermod -aG dialout "$USER"
```

启动节点（先停止已运行的服务，避免同时占用串口）：

```bash
bash /home/jw/uwb_ws/src/uwb_aoa_pkg/start_libAoa_robot.sh
```

脚本会加载 ROS Humble 和脚本上两级的 `../../install/setup.bash`，然后执行：

```bash
ros2 run uwb_aoa_pkg libAoa_robot_example /dev/ttyFT232
```

## 安装开机自启动服务

编译完成后，以普通用户执行一次：

```bash
bash /home/jw/uwb_ws/src/uwb_aoa_pkg/install_uwb_service.sh
```

安装脚本会通过 sudo 安装 `uwb-aoa.service`，启用开机自启动并立即启动节点。
服务负责设置串口的 `dialout` 组读写权限；设备缺失或启动失败时，每 5 秒重试。
启动节点时不执行编译。

## 服务管理

启动服务：

```bash
sudo systemctl start uwb-aoa.service
```

查看状态（按 `q` 退出）：

```bash
systemctl status uwb-aoa.service
```

查看实时日志（按 `Ctrl+C` 退出）：

```bash
journalctl -u uwb-aoa.service -f
```

重启服务：

```bash
sudo systemctl restart uwb-aoa.service
```

停止服务：

```bash
sudo systemctl stop uwb-aoa.service
```

启用开机自启动并立即启动：

```bash
sudo systemctl enable --now uwb-aoa.service
```

取消开机自启动并停止：

```bash
sudo systemctl disable --now uwb-aoa.service
```

source /opt/ros/humble/setup.bash
source /home/jw/uwb_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/jw/dds/fastdds.xml

# 重启发现服务，让新环境生效
ros2 daemon stop
ros2 daemon start

# 查看 UWB 话题
ros2 topic list | grep -Ei 'uwb|libAoa'

# 查看 UWB 数据
ros2 topic echo /libAoa_robot_publisher
