# UWB 板端部署流程

目标板 IP：192.168.50.104，运行用户：jw。
将更新后的完整源码包解压到 `/home/jw/uwb_aoa_pkg`，不要只复制三个修改文件。
目标板需已安装 ROS 2 Humble、colcon、编译工具及本包依赖。
包中 lib/uwb_robot_algo.a 必须与板子架构匹配。

## 1. 登录并检查文件

```bash
ssh jw@192.168.50.104
ls /home/jw/uwb_aoa_pkg/package.xml
ls /home/jw/uwb_aoa_pkg/dds/{uwb-aoa.service,fastdds.xml}
ip -4 addr
ls -l /dev/ttyUSB*
```

包内 dds/fastdds.xml 的网卡白名单为 192.168.50.104，需确认本机确有该 IP。
启动脚本优先选择 ttyUSB0，其次 ttyUSB1；确认对应设备是 UWB 串口。
解压包与工作空间需允许 jw 读写，使用 jw 用户运行下一步。

## 2. 一条命令安装并启动

```bash
bash /home/jw/uwb_aoa_pkg/install_uwb_service.sh
```

脚本自动创建 /home/jw/uwb_ws/src，将包复制进去（保留原解压包，跳过旧 build/install/log），
在 uwb_ws 下编译。成功后通过 sudo 安装 /etc/systemd/system/uwb-aoa.service，
重新加载 systemd、建立开机启动关联并立即重启服务。不需要再手动执行启动脚本。
service 的 DDS XML 路径为 /home/jw/uwb_ws/src/uwb_aoa_pkg/dds/fastdds.xml。
启动脚本只检查 DDS 变量和文件，由 Fast DDS 读取配置；不强制指定 RMW。
没有 WantedBy 的原 service 通过 systemctl add-wants multi-user.target uwb-aoa.service 设置开机启动。

## 3. 检查服务及数据

```bash
systemctl status uwb-aoa.service --no-pager -l
systemctl is-enabled uwb-aoa.service
journalctl -u uwb-aoa.service -n 50 --no-pager
```

状态应为 active (running)，开机启动状态为 enabled。
串口缺失或节点退出会每 5 秒重试，需结合日志和话题数据确认正常。

```bash
source /opt/ros/humble/setup.bash
source /home/jw/uwb_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/jw/uwb_ws/src/uwb_aoa_pkg/dds/fastdds.xml
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
ros2 daemon stop
ros2 topic list --no-daemon
ros2 topic echo /libAoa_robot_publisher
```

服务没有设置 ROS_DOMAIN_ID，通常使用默认域 0；检查终端需与节点一致。
当前 service 不指定 RMW，ROS Humble 默认使用 Fast DDS；若板端默认 RMW 被改成其他实现，需另行确认。

## 服务管理

```bash
sudo systemctl restart uwb-aoa.service
sudo systemctl stop uwb-aoa.service
sudo systemctl disable --now uwb-aoa.service
bash /home/jw/uwb_ws/src/uwb_aoa_pkg/uninstall_uwb_service.sh
```

手动启动前先停止服务，并确保当前用户有串口读写权限及 DDS 环境变量：

```bash
sudo systemctl stop uwb-aoa.service
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/jw/uwb_ws/src/uwb_aoa_pkg/dds/fastdds.xml
bash /home/jw/uwb_ws/src/uwb_aoa_pkg/start_libAoa_robot.sh
```

详细参数与目录说明见 [AUTOSTART.md](AUTOSTART.md)。
