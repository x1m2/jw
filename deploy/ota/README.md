# OTA 部署

以下命令在 `robot_ws` 根目录执行。程序保留在 OTA 目录的 `nav`、`slam`、`slam_manager` 中。

## 手动启动

按雷达二选一：

```bash
bash deploy/ota/start_node_airy.sh
bash deploy/ota/start_node_mid360s.sh
```

默认 OTA 目录为 `/home/jw/ota`；可通过 `OTA_ROOT=/路径` 指定。加 `--check` 只检查文件，不启动节点。

## 开机自启动

按雷达二选一：

```bash
sudo bash deploy/ota/systemd/install_nav_autostart.sh airy
sudo bash deploy/ota/systemd/install_nav_autostart.sh mid360s
```

默认使用 sudo 调用者及其主目录下的 `ota`。自定义时在雷达参数后加 `用户名 OTA目录`。
安装后执行：

```bash
sudo systemctl restart nav-runtime.service
sudo systemctl status nav-runtime.service
sudo journalctl -u nav-runtime.service -f
```

切换雷达需重新安装并重启服务；移动脚本或 OTA 目录后需重新安装。

卸载自启动（保留程序和数据）：

```bash
sudo bash deploy/ota/systemd/uninstall_nav_autostart.sh
```

## 更新程序

```bash
OTA_ROOT=/home/jw/ota bash deploy/ota/update_ota_from_install.sh /home/jw/robot_ws/install --dry-run
```

去掉 `--dry-run` 执行更新，再重启服务。只更新现有二进制、动态库和 XML，保留 YAML。

## 配置与数据

- MID360s 启动前检查 `slam/pack/content/config/MID360s_config.json` 中的主机和雷达 IP。
- 机器人参数位于 `slam_manager/pack/content/config/`：Airy 用 `p3_airy96.yaml`，MID360s 用 `p2.5_mid360s.yaml`。
- 数据固定保存在 `/home/jw/ota/data/slam_data`，不随 `OTA_ROOT` 改变。
- 运行日志位于数据目录下的 `log/runtime/`。
