# OTA 部署

以下命令在 `robot_ws` 根目录执行。程序保留在 OTA 目录的 `nav`、`slam`、`slam_manager` 中。

## 手动启动

按导航方案选择一个：

```bash
bash deploy/ota/start_node_airy.sh
bash deploy/ota/start_node_mid360s_3d.sh
bash deploy/ota/start_node_mid360s_2d.sh
```

方案说明：

- `mid360s_3d`：原 `start_node_mid360s.sh`，使用 scan planner 和 Fast DDS XML 配置。
- `mid360s_2d`：对应 OTA `nav/test/auto-start_2d`，使用 `terrain_filter + costmap_2d + pure_pursuit`，停障配置为 `obstacle_stop.yaml`；各节点共用 SLAM 的 `fastshm.xml`。
- `airy`：Airy 雷达方案。

默认 OTA 目录为 `/home/jw/ota`；可通过 `OTA_ROOT=/路径` 指定。加 `--check` 只检查文件，不启动节点。

## 开机自启动

按导航方案选择一个：

```bash
sudo bash deploy/ota/systemd/install_nav_autostart.sh airy
sudo bash deploy/ota/systemd/install_nav_autostart.sh mid360s_3d
sudo bash deploy/ota/systemd/install_nav_autostart.sh mid360s_2d
```

默认使用 sudo 调用者及其主目录下的 `ota`。自定义时在雷达参数后加 `用户名 OTA目录`。
安装后执行：

```bash
sudo systemctl restart nav-runtime.service
sudo systemctl status nav-runtime.service
sudo journalctl -u nav-runtime.service -f
```

切换导航方案需重新安装并重启服务；移动脚本或 OTA 目录后需重新安装。

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
- 机器人参数位于 `slam_manager/pack/content/config/`：Airy 用 `p3_airy96.yaml`，MID360s 3D 用 `p2.5_mid360s.yaml`，MID360s 2D 优先用 `p2.5_mid360s.yaml`，没有该文件时使用 `robot_param.yaml`。
- 数据固定保存在 `/home/jw/ota/data/slam_data`，不随 `OTA_ROOT` 改变。
- 运行日志位于数据目录下的 `log/runtime/`。

## MID360s 2D 部署要求

- 导航进程为 `terrain_filter`、`obstacle_stop`、`costmap_2d`、`global_path_planner`、`path_smoother`、`pure_pursuit`，由 Manager 按任务激活 Worker。
- 机器人参数优先选择 `p2.5_mid360s.yaml`，缺失时选择 2D OTA 的 `robot_param.yaml`，也可用环境变量 `ROBOT_PARAM_CONFIG=/绝对路径.yaml` 显式指定。该文件同时传给建图、定位、地形过滤、停障、costmap 和 Pure Pursuit，保持尺寸与外参一致；启动时会打印实际使用的路径。
- Manager 必须是支持 `terrain_filter_worker`、`costmap_worker` 和 `pure_pursuit_worker` 的 **2D 版本**。`slam_manager/test/auto-start_3d` 中的 SCAN 控制器无法通过改脚本切成 2D；需要部署匹配的 2D Manager 和接口库。脚本与服务会拒绝已知的 SCAN-only Manager。
- 每个模块优先加载自身动态库；各模块的消息接口定义仍需保持兼容，启动检查不等于二进制接口兼容性验证。
- 先执行 `OTA_ROOT=/实际/ota bash deploy/ota/start_node_mid360s_2d.sh --check`，通过后再安装并启动服务。检查失败不会停止或启动现有节点。
- 所有必要进程均由直接 PID 监督；任意一个退出会清理其他进程并让 systemd 重启整套服务。各节点输出保存在本次运行日志目录，服务状态记录进入 journal。

- 2D 启动需要 Python 3 与 PyYAML（ROS 2 环境通常已提供）。脚本将 `server.yaml` 中 `/robotfs` 的数据、日志和模块路径映射为部署路径，在数据目录的 `.runtime/` 下生成权限为 600 的 `server_mid360s_2d.yaml`（不放入日志导出目录）。原文件保留；参数规则仍引用原配置目录，参数修改继续写入 OTA 的真实 YAML。

- 2D 服务的一次运行使用一个日志目录，失败后的 systemd 自动重试继续追加到同一目录。`supervisor.log` 记录每次启动尝试和退出原因，节点日志内也有尝试时间标记；正常停止或重启服务后，下次启动创建新目录。直接手动执行脚本则每次创建新目录。已有历史目录保留，不会自动删除。此行为需要重新安装更新后的 systemd 模板。
