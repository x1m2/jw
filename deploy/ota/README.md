# OTA 部署

以下命令在 `robot_ws` 根目录执行。程序保留在 OTA 目录的 `nav`、`slam`、`slam_manager` 中。

`update_ota_from_install.sh` 仅更新 3D SCAN 部署及其公共组件；跳过旧 2D 的
`terrain_filter`、`costmap_2d`、`pure_pursuit` 和 `uwb_follow_planner` 程序及专用库。
搜索 XML 和动态库来源时也排除这些旧包。已有 2D 文件不会被删除。

`/home/jw/ota/data/slam_data/current_runtime_log.txt` 按从旧到新保存最近五次
运行的日志目录，每行带编号，最后一行标记“最新”。编号按保留的记录重新从 1 排起。
同一会话的自动重试不重复占用记录；兼容已有的单行纯路径格式。
这里只限制路径记录条数，历史日志目录不会自动删除。

BBS 库统一由 `3d_bbs` 构建：CPU 板先运行 `./src/third_party/3d_bbs/build_cpu.sh`，
OpenCL 板先运行 `./src/third_party/3d_bbs/build_opencl.sh`。定位包仅链接已安装的库，
分别使用 `-DENABLE_OPENCL_BBS=OFF/ON` 编译。
更新脚本默认从 `install/bbs3d/lib/` 获取 BBS 库，从
`install/bbs3d/bin/voxelmaps_saver` 获取地图转换工具；自定义前缀通过
`BBS_INSTALL_DIR=/安装目录` 指定。新工具会补充到 `slam/pack/content/localization/voxelmaps_saver`。

## 手动启动

按导航方案选择一个：

```bash
bash deploy/ota/start_node_airy.sh
bash deploy/ota/start_node_mid360s_3d.sh
bash deploy/ota/start_node_mid360s_2d.sh
```

方案说明：

- `mid360s_3d`：原 `start_node_mid360s.sh`，使用 scan planner 和 Fast DDS 默认配置，不依赖 `fastshm.xml`。
- `mid360s_2d`：对应 OTA `nav/test/auto-start_2d`，使用 `terrain_filter + costmap_2d + pure_pursuit`，停障配置为 `obstacle_stop_p2.5_mid360s.yaml`；使用 Fast DDS 默认配置，不依赖 `fastshm.xml`。
- `airy`：Airy 雷达方案，检查并加载 `slam`、`nav` 中的 `fastshm.xml`。

默认 OTA 目录为 `/home/jw/ota`；可通过 `OTA_ROOT=/路径` 指定。加 `--check` 只检查文件，不启动节点。

三个启动脚本均支持 `--localization fastlio|superlio`；Airy、Mid360 2D 和 Mid360 3D 均默认 `superlio`。
自启动服务安装脚本使用相同默认值。
也可设置环境变量 `LOCALIZATION_ALGORITHM`；命令行选项优先。
例如 Airy 使用 Super LIO，先检查再启动：

```bash
bash deploy/ota/start_node_airy.sh --localization superlio --check
bash deploy/ota/start_node_airy.sh --localization superlio
```

选择会同步切换定位程序、传感器配置和 `slam_manager` 的
`localization.worker_service`。定位 Worker 以 `platform` 模式启动，等待 Manager 激活。
建图仍使用 FAST-LIO 和 PGO。

只需独立建图或检查车体系点云时，可在源码仓库根目录运行：

```bash
bash src/run_mapping.sh --lidar airy --ota-root /实际/ota --check
DATA_DIR=/实际/建图目录 bash src/run_mapping.sh --lidar airy --ota-root /实际/ota
```

此模式直接执行 OTA 驱动、FAST-LIO 和 PGO，并以 `runtime_profile:=test`
自动激活，不需要 `slam_manager` 或源码工作空间的 `install/setup.bash`。
脚本也可单独复制到板端运行。MID360 使用 `--lidar mid360`。
Airy 加载 OTA 中的 `mapping_airy96.yaml` 和小写 `p3_airy96.yaml`；配置路径与
本页的完整 OTA 启动脚本一致。RViz 使用 `base_link` 和 `/cloud_body_downsampled`。
先停止已有雷达/建图服务；退出时 Ctrl+C 保存地图并生成 BBS 数据，默认保存到
`OTA_ROOT/data/slam_data/standalone_<时间>_<进程号>`，可通过 `DATA_DIR` 指定。

两种定位算法的 OTA 文件分开存放，避免同名 YAML 相互覆盖：

| 算法 | `slam/pack/content/localization/` 下的程序 | `slam/pack/content/config/` 下的配置 |
| --- | --- | --- |
| `fastlio` | `fastlio_localization_node` | `reloc_fastlio_airy96.yaml`、`reloc_fastlio_mid360s.yaml` |
| `superlio` | `superlio_localization_node` | `reloc_superlio_airy96.yaml`、`reloc_superlio_mid360s.yaml` |

两套配置分别来源于 `src/slam/localization/fastlio_localization/config/reloc_fastlio_*.yaml`
和 `src/slam/localization/super_lio/config/reloc_superlio_*.yaml`，部署时保留原文件名。
Super LIO 程序编译安装后位于
`install/super_lio/lib/super_lio/superlio_localization_node`；
同时将所需的 `liblio`、`basic`、BBS 等动态库及其依赖部署到 `slam/pack/content/lib/`。
使用目标板对应架构构建，并检查动态库依赖。安装服务不会替你编译或复制程序。
机器人尺寸和 `lidar_to_base_T/R` 仍使用对应雷达的共享机器人配置。

## 开机自启动

以下安装、权限和卸载脚本命令在 `systemd` 目录执行：仓库中为
`deploy/ota/systemd/`，部署到板端后通常为 `ota/systemd/`。

按导航方案选择一个：

```bash
sudo bash  install_nav_autostart.sh airy
sudo bash  install_nav_autostart.sh mid360s_3d
sudo bash  install_nav_autostart.sh mid360s_2d
```

安装时同时选择定位算法，例如：

```bash
sudo bash  install_nav_autostart.sh airy --localization superlio
sudo bash  install_nav_autostart.sh mid360s_3d --localization fastlio
sudo bash  install_nav_autostart.sh mid360s_2d --localization superlio jw /home/jw/ota
```

未指定算法时保持 `fastlio`，兼容原安装命令。算法保存在 systemd 服务环境中，
重启和开机启动均使用同一选择；安装前检查所选定位程序和对应配置是否存在。
切换算法后需重新安装并重启服务。

默认使用 sudo 调用者及其主目录下的 `ota`。自定义时在雷达参数后加 `用户名 OTA目录`。
安装器同时为运行用户配置客户端免密重启权限，三个方案共用此流程。
板端需要安装 `sudo`（包含 `visudo`）；权限校验失败会中止安装。
授权文件为 `/etc/sudoers.d/nav-runtime-restart`，仅允许固定命令：

```bash
/usr/bin/systemctl --no-block restart nav-runtime.service
```

授权不包含其他服务或任意命令；服务仍按指定用户运行。重复安装时替换原运行用户的授权，
运行用户为 root 时无需授权并清理旧规则。安装器不会立即启动或重启服务。

已有板子只补装重启权限时，先确认服务的实际运行用户：

```bash
systemctl show nav-runtime.service --property=User --value
# 以下 jw 替换为上面输出的用户
sudo bash  install_runtime_restart_permission.sh jw
```

检查配置或权限，不实际重启：

```bash
bash  install_runtime_restart_permission.sh --check jw
# 以下命令以服务运行用户执行
sudo -n -l -- /usr/bin/systemctl --no-block restart nav-runtime.service
```

`--check` 仅检查待生成规则的语法；第二条命令检查当前已安装的授权。
首次安装或补装授权需要管理员权限，以后通过客户端重启无需输入密码。
卸载自启动时同步移除上述授权，保留 OTA 程序和数据。

安装后执行：

```bash
sudo systemctl restart nav-runtime.service
sudo systemctl status nav-runtime.service
sudo journalctl -u nav-runtime.service -f
```

切换导航方案需重新安装并重启服务；移动脚本或 OTA 目录后需重新安装。

卸载自启动（保留程序和数据）：

```bash
sudo bash  uninstall_nav_autostart.sh
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

- S01 的 MID360s 2D 停障使用 `obstacle_stop_p2.5_mid360s.yaml` 和 `p2.5_mid360s.yaml`。前方停止/减速厚度从雷达原点起算，左右停止距离从车身侧边起算；后方未启用，左右仅停车。旧区域的 `hard_stop_distance` / 占比阈值不能直接换算为方向栅格阈值，需在新版客户端恢复默认后重新设置。客户端和服务端需同步更新。
