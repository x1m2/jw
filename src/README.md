# robot_ws 源码说明

ROS 2 机器人建图、定位、导航和任务控制工作空间。

## 编译方式

先按板子构建一次 BBS 库和地图转换工具：

```bash
# CPU 板子
./src/depd/3d_bbs/build_cpu.sh
# OpenCL 板子改用：./src/depd/3d_bbs/build_opencl.sh
```

默认安装到 `install/bbs3d`。后续编译定位包只链接这份库；CPU 板子须在
`colcon build` 的 `--cmake-args` 中加入 `-DENABLE_OPENCL_BBS=OFF`。
全工作区构建若包含 `bbs3d`，CPU 板子还需加 `-DBUILD_OPENCL=OFF`。

先单独编译 Quatro：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select quatro \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DQUATRO_TBB=ON
```

再编译其余包：

```bash
source install/setup.bash
colcon build --symlink-install --parallel-workers 3 \
  --packages-skip quatro uwb_follow_planner  uwb_aoa_pkg bbs3d  fastlio_localization pgo_sc fast_lio \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DROS_EDITION=ROS2 \
    -DHUMBLE_ROS=humble

colcon build --symlink-install \
    --parallel-workers 1 \
    --packages-select fast_lio fastlio_localization pgo_sc \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
    
source install/setup.bash
```

source /opt/ros/humble/setup.bash && \
colcon build --symlink-install \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DQUATRO_TBB=ON \
    -DROS_EDITION=ROS2 \
    -DHUMBLE_ROS=humble && \
source install/setup.bash

## 目录结构

```text
src/
├── apps/          # SlamManager、接口和客户端通信
├── config/        # 机器人公共配置
├── depd/          # Nano-GICP、PatchWork++、Quatro 等依赖库
├── drivers/       # Livox 和 UWB 驱动
├── navigation/    # 全局规划、局部规划和自主跟随
├── scripts/       # 辅助脚本
├── slam/          # 建图、回环优化和重定位
├── run_offline_mapping.sh
├── run_offline_reloc.sh
└── run_uwb_follow.sh
```

## PCD

查看 PCD 地图：

```bash
LIBGL_ALWAYS_SOFTWARE=1 pcl_viewer globalmap.pcd --no-vbo
```

## 手动保存地图

```bash
ros2 service call /map_save std_srvs/srv/Trigger
ros2 topic pub --once /save_dir std_msgs/msg/Empty "{}"
```

## 离线建图

离线建图支持两种雷达，默认使用 MID360：

```bash
# MID360：/livox/lidar (CustomMsg) 和 /livox/imu，使用 mapping_mid360s.yaml
src/run_offline_mapping.sh --lidar mid360 --bag /path/to/mid360_bag

# Airy：/rslidar_points 和 /rslidar_imu_data，使用 mapping_airy96.yaml
src/run_offline_mapping.sh --lidar airy --bag /path/to/airy_bag

```

两种模式都支持 `--rate` 设置 bag 回放倍速，例如：

```bash
src/run_offline_mapping.sh --lidar airy --bag /path/to/airy_bag --rate 0.5
```

## 离线重定位

```bash
src/run_offline_reloc.sh \
  --bag /home/jw-zkl/robotics_platform/robot_ws/airy96_fastlio_20250604_222402 \
  --rate 1.0 \
  --scan-index 250
```

离线重定位支持两种雷达，默认使用 Airy：

```bash
# MID360：/livox/lidar (CustomMsg) 和 /livox/imu，使用 reloc_mid360s.yaml
src/run_offline_reloc.sh --lidar mid360 --bag /path/to/mid360_bag

# Airy：/rslidar_points 和 /rslidar_imu_data
src/run_offline_reloc.sh --lidar airy --bag /path/to/airy_bag
```
