## SRC目录结构
- **bringup**：     负责启动系统
- **common**：      通用组件
- **depd**：        依赖库
- **drivers**：     驱动节点
- **fast_lio2**：   SLAM建图
- **map**：         负责地图发布与提供地图服务
- **navigation**：  导航
- **system**：      负责状态机管理
- **pgo_sc**： GTSAM 位姿图优化 + ScanContext 回环检测，结合 Quatro 与 Nano-GICP 做回环配准的 SLAM。
- **localization**：基于保存好的地图（.bag）进行定位的系统，使用 ScanContext 进行候选匹配，Quatro 提供全局粗配准初值，Nano-GICP 进行精配准。

两者可单独使用，也可联动：SLAM 节点在线发布 `odom -> base_link` 与矫正点云，定位节点据此估计并发布 `map -> odom`。


## 依赖
- C++ >= 17, OpenMP >= 4.5, CMake >= 3.10, Eigen >= 3.2, Boost >= 1.54
- ROS
- GTSAM >= 4.1.1
- TEASER++
- TBB

安装要点（示例）：
```bash
# GTSAM（SLAM 需要）
wget -O gtsam.zip https://github.com/borglab/gtsam/archive/refs/tags/4.1.1.zip
unzip gtsam.zip && cd gtsam-4.1.1 && mkdir build && cd build
cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON ..
sudo make install -j$(nproc)

# TEASER++（两包共享）
git clone https://github.com/MIT-SPARK/TEASER-plusplus.git
cd TEASER-plusplus && mkdir build && cd build
cmake .. -DENABLE_DIAGNOSTIC_PRINT=OFF
sudo make install -j$(nproc) && sudo ldconfig

# TBB
sudo apt install -y libtbb-dev
```
# sophus
sudo apt install ros-$ROS_DISTRO-sophus

# 将本仓库放入 src 后回到工作空间根目录
cd ~/your_workspace

# ROS2编译依赖模块（建议 Release）
colcon build --packages-select nano_gicp --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon build --packages-select quatro --cmake-args -DCMAKE_BUILD_TYPE=Release -DQUATRO_TBB=ON

# 再编译整个工作空间
colcon build --symlink-install --parallel-workers 2 --cmake-args -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble
source install/setup.bash
```

LIBGL_ALWAYS_SOFTWARE=1 pcl_viewer result.pcd --no-vbo

## 运行方式
### 1) 运行 SLAM 没有后端（FASTLIVO2）
保存地图：
ros2 service call /map_save std_srvs/srv/Trigger
ros2 topic pub --once /save_dir std_msgs/msg/Empty "{}"
- 启动：
```bash
ros2 launch livox_ros_driver2 msg_MID360s_launch.py 雷达节点
ros2 launch fast_livo mapping_mid360s.launch.py SLAM节点
### 2) 运行基于SC-QN后端优化的SLAM

###4) 运控板子 192.168.50.101 感归控192.168.50.103

 ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.2 -p turn:=0.0
