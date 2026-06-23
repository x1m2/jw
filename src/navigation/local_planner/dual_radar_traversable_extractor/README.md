# dual_radar_traversable_extractor ROS2 Humble 版本

双雷达/单雷达可行区域提取节点，已从 ROS1 catkin 迁移到 ROS2 ament_cmake，并将包内自定义头文件后缀从 `.h` 改为 `.hpp`。

## 编译

```bash
cd ~/robotics_platform/robot_ws
colcon build --packages-select dual_radar_traversable_extractor --symlink-install
source install/setup.bash
```

## 启动

单雷达模式：

```bash
ros2 launch dual_radar_traversable_extractor dual_radar_extractor.launch.py
```

双雷达模式：

```bash
ros2 launch dual_radar_traversable_extractor dual_radar_extractor.launch.py use_dual_radar:=true
```

## 参数文件

参数文件：

```bash
config/dual_radar_config.yaml
```

ROS2 参数格式为：

```yaml
dual_radar_traversable_extractor:
  ros__parameters:
    lidar_topic: "/livox/pointcloud2"
    base_frame: "base_link"
```

## 主要发布话题

- `ground_cloud`
- `obstacle_cloud`
- `combined_cloud`

## 说明

原 ROS1 版本中的 `ros::NodeHandle`、`ros::Subscriber`、`ros::Publisher`、`ROS_INFO`、`roslaunch` 等已替换为 ROS2 写法。PatchWork++ 头文件中的 ROS1 参数读取和时间 API 也已改为 ROS2 参数系统和 `rclcpp::Time`。
