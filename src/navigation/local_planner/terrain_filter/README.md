# terrain_filter

单/双雷达地形过滤节点，分割地面与障碍物点云。

## 编译

```bash
cd ~/robotics_platform/robot_ws
colcon build --packages-select terrain_filter --symlink-install
source install/setup.bash
```

## 启动

单雷达模式：

```bash
ros2 launch terrain_filter terrain_filter.launch.py
```

双雷达模式：

```bash
ros2 launch terrain_filter terrain_filter.launch.py use_dual_radar:=true
```

## 参数文件

参数文件：

```bash
config/terrain_filter.yaml
```

ROS2 参数格式为：

```yaml
terrain_filter_node:
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
