# pure_pursuit_local_planner

ROS 2 Humble 版本的 Pure Pursuit 局部跟踪/避障规划器。

## 编译

```bash
cd ~/robot_ws
colcon build --packages-select pure_pursuit_local_planner --symlink-install
source install/setup.bash
```

## 启动

```bash
ros2 launch pure_pursuit_local_planner pure_pursuit_local_planner.launch.py
```

## 主要修改

- catkin 改为 ament_cmake。
- `roscpp`/`ros::NodeHandle` 改为 `rclcpp::Node`。
- ROS1 消息头与命名空间改为 ROS2：例如 `nav_msgs::msg::Path`。
- `advertise/subscribe/createTimer` 改为 `create_publisher/create_subscription/create_wall_timer`。
- `.h` 头文件改为 `.hpp`。
- ROS1 XML launch 改为 ROS2 Python launch。
