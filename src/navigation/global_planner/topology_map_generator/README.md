# topology_map_generator

ROS 2 手动拓扑打点工具。

该功能包只保留手动打点逻辑：

1. 订阅路径话题，例如 `/pgo/optimized_path`。
2. 保存路径中的最新位姿。
3. 收到 `/topology_node_set` 的 `std_msgs/Bool` 消息且值为 `true` 时，记录一个拓扑点。
4. 每次打点后立即保存文件，节点退出时也会再次保存。

## 输出文件

默认输出两个文件：

- `generated_map.txt`：拓扑点文件，只包含 `id x y z label`
- `generated_map_target_points.txt`：目标点文件，包含 `id x y z qx qy qz qw label`

## 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `output_file` | `/root/slam_data/topology_map/generated_map.txt` | 拓扑点输出文件 |
| `path_topic` | `/pgo/optimized_path` | 输入路径话题，类型为 `nav_msgs/msg/Path` |
| `trigger_topic` | `/topology_node_set` | 手动打点触发话题，类型为 `std_msgs/msg/Bool` |

已删除自动建边、自动双向边、路径点间隔、路径优化回写等无关逻辑。

## 编译

```bash
cd ~/robot_ws
colcon build --packages-select topology_map_generator
source install/setup.bash
```

## 启动

```bash
ros2 launch topology_map_generator topology_map_generator.launch.py
```

或指定输出文件：

```bash
ros2 launch topology_map_generator topology_map_generator.launch.py \
  config_file:=/path/to/topology_map_generator.yaml
```

## 手动打点

路径话题已经有数据后，发送一次 `true` 即可记录当前最新位姿：

```bash
ros2 topic pub --once /topology_node_set std_msgs/msg/Bool "{data: true}"
```

如果你使用键盘节点发布 `/topology_node_set`，按键触发发布 `true` 即可。
