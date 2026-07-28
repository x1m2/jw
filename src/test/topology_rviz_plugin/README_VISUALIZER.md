# 导航目标点可视化节点使用说明

## 概述

`target_points_visualizer_node` 是一个ROS节点，用于在RViz2中可视化导航目标点。它读取目标点配置文件，并在RViz2中显示带有朝向的箭头和标签。

## 功能特性

- **箭头显示**：在每个目标点位置显示箭头，箭头朝向与四元数姿态一致
- **文本标签**：在箭头上方显示目标点的标签和ID
- **颜色区分**：不同目标点使用不同颜色的箭头
- **实时更新**：支持动态更新目标点显示
- **参数配置**：支持自定义箭头大小、文本大小等参数

## 文件结构

```
topology_rviz_plugin/
├── src/
│   └── target_points_visualizer_node.cpp    # 主节点源码
├── launch/
│   └── target_points_visualizer.launch      # 启动文件
├── rviz/
│   └── target_points_visualization.rviz     # RViz2配置文件
└── map/
    └── generated_map_target_points.txt       # 目标点数据文件
```

## 目标点文件格式

目标点文件 `generated_map_target_points.txt` 的格式如下：

```
# 目标点文件 (UTF-8编码)
# 格式: 目标点ID X Y Z QX QY QZ QW 标签
# Target Points
0 8.871257 -0.318777 -0.014845 0.015473 0.017253 -0.693399 0.720181 一号
1 12.135280 0.441980 -0.014845 -0.022662 0.011590 0.636535 0.770828 二号
2 0.384382 0.129524 -0.014845 -0.000211 0.000853 -0.031243 0.999511 三号
```

**字段说明：**
- `目标点ID`: 整数，唯一标识符
- `X Y Z`: 目标点的3D位置坐标（米）
- `QX QY QZ QW`: 四元数表示的姿态
- `标签`: 目标点的文本标签（可包含中文）

## 启动方式

### 1. 基本启动

```bash
# 只启动可视化节点
ros2 launch topology_rviz_plugin target_points_visualizer.launch.py rviz:=false
```

### 2. 启动节点和RViz2

```bash
# 同时启动可视化节点和RViz2
ros2 launch topology_rviz_plugin target_points_visualizer.launch.py
```

### 3. 自定义参数启动

```bash
# 使用自定义目标点文件
ros2 launch topology_rviz_plugin target_points_visualizer.launch.py \
    target_points_file:="/path/to/your/target_points.txt" \
    arrow_scale:=2.0 \
    text_scale:=1.0
```

## 参数配置

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `target_points_file` | string | 包内map文件夹路径 | 目标点文件完整路径 |
| `frame_id` | string | "map" | 参考坐标系 |
| `arrow_scale` | double | 1.5 | 箭头缩放比例 |
| `text_scale` | double | 0.8 | 文本缩放比例 |
| `text_height_offset` | double | 1.2 | 文本相对箭头的高度偏移（米）|
| `publish_rate` | double | 1.0 | 标记发布频率（Hz）|

## ROS话题

### 发布的话题

- `/target_points_markers` (visualization_msgs/MarkerArray)
  - 包含箭头和文本标记的数组
  - 命名空间：
    - `target_arrows`: 箭头标记
    - `target_labels`: 文本标记

## RViz2配置

1. **添加MarkerArray显示**：
   - 在RViz2中添加 "MarkerArray" 显示类型
   - 设置话题为 `/target_points_markers`
   - 确保 `target_arrows` 和 `target_labels` 命名空间都已启用

2. **设置参考坐标系**：
   - 将Global Options中的Fixed Frame设置为 "map"

3. **调整视角**：
   - 使用提供的rviz配置文件可获得最佳视角

## 颜色编码

节点自动为不同ID的目标点分配不同颜色：
- 使用HSV颜色空间
- 每个ID间隔60度色调
- 饱和度和亮度保持最大值
- 文本标签统一使用白色

## 故障排除

### 常见问题

1. **目标点不显示**
   - 检查目标点文件路径是否正确
   - 确认文件格式符合要求
   - 检查RViz2中Fixed Frame设置

2. **文件读取失败**
   - 确认文件存在且有读取权限
   - 检查文件编码是否为UTF-8
   - 查看节点日志获取详细错误信息

3. **箭头朝向错误**
   - 检查四元数数值是否正确
   - 确认四元数已归一化
   - 验证坐标系定义

### 调试命令

```bash
# 检查节点状态
rosnode list | grep visualizer

# 查看发布的话题
rostopic list | grep target_points

# 监听标记消息
rostopic echo /target_points_markers

# 查看节点日志
rosnode info target_points_visualizer
```

## 扩展开发

### 添加新的可视化元素

1. 在 `createVisualizationMarkers()` 函数中添加新的标记类型
2. 实现对应的创建函数（如 `createSphereMarker()`）
3. 添加相应的参数配置

### 支持动态目标点更新

1. 添加文件监控功能
2. 实现文件变化检测
3. 动态重新加载和更新标记

### 与导航系统集成

1. 订阅当前目标点话题
2. 高亮显示当前活动目标点
3. 显示导航路径和状态

## 性能优化

- 使用 `latched` 发布器减少重复发布
- 仅在目标点变化时更新标记
- 优化标记数量和复杂度
- 合理设置发布频率

## 示例用法

```bash
# 启动roscore
roscore

# 启动目标点可视化
ros2 launch topology_rviz_plugin target_points_visualizer.launch.py

# 在另一个终端中检查话题
rostopic echo /target_points_markers
```

这个可视化节点可以与导航系统配合使用，为操作员提供直观的目标点显示。