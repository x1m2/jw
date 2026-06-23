# 全局路径规划器 - 临时节点合法性检查功能

## 功能概述

本更新为全局路径规划器添加了参数控制的临时节点合法性检查功能，允许用户根据不同场景的需求灵活配置路径规划的安全性和灵活性。

## 核心功能

### 临时连接可行域检查控制
新增参数 `enable_temp_connection_feasibility_check` 用于控制是否对临时节点连接进行严格的可行域检查。

```yaml
enable_temp_connection_feasibility_check: false  # 默认值：false
```

## 功能说明

### 参数设置为 `true` 时：
- **优点**：
  - 对所有临时连接进行严格的边合法性检查
  - 使用可行域点云数据验证路径安全性
  - 确保规划的路径在实际环境中可通行
  - 提高路径规划的安全性和可靠性

- **缺点**：
  - 可能过于保守，在某些可通行区域也被判定为不可行
  - 依赖高质量的可行域点云数据
  - 计算开销相对较大

### 参数设置为 `false` 时：
- **优点**：
  - 路径规划更加灵活和高效
  - 不依赖完美的可行域点云数据
  - 基于距离和基本几何检查进行连接
  - 计算速度更快

- **缺点**：
  - 可能在复杂环境中规划出不安全的路径
  - 需要依赖下游的局部规划器进行安全性保障

## 应用场景建议

### 建议启用检查 (`true`) 的场景：
1. **复杂室内环境**：家具密集、障碍物多的环境
2. **安全要求高**：载人或贵重物品运输
3. **可行域数据质量高**：有高精度的环境点云数据
4. **路径规划失败率可接受**：可以容忍偶尔的规划失败

### 建议禁用检查 (`false`) 的场景：
1. **开放环境**：室外空旷场地、大型仓库
2. **实时性要求高**：需要快速响应的导航场景
3. **可行域数据质量差**：环境点云数据不完整或有误
4. **依赖局部规划器**：有可靠的局部避障系统

## 配置示例

### 保守安全配置（适用于复杂环境）
```yaml
enable_temp_connection_feasibility_check: true
valid_point_threshold: 0.95
safety_sphere_radius: 0.4
edge_sample_count: 150
```

### 灵活高效配置（适用于开放环境）
```yaml
enable_temp_connection_feasibility_check: false
direct_connection_threshold: 5.0
connection_distance_threshold: 8.0
```

## 使用方法

1. **配置参数文件**：
   编辑 `config/global_path_planner_params.yaml`，设置合适的参数值。

2. **启动节点**：
   ```bash
   roslaunch global_path_planner global_path_planner_with_config.launch
   ```

3. **运行时调整**：
   ```bash
   rosparam set /global_path_planner/enable_temp_connection_feasibility_check true
   ```

## 调试和监控

### 相关日志消息：
- `"临时连接可行域检查: 启用/禁用"` - 显示当前检查状态
- `"找到第一个合法节点"` - 显示临时节点连接状态
- `"创建合法连接"` - 显示成功创建的连接
- `"候选节点 X 连接非法，跳过"` - 显示被过滤的连接

### 性能监控：
```bash
# 查看路径规划成功率
rostopic echo /global_path

# 查看规划时间
rosrun rqt_console rqt_console
```

## 故障排除

### 常见问题：

1. **路径规划频繁失败**：
   - 尝试设置 `enable_temp_connection_feasibility_check: false`
   - 降低 `valid_point_threshold` 值
   - 检查可行域点云数据质量

2. **规划的路径不安全**：
   - 设置 `enable_temp_connection_feasibility_check: true`
   - 增加 `safety_sphere_radius` 值
   - 提高 `valid_point_threshold` 值

3. **规划速度过慢**：
   - 设置 `enable_temp_connection_feasibility_check: false`
   - 减少 `edge_sample_count` 值
   - 降低 `interpolation_points` 值

## 技术实现细节

- 在 `addTemporaryConnections` 方法中添加了参数控制逻辑
- 对直接连接也添加了相同的参数控制
- 保持了与原有功能的完全兼容性
- 默认设置确保向后兼容