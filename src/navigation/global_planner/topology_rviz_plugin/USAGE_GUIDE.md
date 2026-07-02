# 拓扑地图RViz2插件使用指南

## 功能概述

这个RViz2插件支持交互式构建拓扑地图，包括：
- 在点云可行域上添加节点
- 手动连接节点形成拓扑网络
- 编辑节点属性
- 删除节点和连接
- 保存/加载拓扑地图

## 安装与启动

### 1. 编译插件
```bash
cd /home/server/WS_ROS1/ws_pcl_map
colcon build --packages-select topology_rviz_plugin
source install/setup.bash
```

### 2. 验证插件安装
```bash
ros2 pkg prefix topology_rviz_plugin
```
应该输出：
```
topology_rviz_plugin /path/to/topology_rviz_plugin/plugin_description.xml
```

## 使用步骤

### 步骤1：启动系统

1. **启动ROS核心**
```bash
roscore
```

2. **发布点云数据**（在新终端）
```bash
cd /home/server/WS_ROS1/ws_pcl_map
source install/setup.bash
ros2 launch pcd_map cloud_publisher.launch.py
```

3. **启动RViz2**（在新终端）
```bash
cd /home/server/WS_ROS1/ws_pcl_map
source install/setup.bash
rviz2 -d install/topology_rviz_plugin/share/topology_rviz_plugin/rviz/topology_demo.rviz
```

### 步骤2：加载插件

1. 在RViz2中，点击工具栏的"+"按钮
2. 在工具列表中找到"Topology Tool"
3. 点击选择该工具

### 步骤3：配置插件

在左侧属性面板中，您可以看到：

- **Mode**: 选择工作模式
  - `Add Node`: 添加节点模式
  - `Connect Nodes`: 连接节点模式
  - `Edit Node`: 编辑节点模式
  - `Delete`: 删除模式

- **Node Label**: 新节点的默认标签
- **Max Connection Distance**: 连接节点时的最大检测距离（米）
- **Selection Threshold**: 点选择精确度阈值（米，范围0.05-1.0）
  - 较小值（如0.1）：需要精确点击，但选择更准确
  - 较大值（如0.5）：容易选择，但精确度降低
- **Bidirectional**: 连接是否为双向
- **Save File**: 保存文件路径

### 步骤4：使用功能

#### 添加节点 (Add Node模式)
1. 选择"Add Node"模式
2. **重要**：确保点云数据已加载（在终端中应看到"接收到点云数据"的消息）
3. 在3D视图中精确点击可行区域点云上的点
4. 系统会自动选择最接近鼠标射线的点云点
5. 节点会显示为蓝色球体，带有标签

**节点添加注意事项**：
- 只能在可行区域点云的具体点上添加节点
- 如果点击位置与点云点距离超过"Selection Threshold"阈值，添加会失败
- 调整"Selection Threshold"参数来控制选择精确度（0.05-1.0米）
- 较小的阈值需要更精确的点击，较大的阈值更容易选择但可能不够精确

#### 连接节点 (Connect Nodes模式)
1. 选择"Connect Nodes"模式
2. 点击第一个节点（起始节点）
3. 点击第二个节点（目标节点）
4. 系统会创建连接，显示为红色线条
5. 右键取消当前连接操作

#### 编辑节点 (Edit Node模式)
1. 选择"Edit Node"模式
2. 点击要编辑的节点
3. 在弹出的对话框中修改节点标签
4. 点击确定保存更改

#### 删除节点/连接 (Delete模式)
1. 选择"Delete"模式
2. 点击要删除的节点
3. 节点和相关连接都会被删除

### 步骤5：保存和加载

#### 保存拓扑地图
1. 在属性面板中设置"Save File"路径
2. 点击"Save"属性切换开关
3. 地图会保存为文本文件

#### 加载拓扑地图
1. 确保"Save File"指向正确的文件
2. 点击"Load"属性切换开关
3. 系统会加载并显示拓扑地图

## 可视化效果

- **节点**: 蓝色球体，带有文字标签
- **手动连接**: 红色线条，带有距离标签
- **自动连接**: 绿色线条（如果有）
- **单向连接**: 带有箭头指示方向
- **双向连接**: 普通线条

## 快捷键

- **T**: 激活拓扑工具
- **左键**: 执行当前模式操作
- **右键**: 取消当前操作

## 文件格式

保存的拓扑地图文件格式：
```
# 拓扑地图文件
# 格式: 节点ID X Y Z 标签
# Topology Nodes
0 1.5 2.0 0.1 Node_0
1 3.0 2.5 0.1 Node_1

# 格式: 边ID 起始节点ID 结束节点ID 权重 类型 双向
# Topology Edges
0 0 1 1.58 manual 1
```

## 故障排除

### 插件无法加载
1. 检查编译是否成功
2. 确保source了正确的setup.bash
3. 验证插件XML文件路径

### 无法在点云上添加节点
1. 确保 `topology_tool.pcd_file` 指向的 PCD 文件存在，默认是 `${ROBOT_DATA_DIR}/map/result_free_space.pcd`
2. 检查 PCD 点云是否为空或坐标系是否与 `map` 一致
3. 调整"Selection Threshold"参数（建议从0.3开始）
4. 确保点击位置在点云可视化区域内
5. 检查点云是否包含有效的3D坐标数据

### 连接无法建立
1. 确保两个节点距离在检测范围内
2. 检查是否已经存在相同连接
3. 尝试调整连接距离参数

## 与现有系统集成

这个插件可以与现有的拓扑构建系统配合使用：

1. 使用插件手动创建关键节点
2. 使用现有的自动拓扑构建算法补充连接
3. 通过插件进行手动调整和优化

## 中文标签支持

插件完全支持中文标签，包括：

### 设置中文标签
1. 在"Node Label"属性中输入中文标签（如"起点"、"终点"）
2. 添加节点时会使用设置的中文标签
3. 如果标签为空，会自动生成"节点_X"格式的标签

### 编辑中文标签
1. 选择"Edit Node"模式
2. 点击要编辑的节点
3. 在弹出的对话框中输入新的中文标签
4. 点击确定保存

### 保存和加载
- 文件使用UTF-8编码保存，完全支持中文字符
- 加载时会正确识别UTF-8编码的中文标签
- 文件开头包含UTF-8 BOM标记确保兼容性

### 注意事项
- 确保系统环境支持中文显示（LANG=zh_CN.UTF-8）
- RViz2界面中的中文显示依赖于系统字体设置
- 建议使用支持中文的等宽字体以获得最佳显示效果

这个插件为拓扑地图的交互式构建提供了强大的工具，让您可以精确控制节点位置和连接关系。
