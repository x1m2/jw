# 导航任务状态机使用说明

本文档介绍如何使用Go2机器人的导航任务状态机系统。

## 系统概述

导航任务状态机系统包含以下组件：

1. **导航任务状态机节点** (`navigation_task_state_machine_node`)
2. **GUI控制界面** (`navigation_control_gui.py`)
3. **命令行控制界面** (`navigation_control_cli.py`)
4. **测试工具** (`navigation_task_tester.py`)

## 配置文件

### 任务列表文件 (`task_list.txt`)
```
# 任务列表 (UTF-8编码)
# 格式: 目标点ID 等待延时 抵达后发布的数据
0 0 0
1 0 1
2 12 2
3 10 2
4 5 2
```

### 目标点文件 (`generated_map_target_points.txt`)
```
# 目标点文件 (UTF-8编码)
# 格式: 目标点ID X Y Z QX QY QZ QW 标签
# Target Points
0 8.871257 -0.318777 -0.014845 0.015473 0.017253 -0.693399 0.720181 一号
1 12.135280 0.441980 -0.014845 -0.022662 0.011590 0.636535 0.770828 二号
2 0.384382 0.129524 -0.014845 -0.000211 0.000853 -0.031243 0.999511 三号
```

## ROS话题接口

### 订阅话题
- `/nav_state` (std_msgs/Bool): 导航状态，true表示已到达目标点
- `/nav_task_signal` (std_msgs/Int32): 导航任务信号
  - 1: 开始循环导航
  - 2: 停止任务
  - 3: 继续执行任务
  - 4: 回家

### 发布话题
- `/task_action` (std_msgs/Int32): 任务动作，到达目标点后发布的数据
- `/target_points` (geometry_msgs/PoseStamped): 目标点位姿
- `/emergency_stop` (std_msgs/Bool): 紧急停止信号

## 启动和使用

### 1. 启动导航任务状态机

```bash
# 基本启动
roslaunch user_interface navigation_task_state_machine.launch

# 自定义参数启动
roslaunch user_interface navigation_task_state_machine.launch \
    task_list_file:="/path/to/your/task_list.txt" \
    target_points_file:="/path/to/your/target_points.txt"
```

### 2. 启动GUI控制界面

```bash
# 启动GUI界面（同时启动状态机）
roslaunch user_interface navigation_control_gui.launch

# 或者单独运行GUI脚本
rosrun user_interface navigation_control_gui.py
```

**GUI界面功能：**
- 实时显示当前目标点坐标
- 显示导航状态和紧急停止状态
- 提供控制按钮：
  - 绿色"开始导航"按钮：开始循环导航
  - 红色"紧急停止"按钮：切换紧急停止状态
  - 蓝色"回家"按钮：发送回家信号
  - 橙色"停止任务"按钮：停止当前任务
  - 紫色"继续任务"按钮：继续执行任务
- 操作日志显示

### 3. 启动命令行控制界面

```bash
# 启动命令行界面
rosrun user_interface navigation_control_cli.py
```

**命令行界面命令：**
- `1` 或 `s`: 开始循环导航
- `2` 或 `stop`: 停止任务
- `3` 或 `c`: 继续执行任务
- `4` 或 `h`: 回家
- `e`: 切换紧急停止状态
- `status`: 显示当前状态
- `help`: 显示帮助信息
- `q` 或 `quit`: 退出程序

### 4. 测试工具

```bash
# 启动测试工具
rosrun user_interface navigation_task_tester.py
```

## 工作流程

1. **初始化**：系统启动后处于IDLE状态，加载任务列表和目标点文件
2. **开始导航**：发送信号1，系统进入RUNNING状态
3. **执行任务**：
   - 发布第一个目标点
   - 等待导航完成(`/nav_state`为true)
   - 发布任务动作数据
   - 如果有延时，进入WAITING状态
   - 移动到下一个任务
4. **任务控制**：
   - 信号2：暂停任务进入STOPPED状态
   - 信号3：从STOPPED状态恢复到RUNNING状态
   - 信号4：回家，发布(0,0,0)目标点
5. **完成**：所有任务执行完毕后回到IDLE状态

## 状态机状态

- **IDLE**: 空闲状态，等待启动信号
- **RUNNING**: 运行状态，正在执行导航任务
- **WAITING**: 等待状态，到达目标点后的延时等待
- **STOPPED**: 停止状态，任务被暂停
- **GOING_HOME**: 回家状态，正在返回原点

## 紧急停止功能

紧急停止是独立的安全功能：
- 可在任何时候激活/解除
- 发布到`/emergency_stop`话题
- 不影响状态机的状态，但应该被导航系统监听并立即停止机器人运动

## 故障排除

### 常见问题

1. **配置文件加载失败**
   - 检查文件路径是否正确
   - 确认文件格式是否符合要求
   - 检查文件编码是否为UTF-8

2. **目标点校验失败**
   - 确认任务列表中的目标点ID在目标点文件中都存在
   - 检查目标点文件格式是否正确

3. **GUI界面无法启动**
   - 确认系统已安装tkinter库
   - 使用命令行界面作为替代方案

4. **ROS话题通信问题**
   - 使用`rostopic list`检查话题是否存在
   - 使用`rostopic echo`监听话题消息
   - 检查节点是否正常运行

### 调试命令

```bash
# 检查节点状态
rosnode list | grep navigation

# 监听话题
rostopic echo /nav_state
rostopic echo /target_points
rostopic echo /task_action

# 手动发送信号
rostopic pub /nav_task_signal std_msgs/Int32 "data: 1"
rostopic pub /nav_state std_msgs/Bool "data: true"
```

## 扩展开发

### 添加新的任务信号
1. 在`NavigationTaskSignal`枚举中添加新信号
2. 在`navTaskSignalCallback`函数中添加处理逻辑
3. 在GUI/CLI界面中添加对应的按钮/命令

### 自定义任务行为
1. 修改`processStateMachine`函数中的状态逻辑
2. 添加新的状态枚举（如需要）
3. 更新配置文件格式以支持新的参数

### 集成其他传感器
1. 添加新的订阅器监听传感器数据
2. 在状态机逻辑中集成传感器信息
3. 更新GUI界面显示传感器状态

## 安全注意事项

1. 确保紧急停止功能始终可用
2. 在实际机器人上测试前，先在仿真环境中验证
3. 监控机器人运动，确保路径安全
4. 定期检查目标点坐标的准确性
5. 在人员密集区域使用时要特别小心