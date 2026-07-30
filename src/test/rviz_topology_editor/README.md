# rviz_topology_editor

在PGO建图完成后加载最终PCD，通过RViz内置的 `2D Goal Pose` 打点。
打点完成后，在RViz左侧“拓扑编辑保存”面板点击“保存并生成”，一次生成：

- 原始拓扑点；
- 带四元数的导航目标点；
- 按节点ID顺序自动连边的拓扑地图；
- 导航状态机任务文件。

不需要启动 `topology_tool.launch.py` 或 `topology_connect.launch.py`。
输出格式与 `topology_map_generator` 按K打点保持一致：

- `generated_map.txt`：`id x y z label`
- `generated_map_target_points.txt`：
  `id x y z qx qy qz qw label`
- `connected_topology_map.txt`：节点以及相邻ID之间的双向自动边；
- `task/test.txt`：`目标点ID 等待延时 抵达后发布的数据`

## 编译

```bash
cd /home/jw-zkl/robotics_platform/robot_ws
colcon build --packages-select rviz_topology_editor
source install/setup.bash
```

## 启动

```bash
ros2 launch rviz_topology_editor rviz_topology_editor.launch.py \
  pcd_file:=/home/jw-zkl/robotics_platform/robot_ws/map/globalmap.pcd \
  output_file:=/home/jw-zkl/robotics_platform/robot_ws/topology_map/generated_map.txt \
  connected_topology_file:=/home/jw-zkl/robotics_platform/robot_ws/topology_map/connected_topology_map.txt \
  task_file:=/home/jw-zkl/robotics_platform/robot_ws/task/test.txt
```

启动后在RViz工具栏选择 `2D Goal Pose`：

1. 鼠标左键按下确定位置；
2. 拖动确定朝向；
3. 松开后点进入编辑列表并显示红色ID；
4. 完成全部打点后，在RViz左侧点击“保存并生成”。

保存成功后，RViz会用蓝色线段显示按ID顺序生成的拓扑边。新增、撤销或
清空点后会先隐藏旧边，再次保存后显示最新连边。

编辑话题固定为 `/rviz_topology_editor/goal_pose`，不会直接触发导航。

也可以使用工作空间的一键脚本：

```bash
./src/run_rviz_topology_editor.sh \
  /robotfs/data/slam_data
```

脚本只启动本功能包和RViz，不再启动旧的Topology Tool或Python自动连边节点。

## Z坐标

编辑器用于单层二维导航，不解析或查询高度。RViz每次打点都统一保存
`z=0`；输出文件继续保留z字段，以兼容现有拓扑地图和导航程序的文件格式。

## 撤销与清空

RViz左侧面板提供：

- `撤销最后一点`
- `清空所有点`（操作前弹出确认框）
- `保存并生成`

也可以通过命令行调用：

```bash
ros2 service call /rviz_topology_editor/undo_last std_srvs/srv/Trigger {}
```

```bash
ros2 service call /rviz_topology_editor/clear std_srvs/srv/Trigger {}
```

撤销和清空先修改内存中的编辑列表，需要再次点击“保存并生成”才会覆盖最终文件。
每次启动编辑器都从空列表和ID 0开始，不读取已有目标点继续追加。

默认 `auto_save_raw_points: false`，点击目标只修改内存列表。若改为
`true`，每次点击会额外保存两个原始点文件，但连边拓扑和任务文件仍只在
点击“保存并生成”后更新。
