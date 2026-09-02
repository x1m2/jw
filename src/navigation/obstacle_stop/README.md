# obstacle_stop

独立的MID360停障与最终速度保护节点。节点直接订阅`/livox/lidar`，将有效
三维点转换到`base_link`后直接构建障碍栅格；它接收Pure Pursuit输出的
`/cmd_vel_planned`，经过障碍、急停和数据超时检查后发布最终`/cmd_vel`。

## 接口

订阅：

```text
/livox/lidar       livox_ros_driver2/msg/CustomMsg
/cmd_vel_planned   geometry_msgs/msg/Twist
/emergency_stop    std_msgs/msg/Bool
/obstacle_stop/enabled std_msgs/msg/Bool
```

发布：

```text
/cmd_vel                         geometry_msgs/msg/Twist
/obstacle_stop/active            std_msgs/msg/Bool
/obstacle_stop/detection_marker  visualization_msgs/msg/Marker
```

点云或计划速度超时、原始点云为空以及外部急停时，节点默认发布零速度。
高度、距离和角度过滤后没有障碍点时，按检测区域无障碍处理。

`/obstacle_stop/enabled=false`时透传规划速度并跳过雷达停障；外部急停和
规划速度超时保护仍然有效。默认值为`true`。

前方采用两个连续矩形检测区：远矩形触发时线速度降为40%，进入近矩形
硬停区后强制发布零速度。障碍出现时立即触发，障碍消失后延迟解除；硬停
解除后先经过减速状态，再恢复规划速度。

两个矩形各自划分为左、中央和右三个区域。只有障碍完全位于一侧，并且
规划指令在对应检测区产生了明确朝相反侧的横向速度时，才临时释放该侧的
停障。中央障碍、左右同时受阻或转向速度不足仍按原规则减速/停车；外部
急停、指令超时和雷达异常不受方向释放影响。

机器人尺寸和雷达安装外参统一配置在 `src/config/robot_param.yaml`。

## 地图区域配置

平台导航启动时，Worker会把当前运行时`topology_map.json`路径传给本节点。
节点订阅`/pose_stamped_localization`，根据`regions[].polygon`判断机器人
实际所在区域：

- 节点根据区域ID读取运行时拓扑同级目录下的
  `region_obstacle_stop/<region_id>.yaml`，覆盖默认配置中的五个停障参数并
  重新计算停止区和减速区；
- `enabled=false`的区域不会加载或校验对应ID的YAML；
- 区域YAML不存在时继续使用启动时`obstacle_stop.yaml`参数；
- 区域YAML存在但不可读、包含未知参数或参数越界时，Worker启动失败并把原因
  返回给`slam_manager`，导航不会开始；
- 离开所有区域后恢复启动时默认配置。

区域YAML由本节点直接读取和校验，`slam_manager`不解析其中的停障参数。
配置切换只影响停止距离、减速距离、占用阈值及线/角速度倍率，不改变点云
话题、坐标系、超时和控制频率。
