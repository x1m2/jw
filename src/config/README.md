# 机器人机型参数

- `p2.5_mid360s.yaml`：P2.5 MID360s，车宽0.60 m、车长1.10 m，前雷达输入。
- `p3_airy96.yaml`：P3双Airy96，车宽0.60 m、车长1.10 m，前后雷达输入。

外参与尺寸从 dev/airy_test 引入，部署前按实际安装核对；不再使用旧的 robot_param.yaml。
建图、定位、costmap和Pure Pursuit launch默认使用P2.5；可通过robot_param_file指定P3或自定义文件。
停障launch通过robot_model选择匹配算法配置，并支持robot_param_file覆盖公共外参。
在线建图、离线建图和离线重定位脚本按雷达类型选择机型参数。

main仍使用terrain_filter、costmap_2d和Pure Pursuit，Manager导航接口保持2D；
四区域停障直接读取原始雷达点云，与用于二维导航的地形过滤链路并行。
IMU倾斜检测没有有效输入时恢复正常减速，不影响原有硬停保护。
