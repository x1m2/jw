"""SCAN 统一启动入口：规划与开环/闭环控制在 scan_planner_node 内部运行。

普通导航示例：
    ros2 launch scan_planner run.launch.py is_real_world:=true navi_mode:=3

接口约定：定位消息描述 map -> base_link；/cloud_registered 点云位于 base_link。
Odometry 是位姿消息类型，不代表必须存在名为 odom 的坐标系。
机器人尺寸和 body_height 在 planner.yaml 中设置；整机顶部高度不等于机身参考点高度。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    # launch 参数以字符串传入，将常见的真值写法转换为 Python 布尔值。
    return value.lower() in ("1", "true", "yes", "on")


def _setup(context):
    # 在启动上下文就绪后解析参数，支持命令行覆盖 YAML 和默认话题。
    scan_share = get_package_share_directory("scan_planner")
    planner_yaml = os.path.join(scan_share, "config", "planner.yaml")
    controllers_yaml = os.path.join(scan_share, "config", "controllers.yaml")
    is_real = _as_bool(LaunchConfiguration("is_real_world").perform(context))
    use_sim_time = _as_bool(LaunchConfiguration("use_sim_time").perform(context))
    sensor_type = LaunchConfiguration("sensor_type").perform(context)
    controller_mode = LaunchConfiguration("controller_mode").perform(context)
    keypoints_file = LaunchConfiguration("keypoints_file").perform(context)
    reference_path_file = LaunchConfiguration("reference_path_file").perform(context)
    navi_mode = int(LaunchConfiguration("navi_mode").perform(context))
    runtime_profile = LaunchConfiguration("runtime_profile").perform(context)
    global_frame = LaunchConfiguration("global_frame").perform(context)
    # 检查运行组合：平台导航需要闭环控制，开环仅用于播放模拟位姿。
    # 模式2必须加载关键点参数文件；模式3才允许启动参考路径演示发布器。
    if runtime_profile not in ("test", "platform"):
        raise RuntimeError("runtime_profile must be 'test' or 'platform'")
    if runtime_profile == "platform" and controller_mode != "closed_loop":
        raise RuntimeError("platform navigation requires the closed_loop controller")
    if sensor_type not in ("lidar", "depth"):
        raise RuntimeError("sensor_type must be 'lidar' or 'depth'")
    if controller_mode not in ("open_loop", "closed_loop"):
        raise RuntimeError("controller_mode must be 'open_loop' or 'closed_loop'")
    if navi_mode not in (1, 2, 3):
        raise RuntimeError("navi_mode must be 1, 2, or 3")
    if navi_mode == 2 and (not keypoints_file or not os.path.isfile(keypoints_file)):
        raise RuntimeError(
            "navi_mode=2 requires keypoints_file to reference a ROS 2 parameter YAML"
        )
    if reference_path_file and navi_mode != 3:
        raise RuntimeError("reference_path_file is only valid when navi_mode=3")
    if reference_path_file and not os.path.isfile(reference_path_file):
        raise RuntimeError(
            "reference_path_file must reference an existing ROS 2 parameter YAML"
        )

    # 以下位置只给开环控制器生成初始模拟里程计，不修改真实机器人的定位或 TF。
    # init_z 是模拟位姿的 Z，不是整机顶部高度，也不是 body_height。
    mode_default_init = (-19.0, 1.0, 0.25) if navi_mode == 1 else (-5.5, 5.5, 0.5)
    initial_position = []
    for name, default in zip(("init_x", "init_y", "init_z"), mode_default_init):
        value = LaunchConfiguration(name).perform(context)
        initial_position.append(default if value == "" else float(value))
    init_x, init_y, init_z = initial_position

    if is_real:
        # 硬件默认输入。/lidar_localization 必须实际发布 nav_msgs/Odometry；
        # 仅存在 map -> base_link TF 时，当前规划器仍会等待消息并打印 no odom.
        body_pose = "/lidar_localization"
        # 激光点云已在 base_link 中，使用同一车体位姿将其变换到 map。
        # 深度相机使用独立位姿输入，其坐标系和内参需与实际相机匹配。
        sensor_pose = body_pose if sensor_type == "lidar" else "/imu_localization"
        cloud = "/cloud_registered"
        depth = "/camera/aligned_depth_to_color/image_raw"
        cloud_is_world = False  # 点坐标需要通过 sensor_pose 旋转、平移到全局系。
        need_extrinsic = False  # 不再应用内置传感器外参，防止重复转换。
        # 仅供深度模式投影使用；激光模式不使用这些相机内参。
        intrinsics = {
            "grid_map.cx": 317.19183349609375,
            "grid_map.cy": 256.4806823730469,
            "grid_map.fx": 609.5884399414062,
            "grid_map.fy": 609.22021484375,
        }
    else:
        # 外部仿真输入：此 launch 不启动仿真器，默认点云已处于全局坐标系。
        body_pose = "/quad_0/body_pose"
        sensor_pose = "/quad_0/camera_pose" if sensor_type == "depth" else "/quad_0/lidar_pose"
        cloud = "/quad_0/cloud"
        depth = "/quad_0/depth"
        cloud_is_world = True
        need_extrinsic = False
        intrinsics = {}

    # 非空话题参数优先；硬件激光模式下，改 body_pose 后 sensor_pose 默认同步。
    # 若单独指定 sensor_pose_topic，则以该参数为准。
    body_pose = LaunchConfiguration("body_pose_topic").perform(context) or body_pose
    if is_real and sensor_type == "lidar":
        sensor_pose = body_pose
    sensor_pose = LaunchConfiguration("sensor_pose_topic").perform(context) or sensor_pose
    cloud = LaunchConfiguration("cloud_topic").perform(context) or cloud
    # 空字符串沿用硬件/仿真默认值；显式 true/false 则覆盖点云转换方式。
    # 必须依据点坐标实际所属坐标系选择，不能只修改消息 frame_id。
    cloud_mode = LaunchConfiguration("cloud_is_world").perform(context)
    if cloud_mode:
        cloud_is_world = _as_bool(cloud_mode)
    need_extrinsic = _as_bool(LaunchConfiguration("need_extrinsic").perform(context))
    path_topic = LaunchConfiguration("path_topic").perform(context)
    # test 自动创建算法；platform 仅启动 Worker，等待 SlamManager 的 START 命令。
    common = {"use_sim_time": use_sim_time, "runtime_profile": runtime_profile}
    planner_overrides = {
        **common,
        **intrinsics,
        "fsm.navi_mode": navi_mode,
        "grid_map.sensor_type": sensor_type,
        "grid_map.cloud_is_world": cloud_is_world,
        "grid_map.need_extrinsic": need_extrinsic,
        "grid_map.frame_id": global_frame,  # 地图和定位使用的统一全局坐标系。
        "grid_map.cloud_frame": LaunchConfiguration("cloud_frame").perform(context),
        # 空字符串关闭额外 TF 发布，不会关闭滑动地图算法。
        "grid_map.sliding_map_frame_id": LaunchConfiguration("sliding_map_frame").perform(context),
        "controller_mode": controller_mode,
        "open_loop.frame_id": global_frame,
        "open_loop.init_x": init_x,
        "open_loop.init_y": init_y,
        "open_loop.init_z": init_z,
    }
    actions = [
        Node(
            package="scan_planner",
            executable="scan_planner_node",
            output="screen",
            # 后面的参数覆盖前面的同名值：规划 YAML -> 控制 YAML -> 关键点 -> launch。
            parameters=[planner_yaml, controllers_yaml] + ([keypoints_file] if keypoints_file else []) + [planner_overrides],
            remappings=[
                # body_pose 同时供规划状态机和闭环控制使用；sensor_pose 供点云建图使用。
                ("body_pose", body_pose),
                ("sensor_pose", sensor_pose),
                ("cloud", cloud),
                ("depth", depth),
                ("move_base_simple/goal", "/move_base_simple/goal"),  # 模式1的交互目标。
                ("initial_path", path_topic),  # 模式3的外部参考路径，包含 XYZ。
                ("cmd_vel", "/cmd_vel_planned"),  # 交给 obstacle_stop 检查后再输出 /cmd_vel。
            ],
        )
    ]
    # 真实机器人只使用外部 map -> base_link TF；Go2 模型仅在显式启用时发布。
    if _as_bool(LaunchConfiguration("publish_robot_model").perform(context)):
        go2_share = get_package_share_directory("go2_description")
        actions.append(Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="go2_robot_state_publisher",
            output="screen",
            parameters=[
                common,
                {
                    "robot_description": Command(
                        ["xacro ", os.path.join(go2_share, "xacro", "robot.xacro"),
                         " use_gazebo:=false"]
                    )
                },
            ],
        ))

    # 可选的演示路径发布器：有文件时才启动；正式导航由全局规划器发布路径。
    # 它与外部全局规划器不能同时向同一路径话题发送相互冲突的目标。
    if reference_path_file:
        actions.append(
            Node(
                package="scan_planner",
                executable="reference_path_publisher.py",
                name="reference_path_publisher",
                output="screen",
                parameters=[reference_path_file, common],
                remappings=[
                    ("body_pose", body_pose),
                    ("initial_path", path_topic),
                ],
            )
        )


    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            # true 使用硬件话题；false 使用外部仿真话题，不会在此启动定位或雷达驱动。
            DeclareLaunchArgument("is_real_world", default_value="false"),
            # 1：交互目标；2：YAML关键点序列；3：外部全局路径（run_nav.sh使用此模式）。
            DeclareLaunchArgument("navi_mode", default_value="1"),
            # lidar：PointCloud2；depth：深度图及相机位姿。
            DeclareLaunchArgument("sensor_type", default_value="lidar"),
            # closed_loop 输出速度；open_loop 直接发布模拟 Odometry，不控制真实底盘。
            DeclareLaunchArgument("controller_mode", default_value="closed_loop"),
            # 模式2所需的ROS参数YAML；格式参考config/keypoints.example.yaml。
            DeclareLaunchArgument("keypoints_file", default_value=""),
            # 模式3可选的演示路径YAML；为空时只接收外部路径，不启动演示发布器。
            DeclareLaunchArgument("reference_path_file", default_value=""),
            # 开环模拟初始位置（m）；留空使用上方按导航模式选择的默认位置。
            DeclareLaunchArgument("init_x", default_value=""),
            DeclareLaunchArgument("init_y", default_value=""),
            DeclareLaunchArgument("init_z", default_value=""),
            # true 使用 /clock；真实机器人通常为 false。
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            # test 自动运行；platform 等待 /scan_planner_worker/set_state 启动算法。
            DeclareLaunchArgument("runtime_profile", default_value="test"),
            # 普通定位导航为map；独立FAST-LIO UWB跟随链路可显式设为odom。
            DeclareLaunchArgument("global_frame", default_value="map"),
            # 位姿消息话题，类型为nav_msgs/Odometry，不是TF名称；空值沿用上方默认输入。
            DeclareLaunchArgument("body_pose_topic", default_value=""),
            # 点云坐标系在global_frame下的位姿；硬件激光模式默认与body_pose相同。
            DeclareLaunchArgument("sensor_pose_topic", default_value=""),
            # PointCloud2话题；当前硬件约定 /cloud_registered 的frame_id为base_link。
            DeclareLaunchArgument("cloud_topic", default_value=""),
            # false：局部点云需变换；true：点坐标已在global_frame中，直接建图。
            DeclareLaunchArgument("cloud_is_world", default_value="",
                                  description="Empty: base-frame clouds on hardware, global-frame clouds in simulation"),
            # cloud_is_world=false时，输入点云应使用的局部坐标系。
            DeclareLaunchArgument("cloud_frame", default_value="base_link"),
            # 是否额外使用内置传感器外参；点云已为base_link时保持false。
            DeclareLaunchArgument("need_extrinsic", default_value="false"),
            # 仅控制滑动地图TF名称，默认不增加map/base_link以外的坐标系。
            DeclareLaunchArgument("sliding_map_frame", default_value=""),
            # Go2调试模型开关；开启会发布额外关节TF，P2实机通常保持false。
            DeclareLaunchArgument("publish_robot_model", default_value="false"),
            # 模式3接收nav_msgs/Path的话题；路径高度来自上游，不会自动补出楼梯高度。
            DeclareLaunchArgument("path_topic", default_value="/smoothed_global_path"),
            # 参数解析完成后执行_setup，按选定模式创建实际启动动作。
            OpaqueFunction(function=_setup),
        ]
    )
