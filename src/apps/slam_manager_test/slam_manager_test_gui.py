#!/usr/bin/env python3
"""slam_manager APP-facing integration test GUI."""

import datetime
import json
import math
import os
import queue
import re
import shutil
import subprocess
import threading
import time
import tkinter as tk
from tkinter import messagebox, simpledialog, ttk

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from slam_manager_interfaces.action import NavigateToPose, StartNavigation
from slam_manager_interfaces.msg import (
    MapKeyframe,
    NavigationNode,
    RobotPose,
)
from slam_manager_interfaces.msg import RuntimeState, WorkerStatus
from slam_manager_interfaces.srv import DeleteMap, LoadMap, StartLocalization, StartMapping


RUNTIME_STATE_NAMES = {
    RuntimeState.IDLE: 'IDLE',
    RuntimeState.LOADING_MAP: 'LOADING_MAP',
    RuntimeState.MAPPING: 'MAPPING',
    RuntimeState.SAVING_MAP: 'SAVING_MAP',
    RuntimeState.RELOCALIZING: 'RELOCALIZING',
    RuntimeState.NAVIGATING: 'NAVIGATING',
    RuntimeState.NAVIGATION_SUCCEEDED: 'NAVIGATION_SUCCEEDED',
    RuntimeState.NAVIGATION_PAUSED: 'NAVIGATION_PAUSED',
    RuntimeState.ERROR: 'ERROR',
}

WORKER_STATE_NAMES = {
    WorkerStatus.STOPPED: 'STOPPED',
    WorkerStatus.STARTING: 'STARTING',
    WorkerStatus.RUNNING: 'RUNNING',
    WorkerStatus.STOPPING: 'STOPPING',
    WorkerStatus.ERROR: 'ERROR',
}

EXPECTED_WORKERS = (
    'pgo_worker',
    'fast_lio_worker',
    'fastlio_localization_worker',
    'obstacle_stop_worker',
    'costmap_worker',
    'global_path_planner_worker',
    'path_smoother_worker',
    'pure_pursuit_worker',
)


def find_workspace_dir(start_path):
    """Find the workspace containing the RViz topology editor launcher."""
    current = os.path.abspath(start_path)
    while True:
        launcher = os.path.join(current, 'src', 'run_rviz_topology_editor.sh')
        if os.path.isfile(launcher):
            return current
        parent = os.path.dirname(current)
        if parent == current:
            raise RuntimeError(
                'cannot find workspace containing src/run_rviz_topology_editor.sh'
            )
        current = parent


class RuntimeTestNode(Node):
    def __init__(self, events):
        super().__init__('slam_manager_test_gui')
        self.events = events
        self.worker_states = {}
        self.runtime_state = None
        self.pose_count = 0
        self.last_pose_received_at = 0.0
        self.cloud_count = 0
        self.keyframe_count = 0

        # rclpy.node.Node.clients是只读属性，自定义字典不能使用同名属性。
        self.service_clients = {
            'start_mapping': self.create_client(StartMapping, '/robot/start_mapping'),
            'finish_mapping': self.create_client(Trigger, '/robot/finish_mapping'),
            'load_map': self.create_client(LoadMap, '/robot/load_map'),
            'delete_map': self.create_client(DeleteMap, '/robot/delete_map'),
            'start_localization': self.create_client(
                StartLocalization, '/robot/start_localization'
            ),
            'pause_navigation': self.create_client(
                Trigger, '/robot/pause_navigation'
            ),
            'resume_navigation': self.create_client(
                Trigger, '/robot/resume_navigation'
            ),
        }
        self.patrol_action = ActionClient(
            self, StartNavigation, '/robot/start_navigation'
        )
        self.pose_action = ActionClient(
            self, NavigateToPose, '/robot/navigate_to_pose'
        )
        marker_qos = QoSProfile(depth=1)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.task_marker_pub = self.create_publisher(
            MarkerArray, '/slam_manager_test/task_markers', marker_qos
        )
        self.last_task_markers = None
        self.task_marker_timer = self.create_timer(
            1.0, self._republish_task_markers
        )
        self.active_goal_handle = None

        self.create_subscription(
            RuntimeState, '/robot/runtime_state', self._runtime_state_cb, 10
        )
        self.create_subscription(
            WorkerStatus, '/slam_manager/worker_status', self._worker_status_cb, 50
        )
        self.create_subscription(RobotPose, '/robot/current_pose', self._pose_cb, 20)
        self.create_subscription(
            PointCloud2,
            '/robot/cloud_body',
            self._cloud_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Bool, '/reloc_success', self._relocalization_result_cb, 10
        )
        self.create_subscription(
            PoseStamped, '/goal_pose', self._rviz_goal_pose_cb, 10
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self._rviz_initial_pose_cb,
            10,
        )
        self.create_subscription(
            MapKeyframe, '/robot/keyframe_cloud', self._keyframe_cb, 100
        )

    def _runtime_state_cb(self, message):
        self.runtime_state = message
        self.events.put(('runtime', message))

    def _worker_status_cb(self, message):
        self.worker_states[message.worker_name] = message
        self.events.put(('worker', message))

    def _pose_cb(self, message):
        self.pose_count += 1
        self.last_pose_received_at = time.monotonic()
        self.events.put(('pose', (message, self.pose_count)))

    def _cloud_cb(self, _message):
        self.cloud_count += 1
        if self.cloud_count % 10 == 1:
            self.events.put(('counts', None))

    def _relocalization_result_cb(self, message):
        self.events.put(('relocalization_feedback', bool(message.data)))

    def _rviz_goal_pose_cb(self, message):
        """Forward an RViz 2D Goal Pose to the Tk thread for form filling."""
        frame_id = message.header.frame_id.strip()
        if frame_id and frame_id != 'map':
            self.events.put((
                'log',
                f'[REJECT] RViz目标坐标系为 {frame_id}，单点导航要求 map',
            ))
            return

        orientation = message.pose.orientation
        yaw = math.atan2(
            2.0 * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0 - 2.0 * (
                orientation.y * orientation.y
                + orientation.z * orientation.z
            ),
        )
        position = message.pose.position
        self.events.put((
            'rviz_goal_pose',
            (position.x, position.y, position.z, yaw),
        ))

    def _rviz_initial_pose_cb(self, message):
        """Forward an RViz 2D Pose Estimate to the Tk thread for form filling."""
        frame_id = message.header.frame_id.strip()
        if frame_id and frame_id != 'map':
            self.events.put((
                'log',
                f'[REJECT] RViz初始位姿坐标系为 {frame_id}，重定位要求 map',
            ))
            return

        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0 - 2.0 * (
                orientation.y * orientation.y
                + orientation.z * orientation.z
            ),
        )
        position = message.pose.pose.position
        self.events.put((
            'rviz_initial_pose',
            (position.x, position.y, position.z, yaw),
        ))

    def _keyframe_cb(self, message):
        self.keyframe_count += 1
        self.events.put(('keyframe', (message, self.keyframe_count)))

    def publish_task_markers(self, task_id, nodes):
        """Publish the exact patrol node range selected in the GUI."""
        now = self.get_clock().now().to_msg()
        clear_array = MarkerArray()
        clear = Marker()
        clear.header.frame_id = 'map'
        clear.header.stamp = now
        clear.action = Marker.DELETEALL
        clear_array.markers.append(clear)
        self.task_marker_pub.publish(clear_array)

        marker_array = MarkerArray()

        route = Marker()
        route.header.frame_id = 'map'
        route.header.stamp = now
        route.ns = f'task_{task_id}_route'
        route.id = 0
        route.type = Marker.LINE_STRIP
        route.action = Marker.ADD
        route.pose.orientation.w = 1.0
        route.scale.x = 0.16
        route.color.r = 1.0
        route.color.g = 0.65
        route.color.a = 0.9

        last_index = len(nodes) - 1
        for index, node in enumerate(nodes):
            x, y, z, theta = (
                node['x'], node['y'], node['z'], node['theta']
            )
            point = Point()
            point.x, point.y, point.z = x, y, z + 0.3
            route.points.append(point)

            arrow = Marker()
            arrow.header.frame_id = 'map'
            arrow.header.stamp = now
            arrow.ns = f'task_{task_id}_nodes'
            arrow.id = index * 2 + 1
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.position.x = x
            arrow.pose.position.y = y
            arrow.pose.position.z = z + 0.3
            arrow.pose.orientation.z = math.sin(theta * 0.5)
            arrow.pose.orientation.w = math.cos(theta * 0.5)
            arrow.scale.x = 1.2
            arrow.scale.y = 0.24
            arrow.scale.z = 0.24
            if index == 0:
                arrow.color.g = 1.0
            elif index == last_index:
                arrow.color.r = 1.0
            else:
                arrow.color.g = 0.45
                arrow.color.b = 1.0
            arrow.color.a = 1.0
            marker_array.markers.append(arrow)

            label = Marker()
            label.header.frame_id = 'map'
            label.header.stamp = now
            label.ns = f'task_{task_id}_labels'
            label.id = index * 2 + 2
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = z + 1.0
            label.pose.orientation.w = 1.0
            label.scale.z = 0.7
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = str(node['id'])
            marker_array.markers.append(label)

        marker_array.markers.append(route)
        self.last_task_markers = marker_array
        self.task_marker_pub.publish(marker_array)

    def _republish_task_markers(self):
        """Keep task markers visible even when RViz discovers the topic late."""
        if self.last_task_markers is not None:
            self.task_marker_pub.publish(self.last_task_markers)

    def call_service(self, name, request):
        client = self.service_clients[name]
        if not client.service_is_ready():
            self.events.put(('log', f'[FAIL] {name}: service is not ready'))
            if name == 'start_localization':
                self.events.put(
                    (
                        'localization_service_result',
                        (False, 'start_localization service is not ready'),
                    )
                )
            return
        self.events.put(('log', f'[CALL] {name}'))
        future = client.call_async(request)
        future.add_done_callback(lambda done, label=name: self._service_done(label, done))

    def _service_done(self, name, future):
        try:
            response = future.result()
            success = getattr(response, 'success', False)
            message = getattr(response, 'message', str(response))
            tag = 'PASS' if success else 'REJECT'
            self.events.put(('log', f'[{tag}] {name}: {message}'))

            # 只有真正进入建图后才启动PGO RViz，避免start_mapping被拒绝时留下RViz。
            if name == 'start_mapping' and success:
                self.events.put(('start_pgo_rviz', None))

            # 地图完成并保存成功后关闭PGO RViz。
            if name == 'finish_mapping' and success:
                self.events.put(('stop_pgo_rviz', None))

            # 加载地图成功后，由GUI自行读取globalmap.pcd并启动RViz预览。
            if name == 'load_map' and success:
                self.events.put(('load_map_success', None))

            if name == 'start_localization':
                self.events.put(
                    ('localization_service_result', (success, message))
                )

        except Exception as error:
            self.events.put(('log', f'[ERROR] {name}: {error}'))
            if name == 'start_localization':
                self.events.put(
                    ('localization_service_result', (False, str(error)))
                )

    def send_pose_goal(self, goal):
        if not self.pose_action.server_is_ready():
            self.events.put(('log', '[FAIL] navigate_to_pose action is not ready'))
            return
        self.events.put(('log', '[CALL] navigate_to_pose'))
        future = self.pose_action.send_goal_async(
            goal, feedback_callback=self._action_feedback
        )
        future.add_done_callback(self._goal_response)

    def send_patrol_goal(self, goal):
        if not self.patrol_action.server_is_ready():
            self.events.put(('log', '[FAIL] start_navigation action is not ready'))
            return
        self.events.put(('log', '[CALL] start_navigation'))
        future = self.patrol_action.send_goal_async(
            goal, feedback_callback=self._action_feedback
        )
        future.add_done_callback(self._goal_response)

    def _goal_response(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.events.put(('log', '[REJECT] action goal rejected'))
                return
            self.active_goal_handle = goal_handle
            self.events.put(('log', '[PASS] action goal accepted'))
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._action_result)
        except Exception as error:
            self.events.put(('log', f'[ERROR] action goal: {error}'))

    def _action_feedback(self, feedback):
        self.events.put(('log', f'[FEEDBACK] {feedback.feedback}'))

    def _action_result(self, future):
        try:
            wrapped = future.result()
            self.events.put(
                ('log', f'[RESULT] status={wrapped.status}, result={wrapped.result}')
            )
        except Exception as error:
            self.events.put(('log', f'[ERROR] action result: {error}'))
        self.active_goal_handle = None

    def cancel_goal(self):
        if self.active_goal_handle is None:
            self.events.put(('log', '[INFO] no active navigation goal'))
            return
        future = self.active_goal_handle.cancel_goal_async()
        future.add_done_callback(
            lambda _done: self.events.put(('log', '[CALL] cancel navigation goal'))
        )

    def readiness(self):
        services = {
            name: client.service_is_ready()
            for name, client in self.service_clients.items()
        }
        actions = {
            'start_navigation': self.patrol_action.server_is_ready(),
            'navigate_to_pose': self.pose_action.server_is_ready(),
        }
        workers = {
            name: name in self.worker_states for name in EXPECTED_WORKERS
        }
        return services, actions, workers


class RuntimeTestGui:
    def __init__(self):
        self.workspace_dir = find_workspace_dir(os.path.realpath(__file__))
        self.data_root = '/robotfs/data/slam_data'
        self.log_root = os.path.join(self.data_root, 'log')

        # GUI 启动时尚未选择 map_id，因此先使用临时日志目录。
        # 一旦用户点击“开始建图”或“加载地图”选定 map_id，日志会切换到：
        # /robotfs/data/slam_data/log/map_<id>/<时间戳>/
        self.session_time = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        self.bootstrap_log_directory = os.path.join(
            self.log_root, self.session_time
        )
        self.log_directory = self.bootstrap_log_directory
        self.ros_log_directory = os.path.join(self.log_directory, 'ros')
        os.makedirs(self.log_directory, exist_ok=True)
        self.gui_log_file = os.path.join(
            self.log_directory, 'gui.log'
        )
        self.platform_launch_log_file = os.path.join(
            self.log_directory, 'platform.log'
        )
        self.log_map_id = None
        self.platform_launch_process = None
        self.platform_launch_log_handle = None
        self.topology_editor_script = os.path.join(
            self.workspace_dir, 'src', 'run_rviz_topology_editor.sh'
        )
        self.keyboard_control_script = os.path.join(
            self.workspace_dir, 'src', 'scripts', 'keyboard_control.py'
        )
        self.kill_nodes_script = os.path.join(
            self.workspace_dir, 'src', 'run_kill_node.sh'
        )
        self.topology_editor_process = None
        self.topology_editor_online = False
        self.keyboard_control_process = None
        self.pgo_rviz_process = None
        self.map_preview_pcd_process = None
        self.map_preview_rviz_process = None
        self.exiting = False
        self.last_pose_log_at = 0.0

        self.events = queue.Queue()
        rclpy.init()
        self.node = RuntimeTestNode(self.events)
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

        self.root = tk.Tk()
        self.root.title('slam_manager APP 接口测试')
        self.root.geometry('980x820')
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self._configure_button_styles()

        self.current_map_id = None
        self.initial_pose_enabled = tk.BooleanVar(value=False)
        self.localization_request_active = False
        self.localization_session_started = False
        self.localization_wait_for_worker_stop = False
        self.initial_pose_prompt_shown = False
        self.localization_success_shown = False
        self.current_pose_vars = {
            name: tk.StringVar(value='--')
            for name in ('map_id', 'x', 'y', 'z', 'theta')
        }
        self.localization_pose_vars = {
            name: tk.StringVar(value='0.00')
            for name in ('x', 'y', 'z', 'theta')
        }
        self.navigation_pose_vars = {
            name: tk.StringVar(value='0.00')
            for name in ('x', 'y', 'z', 'theta')
        }
        self.task_id = tk.StringVar(value='task_0')
        self.patrol_start_node_id = tk.StringVar(value='')
        self.patrol_end_node_id = tk.StringVar(value='')
        self.patrol_loop = tk.BooleanVar(value=False)
        self.runtime_label = tk.StringVar(value='Runtime: 未连接')
        self.worker_label = tk.StringVar(
            value=f'Workers: 0/{len(EXPECTED_WORKERS)}'
        )
        self.data_label = tk.StringVar(value='Pose=0  Cloud=0  Keyframe=0')
        self.localization_status = tk.StringVar(value='状态：未启动')
        self.log_path_label = tk.StringVar(
            value=f'本地运行日志: {self.log_directory}'
        )

        self._create_widgets()
        self.task_combo.bind(
            '<<ComboboxSelected>>',
            self._load_selected_task_defaults,
        )
        self.task_combo.bind(
            '<FocusOut>',
            self._load_selected_task_defaults,
        )
        self.root.after(100, self._drain_events)
        self._append_log(
            '[READY] GUI已启动，等待选择 map_id；'
            f'data_root={self.data_root}, log={self.log_directory}'
        )

    def _configure_button_styles(self):
        """为主要操作按钮配置一致且易区分的语义颜色。"""
        style = ttk.Style(self.root)
        if 'clam' in style.theme_names():
            # clam主题支持ttk按钮背景色，避免系统原生主题忽略颜色配置。
            style.theme_use('clam')
        palette = {
            'Primary.TButton': ('#2563eb', '#ffffff', '#1d4ed8'),
            'Success.TButton': ('#16a34a', '#ffffff', '#15803d'),
            'Warning.TButton': ('#f59e0b', '#111827', '#d97706'),
            'Danger.TButton': ('#dc2626', '#ffffff', '#b91c1c'),
            'Accent.TButton': ('#7c3aed', '#ffffff', '#6d28d9'),
            'Secondary.TButton': ('#475569', '#ffffff', '#334155'),
        }
        for name, (background, foreground, active_background) in palette.items():
            style.configure(
                name,
                background=background,
                foreground=foreground,
                padding=(8, 5),
            )
            style.map(
                name,
                background=[
                    ('disabled', '#cbd5e1'),
                    ('active', active_background),
                ],
                foreground=[('disabled', '#64748b')],
            )

    def _configure_log_for_map(self, map_id):
        """按 map_id 切换本次会话日志目录，并迁移 GUI 启动阶段日志。"""
        if self.log_map_id == map_id:
            return

        if (
            self.platform_launch_process is not None
            and self.platform_launch_process.poll() is None
        ):
            # platform 已启动后不能再改变其 ROS_LOG_DIR。
            if self.log_map_id != map_id:
                raise RuntimeError(
                    f'platform 当前日志已绑定到 {self.log_map_id}，'
                    f'请先清理节点后再切换到 {map_id}'
                )
            return

        old_gui_log = self.gui_log_file
        new_log_directory = os.path.join(
            self.log_root, map_id, self.session_time
        )
        os.makedirs(new_log_directory, exist_ok=True)
        new_ros_log_directory = os.path.join(new_log_directory, 'ros')
        os.makedirs(new_ros_log_directory, exist_ok=True)

        new_gui_log = os.path.join(
            new_log_directory, 'gui.log'
        )
        if os.path.isfile(old_gui_log) and old_gui_log != new_gui_log:
            try:
                with open(old_gui_log, 'r', encoding='utf-8') as src, \
                     open(new_gui_log, 'a', encoding='utf-8') as dst:
                    dst.write(src.read())
                os.remove(old_gui_log)
            except OSError as error:
                print(f'[WARN] 迁移GUI启动日志失败: {error}', flush=True)

        self.log_directory = new_log_directory
        self.ros_log_directory = new_ros_log_directory
        self.gui_log_file = new_gui_log
        self.platform_launch_log_file = os.path.join(
            new_log_directory, 'platform.log'
        )
        self.log_map_id = map_id
        self.log_path_label.set(f'本地运行日志: {self.log_directory}')

        # 尽量清理已经为空的临时目录。
        try:
            if (
                self.bootstrap_log_directory != new_log_directory
                and os.path.isdir(self.bootstrap_log_directory)
                and not os.listdir(self.bootstrap_log_directory)
            ):
                os.rmdir(self.bootstrap_log_directory)
                parent = os.path.dirname(self.bootstrap_log_directory)
                if os.path.isdir(parent) and not os.listdir(parent):
                    os.rmdir(parent)
        except OSError:
            pass

        self._append_log(f'[INFO] map_id={map_id} 日志目录: {self.log_directory}')

    def _platform_environment(self):
        environment = os.environ.copy()
        environment.update({
            'ROS_LOG_DIR': self.ros_log_directory,
            'WORKFLOW_LOG_DIR': self.log_directory,
            'LANG': 'C.UTF-8',
            'LC_ALL': 'C.UTF-8',
            'LANGUAGE': 'C.UTF-8',
        })
        return environment

    def _start_platform_launch(self):
        if self.platform_launch_process is not None:
            if self.platform_launch_process.poll() is None:
                self._append_log('[INFO] platform launch 已经在运行')
                return
            self.platform_launch_process = None

        launch_shell = (
            'source /opt/ros/humble/setup.bash && '
            f'source {self._shell_quote(os.path.join(self.workspace_dir, "install", "setup.bash"))} && '
            'exec ros2 launch slam_manager_test slam_manager_platform_test.launch.py '
            f'data_root:={self._shell_quote(self.data_root)}'
        )
        command = ['bash', '-lc', launch_shell]

        try:
            self.platform_launch_log_handle = open(
                self.platform_launch_log_file, 'a', encoding='utf-8', buffering=1
            )
            self._append_log(
                '[CALL] 启动平台 launch: ros2 launch slam_manager_test '
                'slam_manager_platform_test.launch.py '
                f'data_root:={self.data_root}'
            )
            self._append_log(
                f'[INFO] 平台总 stdout/stderr: {self.platform_launch_log_file}'
            )
            self.platform_launch_process = subprocess.Popen(
                command,
                cwd=self.workspace_dir,
                env=self._platform_environment(),
                stdout=self.platform_launch_log_handle,
                stderr=subprocess.STDOUT,
                start_new_session=True,
                text=True,
            )
            threading.Thread(
                target=self._watch_platform_launch, daemon=True
            ).start()
        except Exception as error:
            self._append_log(f'[ERROR] 启动 platform launch 失败: {error}')
            if self.platform_launch_log_handle is not None:
                try:
                    self.platform_launch_log_handle.close()
                except Exception:
                    pass
                self.platform_launch_log_handle = None
            messagebox.showerror('平台启动失败', str(error))

    def _watch_platform_launch(self):
        process = self.platform_launch_process
        if process is None:
            return
        return_code = process.wait()
        self.events.put(('platform_launch_done', return_code))

    def _close_platform_log_handle(self):
        if self.platform_launch_log_handle is not None:
            try:
                self.platform_launch_log_handle.flush()
                self.platform_launch_log_handle.close()
            except Exception:
                pass
            self.platform_launch_log_handle = None

    def _call_service_after_platform_ready(self, service_name, request, timeout_sec=20.0):
        """启动 platform 后等待指定服务就绪，再发送请求，避免刚启动就报 not ready。"""
        client = self.node.service_clients[service_name]
        deadline = datetime.datetime.now().timestamp() + timeout_sec

        def poll():
            if client.service_is_ready():
                self._append_log(f'[READY] {service_name} service 已就绪')
                self.node.call_service(service_name, request)
                return
            if datetime.datetime.now().timestamp() >= deadline:
                self._append_log(
                    f'[FAIL] 等待 {service_name} service 超时（{timeout_sec:.0f}s）'
                )
                messagebox.showerror(
                    '平台启动超时',
                    f'等待 {service_name} 服务超过 {timeout_sec:.0f} 秒。\n\n'
                    f'请查看日志：\n{self.platform_launch_log_file}',
                    parent=self.root,
                )
                return
            self.root.after(200, poll)

        poll()

    @staticmethod
    def _create_pose_input_grid(parent, variables, title):
        """Create compact editable x/y/z/theta fields inside a control group."""
        frame = ttk.LabelFrame(parent, text=title, padding=4)
        frame.pack(fill=tk.X, pady=(4, 6))
        labels = (('x', 'x'), ('y', 'y'), ('z', 'z'), ('theta(rad)', 'theta'))
        for index, (label, key) in enumerate(labels):
            row = index // 2
            column = (index % 2) * 2
            ttk.Label(frame, text=label).grid(
                row=row, column=column, sticky='e', padx=(2, 2), pady=2
            )
            ttk.Entry(
                frame,
                textvariable=variables[key],
                width=9,
            ).grid(row=row, column=column + 1, sticky='ew', pady=2)
        frame.columnconfigure(1, weight=1)
        frame.columnconfigure(3, weight=1)

    def _create_widgets(self):
        root = ttk.Frame(self.root, padding=10)
        root.pack(fill=tk.BOTH, expand=True)

        status = ttk.LabelFrame(root, text='连接与状态', padding=8)
        status.pack(fill=tk.X)
        ttk.Label(status, textvariable=self.runtime_label).grid(row=0, column=0, sticky='w')
        ttk.Label(status, textvariable=self.worker_label).grid(row=0, column=1, padx=20)
        ttk.Label(status, textvariable=self.data_label).grid(row=0, column=2, padx=20)
        ttk.Button(
            status,
            text='检查全部接口',
            command=self.check_readiness,
            style='Secondary.TButton',
        ).grid(
            row=0, column=3, padx=8
        )
        ttk.Label(
            status,
            textvariable=self.log_path_label,
            foreground='#475569',
        ).grid(row=1, column=0, columnspan=4, sticky='w', pady=(6, 0))

        top_right_actions = ttk.Frame(status)
        top_right_actions.grid(
            row=0, column=4, rowspan=2, sticky='ne', padx=(12, 0)
        )
        self.keyboard_control_button = ttk.Button(
            top_right_actions,
            text='键盘控制',
            command=self.start_keyboard_control,
            style='Secondary.TButton',
        )
        self.keyboard_control_button.pack(fill=tk.X)
        self.delete_all_logs_button = ttk.Button(
            top_right_actions,
            text='删除全部日志',
            command=self.delete_all_logs,
            style='Danger.TButton',
        )
        self.delete_all_logs_button.pack(fill=tk.X, pady=(6, 0))
        self.exit_button = ttk.Button(
            top_right_actions,
            text='退出GUI',
            command=self.exit_gui,
            style='Danger.TButton',
        )
        self.exit_button.pack(fill=tk.X, pady=(6, 0))
        status.columnconfigure(4, weight=1)

        common = ttk.LabelFrame(
            root,
            text='当前定位位姿（/robot/current_pose，rad）',
            padding=8,
        )
        common.pack(fill=tk.X, pady=8)
        for column, (name, variable) in enumerate(
            self.current_pose_vars.items()
        ):
            ttk.Label(common, text=name).grid(
                row=0, column=column * 2, padx=(8 if column else 0, 2)
            )
            ttk.Entry(
                common,
                textvariable=variable,
                width=10,
                state='readonly',
            ).grid(
                row=0, column=column * 2 + 1, sticky='ew'
            )

        controls = ttk.Frame(root)
        controls.pack(fill=tk.X)

        mapping = ttk.LabelFrame(controls, text='1. 建图', padding=8)
        mapping.grid(row=0, column=0, sticky='nsew', padx=(0, 4))
        ttk.Button(
            mapping,
            text='开始建图',
            command=self.start_mapping,
            style='Primary.TButton',
        ).pack(fill=tk.X)
        ttk.Button(
            mapping,
            text='完成并保存',
            command=self.finish_mapping,
            style='Success.TButton',
        ).pack(
            fill=tk.X, pady=4
        )
        ttk.Button(
            mapping,
            text='加载地图',
            command=self.load_map,
            style='Primary.TButton',
        ).pack(fill=tk.X)
        ttk.Button(
            mapping,
            text='删除地图',
            command=self.delete_map,
            style='Danger.TButton',
        ).pack(fill=tk.X, pady=4)

        maps = ttk.LabelFrame(controls, text='2. 打点', padding=8)
        maps.grid(row=0, column=1, sticky='nsew', padx=4)
        self.topology_editor_button = ttk.Button(
            maps,
            text='离线打点',
            command=self.start_topology_editor,
            style='Accent.TButton',
        )
        self.topology_editor_button.pack(fill=tk.X)
        self.online_topology_editor_button = ttk.Button(
            maps,
            text='在线打点',
            command=self.start_online_topology_editor,
            style='Success.TButton',
        )
        self.online_topology_editor_button.pack(fill=tk.X, pady=(4, 0))

        localization = ttk.LabelFrame(controls, text='3. 定位', padding=8)
        localization.grid(row=0, column=2, sticky='nsew', padx=4)
        ttk.Checkbutton(
            localization, text='使用初始位姿', variable=self.initial_pose_enabled
        ).pack(anchor='w')
        self._create_pose_input_grid(
            localization,
            self.localization_pose_vars,
            '初始位姿（可手动或RViz设置）',
        )
        self.localization_button = ttk.Button(
            localization,
            text='启动重定位',
            command=self.start_localization,
            style='Primary.TButton',
        )
        self.localization_button.pack(fill=tk.X, pady=4)
        ttk.Label(
            localization,
            textvariable=self.localization_status,
            foreground='#475569',
        ).pack(anchor='w')

        navigation = ttk.LabelFrame(controls, text='4. 单点导航', padding=8)
        navigation.grid(row=0, column=3, sticky='nsew', padx=(4, 0))
        self._create_pose_input_grid(
            navigation,
            self.navigation_pose_vars,
            '目标位姿（可手动或RViz设置）',
        )
        ttk.Button(
            navigation,
            text='发送目标',
            command=self.navigate_to_pose,
            style='Success.TButton',
        ).pack(
            fill=tk.X
        )
        ttk.Button(
            navigation,
            text='取消目标',
            command=self.node.cancel_goal,
            style='Danger.TButton',
        ).pack(
            fill=tk.X, pady=4
        )
        pause_row = ttk.Frame(navigation)
        pause_row.pack(fill=tk.X)
        ttk.Button(
            pause_row,
            text='暂停',
            command=self.pause_navigation,
            style='Warning.TButton',
        ).pack(
            side=tk.LEFT, expand=True, fill=tk.X
        )
        ttk.Button(
            pause_row,
            text='恢复',
            command=self.resume_navigation,
            style='Success.TButton',
        ).pack(
            side=tk.LEFT, expand=True, fill=tk.X
        )

        for column in range(4):
            controls.columnconfigure(column, weight=1)

        patrol = ttk.LabelFrame(
            root,
            text='5. 巡检导航（任务来自当前地图 task/*.json）',
            padding=8,
        )
        patrol.pack(fill=tk.X, pady=8)

        patrol_top = ttk.Frame(patrol)
        patrol_top.pack(fill=tk.X)

        ttk.Label(patrol_top, text='task_id').pack(side=tk.LEFT)

        # 可直接输入task_id，也可从当前map_x/task/*.json下拉选择。
        self.task_combo = ttk.Combobox(
            patrol_top,
            textvariable=self.task_id,
            width=18,
            state='normal',
        )
        self.task_combo.pack(side=tk.LEFT, padx=4)

        ttk.Button(
            patrol_top,
            text='刷新任务',
            command=self.refresh_task_ids,
            style='Secondary.TButton',
        ).pack(side=tk.LEFT, padx=(4, 0))

        ttk.Checkbutton(
            patrol_top,
            text='循环',
            variable=self.patrol_loop,
        ).pack(side=tk.LEFT, padx=(8, 0))

        patrol_actions = ttk.Frame(patrol_top)
        patrol_actions.pack(side=tk.RIGHT)

        ttk.Button(
            patrol_actions,
            text='发送巡检任务',
            command=self.start_patrol,
            style='Success.TButton',
        ).pack(side=tk.LEFT)
        ttk.Button(
            patrol_actions,
            text='暂停巡检',
            command=self.pause_navigation,
            style='Warning.TButton',
        ).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Button(
            patrol_actions,
            text='恢复巡检',
            command=self.resume_navigation,
            style='Success.TButton',
        ).pack(side=tk.LEFT, padx=(6, 0))

        patrol_range = ttk.Frame(patrol)
        patrol_range.pack(fill=tk.X, pady=(8, 0))
        ttk.Label(patrol_range, text='开始节点ID').pack(side=tk.LEFT)
        start_node_entry = ttk.Entry(
            patrol_range,
            textvariable=self.patrol_start_node_id,
            width=10,
        )
        start_node_entry.pack(side=tk.LEFT, padx=(4, 12))
        ttk.Label(patrol_range, text='结束节点ID').pack(side=tk.LEFT)
        end_node_entry = ttk.Entry(
            patrol_range,
            textvariable=self.patrol_end_node_id,
            width=10,
        )
        end_node_entry.pack(side=tk.LEFT, padx=(4, 0))
        start_node_entry.bind(
            '<FocusOut>', lambda _event: self._preview_selected_task_nodes()
        )
        end_node_entry.bind(
            '<FocusOut>', lambda _event: self._preview_selected_task_nodes()
        )

        self.patrol_nodes_var = tk.StringVar(value='节点顺序：未读取任务')
        ttk.Label(
            patrol,
            textvariable=self.patrol_nodes_var,
        ).pack(anchor='w', pady=(6, 0))

        instructions = ttk.LabelFrame(root, text='操作说明', padding=8)
        instructions.pack(fill=tk.X, pady=(0, 8))
        ttk.Label(
            instructions,
            text=(
                '1. 建图：点击“开始建图”进行建图，建图完成后点击'
                '“完成并保存”保存地图。'
            ),
            justify=tk.LEFT,
        ).pack(anchor='w')
        ttk.Label(
            instructions,
            text=(
                '2. 在线打点：先“加载地图” → 点击“启动重定位”并等待定位成功 '
                '→ 点击“在线打点”并输入 task_id → 点击右上角“键盘控制” '
                '→ 移动机器人，到目标位置后按 K 打点并保存。'
            ),
            justify=tk.LEFT,
            wraplength=920,
        ).pack(anchor='w', pady=(6, 0))
        ttk.Label(
            instructions,
            text=(
                '3. 巡检导航：先“加载地图” → 点击“启动重定位”并等待定位成功 '
                '→ 选择 task_id，确认或修改开始/结束节点ID '
                '→ 点击“发送巡检任务”开始巡检。'
            ),
            justify=tk.LEFT,
            wraplength=920,
        ).pack(anchor='w', pady=(6, 0))

    def _uint(self, value, name):
        parsed = int(value)
        if parsed < 0 or parsed > 0xFFFFFFFF:
            raise ValueError(f'{name} 超出 uint32 范围')
        return parsed

    def _identifier(self, value, name, required_prefix=None):
        value = value.strip()
        if not value or len(value) > 64:
            raise ValueError(f'{name} 长度必须为 1～64 个字符')
        if required_prefix is not None and not value.startswith(required_prefix):
            raise ValueError(f'{name} 必须以 {required_prefix} 开头')
        if re.fullmatch(r'[A-Za-z0-9_-]+', value) is None:
            raise ValueError(f"{name} 只允许字母、数字、'_' 和 '-'")
        return value

    @staticmethod
    def _pose_values(variables):
        values = tuple(
            float(variables[name].get())
            for name in ('x', 'y', 'z', 'theta')
        )
        for name, value in zip(('x', 'y', 'z', 'theta'), values):
            variables[name].set(f'{value:.2f}')
        return values

    def _prompt_map_id(self, action_title, require_existing=False):
        """弹窗读取 uint32 地图编号，并统一转换为 map_<id>。"""
        initial_value = ''
        if self.current_map_id and self.current_map_id.startswith('map_'):
            initial_value = self.current_map_id[4:]

        while True:
            value = simpledialog.askstring(
                action_title,
                '请输入 map_id（0～4294967295，不能为空）：',
                initialvalue=initial_value,
                parent=self.root,
            )

            if value is None:
                return None

            value = value.strip()
            if not value:
                messagebox.showerror(
                    'map_id 无效', 'map_id 不能为空。', parent=self.root
                )
                continue

            if not value.isdecimal():
                messagebox.showerror(
                    'map_id 无效',
                    'map_id 必须是 0～4294967295 的非负整数。',
                    parent=self.root,
                )
                continue

            map_number = int(value, 10)
            if map_number > 0xFFFFFFFF:
                messagebox.showerror(
                    'map_id 无效',
                    'map_id 超出 uint32 范围（最大 4294967295）。',
                    parent=self.root,
                )
                continue

            map_id = f'map_{map_number}'
            if require_existing:
                map_directory = os.path.join(self.data_root, map_id)
                if not os.path.isdir(map_directory):
                    messagebox.showerror(
                        '地图不存在',
                        f'没有找到 {map_id}：\n{map_directory}',
                        parent=self.root,
                    )
                    self._append_log(
                        f'[FAIL] {action_title}: 地图目录不存在 {map_directory}'
                    )
                    initial_value = value
                    continue

            self.current_map_id = map_id
            return map_id

    def _prepare_new_map_directories(self, map_id):
        """由GUI创建新地图的数据目录；slam_manager只检查和写入。"""
        map_data_directory = os.path.join(self.data_root, map_id)
        map_directory = os.path.join(map_data_directory, 'map')
        task_directory = os.path.join(map_data_directory, 'task')

        if os.path.exists(map_data_directory):
            delete_existing = messagebox.askyesno(
                '地图已存在',
                f'{map_id} 已经存在：\n{map_data_directory}\n\n'
                '是否删除原地图并重新建图？\n\n'
                '注意：只删除地图数据目录，历史日志不会删除。',
                parent=self.root,
            )
            if not delete_existing:
                return False

            self._append_log(
                f'[WARN] 用户确认删除已有地图并重新建图: {map_data_directory}'
            )
            try:
                if os.path.isdir(map_data_directory):
                    shutil.rmtree(map_data_directory)
                else:
                    os.remove(map_data_directory)
            except OSError as error:
                raise RuntimeError(
                    f'删除已有地图失败：\n{map_data_directory}\n{error}'
                ) from error

        try:
            os.makedirs(map_directory, exist_ok=False)
            os.makedirs(task_directory, exist_ok=False)
        except Exception:
            # 创建过程失败时只清理由本次GUI刚创建的目录。
            try:
                if os.path.isdir(map_data_directory):
                    shutil.rmtree(map_data_directory)
            except OSError:
                pass
            raise

        self._append_log(f'[READY] 建图目录已准备: {map_data_directory}')
        return True

    def start_mapping(self):
        map_id = self._prompt_map_id('开始建图')
        if map_id is None:
            return

        try:
            # 先处理已有地图。用户取消删除时，不启动平台，也不切换日志目录。
            if not self._prepare_new_map_directories(map_id):
                return
            self._configure_log_for_map(map_id)
        except (RuntimeError, OSError) as error:
            messagebox.showerror('无法开始建图', str(error), parent=self.root)
            self._append_log(f'[FAIL] 开始建图目录准备失败: {error}')
            return

        # 确定 map_id 后才启动整个平台，
        # WORKFLOW_LOG_DIR落到session根目录；ROS_LOG_DIR单独落到session/ros/。
        self._start_platform_launch()

        request = StartMapping.Request()
        request.map_id = map_id
        self._append_log(f'[CALL] 准备开始建图: map_id={map_id}')
        self._call_service_after_platform_ready('start_mapping', request)

    def finish_mapping(self):
        self.node.call_service('finish_mapping', Trigger.Request())

    def load_map(self):
        map_id = self._prompt_map_id('加载地图', require_existing=True)
        if map_id is None:
            return

        try:
            self._configure_log_for_map(map_id)
        except RuntimeError as error:
            messagebox.showerror('无法切换地图', str(error), parent=self.root)
            return
        self._start_platform_launch()

        request = LoadMap.Request()
        request.map_id = map_id
        self._append_log(f'[CALL] 准备加载地图: map_id={map_id}')
        self._call_service_after_platform_ready('load_map', request)

    def delete_map(self):
        map_id = self._prompt_map_id('删除地图', require_existing=True)
        if map_id is None:
            return

        if not messagebox.askyesno(
            '确认删除地图',
            f'确定删除 {map_id} 吗？',
            parent=self.root,
        ):
            return

        try:
            self._configure_log_for_map(map_id)
        except RuntimeError as error:
            messagebox.showerror('无法切换地图', str(error), parent=self.root)
            return
        self._start_platform_launch()

        request = DeleteMap.Request()
        request.map_id = map_id
        self._append_log(f'[CALL] 准备删除地图: map_id={map_id}')
        self._call_service_after_platform_ready('delete_map', request)

    def start_localization(self):
        if self.localization_request_active:
            messagebox.showwarning(
                '重定位进行中',
                '当前正在等待重定位结果，请勿重复启动。',
                parent=self.root,
            )
            return
        if not self.current_map_id:
            messagebox.showwarning(
                '未加载地图',
                '请先点击“加载地图”，成功后再启动重定位。',
                parent=self.root,
            )
            return
        try:
            x, y, z, theta = self._pose_values(self.localization_pose_vars)
            request = StartLocalization.Request()
            request.use_initial_pose = self.initial_pose_enabled.get()
            request.x, request.y, request.z, request.theta = x, y, z, theta
            localization_worker = self.node.worker_states.get(
                'fastlio_localization_worker'
            )
            self.localization_wait_for_worker_stop = (
                localization_worker is not None
                and localization_worker.state == WorkerStatus.RUNNING
            )
            self.localization_request_active = True
            self.localization_session_started = False
            self.initial_pose_prompt_shown = False
            self.localization_success_shown = False
            self.localization_status.set('状态：正在启动重定位…')
            self.localization_button.configure(state=tk.DISABLED)
            self._append_log(
                '[INFO] 正在启动重定位；定位Worker就绪后GUI会提示何时设置初始位姿'
            )
            self.node.call_service('start_localization', request)
        except ValueError as error:
            messagebox.showerror('参数错误', str(error))

    def _notify_localization_worker_ready(self):
        if (
            not self.localization_request_active
            or not self.localization_session_started
            or self.initial_pose_prompt_shown
        ):
            return

        worker = self.node.worker_states.get('fastlio_localization_worker')
        if worker is None or worker.state != WorkerStatus.RUNNING:
            return
        if self.localization_wait_for_worker_stop:
            return

        self.initial_pose_prompt_shown = True
        if self.initial_pose_enabled.get():
            self.localization_status.set('状态：定位中（已发送GUI初始位姿）')
            self._append_log(
                '[READY] 定位Worker已启动，GUI初始位姿已自动发送'
            )
            return

        self.localization_status.set('状态：定位中，可在RViz设置初始位姿')
        self._append_log(
            '[READY] 定位Worker已启动，正在全局重定位；'
            '可在RViz使用“2D Pose Estimate”辅助定位'
        )

    def _handle_relocalization_feedback(self, success):
        if not self.localization_request_active or not self.localization_session_started:
            return
        if not success:
            self.localization_status.set('状态：正在搜索匹配位姿…')
            return
        self.localization_status.set('状态：重定位成功')
        if self.localization_success_shown:
            return
        self.localization_success_shown = True
        messagebox.showinfo(
            '重定位成功',
            '已收到 /reloc_success=true。\n\n'
            '定位数据正在持续输出，RViz 中将显示红色的当前定位点云。',
            parent=self.root,
        )

    def _handle_localization_service_result(self, success, message):
        self.localization_request_active = False
        self.localization_session_started = False
        self.localization_button.configure(state=tk.NORMAL)
        if success:
            self.localization_status.set('状态：重定位成功，纯定位运行中')
            if not self.localization_success_shown:
                self.localization_success_shown = True
                messagebox.showinfo(
                    '重定位成功',
                    f'{message}\n\n定位数据正在持续输出。',
                    parent=self.root,
                )
            return

        self.localization_status.set('状态：重定位失败')
        messagebox.showerror(
            '重定位失败',
            f'{message}\n\n请检查定位日志、地图和初始位姿后重试。',
            parent=self.root,
        )

    def navigate_to_pose(self):
        try:
            goal = NavigateToPose.Goal()
            goal.x, goal.y, goal.z, goal.theta = self._pose_values(
                self.navigation_pose_vars
            )
            self.node.send_pose_goal(goal)
        except ValueError as error:
            messagebox.showerror('参数错误', str(error))

    def _current_task_directory(self):
        if not self.current_map_id:
            raise ValueError('请先加载地图，再选择巡检任务')

        task_directory = os.path.join(
            self.data_root,
            self.current_map_id,
            'task',
        )

        if not os.path.isdir(task_directory):
            raise ValueError(
                f'当前地图任务目录不存在：{task_directory}'
            )

        return task_directory

    def refresh_task_ids(self):
        """扫描当前map_x/task/*.json，刷新task_id下拉列表。"""
        try:
            task_directory = self._current_task_directory()

            task_ids = sorted(
                os.path.splitext(filename)[0]
                for filename in os.listdir(task_directory)
                if filename.endswith('.json')
                and filename != 'topology_map.json'
                and os.path.isfile(os.path.join(task_directory, filename))
            )

            self.task_combo['values'] = task_ids

            if task_ids:
                if self.task_id.get().strip() not in task_ids:
                    self.task_id.set(task_ids[0])

                self._append_log(
                    f'[INFO] 当前地图 {self.current_map_id} 发现任务: '
                    + ', '.join(task_ids)
                )
                self._load_selected_task_defaults()
            else:
                self.patrol_start_node_id.set('')
                self.patrol_end_node_id.set('')
                self.patrol_nodes_var.set('节点顺序：当前地图没有task/*.json')
                self._append_log(
                    f'[WARN] 当前地图 {self.current_map_id} 没有任务JSON: '
                    f'{task_directory}'
                )

        except ValueError as error:
            self.task_combo['values'] = ()
            self.patrol_start_node_id.set('')
            self.patrol_end_node_id.set('')
            self.patrol_nodes_var.set('节点顺序：未读取任务')
            messagebox.showwarning(
                '无法读取巡检任务',
                str(error),
                parent=self.root,
            )

    def _read_task_node_ids(self, task_id):
        """从当前map_x/task/task_id.json按nodes[]顺序提取id。"""
        task_directory = self._current_task_directory()
        task_file = os.path.join(
            task_directory,
            f'{task_id}.json',
        )

        if not os.path.isfile(task_file):
            raise ValueError(
                f'任务文件不存在：{task_file}'
            )

        try:
            with open(task_file, 'r', encoding='utf-8') as file:
                document = json.load(file)
        except (OSError, json.JSONDecodeError) as error:
            raise ValueError(
                f'读取任务JSON失败：{task_file}\n{error}'
            ) from error

        nodes = document.get('nodes')
        if not isinstance(nodes, list) or not nodes:
            raise ValueError(
                f'任务JSON缺少有效的 nodes[]：{task_file}'
            )

        node_ids = []
        seen = set()

        for index, item in enumerate(nodes):
            if not isinstance(item, dict) or 'id' not in item:
                raise ValueError(
                    f'nodes[{index}] 缺少 id：{task_file}'
                )

            node_id = self._uint(item['id'], f'nodes[{index}].id')

            if node_id in seen:
                raise ValueError(
                    f'任务JSON存在重复 node_id={node_id}'
                )

            seen.add(node_id)
            node_ids.append(node_id)

        return task_file, node_ids

    def _selected_patrol_node_ids(self, node_ids):
        """Return the inclusive JSON-order range selected by start/end IDs."""
        start_text = self.patrol_start_node_id.get().strip()
        end_text = self.patrol_end_node_id.get().strip()
        if not start_text or not end_text:
            raise ValueError('开始节点ID和结束节点ID不能为空')

        try:
            start_id = self._uint(start_text, '开始节点ID')
            end_id = self._uint(end_text, '结束节点ID')
        except (TypeError, ValueError) as error:
            raise ValueError(
                '开始节点ID和结束节点ID必须是 uint32 非负整数'
            ) from error

        if start_id not in node_ids:
            raise ValueError(f'开始节点ID={start_id} 不在任务JSON的 nodes[] 中')
        if end_id not in node_ids:
            raise ValueError(f'结束节点ID={end_id} 不在任务JSON的 nodes[] 中')

        start_index = node_ids.index(start_id)
        end_index = node_ids.index(end_id)
        if end_index <= start_index:
            raise ValueError(
                f'结束节点ID={end_id} 必须位于开始节点ID={start_id}之后'
            )

        return node_ids[start_index:end_index + 1]

    def _read_task_marker_nodes(self, task_file, selected_node_ids):
        """Read poses for the exact ordered node range sent to slam_manager."""
        try:
            with open(task_file, 'r', encoding='utf-8') as file:
                document = json.load(file)
            nodes_by_id = {}
            for index, item in enumerate(document['nodes']):
                node_id = self._uint(item['id'], f'nodes[{index}].id')
                pose = {
                    'id': node_id,
                    'x': float(item['x']),
                    'y': float(item['y']),
                    'z': float(item['z']),
                    'theta': float(item['theta']),
                }
                if not all(math.isfinite(pose[name]) for name in (
                    'x', 'y', 'z', 'theta'
                )):
                    raise ValueError(f'node_id={node_id} 位姿包含非有限值')
                nodes_by_id[node_id] = pose
        except (KeyError, TypeError, OSError, json.JSONDecodeError) as error:
            raise ValueError(f'无法读取任务节点位姿：{task_file}\n{error}') from error

        return [nodes_by_id[node_id] for node_id in selected_node_ids]

    def _load_selected_task_defaults(self, _event=None):
        """Load a task and initialize its editable range to first/last IDs."""
        try:
            task_id = self._identifier(self.task_id.get(), 'task_id')
            _task_file, node_ids = self._read_task_node_ids(task_id)
            self.patrol_start_node_id.set(str(node_ids[0]))
            self.patrol_end_node_id.set(str(node_ids[-1]))
            self._preview_selected_task_nodes()
        except ValueError as error:
            self.patrol_start_node_id.set('')
            self.patrol_end_node_id.set('')
            self.patrol_nodes_var.set(f'节点顺序：读取失败 - {error}')

    def _preview_selected_task_nodes(self):
        """只在GUI中显示当前task_id解析出的节点顺序。"""
        try:
            task_id = self._identifier(
                self.task_id.get(),
                'task_id',
            )
            task_file, all_node_ids = self._read_task_node_ids(task_id)
            node_ids = self._selected_patrol_node_ids(all_node_ids)
            marker_nodes = self._read_task_marker_nodes(task_file, node_ids)
            self.patrol_nodes_var.set(
                '发送节点顺序：'
                + ' -> '.join(str(node_id) for node_id in node_ids)
            )
            self.node.publish_task_markers(task_id, marker_nodes)
        except ValueError as error:
            self.patrol_nodes_var.set(f'节点顺序：读取失败 - {error}')

    def start_patrol(self):
        try:
            if not self.current_map_id:
                raise ValueError('请先加载地图，再发送巡检任务')

            task_id = self._identifier(
                self.task_id.get(),
                'task_id',
            )

            task_file, all_node_ids = self._read_task_node_ids(task_id)
            node_ids = self._selected_patrol_node_ids(all_node_ids)
            marker_nodes = self._read_task_marker_nodes(task_file, node_ids)

            goal = StartNavigation.Goal()
            goal.task_id = task_id
            goal.loop = self.patrol_loop.get()

            # GUI只发送task_id和node_id。
            # x/y/z/theta填0占位，真实位置由slam_manager根据
            # 当前map_id + task_id + node_id从JSON中解析。
            for node_id in node_ids:
                node = NavigationNode()
                node.node_id = node_id
                node.x = 0.0
                node.y = 0.0
                node.z = 0.0
                node.theta = 0.0
                goal.nodes.append(node)

            self.patrol_nodes_var.set(
                '发送节点顺序：'
                + ' -> '.join(str(node_id) for node_id in node_ids)
            )

            self._append_log(
                f'[CALL] 准备巡检导航: map_id={self.current_map_id}, '
                f'task_id={task_id}, nodes={node_ids}, '
                f'json_nodes={all_node_ids}, '
                f'json={task_file}, loop={goal.loop}'
            )

            self.node.publish_task_markers(task_id, marker_nodes)
            self._append_log(
                '[RVIZ] 已发布本次巡检任务节点、ID和顺序连线: '
                '/slam_manager_test/task_markers'
            )
            self.node.send_patrol_goal(goal)

        except ValueError as error:
            self._append_log(f'[FAIL] 巡检任务准备失败: {error}')
            messagebox.showerror(
                '巡检任务错误',
                str(error),
                parent=self.root,
            )

    def pause_navigation(self):
        self.node.call_service('pause_navigation', Trigger.Request())

    def resume_navigation(self):
        self.node.call_service('resume_navigation', Trigger.Request())

    @staticmethod
    def _find_terminal():
        for command in (
            'xfce4-terminal', 'gnome-terminal', 'konsole',
            'x-terminal-emulator', 'xterm',
        ):
            if shutil.which(command):
                return command
        return None

    @staticmethod
    def _shell_quote(text):
        return "'" + str(text).replace("'", "'\"'\"'") + "'"

    def _terminal_command(self, title, command):
        terminal = self._find_terminal()
        if terminal is None:
            return None
        if terminal == 'gnome-terminal':
            return [
                terminal, '--wait', '--title', title,
                '--', 'bash', '-lc', command,
            ]
        if terminal == 'xfce4-terminal':
            return [
                terminal, '--title', title, '--command',
                f'bash -lc {self._shell_quote(command)}',
            ]
        if terminal == 'konsole':
            return [
                terminal, '--new-tab', '-p', f'tabtitle={title}',
                '-e', 'bash', '-lc', command,
            ]
        return [terminal, '-T', title, '-e', 'bash', '-lc', command]

    def _start_pgo_rviz(self):
        """启动PGO专用RViz，仅由建图流程管理。"""
        if self.pgo_rviz_process is not None:
            if self.pgo_rviz_process.poll() is None:
                self._append_log('[INFO] PGO RViz 已经在运行')
                return
            self.pgo_rviz_process = None

        rviz_config = os.path.join(
            get_package_share_directory('pgo_sc'),
            'config',
            'pgo.rviz',
        )

        if not os.path.isfile(rviz_config):
            self._append_log(f'[ERROR] PGO RViz配置不存在: {rviz_config}')
            messagebox.showerror(
                'PGO RViz启动失败',
                f'找不到RViz配置文件：\n{rviz_config}',
                parent=self.root,
            )
            return

        command = [
            'rviz2',
            '-d',
            rviz_config,
        ]

        try:
            self.pgo_rviz_process = subprocess.Popen(
                command,
                cwd=self.workspace_dir,
                env=self._platform_environment(),
                start_new_session=True,
            )
            self._append_log(f'[CALL] 启动 PGO RViz: {rviz_config}')
            threading.Thread(
                target=self._watch_pgo_rviz,
                daemon=True,
            ).start()
        except Exception as error:
            self.pgo_rviz_process = None
            self._append_log(f'[ERROR] 启动 PGO RViz 失败: {error}')
            messagebox.showerror(
                'PGO RViz启动失败',
                str(error),
                parent=self.root,
            )

    def _stop_pgo_rviz(self):
        """停止GUI启动的PGO RViz进程组。"""
        process = self.pgo_rviz_process
        if process is None:
            return

        if process.poll() is not None:
            self.pgo_rviz_process = None
            return

        self._append_log('[CALL] 关闭 PGO RViz')
        self._stop_process_group(process, 'PGO RViz')
        self.pgo_rviz_process = None
        self._append_log('[RESULT] PGO RViz 已关闭')

    def _watch_pgo_rviz(self):
        process = self.pgo_rviz_process
        if process is None:
            return
        return_code = process.wait()
        self.events.put(('pgo_rviz_done', return_code))

    def _write_map_preview_rviz_config(self):
        """生成已加载PCD地图专用RViz配置。"""
        config_file = os.path.join(
            self.log_directory,
            'loaded_map_preview.rviz',
        )

        config = """Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz_default_plugins/PointCloud2
      Color: 255; 255; 255
      Color Transformer: Intensity
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Min Color: 0; 0; 0
      Name: Loaded Global Map
      Position Transformer: XYZ
      Queue Size: 10
      Selectable: true
      Size (Pixels): 2
      Size (m): 0.03
      Style: Points
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /cloud_pcd
      Use Fixed Frame: true
      Use rainbow: true
      Value: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Axis: Z
      Channel Name: intensity
      Class: rviz_default_plugins/PointCloud2
      Color: 239; 41; 41
      Color Transformer: FlatColor
      Decay Time: 0
      Enabled: true
      Name: Current Localization Scan
      Position Transformer: XYZ
      Queue Size: 10
      Selectable: true
      Size (Pixels): 3
      Size (m): 0.01
      Style: Points
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /cloud_registered
      Use Fixed Frame: true
      Value: true
    - Alpha: 1
      Axes Length: 1
      Axes Radius: 0.1
      Class: rviz_default_plugins/Pose
      Color: 0; 255; 0
      Enabled: true
      Head Length: 0.3
      Head Radius: 0.1
      Name: Current Robot Pose
      Shaft Length: 1
      Shaft Radius: 0.05
      Shape: Arrow
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /pose_stamped_localization
      Value: true
    - Class: rviz_default_plugins/MarkerArray
      Enabled: true
      Marker Topic:
        Depth: 5
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /slam_manager_test/task_markers
      Name: Current Patrol Task
      Namespaces:
        {}
      Value: true
    - Class: rviz_default_plugins/TF
      Enabled: true
      Frame Timeout: 15
      Frames:
        All Enabled: true
      Marker Scale: 1
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: true
      Tree:
        {}
      Update Interval: 0
      Value: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: map
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/SetInitialPose
      Covariance x: 0.25
      Covariance y: 0.25
      Covariance yaw: 0.06853891909122467
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /initialpose
    - Class: rviz_default_plugins/SetGoal
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /goal_pose
  Transformation:
    Current:
      Class: rviz_default_plugins/TF
  Views:
    Current:
      Angle: 0
      Class: rviz_default_plugins/TopDownOrtho
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.01
      Scale: 10
      Target Frame: map
      X: 0
      Y: 0
    Saved:
      - Angle: 0
        Class: rviz_default_plugins/TopDownOrtho
        Invert Z Axis: false
        Name: TopDownOrtho
        Near Clip Distance: 0.01
        Scale: 10
        Target Frame: map
        X: 0
        Y: 0
      - Class: rviz_default_plugins/Orbit
        Distance: 30
        Focal Point:
          X: 0
          Y: 0
          Z: 0
        Invert Z Axis: false
        Name: Orbit
        Near Clip Distance: 0.01
        Pitch: 0.785
        Target Frame: map
        Yaw: 0.785
Window Geometry:
  Height: 900
  Width: 1400
"""

        with open(config_file, 'w', encoding='utf-8') as file:
            file.write(config)

        return config_file

    def _start_loaded_map_preview(self, map_id):
        """GUI自行读取globalmap.pcd，发布/cloud_pcd并启动RViz显示。"""
        if not map_id:
            self._append_log('[ERROR] 地图PCD预览失败: map_id为空')
            return

        pcd_file = os.path.join(
            self.data_root,
            map_id,
            'map',
            'globalmap.pcd',
        )

        if not os.path.isfile(pcd_file):
            self._append_log(
                f'[ERROR] load_map成功，但GUI找不到地图PCD: {pcd_file}'
            )
            messagebox.showerror(
                '地图PCD不存在',
                f'找不到地图点云：\n\n{pcd_file}',
                parent=self.root,
            )
            return

        # 重新加载地图时，先关闭上一次GUI预览。
        self._stop_loaded_map_preview()

        try:
            rviz_config = self._write_map_preview_rviz_config()

            # pcl_ros/pcd_to_pointcloud:
            #   file_name: PCD文件
            #   tf_frame: 点云frame
            #   默认输出topic为 /cloud_pcd
            pcd_command = [
                'ros2',
                'run',
                'pcl_ros',
                'pcd_to_pointcloud',
                '--ros-args',
                '-p',
                f'file_name:={pcd_file}',
                '-p',
                'tf_frame:=map',
                '-p',
                'publishing_period_ms:=1000',
            ]

            self.map_preview_pcd_process = subprocess.Popen(
                pcd_command,
                cwd=self.workspace_dir,
                env=self._platform_environment(),
                stdout=subprocess.DEVNULL,
                stderr=subprocess.STDOUT,
                start_new_session=True,
                text=True,
            )

            self._append_log(
                f'[CALL] GUI加载地图PCD: map_id={map_id}, '
                f'pcd={pcd_file}, topic=/cloud_pcd'
            )

            rviz_command = [
                'rviz2',
                '-d',
                rviz_config,
            ]

            self.map_preview_rviz_process = subprocess.Popen(
                rviz_command,
                cwd=self.workspace_dir,
                env=self._platform_environment(),
                start_new_session=True,
            )

            self._append_log(
                f'[CALL] 启动地图预览RViz: map_id={map_id}, '
                'Fixed Frame=map, topic=/cloud_pcd'
            )

            threading.Thread(
                target=self._watch_map_preview_pcd,
                daemon=True,
            ).start()

            threading.Thread(
                target=self._watch_map_preview_rviz,
                daemon=True,
            ).start()

        except Exception as error:
            self._append_log(f'[ERROR] 启动地图PCD预览失败: {error}')
            self._stop_loaded_map_preview()
            messagebox.showerror(
                '地图预览失败',
                str(error),
                parent=self.root,
            )

    def _stop_loaded_map_preview(self):
        """关闭GUI启动的地图PCD发布器和预览RViz。"""
        had_process = False

        rviz_process = self.map_preview_rviz_process
        if rviz_process is not None:
            had_process = True
            self._stop_process_group(
                rviz_process,
                '加载地图RViz',
            )
            self.map_preview_rviz_process = None

        pcd_process = self.map_preview_pcd_process
        if pcd_process is not None:
            had_process = True
            self._stop_process_group(
                pcd_process,
                'PCD地图发布器',
            )
            self.map_preview_pcd_process = None

        if had_process:
            self._append_log('[RESULT] 已关闭加载地图PCD预览')

    def _watch_map_preview_pcd(self):
        process = self.map_preview_pcd_process
        if process is None:
            return
        return_code = process.wait()
        self.events.put(('map_preview_pcd_done', return_code))

    def _watch_map_preview_rviz(self):
        process = self.map_preview_rviz_process
        if process is None:
            return
        return_code = process.wait()
        self.events.put(('map_preview_rviz_done', return_code))

    def start_topology_editor(self):
        map_id = self._prompt_map_id('启动离线打点', require_existing=True)
        if map_id is None:
            return
        self._start_topology_editor_process(map_id, online=False)

    def start_online_topology_editor(self):
        if not self.current_map_id:
            messagebox.showwarning(
                '尚未加载地图',
                '请先加载地图并启动重定位。',
                parent=self.root,
            )
            return
        pose_age = time.monotonic() - self.node.last_pose_received_at
        if self.node.last_pose_received_at <= 0.0 or pose_age > 2.0:
            messagebox.showwarning(
                '没有有效定位数据',
                '请先点击“启动重定位”，等待重定位成功并持续收到定位数据。',
                parent=self.root,
            )
            return
        task_id_text = simpledialog.askstring(
            '在线打点任务',
            '请输入 task_id（按K后保存为 task/<task_id>.json）：',
            initialvalue=self.task_id.get().strip() or 'task_0',
            parent=self.root,
        )
        if task_id_text is None:
            return
        try:
            task_id = self._identifier(task_id_text, 'task_id')
        except ValueError as error:
            messagebox.showerror('task_id无效', str(error), parent=self.root)
            return
        self.task_id.set(task_id)
        self._start_topology_editor_process(
            self.current_map_id,
            online=True,
            task_id=task_id,
        )

    def _start_topology_editor_process(self, map_id, online, task_id=''):
        mode_name = '在线打点' if online else '离线打点'

        if self.topology_editor_process is not None:
            if self.topology_editor_process.poll() is None:
                messagebox.showwarning('提示', '离线或在线打点程序已经在运行')
                return
            self.topology_editor_process = None

        map_directory = os.path.join(self.data_root, map_id)
        pcd_file = os.path.join(map_directory, 'map', 'globalmap.pcd')
        if not os.path.isfile(pcd_file):
            messagebox.showerror('地图不存在', f'找不到地图点云:\n{pcd_file}')
            self._append_log(f'[FAIL] 启动{mode_name}: 找不到 {pcd_file}')
            return
        if not os.path.isfile(self.topology_editor_script):
            messagebox.showerror(
                '脚本不存在', f'找不到打点脚本:\n{self.topology_editor_script}'
            )
            self._append_log(
                f'[FAIL] 启动{mode_name}: 找不到 {self.topology_editor_script}'
            )
            return

        online_arguments = ''
        if online:
            online_arguments = (
                ' true '
                f'{self._shell_quote(task_id)}'
            )
        command = (
            f'cd {self._shell_quote(self.workspace_dir)} && '
            f'bash {self._shell_quote(self.topology_editor_script)} '
            f'{self._shell_quote(map_directory)} '
            f'{self._shell_quote(pcd_file)} true{online_arguments}; ret=$?; '
            f"echo ''; echo '[INFO] {mode_name}程序已退出，返回码:' $ret; "
            "echo '[INFO] 请关闭此终端。'; exec bash"
        )
        terminal_command = self._terminal_command(
            f'{mode_name} {map_id} - rviz_topology_editor', command
        )
        if terminal_command is None:
            messagebox.showerror('未找到终端', '系统未找到可用终端。')
            self._append_log(f'[FAIL] 启动{mode_name}: 系统未找到可用终端')
            return

        try:
            self.topology_editor_process = subprocess.Popen(
                terminal_command,
                cwd=self.workspace_dir,
                start_new_session=True,
            )
        except Exception as error:
            messagebox.showerror('启动失败', f'启动{mode_name}失败:\n{error}')
            self._append_log(f'[ERROR] 启动{mode_name}失败: {error}')
            return

        self.topology_editor_online = online
        self.topology_editor_button.configure(state=tk.DISABLED)
        self.online_topology_editor_button.configure(state=tk.DISABLED)
        self._append_log(
            f'[CALL] 启动{mode_name}: map_id={map_id}, pcd={pcd_file}'
            + (f', task_id={task_id}' if online else '')
        )
        threading.Thread(
            target=self._watch_topology_editor, daemon=True
        ).start()

    def _watch_topology_editor(self):
        process = self.topology_editor_process
        if process is None:
            return
        return_code = process.wait()
        self.events.put(('topology_editor_done', return_code))

    def start_keyboard_control(self):
        if self.keyboard_control_process is not None:
            if self.keyboard_control_process.poll() is None:
                messagebox.showwarning('提示', '键盘控制已经在运行')
                return
            self.keyboard_control_process = None

        if not os.path.isfile(self.keyboard_control_script):
            messagebox.showerror(
                '脚本不存在', f'找不到键盘控制脚本:\n{self.keyboard_control_script}'
            )
            self._append_log(
                f'[FAIL] 启动键盘控制: 找不到 {self.keyboard_control_script}'
            )
            return

        runtime_state = self.node.runtime_state
        online_recording = (
            self.topology_editor_online
            and self.topology_editor_process is not None
            and self.topology_editor_process.poll() is None
        )
        if (
            runtime_state is not None
            and runtime_state.state != RuntimeState.MAPPING
            and not online_recording
            and not messagebox.askyesno(
                '确认启动键盘控制',
                '当前Runtime不是MAPPING状态。键盘速度可能与定位或导航输出冲突。\n\n'
                '是否仍然启动？',
            )
        ):
            return

        command = (
            f'cd {self._shell_quote(self.workspace_dir)} && '
            'source /opt/ros/humble/setup.bash && '
            'if [ -f install/setup.bash ]; then source install/setup.bash; fi && '
            f'python3 {self._shell_quote(self.keyboard_control_script)}'
        )
        terminal_command = self._terminal_command(
            '键盘控制 - keyboard_control', command
        )
        if terminal_command is None:
            messagebox.showerror('未找到终端', '系统未找到可用终端。')
            self._append_log('[FAIL] 启动键盘控制: 系统未找到可用终端')
            return

        try:
            self.keyboard_control_process = subprocess.Popen(
                terminal_command,
                cwd=self.workspace_dir,
                start_new_session=True,
            )
        except Exception as error:
            messagebox.showerror('启动失败', f'启动键盘控制失败:\n{error}')
            self._append_log(f'[ERROR] 启动键盘控制失败: {error}')
            return

        self.keyboard_control_button.configure(state=tk.DISABLED)
        self._append_log('[CALL] 启动键盘控制')
        threading.Thread(
            target=self._watch_keyboard_control, daemon=True
        ).start()

    def _watch_keyboard_control(self):
        process = self.keyboard_control_process
        if process is None:
            return
        return_code = process.wait()
        self.events.put(('keyboard_control_done', return_code))

    def check_readiness(self):
        services, actions, workers = self.node.readiness()
        unavailable = [
            f'{kind}:{name}'
            for kind, items in (
                ('service', services),
                ('action', actions),
                ('worker', workers),
            )
            for name, ready in items.items()
            if not ready
        ]
        summary = (
            f'services={sum(services.values())}/{len(services)}, '
            f'actions={sum(actions.values())}/{len(actions)}, '
            f'workers={sum(workers.values())}/{len(workers)}'
        )
        if unavailable:
            self._append_log(
                f'[WARN] 接口检查: {summary}; 未就绪={", ".join(unavailable)}'
            )
        else:
            self._append_log(f'[READY] 接口检查: {summary}')

    def _drain_events(self):
        try:
            while True:
                kind, payload = self.events.get_nowait()
                if kind == 'log':
                    self._append_log(payload)
                elif kind == 'runtime':
                    state = RUNTIME_STATE_NAMES.get(payload.state, str(payload.state))
                    self.runtime_label.set(
                        f'Runtime: {state}  error={payload.error_code} {payload.error_msg}'
                    )
                    if (
                        self.localization_request_active
                        and payload.state == RuntimeState.RELOCALIZING
                    ):
                        self.localization_session_started = True
                        self.localization_status.set('状态：正在启动定位Worker…')
                        localization_worker = self.node.worker_states.get(
                            'fastlio_localization_worker'
                        )
                        if (
                            localization_worker is not None
                            and localization_worker.state == WorkerStatus.RUNNING
                        ):
                            self._notify_localization_worker_ready()
                elif kind == 'worker':
                    online = len(self.node.worker_states)
                    running = sum(
                        message.state == WorkerStatus.RUNNING
                        for message in self.node.worker_states.values()
                    )
                    self.worker_label.set(
                        f'Workers: online={online}/{len(EXPECTED_WORKERS)} '
                        f'running={running}'
                    )
                    if payload.worker_name == 'fastlio_localization_worker':
                        if payload.state == WorkerStatus.STOPPED:
                            self.localization_wait_for_worker_stop = False
                        elif payload.state == WorkerStatus.RUNNING:
                            self._notify_localization_worker_ready()
                elif kind == 'pose':
                    message, _count = payload
                    self.current_pose_vars['map_id'].set(message.map_id or '--')
                    for name in ('x', 'y', 'z', 'theta'):
                        self.current_pose_vars[name].set(
                            f'{getattr(message, name):.2f}'
                        )
                    now = time.monotonic()
                    if now - self.last_pose_log_at >= 5.0:
                        self.last_pose_log_at = now
                        self._append_log(
                            f'[POSE] map_id={message.map_id or "--"} '
                            f'x={message.x:.2f} y={message.y:.2f} '
                            f'z={message.z:.2f} theta={message.theta:.2f} '
                            f'score={message.match_score:.3f}'
                        )
                elif kind == 'relocalization_feedback':
                    self._handle_relocalization_feedback(payload)
                elif kind == 'rviz_goal_pose':
                    x, y, z, theta = payload
                    values = (x, y, z, theta)
                    for name, value in zip(
                        ('x', 'y', 'z', 'theta'), values
                    ):
                        self.navigation_pose_vars[name].set(f'{value:.2f}')
                    self._append_log(
                        '[TARGET] 已从RViz接收2D Goal Pose并填入单点目标位姿: '
                        f'x={x:.2f} y={y:.2f} z={z:.2f} theta={theta:.2f}; '
                        '确认后请点击“发送目标”'
                    )
                elif kind == 'rviz_initial_pose':
                    x, y, z, theta = payload
                    values = (x, y, z, theta)
                    for name, value in zip(
                        ('x', 'y', 'z', 'theta'), values
                    ):
                        self.localization_pose_vars[name].set(f'{value:.2f}')
                    self.initial_pose_enabled.set(True)
                    self._append_log(
                        '[INITIAL_POSE] 已从RViz接收2D Pose Estimate并填入'
                        '重定位初始位姿，已勾选“使用初始位姿”: '
                        f'x={x:.2f} y={y:.2f} z={z:.2f} theta={theta:.2f}'
                    )
                elif kind == 'localization_service_result':
                    success, message = payload
                    self._handle_localization_service_result(success, message)
                elif kind == 'keyframe':
                    _message, count = payload
                    if count == 1 or count % 10 == 0:
                        self._append_log(f'[KEYFRAME] received count={count}')
                elif kind == 'platform_launch_done':
                    self._append_log(
                        f'[RESULT] platform launch 已退出，返回码={payload}'
                    )
                    self.platform_launch_process = None
                    self._close_platform_log_handle()
                elif kind == 'topology_editor_done':
                    self.topology_editor_process = None
                    self.topology_editor_online = False
                    self.topology_editor_button.configure(state=tk.NORMAL)
                    self.online_topology_editor_button.configure(state=tk.NORMAL)
                    self._append_log(
                        f'[RESULT] 地图打点终端已关闭，返回码={payload}'
                    )
                    if self.current_map_id:
                        self.refresh_task_ids()
                elif kind == 'keyboard_control_done':
                    self.keyboard_control_process = None
                    self.keyboard_control_button.configure(state=tk.NORMAL)
                    self._append_log(
                        f'[RESULT] 键盘控制终端已关闭，返回码={payload}'
                    )
                elif kind == 'start_pgo_rviz':
                    self._start_pgo_rviz()
                elif kind == 'stop_pgo_rviz':
                    self._stop_pgo_rviz()
                elif kind == 'pgo_rviz_done':
                    self.pgo_rviz_process = None
                    self._append_log(
                        f'[RESULT] PGO RViz 已退出，返回码={payload}'
                    )
                elif kind == 'load_map_success':
                    self._start_loaded_map_preview(self.current_map_id)
                    self.refresh_task_ids()
                elif kind == 'map_preview_pcd_done':
                    self.map_preview_pcd_process = None
                    self._append_log(
                        f'[RESULT] PCD地图发布器已退出，返回码={payload}'
                    )
                elif kind == 'map_preview_rviz_done':
                    self.map_preview_rviz_process = None
                    self._append_log(
                        f'[RESULT] 加载地图RViz已退出，返回码={payload}'
                    )
                elif kind == 'exit_cleanup_done':
                    return_code, output = payload
                    self._finish_exit_cleanup(return_code, output)
                    return
                elif kind == 'exit_cleanup_timeout':
                    self._append_log('[WARN] 退出前清理超时，继续执行平台进程组兜底清理')
                    for line in payload.splitlines():
                        self._append_log(f'[EXIT_CLEANUP] {line}')
                    self._finalize_exit()
                    return
                elif kind == 'exit_cleanup_error':
                    self._append_log(f'[WARN] 退出前清理失败: {payload}')
                    self._finalize_exit()
                    return
                if kind in ('pose', 'counts', 'keyframe'):
                    self.data_label.set(
                        f'Pose={self.node.pose_count} Cloud={self.node.cloud_count} '
                        f'Keyframe={self.node.keyframe_count}'
                    )
        except queue.Empty:
            pass
        self.root.after(100, self._drain_events)

    def _append_log(self, message):
        now = datetime.datetime.now().strftime('%H:%M:%S')
        line = f'[{now}] {message}'
        print(line, flush=True)
        try:
            with open(self.gui_log_file, 'a', encoding='utf-8') as log_file:
                log_file.write(line + '\n')
        except OSError as error:
            print(f'[WARN] 写入GUI日志失败: {error}', flush=True)

    def delete_all_logs(self):
        """删除日志根目录下的全部历史日志，并恢复当前会话日志目录。"""
        if (
            self.platform_launch_process is not None
            and self.platform_launch_process.poll() is None
        ):
            messagebox.showwarning(
                '无法删除日志',
                '平台 launch 正在运行并写入日志。请先退出GUI并完成节点清理，'
                '重新打开GUI后再删除全部日志。',
                parent=self.root,
            )
            return

        if not messagebox.askyesno(
            '确认删除全部日志',
            f'将永久删除以下目录中的所有日志：\n{self.log_root}\n\n'
            '此操作不可恢复，是否继续？',
            icon='warning',
            parent=self.root,
        ):
            return

        self.delete_all_logs_button.configure(state=tk.DISABLED)
        try:
            self._close_platform_log_handle()
            if os.path.lexists(self.log_root):
                if (
                    os.path.islink(self.log_root)
                    or not os.path.isdir(self.log_root)
                ):
                    raise RuntimeError(f'日志路径不是目录: {self.log_root}')
                shutil.rmtree(self.log_root)

            os.makedirs(self.ros_log_directory, exist_ok=True)
            self._append_log(f'[RESULT] 已删除全部历史日志: {self.log_root}')
            messagebox.showinfo(
                '删除完成',
                'log 目录下的日志已全部删除，当前GUI会话日志已重新创建。',
                parent=self.root,
            )
        except Exception as error:
            try:
                os.makedirs(self.ros_log_directory, exist_ok=True)
            except OSError:
                pass
            self._append_log(f'[ERROR] 删除全部日志失败: {error}')
            messagebox.showerror(
                '删除日志失败',
                str(error),
                parent=self.root,
            )
        finally:
            self.delete_all_logs_button.configure(state=tk.NORMAL)

    def exit_gui(self):
        if self.exiting:
            return
        if not messagebox.askyesno(
            '确认退出GUI',
            '退出GUI清理节点。\n\n是否继续？',
        ):
            return

        self.exiting = True
        self.exit_button.configure(state=tk.DISABLED)
        self.keyboard_control_button.configure(state=tk.DISABLED)
        self.delete_all_logs_button.configure(state=tk.DISABLED)
        self.topology_editor_button.configure(state=tk.DISABLED)
        self.online_topology_editor_button.configure(state=tk.DISABLED)
        self._append_log('[CALL] 退出GUI：先清理全部ROS节点和平台进程')
        threading.Thread(target=self._exit_cleanup_worker, daemon=True).start()

    def _exit_cleanup_worker(self):
        if not os.path.isfile(self.kill_nodes_script):
            self.events.put((
                'exit_cleanup_error',
                f'找不到清理脚本: {self.kill_nodes_script}',
            ))
            return

        try:
            environment = os.environ.copy()
            # 清理脚本不要主动杀当前GUI，由GUI在清理完成后自行退出。
            environment['KILL_NAV_GUI'] = '0'
            result = subprocess.run(
                ['bash', self.kill_nodes_script],
                cwd=self.workspace_dir,
                env=environment,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=60,
            )
            self.events.put((
                'exit_cleanup_done',
                (result.returncode, result.stdout or ''),
            ))
        except subprocess.TimeoutExpired as error:
            output = error.stdout or ''
            if isinstance(output, bytes):
                output = output.decode(errors='replace')
            self.events.put(('exit_cleanup_timeout', output))
        except Exception as error:
            self.events.put(('exit_cleanup_error', str(error)))

    def _finish_exit_cleanup(self, return_code, output):
        for line in output.splitlines():
            self._append_log(f'[EXIT_CLEANUP] {line}')
        if return_code == 0:
            self._append_log('[RESULT] 退出前 run_kill_node.sh 清理完成')
        else:
            self._append_log(
                f'[WARN] 退出前 run_kill_node.sh 返回码={return_code}，继续执行兜底清理'
            )
        self._finalize_exit()

    def _stop_process_group(self, process, name):
        if process is None or process.poll() is not None:
            return
        try:
            pgid = os.getpgid(process.pid)
        except OSError:
            return

        self._append_log(f'[EXIT] 停止{name}进程组 pid={process.pid}')
        try:
            os.killpg(pgid, 2)
            process.wait(timeout=5)
            return
        except Exception:
            pass
        try:
            os.killpg(pgid, 15)
            process.wait(timeout=3)
            return
        except Exception:
            pass
        try:
            os.killpg(pgid, 9)
        except Exception:
            pass

    def _finalize_exit(self):
        # run_kill_node.sh 是主要清理手段；这里再对GUI直接持有的进程组做兜底。
        self._stop_process_group(self.keyboard_control_process, '键盘控制')
        self._stop_process_group(self.topology_editor_process, '地图打点')
        self._stop_process_group(self.pgo_rviz_process, 'PGO RViz')
        self._stop_process_group(self.map_preview_rviz_process, '加载地图RViz')
        self._stop_process_group(self.map_preview_pcd_process, 'PCD地图发布器')
        self._stop_process_group(self.platform_launch_process, 'platform launch')
        self._close_platform_log_handle()

        self._append_log('[RESULT] ROS/平台进程清理结束，退出GUI')
        try:
            self.executor.shutdown(timeout_sec=1.0)
            self.node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as error:
            self._append_log(f'[WARN] GUI ROS资源关闭异常: {error}')
        finally:
            self.root.destroy()

    def close(self):
        # 窗口右上角关闭与“退出GUI”按钮使用完全相同的清理流程。
        self.exit_gui()

    def run(self):
        self.root.mainloop()


if __name__ == '__main__':
    RuntimeTestGui().run()
