#!/usr/bin/env python3
"""robot_runtime APP-facing integration test GUI."""

import queue
import re
import threading
import tkinter as tk
from tkinter import messagebox, scrolledtext, ttk

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger

from robot_interfaces.action import NavigateToPose, StartNavigation
from robot_interfaces.msg import MapKeyframe, NavigationNode, RobotPose
from robot_interfaces.msg import RuntimeState, WorkerStatus
from robot_interfaces.srv import DeleteMap, LoadMap, StartLocalization, StartMapping


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


class RuntimeTestNode(Node):
    def __init__(self, events):
        super().__init__('robot_runtime_test_gui')
        self.events = events
        self.worker_states = {}
        self.runtime_state = None
        self.pose_count = 0
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
        self.active_goal_handle = None

        self.create_subscription(
            RuntimeState, '/robot/runtime_state', self._runtime_state_cb, 10
        )
        self.create_subscription(
            WorkerStatus, '/robot_runtime/worker_status', self._worker_status_cb, 50
        )
        self.create_subscription(RobotPose, '/robot/current_pose', self._pose_cb, 20)
        self.create_subscription(
            PointCloud2, '/robot/cloud_body', self._cloud_cb, 20
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
        self.events.put(('pose', (message, self.pose_count)))

    def _cloud_cb(self, _message):
        self.cloud_count += 1
        if self.cloud_count % 10 == 1:
            self.events.put(('counts', None))

    def _keyframe_cb(self, message):
        self.keyframe_count += 1
        self.events.put(('keyframe', (message, self.keyframe_count)))

    def call_service(self, name, request):
        client = self.service_clients[name]
        if not client.service_is_ready():
            self.events.put(('log', f'[FAIL] {name}: service is not ready'))
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
        except Exception as error:
            self.events.put(('log', f'[ERROR] {name}: {error}'))

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
        self.events = queue.Queue()
        rclpy.init()
        self.node = RuntimeTestNode(self.events)
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

        self.root = tk.Tk()
        self.root.title('robot_runtime APP 接口测试')
        self.root.geometry('980x820')
        self.root.protocol('WM_DELETE_WINDOW', self.close)

        self.map_id = tk.StringVar(value='map_0')
        self.initial_pose_enabled = tk.BooleanVar(value=False)
        self.x = tk.StringVar(value='0.0')
        self.y = tk.StringVar(value='0.0')
        self.z = tk.StringVar(value='0.0')
        self.theta = tk.StringVar(value='0.0')
        self.task_id = tk.StringVar(value='task_0')
        self.patrol_loop = tk.BooleanVar(value=False)
        self.runtime_label = tk.StringVar(value='Runtime: 未连接')
        self.worker_label = tk.StringVar(
            value=f'Workers: 0/{len(EXPECTED_WORKERS)}'
        )
        self.data_label = tk.StringVar(value='Pose=0  Cloud=0  Keyframe=0')

        self._create_widgets()
        self.root.after(100, self._drain_events)

    def _create_widgets(self):
        root = ttk.Frame(self.root, padding=10)
        root.pack(fill=tk.BOTH, expand=True)

        status = ttk.LabelFrame(root, text='连接与状态', padding=8)
        status.pack(fill=tk.X)
        ttk.Label(status, textvariable=self.runtime_label).grid(row=0, column=0, sticky='w')
        ttk.Label(status, textvariable=self.worker_label).grid(row=0, column=1, padx=20)
        ttk.Label(status, textvariable=self.data_label).grid(row=0, column=2, padx=20)
        ttk.Button(status, text='检查全部接口', command=self.check_readiness).grid(
            row=0, column=3, padx=8
        )

        common = ttk.LabelFrame(root, text='公共参数', padding=8)
        common.pack(fill=tk.X, pady=8)
        ttk.Label(common, text='map_id').grid(row=0, column=0)
        ttk.Entry(common, textvariable=self.map_id, width=10).grid(row=0, column=1)
        for column, (name, variable) in enumerate(
            [('x', self.x), ('y', self.y), ('z', self.z), ('theta(rad)', self.theta)],
            start=2,
        ):
            ttk.Label(common, text=name).grid(row=0, column=column * 2 - 2, padx=(12, 2))
            ttk.Entry(common, textvariable=variable, width=10).grid(
                row=0, column=column * 2 - 1
            )

        controls = ttk.Frame(root)
        controls.pack(fill=tk.X)

        mapping = ttk.LabelFrame(controls, text='1. 建图', padding=8)
        mapping.grid(row=0, column=0, sticky='nsew', padx=(0, 4))
        ttk.Button(mapping, text='开始建图', command=self.start_mapping).pack(fill=tk.X)
        ttk.Button(mapping, text='完成并保存', command=self.finish_mapping).pack(
            fill=tk.X, pady=4
        )

        maps = ttk.LabelFrame(controls, text='2. 地图', padding=8)
        maps.grid(row=0, column=1, sticky='nsew', padx=4)
        ttk.Button(maps, text='加载地图', command=self.load_map).pack(fill=tk.X)
        ttk.Button(maps, text='删除地图', command=self.delete_map).pack(fill=tk.X, pady=4)

        localization = ttk.LabelFrame(controls, text='3. 定位', padding=8)
        localization.grid(row=0, column=2, sticky='nsew', padx=4)
        ttk.Checkbutton(
            localization, text='使用初始位姿', variable=self.initial_pose_enabled
        ).pack(anchor='w')
        ttk.Button(
            localization, text='启动重定位', command=self.start_localization
        ).pack(fill=tk.X, pady=4)

        navigation = ttk.LabelFrame(controls, text='4. 单点导航', padding=8)
        navigation.grid(row=0, column=3, sticky='nsew', padx=(4, 0))
        ttk.Button(navigation, text='发送目标', command=self.navigate_to_pose).pack(
            fill=tk.X
        )
        ttk.Button(navigation, text='取消目标', command=self.node.cancel_goal).pack(
            fill=tk.X, pady=4
        )
        pause_row = ttk.Frame(navigation)
        pause_row.pack(fill=tk.X)
        ttk.Button(pause_row, text='暂停', command=self.pause_navigation).pack(
            side=tk.LEFT, expand=True, fill=tk.X
        )
        ttk.Button(pause_row, text='恢复', command=self.resume_navigation).pack(
            side=tk.LEFT, expand=True, fill=tk.X
        )

        for column in range(4):
            controls.columnconfigure(column, weight=1)

        patrol = ttk.LabelFrame(root, text='5. 巡检导航（每行：id,x,y,z,theta）', padding=8)
        patrol.pack(fill=tk.X, pady=8)
        patrol_top = ttk.Frame(patrol)
        patrol_top.pack(fill=tk.X)
        ttk.Label(patrol_top, text='task_id').pack(side=tk.LEFT)
        ttk.Entry(patrol_top, textvariable=self.task_id, width=8).pack(side=tk.LEFT, padx=4)
        ttk.Checkbutton(patrol_top, text='循环', variable=self.patrol_loop).pack(side=tk.LEFT)
        ttk.Button(patrol_top, text='发送巡检任务', command=self.start_patrol).pack(
            side=tk.RIGHT
        )
        self.patrol_nodes = tk.Text(patrol, height=4)
        self.patrol_nodes.pack(fill=tk.X, pady=(6, 0))
        self.patrol_nodes.insert('1.0', '1,0.0,0.0,0.0,0.0\n2,2.0,0.0,0.0,0.0')

        log_frame = ttk.LabelFrame(root, text='APP 调用与返回日志', padding=8)
        log_frame.pack(fill=tk.BOTH, expand=True)
        self.log = scrolledtext.ScrolledText(log_frame, height=22, state=tk.DISABLED)
        self.log.pack(fill=tk.BOTH, expand=True)

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

    def _pose_values(self):
        return tuple(float(item.get()) for item in (self.x, self.y, self.z, self.theta))

    def start_mapping(self):
        try:
            request = StartMapping.Request()
            request.map_id = self._identifier(
                self.map_id.get(), 'map_id', required_prefix='map_'
            )
            self.node.call_service('start_mapping', request)
        except ValueError as error:
            messagebox.showerror('参数错误', str(error))

    def finish_mapping(self):
        self.node.call_service('finish_mapping', Trigger.Request())

    def load_map(self):
        try:
            request = LoadMap.Request()
            request.map_id = self._identifier(
                self.map_id.get(), 'map_id', required_prefix='map_'
            )
            self.node.call_service('load_map', request)
        except ValueError as error:
            messagebox.showerror('参数错误', str(error))

    def delete_map(self):
        try:
            request = DeleteMap.Request()
            request.map_id = self._identifier(
                self.map_id.get(), 'map_id', required_prefix='map_'
            )
            self.node.call_service('delete_map', request)
        except ValueError as error:
            messagebox.showerror('参数错误', str(error))

    def start_localization(self):
        try:
            x, y, z, theta = self._pose_values()
            request = StartLocalization.Request()
            request.use_initial_pose = self.initial_pose_enabled.get()
            request.x, request.y, request.z, request.theta = x, y, z, theta
            self.node.call_service('start_localization', request)
        except ValueError as error:
            messagebox.showerror('参数错误', str(error))

    def navigate_to_pose(self):
        try:
            goal = NavigateToPose.Goal()
            goal.x, goal.y, goal.z, goal.theta = self._pose_values()
            self.node.send_pose_goal(goal)
        except ValueError as error:
            messagebox.showerror('参数错误', str(error))

    def start_patrol(self):
        try:
            goal = StartNavigation.Goal()
            goal.task_id = self._identifier(self.task_id.get(), 'task_id')
            goal.loop = self.patrol_loop.get()
            for line in self.patrol_nodes.get('1.0', tk.END).splitlines():
                if not line.strip():
                    continue
                fields = [field.strip() for field in line.split(',')]
                if len(fields) != 5:
                    raise ValueError(f'巡检节点格式错误: {line}')
                node = NavigationNode()
                node.node_id = self._uint(fields[0], 'node_id')
                node.x, node.y, node.z, node.theta = map(float, fields[1:])
                goal.nodes.append(node)
            if not goal.nodes:
                raise ValueError('至少输入一个巡检节点')
            self.node.send_patrol_goal(goal)
        except ValueError as error:
            messagebox.showerror('参数错误', str(error))

    def pause_navigation(self):
        self.node.call_service('pause_navigation', Trigger.Request())

    def resume_navigation(self):
        self.node.call_service('resume_navigation', Trigger.Request())

    def check_readiness(self):
        services, actions, workers = self.node.readiness()
        for name, ready in services.items():
            self._append_log(f'[READY] service {name}: {ready}')
        for name, ready in actions.items():
            self._append_log(f'[READY] action {name}: {ready}')
        for name, ready in workers.items():
            self._append_log(f'[READY] worker {name}: {ready}')

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
                elif kind == 'pose':
                    message, _count = payload
                    self._append_log(
                        f'[POSE] x={message.x:.3f} y={message.y:.3f} '
                        f'z={message.z:.3f} theta={message.theta:.3f} '
                        f'score={message.match_score:.3f}'
                    )
                elif kind == 'keyframe':
                    _message, count = payload
                    self._append_log(f'[KEYFRAME] received count={count}')
                if kind in ('pose', 'counts', 'keyframe'):
                    self.data_label.set(
                        f'Pose={self.node.pose_count} Cloud={self.node.cloud_count} '
                        f'Keyframe={self.node.keyframe_count}'
                    )
        except queue.Empty:
            pass
        self.root.after(100, self._drain_events)

    def _append_log(self, message):
        self.log.configure(state=tk.NORMAL)
        self.log.insert(tk.END, message + '\n')
        self.log.see(tk.END)
        self.log.configure(state=tk.DISABLED)

    def close(self):
        try:
            self.executor.shutdown(timeout_sec=1.0)
            self.node.destroy_node()
            rclpy.shutdown()
        finally:
            self.root.destroy()

    def run(self):
        self.root.mainloop()


if __name__ == '__main__':
    RuntimeTestGui().run()
