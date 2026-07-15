#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""建图 / 地图编辑 / 键盘控制 / 重定位 / 自主导航 GUI。

重定位结果只读取 run_relocalization.sh 写入的：
  relocalization_exit_code.txt
  relocalization_success.txt
不再使用外层终端进程的返回码判断重定位是否成功。
"""

import datetime
import glob
import os
import queue
import shutil
import subprocess
import threading
import time

import tkinter as tk
import tkinter.font as tkfont
from tkinter import messagebox, ttk

import rclpy
from geometry_msgs.msg import Pose
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String


class NavigationWorkflowGUI:
    def __init__(self):
        self.gui_file_path = os.path.realpath(__file__)
        gui_dir = os.path.dirname(self.gui_file_path)
        self.workspace_dir = os.path.dirname(gui_dir)
        self.script_dir = os.path.join(self.workspace_dir, "src")

        self.data_dir = "/robotfs/data/slam_data"
        self.runtime_data_dir = self.data_dir
        self.robot_runtime_dir = os.path.dirname(self.data_dir)

        self.run_kill_node_script = os.path.join(self.script_dir, "run_kill_node.sh")
        self.run_slam_script = os.path.join(self.script_dir, "run_slam.sh")
        self.run_edit_map_script = os.path.join(self.script_dir, "run_edit_map.sh")
        self.keyboard_control_script = os.path.join(
            self.script_dir, "scripts", "keyboard_control.py"
        )
        self.run_relocalization_script = os.path.join(
            self.script_dir, "run_relocalization.sh"
        )
        self.run_nav_script = os.path.join(self.script_dir, "run_nav.sh")

        self.startup_kill_output = ""
        self.startup_kill_returncode = None
        self.startup_tmp_cleanup_output = ""

        self.slam_terminal_process = None
        self.edit_map_terminal_process = None
        self.keyboard_terminal_process = None
        self.relocalization_terminal_process = None
        self.nav_terminal_process = None

        self.slam_running = False
        self.edit_map_running = False
        self.keyboard_running = False
        self.relocalization_running = False
        self.nav_running = False

        self.log_queue = queue.Queue()
        self.terminal_columns = 140
        self.terminal_rows = 36
        self.terminal_font = "Noto Sans Mono CJK SC"
        self.terminal_font_size = 14
        self.nav_control_collapsed = True
        self.gui_log_collapsed = True
        self.nav_control_ros_ready = False
        self.nav_control_node = None
        self.nav_control_executor = None
        self.nav_control_thread = None
        self.nav_task_signal_pub = None
        self.emergency_stop_pub = None
        self.current_target = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.robot_nav_state = False
        self.robot_nav_state_received = False
        self.emergency_stop_state = False
        self.last_target_update = "未接收到目标点"
        self.task_folder_path = os.path.join(self.runtime_data_dir, "task")
        self.map_dir = os.path.join(self.runtime_data_dir, "map")
        self.map_bag_dir = os.path.join(self.map_dir, "result_bag")
        self.map_pcd_file = os.path.join(self.map_dir, "result.pcd")
        self.available_tasks = []

        self.workflow_time = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

        self.root = tk.Tk()
        self.root.withdraw()
        self.selected_task = tk.StringVar()

        self.kill_old_nodes_before_gui_start()
        self.clean_tmp_launch_params_and_runtime_logs_before_gui_start()

        self.workflow_log_dir = os.path.join(
            self.runtime_data_dir, "log", self.workflow_time
        )
        os.makedirs(self.workflow_log_dir, exist_ok=True)

        self.relocalization_result_dir = os.path.join(
            self.workflow_log_dir, "relocalization"
        )
        self.relocalization_exit_code_file = os.path.join(
            self.relocalization_result_dir, "relocalization_exit_code.txt"
        )
        self.relocalization_success_file = os.path.join(
            self.relocalization_result_dir, "relocalization_success.txt"
        )

        self.font_family = self.select_chinese_font()
        self.mono_font_family = self.font_family
        self.configure_tk_fonts()
        self.load_available_tasks()

        self.root.deiconify()
        self.root.title("建图 / 地图编辑 / 键盘控制 / 重定位 / 自主导航控制界面")
        self.root.geometry("820x820")
        self.root.resizable(True, True)
        self.create_widgets()
        self.initialize_nav_control_ros()
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.after(100, self.process_log_queue)
        self.root.after(200, self.update_nav_control_display)
        self.update_map_status_display()
 
        self.add_log("建图 / 编辑地图 / 重定位 / 自主导航 GUI 已启动")
        self.add_log(f"工作空间目录: {self.workspace_dir}")
        self.add_log(f"脚本目录: {self.script_dir}")
        self.add_log(f"运行数据/日志根目录: {self.runtime_data_dir}")
        self.add_log(f"本次任务日志目录: {self.workflow_log_dir}")
        self.add_startup_kill_log_to_gui()

    # ------------------------- startup / cleanup -------------------------
    def _kill_env(self):
        env = os.environ.copy()
        env.update({
            "KILL_RVIZ": "1", "KILL_NAV_GUI": "0", "KILL_BAG": "1",
            "LANG": "C.UTF-8", "LC_ALL": "C.UTF-8", "LANGUAGE": "C.UTF-8",
        })
        return env

    def kill_old_nodes_before_gui_start(self):
        if not os.path.isfile(self.run_kill_node_script):
            self.startup_kill_returncode = -1
            self.startup_kill_output = f"[WARN] 找不到清理脚本: {self.run_kill_node_script}\n"
            return
        try:
            result = subprocess.run(
                ["bash", self.run_kill_node_script],
                cwd=self.workspace_dir, env=self._kill_env(),
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, timeout=25,
            )
            self.startup_kill_returncode = result.returncode
            self.startup_kill_output = result.stdout or ""
        except subprocess.TimeoutExpired as e:
            self.startup_kill_returncode = -2
            output = e.stdout or ""
            if isinstance(output, bytes):
                output = output.decode(errors="ignore")
            self.startup_kill_output = output + "\n[WARN] run_kill_node.sh 执行超时\n"
        except Exception as e:
            self.startup_kill_returncode = -3
            self.startup_kill_output = f"[WARN] 执行 run_kill_node.sh 失败: {e}\n"

    def add_startup_kill_log_to_gui(self):
        if self.startup_tmp_cleanup_output:
            for line in self.startup_tmp_cleanup_output.strip().splitlines()[:80]:
                self.add_log(f"[TMP_CLEAN] {line}")
        if self.startup_kill_returncode == 0:
            self.add_log("GUI 启动前清理完成：旧 ROS2 节点 / launch / rviz2 已尝试清理")
        elif self.startup_kill_returncode is not None:
            self.add_log(f"[WARN] GUI 启动前清理脚本返回码: {self.startup_kill_returncode}")
        for line in self.startup_kill_output.strip().splitlines()[:80]:
            self.add_log(f"[KILL] {line}")

    def kill_old_nodes_before_relocalization(self) -> bool:
        if not os.path.isfile(self.run_kill_node_script):
            messagebox.showerror("清理脚本不存在", f"找不到清理脚本:\n{self.run_kill_node_script}")
            return False
        self.add_log("启动重定位前再次清理旧 ROS2 节点 / launch / rviz2")
        self.set_current_action("启动重定位前正在清理旧 ROS2 节点...", "orange")
        self.root.update_idletasks()
        try:
            result = subprocess.run(
                ["bash", self.run_kill_node_script],
                cwd=self.workspace_dir, env=self._kill_env(),
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, timeout=25,
            )
            for line in (result.stdout or "").strip().splitlines():
                self.add_log(f"[KILL_BEFORE_NAV] {line}")
            if result.returncode == 0:
                self.add_log("启动重定位前清理完成")
                return True
            return messagebox.askyesno(
                "清理异常",
                f"run_kill_node.sh 返回码: {result.returncode}\n\n是否继续启动重定位？",
            )
        except subprocess.TimeoutExpired:
            return messagebox.askyesno(
                "清理超时", "run_kill_node.sh 执行超过 25 秒。\n\n是否继续启动重定位？"
            )
        except Exception as e:
            self.add_log(f"[ERROR] 执行 run_kill_node.sh 失败: {e}")
            messagebox.showerror("清理失败", f"执行 run_kill_node.sh 失败:\n{e}")
            return False

    def clean_tmp_launch_params_and_runtime_logs_before_gui_start(self):
        outputs = []
        def record(message):
            outputs.append(message)

        log_dir = os.path.join(self.runtime_data_dir, "log")
        launch_params_pattern = os.path.join(self.robot_runtime_dir, "launch_params_*")
        try:
            should_clean = messagebox.askyesno(
                "清理旧日志",
                "是否删除旧日志？\n\n会删除：\n"
                f"- {launch_params_pattern}\n- {log_dir}\n\n"
                "不会删除地图、拓扑点和任务文件。",
            )
        except Exception as e:
            should_clean = False
            record(f"[WARN] 弹出清理确认框失败: {e}")

        if not should_clean:
            record("[INFO] 用户选择不删除旧日志")
            self.startup_tmp_cleanup_output = "\n".join(outputs) + "\n"
            return

        try:
            count = 0
            for path in glob.glob(launch_params_pattern):
                if os.path.isfile(path) or os.path.islink(path):
                    os.remove(path)
                    count += 1
            record(f"[INFO] 已删除 ROS2 launch 临时参数文件数量: {count}")
        except Exception as e:
            record(f"[WARN] 清理 launch 临时参数文件失败: {e}")
        try:
            if os.path.isdir(log_dir):
                shutil.rmtree(log_dir)
                record(f"[INFO] 已删除旧运行日志目录: {log_dir}")
        except Exception as e:
            record(f"[WARN] 删除旧运行日志目录失败: {e}")
        self.startup_tmp_cleanup_output = "\n".join(outputs) + "\n"

    # ------------------------- GUI -------------------------
    def select_chinese_font(self):
        available = set(tkfont.families())
        for name in [
            "Noto Sans CJK SC", "WenQuanYi Micro Hei", "WenQuanYi Zen Hei",
            "Microsoft YaHei", "SimHei", "Arial Unicode MS",
        ]:
            if name in available:
                return name
        return "TkDefaultFont"

    def configure_tk_fonts(self):
        try:
            tkfont.nametofont("TkDefaultFont").configure(family=self.font_family, size=10)
            tkfont.nametofont("TkTextFont").configure(family=self.font_family, size=10)
            tkfont.nametofont("TkFixedFont").configure(family=self.mono_font_family, size=9)
        except Exception:
            pass

    def create_widgets(self):
        main = ttk.Frame(self.root, padding="12")
        main.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main.columnconfigure(0, weight=1)
        main.rowconfigure(5, weight=0)

        tk.Label(
            main, text="建图 / 地图编辑 / 键盘控制 / 重定位 / 自主导航控制界面",
            font=(self.font_family, 18, "bold"), fg="navy",
        ).grid(row=0, column=0, pady=(0, 15))

        status = ttk.LabelFrame(main, text="机器人状态", padding="10")
        status.grid(row=1, column=0, sticky=(tk.W, tk.E), pady=(0, 15))
        status.columnconfigure(1, weight=1)
        status.columnconfigure(3, weight=1)
        status.columnconfigure(4, weight=0)
        self.map_status_label = self._compact_status_row(status, 0, 0, "地图状态:", "检查中", "gray")
        self.relocalization_status_label = self._compact_status_row(status, 0, 2, "重定位状态:", "未启动", "gray")
        self.nav_status_label = self._compact_status_row(status, 1, 0, "自主导航状态:", "未启动", "gray")
        self.robot_nav_state_label = self._compact_status_row(status, 1, 2, "导航状态:", "未知", "gray")
        self.robot_target_label = self._compact_status_row(status, 2, 0, "当前目标点:", "等待目标点", "blue")
        self.emergency_status_label = self._compact_status_row(status, 2, 2, "紧急停止:", "关闭", "green")
        self.target_update_label = self._compact_status_row(status, 3, 0, "最后更新:", self.last_target_update, "gray")

        tk.Label(status, text="当前动作:", font=(self.font_family, 11, "bold")).grid(
            row=4, column=0, sticky=tk.W, padx=5, pady=4
        )
        self.current_action_label = tk.Label(
            status, text="等待操作", font=(self.font_family, 11), fg="blue"
        )
        self.current_action_label.grid(row=4, column=1, columnspan=3, sticky=tk.W, padx=5, pady=4)

        tk.Label(status, text="本次运行日志目录:", font=(self.font_family, 11, "bold")).grid(
            row=5, column=0, sticky=tk.W, padx=5, pady=4
        )
        self.log_dir_label = tk.Label(
            status, text=self.workflow_log_dir, font=(self.font_family, 10),
            fg="gray", wraplength=800, justify=tk.LEFT,
        )
        self.log_dir_label.grid(row=5, column=1, columnspan=3, sticky=tk.W, padx=5, pady=4)

        helper_buttons = ttk.Frame(status)
        helper_buttons.grid(row=0, column=4, rowspan=6, sticky=(tk.N, tk.E), padx=(12, 0))
        helper_buttons.columnconfigure(0, weight=1)

        self.start_keyboard_button = tk.Button(
            helper_buttons, text="键盘控制", command=self.start_keyboard_control,
            bg="darkgreen", fg="white", font=(self.font_family, 9, "bold"), height=1,
            width=10,
        )
        self.start_keyboard_button.grid(row=2, column=0, padx=2, pady=2, sticky=(tk.W, tk.E))
        self.kill_nodes_button = tk.Button(
            helper_buttons, text="清理节点", command=self.manual_kill_nodes,
            bg="firebrick", fg="white", font=(self.font_family, 9, "bold"), height=1,
            width=10,
        )
        self.kill_nodes_button.grid(row=1, column=0, padx=2, pady=2, sticky=(tk.W, tk.E))
        self.exit_button = tk.Button(
            helper_buttons, text="退出GUI", command=self.on_closing,
            bg="red", fg="white", font=(self.font_family, 9, "bold"), height=1,
            width=10,
        )
        self.exit_button.grid(row=0, column=0, padx=2, pady=2, sticky=(tk.W, tk.E))

        controls = ttk.LabelFrame(main, text="流程控制", padding="10")
        controls.grid(row=2, column=0, sticky=(tk.W, tk.E), pady=(0, 15))
        for i in range(3):
            controls.columnconfigure(i, weight=1)

        self.start_slam_button = tk.Button(
            controls, text="开始建图并打点", command=self.start_slam,
            bg="green", fg="white", font=(self.font_family, 13, "bold"), height=2,
        )
        self.start_slam_button.grid(row=0, column=0, padx=5, pady=5, sticky=(tk.W, tk.E))
        self.start_edit_map_button = tk.Button(
            controls, text="启动地图编辑", command=self.start_edit_map,
            bg="orange", fg="white", font=(self.font_family, 13, "bold"), height=2,
        )
        self.start_edit_map_button.grid(row=0, column=1, padx=5, pady=5, sticky=(tk.W, tk.E))
        self.start_nav_button = tk.Button(
            controls, text="自主导航", command=self.start_nav,
            bg="purple", fg="white", font=(self.font_family, 13, "bold"), height=2,
        )
        self.start_nav_button.grid(row=0, column=2, padx=5, pady=5, sticky=(tk.W, tk.E))

        self.create_nav_control_panel(main, row=3)

        tips = ttk.LabelFrame(main, text="说明", padding="10")
        tips.grid(row=4, column=0, sticky=(tk.W, tk.E), pady=(0, 15))
        tk.Label(
            tips,
            text=(
                "1. 没有地图时：先点击“开始建图并打点”，完成地图和目标点采集。\n"
                "2. 建图完成后点击“启动地图编辑”，检查并编辑拓扑点、路径和任务。\n"
                "3. 有可用地图和任务后，点击“自主导航”，系统会先启动重定位。"
            ),
            font=(self.font_family, 10), fg="gray", justify=tk.LEFT, anchor=tk.W,
        ).grid(row=0, column=0, sticky=(tk.W, tk.E))

        self.gui_log_frame = ttk.LabelFrame(main, text="GUI 日志", padding="10")
        self.gui_log_frame.grid(row=5, column=0, sticky=(tk.W, tk.E), pady=(0, 0))
        self.gui_log_frame.columnconfigure(0, weight=1)

        self.gui_log_toggle_button = tk.Button(
            self.gui_log_frame, text="展开 GUI 日志", command=self.toggle_gui_log_panel,
            bg="gray", fg="white", font=(self.font_family, 10, "bold"),
        )
        self.gui_log_toggle_button.grid(row=0, column=0, sticky=tk.W)

        self.gui_log_body = ttk.Frame(self.gui_log_frame)
        self.gui_log_body.columnconfigure(0, weight=1)
        self.gui_log_body.rowconfigure(0, weight=1)
        self.log_text = tk.Text(self.gui_log_body, height=14, wrap=tk.WORD,
                                font=(self.mono_font_family, 10))
        scroll = ttk.Scrollbar(self.gui_log_body, orient="vertical", command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scroll.set)
        self.log_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        scroll.grid(row=0, column=1, sticky=(tk.N, tk.S))
        self.status_bar = tk.Label(
            self.root, text="就绪", bd=1, relief=tk.SUNKEN, anchor=tk.W,
            font=(self.font_family, 10),
        )
        self.status_bar.grid(row=1, column=0, sticky=(tk.W, tk.E))

    def _status_row(self, parent, row, name):
        tk.Label(parent, text=name, font=(self.font_family, 11, "bold")).grid(
            row=row, column=0, sticky=tk.W, padx=5, pady=4
        )
        label = tk.Label(parent, text="未启动", font=(self.font_family, 11), fg="gray")
        label.grid(row=row, column=1, sticky=tk.W, padx=5, pady=4)
        return label

    def add_path_row(self, parent, row, name, path):
        tk.Label(parent, text=name, font=(self.font_family, 10, "bold")).grid(
            row=row, column=0, sticky=tk.W, padx=5, pady=3
        )
        tk.Label(
            parent, text=path, font=(self.font_family, 9), fg="gray",
            wraplength=850, justify=tk.LEFT,
        ).grid(row=row, column=1, sticky=tk.W, padx=5, pady=3)

    def toggle_gui_log_panel(self):
        self.gui_log_collapsed = not self.gui_log_collapsed
        if self.gui_log_collapsed:
            self.gui_log_body.grid_remove()
            self.gui_log_frame.grid_configure(sticky=(tk.W, tk.E))
            self.gui_log_toggle_button.config(text="展开 GUI 日志")
            return
        self.gui_log_body.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), pady=(8, 0))
        self.gui_log_frame.grid_configure(sticky=(tk.W, tk.E, tk.N, tk.S))
        self.gui_log_toggle_button.config(text="收起 GUI 日志")

    def create_nav_control_panel(self, parent, row):
        self.nav_control_frame = ttk.LabelFrame(parent, text="导航任务控制", padding="10")
        self.nav_control_frame.grid(row=row, column=0, sticky=(tk.W, tk.E), pady=(0, 15))
        self.nav_control_frame.columnconfigure(0, weight=1)

        header = ttk.Frame(self.nav_control_frame)
        header.grid(row=0, column=0, sticky=(tk.W, tk.E))
        header.columnconfigure(1, weight=1)

        self.nav_control_toggle_button = tk.Button(
            header, text="展开导航任务控制", command=self.toggle_nav_control_panel,
            bg="gray", fg="white", font=(self.font_family, 10, "bold"),
        )
        self.nav_control_toggle_button.grid(row=0, column=0, padx=(0, 8), sticky=tk.W)

        self.nav_control_hint_label = tk.Label(
            header,
            text="未进入自主导航：请先点击“自主导航”，完成重定位确认后再操作导航任务。",
            font=(self.font_family, 10), fg="gray", anchor=tk.W,
        )
        self.nav_control_hint_label.grid(row=0, column=1, sticky=(tk.W, tk.E))

        self.nav_control_body = ttk.Frame(self.nav_control_frame)
        self.nav_control_body.columnconfigure(0, weight=1)

        task_frame = ttk.LabelFrame(self.nav_control_body, text="任务选择", padding="8")
        task_frame.grid(row=0, column=0, sticky=(tk.W, tk.E), pady=(8, 8))
        task_frame.columnconfigure(1, weight=1)
        tk.Label(task_frame, text="选择任务:", font=(self.font_family, 10, "bold")).grid(
            row=0, column=0, sticky=tk.W, padx=5
        )
        self.task_combobox = ttk.Combobox(
            task_frame, textvariable=self.selected_task,
            values=self.available_tasks if self.available_tasks else ["无可用任务"],
            state="readonly", font=(self.font_family, 10),
        )
        self.task_combobox.grid(row=0, column=1, sticky=(tk.W, tk.E), padx=5, pady=4)
        self.refresh_task_button = tk.Button(
            task_frame, text="刷新任务", command=self.refresh_available_tasks,
            bg="gray", fg="white", font=(self.font_family, 10, "bold"),
        )
        self.refresh_task_button.grid(row=0, column=2, sticky=(tk.W, tk.E), padx=5, pady=4)
        tk.Label(task_frame, text="任务目录:", font=(self.font_family, 10, "bold")).grid(
            row=1, column=0, sticky=tk.W, padx=5
        )
        tk.Label(
            task_frame, text=self.task_folder_path, font=(self.font_family, 9),
            fg="gray", wraplength=820, justify=tk.LEFT,
        ).grid(row=1, column=1, columnspan=2, sticky=tk.W, padx=5)

        buttons = ttk.LabelFrame(self.nav_control_body, text="任务操作", padding="8")
        buttons.grid(row=1, column=0, sticky=(tk.W, tk.E))
        for i in range(4):
            buttons.columnconfigure(i, weight=1)

        self.nav_start_button = tk.Button(
            buttons, text="开始导航任务", command=self.start_navigation_task,
            bg="green", fg="white", font=(self.font_family, 11, "bold"), height=2,
        )
        self.nav_start_button.grid(row=0, column=0, padx=5, pady=5, sticky=(tk.W, tk.E))
        self.nav_loop_button = tk.Button(
            buttons, text="循环任务", command=self.loop_navigation_task,
            bg="yellow", fg="black", font=(self.font_family, 11, "bold"), height=2,
        )
        self.nav_loop_button.grid(row=0, column=1, padx=5, pady=5, sticky=(tk.W, tk.E))
        self.nav_home_button = tk.Button(
            buttons, text="回家", command=self.go_home,
            bg="blue", fg="white", font=(self.font_family, 11, "bold"), height=2,
        )
        self.nav_home_button.grid(row=0, column=2, padx=5, pady=5, sticky=(tk.W, tk.E))
        self.emergency_button = tk.Button(
            buttons, text="紧急停止", command=self.toggle_emergency_stop,
            bg="red", fg="white", font=(self.font_family, 11, "bold"), height=2,
        )
        self.emergency_button.grid(row=0, column=3, padx=5, pady=5, sticky=(tk.W, tk.E))
        self.stop_task_button = tk.Button(
            buttons, text="停止任务", command=self.stop_navigation_task,
            bg="orange", fg="white", font=(self.font_family, 10, "bold"),
        )
        self.stop_task_button.grid(row=1, column=0, columnspan=2, padx=5, pady=5, sticky=(tk.W, tk.E))
        self.continue_task_button = tk.Button(
            buttons, text="继续任务", command=self.continue_navigation_task,
            bg="purple", fg="white", font=(self.font_family, 10, "bold"),
        )
        self.continue_task_button.grid(row=1, column=2, columnspan=2, padx=5, pady=5, sticky=(tk.W, tk.E))

        self.set_nav_control_enabled(False)

    def _compact_status_row(self, parent, row, col, name, text, color):
        tk.Label(parent, text=name, font=(self.font_family, 10, "bold")).grid(
            row=row, column=col, sticky=tk.W, padx=5, pady=3
        )
        label = tk.Label(parent, text=text, font=(self.font_family, 10), fg=color)
        label.grid(row=row, column=col + 1, sticky=tk.W, padx=5, pady=3)
        return label

    def toggle_nav_control_panel(self):
        self.nav_control_collapsed = not self.nav_control_collapsed
        if self.nav_control_collapsed:
            self.nav_control_body.grid_remove()
            self.nav_control_toggle_button.config(text="展开导航任务控制")
            return
        self.nav_control_body.grid(row=1, column=0, sticky=(tk.W, tk.E))
        self.nav_control_toggle_button.config(text="收起导航任务控制")

    def expand_nav_control_panel(self):
        if self.nav_control_collapsed:
            self.toggle_nav_control_panel()

    def set_nav_control_enabled(self, enabled):
        state = tk.NORMAL if enabled else tk.DISABLED
        readonly_state = "readonly" if enabled else tk.DISABLED
        for button in [
            self.nav_start_button, self.nav_loop_button, self.nav_home_button,
            self.emergency_button, self.stop_task_button, self.continue_task_button,
        ]:
            button.config(state=state)
        self.task_combobox.config(state=readonly_state)
        self.refresh_task_button.config(state=tk.NORMAL)
        if enabled:
            self.nav_control_hint_label.config(
                text="自主导航运行中：可在此选择任务、急停、回家、停止或继续任务。",
                fg="purple",
            )
        else:
            self.nav_control_hint_label.config(
                text="未进入自主导航：请先点击“自主导航”，完成重定位确认后再操作导航任务。",
                fg="gray",
            )

    def load_available_tasks(self):
        try:
            if not os.path.isdir(self.task_folder_path):
                self.available_tasks = []
                self.selected_task.set("无可用任务")
                return

            self.available_tasks = sorted(
                filename[:-4]
                for filename in os.listdir(self.task_folder_path)
                if filename.endswith(".txt") and
                os.path.isfile(os.path.join(self.task_folder_path, filename))
            )
            self.selected_task.set(
                self.available_tasks[0] if self.available_tasks else "无可用任务"
            )
        except Exception as e:
            self.available_tasks = []
            self.selected_task.set("无可用任务")
            self.add_log_from_thread(f"[WARN] 加载任务列表失败: {e}")

    def refresh_available_tasks(self):
        self.load_available_tasks()
        self.task_combobox.config(
            values=self.available_tasks if self.available_tasks else ["无可用任务"]
        )
        self.add_log(f"任务列表已刷新，数量: {len(self.available_tasks)}")

    def initialize_nav_control_ros(self):
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
            self.nav_control_node = Node("navigation_workflow_control_gui")
            self.nav_task_signal_pub = self.nav_control_node.create_publisher(
                String, "/nav_task_signal", 10
            )
            self.emergency_stop_pub = self.nav_control_node.create_publisher(
                Bool, "/emergency_stop", 10
            )
            self.nav_control_node.create_subscription(
                Pose, "/target_points", self.target_points_callback, 10
            )
            self.nav_control_node.create_subscription(
                Bool, "/nav_state", self.nav_state_callback, 10
            )
            self.nav_control_executor = SingleThreadedExecutor()
            self.nav_control_executor.add_node(self.nav_control_node)
            self.nav_control_thread = threading.Thread(
                target=self.spin_nav_control_ros, daemon=True
            )
            self.nav_control_thread.start()
            self.nav_control_ros_ready = True
            self.add_log("内嵌导航控制 ROS2 topic 已初始化")
        except Exception as e:
            self.nav_control_ros_ready = False
            self.add_log(f"[WARN] 内嵌导航控制 ROS2 初始化失败: {e}")

    def spin_nav_control_ros(self):
        try:
            if self.nav_control_executor is not None:
                self.nav_control_executor.spin()
        except Exception as e:
            self.add_log_from_thread(f"[WARN] 内嵌导航控制 ROS2 spin 退出: {e}")

    def target_points_callback(self, msg):
        self.current_target["x"] = msg.position.x
        self.current_target["y"] = msg.position.y
        self.current_target["z"] = msg.position.z
        self.last_target_update = datetime.datetime.now().strftime("%H:%M:%S")

    def nav_state_callback(self, msg):
        self.robot_nav_state = bool(msg.data)
        self.robot_nav_state_received = True

    def publish_task_signal(self, signal):
        if not self.nav_running:
            messagebox.showwarning("导航未启动", "请先点击“自主导航”，完成重定位并启动导航。")
            return False
        if not self.nav_control_ros_ready or self.nav_task_signal_pub is None:
            messagebox.showwarning("ROS 未就绪", "导航控制 ROS2 publisher 尚未初始化。")
            return False
        msg = String()
        msg.data = signal
        self.nav_task_signal_pub.publish(msg)
        return True

    def start_navigation_task(self):
        task = self.selected_task.get()
        if not task or task == "无可用任务":
            messagebox.showwarning("未选择任务", "请先选择一个任务")
            return
        if self.publish_task_signal(task):
            self.add_log(f"发送开始导航任务: {task}")
            self.set_current_action(f"已发送任务信号: {task}", "green")

    def loop_navigation_task(self):
        task = self.selected_task.get()
        if not task or task == "无可用任务":
            messagebox.showwarning("未选择任务", "请先选择一个任务")
            return
        if self.publish_task_signal(f"LOOP:{task}"):
            self.add_log(f"发送循环导航任务: {task}")
            self.set_current_action(f"已发送循环任务信号: {task}", "green")

    def go_home(self):
        if self.publish_task_signal("GO_HOME"):
            self.add_log("发送回家信号")
            self.set_current_action("已发送回家信号", "blue")

    def stop_navigation_task(self):
        if self.publish_task_signal("STOP"):
            self.add_log("发送停止任务信号")
            self.set_current_action("已发送停止任务信号", "orange")

    def continue_navigation_task(self):
        if self.publish_task_signal("CONTINUE"):
            self.add_log("发送继续任务信号")
            self.set_current_action("已发送继续任务信号", "purple")

    def toggle_emergency_stop(self):
        if not self.nav_running:
            messagebox.showwarning("导航未启动", "请先启动自主导航后再使用急停。")
            return
        if not self.nav_control_ros_ready or self.emergency_stop_pub is None:
            messagebox.showwarning("ROS 未就绪", "急停 ROS2 publisher 尚未初始化。")
            return

        self.emergency_stop_state = not self.emergency_stop_state
        msg = Bool()
        msg.data = self.emergency_stop_state
        self.emergency_stop_pub.publish(msg)

        if self.emergency_stop_state:
            self.emergency_button.config(text="解除停止", bg="darkred")
            self.set_current_action("紧急停止已激活", "red")
            self.add_log("紧急停止: 激活")
        else:
            self.emergency_button.config(text="紧急停止", bg="red")
            self.set_current_action("紧急停止已解除", "green")
            self.add_log("紧急停止: 关闭")

    def update_nav_control_display(self):
        try:
            self.robot_target_label.config(
                text=(
                    f"X: {self.current_target['x']:.3f}, "
                    f"Y: {self.current_target['y']:.3f}, "
                    f"Z: {self.current_target['z']:.3f}"
                )
            )
            if not self.nav_running:
                self.robot_nav_state_label.config(text="未启动", fg="gray")
            elif not self.robot_nav_state_received:
                self.robot_nav_state_label.config(text="等待导航状态", fg="gray")
            elif self.robot_nav_state:
                self.robot_nav_state_label.config(text="已到达目标点", fg="green")
            else:
                self.robot_nav_state_label.config(text="导航中", fg="blue")
            if self.emergency_stop_state:
                self.emergency_status_label.config(text="激活", fg="red")
            else:
                self.emergency_status_label.config(text="关闭", fg="green")
            self.target_update_label.config(text=self.last_target_update)
        except Exception:
            pass
        self.root.after(200, self.update_nav_control_display)

    def add_log(self, message):
        now = datetime.datetime.now().strftime("%H:%M:%S")
        self.log_text.insert(tk.END, f"[{now}] {message}\n")
        self.log_text.see(tk.END)
        if len(self.log_text.get("1.0", tk.END).splitlines()) > 500:
            self.log_text.delete("1.0", "50.0")

    def add_log_from_thread(self, message):
        self.log_queue.put(message)

    def process_log_queue(self):
        try:
            while True:
                self.add_log(self.log_queue.get_nowait())
        except queue.Empty:
            pass
        self.root.after(100, self.process_log_queue)

    def _set(self, label, text, color="gray"):
        label.config(text=text, fg=color)
    def set_slam_status(self, text, color="gray"):
        self.update_map_status_display()
    def set_edit_map_status(self, text, color="gray"):
        return
    def set_keyboard_status(self, text, color="gray"):
        return
    def set_relocalization_status(self, text, color="gray"):
        self._set(self.relocalization_status_label, text, color)
    def set_nav_status(self, text, color="gray"):
        self._set(self.nav_status_label, text, color)
    def set_current_action(self, text, color="blue"):
        self.current_action_label.config(text=text, fg=color)
        self.status_bar.config(text=text)
    def set_slam_button_running(self, running):
        self.start_slam_button.config(state=tk.DISABLED if running else tk.NORMAL)
    def set_edit_map_button_running(self, running):
        self.start_edit_map_button.config(state=tk.DISABLED if running else tk.NORMAL)
    def set_keyboard_button_running(self, running):
        self.start_keyboard_button.config(state=tk.DISABLED if running else tk.NORMAL)
    def set_nav_button_running(self, running):
        self.start_nav_button.config(state=tk.DISABLED if running else tk.NORMAL)

    def has_saved_map(self):
        if os.path.isfile(self.map_pcd_file):
            return True
        if os.path.isdir(self.map_bag_dir):
            return bool(glob.glob(os.path.join(self.map_bag_dir, "*.db3")))
        return False

    def update_map_status_display(self):
        try:
            if self.has_saved_map():
                self._set(self.map_status_label, "有图", "green")
            else:
                self._set(self.map_status_label, "无图", "red")
        except Exception:
            self._set(self.map_status_label, "检查失败", "orange")

    # ------------------------- terminal launcher -------------------------
    def find_terminal(self):
        for cmd in ["xfce4-terminal", "gnome-terminal", "konsole", "x-terminal-emulator", "xterm"]:
            if shutil.which(cmd):
                return cmd
        return None

    @staticmethod
    def shell_quote(text):
        return "'" + text.replace("'", "'\"'\"'") + "'"

    def build_terminal_command(self, title, command):
        terminal = self.find_terminal()
        if terminal is None:
            return None
        geometry = f"{self.terminal_columns}x{self.terminal_rows}"
        command = (
            "export LANG=C.UTF-8; export LC_ALL=C.UTF-8; "
            "export LANGUAGE=C.UTF-8; " + command
        )
        if terminal == "gnome-terminal":
            return [terminal, "--title", title, "--geometry", geometry,
                    "--", "bash", "-lc", command]
        if terminal == "xfce4-terminal":
            return [terminal, "--title", title, "--geometry", geometry,
                    "--font", f"{self.terminal_font} {self.terminal_font_size}",
                    "--command", f"bash -lc {self.shell_quote(command)}"]
        if terminal == "konsole":
            return [terminal, "--new-tab", "-p", f"tabtitle={title}",
                    "-e", "bash", "-lc", command]
        return [terminal, "-T", title, "-geometry", geometry,
                "-fa", self.terminal_font, "-fs", str(self.terminal_font_size),
                "-e", "bash", "-lc", command]

    def _terminal_env(self):
        env = os.environ.copy()
        env.update({
            "WORKFLOW_LOG_DIR": self.workflow_log_dir,
            "LANG": "C.UTF-8", "LC_ALL": "C.UTF-8", "LANGUAGE": "C.UTF-8",
        })
        return env

    def start_script_in_terminal(self, script_path, title, keep_open=True, extra_env=None):
        if not os.path.isfile(script_path):
            messagebox.showerror("脚本不存在", f"找不到脚本:\n{script_path}")
            self.add_log(f"[ERROR] 找不到脚本: {script_path}")
            return None
        command = (
            f"cd {self.shell_quote(self.workspace_dir)} && "
            f"export WORKFLOW_LOG_DIR={self.shell_quote(self.workflow_log_dir)} && "
            "echo '[INFO] 当前目录:' && pwd && "
            "echo '[INFO] WORKFLOW_LOG_DIR='${WORKFLOW_LOG_DIR} && "
            f"echo '[INFO] 执行脚本: {script_path}' && "
        )
        if extra_env:
            for key, value in extra_env.items():
                command += f"export {key}={self.shell_quote(str(value))} && "
        command += f"bash {self.shell_quote(script_path)}; ret=$?"
        if keep_open:
            command += (
                "; echo ''; echo '[INFO] 脚本已退出，返回码:' $ret; "
                "echo '[INFO] 可查看上方日志，关闭此终端即可。'; exec bash"
            )
        else:
            command += "; exit $ret"
        terminal_cmd = self.build_terminal_command(title, command)
        if terminal_cmd is None:
            messagebox.showerror("未找到终端", "系统未找到可用终端。")
            return None
        self.add_log(f"打开终端: {title}")
        self.add_log(f"执行脚本: {script_path}")
        try:
            return subprocess.Popen(
                terminal_cmd, cwd=self.workspace_dir,
                preexec_fn=os.setsid, env=self._terminal_env(),
            )
        except Exception as e:
            messagebox.showerror("启动失败", f"启动脚本失败:\n{e}")
            self.add_log(f"[ERROR] 启动脚本失败: {e}")
            return None

    def start_python_in_terminal(self, python_path, title, keep_open=False):
        if not os.path.isfile(python_path):
            messagebox.showerror("脚本不存在", f"找不到 Python 脚本:\n{python_path}")
            return None
        command = (
            f"cd {self.shell_quote(self.workspace_dir)} && "
            f"export WORKFLOW_LOG_DIR={self.shell_quote(self.workflow_log_dir)} && "
            "source /opt/ros/humble/setup.bash && "
            "if [ -f install/setup.bash ]; then source install/setup.bash; fi && "
            f"python3 {self.shell_quote(python_path)}"
        )
        if keep_open:
            command += "; ret=$?; echo '[INFO] Python 脚本已退出，返回码:' $ret; exec bash"
        terminal_cmd = self.build_terminal_command(title, command)
        if terminal_cmd is None:
            messagebox.showerror("未找到终端", "系统未找到可用终端。")
            return None
        try:
            return subprocess.Popen(
                terminal_cmd, cwd=self.workspace_dir,
                preexec_fn=os.setsid, env=self._terminal_env(),
            )
        except Exception as e:
            messagebox.showerror("启动失败", f"启动 Python 脚本失败:\n{e}")
            self.add_log(f"[ERROR] 启动 Python 脚本失败: {e}")
            return None

    @staticmethod
    def read_int_file(file_path):
        try:
            if not file_path or not os.path.isfile(file_path):
                return None
            with open(file_path, "r", encoding="utf-8") as f:
                value = f.read().strip()
            return int(value) if value else None
        except Exception:
            return None

    # ------------------------- keyboard / slam / edit -------------------------
    def start_keyboard_control(self):
        if self.keyboard_running:
            messagebox.showwarning("提示", "键盘控制已经在运行")
            return
        self.keyboard_terminal_process = self.start_python_in_terminal(
            self.keyboard_control_script, "键盘控制 - keyboard_control", keep_open=False
        )
        if self.keyboard_terminal_process is None:
            return
        self.keyboard_running = True
        self.set_keyboard_status("键盘控制运行中", "blue")
        self.set_current_action("键盘控制已启动，请在弹出的终端中操作机器人", "blue")
        self.set_keyboard_button_running(True)
        threading.Thread(target=self.watch_keyboard_control_process, daemon=True).start()

    def watch_keyboard_control_process(self):
        if self.keyboard_terminal_process is None:
            return
        ret = self.keyboard_terminal_process.wait()
        self.add_log_from_thread(f"keyboard_control.py 终端进程退出，返回码: {ret}")
        self.root.after(0, self._keyboard_finished)

    def _keyboard_finished(self):
        self.keyboard_running = False
        self.set_keyboard_status("键盘控制已退出", "green")
        self.set_current_action("键盘控制流程结束", "green")
        self.set_keyboard_button_running(False)

    def start_slam(self):
        if self.slam_running:
            messagebox.showwarning("提示", "建图已经在运行")
            return
        self.slam_terminal_process = self.start_script_in_terminal(
            self.run_slam_script, "SLAM 建图 - keyboard_control", keep_open=False
        )
        if self.slam_terminal_process is None:
            return
        self.slam_running = True
        self.set_slam_status("建图运行中", "blue")
        self.set_current_action("建图中，请在弹出的终端中使用键盘控制机器人", "blue")
        self.set_slam_button_running(True)
        threading.Thread(target=self.watch_slam_process, daemon=True).start()

    def watch_slam_process(self):
        if self.slam_terminal_process is None:
            return
        ret = self.slam_terminal_process.wait()
        self.add_log_from_thread(f"run_slam.sh 终端进程退出，返回码: {ret}")
        self.root.after(0, self._slam_finished)

    def _slam_finished(self):
        self.slam_running = False
        self.set_slam_status("建图已退出", "gray")
        self.set_current_action("建图终端已退出", "gray")
        self.set_slam_button_running(False)

    def start_edit_map(self):
        if self.edit_map_running:
            messagebox.showwarning("提示", "地图编辑已经在运行")
            return
        self.update_map_status_display()
        self.edit_map_terminal_process = self.start_script_in_terminal(
            self.run_edit_map_script, "地图编辑 - run_edit_map", keep_open=False
        )
        if self.edit_map_terminal_process is None:
            return
        self.edit_map_running = True
        self.set_edit_map_status("地图编辑运行中", "orange")
        self.set_current_action("地图编辑已启动，请在弹出的终端 / RViz 中编辑地图", "orange")
        self.set_edit_map_button_running(True)
        threading.Thread(target=self.watch_edit_map_process, daemon=True).start()

    def watch_edit_map_process(self):
        if self.edit_map_terminal_process is None:
            return
        ret = self.edit_map_terminal_process.wait()
        self.add_log_from_thread(f"run_edit_map.sh 终端进程退出，返回码: {ret}")
        self.root.after(0, self._edit_map_finished)

    def _edit_map_finished(self):
        self.edit_map_running = False
        self.set_edit_map_status("地图编辑已退出", "green")
        self.update_map_status_display()
        self.set_current_action("地图编辑流程结束", "green")
        self.set_edit_map_button_running(False)

    # ------------------------- relocalization + nav -------------------------
    def start_nav(self):
        if self.nav_running or self.relocalization_running:
            messagebox.showwarning("提示", "自主导航或重定位流程已经在运行")
            return
        self.update_map_status_display()
        if not messagebox.askyesno(
            "启动自主导航",
            "清理旧节点并重定位。\n成功后需要确认位置，再启动导航。\n\n是否继续？",
        ):
            return
        if not self.kill_old_nodes_before_relocalization():
            self.set_relocalization_status("清理失败，未启动重定位", "red")
            self.set_nav_status("未启动", "gray")
            self.set_current_action("清理旧节点失败或取消，未启动自主导航", "red")
            self.set_nav_button_running(False)
            return

        try:
            os.makedirs(self.relocalization_result_dir, exist_ok=True)
            for path in [self.relocalization_exit_code_file, self.relocalization_success_file]:
                if os.path.isfile(path):
                    os.remove(path)
        except Exception as e:
            self.add_log(f"[WARN] 删除旧重定位结果文件失败: {e}")

        # 关键：这里不再把 exit_code_file 传给外层终端包装器。
        # 结果文件只能由 run_relocalization.sh 自己写。
        self.relocalization_terminal_process = self.start_script_in_terminal(
            self.run_relocalization_script,
            "重定位 - run_relocalization",
            keep_open=False,
        )
        if self.relocalization_terminal_process is None:
            return
        self.relocalization_running = True
        self.set_relocalization_status("正在重定位", "orange")
        self.set_nav_status("等待重定位成功", "orange")
        self.set_current_action("正在执行重定位，成功后需要人工确认位置", "orange")
        self.set_nav_button_running(True)
        self.add_log("已启动 run_relocalization.sh，等待重定位成功")
        threading.Thread(target=self.watch_relocalization_then_start_nav, daemon=True).start()

    def watch_relocalization_then_start_nav(self):
        """只读结果文件，不根据终端启动器返回码判定重定位。"""
        timeout_sec = 200.0
        interval_sec = 0.5
        start = time.time()
        last_progress = -10.0
        real_return_code = None
        success_flag = None
        terminal_return_code = None
        self.add_log_from_thread(
            "开始等待重定位结果文件：relocalization_exit_code.txt / relocalization_success.txt"
        )

        while time.time() - start < timeout_sec:
            if self.relocalization_terminal_process is not None:
                terminal_return_code = self.relocalization_terminal_process.poll()
            real_return_code = self.read_int_file(self.relocalization_exit_code_file)
            success_flag = self.read_int_file(self.relocalization_success_file)
            if real_return_code is not None and success_flag is not None:
                break
            elapsed = time.time() - start
            if elapsed - last_progress >= 10.0:
                last_progress = elapsed
                self.add_log_from_thread(
                    f"仍在等待重定位结果... 已等待 {elapsed:.0f}s / {timeout_sec:.0f}s，"
                    f"exit_code={real_return_code}，success={success_flag}，"
                    f"终端启动器状态={terminal_return_code}"
                )
            time.sleep(interval_sec)

        self.relocalization_running = False
        self.add_log_from_thread(
            f"重定位结果检查完成：真实返回码={real_return_code}，"
            f"成功标志={success_flag}，终端启动器返回码={terminal_return_code}"
        )

        if real_return_code is None or success_flag is None:
            self.root.after(0, lambda: self._relocalization_timeout(timeout_sec))
            return
        if real_return_code != 0 or success_flag != 1:
            self.root.after(0, lambda: self._relocalization_failed(real_return_code, success_flag))
            return
        self.root.after(0, self.ask_user_confirm_pose)

    def _relocalization_timeout(self, timeout_sec):
        self.set_relocalization_status("重定位超时", "red")
        self.set_nav_status("未启动", "gray")
        self.set_current_action("未等到重定位最终结果，未启动自主导航", "red")
        self.set_nav_button_running(False)
        messagebox.showerror(
            "重定位超时",
            f"等待重定位结果超过 {timeout_sec:.0f} 秒。\n\n"
            f"请查看目录：\n{self.relocalization_result_dir}",
        )

    def _relocalization_failed(self, exit_code, success_flag):
        self.set_relocalization_status("重定位失败", "red")
        self.set_nav_status("未启动", "gray")
        self.set_current_action("重定位失败，未启动自主导航", "red")
        self.set_nav_button_running(False)
        self.add_log(f"重定位失败：exit_code={exit_code}，success_flag={success_flag}")
        messagebox.showerror(
            "重定位失败",
            "run_relocalization.sh 已结束，但未返回成功结果。\n\n"
            f"真实返回码: {exit_code}\n成功标志: {success_flag}\n\n"
            "请查看 relocalization 日志。",
        )

    def ask_user_confirm_pose(self):
        self.set_relocalization_status("重定位成功，等待人工确认", "orange")
        self.set_nav_status("等待用户确认位置", "orange")
        self.set_current_action("重定位成功，请确认当前位置和朝向是否正确", "orange")
        self.add_log("重定位成功，请在 RViz 中确认当前位置和朝向是否正确")
        if messagebox.askyesno(
            "确认重定位结果",
            "请在 RViz 中确认当前位置和朝向。\n\n是否正确并启动导航？",
        ):
            self.set_relocalization_status("重定位已确认", "green")
            self.set_nav_status("正在启动自主导航", "green")
            self.set_current_action("用户确认位置正确，正在启动 run_nav.sh", "green")
            self.add_log("用户确认重定位位置正确，开始启动 run_nav.sh")
            self.start_nav_after_relocalization_success()
            return
        self.set_relocalization_status("重定位成功但未确认", "orange")
        self.set_nav_status("未启动", "gray")
        self.set_current_action("用户未确认重定位结果，已取消启动自主导航", "red")
        self.set_nav_button_running(False)
        self.add_log("用户未确认重定位结果，取消启动 run_nav.sh")

    def start_nav_after_relocalization_success(self):
        if self.nav_running:
            return
        self.robot_nav_state = False
        self.robot_nav_state_received = False
        self.nav_terminal_process = self.start_script_in_terminal(
            self.run_nav_script,
            "自主导航 - run_nav",
            keep_open=False,
        )
        if self.nav_terminal_process is None:
            self.set_nav_status("自主导航启动失败", "red")
            self.set_current_action("run_nav.sh 启动失败", "red")
            self.set_nav_button_running(False)
            return
        self.nav_running = True
        self.set_nav_status("自主导航运行中", "purple")
        self.set_current_action("自主导航已启动，请在主界面导航任务控制区操作", "purple")
        self.set_nav_button_running(True)
        self.refresh_available_tasks()
        self.expand_nav_control_panel()
        self.set_nav_control_enabled(True)
        self.add_log("已启动 run_nav.sh，导航任务控制区已启用")
        threading.Thread(target=self.watch_nav_process, daemon=True).start()

    def watch_nav_process(self):
        if self.nav_terminal_process is None:
            return
        ret = self.nav_terminal_process.wait()
        self.add_log_from_thread(f"run_nav.sh 终端进程退出，返回码: {ret}")
        self.root.after(0, self._nav_finished)

    def _nav_finished(self):
        self.nav_running = False
        self.robot_nav_state = False
        self.robot_nav_state_received = False
        self.set_nav_status("自主导航已退出", "green")
        self.set_current_action("自主导航流程结束", "green")
        self.set_nav_button_running(False)
        self.emergency_stop_state = False
        self.emergency_button.config(text="紧急停止", bg="red")
        self.set_nav_control_enabled(False)

    # ------------------------- manual cleanup / exit -------------------------
    def manual_kill_nodes(self):
        if not os.path.isfile(self.run_kill_node_script):
            messagebox.showerror("清理脚本不存在", f"找不到清理脚本:\n{self.run_kill_node_script}")
            return
        text = (
            "当前仍有任务运行。\n清理会中断建图/重定位/导航。\n\n是否继续？"
            if self.has_running_processes()
            else "将清理旧 ROS2 节点、RViz、bag。\n\n是否继续？"
        )
        if not messagebox.askyesno("确认清理节点", text):
            return
        self.kill_nodes_button.config(state=tk.DISABLED)
        self.set_current_action("正在清理 ROS2 节点...", "orange")
        threading.Thread(target=self._manual_kill_nodes_worker, daemon=True).start()

    def _manual_kill_nodes_worker(self):
        try:
            result = subprocess.run(
                ["bash", self.run_kill_node_script],
                cwd=self.workspace_dir, env=self._kill_env(),
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                text=True, timeout=60,
            )
            if result.returncode == 0:
                self.root.after(0, self._manual_kill_done)
            else:
                self.root.after(0, lambda: self._manual_kill_failed(result.returncode))
        except subprocess.TimeoutExpired:
            self.root.after(0, self._manual_kill_timeout)
        except Exception as e:
            self.root.after(0, lambda: self._manual_kill_exception(e))

    def _manual_kill_done(self):
        self.slam_running = self.edit_map_running = self.keyboard_running = False
        self.relocalization_running = self.nav_running = False
        self.robot_nav_state = False
        self.robot_nav_state_received = False
        self.set_slam_status("已清理/未启动", "gray")
        self.set_edit_map_status("已清理/未启动", "gray")
        self.set_keyboard_status("已清理/未启动", "gray")
        self.set_relocalization_status("已清理/未启动", "gray")
        self.set_nav_status("已清理/未启动", "gray")
        self.set_slam_button_running(False)
        self.set_edit_map_button_running(False)
        self.set_keyboard_button_running(False)
        self.set_nav_button_running(False)
        self.emergency_stop_state = False
        self.emergency_button.config(text="紧急停止", bg="red")
        self.set_nav_control_enabled(False)
        self.kill_nodes_button.config(state=tk.NORMAL)
        self.set_current_action("节点清理完成", "green")
        messagebox.showinfo("清理完成", "run_kill_node.sh 已执行完成。")

    def _manual_kill_failed(self, code):
        self.kill_nodes_button.config(state=tk.NORMAL)
        self.set_current_action("清理脚本返回异常", "red")
        messagebox.showwarning("清理脚本返回异常", f"run_kill_node.sh 返回码: {code}")

    def _manual_kill_timeout(self):
        self.kill_nodes_button.config(state=tk.NORMAL)
        self.set_current_action("清理脚本执行超时", "red")
        messagebox.showwarning("清理超时", "run_kill_node.sh 执行超过 60 秒。")

    def _manual_kill_exception(self, error):
        self.kill_nodes_button.config(state=tk.NORMAL)
        self.set_current_action("清理节点失败", "red")
        messagebox.showerror("清理失败", f"执行 run_kill_node.sh 失败:\n{error}")

    def has_running_processes(self):
        return any([
            self.slam_running, self.edit_map_running, self.keyboard_running,
            self.relocalization_running, self.nav_running,
        ])

    def on_closing(self):
        if self.has_running_processes() and not messagebox.askyesno(
            "确认退出",
            "当前仍有任务运行。\n退出 GUI 不会关闭已打开的终端。\n\n确定退出？",
        ):
            return
        self.shutdown_nav_control_ros()
        self.root.destroy()

    def shutdown_nav_control_ros(self):
        try:
            if self.nav_control_executor is not None:
                self.nav_control_executor.shutdown()
        except Exception:
            pass
        try:
            if self.nav_control_node is not None:
                self.nav_control_node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

    def run(self):
        self.root.mainloop()


def main():
    NavigationWorkflowGUI().run()


if __name__ == "__main__":
    main()
