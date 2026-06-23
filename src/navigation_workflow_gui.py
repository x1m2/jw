#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
建图 / 地图编辑 / 键盘控制 / 重定位 / 自主导航 GUI

功能：
1. 启动一次 GUI，创建一个统一日志目录：
   /robotfs/data/robot_data/log/YYYYMMDD_HHMMSS

2. 开始建图：
   - 新终端运行 run_slam.sh
   - 本界面只负责启动 run_slam.sh，不提供停止 run_slam.sh 的操作

3. 启动地图编辑：
   - 新终端运行 run_edit_map.sh

4. 单独启动键盘控制：
   - 新终端运行 keyboard_control.py

5. 自主导航：
   - 点击自主导航后先再次执行 run_kill_node.sh 清理旧 ROS2 节点
   - 再新终端运行 run_relocalization.sh
   - run_relocalization.sh 重定位成功后返回 0
   - GUI 检测返回码为 0 后，弹窗让用户确认当前位置是否正确
   - 用户确认位置正确后，才启动 run_nav.sh
   - 用户不确认，则取消启动 run_nav.sh

"""

import os
import glob
import shutil
import subprocess
import threading
import datetime
import queue

import tkinter as tk
from tkinter import ttk, messagebox
import tkinter.font as tkfont


class NavigationWorkflowGUI:
    def __init__(self):
        self.gui_file_path = os.path.realpath(__file__)
        self.script_dir = os.path.dirname(self.gui_file_path)
        self.workspace_dir = os.path.dirname(self.script_dir)
        self.script_dir = os.path.join(self.workspace_dir, "src")

        # GUI 每次启动时先执行该脚本，清理上一次残留的 ROS2 节点 / launch / rviz2
        self.run_kill_node_script = os.path.join(self.script_dir, "run_kill_node.sh")
        self.startup_kill_output = ""
        self.startup_kill_returncode = None
        self.kill_old_nodes_before_gui_start()

        self.run_slam_script = os.path.join(self.script_dir, "run_slam.sh")
        self.run_edit_map_script = os.path.join(self.script_dir, "run_edit_map.sh")
        self.keyboard_control_script = os.path.join(
            self.script_dir,
            "scripts",
            "keyboard_control.py",
        )
        self.run_relocalization_script = os.path.join(
            self.script_dir,
            "run_relocalization.sh",
        )
        self.run_nav_script = os.path.join(self.script_dir, "run_nav.sh")

        self.workflow_time = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

        self.runtime_robot_data_dir = os.environ.get(
            "ROBOT_DATA_RUNTIME_DIR",
            "/robotfs/data/robot_data",
        )

        # 先创建 Tk root，后面才能弹 messagebox。
        # 先隐藏主窗口，避免确认框弹出时背后出现半初始化界面。
        self.root = tk.Tk()
        self.root.withdraw()

        # GUI 每次启动时弹窗询问是否清理 ROS2 launch 临时参数文件和旧日志。
        # Yes：删除 
        # No：不删除
        # 不管 Yes / No，后续流程都继续。
        self.startup_tmp_cleanup_output = ""
        self.clean_tmp_launch_params_and_runtime_logs_before_gui_start()

        self.workflow_log_dir = os.path.join(
            self.runtime_robot_data_dir,
            "log",
            self.workflow_time,
        )
        os.makedirs(self.workflow_log_dir, exist_ok=True)

        # 重定位真实结果文件。
        # 不能只看终端进程返回码，因为 xfce4-terminal / gnome-terminal
        # 的返回码不一定等于 run_relocalization.sh 的真实返回码。
        self.relocalization_result_dir = os.path.join(
            self.workflow_log_dir,
            "relocalization",
        )
        self.relocalization_exit_code_file = os.path.join(
            self.relocalization_result_dir,
            "relocalization_exit_code.txt",
        )
        self.relocalization_success_file = os.path.join(
            self.relocalization_result_dir,
            "relocalization_success.txt",
        )

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

        self.font_family = self.select_chinese_font()
        self.mono_font_family = self.font_family
        self.configure_tk_fonts()

        self.root.deiconify()
        self.root.title("建图 / 地图编辑 / 键盘控制 / 重定位 / 自主导航控制界面")
        self.root.geometry("1050x1200")
        self.root.resizable(True, True)

        self.create_widgets()

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.after(100, self.process_log_queue)

        self.add_log("建图 / 编辑地图 / 重定位 / 自主导航 GUI 已启动")
        self.add_log(f"工作空间目录: {self.workspace_dir}")
        self.add_log(f"脚本目录: {self.script_dir}")
        self.add_log(f"运行数据/日志根目录: {self.runtime_robot_data_dir}")
        self.add_log(f"本次任务日志目录: {self.workflow_log_dir}")
        self.add_log(f"run_kill_node.sh: {self.run_kill_node_script}")
        self.add_startup_kill_log_to_gui()
        self.add_log(f"run_slam.sh: {self.run_slam_script}")
        self.add_log(f"run_edit_map.sh: {self.run_edit_map_script}")
        self.add_log(f"keyboard_control.py: {self.keyboard_control_script}")
        self.add_log(f"run_relocalization.sh: {self.run_relocalization_script}")
        self.add_log(f"run_nav.sh: {self.run_nav_script}")

    def kill_old_nodes_before_gui_start(self):
        """
        GUI 每次启动时先执行 run_kill_node.sh。
        目的：清理上一次残留的 ROS2 节点、launch 进程、rviz2、ros2 bag 等。
        注意：这里在 Tk 界面创建之前执行，所以日志先保存到变量，界面创建后再写入 GUI 日志。
        """
        if not os.path.isfile(self.run_kill_node_script):
            self.startup_kill_returncode = -1
            self.startup_kill_output = (
                f"[WARN] 找不到清理脚本: {self.run_kill_node_script}\n"
            )
            print(self.startup_kill_output)
            return

        print("==================================================")
        print("[INFO] GUI 启动前清理旧 ROS2 节点和 RViz")
        print("==================================================")
        print(f"[INFO] 执行清理脚本: {self.run_kill_node_script}")

        env = os.environ.copy()
        env["KILL_RVIZ"] = "1"
        env["KILL_NAV_GUI"] = "0"
        env["KILL_BAG"] = "1"
        env["LANG"] = "C.UTF-8"
        env["LC_ALL"] = "C.UTF-8"
        env["LANGUAGE"] = "C.UTF-8"

        try:
            result = subprocess.run(
                ["bash", self.run_kill_node_script],
                cwd=self.workspace_dir,
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=25,
            )

            self.startup_kill_returncode = result.returncode
            self.startup_kill_output = result.stdout or ""

            print(self.startup_kill_output)

            if result.returncode == 0:
                print("[INFO] GUI 启动前清理完成")
            else:
                print(f"[WARN] 清理脚本返回码: {result.returncode}")

        except subprocess.TimeoutExpired as e:
            self.startup_kill_returncode = -2
            partial_output = ""
            if e.stdout:
                partial_output += e.stdout if isinstance(e.stdout, str) else e.stdout.decode(errors="ignore")
            if e.stderr:
                partial_output += e.stderr if isinstance(e.stderr, str) else e.stderr.decode(errors="ignore")
            self.startup_kill_output = (
                partial_output
                + "\n[WARN] run_kill_node.sh 执行超时，继续启动 GUI\n"
            )
            print(self.startup_kill_output)

        except Exception as e:
            self.startup_kill_returncode = -3
            self.startup_kill_output = f"[WARN] 执行 run_kill_node.sh 失败: {e}\n"
            print(self.startup_kill_output)

    def add_startup_kill_log_to_gui(self):
        """把 GUI 创建前执行的清理结果写入 GUI 日志。"""
        if getattr(self, "startup_tmp_cleanup_output", ""):
            lines = self.startup_tmp_cleanup_output.strip().splitlines()
            for line in lines[:80]:
                self.add_log(f"[TMP_CLEAN] {line}")
            if len(lines) > 80:
                self.add_log(f"[TMP_CLEAN] ... 输出较多，已省略 {len(lines) - 80} 行")

        if self.startup_kill_returncode == 0:
            self.add_log("GUI 启动前清理完成：旧 ROS2 节点 / launch / rviz2 已尝试清理")
        elif self.startup_kill_returncode is None:
            self.add_log("GUI 启动前未执行清理脚本")
        else:
            self.add_log(f"[WARN] GUI 启动前清理脚本返回码: {self.startup_kill_returncode}")

        if self.startup_kill_output:
            lines = self.startup_kill_output.strip().splitlines()
            max_lines = 80
            for line in lines[:max_lines]:
                self.add_log(f"[KILL] {line}")
            if len(lines) > max_lines:
                self.add_log(f"[KILL] ... 输出较多，已省略 {len(lines) - max_lines} 行")

    def kill_old_nodes_before_relocalization(self) -> bool:
        """
        点击“自主导航”后，在启动 run_relocalization.sh 前再次执行 run_kill_node.sh。

        作用：
        1. 清理 GUI 启动后残留或上一次导航/重定位失败后留下的 ROS2 节点。
        2. 避免旧的 localization_sc、pcd2pgm、导航节点、rviz2、ros2 bag 等影响本次重定位。
        3. 把完整清理输出写入 GUI 日志，方便现场排查。
        """
        if not os.path.isfile(self.run_kill_node_script):
            self.add_log(f"[WARN] 找不到清理脚本: {self.run_kill_node_script}")
            messagebox.showerror(
                "清理脚本不存在",
                f"找不到清理脚本:\n{self.run_kill_node_script}\n\n"
                "已取消启动重定位和自主导航。",
            )
            return False

        self.add_log("==================================================")
        self.add_log("[INFO] 点击自主导航：启动重定位前再次清理旧 ROS2 节点 / launch / rviz2")
        self.add_log("==================================================")
        self.add_log(f"[INFO] 执行清理脚本: {self.run_kill_node_script}")

        self.set_current_action("启动重定位前正在清理旧 ROS2 节点...", "orange")
        self.root.update_idletasks()

        env = os.environ.copy()
        env["KILL_RVIZ"] = "1"
        env["KILL_NAV_GUI"] = "0"
        env["KILL_BAG"] = "1"
        env["LANG"] = "C.UTF-8"
        env["LC_ALL"] = "C.UTF-8"
        env["LANGUAGE"] = "C.UTF-8"

        try:
            result = subprocess.run(
                ["bash", self.run_kill_node_script],
                cwd=self.workspace_dir,
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=25,
            )

            output = result.stdout or ""

            if output:
                for line in output.strip().splitlines():
                    self.add_log(f"[KILL_BEFORE_NAV] {line}")

            if result.returncode == 0:
                self.add_log("[INFO] 启动重定位前清理完成")
                return True

            self.add_log(f"[WARN] 清理脚本返回码: {result.returncode}")
            return messagebox.askyesno(
                "清理异常",
                f"run_kill_node.sh 返回码: {result.returncode}\n\n"
                "是否继续启动重定位？",
            )

        except subprocess.TimeoutExpired as e:
            partial_output = ""
            if e.stdout:
                partial_output += e.stdout if isinstance(e.stdout, str) else e.stdout.decode(errors="ignore")
            if e.stderr:
                partial_output += e.stderr if isinstance(e.stderr, str) else e.stderr.decode(errors="ignore")

            if partial_output:
                for line in partial_output.strip().splitlines():
                    self.add_log(f"[KILL_BEFORE_NAV] {line}")

            self.add_log("[WARN] run_kill_node.sh 执行超时")
            return messagebox.askyesno(
                "清理超时",
                "run_kill_node.sh 执行超过 25 秒。\n\n"
                "是否继续启动重定位？",
            )

        except Exception as e:
            self.add_log(f"[ERROR] 执行 run_kill_node.sh 失败: {e}")
            messagebox.showerror(
                "清理失败",
                f"执行 run_kill_node.sh 失败:\n{e}\n\n"
                "已取消启动重定位和自主导航。",
            )
            return False

    def clean_tmp_launch_params_and_runtime_logs_before_gui_start(self):
        """
        GUI 每次启动时弹窗询问是否清理：
        1. /robotfs/data/launch_params_*：ROS2 launch 临时参数文件。
        2. /robotfs/data/robot_data/log：上一次运行产生的运行日志。

        """
        outputs = []

        def record(msg: str):
            outputs.append(msg)
            print(msg)

        record("==================================================")
        record("[INFO] GUI 启动前询问是否清理 ROS2 launch 临时参数文件和旧日志")
        record("==================================================")

        log_dir = os.path.join(self.runtime_robot_data_dir, "log")

        try:
            should_clean = messagebox.askyesno(
                "清理旧日志",
                "是否删除旧日志？\n\n"
                "会删除：\n"
                "- /robotfs/data/launch_params_*\n"
                f"- {log_dir}\n\n"
                "不会删除地图、拓扑点和任务文件。\n\n"
                "选择“是”删除，选择“否”保留。",
            )
        except Exception as e:
            should_clean = False
            record(f"[WARN] 弹出清理确认框失败，默认不删除旧日志: {e}")

        if not should_clean:
            record("[INFO] 用户选择不删除旧日志，跳过 /robotfs/data/launch_params_* 和运行日志清理")
            self.startup_tmp_cleanup_output = "\n".join(outputs) + "\n"
            return

        record("[INFO] 用户选择删除旧日志，开始清理")

        # 1. 清理 /robotfs/data/launch_params_*
        try:
            launch_param_files = glob.glob("/robotfs/data/launch_params_*")
            removed_count = 0

            for file_path in launch_param_files:
                try:
                    if os.path.isfile(file_path) or os.path.islink(file_path):
                        os.remove(file_path)
                        removed_count += 1
                except Exception as e:
                    record(f"[WARN] 删除失败: {file_path}, {e}")

            record(f"[INFO] 已删除 ROS2 launch 临时参数文件数量: {removed_count}")

        except Exception as e:
            record(f"[WARN] 清理 /robotfs/data/launch_params_* 失败: {e}")

        # 2./log
        try:
            if os.path.isdir(log_dir):
                shutil.rmtree(log_dir)
                record(f"[INFO] 已删除旧运行日志目录: {log_dir}")
            else:
                record(f"[INFO] 旧运行日志目录不存在，无需删除: {log_dir}")

        except Exception as e:
            record(f"[WARN] 删除旧运行日志目录失败: {log_dir}, {e}")

        self.startup_tmp_cleanup_output = "\n".join(outputs) + "\n"

    def select_chinese_font(self):
        available_fonts = set(tkfont.families())

        preferred_fonts = [
            "Noto Sans CJK SC",
            "WenQuanYi Micro Hei",
            "WenQuanYi Zen Hei",
            "Microsoft YaHei",
            "SimHei",
            "Arial Unicode MS",
        ]

        for font_name in preferred_fonts:
            if font_name in available_fonts:
                return font_name

        return "TkDefaultFont"

    def configure_tk_fonts(self):
        try:
            default_font = tkfont.nametofont("TkDefaultFont")
            default_font.configure(family=self.font_family, size=10)

            text_font = tkfont.nametofont("TkTextFont")
            text_font.configure(family=self.font_family, size=10)

            fixed_font = tkfont.nametofont("TkFixedFont")
            fixed_font.configure(family=self.mono_font_family, size=9)

            menu_font = tkfont.nametofont("TkMenuFont")
            menu_font.configure(family=self.font_family, size=10)

            heading_font = tkfont.nametofont("TkHeadingFont")
            heading_font.configure(family=self.font_family, size=10)

            self.root.option_add("*Font", default_font)

        except Exception:
            pass

    def create_widgets(self):
        main_frame = ttk.Frame(self.root, padding="12")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        main_frame.columnconfigure(0, weight=1)
        main_frame.rowconfigure(5, weight=1)

        title_label = tk.Label(
            main_frame,
            text="建图 / 地图编辑 / 键盘控制 / 重定位 / 自主导航控制界面",
            font=(self.font_family, 18, "bold"),
            fg="navy",
        )
        title_label.grid(row=0, column=0, pady=(0, 15))

        status_frame = ttk.LabelFrame(main_frame, text="当前状态", padding="10")
        status_frame.grid(row=1, column=0, sticky=(tk.W, tk.E), pady=(0, 15))
        status_frame.columnconfigure(1, weight=1)

        tk.Label(
            status_frame,
            text="建图状态:",
            font=(self.font_family, 11, "bold"),
        ).grid(row=0, column=0, sticky=tk.W, padx=5, pady=4)

        self.slam_status_label = tk.Label(
            status_frame,
            text="未启动",
            font=(self.font_family, 11),
            fg="gray",
        )
        self.slam_status_label.grid(row=0, column=1, sticky=tk.W, padx=5, pady=4)

        tk.Label(
            status_frame,
            text="地图编辑状态:",
            font=(self.font_family, 11, "bold"),
        ).grid(row=1, column=0, sticky=tk.W, padx=5, pady=4)

        self.edit_map_status_label = tk.Label(
            status_frame,
            text="未启动",
            font=(self.font_family, 11),
            fg="gray",
        )
        self.edit_map_status_label.grid(row=1, column=1, sticky=tk.W, padx=5, pady=4)

        tk.Label(
            status_frame,
            text="键盘控制状态:",
            font=(self.font_family, 11, "bold"),
        ).grid(row=2, column=0, sticky=tk.W, padx=5, pady=4)

        self.keyboard_status_label = tk.Label(
            status_frame,
            text="未启动",
            font=(self.font_family, 11),
            fg="gray",
        )
        self.keyboard_status_label.grid(row=2, column=1, sticky=tk.W, padx=5, pady=4)

        tk.Label(
            status_frame,
            text="重定位状态:",
            font=(self.font_family, 11, "bold"),
        ).grid(row=3, column=0, sticky=tk.W, padx=5, pady=4)

        self.relocalization_status_label = tk.Label(
            status_frame,
            text="未启动",
            font=(self.font_family, 11),
            fg="gray",
        )
        self.relocalization_status_label.grid(row=3, column=1, sticky=tk.W, padx=5, pady=4)

        tk.Label(
            status_frame,
            text="自主导航状态:",
            font=(self.font_family, 11, "bold"),
        ).grid(row=4, column=0, sticky=tk.W, padx=5, pady=4)

        self.nav_status_label = tk.Label(
            status_frame,
            text="未启动",
            font=(self.font_family, 11),
            fg="gray",
        )
        self.nav_status_label.grid(row=4, column=1, sticky=tk.W, padx=5, pady=4)

        tk.Label(
            status_frame,
            text="当前动作:",
            font=(self.font_family, 11, "bold"),
        ).grid(row=5, column=0, sticky=tk.W, padx=5, pady=4)

        self.current_action_label = tk.Label(
            status_frame,
            text="等待操作",
            font=(self.font_family, 11),
            fg="blue",
        )
        self.current_action_label.grid(row=5, column=1, sticky=tk.W, padx=5, pady=4)

        tk.Label(
            status_frame,
            text="本次运行日志目录:",
            font=(self.font_family, 11, "bold"),
        ).grid(row=6, column=0, sticky=tk.W, padx=5, pady=4)

        self.log_dir_label = tk.Label(
            status_frame,
            text=self.workflow_log_dir,
            font=(self.font_family, 10),
            fg="gray",
            wraplength=800,
            justify=tk.LEFT,
        )
        self.log_dir_label.grid(row=6, column=1, sticky=tk.W, padx=5, pady=4)

        path_frame = ttk.LabelFrame(main_frame, text="脚本路径", padding="10")
        path_frame.grid(row=2, column=0, sticky=(tk.W, tk.E), pady=(0, 15))
        path_frame.columnconfigure(1, weight=1)

        self.add_path_row(path_frame, 0, "run_kill_node.sh:", self.run_kill_node_script)
        self.add_path_row(path_frame, 1, "run_slam.sh:", self.run_slam_script)
        self.add_path_row(path_frame, 2, "run_edit_map.sh:", self.run_edit_map_script)
        self.add_path_row(path_frame, 3, "keyboard_control.py:", self.keyboard_control_script)
        self.add_path_row(path_frame, 4, "run_relocalization.sh:", self.run_relocalization_script)
        self.add_path_row(path_frame, 5, "run_nav.sh:", self.run_nav_script)

        control_frame = ttk.LabelFrame(main_frame, text="流程控制", padding="10")
        control_frame.grid(row=3, column=0, sticky=(tk.W, tk.E), pady=(0, 15))

        for i in range(3):
            control_frame.columnconfigure(i, weight=1)

        self.start_slam_button = tk.Button(
            control_frame,
            text="开始建图并打点",
            command=self.start_slam,
            bg="green",
            fg="white",
            font=(self.font_family, 13, "bold"),
            height=2,
            relief=tk.RAISED,
        )
        self.start_slam_button.grid(
            row=0, column=0, padx=5, pady=5, sticky=(tk.W, tk.E)
        )

        self.start_edit_map_button = tk.Button(
            control_frame,
            text="启动地图编辑",
            command=self.start_edit_map,
            bg="orange",
            fg="white",
            font=(self.font_family, 13, "bold"),
            height=2,
            relief=tk.RAISED,
        )
        self.start_edit_map_button.grid(
            row=0, column=1, padx=5, pady=5, sticky=(tk.W, tk.E)
        )

        self.start_nav_button = tk.Button(
            control_frame,
            text="自主导航",
            command=self.start_nav,
            bg="purple",
            fg="white",
            font=(self.font_family, 13, "bold"),
            height=2,
            relief=tk.RAISED,
        )
        self.start_nav_button.grid(
            row=0, column=2, padx=5, pady=5, sticky=(tk.W, tk.E)
        )

        self.start_keyboard_button = tk.Button(
            control_frame,
            text="单独启动键盘控制",
            command=self.start_keyboard_control,
            bg="darkgreen",
            fg="white",
            font=(self.font_family, 10, "bold"),
            height=1,
            relief=tk.RAISED,
        )
        self.start_keyboard_button.grid(
            row=1, column=0, padx=5, pady=5, sticky=(tk.W, tk.E)
        )

        self.kill_nodes_button = tk.Button(
            control_frame,
            text="一键清理节点",
            command=self.manual_kill_nodes,
            bg="firebrick",
            fg="white",
            font=(self.font_family, 10, "bold"),
            height=1,
            relief=tk.RAISED,
        )
        self.kill_nodes_button.grid(
            row=1, column=1, padx=5, pady=5, sticky=(tk.W, tk.E)
        )

        self.exit_button = tk.Button(
            control_frame,
            text="退出GUI",
            command=self.on_closing,
            bg="red",
            fg="white",
            font=(self.font_family, 10, "bold"),
            height=1,
            relief=tk.RAISED,
        )
        self.exit_button.grid(
            row=1, column=2, padx=5, pady=5, sticky=(tk.W, tk.E)
        )

        tips_frame = ttk.LabelFrame(main_frame, text="说明", padding="10")
        tips_frame.grid(row=4, column=0, sticky=(tk.W, tk.E), pady=(0, 15))

        tips_text = (
            "1. 启动 GUI 会自动清理旧 ROS2 节点，并弹窗询问是否删除 /robotfs/data/launch_params_* 和 /robotfs/data/log。\n"
            "2. 建图、地图编辑、键盘控制、自主导航都会在新终端中运行。\n"
            "3. 自主导航会先重定位，确认位置正确后再启动导航。\n"
            "4. 一键清理节点用于停止残留 ROS2 节点、RViz、bag。"
        )

        tk.Label(
            tips_frame,
            text=tips_text,
            font=(self.font_family, 10),
            fg="gray",
            justify=tk.LEFT,
            anchor=tk.W,
        ).grid(row=0, column=0, sticky=(tk.W, tk.E))

        log_frame = ttk.LabelFrame(main_frame, text="GUI 日志", padding="10")
        log_frame.grid(row=5, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)

        self.log_text = tk.Text(
            log_frame,
            height=18,
            wrap=tk.WORD,
            font=(self.mono_font_family, 10),
        )

        scrollbar = ttk.Scrollbar(log_frame, orient="vertical", command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scrollbar.set)

        self.log_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))

        self.status_bar = tk.Label(
            self.root,
            text="就绪",
            bd=1,
            relief=tk.SUNKEN,
            anchor=tk.W,
            font=(self.font_family, 10),
        )
        self.status_bar.grid(row=1, column=0, sticky=(tk.W, tk.E))

    def add_path_row(self, parent, row, name, path):
        tk.Label(
            parent,
            text=name,
            font=(self.font_family, 10, "bold"),
        ).grid(row=row, column=0, sticky=tk.W, padx=5, pady=3)

        tk.Label(
            parent,
            text=path,
            font=(self.font_family, 9),
            fg="gray",
            wraplength=850,
            justify=tk.LEFT,
        ).grid(row=row, column=1, sticky=tk.W, padx=5, pady=3)

    def add_log(self, message: str):
        timestamp = datetime.datetime.now().strftime("%H:%M:%S")
        self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.log_text.see(tk.END)

        lines = self.log_text.get("1.0", tk.END).split("\n")
        if len(lines) > 500:
            self.log_text.delete("1.0", "50.0")

    def add_log_from_thread(self, message: str):
        self.log_queue.put(message)

    def process_log_queue(self):
        try:
            while True:
                msg = self.log_queue.get_nowait()
                self.add_log(msg)
        except queue.Empty:
            pass

        self.root.after(100, self.process_log_queue)

    def set_slam_status(self, text: str, color: str = "gray"):
        self.slam_status_label.config(text=text, fg=color)

    def set_edit_map_status(self, text: str, color: str = "gray"):
        self.edit_map_status_label.config(text=text, fg=color)

    def set_keyboard_status(self, text: str, color: str = "gray"):
        self.keyboard_status_label.config(text=text, fg=color)


    def set_relocalization_status(self, text: str, color: str = "gray"):
        self.relocalization_status_label.config(text=text, fg=color)

    def set_nav_status(self, text: str, color: str = "gray"):
        self.nav_status_label.config(text=text, fg=color)

    def set_current_action(self, text: str, color: str = "blue"):
        self.current_action_label.config(text=text, fg=color)
        self.status_bar.config(text=text)
    def set_slam_button_running(self, running: bool):
        self.start_slam_button.config(state=tk.DISABLED if running else tk.NORMAL)

    def set_edit_map_button_running(self, running: bool):
        self.start_edit_map_button.config(state=tk.DISABLED if running else tk.NORMAL)

    def set_keyboard_button_running(self, running: bool):
        self.start_keyboard_button.config(state=tk.DISABLED if running else tk.NORMAL)

    def set_nav_button_running(self, running: bool):
        self.start_nav_button.config(state=tk.DISABLED if running else tk.NORMAL)

    def find_terminal(self):
        candidates = [
            "xfce4-terminal",
            "gnome-terminal",
            "konsole",
            "x-terminal-emulator",
            "xterm",
        ]

        for cmd in candidates:
            if shutil.which(cmd):
                return cmd

        return None

    def shell_quote(self, text: str) -> str:
        return "'" + text.replace("'", "'\"'\"'") + "'"

    def build_terminal_command(self, title: str, command: str):
        terminal = self.find_terminal()

        if terminal is None:
            return None

        geometry = f"{self.terminal_columns}x{self.terminal_rows}"
        font_name = self.terminal_font
        font_size = str(self.terminal_font_size)

        command = (
            "export LANG=C.UTF-8; "
            "export LC_ALL=C.UTF-8; "
            "export LANGUAGE=C.UTF-8; "
            + command
        )

        if terminal == "gnome-terminal":
            return [
                terminal,
                "--title",
                title,
                "--geometry",
                geometry,
                "--",
                "bash",
                "-lc",
                command,
            ]

        if terminal == "xfce4-terminal":
            return [
                terminal,
                "--title",
                title,
                "--geometry",
                geometry,
                "--font",
                f"{font_name} {font_size}",
                "--command",
                f"bash -lc {self.shell_quote(command)}",
            ]

        if terminal == "konsole":
            return [
                terminal,
                "--new-tab",
                "-p",
                f"tabtitle={title}",
                "-e",
                "bash",
                "-lc",
                command,
            ]

        return [
            terminal,
            "-T",
            title,
            "-geometry",
            geometry,
            "-fa",
            font_name,
            "-fs",
            font_size,
            "-e",
            "bash",
            "-lc",
            command,
        ]

    def start_script_in_terminal(
        self,
        script_path: str,
        title: str,
        keep_open: bool = True,
        exit_code_file: str = None,
    ):
        if not os.path.isfile(script_path):
            messagebox.showerror("脚本不存在", f"找不到脚本:\n{script_path}")
            self.add_log(f"[ERROR] 找不到脚本: {script_path}")
            return None

        command = (
            f"cd {self.shell_quote(self.workspace_dir)} && "
            f"export WORKFLOW_LOG_DIR={self.shell_quote(self.workflow_log_dir)} && "
            f"export ROBOT_DATA_DIR={self.shell_quote(self.runtime_robot_data_dir)} && "
            f"echo '[INFO] 当前目录:' && pwd && "
            f"echo '[INFO] WORKFLOW_LOG_DIR='${{WORKFLOW_LOG_DIR}} && "
            f"echo '[INFO] ROBOT_DATA_DIR='${{ROBOT_DATA_DIR}} && "
            f"echo '[INFO] 执行脚本: {script_path}' && "
            f"bash {self.shell_quote(script_path)}; "
            "ret=$?"
        )

        if exit_code_file:
            exit_code_dir = os.path.dirname(exit_code_file)
            command += (
                f"; mkdir -p {self.shell_quote(exit_code_dir)}"
                f"; echo $ret > {self.shell_quote(exit_code_file)}"
            )

        if keep_open:
            command += (
                "; echo ''; "
                "echo '[INFO] 脚本已退出，返回码:' $ret; "
                "echo '[INFO] 可查看上方日志，关闭此终端即可。'; "
                "exec bash"
            )
        else:
            command += "; exit $ret"

        terminal_cmd = self.build_terminal_command(title, command)

        if terminal_cmd is None:
            messagebox.showerror(
                "未找到终端",
                "系统未找到可用终端，请安装 xterm、gnome-terminal 或 xfce4-terminal。",
            )
            self.add_log("[ERROR] 未找到可用终端")
            return None

        env = os.environ.copy()
        env["WORKFLOW_LOG_DIR"] = self.workflow_log_dir
        env["ROBOT_DATA_DIR"] = self.runtime_robot_data_dir
        env["LANG"] = "C.UTF-8"
        env["LC_ALL"] = "C.UTF-8"
        env["LANGUAGE"] = "C.UTF-8"

        self.add_log(f"打开终端: {title}")
        self.add_log(f"执行脚本: {script_path}")
        self.add_log(f"传递日志目录: {self.workflow_log_dir}")

        try:
            process = subprocess.Popen(
                terminal_cmd,
                cwd=self.workspace_dir,
                preexec_fn=os.setsid,
                env=env,
            )
            return process

        except Exception as e:
            messagebox.showerror("启动失败", f"启动脚本失败:\n{e}")
            self.add_log(f"[ERROR] 启动脚本失败: {e}")
            return None

    def read_int_file(self, file_path: str):
        try:
            if not file_path or not os.path.isfile(file_path):
                return None

            with open(file_path, "r", encoding="utf-8") as f:
                value = f.read().strip()

            if value == "":
                return None

            return int(value)

        except Exception as e:
            self.add_log_from_thread(f"[WARN] 读取文件失败: {file_path}, {e}")
            return None

    def start_python_in_terminal(self, python_path: str, title: str, keep_open: bool = False):
        if not os.path.isfile(python_path):
            messagebox.showerror("脚本不存在", f"找不到 Python 脚本:\n{python_path}")
            self.add_log(f"[ERROR] 找不到 Python 脚本: {python_path}")
            return None

        command = (
            f"cd {self.shell_quote(self.workspace_dir)} && "
            f"export WORKFLOW_LOG_DIR={self.shell_quote(self.workflow_log_dir)} && "
            f"export ROBOT_DATA_DIR={self.shell_quote(self.runtime_robot_data_dir)} && "
            f"echo '[INFO] 当前目录:' && pwd && "
            f"echo '[INFO] WORKFLOW_LOG_DIR='${{WORKFLOW_LOG_DIR}} && "
            f"echo '[INFO] ROBOT_DATA_DIR='${{ROBOT_DATA_DIR}} && "
            f"echo '[INFO] 启动键盘控制: {python_path}' && "
            f"source /opt/ros/humble/setup.bash && "
            f"if [ -f install/setup.bash ]; then source install/setup.bash; fi && "
            f"python3 {self.shell_quote(python_path)}"
        )

        if keep_open:
            command += (
                "; ret=$?; "
                "echo ''; "
                "echo '[INFO] Python 脚本已退出，返回码:' $ret; "
                "echo '[INFO] 可查看上方终端输出，关闭此终端即可。'; "
                "exec bash"
            )

        terminal_cmd = self.build_terminal_command(title, command)

        if terminal_cmd is None:
            messagebox.showerror(
                "未找到终端",
                "系统未找到可用终端，请安装 xterm、gnome-terminal 或 xfce4-terminal。",
            )
            self.add_log("[ERROR] 未找到可用终端")
            return None

        env = os.environ.copy()
        env["WORKFLOW_LOG_DIR"] = self.workflow_log_dir
        env["ROBOT_DATA_DIR"] = self.runtime_robot_data_dir
        env["LANG"] = "C.UTF-8"
        env["LC_ALL"] = "C.UTF-8"
        env["LANGUAGE"] = "C.UTF-8"

        self.add_log(f"打开终端: {title}")
        self.add_log(f"执行 Python 脚本: {python_path}")
        self.add_log("键盘控制日志仅在弹出的终端中显示，不额外保存文件")

        try:
            process = subprocess.Popen(
                terminal_cmd,
                cwd=self.workspace_dir,
                preexec_fn=os.setsid,
                env=env,
            )
            return process

        except Exception as e:
            messagebox.showerror("启动失败", f"启动 Python 脚本失败:\n{e}")
            self.add_log(f"[ERROR] 启动 Python 脚本失败: {e}")
            return None

    def start_keyboard_control(self):
        if self.keyboard_running:
            messagebox.showwarning("提示", "键盘控制已经在运行")
            return

        self.keyboard_terminal_process = self.start_python_in_terminal(
            self.keyboard_control_script,
            "键盘控制 - keyboard_control",
            keep_open=False,
        )

        if self.keyboard_terminal_process is None:
            return

        self.keyboard_running = True

        self.set_keyboard_status("键盘控制运行中", "blue")
        self.set_current_action("键盘控制已启动，请在弹出的终端中操作机器人", "blue")
        self.set_keyboard_button_running(True)

        self.add_log("已启动 keyboard_control.py")

        watcher = threading.Thread(
            target=self.watch_keyboard_control_process,
            daemon=True,
        )
        watcher.start()

    def watch_keyboard_control_process(self):
        if self.keyboard_terminal_process is None:
            return

        return_code = self.keyboard_terminal_process.wait()
        self.add_log_from_thread(
            f"keyboard_control.py 终端进程退出，返回码: {return_code}"
        )

        def update_when_exit():
            self.keyboard_running = False
            self.set_keyboard_status("键盘控制已退出", "green")
            self.set_current_action("键盘控制流程结束", "green")
            self.set_keyboard_button_running(False)

        self.root.after(0, update_when_exit)

    def start_slam(self):
        if self.slam_running:
            messagebox.showwarning("提示", "建图已经在运行")
            return

        self.slam_terminal_process = self.start_script_in_terminal(
            self.run_slam_script,
            "SLAM 建图 - keyboard_control",
            keep_open=False,
        )

        if self.slam_terminal_process is None:
            return

        self.slam_running = True
        self.set_slam_status("建图运行中", "blue")
        self.set_current_action("建图中，请在弹出的终端中使用键盘控制机器人", "blue")
        self.set_slam_button_running(True)

        self.add_log("已启动 run_slam.sh")
        self.add_log("建图已启动；如需结束，请在建图终端中 Ctrl+C 或关闭终端")

        watcher = threading.Thread(target=self.watch_slam_process, daemon=True)
        watcher.start()

    def watch_slam_process(self):
        if self.slam_terminal_process is None:
            return

        return_code = self.slam_terminal_process.wait()
        self.add_log_from_thread(f"run_slam.sh 终端进程退出，返回码: {return_code}")

        def update_when_exit():
            if self.slam_running:
                self.slam_running = False
                self.set_slam_status("建图已退出", "gray")
                self.set_current_action("建图终端已退出", "gray")
                self.set_slam_button_running(False)

        self.root.after(0, update_when_exit)

    def start_edit_map(self):
        if self.edit_map_running:
            messagebox.showwarning("提示", "地图编辑已经在运行")
            return

        self.edit_map_terminal_process = self.start_script_in_terminal(
            self.run_edit_map_script,
            "地图编辑 - run_edit_map",
            keep_open=True,
        )

        if self.edit_map_terminal_process is None:
            return

        self.edit_map_running = True
        self.set_edit_map_status("地图编辑运行中", "orange")
        self.set_current_action("地图编辑已启动，请在弹出的终端 / RViz 中编辑地图", "orange")
        self.set_edit_map_button_running(True)

        self.add_log("已启动 run_edit_map.sh")

        watcher = threading.Thread(
            target=self.watch_edit_map_process,
            daemon=True,
        )
        watcher.start()

    def watch_edit_map_process(self):
        if self.edit_map_terminal_process is None:
            return

        return_code = self.edit_map_terminal_process.wait()
        self.add_log_from_thread(f"run_edit_map.sh 终端进程退出，返回码: {return_code}")

        def update_when_exit():
            self.edit_map_running = False
            self.set_edit_map_status("地图编辑已退出", "green")
            self.set_current_action("地图编辑流程结束", "green")
            self.set_edit_map_button_running(False)

        self.root.after(0, update_when_exit)

    def start_nav(self):
        if self.nav_running or self.relocalization_running:
            messagebox.showwarning("提示", "自主导航或重定位流程已经在运行")
            return

        if not messagebox.askyesno(
            "启动自主导航",
            "将先清理旧节点并执行重定位。\n"
            "重定位成功后需要确认位置，再启动导航。\n\n"
            "是否继续？",
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
            for file_path in [
                self.relocalization_exit_code_file,
                self.relocalization_success_file,
            ]:
                if os.path.isfile(file_path):
                    os.remove(file_path)
        except Exception:
            pass

        self.relocalization_terminal_process = self.start_script_in_terminal(
            self.run_relocalization_script,
            "重定位 - run_relocalization",
            keep_open=False,
            exit_code_file=self.relocalization_exit_code_file,
        )

        if self.relocalization_terminal_process is None:
            return

        self.relocalization_running = True
        self.set_relocalization_status("正在重定位", "orange")
        self.set_nav_status("等待重定位成功", "orange")
        self.set_current_action("正在执行重定位，成功后需要人工确认位置", "orange")
        self.set_nav_button_running(True)
        self.add_log("已启动 run_relocalization.sh，等待重定位成功")
        self.add_log("重定位成功后会弹窗，请用户确认当前位置和朝向是否正确")

        watcher = threading.Thread(
            target=self.watch_relocalization_then_start_nav,
            daemon=True,
        )
        watcher.start()

    def watch_relocalization_then_start_nav(self):
        if self.relocalization_terminal_process is None:
            return

        terminal_return_code = self.relocalization_terminal_process.wait()
        real_return_code = self.read_int_file(self.relocalization_exit_code_file)

        if real_return_code is None:
            return_code = 1
            self.add_log_from_thread(
                "未读取到 run_relocalization.sh 真实返回码，按重定位失败处理"
            )
        else:
            return_code = real_return_code

        self.add_log_from_thread(
            f"run_relocalization.sh 真实返回码: {return_code}, 终端进程返回码: {terminal_return_code}"
        )

        self.relocalization_running = False

        if return_code != 0:
            def update_failed():
                self.set_relocalization_status("重定位失败", "red")
                self.set_nav_status("未启动", "gray")
                self.set_current_action("重定位失败，未启动自主导航", "red")
                self.set_nav_button_running(False)
                messagebox.showerror(
                    "重定位失败",
                    "run_relocalization.sh 返回失败。\n"
                    "请查看 relocalization 日志。",
                )

            self.root.after(0, update_failed)
            return

        success_flag = self.read_int_file(self.relocalization_success_file)

        if success_flag != 1:
            def update_success_file_failed():
                self.set_relocalization_status("重定位结果未确认", "red")
                self.set_nav_status("未启动", "gray")
                self.set_current_action("未读取到重定位成功标志，未启动自主导航", "red")
                self.set_nav_button_running(False)
                self.add_log(
                    "run_relocalization.sh 返回 0，"
                    "但 relocalization_success.txt 不是 1，按失败处理"
                )
                messagebox.showerror(
                    "重定位未确认成功",
                    "未读取到重定位成功标志。\n"
                    "请查看 relocalization 日志。",
                )

            self.root.after(0, update_success_file_failed)
            return

        def ask_user_confirm_pose():
            self.set_relocalization_status("重定位成功，等待人工确认", "orange")
            self.set_nav_status("等待用户确认位置", "orange")
            self.set_current_action("重定位成功，请确认当前位置和朝向是否正确", "orange")
            self.add_log("重定位成功，等待用户确认当前位置和朝向是否正确")

            confirmed = messagebox.askyesno(
                "确认重定位结果",
                "请在 RViz 中确认当前位置和朝向。\n\n"
                "是否正确并启动导航？",
            )

            if confirmed:
                self.set_relocalization_status("重定位已确认", "green")
                self.set_nav_status("正在启动自主导航", "green")
                self.set_current_action("用户确认位置正确，正在启动 run_nav.sh", "green")
                self.add_log("用户确认重定位位置正确，开始启动 run_nav.sh")
                self.start_nav_after_relocalization_success()
            else:
                self.set_relocalization_status("重定位成功但未确认", "orange")
                self.set_nav_status("未启动", "gray")
                self.set_current_action("用户未确认重定位结果，已取消启动自主导航", "red")
                self.set_nav_button_running(False)
                self.add_log("用户未确认重定位结果，取消启动 run_nav.sh")

                messagebox.showwarning(
                    "已取消自主导航",
                    "未启动 run_nav.sh。\n请重新重定位或检查定位结果。",
                )

        self.root.after(0, ask_user_confirm_pose)

    def start_nav_after_relocalization_success(self):
        if self.nav_running:
            messagebox.showwarning("提示", "自主导航已经在运行")
            return

        self.nav_terminal_process = self.start_script_in_terminal(
            self.run_nav_script,
            "自主导航 - run_nav",
            keep_open=True,
        )

        if self.nav_terminal_process is None:
            self.set_nav_status("自主导航启动失败", "red")
            self.set_current_action("run_nav.sh 启动失败", "red")
            self.set_nav_button_running(False)
            return

        self.nav_running = True
        self.set_nav_status("自主导航运行中", "purple")
        self.set_current_action("自主导航已启动，请在弹出终端/导航 GUI 中操作", "purple")
        self.set_nav_button_running(True)
        self.add_log("已启动 run_nav.sh")

        watcher = threading.Thread(target=self.watch_nav_process, daemon=True)
        watcher.start()

    def watch_nav_process(self):
        if self.nav_terminal_process is None:
            return

        return_code = self.nav_terminal_process.wait()
        self.add_log_from_thread(f"run_nav.sh 终端进程退出，返回码: {return_code}")

        def update_when_exit():
            self.nav_running = False
            self.set_nav_status("自主导航已退出", "green")
            self.set_current_action("自主导航流程结束", "green")
            self.set_nav_button_running(False)

        self.root.after(0, update_when_exit)

    def manual_kill_nodes(self):
        """静默调用 run_kill_node.sh 清理 ROS2 节点。"""
        if not os.path.isfile(self.run_kill_node_script):
            messagebox.showerror(
                "清理脚本不存在",
                f"找不到清理脚本:\n{self.run_kill_node_script}",
            )
            return

        if self.has_running_processes():
            if not messagebox.askyesno(
                "确认清理节点",
                "当前仍有任务运行。\n"
                "清理会中断建图/重定位/导航。\n\n"
                "是否继续？",
            ):
                return
        else:
            if not messagebox.askyesno(
                "确认清理节点",
                "将清理旧 ROS2 节点、RViz、bag。\n\n"
                "是否继续？",
            ):
                return

        self.kill_nodes_button.config(state=tk.DISABLED)
        self.set_current_action("正在清理 ROS2 节点...", "orange")

        t = threading.Thread(target=self._manual_kill_nodes_worker, daemon=True)
        t.start()

    def _manual_kill_nodes_worker(self):
        env = os.environ.copy()
        env["KILL_RVIZ"] = "1"
        env["KILL_NAV_GUI"] = "0"
        env["KILL_BAG"] = "1"
        env["LANG"] = "C.UTF-8"
        env["LC_ALL"] = "C.UTF-8"
        env["LANGUAGE"] = "C.UTF-8"

        try:
            result = subprocess.run(
                ["bash", self.run_kill_node_script],
                cwd=self.workspace_dir,
                env=env,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                text=True,
                timeout=60,
            )

            if result.returncode == 0:
                def done():
                    self.slam_running = False
                    self.edit_map_running = False
                    self.keyboard_running = False
                    self.relocalization_running = False
                    self.nav_running = False

                    self.set_slam_status("已清理/未启动", "gray")
                    self.set_edit_map_status("已清理/未启动", "gray")
                    self.set_keyboard_status("已清理/未启动", "gray")
                    self.set_relocalization_status("已清理/未启动", "gray")
                    self.set_nav_status("已清理/未启动", "gray")

                    self.set_slam_button_running(False)
                    self.set_edit_map_button_running(False)
                    self.set_keyboard_button_running(False)
                    self.set_nav_button_running(False)
                    self.kill_nodes_button.config(state=tk.NORMAL)

                    self.set_current_action("节点清理完成", "green")
                    messagebox.showinfo("清理完成", "run_kill_node.sh 已执行完成。")

                self.root.after(0, done)
            else:
                def failed():
                    self.kill_nodes_button.config(state=tk.NORMAL)
                    self.set_current_action("清理脚本返回异常", "red")
                    messagebox.showwarning(
                        "清理脚本返回异常",
                        f"run_kill_node.sh 返回码: {result.returncode}",
                    )

                self.root.after(0, failed)

        except subprocess.TimeoutExpired:
            def timeout_failed():
                self.kill_nodes_button.config(state=tk.NORMAL)
                self.set_current_action("清理脚本执行超时", "red")
                messagebox.showwarning("清理超时", "run_kill_node.sh 执行超过 60 秒。")

            self.root.after(0, timeout_failed)

        except Exception as e:
            def exception_failed():
                self.kill_nodes_button.config(state=tk.NORMAL)
                self.set_current_action("清理节点失败", "red")
                messagebox.showerror("清理失败", f"执行 run_kill_node.sh 失败:\n{e}")

            self.root.after(0, exception_failed)

    def has_running_processes(self):
        return (
            self.slam_running
            or self.edit_map_running
            or self.keyboard_running
            or self.relocalization_running
            or self.nav_running
        )

    def confirm_exit_if_running(self):
        if not self.has_running_processes():
            return True

        return messagebox.askyesno(
            "确认退出",
            "当前仍有任务运行。\n"
            "退出 GUI 不会关闭已打开的终端。\n\n"
            "确定退出？",
        )

    def on_closing(self):
        if not self.confirm_exit_if_running():
            return

        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main():
    gui = NavigationWorkflowGUI()
    gui.run()


if __name__ == "__main__":
    main()