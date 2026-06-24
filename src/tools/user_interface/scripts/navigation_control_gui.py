#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
导航任务 GUI 控制界面 - ROS2 版本

功能：
- 可视化控制导航任务
- 显示当前目标点和导航状态
- 支持开始任务、循环任务、停止任务、继续任务、回家、紧急停止
- 从 ROBOT_DATA_DIR 环境变量读取任务目录
- 自动加载 task_folder_path 目录下的 .txt 任务文件
"""

import datetime
import os
import threading
from typing import List, Optional

import tkinter as tk
from tkinter import ttk, messagebox
import tkinter.font as tkfont

import yaml

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose

from ament_index_python.packages import get_package_share_directory


class NavigationControlGUI(Node):
    def __init__(self):
        """初始化 GUI 控制界面"""
        super().__init__("navigation_control_gui")
        self.state_machine_node_name = "navigation_task_state_machine_node"
        self.state_machine_yaml_name = "navigation_task_state_machine.yaml"

        # 从 YAML 中读取任务目录
        self.task_folder_path = self.resolve_task_folder_path()

        # 创建主窗口
        self.root = tk.Tk()

        # 设置中文字体，解决 Tkinter 中文乱码 / 方框问题
        self.font_family = self.select_chinese_font()
        self.mono_font_family = self.font_family

        self.configure_tk_fonts()

        self.root.title("机器人导航控制界面 - ROS2")
        self.root.geometry("800x800")
        self.root.resizable(True, True)

        # 初始化变量
        self.current_target = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.emergency_stop_state = False
        self.nav_state = False
        self.last_target_update = "未接收到目标点"
        self.available_tasks: List[str] = []
        self.selected_task = tk.StringVar()

        # 样式
        self.setup_styles()

        # 加载任务文件
        self.load_available_tasks()

        # ROS2 发布器
        self.nav_task_signal_pub = self.create_publisher(
            String,
            "/nav_task_signal",
            10,
        )

        self.emergency_stop_pub = self.create_publisher(
            Bool,
            "/emergency_stop",
            10,
        )

        # ROS2 订阅器
        self.target_points_sub = self.create_subscription(
            Pose,
            "/target_points",
            self.target_points_callback,
            10,
        )

        self.nav_state_sub = self.create_subscription(
            Bool,
            "/nav_state",
            self.nav_state_callback,
            10,
        )

        # 创建 GUI
        self.create_widgets()

        # ROS2 spin 线程
        # 注意：不要命名为 self.executor，避免和 rclpy Node 内部属性冲突
        self.ros_executor = SingleThreadedExecutor()
        self.ros_executor.add_node(self)

        self.ros_thread = threading.Thread(target=self.ros_spin, daemon=True)
        self.ros_thread.start()

        # GUI 定时刷新
        self.update_display()

        self.get_logger().info("导航控制 GUI 界面已启动")
        self.get_logger().info(f"当前 GUI 字体: {self.font_family}")
        self.get_logger().info(f"当前任务目录: {self.task_folder_path}")

    # ==================================================
    # YAML 读取
    # ==================================================
    def resolve_task_folder_path(self) -> str:
        try:
            robot_data_dir = os.environ["ROBOT_DATA_DIR"]
            task_folder = os.path.join(robot_data_dir, "task")
            self.get_logger().info(
                f"从 ROBOT_DATA_DIR 读取任务目录: {task_folder}"
            )
            return task_folder

        except Exception as e:
            self.get_logger().error(f"读取 ROBOT_DATA_DIR 失败: {e}")
            return ""

    def expand_path(self, path: str) -> str:
        """展开 ~ 和环境变量"""
        return os.path.abspath(
            os.path.expandvars(
                os.path.expanduser(path)
            )
        )

    def read_task_folder_from_yaml(self, yaml_file: str) -> Optional[str]:
        """
        从指定 YAML 中读取：

        navigation_task_state_machine_node:
          ros__parameters:
            task_folder_path: "${ROBOT_DATA_DIR}/task"
        """
        yaml_file = self.expand_path(yaml_file)

        if not os.path.isfile(yaml_file):
            self.get_logger().error(f"YAML 文件不存在: {yaml_file}")
            return None

        try:
            with open(yaml_file, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f)

            if not isinstance(data, dict):
                self.get_logger().error("YAML 内容格式错误，顶层不是字典")
                return None

            node_config = data.get(self.state_machine_node_name)

            if not isinstance(node_config, dict):
                self.get_logger().error(
                    f"YAML 中没有找到节点配置: {self.state_machine_node_name}"
                )
                return None

            ros_params = node_config.get("ros__parameters")

            if not isinstance(ros_params, dict):
                self.get_logger().error(
                    f"YAML 中没有找到 ros__parameters: {self.state_machine_node_name}"
                )
                return None

            task_folder = ros_params.get("task_folder_path")

            if not task_folder:
                self.get_logger().error("YAML 中没有找到 task_folder_path")
                return None

            return self.expand_path(str(task_folder))

        except Exception as e:
            self.get_logger().error(
                f"读取 YAML 失败: {yaml_file}, error: {e}"
            )
            return None

    # ==================================================
    # 字体
    # ==================================================
    def select_chinese_font(self):
        """选择系统中可用的中文字体"""
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

        # 如果没有找到中文字体，退回 Tk 默认字体
        return "TkDefaultFont"

    def configure_tk_fonts(self):
        """配置 Tkinter 默认字体"""
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

        except Exception as e:
            self.get_logger().warn(f"设置 GUI 字体失败: {e}")

    # ==================================================
    # 任务加载
    # ==================================================
    def load_available_tasks(self):
        """
        加载 YAML 中 task_folder_path 指定目录下所有 .txt 任务文件。
        """
        try:
            task_folder = self.task_folder_path

            if not task_folder:
                self.get_logger().error(
                    "任务目录为空，无法加载任务文件。请检查 YAML。"
                )
                self.available_tasks = []
                return

            self.get_logger().info(f"开始加载任务目录: {task_folder}")

            if not os.path.exists(task_folder):
                self.get_logger().error(f"任务文件夹不存在: {task_folder}")
                self.available_tasks = []
                return

            if not os.path.isdir(task_folder):
                self.get_logger().error(f"任务路径不是文件夹: {task_folder}")
                self.available_tasks = []
                return

            files = [
                filename[:-4]
                for filename in os.listdir(task_folder)
                if filename.endswith(".txt")
                and os.path.isfile(os.path.join(task_folder, filename))
            ]

            self.available_tasks = sorted(files)

            self.get_logger().info(
                "找到 {} 个任务文件: {}".format(
                    len(self.available_tasks),
                    ", ".join(self.available_tasks),
                )
            )

            if self.available_tasks:
                self.selected_task.set(self.available_tasks[0])
            else:
                self.selected_task.set("无可用任务")

        except Exception as e:
            self.get_logger().error(f"加载任务文件列表失败: {e}")
            self.available_tasks = []

    # ==================================================
    # GUI 样式
    # ==================================================
    def setup_styles(self):
        """设置 GUI 样式"""
        style = ttk.Style()

        style.configure(
            "Start.TButton",
            foreground="white",
            background="green",
            font=(self.font_family, 12, "bold"),
        )
        style.configure(
            "Stop.TButton",
            foreground="white",
            background="red",
            font=(self.font_family, 12, "bold"),
        )
        style.configure(
            "Home.TButton",
            foreground="white",
            background="blue",
            font=(self.font_family, 12, "bold"),
        )
        style.configure(
            "Emergency.TButton",
            foreground="white",
            background="orange",
            font=(self.font_family, 12, "bold"),
        )

    # ==================================================
    # GUI 控件
    # ==================================================
    def create_widgets(self):
        """创建 GUI 组件"""
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        main_frame.columnconfigure(1, weight=1)

        title_label = tk.Label(
            main_frame,
            text="机器人导航控制",
            font=(self.font_family, 16, "bold"),
            fg="navy",
        )
        title_label.grid(row=0, column=0, columnspan=2, pady=(0, 20))

        # 状态区域
        status_frame = ttk.LabelFrame(main_frame, text="机器人状态", padding="10")
        status_frame.grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 20))
        status_frame.columnconfigure(1, weight=1)

        tk.Label(
            status_frame,
            text="当前目标点:",
            font=(self.font_family, 10, "bold"),
        ).grid(row=0, column=0, sticky=tk.W, pady=2)

        self.target_label = tk.Label(
            status_frame,
            text="等待目标点...",
            font=(self.font_family, 10),
            fg="blue",
            wraplength=300,
        )
        self.target_label.grid(row=0, column=1, sticky=(tk.W, tk.E), pady=2, padx=(10, 0))

        tk.Label(
            status_frame,
            text="导航状态:",
            font=(self.font_family, 10, "bold"),
        ).grid(row=1, column=0, sticky=tk.W, pady=2)

        self.nav_status_label = tk.Label(
            status_frame,
            text="未知",
            font=(self.font_family, 10),
            fg="gray",
        )
        self.nav_status_label.grid(row=1, column=1, sticky=(tk.W, tk.E), pady=2, padx=(10, 0))

        tk.Label(
            status_frame,
            text="紧急停止:",
            font=(self.font_family, 10, "bold"),
        ).grid(row=2, column=0, sticky=tk.W, pady=2)

        self.emergency_status_label = tk.Label(
            status_frame,
            text="关闭",
            font=(self.font_family, 10),
            fg="green",
        )
        self.emergency_status_label.grid(row=2, column=1, sticky=(tk.W, tk.E), pady=2, padx=(10, 0))

        tk.Label(
            status_frame,
            text="最后更新:",
            font=(self.font_family, 10, "bold"),
        ).grid(row=3, column=0, sticky=tk.W, pady=2)

        self.update_time_label = tk.Label(
            status_frame,
            text="-",
            font=(self.font_family, 10),
            fg="gray",
        )
        self.update_time_label.grid(row=3, column=1, sticky=(tk.W, tk.E), pady=2, padx=(10, 0))

        # 任务选择区域
        task_select_frame = ttk.LabelFrame(main_frame, text="任务选择", padding="10")
        task_select_frame.grid(row=2, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 20))
        task_select_frame.columnconfigure(1, weight=1)

        tk.Label(
            task_select_frame,
            text="选择任务:",
            font=(self.font_family, 10, "bold"),
        ).grid(row=0, column=0, sticky=tk.W, padx=5)

        if self.available_tasks:
            self.task_combobox = ttk.Combobox(
                task_select_frame,
                textvariable=self.selected_task,
                values=self.available_tasks,
                state="readonly",
                font=(self.font_family, 10),
                width=30,
            )
        else:
            self.task_combobox = ttk.Combobox(
                task_select_frame,
                textvariable=self.selected_task,
                values=["无可用任务"],
                state="readonly",
                font=(self.font_family, 10),
                width=30,
            )
            self.selected_task.set("无可用任务")

        self.task_combobox.grid(row=0, column=1, sticky=(tk.W, tk.E), padx=5, pady=5)

        tk.Label(
            task_select_frame,
            text="任务目录:",
            font=(self.font_family, 10, "bold"),
        ).grid(row=1, column=0, sticky=tk.W, padx=5, pady=(5, 0))

        self.task_folder_label = tk.Label(
            task_select_frame,
            text=self.task_folder_path if self.task_folder_path else "未读取到任务目录",
            font=(self.font_family, 9),
            fg="gray",
            wraplength=320,
            anchor=tk.W,
            justify=tk.LEFT,
        )
        self.task_folder_label.grid(row=1, column=1, sticky=(tk.W, tk.E), padx=5, pady=(5, 0))

        # 导航控制按钮
        control_frame = ttk.LabelFrame(main_frame, text="导航控制", padding="10")
        control_frame.grid(row=3, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 20))

        for idx in range(3):
            control_frame.columnconfigure(idx, weight=1)

        self.start_button = tk.Button(
            control_frame,
            text="开始导航",
            command=self.start_navigation,
            bg="green",
            fg="white",
            font=(self.font_family, 12, "bold"),
            height=2,
            relief=tk.RAISED,
        )
        self.start_button.grid(row=0, column=0, padx=5, pady=5, sticky=(tk.W, tk.E))

        self.emergency_button = tk.Button(
            control_frame,
            text="紧急停止",
            command=self.toggle_emergency_stop,
            bg="red",
            fg="white",
            font=(self.font_family, 12, "bold"),
            height=2,
            relief=tk.RAISED,
        )
        self.emergency_button.grid(row=0, column=1, padx=5, pady=5, sticky=(tk.W, tk.E))

        self.home_button = tk.Button(
            control_frame,
            text="回家",
            command=self.go_home,
            bg="blue",
            fg="white",
            font=(self.font_family, 12, "bold"),
            height=2,
            relief=tk.RAISED,
        )
        self.home_button.grid(row=0, column=2, padx=5, pady=5, sticky=(tk.W, tk.E))

        self.loop_button = tk.Button(
            control_frame,
            text="循环任务",
            command=self.loop_navigation,
            bg="yellow",
            fg="black",
            font=(self.font_family, 12, "bold"),
            height=1,
            relief=tk.RAISED,
        )
        self.loop_button.grid(row=1, column=0, columnspan=3, padx=5, pady=(0, 5), sticky=(tk.W, tk.E))

        # 任务控制按钮
        task_frame = ttk.LabelFrame(main_frame, text="任务控制", padding="10")
        task_frame.grid(row=4, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 20))
        task_frame.columnconfigure(0, weight=1)
        task_frame.columnconfigure(1, weight=1)

        self.stop_task_button = tk.Button(
            task_frame,
            text="停止任务",
            command=self.stop_task,
            bg="orange",
            fg="white",
            font=(self.font_family, 11, "bold"),
            height=1,
            relief=tk.RAISED,
        )
        self.stop_task_button.grid(row=0, column=0, padx=5, pady=5, sticky=(tk.W, tk.E))

        self.continue_task_button = tk.Button(
            task_frame,
            text="继续任务",
            command=self.continue_task,
            bg="purple",
            fg="white",
            font=(self.font_family, 11, "bold"),
            height=1,
            relief=tk.RAISED,
        )
        self.continue_task_button.grid(row=0, column=1, padx=5, pady=5, sticky=(tk.W, tk.E))

        # 日志区域
        log_frame = ttk.LabelFrame(main_frame, text="操作日志", padding="10")
        log_frame.grid(row=5, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), pady=(0, 10))
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        main_frame.rowconfigure(5, weight=1)

        self.log_text = tk.Text(
            log_frame,
            height=8,
            wrap=tk.WORD,
            font=(self.mono_font_family, 9),
        )
        scrollbar = ttk.Scrollbar(log_frame, orient="vertical", command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scrollbar.set)

        self.log_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))

        self.add_log("导航控制 GUI 界面已启动")
        self.add_log(f"任务目录: {self.task_folder_path if self.task_folder_path else '未读取到任务目录'}")
        self.add_log(f"任务数量: {len(self.available_tasks)}")
        self.add_log("等待 ROS2 消息...")

        self.status_bar = tk.Label(
            self.root,
            text="就绪",
            bd=1,
            relief=tk.SUNKEN,
            anchor=tk.W,
            font=(self.font_family, 10),
        )
        self.status_bar.grid(row=1, column=0, sticky=(tk.W, tk.E))
        self.root.rowconfigure(1, weight=0)

    # ==================================================
    # 日志
    # ==================================================
    def add_log(self, message: str):
        """添加日志消息"""
        timestamp = datetime.datetime.now().strftime("%H:%M:%S")
        log_message = f"[{timestamp}] {message}\n"

        self.log_text.insert(tk.END, log_message)
        self.log_text.see(tk.END)

        lines = self.log_text.get("1.0", tk.END).split("\n")
        if len(lines) > 100:
            self.log_text.delete("1.0", "2.0")

    # ==================================================
    # ROS2 回调
    # ==================================================
    def target_points_callback(self, msg: Pose):
        """目标点回调函数"""
        self.current_target["x"] = msg.position.x
        self.current_target["y"] = msg.position.y
        self.current_target["z"] = msg.position.z
        self.last_target_update = datetime.datetime.now().strftime("%H:%M:%S")

        self.get_logger().debug(
            "收到目标点: ({:.3f}, {:.3f}, {:.3f})".format(
                self.current_target["x"],
                self.current_target["y"],
                self.current_target["z"],
            )
        )

    def nav_state_callback(self, msg: Bool):
        """导航状态回调函数"""
        self.nav_state = msg.data
        self.get_logger().debug(
            "收到导航状态: {}".format("已到达" if self.nav_state else "导航中")
        )

    # ==================================================
    # ROS2 发布
    # ==================================================
    def publish_task_signal(self, signal: str):
        msg = String()
        msg.data = signal
        self.nav_task_signal_pub.publish(msg)

    # ==================================================
    # 按钮功能
    # ==================================================
    def start_navigation(self):
        """开始导航"""
        selected_task = self.selected_task.get()

        if not selected_task or selected_task == "无可用任务":
            messagebox.showwarning("警告", "请先选择一个任务！")
            return

        self.publish_task_signal(selected_task)

        self.add_log(f"开始执行任务: {selected_task}")
        self.status_bar.config(text=f"已发送任务信号: {selected_task}")
        self.get_logger().info(f"开始执行任务: {selected_task}")

    def loop_navigation(self):
        """循环执行任务"""
        selected_task = self.selected_task.get()

        if not selected_task or selected_task == "无可用任务":
            messagebox.showwarning("警告", "请先选择一个任务！")
            return

        self.publish_task_signal(f"LOOP:{selected_task}")

        self.add_log(f"开始循环执行任务: {selected_task}")
        self.status_bar.config(text=f"已发送循环任务信号: {selected_task}")
        self.get_logger().info(f"开始循环执行任务: {selected_task}")

    def toggle_emergency_stop(self):
        """切换紧急停止状态"""
        self.emergency_stop_state = not self.emergency_stop_state

        msg = Bool()
        msg.data = self.emergency_stop_state
        self.emergency_stop_pub.publish(msg)

        status_text = "激活" if self.emergency_stop_state else "关闭"

        self.add_log(f"紧急停止: {status_text}")
        self.status_bar.config(text=f"紧急停止已{status_text}")

        if self.emergency_stop_state:
            self.emergency_button.config(bg="darkred", text="解除停止")
        else:
            self.emergency_button.config(bg="red", text="紧急停止")

        self.get_logger().info(f"紧急停止状态: {status_text}")

    def go_home(self):
        """回家"""
        self.publish_task_signal("GO_HOME")
        self.add_log("发送回家信号")
        self.status_bar.config(text="已发送回家信号")
        self.get_logger().info("发送回家信号")

    def stop_task(self):
        """停止任务"""
        self.publish_task_signal("STOP")
        self.add_log("发送停止任务信号")
        self.status_bar.config(text="已发送停止任务信号")
        self.get_logger().info("发送停止任务信号")

    def continue_task(self):
        """继续任务"""
        self.publish_task_signal("CONTINUE")
        self.add_log("发送继续任务信号")
        self.status_bar.config(text="已发送继续任务信号")
        self.get_logger().info("发送继续任务信号")

    # ==================================================
    # 显示刷新
    # ==================================================
    def update_display(self):
        """更新显示信息"""
        try:
            target_text = (
                f"X: {self.current_target['x']:.3f}, "
                f"Y: {self.current_target['y']:.3f}, "
                f"Z: {self.current_target['z']:.3f}"
            )
            self.target_label.config(text=target_text)

            if self.nav_state:
                self.nav_status_label.config(text="已到达目标点", fg="green")
            else:
                self.nav_status_label.config(text="导航中", fg="blue")

            if self.emergency_stop_state:
                self.emergency_status_label.config(text="激活", fg="red")
            else:
                self.emergency_status_label.config(text="关闭", fg="green")

            self.update_time_label.config(text=self.last_target_update)

        except Exception as e:
            self.get_logger().warn(f"更新显示时出错: {e}")

        self.root.after(100, self.update_display)

    # ==================================================
    # ROS2 spin 和关闭
    # ==================================================
    def ros_spin(self):
        """ROS2 消息处理线程"""
        try:
            self.ros_executor.spin()
        except Exception as e:
            self.get_logger().error(f"ROS2 消息处理出错: {e}")

    def on_closing(self):
        """窗口关闭事件处理"""
        if messagebox.askokcancel("退出", "确定要退出导航控制界面吗？"):
            self.get_logger().info("导航控制 GUI 界面正在关闭...")
            self.shutdown()
            self.root.destroy()

    def shutdown(self):
        """关闭 ROS2 节点和 executor"""
        try:
            if hasattr(self, "ros_executor") and self.ros_executor is not None:
                self.ros_executor.shutdown()
        except Exception:
            pass

        try:
            self.destroy_node()
        except Exception:
            pass

    def run(self):
        """运行 GUI 界面"""
        try:
            self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
            self.get_logger().info("GUI 界面开始运行")
            self.root.mainloop()

        except Exception as e:
            self.get_logger().error(f"GUI 运行出错: {e}")
            messagebox.showerror("错误", f"GUI 运行出错: {e}")


def main(args=None):
    rclpy.init(args=args)

    gui = None

    try:
        gui = NavigationControlGUI()
        gui.run()

    except KeyboardInterrupt:
        if gui is not None:
            gui.get_logger().info("程序被用户中断")

    except Exception as e:
        print(f"错误: {e}")

    finally:
        if gui is not None:
            gui.shutdown()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
