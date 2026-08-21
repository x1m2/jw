#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

import sys
import termios
import tty
import threading
import time
import signal


class KeyboardControl(Node):

    def __init__(self):
        super().__init__('keyboard_control_node')

        self.declare_parameter('angular_scale', 0.50)

        self.linear_speed = 0.55
        self.angular_scale = self.get_parameter('angular_scale').value

        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0

        self.twist_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # 在线拓扑打点只发布记录触发；当前定位位姿和文件写入统一由
        # rviz_topology_editor处理，保证与离线打点格式一致。
        self.topology_pub = self.create_publisher(
            Bool,
            '/topology_node_set',
            10
        )

        self.running = True

        self.publish_thread = threading.Thread(
            target=self.publish_loop,
            daemon=True
        )
        self.publish_thread.start()

        self.get_logger().info('键盘控制节点启动')

    def publish_loop(self):
        while rclpy.ok() and self.running:
            twist = Twist()
            twist.linear.x = self.linear_x
            twist.linear.y = self.linear_y
            twist.angular.z = self.angular_z

            self.twist_pub.publish(twist)
            time.sleep(0.02)  # 50Hz

    def stop_robot(self):
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0

    def publish_stop_once(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0

        for _ in range(10):
            self.twist_pub.publish(twist)
            time.sleep(0.02)

    def print_help(self):
        print(
            """
控制方法
---------------------------
W   : 前进（固定线速度 0.55 m/s）
S   : 后退（固定线速度 0.55 m/s）
Q/E : 左转/右转
K   : 保存当前纯定位位姿为拓扑节点（需先启动在线打点）
空格 : 停止
ESC : 退出
CTRL-C : 退出
---------------------------
""",
            flush=True
        )

    def key_loop(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        # 先打印说明，再进入 raw 模式
        self.print_help()

        tty.setraw(fd)

        try:
            while rclpy.ok() and self.running:
                ch = sys.stdin.read(1)

                # Ctrl+C 在 raw 模式下会被读成 '\x03'
                if ch == '\x03':
                    self.get_logger().info('检测到 Ctrl+C，退出键盘控制')
                    break

                # ESC 退出
                if ch == '\x1b':
                    self.get_logger().info('检测到 ESC，退出键盘控制')
                    break

                # 统一转成小写。
                key = ch.lower()

                self.stop_robot()

                if key == 'w':
                    self.linear_x = self.linear_speed

                elif key == 's':
                    self.linear_x = -self.linear_speed

                # A/D 侧移键已停用。
                # elif key == 'a':
                #     self.linear_y = -self.linear_speed
                # elif key == 'd':
                #     self.linear_y = self.linear_speed

                elif key == 'q':
                    self.angular_z = self.angular_scale

                elif key == 'e':
                    self.angular_z = -self.angular_scale

                elif ch == ' ':
                    self.stop_robot()

                elif key == 'k':
                    # K键先停车，再触发编辑器抓取最新纯定位位姿并原子保存。
                    self.publish_stop_once()
                    msg = Bool()
                    msg.data = True
                    self.topology_pub.publish(msg)
                    self.get_logger().info('已发布在线拓扑节点保存命令')

                else:
                    continue

        finally:
            self.stop_robot()
            self.publish_stop_once()

            termios.tcsetattr(
                fd,
                termios.TCSADRAIN,
                old_settings
            )

    def destroy_node(self):
        self.running = False
        self.stop_robot()

        if self.publish_thread.is_alive():
            self.publish_thread.join(timeout=1.0)

        super().destroy_node()


def main(args=None):
    # 让 Python 输出尽量实时刷新
    try:
        sys.stdout.reconfigure(line_buffering=True)
        sys.stderr.reconfigure(line_buffering=True)
    except Exception:
        pass

    rclpy.init(args=args)

    node = KeyboardControl()

    def signal_handler(sig, frame):
        node.get_logger().info('收到退出信号')
        node.running = False

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        node.key_loop()

    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt，退出键盘控制')

    finally:
        node.get_logger().info('正在关闭 keyboard_control_node')

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
