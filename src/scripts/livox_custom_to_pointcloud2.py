#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from livox_ros_driver2.msg import CustomMsg
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

class LivoxCustomToPointCloud2(Node):
    def __init__(self):
        super().__init__('livox_custom_to_pointcloud2')

        # 参数
        self.declare_parameter('input_topic', '/livox/lidar')
        self.declare_parameter('output_topic', '/livox/pointcloud2')
        self.declare_parameter('output_frame', '')  # 为空时使用原始 frame_id

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.output_frame = self.get_parameter('output_frame').value

        # Livox 点云一般用 SensorData QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.sub = self.create_subscription(
            CustomMsg,
            self.input_topic,
            self.livox_callback,
            qos
        )

        self.pub = self.create_publisher(
            PointCloud2,
            self.output_topic,
            qos
        )

        self.get_logger().info('Livox CustomMsg -> PointCloud2 converter started')
        self.get_logger().info(f'Input : {self.input_topic}')
        self.get_logger().info(f'Output: {self.output_topic}')

    def livox_callback(self, msg: CustomMsg):
        """
        livox_ros_driver2/msg/CustomMsg:
          std_msgs/Header header
          uint64 timebase
          uint32 point_num
          uint8 lidar_id
          uint8[3] rsvd
          CustomPoint[] points

        CustomPoint:
          uint32 offset_time
          float32 x
          float32 y
          float32 z
          uint8 reflectivity
          uint8 tag
          uint8 line
        """

        if msg.point_num == 0 or len(msg.points) == 0:
            return

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = self.output_frame if self.output_frame else msg.header.frame_id

        # PointCloud2 字段定义
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        # 转换点数据
        points = []
        for p in msg.points:
            points.append([
                float(p.x),
                float(p.y),
                float(p.z),
                float(p.reflectivity)
            ])

        cloud_msg = point_cloud2.create_cloud(
            header=header,
            fields=fields,
            points=points
        )

        self.pub.publish(cloud_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LivoxCustomToPointCloud2()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
