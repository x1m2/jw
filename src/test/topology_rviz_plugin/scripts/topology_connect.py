#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import math
from typing import Dict, List, Tuple, Set

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy


class TopologyNode:
    """拓扑节点"""

    def __init__(self, node_id: int, x: float, y: float, z: float, label: str):
        self.id = node_id
        self.x = x
        self.y = y
        self.z = z
        self.label = label

    def distance_to(self, other_node) -> float:
        """计算两个拓扑点之间的欧氏距离，用作边权重"""
        return math.sqrt(
            (self.x - other_node.x) ** 2
            + (self.y - other_node.y) ** 2
            + (self.z - other_node.z) ** 2
        )


class TopologyEdge:
    """拓扑边"""

    def __init__(
        self,
        edge_id: int,
        start_node: int,
        end_node: int,
        weight: float,
        edge_type: str,
        bidirectional: int,
    ):
        self.id = edge_id
        self.start_node = start_node
        self.end_node = end_node
        self.weight = weight
        self.type = edge_type
        self.bidirectional = bidirectional

    def get_node_pair(self) -> Tuple[int, int]:
        """无向连接判断时使用，小 ID 在前"""
        return tuple(sorted([self.start_node, self.end_node]))


class TopologyMapProcessor:
    """拓扑地图处理器"""

    def __init__(self, logger):
        self.logger = logger

        self.nodes: Dict[int, TopologyNode] = {}
        self.edges: Dict[int, TopologyEdge] = {}

        # 已存在连接，用于避免重复生成边
        self.existing_connections: Set[Tuple[int, int]] = set()

        # 新增连接，格式为: 起点ID, 终点ID, 距离
        self.new_connections: List[Tuple[int, int, float]] = []

        self.node_lines: List[str] = []
        self.edge_lines: List[str] = []
        self.header_lines: List[str] = []

    def read_map_file(self, file_path: str) -> bool:
        """读取拓扑地图文件"""

        try:
            with open(file_path, "r", encoding="utf-8-sig") as f:
                lines = f.readlines()

            self.nodes.clear()
            self.edges.clear()
            self.existing_connections.clear()
            self.new_connections.clear()
            self.node_lines.clear()
            self.edge_lines.clear()
            self.header_lines.clear()

            current_section = "header"

            for line_num, line in enumerate(lines):
                stripped_line = line.strip()

                if "# Topology Nodes" in line:
                    current_section = "nodes"
                    self.header_lines.append(line)
                    continue

                if "# Topology Edges" in line:
                    current_section = "edges"
                    continue

                if stripped_line.startswith("#") or not stripped_line:
                    if current_section == "header":
                        self.header_lines.append(line)
                    continue

                parts = stripped_line.split()

                # 读取节点: node_id x y z label
                if current_section == "nodes" and len(parts) >= 5:
                    try:
                        node_id = int(parts[0])
                        x = float(parts[1])
                        y = float(parts[2])
                        z = float(parts[3])

                        # 标签可能包含中文或空格，保留第 5 列之后所有内容
                        label = " ".join(parts[4:])

                        node = TopologyNode(node_id, x, y, z, label)
                        self.nodes[node_id] = node
                        self.node_lines.append(line)

                    except ValueError as e:
                        self.logger.warn(
                            f"解析节点第 {line_num + 1} 行失败: "
                            f"{stripped_line}, error: {e}"
                        )
                        continue

                # 读取边: edge_id start_node end_node weight type bidirectional
                elif current_section == "edges" and len(parts) >= 6:
                    try:
                        edge_id = int(parts[0])
                        start_node = int(parts[1])
                        end_node = int(parts[2])
                        weight = float(parts[3])
                        edge_type = parts[4]
                        bidirectional = int(parts[5])

                        edge = TopologyEdge(
                            edge_id,
                            start_node,
                            end_node,
                            weight,
                            edge_type,
                            bidirectional,
                        )

                        self.edges[edge_id] = edge
                        self.edge_lines.append(line)

                        # 记录已有连接，0->1 和 1->0 认为是同一条边
                        connection_pair = tuple(sorted([start_node, end_node]))
                        self.existing_connections.add(connection_pair)

                    except ValueError as e:
                        self.logger.warn(
                            f"解析边第 {line_num + 1} 行失败: "
                            f"{stripped_line}, error: {e}"
                        )
                        continue

            self.logger.info(
                f"成功读取拓扑地图: {len(self.nodes)} 个节点, "
                f"{len(self.edges)} 条边"
            )
            return True

        except FileNotFoundError:
            self.logger.error(f"找不到输入文件: {file_path}")
            return False

        except Exception as e:
            self.logger.error(f"读取文件失败: {e}")
            return False

    def find_new_connections(self) -> int:
        """
        按节点 ID 顺序连接相邻节点。

        例如节点 ID 为:
        0, 1, 2, 3, 4

        则生成:
        0 -> 1
        1 -> 2
        2 -> 3
        3 -> 4
        """

        self.new_connections.clear()

        if len(self.nodes) < 2:
            self.logger.warn("节点数量少于 2，无法生成顺序连接")
            return 0

        # 按节点 ID 从小到大排序
        sorted_nodes = sorted(self.nodes.values(), key=lambda node: node.id)

        self.logger.info(f"正在按照节点 ID 顺序连接 {len(sorted_nodes)} 个节点")
        self.logger.info(f"当前已有连接数: {len(self.existing_connections)}")

        for i in range(len(sorted_nodes) - 1):
            node1 = sorted_nodes[i]
            node2 = sorted_nodes[i + 1]

            # 判断相邻节点是否已经连接过
            connection_pair = tuple(sorted([node1.id, node2.id]))

            if connection_pair in self.existing_connections:
                self.logger.info(
                    f"节点 {node1.id} 和节点 {node2.id} 已存在连接，跳过"
                )
                continue

            distance = node1.distance_to(node2)

            self.new_connections.append((node1.id, node2.id, distance))

            self.logger.info(
                f"新增顺序连接: 节点 {node1.id} -> 节点 {node2.id}, "
                f"距离: {distance:.3f} m"
            )

        self.logger.info(f"按照节点 ID 顺序新增连接数: {len(self.new_connections)}")

        return len(self.new_connections)

    def write_connected_map(self, output_path: str) -> bool:
        """写入带顺序连接边的拓扑地图"""

        try:
            output_dir = os.path.dirname(output_path)
            if output_dir:
                os.makedirs(output_dir, exist_ok=True)

            with open(output_path, "w", encoding="utf-8") as f:
                # 写入文件头，避免重复写旧的处理信息
                for line in self.header_lines:
                    if (
                        "连接阈值" not in line
                        and "新增连接数" not in line
                        and "连接方式" not in line
                    ):
                        f.write(line)

                f.write("# 连接方式: 按节点 ID 顺序连接\n")
                f.write(f"# 新增连接数: {len(self.new_connections)}\n")

                # 写入节点
                for line in self.node_lines:
                    f.write(line)

                f.write("# 格式: 边ID 起始节点ID 结束节点ID 权重 类型 双向\n")
                f.write("# Topology Edges\n")

                # 写入原有边
                for line in self.edge_lines:
                    f.write(line)

                # 写入新增顺序边
                if self.new_connections:
                    f.write("# 新增连接 (按节点 ID 顺序自动生成)\n")

                    next_edge_id = max(self.edges.keys()) + 1 if self.edges else 0

                    for node1_id, node2_id, distance in self.new_connections:
                        f.write(
                            f"{next_edge_id} {node1_id} {node2_id} "
                            f"{distance:.6f} auto 1\n"
                        )
                        next_edge_id += 1

            self.logger.info(f"成功写入连接后的拓扑地图: {output_path}")
            return True

        except Exception as e:
            self.logger.error(f"写入文件失败: {e}")
            return False

    def print_statistics(self):
        """打印统计信息"""

        if not self.nodes:
            self.logger.warn("没有节点数据，无法统计")
            return

        node_connections = {node_id: 0 for node_id in self.nodes.keys()}

        # 统计原有连接
        for node1_id, node2_id in self.existing_connections:
            if node1_id in node_connections:
                node_connections[node1_id] += 1
            if node2_id in node_connections:
                node_connections[node2_id] += 1

        # 统计新增连接
        for node1_id, node2_id, _ in self.new_connections:
            if node1_id in node_connections:
                node_connections[node1_id] += 1
            if node2_id in node_connections:
                node_connections[node2_id] += 1

        total_connections = len(self.existing_connections) + len(self.new_connections)
        nodes_with_connections = sum(
            1 for count in node_connections.values() if count > 0
        )
        nodes_without_connections = len(self.nodes) - nodes_with_connections

        self.logger.info("========== 拓扑连接统计 ==========")
        self.logger.info(f"总节点数: {len(self.nodes)}")
        self.logger.info(f"原有连接数: {len(self.existing_connections)}")
        self.logger.info(f"新增连接数: {len(self.new_connections)}")
        self.logger.info(f"总连接数: {total_connections}")
        self.logger.info(f"有连接的节点数: {nodes_with_connections}")
        self.logger.info(f"孤立节点数: {nodes_without_connections}")

        avg_connections = (total_connections * 2) / len(self.nodes)
        self.logger.info(f"平均每节点连接数: {avg_connections:.2f}")

        if node_connections:
            self.logger.info(f"最大连接数: {max(node_connections.values())}")
            self.logger.info(f"最小连接数: {min(node_connections.values())}")

        if self.new_connections and len(self.new_connections) <= 50:
            self.logger.info("========== 新增顺序连接详情 ==========")
            for index, (node1_id, node2_id, distance) in enumerate(
                self.new_connections, start=1
            ):
                self.logger.info(
                    f"{index}. 节点 {node1_id} -> 节点 {node2_id}, "
                    f"距离: {distance:.3f} m"
                )


class TopologyConnectNode(Node):
    """ROS2 拓扑顺序连边节点 + RViz Marker 可视化"""

    def __init__(self):
        super().__init__("topology_connect_node")

        # ==================================================
        # 原有参数
        # ==================================================
        self.declare_parameter("input_file", "")
        self.declare_parameter("output_file", "")
        self.declare_parameter("show_statistics", True)
        self.declare_parameter("run_once", True)

        # ==================================================
        # RViz 可视化参数
        # ==================================================
        self.declare_parameter("publish_markers", True)
        self.declare_parameter("marker_topic", "/topology_markers")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("publish_period", 1.0)

        # 高度偏移，避免线和点云/地面重合
        self.declare_parameter("edge_z_offset", 0.15)
        self.declare_parameter("node_z_offset", 0.25)
        self.declare_parameter("text_z_offset", 0.65)

        # 显示尺寸
        self.declare_parameter("old_edge_width", 0.06)
        self.declare_parameter("new_edge_width", 0.09)
        self.declare_parameter("node_scale", 0.25)
        self.declare_parameter("text_scale", 0.35)

        self.input_file = (
            self.get_parameter("input_file").get_parameter_value().string_value
        )
        self.output_file = (
            self.get_parameter("output_file").get_parameter_value().string_value
        )
        self.show_statistics = (
            self.get_parameter("show_statistics").get_parameter_value().bool_value
        )
        self.run_once = (
            self.get_parameter("run_once").get_parameter_value().bool_value
        )

        self.publish_markers = (
            self.get_parameter("publish_markers").get_parameter_value().bool_value
        )
        self.marker_topic = (
            self.get_parameter("marker_topic").get_parameter_value().string_value
        )
        self.frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )
        self.publish_period = (
            self.get_parameter("publish_period").get_parameter_value().double_value
        )

        self.edge_z_offset = (
            self.get_parameter("edge_z_offset").get_parameter_value().double_value
        )
        self.node_z_offset = (
            self.get_parameter("node_z_offset").get_parameter_value().double_value
        )
        self.text_z_offset = (
            self.get_parameter("text_z_offset").get_parameter_value().double_value
        )

        self.old_edge_width = (
            self.get_parameter("old_edge_width").get_parameter_value().double_value
        )
        self.new_edge_width = (
            self.get_parameter("new_edge_width").get_parameter_value().double_value
        )
        self.node_scale = (
            self.get_parameter("node_scale").get_parameter_value().double_value
        )
        self.text_scale = (
            self.get_parameter("text_scale").get_parameter_value().double_value
        )

        self.marker_pub = None
        self.marker_timer = None
        self.processor = TopologyMapProcessor(logger=self.get_logger())
        self.process_ok = False

        if not self.input_file:
            self.get_logger().error("参数 input_file 为空")
            self.shutdown_if_run_once()
            return

        if not self.output_file:
            self.get_logger().error("参数 output_file 为空")
            self.shutdown_if_run_once()
            return

        if self.publish_markers:
            # transient_local 方便 RViz 后启动时收到最后一次 Marker
            qos = QoSProfile(depth=1)
            qos.reliability = QoSReliabilityPolicy.RELIABLE
            qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

            self.marker_pub = self.create_publisher(
                MarkerArray,
                self.marker_topic,
                qos,
            )

        self.get_logger().info("拓扑地图顺序连边节点启动")
        self.get_logger().info(f"输入文件: {self.input_file}")
        self.get_logger().info(f"输出文件: {self.output_file}")
        self.get_logger().info("连接方式: 按节点 ID 顺序连接")
        self.get_logger().info(f"run_once: {self.run_once}")
        self.get_logger().info(f"publish_markers: {self.publish_markers}")

        if self.publish_markers:
            self.get_logger().info(f"RViz Marker 话题: {self.marker_topic}")
            self.get_logger().info(f"RViz Fixed Frame 应设置为: {self.frame_id}")

        self.process_ok = self.process()

        if not self.process_ok:
            self.get_logger().error("拓扑连边处理失败")
            self.shutdown_if_run_once()
            return

        if self.publish_markers:
            # 立即发布一次，方便已经打开的 RViz 立即显示
            self.publish_topology_markers()

        if self.run_once:
            self.get_logger().info("run_once=true，处理完成后节点退出")
            # run_once 模式下稍微延迟一下，给 DDS 发送 Marker 的时间
            if self.publish_markers:
                self.create_timer(0.5, self.shutdown_once)
            else:
                rclpy.shutdown()
        else:
            self.get_logger().info("run_once=false，节点保持运行，持续发布 RViz Marker")

            if self.publish_markers:
                period = max(0.1, float(self.publish_period))
                self.marker_timer = self.create_timer(
                    period,
                    self.publish_topology_markers,
                )

    def shutdown_if_run_once(self):
        if self.run_once:
            rclpy.shutdown()

    def shutdown_once(self):
        self.get_logger().info("Marker 已发布，退出 topology_connect_node")
        rclpy.shutdown()

    def process(self) -> bool:
        if not self.processor.read_map_file(self.input_file):
            return False

        self.processor.find_new_connections()

        if self.show_statistics:
            self.processor.print_statistics()

        if not self.processor.write_connected_map(self.output_file):
            return False

        return True

    def make_point(self, x: float, y: float, z: float) -> Point:
        p = Point()
        p.x = float(x)
        p.y = float(y)
        p.z = float(z)
        return p

    def publish_topology_markers(self):
        if not self.publish_markers:
            return

        if self.marker_pub is None:
            return

        if not self.processor.nodes:
            return

        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        marker_id = 0

        # 先清空历史 Marker，避免 RViz 中残留旧拓扑
        clear_marker = Marker()
        clear_marker.header.frame_id = self.frame_id
        clear_marker.header.stamp = stamp
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        # ==================================================
        # 1. 原有边：绿色 LINE_LIST
        # ==================================================
        old_edge_marker = Marker()
        old_edge_marker.header.frame_id = self.frame_id
        old_edge_marker.header.stamp = stamp
        old_edge_marker.ns = "topology_old_edges"
        old_edge_marker.id = marker_id
        marker_id += 1
        old_edge_marker.type = Marker.LINE_LIST
        old_edge_marker.action = Marker.ADD
        old_edge_marker.pose.orientation.w = 1.0
        old_edge_marker.scale.x = float(self.old_edge_width)

        # 绿色：文件中原本已有的边
        old_edge_marker.color.r = 0.0
        old_edge_marker.color.g = 1.0
        old_edge_marker.color.b = 0.0
        old_edge_marker.color.a = 1.0

        for edge in self.processor.edges.values():
            if edge.start_node not in self.processor.nodes:
                self.get_logger().warn(
                    f"边 {edge.id} 起点不存在: {edge.start_node}"
                )
                continue

            if edge.end_node not in self.processor.nodes:
                self.get_logger().warn(
                    f"边 {edge.id} 终点不存在: {edge.end_node}"
                )
                continue

            start_node = self.processor.nodes[edge.start_node]
            end_node = self.processor.nodes[edge.end_node]

            old_edge_marker.points.append(
                self.make_point(
                    start_node.x,
                    start_node.y,
                    start_node.z + self.edge_z_offset,
                )
            )
            old_edge_marker.points.append(
                self.make_point(
                    end_node.x,
                    end_node.y,
                    end_node.z + self.edge_z_offset,
                )
            )

        marker_array.markers.append(old_edge_marker)

        # ==================================================
        # 2. 新增自动边：黄色 LINE_LIST
        # ==================================================
        new_edge_marker = Marker()
        new_edge_marker.header.frame_id = self.frame_id
        new_edge_marker.header.stamp = stamp
        new_edge_marker.ns = "topology_new_edges"
        new_edge_marker.id = marker_id
        marker_id += 1
        new_edge_marker.type = Marker.LINE_LIST
        new_edge_marker.action = Marker.ADD
        new_edge_marker.pose.orientation.w = 1.0
        new_edge_marker.scale.x = float(self.new_edge_width)

        # 黄色：本次自动新增的顺序连边
        new_edge_marker.color.r = 1.0
        new_edge_marker.color.g = 1.0
        new_edge_marker.color.b = 0.0
        new_edge_marker.color.a = 1.0

        for node1_id, node2_id, _ in self.processor.new_connections:
            if node1_id not in self.processor.nodes:
                continue

            if node2_id not in self.processor.nodes:
                continue

            node1 = self.processor.nodes[node1_id]
            node2 = self.processor.nodes[node2_id]

            new_edge_marker.points.append(
                self.make_point(
                    node1.x,
                    node1.y,
                    node1.z + self.edge_z_offset + 0.05,
                )
            )
            new_edge_marker.points.append(
                self.make_point(
                    node2.x,
                    node2.y,
                    node2.z + self.edge_z_offset + 0.05,
                )
            )

        marker_array.markers.append(new_edge_marker)

        # ==================================================
        # 3. 拓扑节点：红色 SPHERE
        # ==================================================
        for node in self.processor.nodes.values():
            node_marker = Marker()
            node_marker.header.frame_id = self.frame_id
            node_marker.header.stamp = stamp
            node_marker.ns = "topology_nodes"
            node_marker.id = marker_id
            marker_id += 1

            node_marker.type = Marker.SPHERE
            node_marker.action = Marker.ADD

            node_marker.pose.position.x = float(node.x)
            node_marker.pose.position.y = float(node.y)
            node_marker.pose.position.z = float(node.z + self.node_z_offset)
            node_marker.pose.orientation.w = 1.0

            node_marker.scale.x = float(self.node_scale)
            node_marker.scale.y = float(self.node_scale)
            node_marker.scale.z = float(self.node_scale)

            # 红色：拓扑节点
            node_marker.color.r = 1.0
            node_marker.color.g = 0.0
            node_marker.color.b = 0.0
            node_marker.color.a = 1.0

            marker_array.markers.append(node_marker)

        # ==================================================
        # 4. 节点 ID：白色 TEXT_VIEW_FACING
        # ==================================================
        for node in self.processor.nodes.values():
            text_marker = Marker()
            text_marker.header.frame_id = self.frame_id
            text_marker.header.stamp = stamp
            text_marker.ns = "topology_node_ids"
            text_marker.id = marker_id
            marker_id += 1

            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD

            text_marker.pose.position.x = float(node.x)
            text_marker.pose.position.y = float(node.y)
            text_marker.pose.position.z = float(node.z + self.text_z_offset)
            text_marker.pose.orientation.w = 1.0

            text_marker.scale.z = float(self.text_scale)

            # 白色：节点 ID
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0

            text_marker.text = str(node.id)

            marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)

    node = TopologyConnectNode()

    if rclpy.ok():
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()