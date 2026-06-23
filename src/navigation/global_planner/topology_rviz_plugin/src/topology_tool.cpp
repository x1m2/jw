#include "topology_rviz_plugin/topology_tool.hpp"

#include <OgreCamera.h>
#include <OgreRay.h>
#include <OgreRenderWindow.h>
#include <OgreVector3.h>
#include <OgreViewport.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>

#include <QInputDialog>
#include <QLineEdit>
#include <QVariant>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <locale>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_rendering/render_window.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace topology_rviz_plugin {

TopologyTool::TopologyTool()
    : current_mode_(MODE_ADD_NODE),
      selected_node_id_(-1),
      connecting_from_id_(-1),
      last_added_node_id_(-1),
      cloud_received_(false) {
  shortcut_key_ = 't';
  current_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  kdtree_.reset(new pcl::search::KdTree<pcl::PointXYZ>);
}

TopologyTool::~TopologyTool() {
  if (server_) {
    server_->clear();
    server_->applyChanges();
  }
}

void TopologyTool::onInitialize() {
  auto ros_node_abstraction = context_->getRosNodeAbstraction().lock();
  if (!ros_node_abstraction) {
    throw std::runtime_error("Failed to get RViz ROS node abstraction");
  }

  node_ = ros_node_abstraction->get_raw_node();

  auto declare_string_param =
      [this](const std::string& name,
             const std::string& default_value) -> std::string {
    if (!node_->has_parameter(name)) {
      node_->declare_parameter<std::string>(name, default_value);
    }
    return node_->get_parameter(name).as_string();
  };

  auto declare_double_param = [this](const std::string& name,
                                     double default_value) -> double {
    if (!node_->has_parameter(name)) {
      node_->declare_parameter<double>(name, default_value);
    }
    return node_->get_parameter(name).as_double();
  };

  auto declare_bool_param = [this](const std::string& name,
                                   bool default_value) -> bool {
    if (!node_->has_parameter(name)) {
      node_->declare_parameter<bool>(name, default_value);
    }
    return node_->get_parameter(name).as_bool();
  };

  // ============================================================
  // 从 YAML / ROS2 参数读取配置
  // 对应 YAML 参数名：
  // topology_tool.marker_topic
  // topology_tool.point_topic
  // topology_tool.pointcloud_topic
  // topology_tool.default_mode
  // topology_tool.node_label
  // topology_tool.max_connection_distance
  // topology_tool.selection_threshold
  // topology_tool.bidirectional
  // topology_tool.save_file
  // ============================================================

  const std::string marker_topic =
      declare_string_param("topology_tool.marker_topic", "topology_markers");

  const std::string point_topic =
      declare_string_param("topology_tool.point_topic", "topology_points");

  const std::string pointcloud_topic = declare_string_param(
      "topology_tool.pointcloud_topic", "/free_space_cloud");

  const std::string default_mode =
      declare_string_param("topology_tool.default_mode", "Add Node");

  const std::string default_node_label =
      declare_string_param("topology_tool.node_label", "节点");

  const double max_connection_distance =
      declare_double_param("topology_tool.max_connection_distance", 2.0);

  const double selection_threshold =
      declare_double_param("topology_tool.selection_threshold", 0.3);

  const bool bidirectional =
      declare_bool_param("topology_tool.bidirectional", true);

  const std::string save_file =
      declare_string_param("topology_tool.save_file", "topology_map.txt");

  // 创建发布者
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      marker_topic, rclcpp::QoS(1).transient_local().reliable());

  point_pub_ = node_->create_publisher<geometry_msgs::msg::PointStamped>(
      point_topic, rclcpp::QoS(1));

  // 订阅点云数据
  pointcloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic, rclcpp::SensorDataQoS(),
      std::bind(&TopologyTool::pointcloudCallback, this,
                std::placeholders::_1));

  // 创建交互标记服务器
  server_.reset(new interactive_markers::InteractiveMarkerServer(
      "topology_interactive_markers", node_));

  // 创建模式属性
  mode_property_ = new rviz_common::properties::EnumProperty(
      "Mode", "Add Node", "选择工具模式", getPropertyContainer(),
      SLOT(updateProperties()), this);

  mode_property_->addOption("Add Node", MODE_ADD_NODE);
  mode_property_->addOption("Connect Nodes", MODE_CONNECT_NODES);
  mode_property_->addOption("Edit Node", MODE_EDIT_NODE);
  mode_property_->addOption("Delete", MODE_DELETE);
  mode_property_->addOption("Auto Connect", MODE_AUTO_CONNECT);

  if (default_mode == "Add Node") {
    current_mode_ = MODE_ADD_NODE;
    mode_property_->setString("Add Node");
  } else if (default_mode == "Connect Nodes") {
    current_mode_ = MODE_CONNECT_NODES;
    mode_property_->setString("Connect Nodes");
  } else if (default_mode == "Edit Node") {
    current_mode_ = MODE_EDIT_NODE;
    mode_property_->setString("Edit Node");
  } else if (default_mode == "Delete") {
    current_mode_ = MODE_DELETE;
    mode_property_->setString("Delete");
  } else if (default_mode == "Auto Connect") {
    current_mode_ = MODE_AUTO_CONNECT;
    mode_property_->setString("Auto Connect");
  } else {
    current_mode_ = MODE_ADD_NODE;
    mode_property_->setString("Add Node");

    RCLCPP_WARN(node_->get_logger(),
                "未知 default_mode: %s，使用默认模式 Add Node",
                default_mode.c_str());
  }

  node_label_property_ = new rviz_common::properties::StringProperty(
      "Node Label", default_node_label.c_str(), "新节点的标签",
      getPropertyContainer(), SLOT(updateProperties()), this);

  connection_distance_property_ = new rviz_common::properties::FloatProperty(
      "Max Connection Distance", max_connection_distance, "最大连接距离",
      getPropertyContainer(), SLOT(updateProperties()), this);

  connection_distance_property_->setMin(0.1);
  connection_distance_property_->setMax(10.0);

  selection_threshold_property_ = new rviz_common::properties::FloatProperty(
      "Selection Threshold", selection_threshold, "点选择精确度阈值（米）",
      getPropertyContainer(), SLOT(updateProperties()), this);

  selection_threshold_property_->setMin(0.05);
  selection_threshold_property_->setMax(1.0);

  bidirectional_property_ = new rviz_common::properties::BoolProperty(
      "Bidirectional", bidirectional, "连接是否双向", getPropertyContainer(),
      SLOT(updateProperties()), this);

  save_file_property_ = new rviz_common::properties::StringProperty(
      "Save File", save_file.c_str(), "保存文件路径", getPropertyContainer(),
      SLOT(updateProperties()), this);

  // 控制按钮
  rviz_common::properties::Property* control_group =
      new rviz_common::properties::Property("Controls", QVariant(), "控制操作",
                                            getPropertyContainer());

  rviz_common::properties::BoolProperty* clear_button =
      new rviz_common::properties::BoolProperty(
          "Clear All", false, "清除所有节点和边", control_group,
          SLOT(clearAll()), this);

  rviz_common::properties::BoolProperty* save_button =
      new rviz_common::properties::BoolProperty("Save", false, "保存拓扑地图",
                                                control_group,
                                                SLOT(saveTopology()), this);

  rviz_common::properties::BoolProperty* load_button =
      new rviz_common::properties::BoolProperty("Load", false, "加载拓扑地图",
                                                control_group,
                                                SLOT(loadTopology()), this);

  (void)clear_button;
  (void)save_button;
  (void)load_button;

  RCLCPP_INFO(node_->get_logger(), "拓扑地图工具初始化完成");
  RCLCPP_INFO(node_->get_logger(), "marker_topic: %s", marker_topic.c_str());
  RCLCPP_INFO(node_->get_logger(), "point_topic: %s", point_topic.c_str());
  RCLCPP_INFO(node_->get_logger(), "pointcloud_topic: %s",
              pointcloud_topic.c_str());
  RCLCPP_INFO(node_->get_logger(), "default_mode: %s", default_mode.c_str());
  RCLCPP_INFO(node_->get_logger(), "node_label: %s",
              default_node_label.c_str());
  RCLCPP_INFO(node_->get_logger(), "max_connection_distance: %.3f",
              max_connection_distance);
  RCLCPP_INFO(node_->get_logger(), "selection_threshold: %.3f",
              selection_threshold);
  RCLCPP_INFO(node_->get_logger(), "bidirectional: %s",
              bidirectional ? "true" : "false");
  RCLCPP_INFO(node_->get_logger(), "save_file: %s", save_file.c_str());
}

void TopologyTool::activate() {
  RCLCPP_INFO(node_->get_logger(), "拓扑地图工具已激活 - 模式: %d",
              current_mode_);
}

void TopologyTool::deactivate() {
  RCLCPP_INFO(node_->get_logger(), "拓扑地图工具已停用");
}

int TopologyTool::processMouseEvent(rviz_common::ViewportMouseEvent& event) {
  if (event.type == QEvent::MouseButtonPress) {
    return processMousePress(event);
  } else if (event.type == QEvent::MouseButtonRelease) {
    return processMouseRelease(event);
  } else if (event.type == QEvent::MouseMove) {
    return processMouseMove(event);
  }

  return Render;
}

int TopologyTool::processMousePress(rviz_common::ViewportMouseEvent& event) {
  if (event.leftDown()) {
    Ogre::Vector3 point;
    bool point_found = getPoint(event, point);

    switch (current_mode_) {
      case MODE_ADD_NODE: {
        if (point_found) {
          addNode(point.x, point.y, point.z);
        } else {
          RCLCPP_WARN(node_->get_logger(), "无法在当前位置添加节点。请确保：");
          RCLCPP_WARN(node_->get_logger(), "1. 点云数据已正确加载");
          RCLCPP_WARN(node_->get_logger(), "2. 点击位置接近可行区域点云");
          RCLCPP_WARN(node_->get_logger(), "3. 点击的是点云可视化区域内的点");
        }
        break;
      }

      case MODE_CONNECT_NODES: {
        if (point_found) {
          int nearest_id =
              findNearestNode(point.x, point.y, point.z,
                              connection_distance_property_->getFloat());

          if (nearest_id >= 0) {
            if (connecting_from_id_ == -1) {
              connecting_from_id_ = nearest_id;

              RCLCPP_INFO(node_->get_logger(), "选择起始节点: %d (%s)",
                          nearest_id, nodes_[nearest_id].label.c_str());

              updateVisualization();
            } else if (nearest_id != connecting_from_id_) {
              addEdge(connecting_from_id_, nearest_id);
              connecting_from_id_ = -1;

              RCLCPP_INFO(node_->get_logger(), "连接完成");

              updateVisualization();
            } else {
              RCLCPP_INFO(node_->get_logger(), "不能连接节点到自身");
            }
          } else {
            RCLCPP_WARN(node_->get_logger(),
                        "在指定范围内未找到节点，请点击更接近现有节点的位置");
          }
        }
        break;
      }

      case MODE_EDIT_NODE: {
        if (point_found) {
          int nearest_id =
              findNearestNode(point.x, point.y, point.z,
                              connection_distance_property_->getFloat());

          if (nearest_id >= 0) {
            selected_node_id_ = nearest_id;
            createContextMenu(nearest_id);
          } else {
            RCLCPP_WARN(node_->get_logger(), "在指定范围内未找到可编辑的节点");
          }
        }
        break;
      }

      case MODE_DELETE: {
        if (point_found) {
          int nearest_id =
              findNearestNode(point.x, point.y, point.z,
                              connection_distance_property_->getFloat());

          if (nearest_id >= 0) {
            removeNode(nearest_id);
          } else {
            RCLCPP_WARN(node_->get_logger(), "在指定范围内未找到可删除的节点");
          }
        }
        break;
      }

      case MODE_AUTO_CONNECT: {
        if (point_found) {
          addNodeAutoConnect(point.x, point.y, point.z);
        } else {
          RCLCPP_WARN(node_->get_logger(), "无法在当前位置添加节点。请确保：");
          RCLCPP_WARN(node_->get_logger(), "1. 点云数据已正确加载");
          RCLCPP_WARN(node_->get_logger(), "2. 点击位置接近可行区域点云");
          RCLCPP_WARN(node_->get_logger(), "3. 点击的是点云可视化区域内的点");
        }
        break;
      }
    }
  } else if (event.rightDown()) {
    if (connecting_from_id_ != -1) {
      RCLCPP_INFO(node_->get_logger(), "取消连接操作");
      connecting_from_id_ = -1;
      updateVisualization();
    }

    if (current_mode_ == MODE_AUTO_CONNECT) {
      RCLCPP_INFO(node_->get_logger(),
                  "重置自动连接链，下一个节点将开始新的连接序列");
      last_added_node_id_ = -1;
    }

    selected_node_id_ = -1;
  }

  return Render;
}

int TopologyTool::processMouseRelease(rviz_common::ViewportMouseEvent& event) {
  (void)event;
  return Render;
}

int TopologyTool::processMouseMove(rviz_common::ViewportMouseEvent& event) {
  (void)event;
  return Render;
}

void TopologyTool::addNode(double x, double y, double z) {
  int new_id = nodes_.size();

  std::string label = node_label_property_->getStdString();
  if (label.empty()) {
    std::stringstream ss;
    ss << "节点_" << new_id;
    label = ss.str();
  }

  double adjusted_z = z + 0.3;

  TopologyNode new_node(new_id, x, y, adjusted_z, label);
  nodes_.push_back(new_node);

  createInteractiveMarker(new_node);

  geometry_msgs::msg::PointStamped point_msg;
  point_msg.header.frame_id = "map";
  point_msg.header.stamp = node_->now();
  point_msg.point.x = x;
  point_msg.point.y = y;
  point_msg.point.z = adjusted_z;

  point_pub_->publish(point_msg);

  updateVisualization();

  RCLCPP_INFO(
      node_->get_logger(),
      "添加节点 #%d: %s at (%.2f, %.2f, %.2f) [原始z: %.2f, 抬高0.3m后: %.2f]",
      new_id, label.c_str(), x, y, adjusted_z, z, adjusted_z);
}

void TopologyTool::addNodeAutoConnect(double x, double y, double z) {
  int new_id = nodes_.size();

  std::string label = node_label_property_->getStdString();
  if (label.empty()) {
    std::stringstream ss;
    ss << "节点_" << new_id;
    label = ss.str();
  }

  double adjusted_z = z + 0.3;

  TopologyNode new_node(new_id, x, y, adjusted_z, label);
  nodes_.push_back(new_node);

  if (last_added_node_id_ >= 0 &&
      last_added_node_id_ < static_cast<int>(nodes_.size()) - 1) {
    bool bidirectional = bidirectional_property_->getBool();
    int new_edge_id = edges_.size();
    double weight = calculateDistance(nodes_[last_added_node_id_], new_node);

    TopologyEdge new_edge(new_edge_id, last_added_node_id_, new_id, weight,
                          "auto", bidirectional);

    edges_.push_back(new_edge);

    RCLCPP_INFO(node_->get_logger(),
                "自动连接: 节点 %d -> 节点 %d (距离: %.2f m, 双向: %s)",
                last_added_node_id_, new_id, weight,
                bidirectional ? "是" : "否");
  } else {
    RCLCPP_INFO(node_->get_logger(), "第一个节点，无需连接");
  }

  last_added_node_id_ = new_id;

  createInteractiveMarker(new_node);

  geometry_msgs::msg::PointStamped point_msg;
  point_msg.header.frame_id = "map";
  point_msg.header.stamp = node_->now();
  point_msg.point.x = x;
  point_msg.point.y = y;
  point_msg.point.z = adjusted_z;

  point_pub_->publish(point_msg);

  updateVisualization();

  RCLCPP_INFO(
      node_->get_logger(),
      "添加节点 #%d: %s at (%.2f, %.2f, %.2f) [原始z: %.2f, 抬高0.3m后: %.2f]",
      new_id, label.c_str(), x, y, adjusted_z, z, adjusted_z);
}

void TopologyTool::addEdge(int from_id, int to_id) {
  if (from_id < 0 || from_id >= static_cast<int>(nodes_.size()) || to_id < 0 ||
      to_id >= static_cast<int>(nodes_.size())) {
    RCLCPP_WARN(node_->get_logger(), "无效的节点ID: %d -> %d", from_id, to_id);
    return;
  }

  for (const auto& edge : edges_) {
    if ((edge.from_id == from_id && edge.to_id == to_id) ||
        (edge.bidirectional && edge.from_id == to_id &&
         edge.to_id == from_id)) {
      RCLCPP_WARN(node_->get_logger(), "节点 %d 和 %d 之间已存在连接", from_id,
                  to_id);
      return;
    }
  }

  int new_edge_id = edges_.size();
  double weight = calculateDistance(nodes_[from_id], nodes_[to_id]);
  bool bidirectional = bidirectional_property_->getBool();

  TopologyEdge new_edge(new_edge_id, from_id, to_id, weight, "manual",
                        bidirectional);
  edges_.push_back(new_edge);

  updateVisualization();

  RCLCPP_INFO(node_->get_logger(),
              "添加连接 #%d: 节点 %d -> 节点 %d (距离: %.2f m, 双向: %s)",
              new_edge_id, from_id, to_id, weight, bidirectional ? "是" : "否");
}

void TopologyTool::removeNode(int node_id) {
  if (node_id < 0 || node_id >= static_cast<int>(nodes_.size())) {
    return;
  }

  edges_.erase(std::remove_if(edges_.begin(), edges_.end(),
                              [node_id](const TopologyEdge& edge) {
                                return edge.from_id == node_id ||
                                       edge.to_id == node_id;
                              }),
               edges_.end());

  nodes_.erase(nodes_.begin() + node_id);

  for (auto& edge : edges_) {
    if (edge.from_id > node_id) {
      edge.from_id--;
    }

    if (edge.to_id > node_id) {
      edge.to_id--;
    }
  }

  for (size_t i = 0; i < nodes_.size(); ++i) {
    nodes_[i].id = i;
  }

  for (size_t i = 0; i < edges_.size(); ++i) {
    edges_[i].id = i;
  }

  if (last_added_node_id_ == node_id) {
    last_added_node_id_ =
        nodes_.empty() ? -1 : static_cast<int>(nodes_.size()) - 1;
  } else if (last_added_node_id_ > node_id) {
    last_added_node_id_--;
  }

  updateVisualization();

  RCLCPP_INFO(node_->get_logger(), "删除节点 #%d", node_id);
}

void TopologyTool::removeEdge(int edge_id) {
  if (edge_id >= 0 && edge_id < static_cast<int>(edges_.size())) {
    edges_.erase(edges_.begin() + edge_id);

    for (size_t i = 0; i < edges_.size(); ++i) {
      edges_[i].id = i;
    }

    updateVisualization();

    RCLCPP_INFO(node_->get_logger(), "删除边 #%d", edge_id);
  }
}

void TopologyTool::createInteractiveMarker(const TopologyNode& node) {
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = "map";
  int_marker.header.stamp = node_->now();

  std::stringstream ss;
  ss << "node_" << node.id;

  int_marker.name = ss.str();
  int_marker.description = node.label;

  int_marker.pose.position.x = node.x;
  int_marker.pose.position.y = node.y;
  int_marker.pose.position.z = node.z;
  int_marker.pose.orientation.w = 1.0;

  visualization_msgs::msg::InteractiveMarkerControl control;
  control.always_visible = true;
  control.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::BUTTON;

  visualization_msgs::msg::Marker marker;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 0.8;

  control.markers.push_back(marker);
  int_marker.controls.push_back(control);

  server_->insert(int_marker);
  server_->setCallback(int_marker.name,
                       std::bind(&TopologyTool::processMarkerFeedback, this,
                                 std::placeholders::_1));

  server_->applyChanges();
}

void TopologyTool::processMarkerFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr&
        feedback) {
  RCLCPP_INFO(node_->get_logger(), "交互标记反馈: %s",
              feedback->marker_name.c_str());

  if (feedback->event_type ==
      visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK) {
    std::string marker_name = feedback->marker_name;

    if (marker_name.find("node_") == 0) {
      int node_id = std::stoi(marker_name.substr(5));
      createContextMenu(node_id);
    }
  }
}

void TopologyTool::createContextMenu(int node_id) {
  RCLCPP_INFO(node_->get_logger(), "节点 #%d 的上下文菜单", node_id);

  bool ok;

  QString current_label = QString::fromUtf8(nodes_[node_id].label.c_str());

  QString new_label = QInputDialog::getText(
      nullptr, QString::fromUtf8("编辑节点"), QString::fromUtf8("节点标签:"),
      QLineEdit::Normal, current_label, &ok);

  if (ok && !new_label.isEmpty()) {
    nodes_[node_id].label = new_label.toUtf8().toStdString();

    updateVisualization();

    RCLCPP_INFO(node_->get_logger(), "节点 #%d 标签更新为: %s", node_id,
                nodes_[node_id].label.c_str());
  }
}

void TopologyTool::updateVisualization() {
  publishNodes();
  publishEdges();
}

void TopologyTool::publishNodes() {
  visualization_msgs::msg::MarkerArray markers;

  for (size_t i = 0; i < nodes_.size(); ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = node_->now();
    marker.ns = "topology_nodes";
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = nodes_[i].x;
    marker.pose.position.y = nodes_[i].y;
    marker.pose.position.z = nodes_[i].z;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.4;
    marker.scale.y = 0.4;
    marker.scale.z = 0.4;

    if (static_cast<int>(i) == connecting_from_id_) {
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;

      marker.scale.x = 0.5;
      marker.scale.y = 0.5;
      marker.scale.z = 0.5;
    } else if (static_cast<int>(i) == selected_node_id_) {
      marker.color.r = 1.0;
      marker.color.g = 0.5;
      marker.color.b = 0.0;
      marker.color.a = 1.0;
    } else {
      marker.color.r = 0.0;
      marker.color.g = 0.8;
      marker.color.b = 1.0;
      marker.color.a = 0.9;
    }

    markers.markers.push_back(marker);

    visualization_msgs::msg::Marker text_marker;
    text_marker.header.frame_id = "map";
    text_marker.header.stamp = node_->now();
    text_marker.ns = "topology_labels";
    text_marker.id = i;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;

    text_marker.pose.position.x = nodes_[i].x;
    text_marker.pose.position.y = nodes_[i].y;
    text_marker.pose.position.z = nodes_[i].z + 0.5;
    text_marker.pose.orientation.w = 1.0;

    text_marker.scale.z = 0.3;
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;

    text_marker.text = std::to_string(nodes_[i].id) + ": " + nodes_[i].label;

    markers.markers.push_back(text_marker);
  }

  marker_pub_->publish(markers);
}

void TopologyTool::publishEdges() {
  visualization_msgs::msg::MarkerArray edge_markers;

  for (size_t i = 0; i < edges_.size(); ++i) {
    const auto& edge = edges_[i];

    if (edge.from_id >= 0 && edge.from_id < static_cast<int>(nodes_.size()) &&
        edge.to_id >= 0 && edge.to_id < static_cast<int>(nodes_.size())) {
      visualization_msgs::msg::Marker edge_marker;
      edge_marker.header.frame_id = "map";
      edge_marker.header.stamp = node_->now();
      edge_marker.ns = "topology_edges";
      edge_marker.id = i * 2;
      edge_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      edge_marker.action = visualization_msgs::msg::Marker::ADD;

      geometry_msgs::msg::Point p1;
      geometry_msgs::msg::Point p2;

      p1.x = nodes_[edge.from_id].x;
      p1.y = nodes_[edge.from_id].y;
      p1.z = nodes_[edge.from_id].z + 0.1;

      p2.x = nodes_[edge.to_id].x;
      p2.y = nodes_[edge.to_id].y;
      p2.z = nodes_[edge.to_id].z + 0.1;

      edge_marker.points.push_back(p1);
      edge_marker.points.push_back(p2);

      edge_marker.scale.x = 0.08;

      edge_marker.color.r = 1.0;
      edge_marker.color.g = 0.0;
      edge_marker.color.b = 0.0;
      edge_marker.color.a = 0.8;

      edge_markers.markers.push_back(edge_marker);

      if (!edge.bidirectional) {
        visualization_msgs::msg::Marker arrow_marker;
        arrow_marker.header.frame_id = "map";
        arrow_marker.header.stamp = node_->now();
        arrow_marker.ns = "topology_arrows";
        arrow_marker.id = i * 2 + 1;
        arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
        arrow_marker.action = visualization_msgs::msg::Marker::ADD;

        double mid_x = (p1.x + p2.x) / 2.0;
        double mid_y = (p1.y + p2.y) / 2.0;
        double mid_z = (p1.z + p2.z) / 2.0;

        arrow_marker.pose.position.x = mid_x;
        arrow_marker.pose.position.y = mid_y;
        arrow_marker.pose.position.z = mid_z;

        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        double dz = p2.z - p1.z;
        double norm = std::sqrt(dx * dx + dy * dy + dz * dz);

        if (norm > 0.001) {
          dx /= norm;
          dy /= norm;
          dz /= norm;

          double yaw = std::atan2(dy, dx);
          double pitch = std::asin(-dz);

          arrow_marker.pose.orientation.x = 0.0;
          arrow_marker.pose.orientation.y = std::sin(pitch / 2.0);
          arrow_marker.pose.orientation.z =
              std::sin(yaw / 2.0) * std::cos(pitch / 2.0);
          arrow_marker.pose.orientation.w =
              std::cos(yaw / 2.0) * std::cos(pitch / 2.0);
        } else {
          arrow_marker.pose.orientation.w = 1.0;
        }

        arrow_marker.scale.x = 0.3;
        arrow_marker.scale.y = 0.1;
        arrow_marker.scale.z = 0.1;

        arrow_marker.color = edge_marker.color;

        edge_markers.markers.push_back(arrow_marker);
      }

      visualization_msgs::msg::Marker weight_marker;
      weight_marker.header.frame_id = "map";
      weight_marker.header.stamp = node_->now();
      weight_marker.ns = "edge_weights";
      weight_marker.id = i;
      weight_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      weight_marker.action = visualization_msgs::msg::Marker::ADD;

      weight_marker.pose.position.x = (p1.x + p2.x) / 2.0;
      weight_marker.pose.position.y = (p1.y + p2.y) / 2.0;
      weight_marker.pose.position.z = (p1.z + p2.z) / 2.0 + 0.2;
      weight_marker.pose.orientation.w = 1.0;

      weight_marker.scale.z = 0.2;
      weight_marker.color.r = 1.0;
      weight_marker.color.g = 1.0;
      weight_marker.color.b = 0.0;
      weight_marker.color.a = 0.8;

      std::ostringstream oss;
      oss << std::fixed << std::setprecision(2) << edge.weight << "m";
      weight_marker.text = oss.str();

      edge_markers.markers.push_back(weight_marker);
    }
  }

  marker_pub_->publish(edge_markers);
}

double TopologyTool::calculateDistance(const TopologyNode& n1,
                                       const TopologyNode& n2) {
  double dx = n1.x - n2.x;
  double dy = n1.y - n2.y;
  double dz = n1.z - n2.z;

  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

int TopologyTool::findNearestNode(double x, double y, double z,
                                  double max_distance) {
  int nearest_id = -1;
  double min_distance = max_distance;

  for (size_t i = 0; i < nodes_.size(); ++i) {
    double distance = calculateDistance(TopologyNode(-1, x, y, z), nodes_[i]);

    if (distance < min_distance) {
      min_distance = distance;
      nearest_id = i;
    }
  }

  return nearest_id;
}

bool TopologyTool::getPoint(rviz_common::ViewportMouseEvent& event,
                            Ogre::Vector3& point_out) {
  if (!cloud_received_ || current_cloud_->empty()) {
    RCLCPP_WARN(node_->get_logger(),
                "未接收到可行区域点云数据，无法添加/选择节点。请检查 YAML 中 "
                "topology_tool.pointcloud_topic 对应的话题是否正在发布。");
    return false;
  }

  if (event.panel == nullptr || event.panel->getRenderWindow() == nullptr) {
    RCLCPP_WARN(node_->get_logger(), "无法获取RViz渲染窗口，无法添加节点。");
    return false;
  }

  Ogre::Viewport* viewport =
      rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(
          event.panel->getRenderWindow());

  if (viewport == nullptr || viewport->getCamera() == nullptr) {
    RCLCPP_WARN(node_->get_logger(),
                "无法获取RViz viewport/camera，无法添加节点。");
    return false;
  }

  Ogre::Ray mouse_ray = viewport->getCamera()->getCameraToViewportRay(
      event.x / static_cast<float>(viewport->getActualWidth()),
      event.y / static_cast<float>(viewport->getActualHeight()));

  float min_distance_to_ray = std::numeric_limits<float>::max();
  pcl::PointXYZ best_point;
  bool found_valid_point = false;

  for (size_t i = 0; i < current_cloud_->points.size(); ++i) {
    const auto& cloud_point = current_cloud_->points[i];

    if (!std::isfinite(cloud_point.x) || !std::isfinite(cloud_point.y) ||
        !std::isfinite(cloud_point.z)) {
      continue;
    }

    Ogre::Vector3 ogre_point(cloud_point.x, cloud_point.y, cloud_point.z);

    Ogre::Vector3 ray_to_point = ogre_point - mouse_ray.getOrigin();
    float projection_length = ray_to_point.dotProduct(mouse_ray.getDirection());

    if (projection_length < 0.1f) {
      continue;
    }

    Ogre::Vector3 closest_on_ray =
        mouse_ray.getOrigin() + projection_length * mouse_ray.getDirection();

    float distance_to_ray = ogre_point.distance(closest_on_ray);

    float selection_threshold = selection_threshold_property_->getFloat();

    if (distance_to_ray < selection_threshold &&
        distance_to_ray < min_distance_to_ray) {
      min_distance_to_ray = distance_to_ray;
      best_point = cloud_point;
      found_valid_point = true;
    }
  }

  if (found_valid_point) {
    point_out.x = best_point.x;
    point_out.y = best_point.y;
    point_out.z = best_point.z;

    RCLCPP_INFO(
        node_->get_logger(),
        "选择了可行区域点云中的点: (%.3f, %.3f, %.3f), 与射线距离: %.3f m",
        point_out.x, point_out.y, point_out.z, min_distance_to_ray);

    return true;
  }

  RCLCPP_WARN(node_->get_logger(),
              "未在可行区域点云中找到合适的点。请点击更接近点云的位置。");

  return false;
}

bool TopologyTool::findNearestCloudPoint(const Ogre::Vector3& ray_point,
                                         Ogre::Vector3& cloud_point) {
  if (!cloud_received_ || current_cloud_->empty()) {
    return false;
  }

  pcl::PointXYZ search_point;
  search_point.x = ray_point.x;
  search_point.y = ray_point.y;
  search_point.z = ray_point.z;

  std::vector<int> point_indices(1);
  std::vector<float> point_distances(1);

  if (kdtree_->nearestKSearch(search_point, 1, point_indices, point_distances) >
      0) {
    const auto& nearest_point = current_cloud_->points[point_indices[0]];

    cloud_point.x = nearest_point.x;
    cloud_point.y = nearest_point.y;
    cloud_point.z = nearest_point.z;

    return true;
  }

  return false;
}

void TopologyTool::pointcloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg) {
  pcl::fromROSMsg(*cloud_msg, *current_cloud_);

  if (!current_cloud_->empty()) {
    kdtree_->setInputCloud(current_cloud_);
    cloud_received_ = true;

    RCLCPP_INFO_ONCE(node_->get_logger(), "接收到点云数据，共 %lu 个点",
                     current_cloud_->size());
  }
}

void TopologyTool::updateProperties() {
  current_mode_ = static_cast<ToolMode>(mode_property_->getOptionInt());

  RCLCPP_INFO(node_->get_logger(), "工具模式更改为: %d", current_mode_);
}

void TopologyTool::clearAll() {
  nodes_.clear();
  edges_.clear();

  server_->clear();
  server_->applyChanges();

  updateVisualization();

  connecting_from_id_ = -1;
  selected_node_id_ = -1;
  last_added_node_id_ = -1;

  RCLCPP_INFO(node_->get_logger(), "清除所有节点和边，重置自动连接状态");
}

void TopologyTool::saveTopology() {
  std::string filename = save_file_property_->getStdString();

  try {
    std::ofstream file(filename, std::ios::out | std::ios::binary);

    if (!file.is_open()) {
      RCLCPP_ERROR(node_->get_logger(), "无法创建文件: %s", filename.c_str());
      return;
    }

    file << "\xEF\xBB\xBF";

    file << "# 拓扑地图文件 (UTF-8编码)\n";
    file << "# 格式: 节点ID X Y Z 标签\n";
    file << "# Topology Nodes\n";

    for (const auto& node : nodes_) {
      file << node.id << " " << std::fixed << std::setprecision(6) << node.x
           << " " << node.y << " " << node.z << " " << node.label << "\n";
    }

    file << "# 格式: 边ID 起始节点ID 结束节点ID 权重 类型 双向\n";
    file << "# Topology Edges\n";

    for (const auto& edge : edges_) {
      file << edge.id << " " << edge.from_id << " " << edge.to_id << " "
           << std::fixed << std::setprecision(6) << edge.weight << " "
           << edge.type << " " << edge.bidirectional << "\n";
    }

    file.close();

    RCLCPP_INFO(node_->get_logger(), "拓扑地图已保存到: %s", filename.c_str());
    RCLCPP_INFO(node_->get_logger(), "保存了 %lu 个节点和 %lu 条边",
                nodes_.size(), edges_.size());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "保存拓扑地图时出错: %s", e.what());
  }
}

void TopologyTool::loadTopology() {
  std::string filename = save_file_property_->getStdString();

  try {
    std::ifstream file(filename, std::ios::in | std::ios::binary);

    if (!file.is_open()) {
      RCLCPP_ERROR(node_->get_logger(), "无法打开文件: %s", filename.c_str());
      return;
    }

    clearAll();

    std::string line;
    bool reading_nodes = false;
    bool reading_edges = false;

    char bom[3];
    file.read(bom, 3);

    if (!(bom[0] == '\xEF' && bom[1] == '\xBB' && bom[2] == '\xBF')) {
      file.seekg(0);
    }

    while (std::getline(file, line)) {
      if (line.empty() || line[0] == '#') {
        if (line.find("Topology Nodes") != std::string::npos) {
          reading_nodes = true;
          reading_edges = false;
          continue;
        }

        if (line.find("Topology Edges") != std::string::npos) {
          reading_nodes = false;
          reading_edges = true;
          continue;
        }

        continue;
      }

      std::istringstream iss(line);

      if (reading_nodes) {
        int id;
        double x;
        double y;
        double z;
        std::string label;

        if (iss >> id >> x >> y >> z) {
          std::string remaining;
          std::getline(iss, remaining);

          if (!remaining.empty() && remaining[0] == ' ') {
            label = remaining.substr(1);
          } else {
            label = remaining;
          }

          if (label.empty()) {
            label = "节点_" + std::to_string(id);
          }

          TopologyNode node(id, x, y, z, label);
          nodes_.push_back(node);

          createInteractiveMarker(node);
        }
      } else if (reading_edges) {
        int id;
        int from_id;
        int to_id;
        double weight;
        std::string type;
        bool bidirectional;

        if (iss >> id >> from_id >> to_id >> weight >> type >> bidirectional) {
          TopologyEdge edge(id, from_id, to_id, weight, type, bidirectional);
          edges_.push_back(edge);
        }
      }
    }

    file.close();

    updateVisualization();

    RCLCPP_INFO(node_->get_logger(), "成功加载拓扑地图: %lu 个节点, %lu 条边",
                nodes_.size(), edges_.size());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "加载拓扑地图时出错: %s", e.what());
  }
}

}  // namespace topology_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(topology_rviz_plugin::TopologyTool, rviz_common::Tool)