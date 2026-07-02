#ifndef TOPOLOGY_RVIZ_PLUGIN_TOPOLOGY_TOOL_HPP
#define TOPOLOGY_RVIZ_PLUGIN_TOPOLOGY_TOOL_HPP

#include <rclcpp/rclcpp.hpp>

#include <rviz_common/tool.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/property.hpp>

#include <OgreVector3.h>
#include <OgreRay.h>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>

#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include <QObject>
#include <memory>
#include <stdexcept>
#include <mutex>
#include <string>
#include <vector>

namespace topology_rviz_plugin
{

struct TopologyNode
{
  int id;
  double x, y, z;
  std::string label;

  TopologyNode(int _id, double _x, double _y, double _z, const std::string& _label = "")
  : id(_id), x(_x), y(_y), z(_z), label(_label) {}
};

struct TopologyEdge
{
  int id;
  int from_id;
  int to_id;
  double weight;
  std::string type;
  bool bidirectional;

  TopologyEdge(
    int _id, int _from, int _to, double _weight,
    const std::string& _type = "normal", bool _bidir = true)
  : id(_id), from_id(_from), to_id(_to), weight(_weight), type(_type), bidirectional(_bidir) {}
};

class TopologyTool : public rviz_common::Tool
{
  Q_OBJECT

public:
  TopologyTool();
  ~TopologyTool() override;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;

  int processMouseEvent(rviz_common::ViewportMouseEvent& event) override;

private Q_SLOTS:
  void updateProperties();
  void clearAll();
  void saveTopology();
  void loadTopology();
  void connectNodesFromProperties();
  void loadEditNodeToProperties();
  void applyEditNodeFromProperties();
  void deleteNodeFromProperties();

private:
  // 鼠标事件处理
  int processMousePress(rviz_common::ViewportMouseEvent& event);
  int processMouseRelease(rviz_common::ViewportMouseEvent& event);
  int processMouseMove(rviz_common::ViewportMouseEvent& event);

  // 节点和边管理
  void addNode(double x, double y, double z);
  void addEdge(int from_id, int to_id);
  void removeNode(int node_id);
  void removeEdge(int edge_id);

  // 可视化
  void updateVisualization();
  void clearVisualization();
  void publishNodes();
  void publishEdges();
  void rebuildInteractiveMarkers();

  // 交互标记相关
  void createInteractiveMarker(const TopologyNode& node);
  void processMarkerFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback);
  void createContextMenu(int node_id);

  // 工具函数
  double calculateDistance(const TopologyNode& n1, const TopologyNode& n2);
  int nextNodeId() const;
  int nextEdgeId() const;
  int findNodeIndexById(int node_id) const;
  int findEdgeIndexById(int edge_id) const;
  void updateEdgeWeightsForNode(int node_id);
  bool getPoint(rviz_common::ViewportMouseEvent& event, Ogre::Vector3& point_out);
  bool findNearestCloudPoint(const Ogre::Vector3& ray_point, Ogre::Vector3& cloud_point);
  bool loadPointCloudFromPcd(const std::string& pcd_file);

  // ROS2相关
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_pub_;

  // 点云数据
  pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_;
  bool cloud_received_;

  // 交互标记服务器
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  interactive_markers::MenuHandler menu_handler_;

  // 数据存储
  std::vector<TopologyNode> nodes_;
  std::vector<TopologyEdge> edges_;

  // 状态变量
  int selected_node_id_;
  int connecting_from_id_;
  int last_added_node_id_;

  // 属性
  rviz_common::properties::StringProperty* node_label_property_;
  rviz_common::properties::FloatProperty* selection_threshold_property_;
  rviz_common::properties::BoolProperty* bidirectional_property_;
  rviz_common::properties::StringProperty* save_file_property_;
  rviz_common::properties::IntProperty* connect_from_id_property_;
  rviz_common::properties::IntProperty* connect_to_id_property_;
  rviz_common::properties::BoolProperty* connect_button_property_;
  rviz_common::properties::IntProperty* edit_node_id_property_;
  rviz_common::properties::StringProperty* edit_label_property_;
  rviz_common::properties::FloatProperty* edit_x_property_;
  rviz_common::properties::FloatProperty* edit_y_property_;
  rviz_common::properties::FloatProperty* edit_z_property_;
  rviz_common::properties::BoolProperty* load_edit_button_property_;
  rviz_common::properties::BoolProperty* apply_edit_button_property_;
  rviz_common::properties::IntProperty* delete_node_id_property_;
  rviz_common::properties::BoolProperty* delete_button_property_;
};

}  // namespace topology_rviz_plugin

#endif  // TOPOLOGY_RVIZ_PLUGIN_TOPOLOGY_TOOL_HPP
