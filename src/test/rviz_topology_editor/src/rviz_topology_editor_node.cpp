#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "rviz_topology_editor/rviz_topology_editor_node.hpp"

namespace rviz_topology_editor {
namespace {

struct SavedPoint {
  int id{0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double qx{0.0};
  double qy{0.0};
  double qz{0.0};
  double qw{1.0};
  std::string label;
};

std::string makeTargetPointsPath(const std::string &output_file) {
  const auto extension = output_file.find_last_of('.');
  if (extension == std::string::npos) {
    return output_file + "_target_points.txt";
  }
  return output_file.substr(0, extension) + "_target_points.txt";
}

void ensureParentDirectory(const std::string &file_path) {
  const auto parent = std::filesystem::path(file_path).parent_path();
  if (!parent.empty()) {
    std::filesystem::create_directories(parent);
  }
}

void replaceFile(const std::filesystem::path &temporary_path,
                 const std::filesystem::path &final_path) {
  std::error_code error;
  std::filesystem::rename(temporary_path, final_path, error);
  if (!error) {
    return;
  }

  std::filesystem::remove(final_path, error);
  error.clear();
  std::filesystem::rename(temporary_path, final_path, error);
  if (error) {
    throw std::runtime_error("cannot replace " + final_path.string() + ": " +
                             error.message());
  }
}

} // namespace

class RvizTopologyEditor : public rclcpp::Node {
public:
  RvizTopologyEditor() : Node("rviz_topology_editor") {
    declareParameters();
    readParameters();
    validateParameters();

    original_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    display_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    loadPcd();

    auto static_qos = rclcpp::QoS(1).transient_local().reliable();
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic_,
                                                                 static_qos);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        marker_topic_, static_qos);

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        goal_topic_, rclcpp::QoS(10),
        std::bind(&RvizTopologyEditor::goalCallback, this,
                  std::placeholders::_1));

    undo_service_ = create_service<std_srvs::srv::Trigger>(
        "~/undo_last", std::bind(&RvizTopologyEditor::undoLast, this,
                                 std::placeholders::_1, std::placeholders::_2));
    clear_service_ = create_service<std_srvs::srv::Trigger>(
        "~/clear", std::bind(&RvizTopologyEditor::clearPoints, this,
                             std::placeholders::_1, std::placeholders::_2));
    save_service_ = create_service<std_srvs::srv::Trigger>(
        "~/save", std::bind(&RvizTopologyEditor::saveAll, this,
                            std::placeholders::_1, std::placeholders::_2));

    publishCloud();
    publishMarkers();

    RCLCPP_INFO(get_logger(), "RViz topology editor ready");
    RCLCPP_INFO(get_logger(), "PCD: %s", pcd_file_.c_str());
    RCLCPP_INFO(get_logger(), "2D Goal topic: %s", goal_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Output: %s", output_file_.c_str());
    RCLCPP_INFO(get_logger(), "Target output: %s", target_points_file_.c_str());
    RCLCPP_INFO(get_logger(), "Connected topology: %s",
                connected_topology_file_.c_str());
    RCLCPP_INFO(get_logger(), "Task file: %s", task_file_.c_str());
  }

private:
  void declareParameters() {
    declare_parameter<std::string>("pcd_file",
                                   "/robotfs/data/slam_data/map/globalmap.pcd");
    declare_parameter<std::string>(
        "output_file",
        "/robotfs/data/slam_data/topology_map/generated_map.txt");
    declare_parameter<std::string>(
        "connected_topology_file",
        "/robotfs/data/slam_data/topology_map/connected_topology_map.txt");
    declare_parameter<std::string>("task_file",
                                   "/robotfs/data/slam_data/task/test.txt");
    declare_parameter<std::string>("frame_id", "map");
    declare_parameter<std::string>("goal_topic",
                                   "/rviz_topology_editor/goal_pose");
    declare_parameter<std::string>("cloud_topic",
                                   "/rviz_topology_editor/map_cloud");
    declare_parameter<std::string>("marker_topic",
                                   "/rviz_topology_editor/markers");
    declare_parameter<bool>("flatten_cloud_for_display", true);
    declare_parameter<double>("marker_arrow_length", 2.0);
    declare_parameter<double>("marker_arrow_width", 0.4);
    declare_parameter<double>("marker_text_scale", 1.0);
    declare_parameter<double>("marker_text_height_offset", 1.2);
    declare_parameter<double>("edge_width", 0.15);
    declare_parameter<bool>("auto_save_raw_points", false);
    declare_parameter<int64_t>("task_wait_sec", 0);
    declare_parameter<int64_t>("task_publish_data", 1);
  }

  void readParameters() {
    pcd_file_ = get_parameter("pcd_file").as_string();
    output_file_ = get_parameter("output_file").as_string();
    target_points_file_ = makeTargetPointsPath(output_file_);
    connected_topology_file_ =
        get_parameter("connected_topology_file").as_string();
    task_file_ = get_parameter("task_file").as_string();
    frame_id_ = get_parameter("frame_id").as_string();
    goal_topic_ = get_parameter("goal_topic").as_string();
    cloud_topic_ = get_parameter("cloud_topic").as_string();
    marker_topic_ = get_parameter("marker_topic").as_string();
    flatten_cloud_for_display_ =
        get_parameter("flatten_cloud_for_display").as_bool();
    marker_arrow_length_ =
        get_parameter("marker_arrow_length").as_double();
    marker_arrow_width_ = get_parameter("marker_arrow_width").as_double();
    marker_text_scale_ = get_parameter("marker_text_scale").as_double();
    marker_text_height_offset_ =
        get_parameter("marker_text_height_offset").as_double();
    edge_width_ = get_parameter("edge_width").as_double();
    auto_save_raw_points_ = get_parameter("auto_save_raw_points").as_bool();
    task_wait_sec_ = get_parameter("task_wait_sec").as_int();
    task_publish_data_ = get_parameter("task_publish_data").as_int();
  }

  void validateParameters() const {
    if (pcd_file_.empty() || output_file_.empty() ||
        connected_topology_file_.empty() || task_file_.empty() ||
        frame_id_.empty()) {
      throw std::invalid_argument(
          "pcd_file, output files and frame_id must not be empty");
    }
    if (marker_arrow_length_ <= 0.0 || marker_arrow_width_ <= 0.0 ||
        marker_text_scale_ <= 0.0 || marker_text_height_offset_ < 0.0 ||
        edge_width_ <= 0.0) {
      throw std::invalid_argument(
          "marker sizes and edge_width must be positive; "
          "marker_text_height_offset must be non-negative");
    }
  }

  void loadPcd() {
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_, *original_cloud_) != 0) {
      throw std::runtime_error("cannot load PCD: " + pcd_file_);
    }
    if (original_cloud_->empty()) {
      throw std::runtime_error("PCD is empty: " + pcd_file_);
    }

    *display_cloud_ = *original_cloud_;
    if (flatten_cloud_for_display_) {
      for (auto &point : display_cloud_->points) {
        if (std::isfinite(point.z)) {
          point.z = 0.0F;
        }
      }
    }

    RCLCPP_INFO(get_logger(), "Loaded PCD points: %zu",
                original_cloud_->size());
  }

  void publishCloud() {
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(*display_cloud_, message);
    message.header.stamp = now();
    message.header.frame_id = frame_id_;
    cloud_pub_->publish(message);
  }

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr message) {
    if (!message) {
      return;
    }
    if (!message->header.frame_id.empty() &&
        message->header.frame_id != frame_id_) {
      RCLCPP_ERROR(get_logger(), "Rejected point in frame '%s'; expected '%s'",
                   message->header.frame_id.c_str(), frame_id_.c_str());
      return;
    }

    SavedPoint point;
    point.id = next_id_;
    point.x = message->pose.position.x;
    point.y = message->pose.position.y;
    point.z = 0.0;
    point.qx = message->pose.orientation.x;
    point.qy = message->pose.orientation.y;
    point.qz = message->pose.orientation.z;
    point.qw = message->pose.orientation.w;
    point.label = std::to_string(point.id) + "号";

    const double quaternion_norm =
        std::sqrt(point.qx * point.qx + point.qy * point.qy +
                  point.qz * point.qz + point.qw * point.qw);
    if (quaternion_norm < 1e-6) {
      point.qx = 0.0;
      point.qy = 0.0;
      point.qz = 0.0;
      point.qw = 1.0;
    } else {
      point.qx /= quaternion_norm;
      point.qy /= quaternion_norm;
      point.qz /= quaternion_norm;
      point.qw /= quaternion_norm;
    }

    points_.push_back(point);
    ++next_id_;
    show_edges_ = false;

    if (auto_save_raw_points_) {
      try {
        saveRawPointFiles();
      } catch (const std::exception &error) {
        points_.pop_back();
        --next_id_;
        RCLCPP_ERROR(get_logger(), "Failed to auto-save point: %s",
                     error.what());
        return;
      }
    }

    publishMarkers();
    RCLCPP_INFO(get_logger(),
                "Added point id=%d label=%s xyz=(%.3f, %.3f, %.3f), "
                "pending save=%s",
                point.id, point.label.c_str(), point.x, point.y, point.z,
                auto_save_raw_points_ ? "no" : "yes");
  }

  void writeTopologyFile(const std::filesystem::path &path) const {
    std::ofstream output(path, std::ios::trunc);
    if (!output.is_open()) {
      throw std::runtime_error("cannot open " + path.string());
    }

    output << "# 拓扑地图文件 (UTF-8编码)\n";
    output << "# 格式: 节点ID X Y Z 标签\n";
    output << "# Topology Nodes\n";
    for (const auto &point : points_) {
      output << point.id << " " << std::fixed << std::setprecision(6) << point.x
             << " " << point.y << " " << point.z << " " << point.label << '\n';
    }
    output.flush();
    if (!output) {
      throw std::runtime_error("failed writing " + path.string());
    }
  }

  void writeTargetPointsFile(const std::filesystem::path &path) const {
    std::ofstream output(path, std::ios::trunc);
    if (!output.is_open()) {
      throw std::runtime_error("cannot open " + path.string());
    }

    for (const auto &point : points_) {
      output << point.id << " " << std::fixed << std::setprecision(6) << point.x
             << " " << point.y << " " << point.z << " " << point.qx << " "
             << point.qy << " " << point.qz << " " << point.qw << " "
             << point.label << '\n';
    }
    output.flush();
    if (!output) {
      throw std::runtime_error("failed writing " + path.string());
    }
  }

  void writeConnectedTopologyFile(const std::filesystem::path &path) const {
    std::ofstream output(path, std::ios::trunc);
    if (!output.is_open()) {
      throw std::runtime_error("cannot open " + path.string());
    }

    output << "# 拓扑地图文件 (UTF-8编码)\n";
    output << "# 格式: 节点ID X Y Z 标签\n";
    output << "# Topology Nodes\n";
    output << "# 连接方式: 按节点 ID 顺序连接\n";
    output << "# 新增连接数: " << (points_.empty() ? 0U : points_.size() - 1U)
           << "\n";

    for (const auto &point : points_) {
      output << point.id << " " << std::fixed << std::setprecision(6) << point.x
             << " " << point.y << " " << point.z << " " << point.label << '\n';
    }

    output << "# 格式: 边ID 起始节点ID 结束节点ID 权重 类型 双向\n";
    output << "# Topology Edges\n";
    output << "# 新增连接 (按节点 ID 顺序自动生成)\n";

    for (std::size_t index = 0; index + 1U < points_.size(); ++index) {
      const auto &from = points_[index];
      const auto &to = points_[index + 1U];
      const double dx = to.x - from.x;
      const double dy = to.y - from.y;
      const double dz = to.z - from.z;
      const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

      output << index << " " << from.id << " " << to.id << " " << std::fixed
             << std::setprecision(6) << distance << " auto 1\n";
    }

    output.flush();
    if (!output) {
      throw std::runtime_error("failed writing " + path.string());
    }
  }

  void writeTaskFile(const std::filesystem::path &path) const {
    std::ofstream output(path, std::ios::trunc);
    if (!output.is_open()) {
      throw std::runtime_error("cannot open " + path.string());
    }

    output << "# 任务列表 (UTF-8编码)\n";
    output << "# 格式：目标点ID 等待延时 抵达后发布的数据\n";
    for (const auto &point : points_) {
      output << point.id << " " << task_wait_sec_ << " " << task_publish_data_
             << '\n';
    }

    output.flush();
    if (!output) {
      throw std::runtime_error("failed writing " + path.string());
    }
  }

  void saveRawPointFiles() const {
    ensureParentDirectory(output_file_);
    ensureParentDirectory(target_points_file_);

    const std::filesystem::path topology_path(output_file_);
    const std::filesystem::path target_path(target_points_file_);
    const auto topology_temporary =
        std::filesystem::path(output_file_ + ".tmp");
    const auto target_temporary =
        std::filesystem::path(target_points_file_ + ".tmp");

    writeTopologyFile(topology_temporary);
    writeTargetPointsFile(target_temporary);
    replaceFile(topology_temporary, topology_path);
    replaceFile(target_temporary, target_path);
  }

  void saveAllOutputs() const {
    saveRawPointFiles();
    ensureParentDirectory(connected_topology_file_);
    ensureParentDirectory(task_file_);

    const std::filesystem::path connected_path(connected_topology_file_);
    const std::filesystem::path task_path(task_file_);
    const auto connected_temporary =
        std::filesystem::path(connected_topology_file_ + ".tmp");
    const auto task_temporary = std::filesystem::path(task_file_ + ".tmp");

    writeConnectedTopologyFile(connected_temporary);
    writeTaskFile(task_temporary);
    replaceFile(connected_temporary, connected_path);
    replaceFile(task_temporary, task_path);
  }

  void publishMarkers() {
    visualization_msgs::msg::MarkerArray markers;

    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = frame_id_;
    clear.header.stamp = now();
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);

    if (show_edges_ && points_.size() >= 2U) {
      visualization_msgs::msg::Marker edges;
      edges.header.frame_id = frame_id_;
      edges.header.stamp = now();
      edges.ns = "rviz_topology_edges";
      edges.id = 0;
      edges.type = visualization_msgs::msg::Marker::LINE_LIST;
      edges.action = visualization_msgs::msg::Marker::ADD;
      edges.pose.orientation.w = 1.0;
      edges.scale.x = edge_width_;
      edges.color.r = 0.0F;
      edges.color.g = 0.7F;
      edges.color.b = 1.0F;
      edges.color.a = 1.0F;

      for (std::size_t index = 0; index + 1U < points_.size(); ++index) {
        geometry_msgs::msg::Point from;
        from.x = points_[index].x;
        from.y = points_[index].y;
        from.z = flatten_cloud_for_display_ ? 0.05 : points_[index].z + 0.05;

        geometry_msgs::msg::Point to;
        to.x = points_[index + 1U].x;
        to.y = points_[index + 1U].y;
        to.z = flatten_cloud_for_display_ ? 0.05 : points_[index + 1U].z + 0.05;

        edges.points.push_back(from);
        edges.points.push_back(to);
      }
      markers.markers.push_back(edges);
    }

    for (const auto &point : points_) {
      visualization_msgs::msg::Marker arrow;
      arrow.header.frame_id = frame_id_;
      arrow.header.stamp = now();
      arrow.ns = "rviz_topology_points";
      arrow.id = point.id * 2;
      arrow.type = visualization_msgs::msg::Marker::ARROW;
      arrow.action = visualization_msgs::msg::Marker::ADD;
      arrow.pose.position.x = point.x;
      arrow.pose.position.y = point.y;
      arrow.pose.position.z = flatten_cloud_for_display_ ? 0.1 : point.z;
      arrow.pose.orientation.x = point.qx;
      arrow.pose.orientation.y = point.qy;
      arrow.pose.orientation.z = point.qz;
      arrow.pose.orientation.w = point.qw;
      arrow.scale.x = marker_arrow_length_;
      arrow.scale.y = marker_arrow_width_;
      arrow.scale.z = marker_arrow_width_;
      arrow.color.r = 0.1F;
      arrow.color.g = 0.9F;
      arrow.color.b = 0.2F;
      arrow.color.a = 1.0F;
      markers.markers.push_back(arrow);

      visualization_msgs::msg::Marker label = arrow;
      label.ns = "rviz_topology_point_labels";
      label.id = point.id * 2 + 1;
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.pose.position.z += marker_text_height_offset_;
      label.pose.orientation.x = 0.0;
      label.pose.orientation.y = 0.0;
      label.pose.orientation.z = 0.0;
      label.pose.orientation.w = 1.0;
      label.scale.x = 0.0;
      label.scale.y = 0.0;
      label.scale.z = marker_text_scale_;
      label.color.r = 1.0F;
      label.color.g = 0.0F;
      label.color.b = 0.0F;
      label.text = point.label;
      markers.markers.push_back(label);
    }

    marker_pub_->publish(markers);
  }

  void undoLast(const std_srvs::srv::Trigger::Request::SharedPtr,
                std_srvs::srv::Trigger::Response::SharedPtr response) {
    if (points_.empty()) {
      response->success = false;
      response->message = "no point to undo";
      return;
    }

    const auto removed_id = points_.back().id;
    points_.pop_back();
    next_id_ = points_.empty() ? 0 : points_.back().id + 1;
    show_edges_ = false;
    try {
      if (auto_save_raw_points_) {
        saveRawPointFiles();
      }
      publishMarkers();
      response->success = true;
      response->message = "removed point id=" + std::to_string(removed_id) +
                          "; click Save to finalize";
    } catch (const std::exception &error) {
      response->success = false;
      response->message = error.what();
    }
  }

  void clearPoints(const std_srvs::srv::Trigger::Request::SharedPtr,
                   std_srvs::srv::Trigger::Response::SharedPtr response) {
    const auto previous_points = points_;
    const auto previous_next_id = next_id_;
    points_.clear();
    next_id_ = 0;
    show_edges_ = false;
    try {
      if (auto_save_raw_points_) {
        saveRawPointFiles();
      }
      publishMarkers();
      response->success = true;
      response->message =
          "all current points cleared; save to overwrite output files";
    } catch (const std::exception &error) {
      points_ = previous_points;
      next_id_ = previous_next_id;
      response->success = false;
      response->message = error.what();
    }
  }

  void saveAll(const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
    try {
      saveAllOutputs();
      const auto edge_count = points_.empty() ? 0U : points_.size() - 1U;
      response->success = true;
      response->message = "points=" + std::to_string(points_.size()) +
                          ", edges=" + std::to_string(edge_count) +
                          ", task=" + task_file_;
      show_edges_ = true;
      publishMarkers();
      RCLCPP_INFO(get_logger(),
                  "Save completed: points=%zu edges=%zu topology=%s task=%s",
                  points_.size(), edge_count, connected_topology_file_.c_str(),
                  task_file_.c_str());
    } catch (const std::exception &error) {
      response->success = false;
      response->message = error.what();
      RCLCPP_ERROR(get_logger(), "Save failed: %s", error.what());
    }
  }

  std::string pcd_file_;
  std::string output_file_;
  std::string target_points_file_;
  std::string connected_topology_file_;
  std::string task_file_;
  std::string frame_id_;
  std::string goal_topic_;
  std::string cloud_topic_;
  std::string marker_topic_;

  bool flatten_cloud_for_display_{true};
  bool auto_save_raw_points_{false};
  bool show_edges_{false};
  double marker_arrow_length_{2.0};
  double marker_arrow_width_{0.4};
  double marker_text_scale_{1.0};
  double marker_text_height_offset_{1.2};
  double edge_width_{0.15};
  int64_t task_wait_sec_{0};
  int64_t task_publish_data_{1};

  int next_id_{0};
  std::vector<SavedPoint> points_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr display_cloud_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr undo_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
};

rclcpp::Node::SharedPtr makeRvizTopologyEditorNode() {
  return std::make_shared<RvizTopologyEditor>();
}

} // namespace rviz_topology_editor
