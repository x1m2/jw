#pragma once

#include <chrono>
#include <fstream>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>
#include <vector>

struct TopologyNode {
  int id;
  double x;
  double y;
  double z;
  std::string label;

  TopologyNode(int node_id, double px, double py, double pz,
               const std::string &node_label)
      : id(node_id), x(px), y(py), z(pz), label(node_label) {}
};

struct TargetPoint {
  int id;
  double x;
  double y;
  double z;
  double qx;
  double qy;
  double qz;
  double qw;
  std::string label;

  TargetPoint(int point_id, const geometry_msgs::msg::PoseStamped &pose,
              const std::string &point_label)
      : id(point_id),
        x(pose.pose.position.x),
        y(pose.pose.position.y),
        z(pose.pose.position.z),
        qx(pose.pose.orientation.x),
        qy(pose.pose.orientation.y),
        qz(pose.pose.orientation.z),
        qw(pose.pose.orientation.w),
        label(point_label) {}
};

class TopologyMapGenerator : public rclcpp::Node {
 public:
  TopologyMapGenerator();
  ~TopologyMapGenerator() override;

 private:
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void topologyNodeSetCallback(const std_msgs::msg::Bool::SharedPtr msg);

  void addManualPoint(const geometry_msgs::msg::PoseStamped &pose);
  void saveFiles();
  void saveTopologyMap() const;
  void saveTargetPoints() const;

 private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr topology_node_set_sub_;

  std::vector<TopologyNode> nodes_;
  std::vector<TargetPoint> target_points_;

  geometry_msgs::msg::PoseStamped latest_pose_;
  bool has_latest_pose_{false};

  int next_node_id_{0};
  int next_target_id_{0};

  std::string output_file_path_;
  std::string target_points_file_path_;
  std::string pose_topic_;
  std::string trigger_topic_;

  std::chrono::steady_clock::time_point last_manual_trigger_time_{};
  static constexpr int kManualTriggerDelayMs = 2000;
};
