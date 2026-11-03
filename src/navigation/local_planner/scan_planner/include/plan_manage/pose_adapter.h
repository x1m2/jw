#pragma once
#include <cmath>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace scan_planner {
// Adapt real-robot poses to the internal Odometry API and reference height.
inline nav_msgs::msg::Odometry::SharedPtr adaptPose(
    rclcpp::Node &node, const geometry_msgs::msg::PoseStamped &message) {
  const auto &p = message.pose.position;
  const auto &q = message.pose.orientation;
  std::string frame;
  node.get_parameter("grid_map.frame_id", frame);
  if (message.header.frame_id != frame || !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return {};
  double pose_z_offset;
  node.get_parameter_or("grid_map.pose_z_offset", pose_z_offset, 0.0);
  if (!std::isfinite(pose_z_offset) || !std::isfinite(p.z + pose_z_offset)) return {};
  Eigen::Quaterniond orientation(q.w, q.x, q.y, q.z);
  if (!orientation.coeffs().allFinite() || orientation.norm() < 1e-6) return {};
  orientation.normalize();
  auto result = std::make_shared<nav_msgs::msg::Odometry>();
  result->header = message.header;
  result->child_frame_id = "base_link";
  result->pose.pose = message.pose;
  result->pose.pose.position.z += pose_z_offset;
  result->pose.pose.orientation.w = orientation.w();
  result->pose.pose.orientation.x = orientation.x();
  result->pose.pose.orientation.y = orientation.y();
  result->pose.pose.orientation.z = orientation.z();
  return result;
}
} // namespace scan_planner
