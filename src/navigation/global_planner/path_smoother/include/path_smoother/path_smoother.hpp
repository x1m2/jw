#ifndef PATH_SMOOTHER_PATH_SMOOTHER_HPP
#define PATH_SMOOTHER_PATH_SMOOTHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <navigation_interfaces/msg/path_with_policy.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace path_smoother
{

class PathSmoother
{
public:
  explicit PathSmoother(const rclcpp::Node::SharedPtr & node);
  ~PathSmoother();

  bool initialize();

private:
  // ROS2组件
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<navigation_interfaces::msg::PathWithPolicy>::SharedPtr
      raw_path_with_policy_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr smoothed_path_pub_;
  rclcpp::Publisher<navigation_interfaces::msg::PathWithPolicy>::SharedPtr
      smoothed_path_with_policy_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualization_pub_;

  // 参数
  std::string global_frame_;
  double smoothing_factor_;          // 平滑强度因子 (0.0-1.0)
  int smoothing_iterations_;         // 平滑迭代次数
  double min_point_distance_;        // 最小点间距（用于路径点过滤）
  double max_deviation_;             // 最大允许偏差
  bool enable_visualization_;
  bool enable_point_filtering_;      // 是否启用路径点过滤

  // 数据
  nav_msgs::msg::Path last_raw_path_;
  nav_msgs::msg::Path last_smoothed_path_;
  bool has_path_;

  // 回调函数
  void pathWithPolicyCallback(
    const navigation_interfaces::msg::PathWithPolicy::SharedPtr msg);
  std::vector<bool> remapPolicyToSmoothedPath(
    const nav_msgs::msg::Path & raw_path,
    const std::vector<bool> & raw_policy,
    const nav_msgs::msg::Path & smoothed_path);

  // 路径平滑核心方法
  nav_msgs::msg::Path smoothPath(const nav_msgs::msg::Path & raw_path);
  nav_msgs::msg::Path weightedNeighborhoodSmooth(const nav_msgs::msg::Path & path);
  nav_msgs::msg::Path filterPathPoints(const nav_msgs::msg::Path & path);

  // 工具方法
  double calculateDistance(
    const geometry_msgs::msg::Point & p1,
    const geometry_msgs::msg::Point & p2);

  geometry_msgs::msg::Quaternion calculateOrientation(
    const geometry_msgs::msg::Point & from,
    const geometry_msgs::msg::Point & to);

  bool validateSmoothedPath(
    const nav_msgs::msg::Path & original_path,
    const nav_msgs::msg::Path & smoothed_path);

  // 可视化方法
  void publishVisualization(
    const nav_msgs::msg::Path & raw_path,
    const nav_msgs::msg::Path & smoothed_path);

  // 路径质量评估
  double calculatePathLength(const nav_msgs::msg::Path & path);
  double calculateMaxDeviation(
    const nav_msgs::msg::Path & original_path,
    const nav_msgs::msg::Path & smoothed_path);
  double calculatePathSmoothness(const nav_msgs::msg::Path & path);
};

}  // namespace path_smoother

#endif  // PATH_SMOOTHER_PATH_SMOOTHER_HPP
