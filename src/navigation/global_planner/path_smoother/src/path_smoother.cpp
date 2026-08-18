#include "path_smoother/path_smoother.hpp"

#include <functional>
#include <limits>

namespace path_smoother {

PathSmoother::PathSmoother(const rclcpp::Node::SharedPtr &node)
    : node_(node), has_path_(false) {}

PathSmoother::~PathSmoother() {}

bool PathSmoother::initialize() {
  // 读取参数
  node_->declare_parameter<std::string>("global_frame", "map");
  node_->declare_parameter<double>("smoothing_factor", 0.3);
  node_->declare_parameter<int>("smoothing_iterations", 3);
  node_->declare_parameter<double>("min_point_distance", 0.1);
  node_->declare_parameter<double>("max_deviation", 0.5);
  node_->declare_parameter<bool>("enable_visualization", true);
  node_->declare_parameter<bool>("enable_point_filtering", true);

  node_->get_parameter("global_frame", global_frame_);
  node_->get_parameter("smoothing_factor", smoothing_factor_);
  node_->get_parameter("smoothing_iterations", smoothing_iterations_);
  node_->get_parameter("min_point_distance", min_point_distance_);
  node_->get_parameter("max_deviation", max_deviation_);
  node_->get_parameter("enable_visualization", enable_visualization_);
  node_->get_parameter("enable_point_filtering", enable_point_filtering_);

  // 参数验证
  smoothing_factor_ = std::max(0.0, std::min(1.0, smoothing_factor_));
  smoothing_iterations_ = std::max(1, smoothing_iterations_);
  min_point_distance_ = std::max(0.01, min_point_distance_);
  max_deviation_ = std::max(0.01, max_deviation_);

  // 初始化发布者和订阅者
  raw_path_with_policy_sub_ =
      node_->create_subscription<navigation_interfaces::msg::PathWithPolicy>(
          "global_path_with_policy", rclcpp::QoS(1),
          std::bind(&PathSmoother::pathWithPolicyCallback, this,
                    std::placeholders::_1));

  smoothed_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
      "smoothed_global_path", rclcpp::QoS(1).transient_local());
  smoothed_path_with_policy_pub_ =
      node_->create_publisher<navigation_interfaces::msg::PathWithPolicy>(
          "smoothed_global_path_with_policy",
          rclcpp::QoS(1).transient_local());

  if (enable_visualization_) {
    visualization_pub_ =
        node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            "path_visualization", rclcpp::QoS(1).transient_local());
  }

  RCLCPP_INFO(node_->get_logger(), "PathSmoother initialized with parameters:");
  RCLCPP_INFO(node_->get_logger(), "  - Global frame: %s",
              global_frame_.c_str());
  RCLCPP_INFO(node_->get_logger(), "  - Smoothing factor: %.3f",
              smoothing_factor_);
  RCLCPP_INFO(node_->get_logger(), "  - Smoothing iterations: %d",
              smoothing_iterations_);
  RCLCPP_INFO(node_->get_logger(), "  - Min point distance: %.3f m",
              min_point_distance_);
  RCLCPP_INFO(node_->get_logger(), "  - Max deviation: %.3f m", max_deviation_);
  RCLCPP_INFO(node_->get_logger(), "  - Point filtering: %s",
              enable_point_filtering_ ? "enabled" : "disabled");
  RCLCPP_INFO(node_->get_logger(), "  - Visualization: %s",
              enable_visualization_ ? "enabled" : "disabled");

  return true;
}

void PathSmoother::pathWithPolicyCallback(
    const navigation_interfaces::msg::PathWithPolicy::SharedPtr msg) {
  if (msg->path.poses.empty()) {
    RCLCPP_WARN(node_->get_logger(), "Received empty path, skipping smoothing");
    return;
  }

  std::vector<navigation_interfaces::msg::EdgePolicy> raw_policy =
      msg->edge_policies;
  if (raw_policy.size() != msg->path.poses.size()) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Path policy size mismatch: poses=%zu, policies=%zu; "
                 "falling back to the safe default policy",
                 msg->path.poses.size(), raw_policy.size());
    const auto default_policy =
        navigation_interfaces::makeDefaultEdgePolicy(false);
    raw_policy.assign(msg->path.poses.size(), default_policy);
  }
  rclcpp::Time start_time = node_->now();

  // 保存原始路径
  last_raw_path_ = msg->path;

  // 保留路径源的坐标系。普通拓扑导航使用map，FAST-LIO UWB跟随使用odom；
  // 平滑器不能把odom坐标数值仅通过改header伪装成map。
  nav_msgs::msg::Path path_to_smooth = msg->path;
  if (path_to_smooth.header.frame_id.empty()) {
    RCLCPP_WARN(node_->get_logger(),
                "Input path frame is empty, falling back to configured frame '%s'",
                global_frame_.c_str());
    path_to_smooth.header.frame_id = global_frame_;
  }
  const bool dynamic_path = msg->dynamic_path;
  const auto publish_path =
      [this, &path_to_smooth, &raw_policy, dynamic_path](
          const nav_msgs::msg::Path &path) {
    smoothed_path_pub_->publish(path);
    navigation_interfaces::msg::PathWithPolicy path_with_policy;
    path_with_policy.path = path;
    path_with_policy.edge_policies =
        remapPolicyToSmoothedPath(path_to_smooth, raw_policy, path);
    path_with_policy.dynamic_path = dynamic_path;
    smoothed_path_with_policy_pub_->publish(path_with_policy);
  };
  {
    RCLCPP_INFO(node_->get_logger(),
                "========== PATH_SMOOTHER INPUT /global_path frame=%s size=%zu "
                "==========",
                path_to_smooth.header.frame_id.c_str(),
                path_to_smooth.poses.size());

    for (size_t i = 0; i < path_to_smooth.poses.size(); ++i) {
      const auto &p = path_to_smooth.poses[i].pose.position;
      RCLCPP_INFO(node_->get_logger(),
                  "[SMOOTHER_IN] idx=%zu x=%.3f y=%.3f z=%.3f", i, p.x, p.y,
                  p.z);
    }
  }
  try {
    // 执行路径平滑
    nav_msgs::msg::Path smoothed_path = smoothPath(path_to_smooth);
    {
      RCLCPP_INFO(node_->get_logger(),
                  "========== PATH_SMOOTHER OUTPUT /smoothed_global_path "
                  "frame=%s size=%zu ==========",
                  smoothed_path.header.frame_id.c_str(),
                  smoothed_path.poses.size());

      for (size_t i = 0; i < smoothed_path.poses.size(); ++i) {
        const auto &p = smoothed_path.poses[i].pose.position;
        RCLCPP_INFO(node_->get_logger(),
                    "[SMOOTHER_OUT] idx=%zu x=%.3f y=%.3f z=%.3f", i, p.x, p.y,
                    p.z);
      }
    }

    // 验证平滑后的路径
    if (validateSmoothedPath(path_to_smooth, smoothed_path)) {
      last_smoothed_path_ = smoothed_path;
      has_path_ = true;

      // 发布平滑后的路径
      publish_path(smoothed_path);

      // 计算处理时间和质量指标
      double processing_time = (node_->now() - start_time).seconds() * 1000.0;
      double original_length = calculatePathLength(path_to_smooth);
      double smoothed_length = calculatePathLength(smoothed_path);
      double max_dev = calculateMaxDeviation(path_to_smooth, smoothed_path);
      double smoothness = calculatePathSmoothness(smoothed_path);

      RCLCPP_INFO(
          node_->get_logger(),
          "Path smoothed: %zu->%zu points, %.1fms, length: %.2f->%.2f m, "
          "max_dev: %.3f m, smoothness: %.3f",
          path_to_smooth.poses.size(), smoothed_path.poses.size(),
          processing_time, original_length, smoothed_length, max_dev,
          smoothness);

      // 发布可视化
      if (enable_visualization_) {
        publishVisualization(path_to_smooth, smoothed_path);
      }
    } else {
      RCLCPP_WARN(node_->get_logger(),
                  "Smoothed path validation failed, publishing original path");
      publish_path(path_to_smooth);
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "Path smoothing failed: %s", e.what());
    publish_path(path_to_smooth);
  }
}

std::vector<navigation_interfaces::msg::EdgePolicy>
PathSmoother::remapPolicyToSmoothedPath(
    const nav_msgs::msg::Path &raw_path,
    const std::vector<navigation_interfaces::msg::EdgePolicy> &raw_policy,
    const nav_msgs::msg::Path &smoothed_path) {
  std::vector<navigation_interfaces::msg::EdgePolicy> result;
  result.reserve(smoothed_path.poses.size());
  if (raw_path.poses.empty() || raw_policy.size() != raw_path.poses.size()) {
    const auto default_policy =
        navigation_interfaces::makeDefaultEdgePolicy(false);
    result.assign(smoothed_path.poses.size(), default_policy);
    return result;
  }

  std::size_t search_start = 0U;
  for (const auto &smoothed_pose : smoothed_path.poses) {
    std::size_t nearest_index = search_start;
    double nearest_distance = std::numeric_limits<double>::max();
    for (std::size_t i = search_start; i < raw_path.poses.size(); ++i) {
      const double distance = calculateDistance(
          smoothed_pose.pose.position, raw_path.poses[i].pose.position);
      if (distance < nearest_distance) {
        nearest_distance = distance;
        nearest_index = i;
      }
    }
    search_start = nearest_index;
    result.push_back(raw_policy[nearest_index]);
  }
  return result;
}

nav_msgs::msg::Path PathSmoother::smoothPath(
    const nav_msgs::msg::Path &raw_path) {
  if (raw_path.poses.size() < 3) {
    RCLCPP_WARN(node_->get_logger(),
                "Path too short for smoothing (< 3 points)");
    return raw_path;
  }

  nav_msgs::msg::Path result = raw_path;

  // 1. 可选的路径点过滤
  if (enable_point_filtering_) {
    result = filterPathPoints(result);
    if (result.poses.size() < 3) {
      RCLCPP_WARN(node_->get_logger(),
                  "Path too short after filtering, returning original");
      return raw_path;
    }
  }

  // 2. 执行简单三点平滑算法（更简单高效）
  for (int iter = 0; iter < smoothing_iterations_; ++iter) {
    result = weightedNeighborhoodSmooth(result);
  }

  return result;
}

nav_msgs::msg::Path PathSmoother::weightedNeighborhoodSmooth(
    const nav_msgs::msg::Path &path) {
  if (path.poses.size() < 3) {
    return path;
  }

  nav_msgs::msg::Path smoothed_path = path;

  // 简单的三点平滑：对每个点使用前后邻居的加权平均
  for (size_t i = 1; i < path.poses.size() - 1; ++i) {
    geometry_msgs::msg::Point prev = path.poses[i - 1].pose.position;
    geometry_msgs::msg::Point curr = path.poses[i].pose.position;
    geometry_msgs::msg::Point next = path.poses[i + 1].pose.position;

    // 简单的三点加权平均：权重为 [0.25, 0.5, 0.25]
    geometry_msgs::msg::Point smoothed;
    smoothed.x = 0.25 * prev.x + 0.5 * curr.x + 0.25 * next.x;
    smoothed.y = 0.25 * prev.y + 0.5 * curr.y + 0.25 * next.y;
    smoothed.z = curr.z;  // 保持高度不变

    // 使用平滑因子混合原始点和平滑点
    smoothed_path.poses[i].pose.position.x =
        (1.0 - smoothing_factor_) * curr.x + smoothing_factor_ * smoothed.x;
    smoothed_path.poses[i].pose.position.y =
        (1.0 - smoothing_factor_) * curr.y + smoothing_factor_ * smoothed.y;
    smoothed_path.poses[i].pose.position.z = curr.z;
  }

  // 重新计算方向
  for (size_t i = 0; i < smoothed_path.poses.size(); ++i) {
    if (i < smoothed_path.poses.size() - 1) {
      smoothed_path.poses[i].pose.orientation =
          calculateOrientation(smoothed_path.poses[i].pose.position,
                               smoothed_path.poses[i + 1].pose.position);
    } else if (i > 0) {
      smoothed_path.poses[i].pose.orientation =
          smoothed_path.poses[i - 1].pose.orientation;
    }
  }

  return smoothed_path;
}

nav_msgs::msg::Path PathSmoother::filterPathPoints(
    const nav_msgs::msg::Path &path) {
  if (path.poses.size() < 2) {
    return path;
  }

  nav_msgs::msg::Path filtered_path = path;
  filtered_path.poses.clear();

  // 总是保留第一个点
  filtered_path.poses.push_back(path.poses[0]);

  for (size_t i = 1; i < path.poses.size(); ++i) {
    double distance = calculateDistance(
        filtered_path.poses.back().pose.position, path.poses[i].pose.position);

    // 如果距离大于最小距离阈值，则保留该点
    if (distance >= min_point_distance_ || i == path.poses.size() - 1) {
      filtered_path.poses.push_back(path.poses[i]);
    }
  }

  RCLCPP_DEBUG(node_->get_logger(), "Path filtering: %zu -> %zu points",
               path.poses.size(), filtered_path.poses.size());

  return filtered_path;
}

double PathSmoother::calculateDistance(const geometry_msgs::msg::Point &p1,
                                       const geometry_msgs::msg::Point &p2) {
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  return std::sqrt(dx * dx + dy * dy);
}

geometry_msgs::msg::Quaternion PathSmoother::calculateOrientation(
    const geometry_msgs::msg::Point &from,
    const geometry_msgs::msg::Point &to) {
  double dx = to.x - from.x;
  double dy = to.y - from.y;
  double yaw = std::atan2(dy, dx);

  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);

  return q;
}

bool PathSmoother::validateSmoothedPath(
    const nav_msgs::msg::Path &original_path,
    const nav_msgs::msg::Path &smoothed_path) {
  if (smoothed_path.poses.empty()) {
    RCLCPP_WARN(node_->get_logger(), "Smoothed path is empty");
    return false;
  }

  // 检查长度变化
  double original_length = calculatePathLength(original_path);
  double smoothed_length = calculatePathLength(smoothed_path);

  if (original_length > 0.0) {
    double length_change_ratio =
        std::abs(smoothed_length - original_length) / original_length;

    if (length_change_ratio > 0.5) {  // 长度变化超过50%
      RCLCPP_WARN(node_->get_logger(),
                  "Path length changed too much: %.1f%% (%.2f -> %.2f m)",
                  length_change_ratio * 100.0, original_length,
                  smoothed_length);
      return false;
    }
  }

  // 检查最大偏差
  double max_deviation = calculateMaxDeviation(original_path, smoothed_path);
  if (max_deviation > max_deviation_) {
    RCLCPP_WARN(node_->get_logger(),
                "Max deviation too large: %.3f m (limit: %.3f m)",
                max_deviation, max_deviation_);
    return false;
  }

  return true;
}

void PathSmoother::publishVisualization(
    const nav_msgs::msg::Path &raw_path,
    const nav_msgs::msg::Path &smoothed_path) {
  visualization_msgs::msg::MarkerArray marker_array;

  // 原始路径（红色）
  visualization_msgs::msg::Marker raw_marker;
  raw_marker.header.frame_id = global_frame_;
  raw_marker.header.stamp = node_->now();
  raw_marker.ns = "raw_path";
  raw_marker.id = 0;
  raw_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  raw_marker.action = visualization_msgs::msg::Marker::ADD;
  raw_marker.scale.x = 0.05;
  raw_marker.color.r = 1.0;
  raw_marker.color.g = 0.0;
  raw_marker.color.b = 0.0;
  raw_marker.color.a = 0.8;

  for (const auto &pose : raw_path.poses) {
    raw_marker.points.push_back(pose.pose.position);
  }
  marker_array.markers.push_back(raw_marker);

  // 平滑路径（绿色）
  visualization_msgs::msg::Marker smoothed_marker;
  smoothed_marker.header.frame_id = global_frame_;
  smoothed_marker.header.stamp = node_->now();
  smoothed_marker.ns = "smoothed_path";
  smoothed_marker.id = 1;
  smoothed_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  smoothed_marker.action = visualization_msgs::msg::Marker::ADD;
  smoothed_marker.scale.x = 0.08;
  smoothed_marker.color.r = 0.0;
  smoothed_marker.color.g = 1.0;
  smoothed_marker.color.b = 0.0;
  smoothed_marker.color.a = 1.0;

  for (const auto &pose : smoothed_path.poses) {
    smoothed_marker.points.push_back(pose.pose.position);
  }
  marker_array.markers.push_back(smoothed_marker);

  // 路径点（蓝色）
  visualization_msgs::msg::Marker points_marker;
  points_marker.header.frame_id = global_frame_;
  points_marker.header.stamp = node_->now();
  points_marker.ns = "path_points";
  points_marker.id = 2;
  points_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  points_marker.action = visualization_msgs::msg::Marker::ADD;
  points_marker.scale.x = 0.1;
  points_marker.scale.y = 0.1;
  points_marker.scale.z = 0.1;
  points_marker.color.r = 0.0;
  points_marker.color.g = 0.0;
  points_marker.color.b = 1.0;
  points_marker.color.a = 0.6;

  for (const auto &pose : smoothed_path.poses) {
    points_marker.points.push_back(pose.pose.position);
  }
  marker_array.markers.push_back(points_marker);

  visualization_pub_->publish(marker_array);
}

double PathSmoother::calculatePathLength(const nav_msgs::msg::Path &path) {
  if (path.poses.size() < 2) {
    return 0.0;
  }

  double total_length = 0.0;
  for (size_t i = 1; i < path.poses.size(); ++i) {
    total_length += calculateDistance(path.poses[i - 1].pose.position,
                                      path.poses[i].pose.position);
  }

  return total_length;
}

double PathSmoother::calculateMaxDeviation(
    const nav_msgs::msg::Path &original_path,
    const nav_msgs::msg::Path &smoothed_path) {
  if (original_path.poses.empty() || smoothed_path.poses.empty()) {
    return 0.0;
  }

  double max_deviation = 0.0;

  // 对每个平滑路径点，找到原始路径上最近的点
  for (const auto &smoothed_pose : smoothed_path.poses) {
    double min_distance = std::numeric_limits<double>::max();

    for (const auto &original_pose : original_path.poses) {
      double distance = calculateDistance(smoothed_pose.pose.position,
                                          original_pose.pose.position);
      min_distance = std::min(min_distance, distance);
    }

    max_deviation = std::max(max_deviation, min_distance);
  }

  return max_deviation;
}

double PathSmoother::calculatePathSmoothness(const nav_msgs::msg::Path &path) {
  if (path.poses.size() < 3) {
    return 0.0;
  }

  double total_curvature = 0.0;
  int curvature_count = 0;

  for (size_t i = 1; i < path.poses.size() - 1; ++i) {
    geometry_msgs::msg::Point p1 = path.poses[i - 1].pose.position;
    geometry_msgs::msg::Point p2 = path.poses[i].pose.position;
    geometry_msgs::msg::Point p3 = path.poses[i + 1].pose.position;

    // 计算连续三点的角度变化
    double angle1 = std::atan2(p2.y - p1.y, p2.x - p1.x);
    double angle2 = std::atan2(p3.y - p2.y, p3.x - p2.x);
    double angle_diff = std::abs(angle2 - angle1);

    // 归一化角度差
    while (angle_diff > M_PI) {
      angle_diff -= 2.0 * M_PI;
    }
    angle_diff = std::abs(angle_diff);

    total_curvature += angle_diff;
    curvature_count++;
  }

  return curvature_count > 0 ? total_curvature / curvature_count : 0.0;
}

}  // namespace path_smoother
