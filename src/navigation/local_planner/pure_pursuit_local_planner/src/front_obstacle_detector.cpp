#include "pure_pursuit_local_planner/front_obstacle_detector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "pure_pursuit_local_planner/param_utils.hpp"

namespace pure_pursuit_local_planner {

FrontObstacleDetector::FrontObstacleDetector()
    : has_costmap_(false),
      has_global_path_(false),
      detection_width_(0.3)  // 0.3m 宽度
      ,
      detection_length_(0.5)  // 0.5m 长度
      ,
      sector_radius_(0.8)  // 0.8m 扇形半径
      ,
      sector_angle_(60.0)  // 60度扇形角度（正负30度）
      ,
      obstacle_threshold_(50)  // 代价值阈值
      ,
      linear_velocity_factor_(0.4)  // 线速度乘以0.4
      ,
      angular_velocity_factor_(2.0)  // 角速度乘以2
      ,
      detection_shape_("rectangle")  // 默认使用矩形
      ,
      obstacle_ratio_threshold_(0.1),
      robot_width_(0.55),
      robot_length_(0.75),
      safety_margin_(0.1),
      hard_stop_distance_(1.0),
      avoid_min_distance_(1.0),
      avoid_max_distance_(1.5),
      avoid_sector_angle_(60.0),
      side_passage_width_(0.75),
      enable_path_corridor_filter_(true),
      path_obstacle_margin_(0.15),
      path_check_distance_(2.0),
      debug_mode_(false),
      enable_visualization_(true) {}

FrontObstacleDetector::~FrontObstacleDetector() {}

bool FrontObstacleDetector::initialize(
    rclcpp::Node::SharedPtr node, std::shared_ptr<tf2_ros::Buffer> tf_buffer) {
  node_ = node;
  tf_buffer_ = tf_buffer;

  // 加载参数
  detection_width_ = declareOrGetParameter(
      node_, "front_obstacle_detector.detection_width", 0.3);
  detection_length_ = declareOrGetParameter(
      node_, "front_obstacle_detector.detection_length", 0.5);
  sector_radius_ = declareOrGetParameter(
      node_, "front_obstacle_detector.sector_radius", 0.8);
  sector_angle_ = declareOrGetParameter(
      node_, "front_obstacle_detector.sector_angle", 60.0);
  obstacle_threshold_ = declareOrGetParameter(
      node_, "front_obstacle_detector.obstacle_threshold", 50);
  linear_velocity_factor_ = declareOrGetParameter(
      node_, "front_obstacle_detector.linear_velocity_factor", 0.4);
  angular_velocity_factor_ = declareOrGetParameter(
      node_, "front_obstacle_detector.angular_velocity_factor", 2.0);
  detection_shape_ =
      declareOrGetParameter(node_, "front_obstacle_detector.detection_shape",
                            std::string("rectangle"));
  obstacle_ratio_threshold_ = declareOrGetParameter(
      node_, "front_obstacle_detector.obstacle_ratio_threshold", 0.1);
  robot_width_ = declareOrGetParameter(
      node_, "front_obstacle_detector.robot_width", 0.55);
  robot_length_ = declareOrGetParameter(
      node_, "front_obstacle_detector.robot_length", 0.75);
  safety_margin_ = declareOrGetParameter(
      node_, "front_obstacle_detector.safety_margin", 0.1);
  hard_stop_distance_ = declareOrGetParameter(
      node_, "front_obstacle_detector.hard_stop_distance", 1.0);
  avoid_min_distance_ = declareOrGetParameter(
      node_, "front_obstacle_detector.avoid_min_distance", 1.0);
  avoid_max_distance_ = declareOrGetParameter(
      node_, "front_obstacle_detector.avoid_max_distance", 1.5);
  avoid_sector_angle_ = declareOrGetParameter(
      node_, "front_obstacle_detector.avoid_sector_angle", 60.0);
  side_passage_width_ = declareOrGetParameter(
      node_, "front_obstacle_detector.side_passage_width", 0.75);
  enable_path_corridor_filter_ = declareOrGetParameter(
      node_, "front_obstacle_detector.enable_path_corridor_filter", true);
  path_obstacle_margin_ = declareOrGetParameter(
      node_, "front_obstacle_detector.path_obstacle_margin", 0.15);
  path_check_distance_ = declareOrGetParameter(
      node_, "front_obstacle_detector.path_check_distance", 2.0);
  debug_mode_ =
      declareOrGetParameter(node_, "front_obstacle_detector.debug_mode", false);
  enable_visualization_ = declareOrGetParameter(
      node_, "front_obstacle_detector.enable_visualization", true);

  // 初始化发布器
  if (enable_visualization_) {
    detection_area_pub_ =
        node_->create_publisher<visualization_msgs::msg::Marker>(
            "front_detection_area", 1);
  }

  RCLCPP_INFO(node_->get_logger(),
              "Front Obstacle Detector initialized with parameters:");
  if (detection_shape_ == "rectangle") {
    RCLCPP_INFO(node_->get_logger(),
                "  - Detection shape: Rectangle (%.2fm x %.2fm)",
                detection_width_, detection_length_);
  } else if (detection_shape_ == "sector") {
    RCLCPP_INFO(node_->get_logger(),
                "  - Detection shape: Sector (radius: %.2fm, angle: %.1f°)",
                sector_radius_, sector_angle_);
  }
  RCLCPP_INFO(node_->get_logger(), "  - Obstacle threshold: %d",
              obstacle_threshold_);
  RCLCPP_INFO(node_->get_logger(), "  - Linear velocity factor: %.2f",
              linear_velocity_factor_);
  RCLCPP_INFO(node_->get_logger(), "  - Angular velocity factor: %.2f",
              angular_velocity_factor_);
  RCLCPP_INFO(node_->get_logger(),
              "  - Hard stop: robot %.2fm x %.2fm, margin %.2fm, front+%.2fm",
              robot_length_, robot_width_, safety_margin_, hard_stop_distance_);
  RCLCPP_INFO(node_->get_logger(),
              "  - Avoidance sector: r=[%.2f, %.2f], angle=%.1f, side passage=%.2f",
              avoid_min_distance_, avoid_max_distance_, avoid_sector_angle_,
              side_passage_width_);
  RCLCPP_INFO(
      node_->get_logger(),
      "  - Path corridor filter: %s, path_check=%.2fm, extra_margin=%.2fm",
      enable_path_corridor_filter_ ? "true" : "false",
      path_check_distance_, path_obstacle_margin_);
  RCLCPP_INFO(node_->get_logger(), "  - Visualization enabled: %s",
              enable_visualization_ ? "true" : "false");

  return true;
}

void FrontObstacleDetector::updateCostmap(
    const nav_msgs::msg::OccupancyGrid& costmap) {
  costmap_ = costmap;
  has_costmap_ = true;
}

void FrontObstacleDetector::setGlobalPath(
    const nav_msgs::msg::Path& global_path) {
  std::lock_guard<std::mutex> lock(global_path_mutex_);
  global_path_ = global_path;
  has_global_path_ = !global_path_.poses.empty();
  RCLCPP_WARN(
      node_->get_logger(),
      "[AVOID_PATH_FILTER] detector received global path: poses=%zu, has_path=%s",
      global_path_.poses.size(), has_global_path_ ? "true" : "false");
}

geometry_msgs::msg::Twist FrontObstacleDetector::adjustVelocityForObstacles(
    const geometry_msgs::msg::Twist& cmd_vel,
    const geometry_msgs::msg::PoseStamped& robot_pose) {
  geometry_msgs::msg::Twist adjusted_cmd_vel = cmd_vel;

  // 如果没有代价地图，直接返回原始速度
  if (!has_costmap_) {
    if (debug_mode_) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "No costmap available for obstacle detection");
    }
    return adjusted_cmd_vel;
  }

  const double half_width = robot_width_ * 0.5 + safety_margin_;
  const double front_edge = robot_length_ * 0.5;
  const double stop_x_min = std::max(0.0, front_edge);
  const double stop_x_max = stop_x_min + std::max(0.0, hard_stop_distance_);
  const bool hard_stop = detectObstacleInRectLocal(
      robot_pose, stop_x_min, stop_x_max, -half_width, half_width);
  if (hard_stop) {
    adjusted_cmd_vel = geometry_msgs::msg::Twist();
    if (enable_visualization_) {
      publishVisualization(robot_pose, true);
    }
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
                         "[FRONT_STOP] hard stop obstacle in base_link x=[%.2f, %.2f], y=[%.2f, %.2f], cmd_vel=0",
                         stop_x_min, stop_x_max, -half_width, half_width);
    return adjusted_cmd_vel;
  }

  // 检测前方区域是否有障碍物
  bool obstacle_detected = false;
  if (detection_shape_ == "sector") {
    obstacle_detected = detectObstacleInSectorArea(robot_pose);
  } else  // 默认使用矩形
  {
    obstacle_detected = detectObstacleInFrontArea(robot_pose);
  }

  // 发布可视化
  if (enable_visualization_) {
    publishVisualization(robot_pose, obstacle_detected);
  }

  if (obstacle_detected) {
    // 调整速度：线速度乘以0.4，角速度乘以2
    adjusted_cmd_vel.linear.x *= linear_velocity_factor_;
    adjusted_cmd_vel.linear.y *= linear_velocity_factor_;
    adjusted_cmd_vel.angular.z *= angular_velocity_factor_;

    if (debug_mode_) {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 500,
          "Obstacle detected in front area! Adjusting velocity:");
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
                           "  Original: linear_x=%.3f, angular_z=%.3f",
                           cmd_vel.linear.x, cmd_vel.angular.z);
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
                           "  Adjusted: linear_x=%.3f, angular_z=%.3f",
                           adjusted_cmd_vel.linear.x,
                           adjusted_cmd_vel.angular.z);
    }
  } else if (debug_mode_) {
    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                          "No obstacles detected in front area");
  }

  return adjusted_cmd_vel;
}

FrontObstacleDetector::AvoidanceInfo FrontObstacleDetector::analyzeAvoidance(
    const geometry_msgs::msg::PoseStamped& robot_pose) {
  AvoidanceInfo info;
  info.obstacle_center.z = robot_pose.pose.position.z;
  info.closest_obstacle.z = robot_pose.pose.position.z;

  if (!has_costmap_) {
    if (debug_mode_) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "No costmap available for avoidance analysis");
    }
    return info;
  }

  const double half_width = robot_width_ * 0.5 + safety_margin_;
  const double front_edge = robot_length_ * 0.5;
  const double stop_x_min = std::max(0.0, front_edge);
  const double stop_x_max = stop_x_min + std::max(0.0, hard_stop_distance_);

  if (detectObstacleInRectLocal(robot_pose, stop_x_min, stop_x_max,
                                -half_width, half_width)) {
    info.decision = ObstacleDecision::HARD_STOP;
    if (enable_visualization_) {
      publishVisualization(robot_pose, true);
    }
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
                         "[AVOID_ANALYZE] HARD_STOP in x=[%.2f, %.2f], y=[%.2f, %.2f]",
                         stop_x_min, stop_x_max, -half_width, half_width);
    return info;
  }

  if (!findObstacleInSectorLocal(robot_pose, avoid_min_distance_,
                                 avoid_max_distance_, avoid_sector_angle_,
                                 info)) {
    info.decision = ObstacleDecision::CLEAR;
    if (enable_visualization_) {
      publishVisualization(robot_pose, false);
    }
    return info;
  }

  info.left_clear = isPassageClear(robot_pose, true);
  info.right_clear = isPassageClear(robot_pose, false);
  info.decision = (info.left_clear || info.right_clear)
                      ? ObstacleDecision::AVOIDABLE
                      : ObstacleDecision::BLOCKED;

  if (enable_visualization_) {
    publishVisualization(robot_pose, true);
  }

  RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 500,
      "[AVOID_ANALYZE] %s obstacle_center=(%.3f, %.3f), closest=(%.3f, %.3f), "
      "dist=%.2f, left_clear=%s, right_clear=%s",
      decisionToString(info.decision), info.obstacle_center.x,
      info.obstacle_center.y, info.closest_obstacle.x, info.closest_obstacle.y,
      info.obstacle_distance, info.left_clear ? "true" : "false",
      info.right_clear ? "true" : "false");

  return info;
}

bool FrontObstacleDetector::detectObstacleInFrontArea(
    const geometry_msgs::msg::PoseStamped& robot_pose) {
  // 生成检测区域的四个角点（机器人前方矩形区域）
  std::vector<std::pair<double, double>> corners;
  generateDetectionAreaCorners(robot_pose, corners);

  // 确定检测区域的边界
  double min_x = std::min(
      {corners[0].first, corners[1].first, corners[2].first, corners[3].first});
  double max_x = std::max(
      {corners[0].first, corners[1].first, corners[2].first, corners[3].first});
  double min_y = std::min({corners[0].second, corners[1].second,
                           corners[2].second, corners[3].second});
  double max_y = std::max({corners[0].second, corners[1].second,
                           corners[2].second, corners[3].second});

  // 转换为地图坐标
  int min_map_x, min_map_y, max_map_x, max_map_y;
  if (!worldToMap(min_x, min_y, min_map_x, min_map_y) ||
      !worldToMap(max_x, max_y, max_map_x, max_map_y)) {
    if (debug_mode_) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Detection area outside of costmap bounds");
    }
    return false;
  }

  // 确保地图坐标在有效范围内
  min_map_x = std::max(0, min_map_x);
  min_map_y = std::max(0, min_map_y);
  max_map_x = std::min(static_cast<int>(costmap_.info.width) - 1, max_map_x);
  max_map_y = std::min(static_cast<int>(costmap_.info.height) - 1, max_map_y);

  // 检查区域内的代价值
  int obstacle_cells = 0;
  int total_cells = 0;

  for (int map_x = min_map_x; map_x <= max_map_x; ++map_x) {
    for (int map_y = min_map_y; map_y <= max_map_y; ++map_y) {
      int cost = getCostAtMapCoordinate(map_x, map_y);
      if (cost >= 0)  // 有效的代价值
      {
        total_cells++;
        if (cost >= obstacle_threshold_) {
          obstacle_cells++;
        }
      }
    }
  }

  // 如果障碍物占比超过阈值，认为检测到障碍物
  double obstacle_ratio =
      (total_cells > 0) ? static_cast<double>(obstacle_cells) / total_cells
                        : 0.0;
  bool obstacle_detected = obstacle_ratio > obstacle_ratio_threshold_;

  if (debug_mode_ && obstacle_detected) {
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 500,
        "Obstacle detection: %d/%d cells above threshold (%.1f%%)",
        obstacle_cells, total_cells, obstacle_ratio * 100.0);
  }

  return obstacle_detected;
}

bool FrontObstacleDetector::worldToMap(double world_x, double world_y,
                                       int& map_x, int& map_y) {
  if (!has_costmap_) return false;

  // 转换世界坐标到地图坐标
  map_x = static_cast<int>((world_x - costmap_.info.origin.position.x) /
                           costmap_.info.resolution);
  map_y = static_cast<int>((world_y - costmap_.info.origin.position.y) /
                           costmap_.info.resolution);

  // 检查边界
  return (map_x >= 0 && map_x < static_cast<int>(costmap_.info.width) &&
          map_y >= 0 && map_y < static_cast<int>(costmap_.info.height));
}

int FrontObstacleDetector::getCostAtMapCoordinate(int map_x, int map_y) {
  if (!has_costmap_ || map_x < 0 ||
      map_x >= static_cast<int>(costmap_.info.width) || map_y < 0 ||
      map_y >= static_cast<int>(costmap_.info.height)) {
    return -1;  // 越界
  }

  int index = map_y * costmap_.info.width + map_x;
  if (index >= 0 && index < static_cast<int>(costmap_.data.size())) {
    return costmap_.data[index];
  }

  return -1;
}

void FrontObstacleDetector::generateDetectionAreaCorners(
    const geometry_msgs::msg::PoseStamped& robot_pose,
    std::vector<std::pair<double, double>>& corners) {
  corners.clear();

  double robot_x = robot_pose.pose.position.x;
  double robot_y = robot_pose.pose.position.y;
  double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);

  // 计算机器人前方矩形区域的四个角点
  // 矩形：宽度0.3m，长度0.5m，位于机器人前方

  double half_width = detection_width_ / 2.0;  // 0.15m
  double length = detection_length_;           // 0.5m

  // 机器人坐标系下的四个角点（机器人前方的矩形）
  std::vector<std::pair<double, double>> local_corners = {
      {0.0, -half_width},    // 左后角 (机器人位置)
      {0.0, half_width},     // 右后角 (机器人位置)
      {length, half_width},  // 右前角
      {length, -half_width}  // 左前角
  };

  // 转换到世界坐标系
  for (const auto& local_corner : local_corners) {
    double local_x = local_corner.first;
    double local_y = local_corner.second;

    // 旋转变换
    double world_x =
        robot_x + local_x * std::cos(robot_yaw) - local_y * std::sin(robot_yaw);
    double world_y =
        robot_y + local_x * std::sin(robot_yaw) + local_y * std::cos(robot_yaw);

    corners.push_back({world_x, world_y});
  }

  if (debug_mode_) {
    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                          "Detection area corners in world frame:");
    for (size_t i = 0; i < corners.size(); ++i) {
      RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                            "  Corner %zu: (%.3f, %.3f)", i, corners[i].first,
                            corners[i].second);
    }
  }
}

void FrontObstacleDetector::publishVisualization(
    const geometry_msgs::msg::PoseStamped& robot_pose, bool obstacle_detected) {
  if (!enable_visualization_) return;

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = robot_pose.header.frame_id;
  marker.header.stamp = node_->now();
  marker.ns = "front_obstacle_detection";
  marker.id = 0;
  marker.action = visualization_msgs::msg::Marker::ADD;

  double robot_x = robot_pose.pose.position.x;
  double robot_y = robot_pose.pose.position.y;
  double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);

  if (detection_shape_ == "sector") {
    // 扇形可视化 - 使用 LINE_STRIP
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.scale.x = 0.02;  // 线条宽度

    // 生成扇形边界点
    std::vector<std::pair<double, double>> sector_points;
    generateSectorAreaPoints(robot_pose, sector_points);

    // 添加边界点到marker
    for (const auto& point : sector_points) {
      geometry_msgs::msg::Point p;
      p.x = point.first;
      p.y = point.second;
      p.z = robot_pose.pose.position.z + 0.1;
      marker.points.push_back(p);
    }

    // 闭合扇形（连接最后一点到第一点）
    if (!marker.points.empty()) {
      marker.points.push_back(marker.points[0]);
    }
  } else {
    // 矩形可视化 - 使用 CUBE
    marker.type = visualization_msgs::msg::Marker::CUBE;

    // 计算检测区域中心点位置（机器人前方 detection_length_/2 距离处）
    double center_offset = detection_length_ / 2.0;
    marker.pose.position.x = robot_x + center_offset * std::cos(robot_yaw);
    marker.pose.position.y = robot_y + center_offset * std::sin(robot_yaw);
    marker.pose.position.z =
        robot_pose.pose.position.z + 0.1;  // 稍微抬高以便可视化

    // 设置朝向（与机器人朝向一致）
    marker.pose.orientation = robot_pose.pose.orientation;

    // 设置尺寸
    marker.scale.x = detection_length_;  // 长度
    marker.scale.y = detection_width_;   // 宽度
    marker.scale.z = 0.05;               // 高度（薄片状）
  }

  marker.color.a = 0.35;
  marker.color.r = 0.0;
  marker.color.g = 0.7;
  marker.color.b = 1.0;

  // 设置生存时间
  marker.lifetime.sec = 0;
  marker.lifetime.nanosec = 200000000;  // 0.2s = 200ms

  detection_area_pub_->publish(marker);

  visualization_msgs::msg::Marker hard_stop_marker;
  hard_stop_marker.header.frame_id = robot_pose.header.frame_id;
  hard_stop_marker.header.stamp = marker.header.stamp;
  hard_stop_marker.ns = "front_hard_stop_area";
  hard_stop_marker.id = 1;
  hard_stop_marker.type = visualization_msgs::msg::Marker::CUBE;
  hard_stop_marker.action = obstacle_detected
                                ? visualization_msgs::msg::Marker::ADD
                                : visualization_msgs::msg::Marker::DELETE;

  const double half_width = robot_width_ * 0.5 + safety_margin_;
  const double front_edge = robot_length_ * 0.5;
  const double stop_x_min = std::max(0.0, front_edge);
  const double stop_x_max = stop_x_min + std::max(0.0, hard_stop_distance_);
  const double stop_length = std::max(0.01, stop_x_max - stop_x_min);
  const double center_offset = stop_x_min + stop_length * 0.5;

  hard_stop_marker.pose.position.x =
      robot_x + center_offset * std::cos(robot_yaw);
  hard_stop_marker.pose.position.y =
      robot_y + center_offset * std::sin(robot_yaw);
  hard_stop_marker.pose.position.z = robot_pose.pose.position.z + 0.16;
  hard_stop_marker.pose.orientation = robot_pose.pose.orientation;
  hard_stop_marker.scale.x = stop_length;
  hard_stop_marker.scale.y = 2.0 * half_width;
  hard_stop_marker.scale.z = 0.04;
  hard_stop_marker.color.a = 0.75;
  hard_stop_marker.color.r = 1.0;
  hard_stop_marker.color.g = 0.0;
  hard_stop_marker.color.b = 0.0;
  hard_stop_marker.lifetime.sec = 0;
  hard_stop_marker.lifetime.nanosec = 200000000;

  detection_area_pub_->publish(hard_stop_marker);

  if (debug_mode_) {
    RCLCPP_DEBUG_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "Published %s detection area visualization: %s",
        detection_shape_.c_str(),
        obstacle_detected ? "RED (obstacle)" : "GREEN (clear)");
  }
}

bool FrontObstacleDetector::detectObstacleInSectorArea(
    const geometry_msgs::msg::PoseStamped& robot_pose) {
  double robot_x = robot_pose.pose.position.x;
  double robot_y = robot_pose.pose.position.y;
  double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);

  // 计算扇形边界框
  double min_x = robot_x - sector_radius_;
  double max_x = robot_x + sector_radius_;
  double min_y = robot_y - sector_radius_;
  double max_y = robot_y + sector_radius_;

  // 转换为地图坐标
  int min_map_x, min_map_y, max_map_x, max_map_y;
  if (!worldToMap(min_x, min_y, min_map_x, min_map_y) ||
      !worldToMap(max_x, max_y, max_map_x, max_map_y)) {
    if (debug_mode_) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Sector detection area outside of costmap bounds");
    }
    return false;
  }

  // 确保地图坐标在有效范围内
  min_map_x = std::max(0, min_map_x);
  min_map_y = std::max(0, min_map_y);
  max_map_x = std::min(static_cast<int>(costmap_.info.width) - 1, max_map_x);
  max_map_y = std::min(static_cast<int>(costmap_.info.height) - 1, max_map_y);

  // 检查扇形区域内的代价值
  int obstacle_cells = 0;
  int total_cells = 0;

  for (int map_x = min_map_x; map_x <= max_map_x; ++map_x) {
    for (int map_y = min_map_y; map_y <= max_map_y; ++map_y) {
      // 转换地图坐标回世界坐标
      double world_x =
          costmap_.info.origin.position.x + map_x * costmap_.info.resolution;
      double world_y =
          costmap_.info.origin.position.y + map_y * costmap_.info.resolution;

      // 检查点是否在扇形内
      if (isPointInSector(world_x, world_y, robot_x, robot_y, robot_yaw)) {
        int cost = getCostAtMapCoordinate(map_x, map_y);
        if (cost >= 0)  // 有效的代价值
        {
          total_cells++;
          if (cost >= obstacle_threshold_) {
            obstacle_cells++;
          }
        }
      }
    }
  }

  // 如果障碍物占比超过阈值，认为检测到障碍物
  double obstacle_ratio =
      (total_cells > 0) ? static_cast<double>(obstacle_cells) / total_cells
                        : 0.0;
  bool obstacle_detected = obstacle_ratio > obstacle_ratio_threshold_;

  if (debug_mode_ && obstacle_detected) {
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 500,
        "Sector obstacle detection: %d/%d cells above threshold (%.1f%%)",
        obstacle_cells, total_cells, obstacle_ratio * 100.0);
  }

  return obstacle_detected;
}

bool FrontObstacleDetector::detectObstacleInRectLocal(
    const geometry_msgs::msg::PoseStamped& robot_pose,
    double x_min, double x_max, double y_min, double y_max) {
  if (!has_costmap_ || costmap_.info.resolution <= 0.0 ||
      costmap_.info.width == 0 || costmap_.info.height == 0) {
    return false;
  }

  const double robot_x = robot_pose.pose.position.x;
  const double robot_y = robot_pose.pose.position.y;
  const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
  const double cos_yaw = std::cos(robot_yaw);
  const double sin_yaw = std::sin(robot_yaw);

  std::vector<std::pair<double, double>> corners;
  corners.reserve(4);
  const std::vector<std::pair<double, double>> local_corners = {
      {x_min, y_min}, {x_min, y_max}, {x_max, y_min}, {x_max, y_max}};
  for (const auto& corner : local_corners) {
    const double world_x =
        robot_x + corner.first * cos_yaw - corner.second * sin_yaw;
    const double world_y =
        robot_y + corner.first * sin_yaw + corner.second * cos_yaw;
    corners.emplace_back(world_x, world_y);
  }

  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();
  for (const auto& corner : corners) {
    min_x = std::min(min_x, corner.first);
    min_y = std::min(min_y, corner.second);
    max_x = std::max(max_x, corner.first);
    max_y = std::max(max_y, corner.second);
  }

  int min_map_x = static_cast<int>(
      std::floor((min_x - costmap_.info.origin.position.x) /
                 costmap_.info.resolution));
  int min_map_y = static_cast<int>(
      std::floor((min_y - costmap_.info.origin.position.y) /
                 costmap_.info.resolution));
  int max_map_x = static_cast<int>(
      std::ceil((max_x - costmap_.info.origin.position.x) /
                costmap_.info.resolution));
  int max_map_y = static_cast<int>(
      std::ceil((max_y - costmap_.info.origin.position.y) /
                costmap_.info.resolution));

  min_map_x = std::max(0, min_map_x);
  min_map_y = std::max(0, min_map_y);
  max_map_x = std::min(static_cast<int>(costmap_.info.width) - 1, max_map_x);
  max_map_y = std::min(static_cast<int>(costmap_.info.height) - 1, max_map_y);

  if (min_map_x > max_map_x || min_map_y > max_map_y) {
    return false;
  }

  int obstacle_cells = 0;
  int total_cells = 0;
  for (int map_x = min_map_x; map_x <= max_map_x; ++map_x) {
    for (int map_y = min_map_y; map_y <= max_map_y; ++map_y) {
      const double world_x =
          costmap_.info.origin.position.x + map_x * costmap_.info.resolution;
      const double world_y =
          costmap_.info.origin.position.y + map_y * costmap_.info.resolution;

      const double dx = world_x - robot_x;
      const double dy = world_y - robot_y;
      const double local_x = dx * cos_yaw + dy * sin_yaw;
      const double local_y = -dx * sin_yaw + dy * cos_yaw;

      if (local_x < x_min || local_x > x_max ||
          local_y < y_min || local_y > y_max) {
        continue;
      }

      const int cost = getCostAtMapCoordinate(map_x, map_y);
      if (cost >= 0) {
        ++total_cells;
        if (cost >= obstacle_threshold_) {
          ++obstacle_cells;
        }
      }
    }
  }

  const double obstacle_ratio =
      (total_cells > 0) ? static_cast<double>(obstacle_cells) / total_cells
                        : 0.0;
  if (debug_mode_) {
    RCLCPP_DEBUG_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "[FRONT_STOP_CHECK] x=[%.2f, %.2f], y=[%.2f, %.2f], cells=%d/%d, ratio=%.3f, threshold=%.3f",
        x_min, x_max, y_min, y_max, obstacle_cells, total_cells,
        obstacle_ratio, obstacle_ratio_threshold_);
  }

  return obstacle_ratio > obstacle_ratio_threshold_;
}

bool FrontObstacleDetector::findObstacleInSectorLocal(
    const geometry_msgs::msg::PoseStamped& robot_pose,
    double min_radius, double max_radius, double angle_deg,
    AvoidanceInfo& info) {
  if (!has_costmap_ || costmap_.info.resolution <= 0.0 ||
      costmap_.info.width == 0 || costmap_.info.height == 0) {
    return false;
  }

  const double robot_x = robot_pose.pose.position.x;
  const double robot_y = robot_pose.pose.position.y;
  const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
  const double cos_yaw = std::cos(robot_yaw);
  const double sin_yaw = std::sin(robot_yaw);

  const double min_x = robot_x - max_radius;
  const double max_x = robot_x + max_radius;
  const double min_y = robot_y - max_radius;
  const double max_y = robot_y + max_radius;

  int min_map_x = static_cast<int>(
      std::floor((min_x - costmap_.info.origin.position.x) /
                 costmap_.info.resolution));
  int min_map_y = static_cast<int>(
      std::floor((min_y - costmap_.info.origin.position.y) /
                 costmap_.info.resolution));
  int max_map_x = static_cast<int>(
      std::ceil((max_x - costmap_.info.origin.position.x) /
                costmap_.info.resolution));
  int max_map_y = static_cast<int>(
      std::ceil((max_y - costmap_.info.origin.position.y) /
                costmap_.info.resolution));

  min_map_x = std::max(0, min_map_x);
  min_map_y = std::max(0, min_map_y);
  max_map_x = std::min(static_cast<int>(costmap_.info.width) - 1, max_map_x);
  max_map_y = std::min(static_cast<int>(costmap_.info.height) - 1, max_map_y);

  if (min_map_x > max_map_x || min_map_y > max_map_y) {
    return false;
  }

  int total_cells = 0;
  int obstacle_cells = 0;
  int sector_total_cells = 0;
  int sector_obstacle_cells = 0;
  double weighted_x = 0.0;
  double weighted_y = 0.0;
  double weight_sum = 0.0;
  double closest_dist = std::numeric_limits<double>::max();
  double min_sector_obstacle_path_dist = std::numeric_limits<double>::max();
  double min_corridor_obstacle_path_dist = std::numeric_limits<double>::max();
  geometry_msgs::msg::Point closest_point;
  closest_point.z = robot_pose.pose.position.z;

  nav_msgs::msg::Path path_snapshot;
  bool has_path_snapshot = false;
  {
    std::lock_guard<std::mutex> lock(global_path_mutex_);
    has_path_snapshot = has_global_path_ && global_path_.poses.size() >= 2;
    if (has_path_snapshot) {
      path_snapshot = global_path_;
    }
  }

  int closest_path_index = 0;
  if (has_path_snapshot) {
    double closest_path_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < path_snapshot.poses.size(); ++i) {
      const auto& p = path_snapshot.poses[i].pose.position;
      const double dist = std::hypot(p.x - robot_x, p.y - robot_y);
      if (dist < closest_path_dist) {
        closest_path_dist = dist;
        closest_path_index = static_cast<int>(i);
      }
    }
  }

  const double corridor_half_width =
      robot_width_ * 0.5 + safety_margin_ +
      std::max(0.0, path_obstacle_margin_);
  const double max_check_distance =
      std::max(0.1, std::max(path_check_distance_, avoid_max_distance_));

  auto distance_to_checked_path = [&](double world_x,
                                      double world_y) -> double {
    if (!enable_path_corridor_filter_) {
      return 0.0;
    }
    if (!has_path_snapshot) {
      return std::numeric_limits<double>::max();
    }

    double checked_distance = 0.0;
    double min_dist = std::numeric_limits<double>::max();
    for (int i = closest_path_index;
         i + 1 < static_cast<int>(path_snapshot.poses.size()) &&
         checked_distance <= max_check_distance;
         ++i) {
      const auto& a = path_snapshot.poses[i].pose.position;
      const auto& b = path_snapshot.poses[i + 1].pose.position;
      const double segment_length = std::hypot(b.x - a.x, b.y - a.y);
      if (segment_length <= 1e-6) {
        continue;
      }

      min_dist =
          std::min(min_dist, pointToSegmentDistance(world_x, world_y, a, b));
      checked_distance += segment_length;
    }

    return min_dist;
  };

  for (int map_x = min_map_x; map_x <= max_map_x; ++map_x) {
    for (int map_y = min_map_y; map_y <= max_map_y; ++map_y) {
      const double world_x =
          costmap_.info.origin.position.x +
          (map_x + 0.5) * costmap_.info.resolution;
      const double world_y =
          costmap_.info.origin.position.y +
          (map_y + 0.5) * costmap_.info.resolution;

      const double dx = world_x - robot_x;
      const double dy = world_y - robot_y;
      const double local_x = dx * cos_yaw + dy * sin_yaw;
      const double local_y = -dx * sin_yaw + dy * cos_yaw;
      if (!isPointInSectorLocal(local_x, local_y, min_radius, max_radius,
                                angle_deg)) {
        continue;
      }

      const int cost = getCostAtMapCoordinate(map_x, map_y);
      if (cost < 0) {
        continue;
      }

      ++sector_total_cells;
      const double path_dist = distance_to_checked_path(world_x, world_y);
      const bool in_path_corridor =
          !enable_path_corridor_filter_ ||
          (has_path_snapshot && path_dist <= corridor_half_width);

      if (cost >= obstacle_threshold_) {
        ++sector_obstacle_cells;
        min_sector_obstacle_path_dist =
            std::min(min_sector_obstacle_path_dist, path_dist);
      }

      if (!in_path_corridor) {
        continue;
      }

      ++total_cells;
      if (cost >= obstacle_threshold_) {
        ++obstacle_cells;
        min_corridor_obstacle_path_dist =
            std::min(min_corridor_obstacle_path_dist, path_dist);
        const double weight = std::max(1.0, static_cast<double>(cost));
        weighted_x += world_x * weight;
        weighted_y += world_y * weight;
        weight_sum += weight;

        const double dist = std::hypot(dx, dy);
        if (dist < closest_dist) {
          closest_dist = dist;
          closest_point.x = world_x;
          closest_point.y = world_y;
        }
      }
    }
  }

  const double obstacle_ratio =
      (total_cells > 0) ? static_cast<double>(obstacle_cells) / total_cells
                        : 0.0;
  const double sector_obstacle_ratio =
      (sector_total_cells > 0)
          ? static_cast<double>(sector_obstacle_cells) / sector_total_cells
          : 0.0;
  const double printed_sector_min_dist =
      min_sector_obstacle_path_dist < std::numeric_limits<double>::max()
          ? min_sector_obstacle_path_dist
          : -1.0;
  const double printed_corridor_min_dist =
      min_corridor_obstacle_path_dist < std::numeric_limits<double>::max()
          ? min_corridor_obstacle_path_dist
          : -1.0;

  if (debug_mode_) {
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 500,
        "[AVOID_PATH_FILTER] path_filter=%s, has_path=%s, path_size=%zu, closest_idx=%d, corridor_half=%.2f, check_dist=%.2f, sector_obs=%d/%d(%.3f), corridor_obs=%d/%d(%.3f), min_path_dist sector/corridor=%.3f/%.3f",
        enable_path_corridor_filter_ ? "true" : "false",
        has_path_snapshot ? "true" : "false", path_snapshot.poses.size(),
        closest_path_index, corridor_half_width, max_check_distance,
        sector_obstacle_cells, sector_total_cells, sector_obstacle_ratio,
        obstacle_cells, total_cells, obstacle_ratio, printed_sector_min_dist,
        printed_corridor_min_dist);
  }

  if (obstacle_cells == 0 || obstacle_ratio <= obstacle_ratio_threshold_ ||
      weight_sum <= 0.0) {
    return false;
  }

  info.obstacle_center.x = weighted_x / weight_sum;
  info.obstacle_center.y = weighted_y / weight_sum;
  info.obstacle_center.z = robot_pose.pose.position.z;
  info.closest_obstacle = closest_point;
  info.obstacle_distance =
      std::hypot(info.obstacle_center.x - robot_x,
                 info.obstacle_center.y - robot_y);
  return true;
}

bool FrontObstacleDetector::isPassageClear(
    const geometry_msgs::msg::PoseStamped& robot_pose, bool check_left) {
  const double half_width = robot_width_ * 0.5 + safety_margin_;
  const double side_sign = check_left ? 1.0 : -1.0;
  const double x_min = std::max(0.0, robot_length_ * 0.5);
  const double x_max = std::max(x_min + 0.1, avoid_max_distance_ + 0.5);
  const double y_inner = side_sign * half_width;
  const double y_outer = side_sign * (half_width + side_passage_width_);
  const double y_min = std::min(y_inner, y_outer);
  const double y_max = std::max(y_inner, y_outer);
  return !detectObstacleInRectLocal(robot_pose, x_min, x_max, y_min, y_max);
}

bool FrontObstacleDetector::isPointInPathCorridor(
    const geometry_msgs::msg::PoseStamped& robot_pose, double world_x,
    double world_y) const {
  if (!enable_path_corridor_filter_) {
    return true;
  }

  nav_msgs::msg::Path path;
  {
    std::lock_guard<std::mutex> lock(global_path_mutex_);
    if (!has_global_path_ || global_path_.poses.size() < 2) {
      return true;
    }
    path = global_path_;
  }

  const double robot_x = robot_pose.pose.position.x;
  const double robot_y = robot_pose.pose.position.y;
  int closest_index = 0;
  double closest_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i < path.poses.size(); ++i) {
    const auto& p = path.poses[i].pose.position;
    const double dist = std::hypot(p.x - robot_x, p.y - robot_y);
    if (dist < closest_dist) {
      closest_dist = dist;
      closest_index = static_cast<int>(i);
    }
  }

  const double corridor_half_width =
      robot_width_ * 0.5 + safety_margin_ + std::max(0.0, path_obstacle_margin_);
  const double max_check_distance =
      std::max(0.1, std::max(path_check_distance_, avoid_max_distance_));

  double checked_distance = 0.0;
  for (int i = closest_index;
       i + 1 < static_cast<int>(path.poses.size()) &&
       checked_distance <= max_check_distance;
       ++i) {
    const auto& a = path.poses[i].pose.position;
    const auto& b = path.poses[i + 1].pose.position;
    const double segment_length = std::hypot(b.x - a.x, b.y - a.y);
    if (segment_length <= 1e-6) {
      continue;
    }

    if (pointToSegmentDistance(world_x, world_y, a, b) <=
        corridor_half_width) {
      return true;
    }
    checked_distance += segment_length;
  }

  return false;
}

double FrontObstacleDetector::pointToSegmentDistance(
    double px, double py, const geometry_msgs::msg::Point& a,
    const geometry_msgs::msg::Point& b) const {
  const double vx = b.x - a.x;
  const double vy = b.y - a.y;
  const double wx = px - a.x;
  const double wy = py - a.y;
  const double len_sq = vx * vx + vy * vy;
  if (len_sq <= 1e-12) {
    return std::hypot(px - a.x, py - a.y);
  }

  const double t = std::clamp((wx * vx + wy * vy) / len_sq, 0.0, 1.0);
  const double proj_x = a.x + t * vx;
  const double proj_y = a.y + t * vy;
  return std::hypot(px - proj_x, py - proj_y);
}

bool FrontObstacleDetector::isPointInSectorLocal(
    double local_x, double local_y, double min_radius, double max_radius,
    double angle_deg) const {
  if (local_x < 0.0) {
    return false;
  }
  const double distance = std::hypot(local_x, local_y);
  if (distance < min_radius || distance > max_radius) {
    return false;
  }
  const double half_angle = (angle_deg * M_PI / 180.0) * 0.5;
  const double angle = std::atan2(local_y, local_x);
  return std::abs(angle) <= half_angle;
}

const char* FrontObstacleDetector::decisionToString(
    ObstacleDecision decision) const {
  switch (decision) {
    case ObstacleDecision::AVOIDABLE:
      return "AVOIDABLE";
    case ObstacleDecision::BLOCKED:
      return "BLOCKED";
    case ObstacleDecision::HARD_STOP:
      return "HARD_STOP";
    default:
      return "CLEAR";
  }
}

void FrontObstacleDetector::generateSectorAreaPoints(
    const geometry_msgs::msg::PoseStamped& robot_pose,
    std::vector<std::pair<double, double>>& sector_points) {
  sector_points.clear();

  double robot_x = robot_pose.pose.position.x;
  double robot_y = robot_pose.pose.position.y;
  double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);

  // 扇形参数
  double half_angle = (sector_angle_ * M_PI / 180.0) / 2.0;  // 转换为弧度的半角
  double start_angle = robot_yaw - half_angle;
  double end_angle = robot_yaw + half_angle;

  // 添加扇形中心点（机器人位置）
  sector_points.push_back({robot_x, robot_y});

  // 生成扇形弧线上的点
  int num_arc_points = 20;  // 弧线分段数
  for (int i = 0; i <= num_arc_points; ++i) {
    double angle = start_angle + (end_angle - start_angle) * i / num_arc_points;
    double arc_x = robot_x + sector_radius_ * std::cos(angle);
    double arc_y = robot_y + sector_radius_ * std::sin(angle);
    sector_points.push_back({arc_x, arc_y});
  }

  if (debug_mode_) {
    RCLCPP_DEBUG_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "Generated sector with %zu points, radius: %.2fm, angle: %.1f°",
        sector_points.size(), sector_radius_, sector_angle_);
  }
}

bool FrontObstacleDetector::isPointInSector(double point_x, double point_y,
                                            double robot_x, double robot_y,
                                            double robot_yaw) {
  // 计算点到机器人的距离
  double dx = point_x - robot_x;
  double dy = point_y - robot_y;
  double distance = std::sqrt(dx * dx + dy * dy);

  // 检查距离是否在扇形半径内
  if (distance > sector_radius_) return false;

  // 计算点相对于机器人的角度
  double point_angle = std::atan2(dy, dx);
  double angle_diff = point_angle - robot_yaw;

  // 标准化角度差到 [-π, π]
  while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
  while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;

  // 检查角度是否在扇形范围内
  double half_angle = (sector_angle_ * M_PI / 180.0) / 2.0;  // 转换为弧度的半角
  return std::abs(angle_diff) <= half_angle;
}

}  // namespace pure_pursuit_local_planner
