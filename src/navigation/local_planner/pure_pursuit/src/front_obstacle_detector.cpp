#include "pure_pursuit/front_obstacle_detector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "pure_pursuit/param_utils.hpp"

namespace pure_pursuit_local_planner {

FrontObstacleDetector::FrontObstacleDetector()
    : has_costmap_(false),
      has_global_path_(false),
      obstacle_threshold_(50),
      obstacle_ratio_threshold_(0.1),
      robot_width_(0.80),
      robot_length_(1.10),
      safety_margin_(0.1),
      avoid_min_distance_(1.0),
      avoid_max_distance_(1.5),
      avoid_sector_angle_(60.0),
      side_passage_width_(0.75),
      enable_path_corridor_filter_(true),
      path_obstacle_margin_(0.15),
      path_check_distance_(2.0),
      debug_mode_(false),
      enable_visualization_(true) {}

bool FrontObstacleDetector::initialize(rclcpp::Node::SharedPtr node) {
  node_ = node;

  // 加载参数
  // 与A*共用阻塞阈值，保证检测和可通行性判断采用相同代价语义。
  obstacle_threshold_ =
      declareOrGetParameter(node_, "obstacle_cost_threshold", 50);
  obstacle_ratio_threshold_ = declareOrGetParameter(
      node_, "front_obstacle_detector.obstacle_ratio_threshold", 0.1);
  robot_width_ = declareOrGetParameter(node_, "robot_width", 0.80);
  robot_length_ = declareOrGetParameter(node_, "robot_length", 1.10);
  safety_margin_ = declareOrGetParameter(
      node_, "front_obstacle_detector.safety_margin", 0.1);
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
  RCLCPP_INFO(node_->get_logger(), "  - Obstacle threshold: %d",
              obstacle_threshold_);
  RCLCPP_INFO(
      node_->get_logger(),
      "  - Avoidance sector: r=[%.2f, %.2f], angle=%.1f, side passage=%.2f",
      avoid_min_distance_, avoid_max_distance_, avoid_sector_angle_,
      side_passage_width_);
  RCLCPP_INFO(
      node_->get_logger(),
      "  - Path corridor filter: %s, path_check=%.2fm, extra_margin=%.2fm",
      enable_path_corridor_filter_ ? "true" : "false", path_check_distance_,
      path_obstacle_margin_);
  RCLCPP_INFO(node_->get_logger(), "  - Visualization enabled: %s",
              enable_visualization_ ? "true" : "false");

  return true;
}

void FrontObstacleDetector::updateCostmap(
    CostmapConstPtr costmap) {
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  costmap_ = std::move(costmap);
  has_costmap_ = static_cast<bool>(costmap_);
}

void FrontObstacleDetector::setGlobalPath(
    const nav_msgs::msg::Path &global_path) {
  std::lock_guard<std::mutex> lock(global_path_mutex_);
  global_path_ = global_path;
  has_global_path_ = !global_path_.poses.empty();
  RCLCPP_DEBUG(node_->get_logger(),
               "[AVOID_PATH_FILTER] global path: poses=%zu, available=%s",
               global_path_.poses.size(), has_global_path_ ? "true" : "false");
}

FrontObstacleDetector::AvoidanceInfo FrontObstacleDetector::analyzeAvoidance(
    const geometry_msgs::msg::PoseStamped &robot_pose) {
  // 整次检测固定使用同一张只读Costmap；更新回调只替换shared_ptr。
  std::lock_guard<std::mutex> costmap_lock(costmap_mutex_);
  AvoidanceInfo info;
  info.obstacle_center.z = robot_pose.pose.position.z;

  if (!has_costmap_) {
    if (debug_mode_) {
      RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                            "No costmap available for avoidance analysis");
    }
    return info;
  }

  // 绕障触发区以内不再尝试临时侧向目标，直接判为阻塞并交给停障兜底。
  // 从车体前缘开始检查，避免把机器人自身附近的栅格纳入统计。
  const double near_min_distance = std::max(0.0, robot_length_ * 0.5);
  if (avoid_min_distance_ > near_min_distance &&
      findObstacleInSectorLocal(robot_pose, near_min_distance,
                                avoid_min_distance_, avoid_sector_angle_,
                                info)) {
    info.decision = ObstacleDecision::BLOCKED;
    if (enable_visualization_) {
      publishVisualization(robot_pose, true);
    }
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 500,
        "[AVOID_ANALYZE] BLOCKED near obstacle, dist=%.2fm < %.2fm",
        info.obstacle_distance, avoid_min_distance_);
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

  if (debug_mode_) {
    RCLCPP_DEBUG_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 500,
        "[AVOID_ANALYZE] %s center=(%.3f, %.3f), dist=%.2f, left=%s, "
        "right=%s",
        decisionToString(info.decision), info.obstacle_center.x,
        info.obstacle_center.y, info.obstacle_distance,
        info.left_clear ? "true" : "false",
        info.right_clear ? "true" : "false");
  }

  return info;
}

int FrontObstacleDetector::getCostAtMapCoordinate(int map_x, int map_y) {
  if (!has_costmap_ || map_x < 0 ||
      map_x >= static_cast<int>(costmap_->info.width) || map_y < 0 ||
      map_y >= static_cast<int>(costmap_->info.height)) {
    return -1;  // 越界
  }

  int index = map_y * costmap_->info.width + map_x;
  if (index >= 0 && index < static_cast<int>(costmap_->data.size())) {
    return costmap_->data[index];
  }

  return -1;
}

void FrontObstacleDetector::publishVisualization(
    const geometry_msgs::msg::PoseStamped &robot_pose, bool obstacle_detected) {
  if (!enable_visualization_) return;

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = robot_pose.header.frame_id;
  marker.header.stamp = node_->now();
  marker.ns = "front_obstacle_detection";
  marker.id = 0;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.scale.x = 0.02;

  std::vector<std::pair<double, double>> sector_points;
  generateSectorAreaPoints(robot_pose, sector_points);
  for (const auto &point : sector_points) {
    geometry_msgs::msg::Point p;
    p.x = point.first;
    p.y = point.second;
    p.z = robot_pose.pose.position.z + 0.1;
    marker.points.push_back(p);
  }

  marker.color.a = 0.35;
  marker.color.r = obstacle_detected ? 1.0 : 0.0;
  marker.color.g = obstacle_detected ? 0.0 : 1.0;
  marker.color.b = 0.0;

  // 设置生存时间
  marker.lifetime.sec = 0;
  marker.lifetime.nanosec = 200000000;  // 0.2s = 200ms

  detection_area_pub_->publish(marker);

}

bool FrontObstacleDetector::detectObstacleInRectLocal(
    const geometry_msgs::msg::PoseStamped &robot_pose, double x_min,
    double x_max, double y_min, double y_max) {
  if (!has_costmap_ || costmap_->info.resolution <= 0.0 ||
      costmap_->info.width == 0 || costmap_->info.height == 0) {
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
  for (const auto &corner : local_corners) {
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
  for (const auto &corner : corners) {
    min_x = std::min(min_x, corner.first);
    min_y = std::min(min_y, corner.second);
    max_x = std::max(max_x, corner.first);
    max_y = std::max(max_y, corner.second);
  }

  int min_map_x = static_cast<int>(std::floor(
      (min_x - costmap_->info.origin.position.x) / costmap_->info.resolution));
  int min_map_y = static_cast<int>(std::floor(
      (min_y - costmap_->info.origin.position.y) / costmap_->info.resolution));
  int max_map_x = static_cast<int>(std::ceil(
      (max_x - costmap_->info.origin.position.x) / costmap_->info.resolution));
  int max_map_y = static_cast<int>(std::ceil(
      (max_y - costmap_->info.origin.position.y) / costmap_->info.resolution));

  min_map_x = std::max(0, min_map_x);
  min_map_y = std::max(0, min_map_y);
  max_map_x = std::min(static_cast<int>(costmap_->info.width) - 1, max_map_x);
  max_map_y = std::min(static_cast<int>(costmap_->info.height) - 1, max_map_y);

  if (min_map_x > max_map_x || min_map_y > max_map_y) {
    return false;
  }

  int obstacle_cells = 0;
  int total_cells = 0;
  for (int map_x = min_map_x; map_x <= max_map_x; ++map_x) {
    for (int map_y = min_map_y; map_y <= max_map_y; ++map_y) {
      const double world_x =
          costmap_->info.origin.position.x + map_x * costmap_->info.resolution;
      const double world_y =
          costmap_->info.origin.position.y + map_y * costmap_->info.resolution;

      const double dx = world_x - robot_x;
      const double dy = world_y - robot_y;
      const double local_x = dx * cos_yaw + dy * sin_yaw;
      const double local_y = -dx * sin_yaw + dy * cos_yaw;

      if (local_x < x_min || local_x > x_max || local_y < y_min ||
          local_y > y_max) {
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
    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                          "[FRONT_STOP_CHECK] x=[%.2f, %.2f], y=[%.2f, %.2f], "
                          "cells=%d/%d, ratio=%.3f, threshold=%.3f",
                          x_min, x_max, y_min, y_max, obstacle_cells,
                          total_cells, obstacle_ratio,
                          obstacle_ratio_threshold_);
  }

  return obstacle_ratio > obstacle_ratio_threshold_;
}

bool FrontObstacleDetector::findObstacleInSectorLocal(
    const geometry_msgs::msg::PoseStamped &robot_pose, double min_radius,
    double max_radius, double angle_deg, AvoidanceInfo &info) {
  if (!has_costmap_ || costmap_->info.resolution <= 0.0 ||
      costmap_->info.width == 0 || costmap_->info.height == 0) {
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

  int min_map_x = static_cast<int>(std::floor(
      (min_x - costmap_->info.origin.position.x) / costmap_->info.resolution));
  int min_map_y = static_cast<int>(std::floor(
      (min_y - costmap_->info.origin.position.y) / costmap_->info.resolution));
  int max_map_x = static_cast<int>(std::ceil(
      (max_x - costmap_->info.origin.position.x) / costmap_->info.resolution));
  int max_map_y = static_cast<int>(std::ceil(
      (max_y - costmap_->info.origin.position.y) / costmap_->info.resolution));

  min_map_x = std::max(0, min_map_x);
  min_map_y = std::max(0, min_map_y);
  max_map_x = std::min(static_cast<int>(costmap_->info.width) - 1, max_map_x);
  max_map_y = std::min(static_cast<int>(costmap_->info.height) - 1, max_map_y);

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
  double min_sector_obstacle_path_dist = std::numeric_limits<double>::max();
  double min_corridor_obstacle_path_dist = std::numeric_limits<double>::max();

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
      const auto &p = path_snapshot.poses[i].pose.position;
      const double dist = std::hypot(p.x - robot_x, p.y - robot_y);
      if (dist < closest_path_dist) {
        closest_path_dist = dist;
        closest_path_index = static_cast<int>(i);
      }
    }
  }

  const double corridor_half_width = robot_width_ * 0.5 + safety_margin_ +
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
      const auto &a = path_snapshot.poses[i].pose.position;
      const auto &b = path_snapshot.poses[i + 1].pose.position;
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
      const double world_x = costmap_->info.origin.position.x +
                             (map_x + 0.5) * costmap_->info.resolution;
      const double world_y = costmap_->info.origin.position.y +
                             (map_y + 0.5) * costmap_->info.resolution;

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
    RCLCPP_DEBUG_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 500,
        "[AVOID_PATH_FILTER] path_filter=%s, has_path=%s, path_size=%zu, "
        "closest_idx=%d, corridor_half=%.2f, check_dist=%.2f, "
        "sector_obs=%d/%d(%.3f), corridor_obs=%d/%d(%.3f), min_path_dist "
        "sector/corridor=%.3f/%.3f",
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
  info.obstacle_distance = std::hypot(info.obstacle_center.x - robot_x,
                                      info.obstacle_center.y - robot_y);
  return true;
}

bool FrontObstacleDetector::isPassageClear(
    const geometry_msgs::msg::PoseStamped &robot_pose, bool check_left) {
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

double FrontObstacleDetector::pointToSegmentDistance(
    double px, double py, const geometry_msgs::msg::Point &a,
    const geometry_msgs::msg::Point &b) const {
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

bool FrontObstacleDetector::isPointInSectorLocal(double local_x, double local_y,
                                                 double min_radius,
                                                 double max_radius,
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

const char *FrontObstacleDetector::decisionToString(
    ObstacleDecision decision) const {
  switch (decision) {
    case ObstacleDecision::AVOIDABLE:
      return "AVOIDABLE";
    case ObstacleDecision::BLOCKED:
      return "BLOCKED";
    default:
      return "CLEAR";
  }
}

void FrontObstacleDetector::generateSectorAreaPoints(
    const geometry_msgs::msg::PoseStamped &robot_pose,
    std::vector<std::pair<double, double>> &sector_points) {
  sector_points.clear();

  const double robot_x = robot_pose.pose.position.x;
  const double robot_y = robot_pose.pose.position.y;
  const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
  const double half_angle = (avoid_sector_angle_ * M_PI / 180.0) * 0.5;
  const double start_angle = robot_yaw - half_angle;
  const double end_angle = robot_yaw + half_angle;
  constexpr int num_arc_points = 20;

  // 按“内弧起点 -> 外弧 -> 内弧反向”的顺序生成真实绕障扇环。
  sector_points.push_back(
      {robot_x + avoid_min_distance_ * std::cos(start_angle),
       robot_y + avoid_min_distance_ * std::sin(start_angle)});
  for (int i = 0; i <= num_arc_points; ++i) {
    const double angle =
        start_angle + (end_angle - start_angle) * i / num_arc_points;
    sector_points.push_back({robot_x + avoid_max_distance_ * std::cos(angle),
                             robot_y + avoid_max_distance_ * std::sin(angle)});
  }
  for (int i = num_arc_points; i >= 0; --i) {
    const double angle =
        start_angle + (end_angle - start_angle) * i / num_arc_points;
    sector_points.push_back({robot_x + avoid_min_distance_ * std::cos(angle),
                             robot_y + avoid_min_distance_ * std::sin(angle)});
  }

  if (debug_mode_) {
    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                          "Generated avoidance sector with %zu points, radius: "
                          "[%.2f, %.2f]m, angle: %.1f°",
                          sector_points.size(), avoid_min_distance_,
                          avoid_max_distance_, avoid_sector_angle_);
  }
}

}  // namespace pure_pursuit_local_planner
