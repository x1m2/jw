#include "pure_pursuit/avoidance_path_planner.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace pure_pursuit_local_planner {
namespace {

constexpr double kPointEpsilon = 1e-4;
constexpr double kMinimumCornerDistance = 0.03;
constexpr double kMinimumTurnAngle = 8.0 * M_PI / 180.0;
constexpr int kBezierCollisionRetries = 6;

double pointToSegmentDistance(const geometry_msgs::msg::Point &point,
                              const geometry_msgs::msg::Point &start,
                              const geometry_msgs::msg::Point &end) {
  const double dx = end.x - start.x;
  const double dy = end.y - start.y;
  const double length_squared = dx * dx + dy * dy;
  if (length_squared <= 1e-12) {
    return std::hypot(point.x - start.x, point.y - start.y);
  }
  const double ratio = std::clamp(
      ((point.x - start.x) * dx + (point.y - start.y) * dy) /
          length_squared,
      0.0, 1.0);
  return std::hypot(point.x - (start.x + ratio * dx),
                    point.y - (start.y + ratio * dy));
}

double pointToPathDistance(const geometry_msgs::msg::Point &point,
                           const nav_msgs::msg::Path &path) {
  if (path.poses.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  if (path.poses.size() == 1) {
    const auto &only = path.poses.front().pose.position;
    return std::hypot(point.x - only.x, point.y - only.y);
  }
  double minimum_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 1; i < path.poses.size(); ++i) {
    minimum_distance = std::min(
        minimum_distance,
        pointToSegmentDistance(point, path.poses[i - 1].pose.position,
                               path.poses[i].pose.position));
  }
  return minimum_distance;
}

geometry_msgs::msg::Point cubicBezierPoint(
    const geometry_msgs::msg::Point &p0,
    const geometry_msgs::msg::Point &p1,
    const geometry_msgs::msg::Point &p2,
    const geometry_msgs::msg::Point &p3, double t) {
  const double one_minus_t = 1.0 - t;
  const double b0 = one_minus_t * one_minus_t * one_minus_t;
  const double b1 = 3.0 * one_minus_t * one_minus_t * t;
  const double b2 = 3.0 * one_minus_t * t * t;
  const double b3 = t * t * t;
  geometry_msgs::msg::Point point;
  point.x = b0 * p0.x + b1 * p1.x + b2 * p2.x + b3 * p3.x;
  point.y = b0 * p0.y + b1 * p1.y + b2 * p2.y + b3 * p3.y;
  point.z = b0 * p0.z + b1 * p1.z + b2 * p2.z + b3 * p3.z;
  return point;
}

double cubicBezierMinimumRadius(
    const geometry_msgs::msg::Point &p0,
    const geometry_msgs::msg::Point &p1,
    const geometry_msgs::msg::Point &p2,
    const geometry_msgs::msg::Point &p3, int sample_count) {
  double maximum_curvature = 0.0;
  for (int sample = 0; sample <= sample_count; ++sample) {
    const double t = static_cast<double>(sample) / sample_count;
    const double one_minus_t = 1.0 - t;
    const double dx = 3.0 *
        (one_minus_t * one_minus_t * (p1.x - p0.x) +
         2.0 * one_minus_t * t * (p2.x - p1.x) +
         t * t * (p3.x - p2.x));
    const double dy = 3.0 *
        (one_minus_t * one_minus_t * (p1.y - p0.y) +
         2.0 * one_minus_t * t * (p2.y - p1.y) +
         t * t * (p3.y - p2.y));
    const double ddx = 6.0 *
        (one_minus_t * (p2.x - 2.0 * p1.x + p0.x) +
         t * (p3.x - 2.0 * p2.x + p1.x));
    const double ddy = 6.0 *
        (one_minus_t * (p2.y - 2.0 * p1.y + p0.y) +
         t * (p3.y - 2.0 * p2.y + p1.y));
    const double speed_squared = dx * dx + dy * dy;
    if (speed_squared <= 1e-12) return 0.0;
    const double curvature =
        std::abs(dx * ddy - dy * ddx) /
        std::pow(speed_squared, 1.5);
    maximum_curvature = std::max(maximum_curvature, curvature);
  }
  if (maximum_curvature <= 1e-9) {
    return std::numeric_limits<double>::infinity();
  }
  return 1.0 / maximum_curvature;
}

void appendDistinctPose(nav_msgs::msg::Path &path,
                        geometry_msgs::msg::PoseStamped pose) {
  pose.header = path.header;
  if (!path.poses.empty()) {
    const auto &last = path.poses.back().pose.position;
    if (std::hypot(pose.pose.position.x - last.x,
                   pose.pose.position.y - last.y) < kPointEpsilon) {
      return;
    }
  }
  path.poses.push_back(std::move(pose));
}

void updatePathOrientations(
    nav_msgs::msg::Path &path,
    const geometry_msgs::msg::Quaternion &terminal_orientation) {
  if (path.poses.empty()) return;

  for (std::size_t i = 0; i + 1 < path.poses.size(); ++i) {
    const auto &current = path.poses[i].pose.position;
    const auto &next = path.poses[i + 1].pose.position;
    const double dx = next.x - current.x;
    const double dy = next.y - current.y;
    if (std::hypot(dx, dy) < kPointEpsilon) continue;
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, std::atan2(dy, dx));
    path.poses[i].pose.orientation = tf2::toMsg(orientation);
  }
  path.poses.back().pose.orientation = terminal_orientation;
}

nav_msgs::msg::Path densifyPath(const nav_msgs::msg::Path &path,
                                double maximum_spacing) {
  if (path.poses.size() < 2) return path;
  nav_msgs::msg::Path result;
  result.header = path.header;
  appendDistinctPose(result, path.poses.front());
  for (std::size_t i = 1; i < path.poses.size(); ++i) {
    const auto &start = path.poses[i - 1].pose.position;
    const auto &end = path.poses[i].pose.position;
    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const double dz = end.z - start.z;
    const int sample_count = std::max(
        1, static_cast<int>(std::ceil(std::hypot(dx, dy) /
                                     maximum_spacing)));
    for (int sample = 1; sample <= sample_count; ++sample) {
      const double ratio = static_cast<double>(sample) / sample_count;
      geometry_msgs::msg::PoseStamped pose = path.poses[i];
      pose.pose.position.x = start.x + ratio * dx;
      pose.pose.position.y = start.y + ratio * dy;
      pose.pose.position.z = start.z + ratio * dz;
      appendDistinctPose(result, std::move(pose));
    }
  }
  return result;
}

}  // namespace

bool AvoidancePathPlanner::initialize(
    rclcpp::Node::SharedPtr node, const AvoidancePathPlannerConfig &config) {
  node_ = std::move(node);
  config_ = config;
  astar_planner_ = std::make_unique<AStarPlanner>();
  if (!astar_planner_->initialize(node_)) return false;
  if (config_.bezier_visualization_enabled) {
    raw_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
        "local_astar_raw_path", 1);
    smoothed_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
        "local_astar_bezier_path", 1);
  }
  return true;
}

void AvoidancePathPlanner::setGlobalPath(
    const nav_msgs::msg::Path &global_path) {
  if (astar_planner_)
    astar_planner_->setGlobalPath(global_path);
}

void AvoidancePathPlanner::updateCostmap(
    CostmapConstPtr costmap) {
  if (astar_planner_)
    astar_planner_->setCostmap(std::move(costmap));
}

bool AvoidancePathPlanner::prepareLatestCostmap() {
  return astar_planner_ && astar_planner_->prepareLatestCostmap();
}

bool AvoidancePathPlanner::hasPreparedCostmap() const {
  return astar_planner_ && astar_planner_->hasPreparedCostmap();
}

bool AvoidancePathPlanner::buildSideGoal(
    const geometry_msgs::msg::PoseStamped &robot_pose,
    const geometry_msgs::msg::Point &obstacle_center, AvoidanceSide side,
    const nav_msgs::msg::Path &global_path, int closest_index,
    geometry_msgs::msg::Point &side_goal, double &side_goal_yaw,
    double lateral_clearance, double max_entry_angle) {
  if (!astar_planner_ || global_path.poses.size() < 2)
    return false;
  if (closest_index < 0)
    closest_index = 0;

  double path_yaw = 0.0;
  if (!getPathYawAtIndex(global_path, closest_index, path_yaw)) {
    // 全局路径切向无法计算时，沿用原逻辑退化为机器人当前朝向。
    path_yaw = tf2::getYaw(robot_pose.pose.orientation);
  }
  side_goal_yaw = path_yaw;

  const double side_sign = side == AvoidanceSide::LEFT ? 1.0 : -1.0;
  const double path_x = std::cos(path_yaw);
  const double path_y = std::sin(path_yaw);
  const double normal_x = -std::sin(path_yaw) * side_sign;
  const double normal_y = std::cos(path_yaw) * side_sign;
  const double base_clearance = std::max(
      0.2, std::isfinite(lateral_clearance) ? lateral_clearance
                                            : config_.lateral_clearance);
  const double robot_lateral =
      (robot_pose.pose.position.x - obstacle_center.x) * normal_x +
      (robot_pose.pose.position.y - obstacle_center.y) * normal_y;
  const double lateral_delta = std::abs(base_clearance - robot_lateral);
  const double entry_angle = std::clamp(
      max_entry_angle, 1.0 * M_PI / 180.0, 89.0 * M_PI / 180.0);
  const double minimum_forward = lateral_delta / std::tan(entry_angle);
  const double robot_forward =
      (robot_pose.pose.position.x - obstacle_center.x) * path_x +
      (robot_pose.pose.position.y - obstacle_center.y) * path_y;
  const double goal_forward = std::max(0.0, robot_forward + minimum_forward);

  side_goal.x = obstacle_center.x + path_x * goal_forward +
                normal_x * base_clearance;
  side_goal.y = obstacle_center.y + path_y * goal_forward +
                normal_y * base_clearance;
  side_goal.z = obstacle_center.z;

  RCLCPP_INFO(node_->get_logger(),
              "[AVOID_POINT_GEN] side=%s obstacle=(%.3f, %.3f), "
              "path_yaw=%.1f deg, lateral=%.3f forward=%.3f "
              "max_entry=%.1f deg, goal=(%.3f, %.3f), obstacle_dist=%.3f",
              side == AvoidanceSide::LEFT ? "LEFT" : "RIGHT",
              obstacle_center.x, obstacle_center.y, path_yaw * 180.0 / M_PI,
              base_clearance, goal_forward, entry_angle * 180.0 / M_PI,
              side_goal.x, side_goal.y,
              calculateDistance(side_goal, obstacle_center));

  if (isPointInObstacleWithDiagnostics(side_goal, "SIDE_GENERATED", path_yaw)) {
    RCLCPP_INFO(node_->get_logger(), "[AVOID_POINT_GEN] rejected: blocked");
    return false;
  }

  RCLCPP_DEBUG(node_->get_logger(),
               "[AVOID_PLAN] selected side=%s obstacle=(%.3f, %.3f), "
               "path_yaw=%.1f deg, goal=(%.3f, %.3f)",
               side == AvoidanceSide::LEFT ? "LEFT" : "RIGHT",
               obstacle_center.x, obstacle_center.y, path_yaw * 180.0 / M_PI,
               side_goal.x, side_goal.y);
  return true;
}

bool AvoidancePathPlanner::buildReturnPath(
    const geometry_msgs::msg::Point &start_point, int closest_index,
    const geometry_msgs::msg::Point &obstacle_center,
    const nav_msgs::msg::Path &global_path,
    geometry_msgs::msg::Point &return_goal, nav_msgs::msg::Path &return_path,
    double max_return_angle, double required_obstacle_clearance,
    bool &used_direct_path) {
  if (closest_index < 0 ||
      closest_index >= static_cast<int>(global_path.poses.size()) - 1) {
    return false;
  }

  const double angle_limit = std::clamp(
      max_return_angle, 1.0 * M_PI / 180.0, 89.0 * M_PI / 180.0);
  const double hard_clearance = std::max(0.0, required_obstacle_clearance);
  auto is_return_candidate_safe = [this, &obstacle_center, &start_point,
                                   angle_limit](const auto &candidate_return,
                                                double path_yaw) {
    const double obstacle_distance =
        calculateDistance(candidate_return, obstacle_center);
    if (obstacle_distance < config_.return_obstacle_clearance)
      return false;
    if (isPointInObstacle(candidate_return))
      return false;
    if (!isPointSafeForRobot(candidate_return, config_.return_goal_clearance))
      return false;
    const double return_yaw = std::atan2(candidate_return.y - start_point.y,
                                         candidate_return.x - start_point.x);
    const double angle_error = std::abs(std::atan2(
        std::sin(return_yaw - path_yaw), std::cos(return_yaw - path_yaw)));
    return angle_error <= angle_limit;
  };

  auto make_direct_path = [this, &global_path](
                              const geometry_msgs::msg::Point &start,
                              const geometry_msgs::msg::Point &goal) {
    nav_msgs::msg::Path path;
    path.header = global_path.header;
    path.header.stamp = node_->now();
    const double dx = goal.x - start.x;
    const double dy = goal.y - start.y;
    const double distance = std::hypot(dx, dy);
    const double yaw = std::atan2(dy, dx);
    const int sample_count = std::max(1, static_cast<int>(std::ceil(
        distance / 0.10)));
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, yaw);
    for (int sample = 0; sample <= sample_count; ++sample) {
      const double ratio = static_cast<double>(sample) / sample_count;
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = start.x + ratio * dx;
      pose.pose.position.y = start.y + ratio * dy;
      pose.pose.position.z = start.z + ratio * (goal.z - start.z);
      pose.pose.orientation = tf2::toMsg(orientation);
      path.poses.push_back(std::move(pose));
    }
    return path;
  };

  // 原始全局路径可能很稀疏；每0.25m检查一个插值候选，按累计前进顺序
  // 选择第一个满足角度、点安全和路径安全的回归点。
  constexpr double kCandidateSpacing = 0.25;
  for (int segment_index = closest_index;
       segment_index + 1 < static_cast<int>(global_path.poses.size());
       ++segment_index) {
    const auto &segment_start =
        global_path.poses[segment_index].pose.position;
    const auto &segment_end =
        global_path.poses[segment_index + 1].pose.position;
    const double segment_dx = segment_end.x - segment_start.x;
    const double segment_dy = segment_end.y - segment_start.y;
    const double segment_length = std::hypot(segment_dx, segment_dy);
    if (segment_length < 1e-6) continue;
    const double path_yaw = std::atan2(segment_dy, segment_dx);
    const int sample_count = std::max(
        1, static_cast<int>(std::ceil(segment_length / kCandidateSpacing)));

    for (int sample = 1; sample <= sample_count; ++sample) {
      const double ratio = static_cast<double>(sample) / sample_count;
      geometry_msgs::msg::Point candidate_return;
      candidate_return.x = segment_start.x + ratio * segment_dx;
      candidate_return.y = segment_start.y + ratio * segment_dy;
      candidate_return.z = segment_start.z +
                           ratio * (segment_end.z - segment_start.z);
      if (!is_return_candidate_safe(candidate_return, path_yaw)) continue;

      nav_msgs::msg::Path candidate_path =
          make_direct_path(start_point, candidate_return);
      double minimum_clearance = 0.0;
      const bool direct_safe = isPathCollisionFree(candidate_path) &&
          isPathOutsideObstacleRadius(candidate_path, obstacle_center,
                                      hard_clearance, &minimum_clearance);
      if (direct_safe) {
        return_goal = candidate_return;
        return_path = std::move(candidate_path);
        used_direct_path = true;
        RCLCPP_INFO(node_->get_logger(),
                    "[AVOID_PLAN] selected interpolated return goal "
                    "segment=%d ratio=%.2f: (%.3f, %.3f), mode=DIRECT",
                    segment_index, ratio, return_goal.x, return_goal.y);
        return true;
      }

      candidate_path.poses.clear();
      if (!planPath(start_point, candidate_return, candidate_path) ||
          candidate_path.poses.empty() ||
          !isPathOutsideObstacleRadius(candidate_path, obstacle_center,
                                       hard_clearance, &minimum_clearance)) {
        continue;
      }

      nav_msgs::msg::Path smoothed_candidate;
      if (smoothPathWithBezier(candidate_path, smoothed_candidate)) {
        double smoothed_minimum_clearance = 0.0;
        if (isPathOutsideObstacleRadius(
                smoothed_candidate, obstacle_center, hard_clearance,
                &smoothed_minimum_clearance)) {
          candidate_path = std::move(smoothed_candidate);
        } else {
          RCLCPP_WARN(
              node_->get_logger(),
              "[BEZIER_CLEARANCE_FALLBACK] return path smooth_min=%.3f "
              "required=%.3f; using raw A*",
              smoothed_minimum_clearance, hard_clearance);
        }
      }
      return_goal = candidate_return;
      return_path = std::move(candidate_path);
      used_direct_path = false;
      RCLCPP_INFO(node_->get_logger(),
                  "[AVOID_PLAN] selected interpolated return goal "
                  "segment=%d ratio=%.2f: (%.3f, %.3f), mode=ASTAR",
                  segment_index, ratio, return_goal.x, return_goal.y);
      return true;
    }
  }
  return false;
}

bool AvoidancePathPlanner::planPath(const geometry_msgs::msg::Point &start,
                                    const geometry_msgs::msg::Point &goal,
                                    nav_msgs::msg::Path &path,
                                    double goal_yaw) {
  if (!astar_planner_ ||
      !astar_planner_->planPath(start, goal, path, goal_yaw)) {
    return false;
  }
  const double terminal_yaw = std::isfinite(goal_yaw)
                                  ? goal_yaw
                                  : std::atan2(goal.y - start.y,
                                               goal.x - start.x);
  tf2::Quaternion terminal_orientation;
  terminal_orientation.setRPY(0.0, 0.0, terminal_yaw);
  updatePathOrientations(path, tf2::toMsg(terminal_orientation));
  return true;
}

bool AvoidancePathPlanner::smoothPathWithBezier(
    const nav_msgs::msg::Path &raw_path,
    nav_msgs::msg::Path &smoothed_path) {
  smoothed_path = raw_path;
  if (!config_.bezier_smoothing_enabled || !astar_planner_ ||
      raw_path.poses.size() < 3) {
    publishSmoothingPaths(raw_path, smoothed_path);
    return false;
  }

  // 只清理完全重复的相邻点；失败回退时仍使用调用者持有的原始A*路径。
  nav_msgs::msg::Path duplicate_free_path;
  duplicate_free_path.header = raw_path.header;
  for (const auto &pose : raw_path.poses) {
    appendDistinctPose(duplicate_free_path, pose);
  }
  if (duplicate_free_path.poses.size() < 3) {
    publishSmoothingPaths(raw_path, smoothed_path);
    return false;
  }

  // A*同一方向上的连续栅格点不承担几何约束，先压缩成折线顶点，才能
  // 根据整段长度生成稳定圆角，而不是被单个栅格的长度限制。
  nav_msgs::msg::Path clean_path;
  clean_path.header = raw_path.header;
  appendDistinctPose(clean_path, duplicate_free_path.poses.front());
  for (std::size_t i = 1; i + 1 < duplicate_free_path.poses.size(); ++i) {
    const auto &previous = duplicate_free_path.poses[i - 1].pose.position;
    const auto &current = duplicate_free_path.poses[i].pose.position;
    const auto &next = duplicate_free_path.poses[i + 1].pose.position;
    const double first_x = current.x - previous.x;
    const double first_y = current.y - previous.y;
    const double second_x = next.x - current.x;
    const double second_y = next.y - current.y;
    const double scale = std::hypot(first_x, first_y) *
                         std::hypot(second_x, second_y);
    const double cross = first_x * second_y - first_y * second_x;
    const double dot = first_x * second_x + first_y * second_y;
    if (scale > 1e-12 && std::abs(cross) <= 1e-6 * scale && dot > 0.0) {
      continue;
    }
    appendDistinctPose(clean_path, duplicate_free_path.poses[i]);
  }
  appendDistinctPose(clean_path, duplicate_free_path.poses.back());

  const std::size_t direction_vertex_count = clean_path.poses.size();
  const double sample_spacing = std::max(
      0.01, config_.bezier_sample_spacing);
  const double simplify_tolerance = std::max(
      0.0, config_.bezier_simplify_tolerance);
  bool simplification_applied = false;

  // A*常用的8邻域会产生“直线-斜线-直线”的短阶梯。逐个阶梯做圆角
  // 不仅不平滑，还会因短线段无法满足最小转弯半径而大量回退。这里从
  // 当前锚点选择偏差合格且完整footprint可通行的最远顶点，先把同一走向
  // 的栅格阶梯合并为长折线；碰撞校验保证不会用直线抄近路穿过障碍物。
  if (simplify_tolerance > 0.0 && clean_path.poses.size() >= 3) {
    nav_msgs::msg::Path simplified;
    simplified.header = clean_path.header;
    appendDistinctPose(simplified, clean_path.poses.front());
    std::size_t anchor = 0;
    while (anchor + 1 < clean_path.poses.size()) {
      std::size_t selected = anchor + 1;
      for (std::size_t candidate_index = clean_path.poses.size() - 1;
           candidate_index > anchor + 1; --candidate_index) {
        const auto &segment_start =
            clean_path.poses[anchor].pose.position;
        const auto &segment_end =
            clean_path.poses[candidate_index].pose.position;
        bool deviation_valid = true;
        for (std::size_t point_index = anchor + 1;
             point_index < candidate_index; ++point_index) {
          if (pointToSegmentDistance(
                  clean_path.poses[point_index].pose.position,
                  segment_start, segment_end) >
              simplify_tolerance + 1e-9) {
            deviation_valid = false;
            break;
          }
        }
        if (!deviation_valid) continue;

        nav_msgs::msg::Path shortcut;
        shortcut.header = clean_path.header;
        appendDistinctPose(shortcut, clean_path.poses[anchor]);
        appendDistinctPose(shortcut, clean_path.poses[candidate_index]);
        shortcut = densifyPath(shortcut, sample_spacing);
        updatePathOrientations(
            shortcut,
            clean_path.poses[candidate_index].pose.orientation);
        if (!isPathCollisionFree(shortcut)) continue;

        selected = candidate_index;
        break;
      }
      appendDistinctPose(simplified, clean_path.poses[selected]);
      anchor = selected;
    }
    if (simplified.poses.size() < clean_path.poses.size()) {
      clean_path = std::move(simplified);
      simplification_applied = true;
    }
  }

  // 阶梯简化后可能已经成为一条安全直线，此时无需为了“必须存在贝塞尔
  // 拐点”而退回锯齿状原始A*。
  if (clean_path.poses.size() < 3) {
    if (!simplification_applied) {
      publishSmoothingPaths(raw_path, smoothed_path);
      return false;
    }
    clean_path = densifyPath(clean_path, sample_spacing);
    updatePathOrientations(clean_path,
                           raw_path.poses.back().pose.orientation);
    if (!isPathCollisionFree(clean_path)) {
      publishSmoothingPaths(raw_path, smoothed_path);
      return false;
    }
    smoothed_path = std::move(clean_path);
    publishSmoothingPaths(raw_path, smoothed_path);
    RCLCPP_INFO(node_->get_logger(),
                "[BEZIER_SMOOTH] raw=%zu direction_vertices=%zu "
                "simplified_vertices=2 smooth=%zu corners=0 accepted=0 "
                "fallback=0 mode=shortcut",
                raw_path.poses.size(), direction_vertex_count,
                smoothed_path.poses.size());
    return true;
  }

  nav_msgs::msg::Path candidate;
  candidate.header = raw_path.header;
  appendDistinctPose(candidate, clean_path.poses.front());

  const double configured_corner_distance =
      std::max(kMinimumCornerDistance, config_.bezier_corner_distance);
  const double minimum_corner_distance =
      std::min(kMinimumCornerDistance, configured_corner_distance);
  const double minimum_turn = kMinimumTurnAngle;
  const double target_turn_radius = std::max(
      config_.bezier_min_turn_radius, config_.bezier_target_turn_radius);
  const double minimum_turn_radius = std::max(
      0.0, config_.bezier_min_turn_radius);
  const double maximum_deviation = std::max(
      0.0, config_.bezier_max_deviation);

  int considered_corners = 0;
  int accepted_corners = 0;
  int fallback_corners = 0;
  double achieved_minimum_radius = std::numeric_limits<double>::infinity();

  for (std::size_t i = 1; i + 1 < clean_path.poses.size(); ++i) {
    const auto &previous = clean_path.poses[i - 1].pose.position;
    const auto &corner = clean_path.poses[i].pose.position;
    const auto &next = clean_path.poses[i + 1].pose.position;
    const double incoming_x = corner.x - previous.x;
    const double incoming_y = corner.y - previous.y;
    const double outgoing_x = next.x - corner.x;
    const double outgoing_y = next.y - corner.y;
    const double incoming_length = std::hypot(incoming_x, incoming_y);
    const double outgoing_length = std::hypot(outgoing_x, outgoing_y);
    if (incoming_length < kPointEpsilon ||
        outgoing_length < kPointEpsilon) {
      appendDistinctPose(candidate, clean_path.poses[i]);
      continue;
    }

    const double incoming_unit_x = incoming_x / incoming_length;
    const double incoming_unit_y = incoming_y / incoming_length;
    const double outgoing_unit_x = outgoing_x / outgoing_length;
    const double outgoing_unit_y = outgoing_y / outgoing_length;
    const double turn_angle = std::acos(std::clamp(
        incoming_unit_x * outgoing_unit_x +
            incoming_unit_y * outgoing_unit_y,
        -1.0, 1.0));
    // 近似直线无需增加点；接近掉头的路径保留原折角，避免贝塞尔产生尖点。
    if (turn_angle < minimum_turn ||
        turn_angle > 170.0 * M_PI / 180.0) {
      appendDistinctPose(candidate, clean_path.poses[i]);
      continue;
    }

    ++considered_corners;
    const double maximum_tangent_distance = std::min(
        {configured_corner_distance, incoming_length * 0.4,
         outgoing_length * 0.4});
    const double corner_minimum_distance =
        std::min(minimum_corner_distance, maximum_tangent_distance);
    const double tangent_scale = std::tan(turn_angle * 0.5);
    if (!std::isfinite(tangent_scale) || tangent_scale <= 1e-6) {
      appendDistinctPose(candidate, clean_path.poses[i]);
      ++fallback_corners;
      continue;
    }

    // 用圆弧的切线几何决定贝塞尔入口/出口：
    // tangent_distance = radius * tan(turn_angle / 2)。先尽量达到
    // 期望半径，路段长度不足时使用当前几何能容纳的最大半径。
    double turn_radius = std::min(
        target_turn_radius, maximum_tangent_distance / tangent_scale);
    bool corner_accepted = false;
    nav_msgs::msg::Path accepted_corner;
    double accepted_radius = 0.0;

    for (int attempt = 0;
         attempt <= kBezierCollisionRetries &&
         turn_radius + 1e-9 >= minimum_turn_radius;
         ++attempt, turn_radius *= 0.75) {
      const double tangent_distance = turn_radius * tangent_scale;
      if (tangent_distance + 1e-9 < corner_minimum_distance ||
          tangent_distance > maximum_tangent_distance + 1e-9) {
        continue;
      }
      geometry_msgs::msg::Point entry;
      entry.x = corner.x - incoming_unit_x * tangent_distance;
      entry.y = corner.y - incoming_unit_y * tangent_distance;
      entry.z = corner.z -
                (corner.z - previous.z) *
                    (tangent_distance / incoming_length);
      geometry_msgs::msg::Point exit;
      exit.x = corner.x + outgoing_unit_x * tangent_distance;
      exit.y = corner.y + outgoing_unit_y * tangent_distance;
      exit.z = corner.z +
               (next.z - corner.z) *
                   (tangent_distance / outgoing_length);

      // k = 4/3 * tan(theta/4) 是用三次贝塞尔近似圆弧的标准控制距离。
      // 两端切线与A*前后直线一致，避免二阶贝塞尔在短圆角中产生过大的曲率峰值。
      const double control_distance =
          (4.0 / 3.0) * turn_radius * std::tan(turn_angle * 0.25);
      geometry_msgs::msg::Point control_1 = entry;
      control_1.x += incoming_unit_x * control_distance;
      control_1.y += incoming_unit_y * control_distance;
      geometry_msgs::msg::Point control_2 = exit;
      control_2.x -= outgoing_unit_x * control_distance;
      control_2.y -= outgoing_unit_y * control_distance;
      const double control_ratio = std::clamp(
          control_distance / std::max(tangent_distance, 1e-9), 0.0, 1.0);
      control_1.z += (corner.z - entry.z) * control_ratio;
      control_2.z -= (exit.z - corner.z) * control_ratio;

      nav_msgs::msg::Path corner_path;
      corner_path.header = raw_path.header;
      const double control_polygon_length =
          std::hypot(control_1.x - entry.x, control_1.y - entry.y) +
          std::hypot(control_2.x - control_1.x,
                     control_2.y - control_1.y) +
          std::hypot(exit.x - control_2.x, exit.y - control_2.y);
      const int sample_count = std::max(
          4, static_cast<int>(std::ceil(
                 control_polygon_length / (sample_spacing * 0.95))));
      const double curve_minimum_radius = cubicBezierMinimumRadius(
          entry, control_1, control_2, exit, sample_count);
      // 圆弧的三次贝塞尔是近似曲线，容许2%数值误差。
      if (minimum_turn_radius > 0.0 &&
          curve_minimum_radius + 1e-9 < minimum_turn_radius * 0.98) {
        continue;
      }
      bool deviation_valid = true;
      for (int sample = 0; sample <= sample_count; ++sample) {
        const double t = static_cast<double>(sample) / sample_count;
        geometry_msgs::msg::PoseStamped pose = clean_path.poses[i];
        pose.pose.position = cubicBezierPoint(
            entry, control_1, control_2, exit, t);
        const double deviation = std::min(
            pointToSegmentDistance(pose.pose.position, entry, corner),
            pointToSegmentDistance(pose.pose.position, corner, exit));
        const double raw_path_deviation =
            pointToPathDistance(pose.pose.position, duplicate_free_path);
        if (deviation > maximum_deviation + 1e-9 ||
            raw_path_deviation > maximum_deviation + 1e-9) {
          deviation_valid = false;
          break;
        }
        appendDistinctPose(corner_path, std::move(pose));
      }

      if (!deviation_valid || corner_path.poses.size() < 2) continue;
      updatePathOrientations(corner_path,
                             clean_path.poses[i].pose.orientation);
      if (!isPathCollisionFree(corner_path)) continue;

      accepted_corner = std::move(corner_path);
      accepted_radius = curve_minimum_radius;
      corner_accepted = true;
      break;
    }

    if (corner_accepted) {
      for (auto &pose : accepted_corner.poses) {
        appendDistinctPose(candidate, std::move(pose));
      }
      ++accepted_corners;
      achieved_minimum_radius = std::min(
          achieved_minimum_radius, accepted_radius);
    } else {
      appendDistinctPose(candidate, clean_path.poses[i]);
      ++fallback_corners;
    }
  }

  appendDistinctPose(candidate, clean_path.poses.back());
  candidate = densifyPath(candidate, sample_spacing);
  updatePathOrientations(candidate,
                         raw_path.poses.back().pose.orientation);

  if ((!simplification_applied && accepted_corners == 0) ||
      candidate.poses.size() < 2 ||
      !isPathCollisionFree(candidate)) {
    RCLCPP_WARN(node_->get_logger(),
                "[BEZIER_FALLBACK] raw=%zu direction_vertices=%zu "
                "simplified_vertices=%zu candidate=%zu corners=%d "
                "accepted=%d fallback=%d; using raw A*",
                raw_path.poses.size(), direction_vertex_count,
                clean_path.poses.size(), candidate.poses.size(),
                considered_corners, accepted_corners, fallback_corners);
    smoothed_path = raw_path;
    publishSmoothingPaths(raw_path, smoothed_path);
    return false;
  }

  smoothed_path = std::move(candidate);
  publishSmoothingPaths(raw_path, smoothed_path);
  RCLCPP_INFO(node_->get_logger(),
              "[BEZIER_SMOOTH] raw=%zu direction_vertices=%zu "
              "simplified_vertices=%zu smooth=%zu corners=%d accepted=%d "
              "fallback=%d min_radius=%.3f target_radius=%.3f",
              raw_path.poses.size(), direction_vertex_count,
              clean_path.poses.size(), smoothed_path.poses.size(),
              considered_corners, accepted_corners, fallback_corners,
              achieved_minimum_radius, target_turn_radius);
  return true;
}

void AvoidancePathPlanner::publishSmoothingPaths(
    const nav_msgs::msg::Path &raw_path,
    const nav_msgs::msg::Path &smoothed_path) {
  if (!config_.bezier_visualization_enabled || !raw_path_pub_ ||
      !smoothed_path_pub_) {
    return;
  }
  nav_msgs::msg::Path raw_visualization = raw_path;
  nav_msgs::msg::Path smoothed_visualization = smoothed_path;
  const auto stamp = node_->now();
  raw_visualization.header.stamp = stamp;
  smoothed_visualization.header.stamp = stamp;
  raw_path_pub_->publish(raw_visualization);
  smoothed_path_pub_->publish(smoothed_visualization);
}

bool AvoidancePathPlanner::isPointInObstacle(
    const geometry_msgs::msg::Point &point) {
  return !astar_planner_ || astar_planner_->isObstacle(point);
}

bool AvoidancePathPlanner::isPointInObstacleWithDiagnostics(
    const geometry_msgs::msg::Point &point, const char *point_name,
    double yaw) {
  return !astar_planner_ ||
         astar_planner_->isObstacleWithDiagnostics(point, point_name, yaw);
}

bool AvoidancePathPlanner::isPathCollisionFree(
    const nav_msgs::msg::Path &path) const {
  return astar_planner_ && astar_planner_->isPathCollisionFree(path);
}

bool AvoidancePathPlanner::isRotationCollisionFree(
    const geometry_msgs::msg::Point &center, double start_yaw,
    double end_yaw) const {
  return astar_planner_ &&
         astar_planner_->isRotationCollisionFree(center, start_yaw, end_yaw);
}

bool AvoidancePathPlanner::isInitialEscapePathCollisionFree(
    const nav_msgs::msg::Path &path,
    const geometry_msgs::msg::Point &obstacle_center,
    double required_clearance) const {
  if (!astar_planner_ || path.poses.empty()) return false;
  // 几何层先保证路径不会继续靠近触发障碍且最终退出净空圈；栅格层再
  // 只放宽起点附近连续的footprint冲突前缀。二者同时满足才可继续执行。
  return isPathOutsideObstacleRadius(path, obstacle_center,
                                     required_clearance, nullptr, true) &&
         astar_planner_->isPathCollisionFree(path, true);
}

bool AvoidancePathPlanner::isPathOutsideObstacleRadius(
    const nav_msgs::msg::Path &path,
    const geometry_msgs::msg::Point &obstacle_center,
    double required_clearance, double *minimum_clearance,
    bool allow_initial_escape) const {
  double minimum = std::numeric_limits<double>::max();
  if (path.poses.empty()) {
    if (minimum_clearance) *minimum_clearance = minimum;
    return false;
  }

  const double start_clearance = calculateDistance(
      path.poses.front().pose.position, obstacle_center);
  minimum = start_clearance;
  const bool starts_inside = start_clearance < required_clearance;
  const bool escaping_from_inside = allow_initial_escape && starts_inside;
  // 允许一个Costmap栅格量级的离散误差，但不能借“脱离”名义继续明显
  // 靠近障碍。退出净空圈后的所有路径段仍执行严格的0.80m校验。
  const double escape_tolerance =
      std::min(0.05, std::max(0.0, required_clearance) * 0.10);
  const double escape_floor =
      std::max(0.0, start_clearance - escape_tolerance);
  bool escaped = !starts_inside;

  for (std::size_t i = 1; i < path.poses.size(); ++i) {
    const auto &a = path.poses[i - 1].pose.position;
    const auto &b = path.poses[i].pose.position;
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double length_squared = dx * dx + dy * dy;
    double segment_minimum = calculateDistance(b, obstacle_center);
    if (length_squared <= 1e-12) {
      segment_minimum = std::min(
          segment_minimum, calculateDistance(a, obstacle_center));
    } else {
      const double projection = std::clamp(
          ((obstacle_center.x - a.x) * dx +
           (obstacle_center.y - a.y) * dy) /
              length_squared,
          0.0, 1.0);
      geometry_msgs::msg::Point closest;
      closest.x = a.x + projection * dx;
      closest.y = a.y + projection * dy;
      segment_minimum = calculateDistance(closest, obstacle_center);
    }
    minimum = std::min(minimum, segment_minimum);

    if (!escaping_from_inside) continue;
    if (!escaped) {
      if (segment_minimum < escape_floor) {
        if (minimum_clearance) *minimum_clearance = minimum;
        return false;
      }
      if (calculateDistance(b, obstacle_center) >= required_clearance) {
        escaped = true;
      }
    } else if (segment_minimum < required_clearance) {
      if (minimum_clearance) *minimum_clearance = minimum;
      return false;
    }
  }

  if (minimum_clearance) *minimum_clearance = minimum;
  if (!std::isfinite(minimum)) return false;
  if (escaping_from_inside) return escaped;
  return minimum >= required_clearance;
}

void AvoidancePathPlanner::publishVisualization(
    const geometry_msgs::msg::Point &start,
    const geometry_msgs::msg::Point &goal, const nav_msgs::msg::Path &path) {
  if (astar_planner_)
    astar_planner_->publishVisualization(start, goal, path);
}

bool AvoidancePathPlanner::getPathYawAtIndex(const nav_msgs::msg::Path &path,
                                             int index, double &yaw) const {
  if (path.poses.size() < 2 || index < 0 ||
      index >= static_cast<int>(path.poses.size())) {
    return false;
  }
  const int prev_index = std::max(0, index - 1);
  const int next_index =
      std::min(static_cast<int>(path.poses.size()) - 1, index + 1);
  if (prev_index == next_index)
    return false;

  const auto &p0 = path.poses[prev_index].pose.position;
  const auto &p1 = path.poses[next_index].pose.position;
  const double dx = p1.x - p0.x;
  const double dy = p1.y - p0.y;
  if (std::hypot(dx, dy) < 1e-6)
    return false;
  yaw = std::atan2(dy, dx);
  return true;
}

bool AvoidancePathPlanner::isPointSafeForRobot(
    const geometry_msgs::msg::Point &point, double clearance_radius) const {
  const CostmapConstPtr costmap =
      astar_planner_ ? astar_planner_->getPreparedCostmap() : nullptr;
  if (!costmap || costmap->info.resolution <= 0.0 || costmap->data.empty()) {
    return false;
  }

  const double resolution = costmap->info.resolution;
  const int center_x = static_cast<int>(
      (point.x - costmap->info.origin.position.x) / resolution);
  const int center_y = static_cast<int>(
      (point.y - costmap->info.origin.position.y) / resolution);
  const int width = static_cast<int>(costmap->info.width);
  const int height = static_cast<int>(costmap->info.height);
  if (center_x < 0 || center_y < 0 || center_x >= width || center_y >= height) {
    return false;
  }

  const int radius_cells =
      std::max(1, static_cast<int>(std::ceil(clearance_radius / resolution)));
  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      const double cell_distance =
          std::hypot(static_cast<double>(dx), static_cast<double>(dy)) *
          resolution;
      if (cell_distance > clearance_radius)
        continue;

      const int grid_x = center_x + dx;
      const int grid_y = center_y + dy;
      if (grid_x < 0 || grid_y < 0 || grid_x >= width || grid_y >= height) {
        return false;
      }
      const int index = grid_y * width + grid_x;
      if (index < 0 || index >= static_cast<int>(costmap->data.size())) {
        return false;
      }
      const int cost = costmap->data[index];
      if (cost < 0 || cost >= config_.obstacle_threshold)
        return false;
    }
  }
  return true;
}

double
AvoidancePathPlanner::calculateDistance(const geometry_msgs::msg::Point &p1,
                                        const geometry_msgs::msg::Point &p2) {
  return std::hypot(p1.x - p2.x, p1.y - p2.y);
}

} // namespace pure_pursuit_local_planner
