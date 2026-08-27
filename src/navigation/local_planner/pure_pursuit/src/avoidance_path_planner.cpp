#include "pure_pursuit/avoidance_path_planner.hpp"

#include <algorithm>
#include <cmath>

#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace pure_pursuit_local_planner {

bool AvoidancePathPlanner::initialize(
    rclcpp::Node::SharedPtr node, const AvoidancePathPlannerConfig &config) {
  node_ = std::move(node);
  config_ = config;
  astar_planner_ = std::make_unique<AStarPlanner>();
  return astar_planner_->initialize(node_);
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
  return astar_planner_ && astar_planner_->planPath(start, goal, path, goal_yaw);
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
