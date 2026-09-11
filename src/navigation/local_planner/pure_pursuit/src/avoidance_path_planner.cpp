#include "pure_pursuit/avoidance_path_planner.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <utility>
#include <vector>

#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace pure_pursuit_local_planner {
namespace {

constexpr double kPointEpsilon = 1e-4;
constexpr double kMinimumCornerDistance = 0.03;
constexpr double kMinimumTurnAngle = 8.0 * M_PI / 180.0;
constexpr int kBezierCollisionRetries = 6;

struct TemporaryGoalPathSample {
  geometry_msgs::msg::Point point;
  double yaw{0.0};
  double incoming_yaw{0.0};
  double arc_length{0.0};
  std::size_t following_path_index{0};
  bool is_original_pose{false};
  bool requires_rotation_check{false};
};

struct TemporaryGoalSearchProjection {
  geometry_msgs::msg::Point point;
  std::size_t segment_index{0};
  double segment_ratio{0.0};
  double yaw{0.0};
};

bool pathTangentAtIndex(const nav_msgs::msg::Path &path,
                        std::size_t index, double &yaw) {
  if (path.poses.size() < 2 || index >= path.poses.size()) return false;

  const auto &current = path.poses[index].pose.position;
  for (std::size_t next = index + 1; next < path.poses.size(); ++next) {
    const auto &point = path.poses[next].pose.position;
    const double dx = point.x - current.x;
    const double dy = point.y - current.y;
    if (std::hypot(dx, dy) >= kPointEpsilon) {
      yaw = std::atan2(dy, dx);
      return true;
    }
  }

  for (std::size_t previous = index; previous > 0; --previous) {
    const auto &point = path.poses[previous - 1].pose.position;
    const double dx = current.x - point.x;
    const double dy = current.y - point.y;
    if (std::hypot(dx, dy) >= kPointEpsilon) {
      yaw = std::atan2(dy, dx);
      return true;
    }
  }
  return false;
}

bool findTemporaryGoalSearchProjection(
    const nav_msgs::msg::Path &path,
    const geometry_msgs::msg::Point &robot_position,
    std::size_t progress_index, TemporaryGoalSearchProjection &projection) {
  if (path.poses.size() < 2 || progress_index >= path.poses.size()) {
    return false;
  }

  const std::size_t first_segment = progress_index > 0
                                        ? progress_index - 1
                                        : progress_index;
  const std::size_t last_segment = std::min(
      path.poses.size() - 2, progress_index + 1);
  double minimum_distance_squared = std::numeric_limits<double>::infinity();
  bool found = false;
  for (std::size_t segment_index = first_segment;
       segment_index <= last_segment; ++segment_index) {
    const auto &start = path.poses[segment_index].pose.position;
    const auto &end = path.poses[segment_index + 1].pose.position;
    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const double dz = end.z - start.z;
    const double length_squared = dx * dx + dy * dy;
    if (!std::isfinite(length_squared) ||
        length_squared < kPointEpsilon * kPointEpsilon) {
      continue;
    }

    const double ratio = std::clamp(
        ((robot_position.x - start.x) * dx +
         (robot_position.y - start.y) * dy) /
            length_squared,
        0.0, 1.0);
    geometry_msgs::msg::Point candidate;
    candidate.x = start.x + ratio * dx;
    candidate.y = start.y + ratio * dy;
    candidate.z = start.z + ratio * dz;
    const double distance_squared =
        (candidate.x - robot_position.x) *
            (candidate.x - robot_position.x) +
        (candidate.y - robot_position.y) *
            (candidate.y - robot_position.y);
    if (distance_squared < minimum_distance_squared) {
      minimum_distance_squared = distance_squared;
      projection.point = candidate;
      projection.segment_index = segment_index;
      projection.segment_ratio = ratio;
      projection.yaw = std::atan2(dy, dx);
      found = true;
    }
  }
  return found;
}

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

double pathLength2d(const nav_msgs::msg::Path &path) {
  double length = 0.0;
  for (std::size_t index = 1U; index < path.poses.size(); ++index) {
    const auto &previous = path.poses[index - 1U].pose.position;
    const auto &current = path.poses[index].pose.position;
    length += std::hypot(current.x - previous.x, current.y - previous.y);
  }
  return length;
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

// B样条使用均匀节点向量，因此控制点的序号会直接影响曲线形状。A*经
// RDP简化后各线段长度通常差异很大，若直接作为控制点，短线段会被长线段
// 过度拉扯。这里按XY弧长重新均匀采样，同时保留精确首尾点。
nav_msgs::msg::Path resamplePathByArcLength(const nav_msgs::msg::Path &path,
                                            double target_spacing) {
  if (path.poses.size() < 2U || target_spacing <= kPointEpsilon) return path;

  std::vector<double> cumulative(path.poses.size(), 0.0);
  for (std::size_t index = 1U; index < path.poses.size(); ++index) {
    cumulative[index] = cumulative[index - 1U] + std::hypot(
        path.poses[index].pose.position.x -
            path.poses[index - 1U].pose.position.x,
        path.poses[index].pose.position.y -
            path.poses[index - 1U].pose.position.y);
  }
  const double total_length = cumulative.back();
  if (total_length <= kPointEpsilon) return path;

  // 至少保留4个控制点，使三次B样条在短路径上也能正常工作；长路径按
  // 配置间距均匀采样。实际间距使用total/segment_count，避免末段过短。
  const int segment_count = std::max(
      3, static_cast<int>(std::ceil(total_length / target_spacing)));
  nav_msgs::msg::Path result;
  result.header = path.header;
  result.poses.reserve(static_cast<std::size_t>(segment_count) + 1U);
  std::size_t source_segment = 0U;
  for (int sample = 0; sample <= segment_count; ++sample) {
    const double arc = total_length * static_cast<double>(sample) /
                       static_cast<double>(segment_count);
    while (source_segment + 1U < cumulative.size() - 1U &&
           cumulative[source_segment + 1U] < arc) {
      ++source_segment;
    }
    const double segment_length =
        cumulative[source_segment + 1U] - cumulative[source_segment];
    const double ratio = segment_length > kPointEpsilon
                             ? std::clamp((arc - cumulative[source_segment]) /
                                              segment_length,
                                          0.0, 1.0)
                             : 0.0;
    geometry_msgs::msg::PoseStamped pose = path.poses[source_segment];
    const auto &start = path.poses[source_segment].pose.position;
    const auto &end = path.poses[source_segment + 1U].pose.position;
    pose.pose.position.x = start.x + ratio * (end.x - start.x);
    pose.pose.position.y = start.y + ratio * (end.y - start.y);
    pose.pose.position.z = start.z + ratio * (end.z - start.z);
    appendDistinctPose(result, std::move(pose));
  }
  if (!result.poses.empty()) {
    result.poses.front().pose.position = path.poses.front().pose.position;
    result.poses.back().pose.position = path.poses.back().pose.position;
  }
  return result;
}

nav_msgs::msg::Path simplifyPathRdp(const nav_msgs::msg::Path &path,
                                    double tolerance,
                                    double protected_start_distance = 0.0) {
  if (path.poses.size() < 3U || tolerance <= 0.0) return path;

  std::vector<bool> keep(path.poses.size(), false);
  keep.front() = true;
  keep.back() = true;
  std::vector<std::pair<std::size_t, std::size_t>> ranges;
  std::size_t protected_index = 0U;
  double accumulated_distance = 0.0;
  for (std::size_t index = 1U; index < path.poses.size(); ++index) {
    const auto &previous = path.poses[index - 1U].pose.position;
    const auto &current = path.poses[index].pose.position;
    accumulated_distance +=
        std::hypot(current.x - previous.x, current.y - previous.y);
    protected_index = index;
    if (accumulated_distance + 1e-9 >= protected_start_distance) break;
  }
  if (protected_start_distance > 0.0 && protected_index > 0U &&
      protected_index + 1U < path.poses.size()) {
    // 不允许RDP用“起点到远端”的可视直线跨过第一个绕障趋势锚点。
    keep[protected_index] = true;
    ranges.emplace_back(0U, protected_index);
    ranges.emplace_back(protected_index, path.poses.size() - 1U);
  } else {
    ranges.emplace_back(0U, path.poses.size() - 1U);
  }
  while (!ranges.empty()) {
    const auto range = ranges.back();
    ranges.pop_back();
    double maximum_distance = 0.0;
    std::size_t split = range.first;
    for (std::size_t index = range.first + 1U; index < range.second; ++index) {
      const double distance = pointToSegmentDistance(
          path.poses[index].pose.position,
          path.poses[range.first].pose.position,
          path.poses[range.second].pose.position);
      if (distance > maximum_distance) {
        maximum_distance = distance;
        split = index;
      }
    }
    if (maximum_distance > tolerance && split > range.first &&
        split < range.second) {
      keep[split] = true;
      ranges.emplace_back(range.first, split);
      ranges.emplace_back(split, range.second);
    }
  }

  nav_msgs::msg::Path simplified;
  simplified.header = path.header;
  for (std::size_t index = 0; index < path.poses.size(); ++index) {
    if (keep[index]) appendDistinctPose(simplified, path.poses[index]);
  }
  return simplified;
}

geometry_msgs::msg::Point evaluateClampedBSpline(
    const std::vector<geometry_msgs::msg::Point> &control_points,
    int requested_degree, double parameter) {
  if (control_points.empty()) return geometry_msgs::msg::Point();
  if (control_points.size() == 1U) return control_points.front();

  const int last_control = static_cast<int>(control_points.size()) - 1;
  const int degree = std::clamp(requested_degree, 1, last_control);
  const int knot_count = last_control + degree + 2;
  std::vector<double> knots(static_cast<std::size_t>(knot_count), 0.0);
  const int internal_count = last_control - degree;
  for (int index = degree + 1; index <= last_control; ++index) {
    knots[static_cast<std::size_t>(index)] =
        static_cast<double>(index - degree) /
        static_cast<double>(internal_count + 1);
  }
  for (int index = last_control + 1; index < knot_count; ++index) {
    knots[static_cast<std::size_t>(index)] = 1.0;
  }

  const double u = std::clamp(parameter, 0.0, 1.0);
  if (u >= 1.0) return control_points.back();
  int span = degree;
  while (span < last_control &&
         u >= knots[static_cast<std::size_t>(span + 1)]) {
    ++span;
  }

  std::vector<geometry_msgs::msg::Point> work(
      static_cast<std::size_t>(degree + 1));
  for (int index = 0; index <= degree; ++index) {
    work[static_cast<std::size_t>(index)] =
        control_points[static_cast<std::size_t>(span - degree + index)];
  }
  for (int level = 1; level <= degree; ++level) {
    for (int index = degree; index >= level; --index) {
      const int control_index = span - degree + index;
      const double denominator =
          knots[static_cast<std::size_t>(control_index + degree - level + 1)] -
          knots[static_cast<std::size_t>(control_index)];
      const double alpha = denominator > 1e-12
                               ? (u - knots[static_cast<std::size_t>(control_index)]) /
                                     denominator
                               : 0.0;
      auto &current = work[static_cast<std::size_t>(index)];
      const auto &previous = work[static_cast<std::size_t>(index - 1)];
      current.x = (1.0 - alpha) * previous.x + alpha * current.x;
      current.y = (1.0 - alpha) * previous.y + alpha * current.y;
      current.z = (1.0 - alpha) * previous.z + alpha * current.z;
    }
  }
  return work[static_cast<std::size_t>(degree)];
}

double minimumDiscretePathRadius(const nav_msgs::msg::Path &path) {
  double minimum_radius = std::numeric_limits<double>::infinity();
  for (std::size_t index = 1U; index + 1U < path.poses.size(); ++index) {
    const auto &a = path.poses[index - 1U].pose.position;
    const auto &b = path.poses[index].pose.position;
    const auto &c = path.poses[index + 1U].pose.position;
    const double ab = std::hypot(b.x - a.x, b.y - a.y);
    const double bc = std::hypot(c.x - b.x, c.y - b.y);
    const double ac = std::hypot(c.x - a.x, c.y - a.y);
    const double twice_area =
        std::abs((b.x - a.x) * (c.y - a.y) -
                 (b.y - a.y) * (c.x - a.x));
    if (ab < kPointEpsilon || bc < kPointEpsilon || ac < kPointEpsilon ||
        twice_area < 1e-9) {
      continue;
    }
    minimum_radius =
        std::min(minimum_radius, ab * bc * ac / (2.0 * twice_area));
  }
  return minimum_radius;
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
    bspline_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
        "local_astar_bspline_path", 1);
    pruned_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
        "local_astar_pruned_path", 1);
    selected_smoothed_path_pub_ =
        node_->create_publisher<nav_msgs::msg::Path>(
            "local_astar_smoothed_path", 1);
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

std::uint64_t AvoidancePathPlanner::preparedCostmapVersion() const {
  return astar_planner_ ? astar_planner_->getPreparedCostmapVersion() : 0U;
}

TemporaryGoalSearchResult AvoidancePathPlanner::findTemporaryGoal(
    const nav_msgs::msg::Path &global_path,
    const geometry_msgs::msg::Point &robot_position,
    std::size_t start_index, std::size_t minimum_goal_index,
    double search_distance_override) {
  TemporaryGoalSearchResult result;
  result.search_start_index = start_index;
  const double search_distance =
      std::isfinite(search_distance_override) &&
              search_distance_override > 0.0
          ? std::max(config_.temporary_goal_search_distance,
                     search_distance_override)
          : config_.temporary_goal_search_distance;

  if (global_path.poses.size() < 2 ||
      start_index >= global_path.poses.size() ||
      !std::isfinite(robot_position.x) ||
      !std::isfinite(robot_position.y) ||
      !std::isfinite(robot_position.z) ||
      !std::isfinite(search_distance) ||
      !std::isfinite(config_.temporary_goal_clearance) ||
      !std::isfinite(config_.temporary_goal_safe_path_length) ||
      !std::isfinite(config_.min_temporary_goal_distance_from_obstacle) ||
      !std::isfinite(config_.temporary_goal_sample_spacing) ||
      search_distance <= 0.0 ||
      config_.temporary_goal_safe_path_length <= 0.0 ||
      config_.temporary_goal_sample_spacing <= 0.0 ||
      config_.temporary_goal_clearance < 0.0 ||
      config_.min_temporary_goal_distance_from_obstacle < 0.0) {
    result.status = TemporaryGoalSearchStatus::INVALID_INPUT;
    return result;
  }

  for (const auto &pose : global_path.poses) {
    const auto &point = pose.pose.position;
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      result.status = TemporaryGoalSearchStatus::INVALID_INPUT;
      return result;
    }
  }

  const CostmapConstPtr costmap =
      astar_planner_ ? astar_planner_->getPreparedCostmap() : nullptr;
  if (!costmap || !std::isfinite(costmap->info.resolution) ||
      costmap->info.resolution <= 0.0 ||
      costmap->info.width == 0 || costmap->info.height == 0 ||
      costmap->info.width >
          static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
      costmap->info.height >
          static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
      !std::isfinite(costmap->info.origin.position.x) ||
      !std::isfinite(costmap->info.origin.position.y) ||
      costmap->data.size() != static_cast<std::size_t>(costmap->info.width) *
                                  static_cast<std::size_t>(costmap->info.height)) {
    result.status = TemporaryGoalSearchStatus::NO_COSTMAP;
    return result;
  }
  if (!global_path.header.frame_id.empty() &&
      !costmap->header.frame_id.empty() &&
      global_path.header.frame_id != costmap->header.frame_id) {
    result.status = TemporaryGoalSearchStatus::INVALID_INPUT;
    return result;
  }

  // 半栅格采样与AStarPlanner::isPathCollisionFree()保持一致；配置值只能
  // 进一步加密，不能让稀疏路径跨过一个障碍栅格。
  const double sample_spacing = std::max(
      0.001, std::min(config_.temporary_goal_sample_spacing,
                      costmap->info.resolution * 0.5));
  const double validation_scan_distance =
      search_distance +
      config_.temporary_goal_safe_path_length;
  constexpr std::size_t kMaximumTemporaryGoalSamples = 20000U;
  const double requested_sample_count =
      std::ceil(validation_scan_distance / sample_spacing) + 2.0;
  if (!std::isfinite(validation_scan_distance) ||
      !std::isfinite(requested_sample_count) ||
      requested_sample_count >
          static_cast<double>(kMaximumTemporaryGoalSamples)) {
    result.status = TemporaryGoalSearchStatus::INVALID_INPUT;
    return result;
  }
  std::vector<TemporaryGoalPathSample> samples;
  samples.reserve(static_cast<std::size_t>(requested_sample_count));

  TemporaryGoalSearchProjection projection;
  if (!findTemporaryGoalSearchProjection(global_path, robot_position, start_index,
                                    projection)) {
    result.status = TemporaryGoalSearchStatus::INVALID_INPUT;
    return result;
  }
  result.search_start_segment_index = projection.segment_index;

  TemporaryGoalPathSample initial_sample;
  initial_sample.point = projection.point;
  initial_sample.yaw = projection.yaw;
  initial_sample.incoming_yaw = projection.yaw;
  initial_sample.following_path_index = projection.segment_index + 1;
  initial_sample.is_original_pose = projection.segment_ratio >= 1.0 - 1e-9;
  if (initial_sample.is_original_pose) {
    double outgoing_yaw = projection.yaw;
    if (pathTangentAtIndex(global_path, projection.segment_index + 1,
                           outgoing_yaw)) {
      initial_sample.yaw = outgoing_yaw;
      initial_sample.requires_rotation_check =
          std::abs(std::atan2(std::sin(outgoing_yaw - projection.yaw),
                             std::cos(outgoing_yaw - projection.yaw))) >
          1e-6;
    }
  }
  samples.push_back(initial_sample);

  double accumulated_length = 0.0;
  bool validation_limit_reached = false;
  for (std::size_t segment_index = projection.segment_index;
       segment_index + 1 < global_path.poses.size() &&
       !validation_limit_reached;
       ++segment_index) {
    const geometry_msgs::msg::Point segment_start =
        segment_index == projection.segment_index
            ? projection.point
            : global_path.poses[segment_index].pose.position;
    const auto &segment_end =
        global_path.poses[segment_index + 1].pose.position;
    const double dx = segment_end.x - segment_start.x;
    const double dy = segment_end.y - segment_start.y;
    const double dz = segment_end.z - segment_start.z;
    const double segment_length = std::hypot(dx, dy);
    if (!std::isfinite(segment_length)) {
      result.status = TemporaryGoalSearchStatus::INVALID_INPUT;
      return result;
    }
    if (segment_length < kPointEpsilon) continue;

    const double segment_yaw = std::atan2(dy, dx);
    const double remaining_validation_distance =
        validation_scan_distance - accumulated_length;
    if (remaining_validation_distance <= 1e-9) break;
    const double traversed_segment_length =
        std::min(segment_length, remaining_validation_distance);
    const int sample_count = std::max(
        1, static_cast<int>(
               std::ceil(traversed_segment_length / sample_spacing)));
    for (int sample = 1; sample <= sample_count; ++sample) {
      const double ratio = static_cast<double>(sample) / sample_count;
      const double sample_arc_length =
          accumulated_length + ratio * traversed_segment_length;
      const double segment_ratio =
          ratio * traversed_segment_length / segment_length;

      TemporaryGoalPathSample path_sample;
      path_sample.point.x = segment_start.x + segment_ratio * dx;
      path_sample.point.y = segment_start.y + segment_ratio * dy;
      path_sample.point.z = segment_start.z + segment_ratio * dz;
      path_sample.yaw = segment_yaw;
      path_sample.incoming_yaw = segment_yaw;
      path_sample.arc_length = sample_arc_length;
      path_sample.following_path_index = segment_index + 1;
      path_sample.is_original_pose =
          sample == sample_count &&
          traversed_segment_length >= segment_length - 1e-9;
      if (path_sample.is_original_pose) {
        double original_pose_yaw = segment_yaw;
        if (pathTangentAtIndex(global_path, segment_index + 1,
                               original_pose_yaw)) {
          path_sample.yaw = original_pose_yaw;
          path_sample.requires_rotation_check =
              std::abs(std::atan2(
                  std::sin(original_pose_yaw - segment_yaw),
                  std::cos(original_pose_yaw - segment_yaw))) > 1e-6;
        }
      }
      if (samples.size() >= kMaximumTemporaryGoalSamples) {
        result.status = TemporaryGoalSearchStatus::INVALID_INPUT;
        return result;
      }
      samples.push_back(std::move(path_sample));
    }
    accumulated_length += traversed_segment_length;
    validation_limit_reached =
        traversed_segment_length < segment_length - 1e-9;
  }

  if (samples.empty()) {
    result.status = TemporaryGoalSearchStatus::INVALID_INPUT;
    return result;
  }
  result.scanned_path_length = samples.back().arc_length;

  std::vector<uint8_t> footprint_free(samples.size(), 0U);
  std::vector<uint8_t> temporary_goal_safe(samples.size(), 0U);
  for (std::size_t index = 0; index < samples.size(); ++index) {
    bool footprint_is_free = !astar_planner_->isObstacle(
        samples[index].point, samples[index].yaw);
    if (footprint_is_free && samples[index].requires_rotation_check) {
      footprint_is_free = astar_planner_->isRotationCollisionFree(
          samples[index].point, samples[index].incoming_yaw,
          samples[index].yaw);
    }
    footprint_free[index] = footprint_is_free ? 1U : 0U;
    temporary_goal_safe[index] =
        footprint_is_free &&
                isPointSafeForTemporaryGoal(samples[index].point,
                                       config_.temporary_goal_clearance, costmap)
            ? 1U
            : 0U;
  }

  std::size_t first_blocked_sample = samples.size();
  for (std::size_t index = 0; index < samples.size(); ++index) {
    if (samples[index].arc_length >
        search_distance + 1e-9) {
      break;
    }
    // “路径被阻断”与后续临时点搜索使用同一套临时点安全语义。
    // 即使障碍尚未直接压住 footprint，只要已经侵入配置的临时点净空，
    // 也必须启动绕障搜索，不能误报 NO_BLOCKED_PATH 后长期停车等待。
    if (temporary_goal_safe[index] == 0U) {
      first_blocked_sample = index;
      result.blocked_start_index = samples[index].following_path_index;
      result.blocked_start_path_distance = samples[index].arc_length;
      break;
    }
  }
  if (first_blocked_sample == samples.size()) {
    result.status = TemporaryGoalSearchStatus::NO_BLOCKED_PATH;
    return result;
  }

  double last_unsafe_arc_length = samples[first_blocked_sample].arc_length;
  result.blocked_end_index =
      samples[first_blocked_sample].following_path_index;
  result.blocked_end_path_distance = last_unsafe_arc_length;

  for (std::size_t index = first_blocked_sample; index < samples.size();
       ++index) {
    if (samples[index].arc_length >
        search_distance + 1e-9) {
      break;
    }
    if (temporary_goal_safe[index] == 0U) {
      last_unsafe_arc_length = samples[index].arc_length;
      result.blocked_end_index = samples[index].following_path_index;
      result.blocked_end_path_distance = last_unsafe_arc_length;
      continue;
    }
    const double distance_from_last_unsafe =
        samples[index].arc_length - last_unsafe_arc_length;
    if (distance_from_last_unsafe + 1e-9 <
        config_.min_temporary_goal_distance_from_obstacle) {
      continue;
    }

    nav_msgs::msg::Path verified_path;
    verified_path.header = global_path.header;
    double verified_safe_length = 0.0;
    for (std::size_t safe_index = index; safe_index < samples.size();
         ++safe_index) {
      if (temporary_goal_safe[safe_index] == 0U) break;

      geometry_msgs::msg::PoseStamped pose;
      pose.header = global_path.header;
      pose.pose.position = samples[safe_index].point;
      tf2::Quaternion orientation;
      orientation.setRPY(0.0, 0.0, samples[safe_index].yaw);
      pose.pose.orientation = tf2::toMsg(orientation);
      verified_path.poses.push_back(std::move(pose));
      verified_safe_length =
          samples[safe_index].arc_length - samples[index].arc_length;
      if (verified_safe_length + 1e-9 >=
          config_.temporary_goal_safe_path_length) {
        break;
      }
    }

    if (verified_safe_length + 1e-9 <
            config_.temporary_goal_safe_path_length ||
        !isPathMotionCollisionFree(verified_path)) {
      continue;
    }
    // A*已确认不可达的临时点之后继续选候选，但障碍扫描仍从原始
    // start_index开始，避免把障碍段本身一起跳过。
    if (samples[index].following_path_index < minimum_goal_index) {
      continue;
    }

    result.status = TemporaryGoalSearchStatus::FOUND;
    result.temporary_goal_index = samples[index].following_path_index;
    result.goal.header = global_path.header;
    result.goal.pose.position = samples[index].point;
    result.goal_yaw = samples[index].yaw;
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, result.goal_yaw);
    result.goal.pose.orientation = tf2::toMsg(orientation);
    result.temporary_goal_path_distance = samples[index].arc_length;
    result.distance_from_last_unsafe = distance_from_last_unsafe;
    result.verified_safe_length = verified_safe_length;
    return result;
  }

  result.status = TemporaryGoalSearchStatus::NO_SAFE_TEMPORARY_GOAL;
  return result;
}

bool AvoidancePathPlanner::isTemporaryGoalValid(
    const nav_msgs::msg::Path &global_path,
    std::size_t temporary_goal_index) {
  if (temporary_goal_index >= global_path.poses.size()) return false;
  geometry_msgs::msg::PoseStamped temporary_goal =
      global_path.poses[temporary_goal_index];
  double goal_yaw = 0.0;
  if (!pathTangentAtIndex(global_path, temporary_goal_index, goal_yaw)) {
    return false;
  }
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, goal_yaw);
  temporary_goal.pose.orientation = tf2::toMsg(orientation);
  return isTemporaryGoalValid(global_path, temporary_goal,
                              temporary_goal_index);
}

bool AvoidancePathPlanner::isTemporaryGoalValid(
    const nav_msgs::msg::Path &global_path,
    const geometry_msgs::msg::PoseStamped &temporary_goal,
    std::size_t following_path_index) {
  if (!astar_planner_ || global_path.poses.size() < 2 ||
      following_path_index >= global_path.poses.size() ||
      !std::isfinite(config_.temporary_goal_clearance) ||
      !std::isfinite(config_.temporary_goal_safe_path_length) ||
      !std::isfinite(config_.temporary_goal_sample_spacing) ||
      config_.temporary_goal_clearance < 0.0 ||
      config_.temporary_goal_safe_path_length <= 0.0 ||
      config_.temporary_goal_sample_spacing <= 0.0) {
    return false;
  }

  const CostmapConstPtr costmap = astar_planner_->getPreparedCostmap();
  if (!costmap || !std::isfinite(costmap->info.resolution) ||
      costmap->info.resolution <= 0.0 || costmap->info.width == 0 ||
      costmap->info.height == 0 ||
      costmap->data.size() != static_cast<std::size_t>(costmap->info.width) *
                                  static_cast<std::size_t>(costmap->info.height) ||
      (!global_path.header.frame_id.empty() &&
       !costmap->header.frame_id.empty() &&
       global_path.header.frame_id != costmap->header.frame_id)) {
    return false;
  }

  const auto &goal_point = temporary_goal.pose.position;
  if (!std::isfinite(goal_point.x) || !std::isfinite(goal_point.y) ||
      !std::isfinite(goal_point.z)) {
    return false;
  }
  for (std::size_t index = following_path_index;
       index < global_path.poses.size(); ++index) {
    const auto &point = global_path.poses[index].pose.position;
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      return false;
    }
  }

  const double sample_spacing = std::max(
      0.001, std::min(config_.temporary_goal_sample_spacing,
                      costmap->info.resolution * 0.5));
  constexpr std::size_t kMaximumTemporaryGoalSamples = 20000U;
  const double requested_sample_count =
      std::ceil(config_.temporary_goal_safe_path_length / sample_spacing) + 2.0;
  if (!std::isfinite(requested_sample_count) ||
      requested_sample_count >
          static_cast<double>(kMaximumTemporaryGoalSamples)) {
    return false;
  }
  std::size_t checked_sample_count = 1U;
  const auto pose_is_safe = [this, &costmap](
                                const geometry_msgs::msg::Point &point,
                                double yaw) {
    return !astar_planner_->isObstacle(point, yaw) &&
           isPointSafeForTemporaryGoal(point, config_.temporary_goal_clearance, costmap);
  };

  const double goal_yaw = tf2::getYaw(temporary_goal.pose.orientation);
  if (!std::isfinite(goal_yaw) || !pose_is_safe(goal_point, goal_yaw)) {
    return false;
  }

  double incoming_yaw = goal_yaw;
  for (std::size_t previous = following_path_index; previous > 0; --previous) {
    const auto &point = global_path.poses[previous - 1].pose.position;
    const double dx = goal_point.x - point.x;
    const double dy = goal_point.y - point.y;
    if (std::hypot(dx, dy) >= kPointEpsilon) {
      incoming_yaw = std::atan2(dy, dx);
      break;
    }
  }
  if (!astar_planner_->isRotationCollisionFree(
          goal_point, incoming_yaw, goal_yaw)) {
    return false;
  }

  double verified_length = 0.0;
  geometry_msgs::msg::Point start = goal_point;
  double start_yaw = goal_yaw;
  for (std::size_t path_index = following_path_index;
       path_index < global_path.poses.size() &&
       verified_length + 1e-9 < config_.temporary_goal_safe_path_length;
       ++path_index) {
    const auto &end = global_path.poses[path_index].pose.position;
    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const double dz = end.z - start.z;
    const double segment_length = std::hypot(dx, dy);
    if (!std::isfinite(segment_length)) return false;
    if (segment_length < kPointEpsilon) {
      start = end;
      continue;
    }

    const double segment_yaw = std::atan2(dy, dx);
    if (!astar_planner_->isRotationCollisionFree(start, start_yaw,
                                                  segment_yaw)) {
      return false;
    }
    const double checked_length = std::min(
        segment_length,
        config_.temporary_goal_safe_path_length - verified_length);
    const int sample_count = std::max(
        1, static_cast<int>(std::ceil(checked_length / sample_spacing)));
    for (int sample = 1; sample <= sample_count; ++sample) {
      if (checked_sample_count >= kMaximumTemporaryGoalSamples) return false;
      ++checked_sample_count;
      const double checked_ratio =
          static_cast<double>(sample) / sample_count;
      const double segment_ratio =
          checked_ratio * checked_length / segment_length;
      geometry_msgs::msg::Point point;
      point.x = start.x + segment_ratio * dx;
      point.y = start.y + segment_ratio * dy;
      point.z = start.z + segment_ratio * dz;

      double sample_yaw = segment_yaw;
      bool check_rotation = false;
      if (sample == sample_count &&
          checked_length >= segment_length - 1e-9) {
        double next_yaw = segment_yaw;
        if (pathTangentAtIndex(global_path, path_index, next_yaw)) {
          sample_yaw = next_yaw;
          check_rotation = true;
        }
      }
      if (!pose_is_safe(point, sample_yaw) ||
          (check_rotation &&
           !astar_planner_->isRotationCollisionFree(
               point, segment_yaw, sample_yaw))) {
        return false;
      }
    }
    verified_length += checked_length;
    start = end;
    if (checked_length >= segment_length - 1e-9) {
      double next_yaw = segment_yaw;
      if (pathTangentAtIndex(global_path, path_index, next_yaw)) {
        start_yaw = next_yaw;
      } else {
        start_yaw = segment_yaw;
      }
    } else {
      start_yaw = segment_yaw;
    }
  }

  return verified_length + 1e-9 >= config_.temporary_goal_safe_path_length;
}

bool AvoidancePathPlanner::buildSideGoal(
    const geometry_msgs::msg::PoseStamped &robot_pose,
    const geometry_msgs::msg::Point &obstacle_center, AvoidanceSide side,
    const nav_msgs::msg::Path &global_path, int closest_index,
    geometry_msgs::msg::Point &side_goal, double &side_goal_yaw,
    double lateral_clearance, double max_entry_angle,
    double minimum_forward_offset) {
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
  const double retry_forward =
      std::max(0.0, std::isfinite(minimum_forward_offset)
                        ? minimum_forward_offset
                        : 0.0);
  const double goal_forward =
      std::max(retry_forward, std::max(0.0, robot_forward + minimum_forward));

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

std::vector<SideGoalCandidate>
AvoidancePathPlanner::buildSideGoalCandidates(
    const geometry_msgs::msg::PoseStamped &robot_pose,
    const geometry_msgs::msg::Point &obstacle_center, AvoidanceSide side,
    const nav_msgs::msg::Path &global_path, int closest_index,
    double max_entry_angle) {
  std::vector<SideGoalCandidate> candidates;
  if (!astar_planner_ || side == AvoidanceSide::NONE ||
      global_path.poses.size() < 2) {
    return candidates;
  }

  if (closest_index < 0) closest_index = 0;
  double path_yaw = 0.0;
  if (!getPathYawAtIndex(global_path, closest_index, path_yaw)) {
    path_yaw = tf2::getYaw(robot_pose.pose.orientation);
  }

  const auto costmap = astar_planner_->getPreparedCostmap();
  if (!costmap || costmap->info.width == 0U || costmap->info.height == 0U ||
      costmap->data.size() != static_cast<std::size_t>(costmap->info.width) *
                                  costmap->info.height ||
      costmap->info.resolution <= 0.0) {
    return candidates;
  }

  const double path_x = std::cos(path_yaw);
  const double path_y = std::sin(path_yaw);
  const double left_x = -path_y;
  const double left_y = path_x;
  const double search_radius =
      std::max(static_cast<double>(costmap->info.resolution),
               config_.side_goal_boundary_search_radius);
  const double longitudinal_band =
      std::max(static_cast<double>(costmap->info.resolution),
               config_.side_goal_boundary_longitudinal_band);
  const double lateral_band =
      std::max(static_cast<double>(costmap->info.resolution),
               config_.side_goal_boundary_lateral_band);

  // 在路径坐标系的十字形关联区域中估计当前障碍的侧边界和前向边界。
  // 相比直接使用障碍中心，这能适应箱体/柱体宽度；限制关联带宽可减少把
  // 远处墙面或另一独立障碍合并进当前障碍包络的概率。
  double left_boundary = 0.0;
  double right_boundary = 0.0;
  double forward_boundary = 0.0;
  bool lateral_boundary_found = false;
  bool forward_boundary_found = false;
  const double origin_yaw = tf2::getYaw(costmap->info.origin.orientation);
  const double origin_cos = std::cos(origin_yaw);
  const double origin_sin = std::sin(origin_yaw);
  for (std::uint32_t map_y = 0; map_y < costmap->info.height; ++map_y) {
    for (std::uint32_t map_x = 0; map_x < costmap->info.width; ++map_x) {
      const std::size_t map_index =
          static_cast<std::size_t>(map_y) * costmap->info.width + map_x;
      const int8_t cost = costmap->data[map_index];
      if (cost < 0 || static_cast<double>(cost) < config_.obstacle_threshold) {
        continue;
      }

      const double local_x =
          (static_cast<double>(map_x) + 0.5) * costmap->info.resolution;
      const double local_y =
          (static_cast<double>(map_y) + 0.5) * costmap->info.resolution;
      const double world_x = costmap->info.origin.position.x +
                             origin_cos * local_x - origin_sin * local_y;
      const double world_y = costmap->info.origin.position.y +
                             origin_sin * local_x + origin_cos * local_y;
      const double dx = world_x - obstacle_center.x;
      const double dy = world_y - obstacle_center.y;
      const double forward = dx * path_x + dy * path_y;
      const double lateral = dx * left_x + dy * left_y;
      if (std::hypot(forward, lateral) > search_radius) continue;

      if (std::abs(forward) <= longitudinal_band) {
        if (!lateral_boundary_found) {
          left_boundary = lateral;
          right_boundary = lateral;
          lateral_boundary_found = true;
        } else {
          left_boundary = std::max(left_boundary, lateral);
          right_boundary = std::min(right_boundary, lateral);
        }
      }
      if (std::abs(lateral) <= lateral_band) {
        if (!forward_boundary_found) {
          forward_boundary = forward;
          forward_boundary_found = true;
        } else {
          forward_boundary = std::max(forward_boundary, forward);
        }
      }
    }
  }
  if (!lateral_boundary_found) {
    left_boundary = 0.0;
    right_boundary = 0.0;
  }
  if (!forward_boundary_found) forward_boundary = 0.0;

  const double robot_forward =
      (robot_pose.pose.position.x - obstacle_center.x) * path_x +
      (robot_pose.pose.position.y - obstacle_center.y) * path_y;
  const double robot_lateral =
      (robot_pose.pose.position.x - obstacle_center.x) * left_x +
      (robot_pose.pose.position.y - obstacle_center.y) * left_y;
  const double entry_angle = std::clamp(
      max_entry_angle, 1.0 * M_PI / 180.0, 89.0 * M_PI / 180.0);
  const int lateral_count =
      std::clamp(config_.side_goal_lateral_sample_count, 1, 10);
  const int forward_count =
      std::clamp(config_.side_goal_forward_sample_count, 1, 10);
  const double minimum_boundary_clearance =
      std::max(0.0, config_.side_goal_min_boundary_clearance);
  const double minimum_forward_clearance =
      std::max(0.0, config_.side_goal_min_forward_clearance);
  const double lateral_step =
      std::max(0.0, config_.side_goal_lateral_sample_step);
  const double forward_step =
      std::max(0.0, config_.side_goal_forward_sample_step);
  candidates.reserve(static_cast<std::size_t>(lateral_count * forward_count));

  const int expansion_rounds =
      std::clamp(config_.side_goal_expansion_rounds, 0, 6);
  const double expansion_lateral_step =
      std::max(0.0, config_.side_goal_expansion_lateral_step);
  const double expansion_forward_step =
      std::max(0.0, config_.side_goal_expansion_forward_step);
  const char *side_name = side == AvoidanceSide::LEFT ? "LEFT" : "RIGHT";

  // 先检查基础3x4候选。只有当本轮所有终点均被完整
  // footprint拒绝时，才同时向障碍侧外和障碍后方扩展下一轮。
  // 找到任意可用终点后立即返回该轮，避免无必要地扩大A*搜索范围。
  for (int expansion_round = 0; expansion_round <= expansion_rounds;
       ++expansion_round) {
    candidates.clear();
    const double extra_lateral =
        expansion_round * expansion_lateral_step;
    const double extra_forward =
        expansion_round * expansion_forward_step;
    int rejected_endpoints = 0;

    for (int forward_index = 0; forward_index < forward_count;
         ++forward_index) {
      const double forward_clearance = minimum_forward_clearance +
                                       extra_forward +
                                       forward_index * forward_step;
      for (int lateral_index = 0; lateral_index < lateral_count;
           ++lateral_index) {
        const double boundary_clearance = minimum_boundary_clearance +
                                          extra_lateral +
                                          lateral_index * lateral_step;
        const double goal_lateral =
            side == AvoidanceSide::LEFT
                ? left_boundary + boundary_clearance
                : right_boundary - boundary_clearance;
        const double lateral_motion = std::abs(goal_lateral - robot_lateral);
        const double entry_limited_forward =
            robot_forward + lateral_motion / std::tan(entry_angle);
        const double goal_forward =
            std::max(forward_boundary + forward_clearance,
                     entry_limited_forward);

        SideGoalCandidate candidate;
        candidate.point.x = obstacle_center.x + path_x * goal_forward +
                            left_x * goal_lateral;
        candidate.point.y = obstacle_center.y + path_y * goal_forward +
                            left_y * goal_lateral;
        candidate.point.z = obstacle_center.z;
        candidate.yaw = path_yaw;
        candidate.side = side;
        candidate.boundary_lateral = side == AvoidanceSide::LEFT
                                         ? left_boundary
                                         : right_boundary;
        candidate.boundary_forward = forward_boundary;
        candidate.boundary_clearance = boundary_clearance;
        candidate.forward_clearance = forward_clearance;

        if (isPointInObstacle(candidate.point, candidate.yaw)) {
          ++rejected_endpoints;
          const int candidate_number =
              forward_index * lateral_count + lateral_index;
          const std::string diagnostic_name =
              std::string("side_") + side_name + "_round_" +
              std::to_string(expansion_round) + "_candidate_" +
              std::to_string(candidate_number);
          isPointInObstacleWithDiagnostics(
              candidate.point, diagnostic_name.c_str(), candidate.yaw);
          RCLCPP_INFO(
              node_->get_logger(),
              "[SIDE_CANDIDATE_REJECT] side=%s round=%d candidate=%d "
              "goal=(%.3f,%.3f) boundary_clearance=%.3f "
              "forward_clearance=%.3f reason=ENDPOINT_FOOTPRINT",
              side_name, expansion_round, candidate_number,
              candidate.point.x, candidate.point.y, boundary_clearance,
              forward_clearance);
          continue;
        }
        candidates.push_back(std::move(candidate));
      }
    }

    RCLCPP_INFO(
        node_->get_logger(),
        "[SIDE_CANDIDATES] side=%s boundary_lat=%.3f "
        "boundary_forward=%.3f round=%d extra_lat=%.3f extra_forward=%.3f "
        "generated=%zu rejected=%d grid=%dx%d",
        side_name,
        side == AvoidanceSide::LEFT ? left_boundary : right_boundary,
        forward_boundary, expansion_round, extra_lateral, extra_forward,
        candidates.size(), rejected_endpoints, lateral_count, forward_count);
    if (!candidates.empty()) {
      return candidates;
    }
    if (expansion_round < expansion_rounds) {
      RCLCPP_WARN(
          node_->get_logger(),
          "[SIDE_CANDIDATE_EXPAND] side=%s round=%d all=%d rejected; "
          "trying round=%d with extra_lat=%.3f extra_forward=%.3f",
          side_name, expansion_round, lateral_count * forward_count,
          expansion_round + 1,
          (expansion_round + 1) * expansion_lateral_step,
          (expansion_round + 1) * expansion_forward_step);
    }
  }

  RCLCPP_WARN(node_->get_logger(),
              "[SIDE_CANDIDATE_EXHAUSTED] side=%s rounds=%d; no "
              "footprint-safe endpoint found",
              side_name, expansion_rounds + 1);
  return candidates;
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
      const bool direct_safe = isPathMotionCollisionFree(candidate_path) &&
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
      if (smoothPath(candidate_path, smoothed_candidate)) {
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
      if (!isPathMotionCollisionFree(candidate_path)) {
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
                                    double goal_yaw,
                                    const AStarAvoidanceHint &avoidance_hint) {
  // 失败时调用者绝不能继续看到上一次成功规划留下的旧路径。
  path = nav_msgs::msg::Path();
  const bool finite_points =
      std::isfinite(start.x) && std::isfinite(start.y) &&
      std::isfinite(start.z) && std::isfinite(goal.x) &&
      std::isfinite(goal.y) && std::isfinite(goal.z);
  // NaN 是“不指定终点朝向”的既有哨兵值；正负无穷不是合法输入。
  if (!finite_points || (!std::isnan(goal_yaw) && !std::isfinite(goal_yaw)) ||
      !astar_planner_ ||
      !astar_planner_->planPath(start, goal, path, goal_yaw,
                                avoidance_hint)) {
    path.poses.clear();
    return false;
  }
  if (path.poses.empty()) {
    path.poses.clear();
    return false;
  }

  // A*回溯默认输出栅格中心。控制目标必须精确落在锁定的原路径临时点，
  // 因此将首尾恢复为真实世界坐标，再复检两端新增的小线段。
  path.poses.front().pose.position = start;
  if (path.poses.size() == 1 &&
      std::hypot(goal.x - start.x, goal.y - start.y) > kPointEpsilon) {
    geometry_msgs::msg::PoseStamped goal_pose = path.poses.front();
    goal_pose.pose.position = goal;
    path.poses.push_back(std::move(goal_pose));
  } else {
    path.poses.back().pose.position = goal;
  }
  const double terminal_yaw = std::isfinite(goal_yaw)
                                  ? goal_yaw
                                  : std::atan2(goal.y - start.y,
                                               goal.x - start.x);
  tf2::Quaternion terminal_orientation;
  terminal_orientation.setRPY(0.0, 0.0, terminal_yaw);
  updatePathOrientations(path, tf2::toMsg(terminal_orientation));
  // 这里保留原始 A* 折线供上层尝试贝塞尔平滑。上层只有在
  // isPathMotionCollisionFree() 通过后才可直接使用该原始折线；这样
  // “原地转弯扫掠不安全、但平滑曲线可安全通过”的路径仍有修复机会。
  if (!astar_planner_->isPathCollisionFree(path, true)) {
    path.poses.clear();
    return false;
  }
  publishRawPath(path);
  return true;
}

bool AvoidancePathPlanner::smoothPath(
    const nav_msgs::msg::Path &raw_path,
    nav_msgs::msg::Path &smoothed_path) {
  if (config_.smoothing_method == "bspline") {
    return smoothPathWithBSpline(raw_path, smoothed_path);
  }
  if (config_.smoothing_method == "none") {
    smoothed_path = raw_path;
    publishPrunedPath(raw_path);
    publishSmoothingPaths(raw_path, smoothed_path);
    return false;
  }
  if (config_.smoothing_method != "bezier") {
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000,
        "Unknown local_astar_smoothing_method='%s'; using bezier",
        config_.smoothing_method.c_str());
  }
  return smoothPathWithBezier(raw_path, smoothed_path);
}

bool AvoidancePathPlanner::smoothPathWithBSpline(
    const nav_msgs::msg::Path &raw_path,
    nav_msgs::msg::Path &smoothed_path) {
  smoothed_path = raw_path;
  if (!config_.bspline_smoothing_enabled || !astar_planner_ ||
      raw_path.poses.size() < 3U) {
    publishPrunedPath(raw_path);
    publishSmoothingPaths(raw_path, smoothed_path, true);
    return false;
  }

  nav_msgs::msg::Path duplicate_free;
  duplicate_free.header = raw_path.header;
  for (const auto &pose : raw_path.poses) {
    appendDistinctPose(duplicate_free, pose);
  }
  if (duplicate_free.poses.size() < 3U) {
    publishPrunedPath(duplicate_free);
    publishSmoothingPaths(raw_path, smoothed_path, true);
    return false;
  }

  const nav_msgs::msg::Path base_control_path = simplifyPathRdp(
      duplicate_free, std::max(0.0, config_.bspline_simplify_tolerance),
      std::max(0.0, config_.smoothing_start_trend_distance));
  publishPrunedPath(base_control_path);
  if (base_control_path.poses.size() < 3U) {
    // 直线路径无需以B样条重复表达。
    publishSmoothingPaths(raw_path, smoothed_path, true);
    return false;
  }

  const double sample_spacing =
      std::max(0.01, config_.bspline_sample_spacing);
  const int attempt_count = std::max(1, config_.bspline_adaptive_attempts);
  const double initial_control_spacing = std::max(
      config_.bspline_min_control_point_spacing,
      config_.bspline_control_point_spacing);
  bool saw_deviation_failure = false;
  bool saw_curvature_failure = false;
  bool saw_collision_failure = false;
  std::size_t last_control_count = 0U;
  std::size_t last_candidate_count = 0U;
  double last_minimum_radius = std::numeric_limits<double>::infinity();
  double accepted_control_spacing = initial_control_spacing;
  int accepted_attempt = -1;

  for (int attempt = 0; attempt < attempt_count; ++attempt) {
    // 保留原RDP控制折线作为首次尝试，避免改变原来已经能通过的宽阔
    // 转角。失败后才按弧长均匀重采样，并逐步缩小平滑作用范围，让
    // 曲线更贴近A*安全走廊，而不是立即退回折线。
    const double control_spacing = std::max(
        config_.bspline_min_control_point_spacing,
        initial_control_spacing * std::pow(0.75, std::max(0, attempt - 1)));
    const nav_msgs::msg::Path uniform_control_path =
        attempt == 0
            ? base_control_path
            : resamplePathByArcLength(base_control_path, control_spacing);
    if (uniform_control_path.poses.size() < 3U) continue;

    std::vector<geometry_msgs::msg::Point> control_points;
    control_points.reserve(uniform_control_path.poses.size());
    for (const auto &pose : uniform_control_path.poses) {
      control_points.push_back(pose.pose.position);
    }
    if (control_points.size() >= 3U &&
        config_.bspline_start_control_blend > 0.0) {
      // 夹持B样条的起点位置仍由P0严格保持；只将P1轻微朝P2移动，令
      // 起点切线从第一小段之后就逐步进入绕障曲率，而非延续长直线。
      const double blend =
          std::clamp(config_.bspline_start_control_blend, 0.0, 0.45);
      control_points[1U].x =
          (1.0 - blend) * control_points[1U].x +
          blend * control_points[2U].x;
      control_points[1U].y =
          (1.0 - blend) * control_points[1U].y +
          blend * control_points[2U].y;
      control_points[1U].z =
          (1.0 - blend) * control_points[1U].z +
          blend * control_points[2U].z;
    }
    last_control_count = control_points.size();
    const int sample_count = std::max(
        static_cast<int>(control_points.size()) * 4,
        static_cast<int>(std::ceil(
            pathLength2d(uniform_control_path) / sample_spacing)));
    nav_msgs::msg::Path candidate;
    candidate.header = raw_path.header;
    bool deviation_valid = true;
    for (int sample = 0; sample <= sample_count; ++sample) {
      const double parameter = static_cast<double>(sample) / sample_count;
      geometry_msgs::msg::PoseStamped pose = raw_path.poses.front();
      pose.pose.position = evaluateClampedBSpline(
          control_points, config_.bspline_degree, parameter);
      if (pointToPathDistance(pose.pose.position, duplicate_free) >
          config_.bspline_max_deviation + 1e-9) {
        deviation_valid = false;
        break;
      }
      appendDistinctPose(candidate, std::move(pose));
    }
    last_candidate_count = candidate.poses.size();
    if (!deviation_valid || candidate.poses.size() < 3U) {
      saw_deviation_failure = true;
      continue;
    }

    candidate.poses.front().pose.position =
        raw_path.poses.front().pose.position;
    candidate.poses.back().pose.position = raw_path.poses.back().pose.position;
    updatePathOrientations(candidate, raw_path.poses.back().pose.orientation);
    last_minimum_radius = minimumDiscretePathRadius(candidate);
    if (config_.bspline_min_turn_radius > 0.0 &&
        last_minimum_radius + 1e-9 < config_.bspline_min_turn_radius) {
      saw_curvature_failure = true;
      continue;
    }
    if (!isPathMotionCollisionFree(candidate)) {
      saw_collision_failure = true;
      continue;
    }

    smoothed_path = std::move(candidate);
    accepted_control_spacing = attempt == 0 ? 0.0 : control_spacing;
    accepted_attempt = attempt;
    break;
  }

  if (accepted_attempt < 0) {
    const char *reason = saw_collision_failure
                             ? "COLLISION"
                             : (saw_curvature_failure ? "CURVATURE"
                                                      : "MAX_DEVIATION");
    RCLCPP_WARN(node_->get_logger(),
                "[BSPLINE_FALLBACK] attempts=%d control=%zu candidate=%zu "
                "min_radius=%.3f required=%.3f deviation_failed=%d "
                "curvature_failed=%d collision_failed=%d reason=%s; "
                "using raw A*",
                attempt_count, last_control_count, last_candidate_count,
                last_minimum_radius, config_.bspline_min_turn_radius,
                saw_deviation_failure ? 1 : 0,
                saw_curvature_failure ? 1 : 0,
                saw_collision_failure ? 1 : 0, reason);
    publishSmoothingPaths(raw_path, smoothed_path, true);
    return false;
  }

  publishSmoothingPaths(raw_path, smoothed_path, true);
  RCLCPP_INFO(node_->get_logger(),
              "[BSPLINE_SMOOTH] raw=%zu control=%zu degree=%d smooth=%zu "
              "attempt=%d control_spacing=%.3f min_radius=%.3f "
              "max_deviation=%.3f",
              raw_path.poses.size(), last_control_count,
              std::min(config_.bspline_degree,
                       static_cast<int>(last_control_count) - 1),
              smoothed_path.poses.size(), accepted_attempt + 1,
              accepted_control_spacing, last_minimum_radius,
              config_.bspline_max_deviation);
  return true;
}

bool AvoidancePathPlanner::smoothPathWithBezier(
    const nav_msgs::msg::Path &raw_path,
    nav_msgs::msg::Path &smoothed_path) {
  smoothed_path = raw_path;
  if (!config_.bezier_smoothing_enabled || !astar_planner_ ||
      raw_path.poses.size() < 3) {
    publishPrunedPath(raw_path);
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
    publishPrunedPath(duplicate_free_path);
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
  std::size_t protected_start_index = 0U;
  double protected_start_length = 0.0;
  if (config_.smoothing_start_trend_distance > 0.0) {
    for (std::size_t index = 1U; index < clean_path.poses.size(); ++index) {
      const auto &previous = clean_path.poses[index - 1U].pose.position;
      const auto &current = clean_path.poses[index].pose.position;
      protected_start_length +=
          std::hypot(current.x - previous.x, current.y - previous.y);
      protected_start_index = index;
      if (protected_start_length + 1e-9 >=
          config_.smoothing_start_trend_distance) {
        break;
      }
    }
  }

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
      // 起点附近禁止LOS直接跨过首个侧绕趋势锚点；到达该点以后恢复
      // 原有的最远可视顶点裁剪。
      const std::size_t farthest_candidate =
          anchor < protected_start_index
              ? protected_start_index
              : clean_path.poses.size() - 1U;
      for (std::size_t candidate_index = farthest_candidate;
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

  publishPrunedPath(clean_path);

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
    if (!isPathMotionCollisionFree(clean_path)) {
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
      !isPathMotionCollisionFree(candidate)) {
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
    const nav_msgs::msg::Path &smoothed_path, bool bspline) {
  const auto &selected_publisher =
      bspline ? bspline_path_pub_ : smoothed_path_pub_;
  if (!config_.bezier_visualization_enabled || !raw_path_pub_ ||
      !selected_publisher) {
    return;
  }
  nav_msgs::msg::Path raw_visualization = raw_path;
  nav_msgs::msg::Path smoothed_visualization = smoothed_path;
  const auto stamp = node_->now();
  raw_visualization.header.stamp = stamp;
  smoothed_visualization.header.stamp = stamp;
  raw_path_pub_->publish(raw_visualization);
  selected_publisher->publish(smoothed_visualization);
  if (selected_smoothed_path_pub_) {
    selected_smoothed_path_pub_->publish(smoothed_visualization);
  }
}

void AvoidancePathPlanner::publishRawPath(
    const nav_msgs::msg::Path &raw_path) {
  if (!config_.bezier_visualization_enabled || !raw_path_pub_) return;
  nav_msgs::msg::Path visualization = raw_path;
  visualization.header.stamp = node_->now();
  raw_path_pub_->publish(visualization);
}

void AvoidancePathPlanner::publishPrunedPath(
    const nav_msgs::msg::Path &pruned_path) {
  if (!config_.bezier_visualization_enabled || !pruned_path_pub_) return;
  nav_msgs::msg::Path visualization = pruned_path;
  visualization.header.stamp = node_->now();
  pruned_path_pub_->publish(visualization);
}

bool AvoidancePathPlanner::isPointInObstacle(
    const geometry_msgs::msg::Point &point, double yaw) {
  return !astar_planner_ || astar_planner_->isObstacle(point, yaw);
}

bool AvoidancePathPlanner::isPointInObstacleWithDiagnostics(
    const geometry_msgs::msg::Point &point, const char *point_name,
    double yaw) {
  return !astar_planner_ ||
         astar_planner_->isObstacleWithDiagnostics(point, point_name, yaw);
}

bool AvoidancePathPlanner::isPathCollisionFree(
    const nav_msgs::msg::Path &path, bool allow_initial_escape) const {
  return astar_planner_ &&
         astar_planner_->isPathCollisionFree(path, allow_initial_escape);
}

bool AvoidancePathPlanner::isPathMotionCollisionFree(
    const nav_msgs::msg::Path &path, bool allow_initial_escape) const {
  if (!astar_planner_ || path.poses.empty()) {
    return false;
  }
  for (const auto &pose : path.poses) {
    const auto &point = pose.pose.position;
    const auto &orientation = pose.pose.orientation;
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z) || !std::isfinite(orientation.x) ||
        !std::isfinite(orientation.y) || !std::isfinite(orientation.z) ||
        !std::isfinite(orientation.w)) {
      return false;
    }
  }
  if (!astar_planner_->isPathCollisionFree(path, allow_initial_escape)) {
    return false;
  }

  // 线段碰撞检查只能证明平移时安全。四足机器人到达折点后还需要从
  // 入射方向转到出射方向，终点也需要转到目标姿态；逐点验证完整
  // footprint 的旋转扫掠，原始 A* 与贝塞尔回退共用同一安全语义。
  bool reached_strictly_free_pose = !allow_initial_escape;
  for (std::size_t index = 1; index < path.poses.size(); ++index) {
    const auto &center = path.poses[index].pose.position;
    if (!std::isfinite(center.x) || !std::isfinite(center.y) ||
        !std::isfinite(center.z)) {
      return false;
    }

    double incoming_yaw = 0.0;
    bool has_incoming = false;
    for (std::size_t previous = index; previous > 0; --previous) {
      const auto &point = path.poses[previous - 1].pose.position;
      const double dx = center.x - point.x;
      const double dy = center.y - point.y;
      if (!std::isfinite(dx) || !std::isfinite(dy)) return false;
      if (std::hypot(dx, dy) >= kPointEpsilon) {
        incoming_yaw = std::atan2(dy, dx);
        has_incoming = true;
        break;
      }
    }
    if (!has_incoming) continue;

    double outgoing_yaw = 0.0;
    bool has_outgoing = false;
    for (std::size_t next = index + 1; next < path.poses.size(); ++next) {
      const auto &point = path.poses[next].pose.position;
      const double dx = point.x - center.x;
      const double dy = point.y - center.y;
      if (!std::isfinite(dx) || !std::isfinite(dy)) return false;
      if (std::hypot(dx, dy) >= kPointEpsilon) {
        outgoing_yaw = std::atan2(dy, dx);
        has_outgoing = true;
        break;
      }
    }
    if (!has_outgoing) {
      const auto &orientation = path.poses.back().pose.orientation;
      outgoing_yaw = tf2::getYaw(orientation);
    }

    // isPathCollisionFree已经确认：宽松部分只能是路径起点处连续的
    // footprint冲突前缀，且中心没有进入硬障碍。这个前缀尚无空间完成
    // 原地旋转，因此只允许保持当前平移方向向外退出；首次进入严格安全
    // 姿态后，所有折点和终点旋转恢复完整检查。
    const bool incoming_footprint_free =
        !astar_planner_->isObstacle(center, incoming_yaw);
    if (!reached_strictly_free_pose && !incoming_footprint_free) {
      const double turn = std::abs(std::atan2(
          std::sin(outgoing_yaw - incoming_yaw),
          std::cos(outgoing_yaw - incoming_yaw)));
      if (turn > 1e-3) return false;
      continue;
    }
    reached_strictly_free_pose = true;

    if (!astar_planner_->isRotationCollisionFree(
            center, incoming_yaw, outgoing_yaw)) {
      return false;
    }
  }
  return true;
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
         isPathMotionCollisionFree(path, true);
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

bool AvoidancePathPlanner::isPointSafeForTemporaryGoal(
    const geometry_msgs::msg::Point &point, double clearance_radius,
    const CostmapConstPtr &costmap) const {
  if (!costmap || !std::isfinite(costmap->info.resolution) ||
      costmap->info.resolution <= 0.0 ||
      costmap->info.width == 0 || costmap->info.height == 0 ||
      costmap->info.width >
          static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
      costmap->info.height >
          static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
      !std::isfinite(costmap->info.origin.position.x) ||
      !std::isfinite(costmap->info.origin.position.y) ||
      costmap->data.size() != static_cast<std::size_t>(costmap->info.width) *
                                  static_cast<std::size_t>(costmap->info.height)) {
    return false;
  }

  if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
      !std::isfinite(clearance_radius) || clearance_radius < 0.0) {
    return false;
  }

  const double resolution = costmap->info.resolution;
  const double map_min_x = costmap->info.origin.position.x;
  const double map_min_y = costmap->info.origin.position.y;
  const double map_max_x =
      map_min_x + static_cast<double>(costmap->info.width) * resolution;
  const double map_max_y =
      map_min_y + static_cast<double>(costmap->info.height) * resolution;
  // 临时点安全圆必须完整落在当前 Costmap 内；地图外区域按未知处理。
  if (point.x - clearance_radius < map_min_x - 1e-9 ||
      point.y - clearance_radius < map_min_y - 1e-9 ||
      point.x + clearance_radius > map_max_x + 1e-9 ||
      point.y + clearance_radius > map_max_y + 1e-9) {
    return false;
  }
  const int center_x = static_cast<int>(std::floor(
      (point.x - map_min_x) / resolution));
  const int center_y = static_cast<int>(std::floor(
      (point.y - map_min_y) / resolution));
  const int width = static_cast<int>(costmap->info.width);
  const int height = static_cast<int>(costmap->info.height);
  if (center_x < 0 || center_y < 0 || center_x >= width || center_y >= height) {
    return false;
  }

  // 多检查一圈栅格，再用点到栅格方块(AABB)的真实最短距离筛选。
  // 不能只用待检查点所属栅格与障碍栅格的整数索引距离，否则点位于
  // 栅格边缘时会少检查最多约一个栅格对角线的净空。
  const int radius_cells = std::max(
      1, static_cast<int>(std::ceil(clearance_radius / resolution)) + 1);
  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      const int grid_x = center_x + dx;
      const int grid_y = center_y + dy;
      if (grid_x < 0 || grid_y < 0 || grid_x >= width || grid_y >= height) {
        continue;
      }
      const double cell_min_x =
          map_min_x + static_cast<double>(grid_x) * resolution;
      const double cell_min_y =
          map_min_y + static_cast<double>(grid_y) * resolution;
      const double cell_max_x = cell_min_x + resolution;
      const double cell_max_y = cell_min_y + resolution;
      const double distance_x =
          std::max({cell_min_x - point.x, 0.0, point.x - cell_max_x});
      const double distance_y =
          std::max({cell_min_y - point.y, 0.0, point.y - cell_max_y});
      if (std::hypot(distance_x, distance_y) > clearance_radius + 1e-9) {
        continue;
      }
      const std::size_t index =
          static_cast<std::size_t>(grid_y) * costmap->info.width +
          static_cast<std::size_t>(grid_x);
      const int cost = static_cast<int>(costmap->data[index]);
      if (cost < 0 || cost >= config_.obstacle_threshold) return false;
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
