#include "pure_pursuit/path_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace pure_pursuit_local_planner {

bool PathTracker::initialize(rclcpp::Node::SharedPtr node,
                             const PathTrackerConfig &config) {
  node_ = node;
  if (config.lookahead_distance_min < 0.0 ||
      config.lookahead_distance_max < config.lookahead_distance_min ||
      config.max_linear_velocity < 0.0 || config.output_frame.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "Invalid path tracker configuration");
    return false;
  }
  config_ = config;

  // 初始化发布器
  path_markers_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
      "path_markers", 1);
  lookahead_marker_pub_ =
      node_->create_publisher<visualization_msgs::msg::Marker>(
          "lookahead_marker", 1);

  RCLCPP_INFO(node_->get_logger(), "Path tracker initialized");
  return true;
}

bool PathTracker::setPath(const nav_msgs::msg::Path &path) {
  if (path.poses.empty()) {
    RCLCPP_WARN(node_->get_logger(), "Received empty path");
    path_ = nav_msgs::msg::Path();
    return false;
  }

  // 当前跟踪接口直接接收路径快照，这里只保存全局路径用于可视化。
  path_ = path;
  RCLCPP_INFO(node_->get_logger(), "Path set with %zu points",
              path_.poses.size());
  return true;
}

std::optional<TrackingResult> PathTracker::computeTrackingLookahead(
    const geometry_msgs::msg::PoseStamped &robot_pose,
    const nav_msgs::msg::Path &path, const TrackingContext &context) {
  if (path.poses.empty())
    return std::nullopt;

  const double velocity_factor =
      config_.max_linear_velocity > 0.0
          ? std::clamp(
                context.current_velocity / config_.max_linear_velocity, 0.0,
                1.0)
          : 0.0;
  const double lookahead_distance =
      config_.lookahead_distance_min +
      velocity_factor *
          (config_.lookahead_distance_max - config_.lookahead_distance_min);
  const double robot_x = robot_pose.pose.position.x;
  const double robot_y = robot_pose.pose.position.y;
  // 全局和局部路径都从各自的单调进度开始搜索，禁止回归阶段重新追踪
  // 已经驶过的局部路径段。
  const std::size_t start_index = context.progress_index;
  if (start_index >= path.poses.size())
    return std::nullopt;

  bool intersection_found = false;
  std::size_t best_segment_index = 0;
  double best_path_progress = -1.0;
  double cumulative_path_length = 0.0;
  geometry_msgs::msg::Point best_intersection;

  // 在所有圆线交点中选择路径累计进度最大的交点，保持原跟踪行为不变。
  for (std::size_t i = start_index; i + 1 < path.poses.size(); ++i) {
    const auto &p1 = path.poses[i].pose.position;
    const auto &p2 = path.poses[i + 1].pose.position;
    std::array<geometry_msgs::msg::Point, 2> segment_intersections;
    const std::size_t intersection_count = computeCircleLineIntersection(
        robot_x, robot_y, lookahead_distance, p1, p2, segment_intersections);

    for (std::size_t j = 0; j < intersection_count; ++j) {
      const auto &intersection = segment_intersections[j];
      const double path_progress =
          cumulative_path_length + calculateDistance(p1, intersection);
      if (!intersection_found || path_progress > best_path_progress) {
        intersection_found = true;
        best_segment_index = i;
        best_path_progress = path_progress;
        best_intersection = intersection;
      }
    }
    cumulative_path_length += calculateDistance(p1, p2);
  }

  TrackingResult result;
  result.lookahead_point.header.frame_id = config_.output_frame;
  result.lookahead_point.header.stamp = node_->now();
  if (intersection_found) {
    result.lookahead_point.point = best_intersection;
    result.path_index = best_segment_index;
    return result;
  }

  // 机器人横向偏离路径超过前瞻半径时，圆与路径没有交点。此时不能按
  // “点索引+10”跳转，因为路径点可能很稀疏，10个点可能代表数米距离。
  // 先求机器人在前方路径上的最近投影，再沿路径弧长前进一个前瞻距离。
  std::size_t projection_segment = start_index;
  double projection_ratio = 0.0;
  double min_projection_distance_sq =
      std::numeric_limits<double>::infinity();
  for (std::size_t i = start_index; i + 1 < path.poses.size(); ++i) {
    const auto &p1 = path.poses[i].pose.position;
    const auto &p2 = path.poses[i + 1].pose.position;
    const double segment_x = p2.x - p1.x;
    const double segment_y = p2.y - p1.y;
    const double segment_length_sq =
        segment_x * segment_x + segment_y * segment_y;
    if (segment_length_sq < 1e-8)
      continue;

    const double ratio = std::clamp(
        ((robot_x - p1.x) * segment_x + (robot_y - p1.y) * segment_y) /
            segment_length_sq,
        0.0, 1.0);
    const double projected_x = p1.x + ratio * segment_x;
    const double projected_y = p1.y + ratio * segment_y;
    const double distance_x = robot_x - projected_x;
    const double distance_y = robot_y - projected_y;
    const double distance_sq =
        distance_x * distance_x + distance_y * distance_y;
    if (distance_sq < min_projection_distance_sq) {
      min_projection_distance_sq = distance_sq;
      projection_segment = i;
      projection_ratio = ratio;
    }
  }

  if (!std::isfinite(min_projection_distance_sq)) {
    result.lookahead_point.point = path.poses[start_index].pose.position;
    result.path_index = start_index;
    return result;
  }

  double remaining_distance = lookahead_distance;
  for (std::size_t i = projection_segment; i + 1 < path.poses.size(); ++i) {
    const auto &p1 = path.poses[i].pose.position;
    const auto &p2 = path.poses[i + 1].pose.position;
    const double segment_x = p2.x - p1.x;
    const double segment_y = p2.y - p1.y;
    const double segment_length = std::hypot(segment_x, segment_y);
    if (segment_length < 1e-8)
      continue;

    const double segment_start_ratio =
        i == projection_segment ? projection_ratio : 0.0;
    const double available_length =
        (1.0 - segment_start_ratio) * segment_length;
    if (remaining_distance <= available_length) {
      const double target_ratio =
          segment_start_ratio + remaining_distance / segment_length;
      result.lookahead_point.point.x = p1.x + target_ratio * segment_x;
      result.lookahead_point.point.y = p1.y + target_ratio * segment_y;
      result.lookahead_point.point.z =
          p1.z + target_ratio * (p2.z - p1.z);
      result.path_index = i;
      RCLCPP_INFO_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "[PP_LOOKAHEAD_FALLBACK] path=%s projection_segment=%zu, "
          "lateral_error=%.3f, lookahead=%.3f, target_segment=%zu, "
          "target=(%.3f, %.3f)",
          pathTypeLabel(context.path_type), projection_segment,
          std::sqrt(min_projection_distance_sq), lookahead_distance, i,
          result.lookahead_point.point.x, result.lookahead_point.point.y);
      return result;
    }
    remaining_distance -= available_length;
  }

  result.lookahead_point.point = path.poses.back().pose.position;
  result.path_index = path.poses.size() - 1;
  RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "[PP_LOOKAHEAD_FALLBACK] path=%s projection_segment=%zu, "
      "lateral_error=%.3f, lookahead reaches path end",
      pathTypeLabel(context.path_type), projection_segment,
      std::sqrt(min_projection_distance_sq));
  return result;
}

const char *PathTracker::pathTypeLabel(TrackingPathType path_type) {
  switch (path_type) {
  case TrackingPathType::kGlobal:
    return "global path";
  case TrackingPathType::kLocal:
    return "original local path";
  case TrackingPathType::kSmoothedLocal:
    return "smoothed local path";
  }
  return "unknown path";
}

std::size_t PathTracker::computeCircleLineIntersection(
    double circle_x, double circle_y, double radius,
    const geometry_msgs::msg::Point &line_start,
    const geometry_msgs::msg::Point &line_end,
    std::array<geometry_msgs::msg::Point, 2> &intersections) {
  const double dx = line_end.x - line_start.x;
  const double dy = line_end.y - line_start.y;
  const double fx = line_start.x - circle_x;
  const double fy = line_start.y - circle_y;
  const double a = dx * dx + dy * dy;
  const double b = 2.0 * (fx * dx + fy * dy);
  const double c = fx * fx + fy * fy - radius * radius;
  const double discriminant = b * b - 4.0 * a * c;
  if (discriminant < 0.0 || a < 1e-8)
    return 0;

  const double sqrt_discriminant = std::sqrt(discriminant);
  const std::array<double, 2> parameters = {
      (-b - sqrt_discriminant) / (2.0 * a),
      (-b + sqrt_discriminant) / (2.0 * a)};
  std::size_t count = 0;
  for (const double t : parameters) {
    if (t < 0.0 || t > 1.0)
      continue;
    if (count > 0 && std::abs(t - parameters[0]) <= 1e-8)
      continue;
    intersections[count].x = line_start.x + t * dx;
    intersections[count].y = line_start.y + t * dy;
    intersections[count].z = line_start.z;
    ++count;
  }
  return count;
}

double PathTracker::computeTargetVelocity(
    const geometry_msgs::msg::PoseStamped &robot_pose,
    const geometry_msgs::msg::PointStamped &lookahead_point,
    double max_velocity, int motion_direction) {
  const int direction = motion_direction == -1 ? -1 : 1;
  // 倒车时使用机器人后方作为实际运动方向。
  double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
  double robot_dir_x = direction * std::cos(robot_yaw);
  double robot_dir_y = direction * std::sin(robot_yaw);

  // 计算机器人位置到前瞻点的方向向量
  double dx = lookahead_point.point.x - robot_pose.pose.position.x;
  double dy = lookahead_point.point.y - robot_pose.pose.position.y;
  double distance = std::hypot(dx, dy);

  if (distance < 1e-6) {
    // 如果距离过小，返回最大速度
    return max_velocity;
  }

  // 计算前瞻点方向角度
  double theta = std::atan2(dy, dx);

  // 计算方向误差
  const double motion_yaw =
      robot_yaw + (direction == -1 ? M_PI : 0.0);
  double angle_error = theta - motion_yaw;
  // 标准化角度到[-π, π]
  while (angle_error > M_PI)
    angle_error -= 2.0 * M_PI;
  while (angle_error < -M_PI)
    angle_error += 2.0 * M_PI;

  // 当误差大于1.57弧度(90度)时，降低速度进行方向调整
  if (std::abs(angle_error) > 1.57) {
    // 方向误差过大，原地转向
    return 0;
  } else {
    // 计算前瞻点方向单位向量
    double lookahead_dir_x = dx / distance;
    double lookahead_dir_y = dy / distance;

    // 计算两个单位向量的点乘（余弦值）
    double dot_product =
        robot_dir_x * lookahead_dir_x + robot_dir_y * lookahead_dir_y;

    // 确保点乘结果在合理范围内 [0, 1]
    dot_product = std::max(0.0, std::min(1.0, dot_product));

    // 返回点乘结果乘以最大速度
    return dot_product * max_velocity;
  }
}

void PathTracker::publishVisualization(
    const geometry_msgs::msg::PointStamped &lookahead_point) {
  // 发布路径标记
  visualization_msgs::msg::Marker path_marker = createPathMarker();
  path_markers_pub_->publish(path_marker);

  // 发布前瞻点标记
  visualization_msgs::msg::Marker lookahead_marker =
      createLookaheadMarker(lookahead_point);
  lookahead_marker_pub_->publish(lookahead_marker);

}

visualization_msgs::msg::Marker PathTracker::createPathMarker() {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = path_.header.frame_id;
  marker.header.stamp = node_->now();
  marker.ns = "path";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.scale.x = 0.05;
  marker.color.a = 0.8;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  for (const auto &pose : path_.poses) {
    marker.points.push_back(pose.pose.position);
  }

  return marker;
}

visualization_msgs::msg::Marker PathTracker::createLookaheadMarker(
    const geometry_msgs::msg::PointStamped &lookahead_point) {
  visualization_msgs::msg::Marker marker;
  marker.header = lookahead_point.header;
  marker.ns = "lookahead";
  marker.id = 1;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.position = lookahead_point.point;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;

  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;

  return marker;
}

double PathTracker::calculateDistance(const geometry_msgs::msg::Point &p1,
                                      const geometry_msgs::msg::Point &p2) {
  return std::hypot(p1.x - p2.x, p1.y - p2.y);
}

} // namespace pure_pursuit_local_planner
