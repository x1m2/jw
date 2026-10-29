#include "pure_pursuit/front_obstacle_detector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <optional>

#include <std_msgs/msg/color_rgba.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include "pure_pursuit/param_utils.hpp"

namespace pure_pursuit_local_planner {

namespace {

struct NarrowPassageBin {
  double s{0.0};
  std::vector<double> left_obstacles;
  std::vector<double> right_obstacles;
  std::optional<double> left_boundary;
  std::optional<double> right_boundary;
  double width{0.0};
  double center{0.0};
  bool valid{false};
};

double median(std::vector<double> values) {
  if (values.empty()) return 0.0;
  const auto middle = values.begin() + values.size() / 2;
  std::nth_element(values.begin(), middle, values.end());
  double result = *middle;
  if (values.size() % 2 == 0) {
    const auto lower = std::max_element(values.begin(), middle);
    result = 0.5 * (result + *lower);
  }
  return result;
}

std::pair<double, double> fitLine(
    const std::vector<std::pair<double, double>> &samples) {
  if (samples.empty()) return {0.0, 0.0};
  if (samples.size() == 1) return {0.0, samples.front().second};

  double mean_s = 0.0;
  double mean_d = 0.0;
  for (const auto &sample : samples) {
    mean_s += sample.first;
    mean_d += sample.second;
  }
  mean_s /= static_cast<double>(samples.size());
  mean_d /= static_cast<double>(samples.size());

  double numerator = 0.0;
  double denominator = 0.0;
  for (const auto &sample : samples) {
    const double ds = sample.first - mean_s;
    numerator += ds * (sample.second - mean_d);
    denominator += ds * ds;
  }
  const double slope = denominator > 1e-9 ? numerator / denominator : 0.0;
  return {slope, mean_d - slope * mean_s};
}

std_msgs::msg::ColorRGBA color(float r, float g, float b, float a = 1.0F) {
  std_msgs::msg::ColorRGBA value;
  value.r = r;
  value.g = g;
  value.b = b;
  value.a = a;
  return value;
}

}  // namespace

FrontObstacleDetector::FrontObstacleDetector()
    : has_costmap_(false),
      has_global_path_(false),
      has_narrow_passage_reference_path_(false),
      obstacle_threshold_(50),
      obstacle_ratio_threshold_(0.1),
      robot_width_(0.80),
      robot_length_(1.10),
      safety_margin_(0.1),
      avoid_min_distance_(1.0),
      avoid_max_distance_(1.5),
      avoid_sector_angle_(60.0),
      side_passage_width_(0.75),
      side_passage_obstacle_ratio_threshold_(0.02),
      near_side_obstacle_lateral_threshold_(0.20),
      enable_path_corridor_filter_(true),
      path_obstacle_margin_(0.15),
      path_check_distance_(2.0),
      narrow_passage_enabled_(true),
      narrow_passage_margin_(0.12),
      narrow_passage_forward_range_(2.0),
      narrow_passage_lateral_range_(1.2),
      narrow_passage_bin_size_(0.15),
      narrow_passage_min_length_(1.0),
      narrow_passage_valid_bin_ratio_(0.70),
      narrow_passage_center_smoothing_window_(3),
      narrow_passage_min_confidence_(0.70),
      narrow_passage_boundary_continuity_tolerance_(0.20),
      narrow_passage_direction_tolerance_degrees_(20.0),
      narrow_passage_width_stddev_tolerance_(0.15),
      narrow_passage_center_obstacle_ratio_threshold_(0.02),
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
  side_passage_obstacle_ratio_threshold_ = std::clamp(
      declareOrGetParameter(
          node_, "front_obstacle_detector.side_passage_obstacle_ratio_threshold",
          0.02),
      0.0, 1.0);
  near_side_obstacle_lateral_threshold_ = std::clamp(
      declareOrGetParameter(
          node_,
          "front_obstacle_detector.near_side_obstacle_lateral_threshold",
          0.20),
      0.0, robot_width_ * 0.5 + safety_margin_);
  enable_path_corridor_filter_ = declareOrGetParameter(
      node_, "front_obstacle_detector.enable_path_corridor_filter", true);
  path_obstacle_margin_ = declareOrGetParameter(
      node_, "front_obstacle_detector.path_obstacle_margin", 0.15);
  path_check_distance_ = declareOrGetParameter(
      node_, "front_obstacle_detector.path_check_distance", 2.0);
  narrow_passage_enabled_ =
      declareOrGetParameter(node_, "narrow_passage_enabled", true);
  narrow_passage_margin_ = std::max(
      0.0, declareOrGetParameter(node_, "narrow_passage_margin", 0.12));
  narrow_passage_forward_range_ = std::max(
      0.1,
      declareOrGetParameter(node_, "narrow_passage_forward_range", 2.0));
  narrow_passage_lateral_range_ = std::max(
      robot_width_ * 0.5 + narrow_passage_margin_,
      declareOrGetParameter(node_, "narrow_passage_lateral_range", 1.2));
  narrow_passage_bin_size_ = std::max(
      0.05, declareOrGetParameter(node_, "narrow_passage_bin_size", 0.15));
  narrow_passage_min_length_ = std::clamp(
      declareOrGetParameter(node_, "narrow_passage_min_length", 1.0),
      narrow_passage_bin_size_, narrow_passage_forward_range_);
  narrow_passage_valid_bin_ratio_ = std::clamp(
      declareOrGetParameter(node_, "narrow_passage_valid_bin_ratio", 0.70),
      0.0, 1.0);
  narrow_passage_center_smoothing_window_ = std::max(
      1, declareOrGetParameter(
             node_, "narrow_passage_center_smoothing_window", 3));
  narrow_passage_min_confidence_ = std::clamp(
      declareOrGetParameter(node_, "narrow_passage_min_confidence", 0.70),
      0.0, 1.0);
  narrow_passage_boundary_continuity_tolerance_ = std::max(
      0.05, declareOrGetParameter(
                node_, "narrow_passage_boundary_continuity_tolerance", 0.20));
  narrow_passage_direction_tolerance_degrees_ = std::clamp(
      declareOrGetParameter(
          node_, "narrow_passage_direction_tolerance_degrees", 20.0),
      1.0, 89.0);
  narrow_passage_width_stddev_tolerance_ = std::max(
      0.01, declareOrGetParameter(
                node_, "narrow_passage_width_stddev_tolerance", 0.15));
  narrow_passage_center_obstacle_ratio_threshold_ = std::clamp(
      declareOrGetParameter(
          node_, "narrow_passage_center_obstacle_ratio_threshold", 0.02),
      0.0, 1.0);
  debug_mode_ =
      declareOrGetParameter(node_, "front_obstacle_detector.debug_mode", false);
  enable_visualization_ = declareOrGetParameter(
      node_, "front_obstacle_detector.enable_visualization", true);

  // 初始化发布器
  if (enable_visualization_) {
    detection_area_pub_ =
        node_->create_publisher<visualization_msgs::msg::Marker>(
            "front_detection_area", 1);
    collision_point_pub_ =
        node_->create_publisher<visualization_msgs::msg::Marker>(
            "/pure_pursuit/collision_point", 10);
    collision_geometry_pub_ =
        node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/pure_pursuit/collision_geometry", rclcpp::QoS(10).reliable());
  }
  if (narrow_passage_enabled_) {
    narrow_passage_center_path_pub_ =
        node_->create_publisher<nav_msgs::msg::Path>(
            "narrow_passage_center_path", 1);
    narrow_passage_debug_marker_pub_ =
        node_->create_publisher<visualization_msgs::msg::Marker>(
            "narrow_passage_debug_marker", 1);
  }

  RCLCPP_INFO(node_->get_logger(),
              "Front Obstacle Detector initialized with parameters:");
  RCLCPP_INFO(node_->get_logger(),
              "  - Obstacle threshold: %d, trigger ratio: %.3f",
              obstacle_threshold_, obstacle_ratio_threshold_);
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
  RCLCPP_INFO(
      node_->get_logger(),
      "  - Narrow passage shadow detector: %s, range=%.2fx+/-%.2fm, "
      "bin=%.2fm, min_width=%.2fm, min_length=%.2fm",
      narrow_passage_enabled_ ? "true" : "false",
      narrow_passage_forward_range_, narrow_passage_lateral_range_,
      narrow_passage_bin_size_, robot_width_ + 2.0 * narrow_passage_margin_,
      narrow_passage_min_length_);

  return true;
}

void FrontObstacleDetector::updateCostmap(
    CostmapConstPtr costmap) {
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  costmap_ = std::move(costmap);
  ++costmap_version_;
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

void FrontObstacleDetector::setNarrowPassageReferencePath(
    const nav_msgs::msg::Path &global_path) {
  std::lock_guard<std::mutex> lock(global_path_mutex_);
  narrow_passage_reference_path_ = global_path;
  has_narrow_passage_reference_path_ =
      narrow_passage_reference_path_.poses.size() >= 2U;
}

void FrontObstacleDetector::setAvoidMaxDistance(double distance_m) {
  if (!std::isfinite(distance_m) || distance_m < 0.0) {
    RCLCPP_WARN(node_->get_logger(),
                "Ignoring invalid edge obstacle avoidance distance: %.3f",
                distance_m);
    return;
  }

  std::lock_guard<std::mutex> lock(costmap_mutex_);
  avoid_max_distance_ = distance_m;
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

  // 第一阶段影子模式：只计算并发布，不允许结果进入原有绕障决策。
  if (narrow_passage_enabled_) {
    const auto narrow_passage = detectNarrowPassageLocked(robot_pose);
    publishNarrowPassageVisualization(robot_pose, narrow_passage);
  }

  // 从车体前缘开始检查近障碍，避免把机器人自身附近的栅格纳入统计。
  // 单侧近障碍不能直接按正前方阻塞处理：先确认另一侧通道，再只允许
  // 向障碍相反方向规划。正前方或对侧通道不足时仍然强制停车。
  const double near_min_distance = std::max(0.0, robot_length_ * 0.5);
  if (avoid_min_distance_ > near_min_distance &&
      findObstacleInSectorLocal(robot_pose, near_min_distance,
                                avoid_min_distance_, avoid_sector_angle_,
                                info)) {
    info.left_clear = isPassageClear(robot_pose, true);
    info.right_clear = isPassageClear(robot_pose, false);

    const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
    const double dx = info.obstacle_center.x - robot_pose.pose.position.x;
    const double dy = info.obstacle_center.y - robot_pose.pose.position.y;
    const double obstacle_local_y =
        -dx * std::sin(robot_yaw) + dy * std::cos(robot_yaw);
    info.obstacle_local_y = obstacle_local_y;
    const bool obstacle_on_left =
        obstacle_local_y >= near_side_obstacle_lateral_threshold_;
    const bool obstacle_on_right =
        obstacle_local_y <= -near_side_obstacle_lateral_threshold_;

    if (obstacle_on_left) {
      // 贴近障碍的一侧即使外侧矩形为空，也不能作为绕行方向。
      info.left_clear = false;
      info.decision = info.right_clear ? ObstacleDecision::AVOIDABLE
                                       : ObstacleDecision::BLOCKED;
    } else if (obstacle_on_right) {
      info.right_clear = false;
      info.decision = info.left_clear ? ObstacleDecision::AVOIDABLE
                                      : ObstacleDecision::BLOCKED;
    } else {
      // 障碍位于车头中部，原地转向或向任一侧切出都可能扫碰障碍。
      info.left_clear = false;
      info.right_clear = false;
      info.decision = ObstacleDecision::BLOCKED;
    }

    if (enable_visualization_) {
      publishVisualization(robot_pose, true);
    }
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 500,
        "[AVOID_NEAR_SIDE] decision=%s dist=%.2fm local_y=%.2fm "
        "left=%d right=%d",
        decisionToString(info.decision), info.obstacle_distance,
        obstacle_local_y, info.left_clear, info.right_clear);
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
  const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
  const double dx = info.obstacle_center.x - robot_pose.pose.position.x;
  const double dy = info.obstacle_center.y - robot_pose.pose.position.y;
  info.obstacle_local_y =
      -dx * std::sin(robot_yaw) + dy * std::cos(robot_yaw);
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
        "right=%s, local_y=%.3f",
        decisionToString(info.decision), info.obstacle_center.x,
        info.obstacle_center.y, info.obstacle_distance,
        info.left_clear ? "true" : "false",
        info.right_clear ? "true" : "false", info.obstacle_local_y);
  }

  return info;
}

bool FrontObstacleDetector::isMotionCollisionFree(
    const geometry_msgs::msg::PoseStamped &robot_pose,
    const geometry_msgs::msg::Twist &velocity, double prediction_time,
    double prediction_step, double extra_margin,
    std::uint64_t *checked_version,
    const geometry_msgs::msg::Point *goal_position,
    double goal_tolerance, const std::string &check_name) const {
  std::lock_guard<std::mutex> costmap_lock(costmap_mutex_);
  if (checked_version) *checked_version = costmap_version_;
  if (!has_costmap_ || !costmap_ || costmap_->info.resolution <= 0.0 ||
      costmap_->info.width == 0 || costmap_->info.height == 0 ||
      costmap_->data.size() != static_cast<std::size_t>(costmap_->info.width) *
                                   costmap_->info.height ||
      !std::isfinite(velocity.linear.x) ||
      !std::isfinite(velocity.linear.y) ||
      !std::isfinite(velocity.angular.z) ||
      !std::isfinite(prediction_time) || prediction_time < 0.0 ||
      !std::isfinite(prediction_step) || prediction_step <= 0.0 ||
      !std::isfinite(extra_margin) || extra_margin < 0.0) {
    publishCollisionPoint(robot_pose, check_name, &robot_pose.pose.position,
                          "invalid_map_or_input", -1, 0.0);
    return false;
  }
  if (goal_position &&
      (!std::isfinite(goal_position->x) || !std::isfinite(goal_position->y) ||
       !std::isfinite(goal_tolerance) || goal_tolerance <= 0.0)) {
    publishCollisionPoint(robot_pose, check_name, &robot_pose.pose.position,
                          "invalid_goal", -1, 0.0);
    return false;
  }
  // 只有平移预测使用到点停车约束，最终纯旋转仍检查完整预测时域。
  const bool stop_at_goal = goal_position &&
      std::hypot(velocity.linear.x, velocity.linear.y) > 1e-6;

  const auto &map = *costmap_;
  const double resolution = map.info.resolution;
  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);
  const double half_length = robot_length_ * 0.5;
  // 恢复净空仅增加前后长度，左右保留正常安全边距，避免窄通道无法恢复。
  const double half_width = robot_width_ * 0.5 + safety_margin_;
  const double radius = std::hypot(half_length, half_width);
  double robot_x = robot_pose.pose.position.x;
  double robot_y = robot_pose.pose.position.y;
  const double initial_yaw = tf2::getYaw(robot_pose.pose.orientation);
  // 限制相邻姿态的最大扫掠距离，防止高速运动跨过障碍栅格。
  const double swept_speed = std::hypot(velocity.linear.x, velocity.linear.y) +
                             radius * std::abs(velocity.angular.z);
  const double step = swept_speed > 1e-6
                          ? std::min(prediction_step,
                                     resolution * 0.5 / swept_speed)
                          : prediction_step;
  const int sample_count = std::max(
      1, static_cast<int>(std::ceil(prediction_time / step)));
  double previous_time = 0.0;

  auto world_to_grid = [&](double wx, double wy, int &gx, int &gy) {
    gx = static_cast<int>(
        std::floor((wx - map.info.origin.position.x) / resolution));
    gy = static_cast<int>(
        std::floor((wy - map.info.origin.position.y) / resolution));
    return gx >= 0 && gx < width && gy >= 0 && gy < height;
  };

  for (int sample = 0; sample <= sample_count; ++sample) {
    const double previous_x = robot_x;
    const double previous_y = robot_y;
    const double time = std::min(
        prediction_time, static_cast<double>(sample) * step);
    // 恒定车体速度的精确平面运动积分，同时支持前进、横移和曲线运动。
    const double previous_yaw =
        initial_yaw + velocity.angular.z * previous_time;
    double yaw = initial_yaw + velocity.angular.z * time;
    if (std::abs(velocity.angular.z) > 1e-6) {
      robot_x += (velocity.linear.x * (std::sin(yaw) - std::sin(previous_yaw)) +
                  velocity.linear.y * (std::cos(yaw) - std::cos(previous_yaw))) /
                 velocity.angular.z;
      robot_y += (velocity.linear.x * (std::cos(previous_yaw) - std::cos(yaw)) +
                  velocity.linear.y * (std::sin(yaw) - std::sin(previous_yaw))) /
                 velocity.angular.z;
    } else {
      const double dt = time - previous_time;
      robot_x += (velocity.linear.x * std::cos(previous_yaw) -
                  velocity.linear.y * std::sin(previous_yaw)) * dt;
      robot_y += (velocity.linear.x * std::sin(previous_yaw) +
                  velocity.linear.y * std::cos(previous_yaw)) * dt;
    }
    bool goal_reached = false;
    if (stop_at_goal) {
      const double gx = previous_x - goal_position->x;
      const double gy = previous_y - goal_position->y;
      const double dx = robot_x - previous_x;
      const double dy = robot_y - previous_y;
      const double a = dx * dx + dy * dy;
      const double b = 2.0 * (gx * dx + gy * dy);
      const double c_goal = gx * gx + gy * gy -
                            goal_tolerance * goal_tolerance;
      double fraction = 1.0;
      if (c_goal <= 0.0) {
        goal_reached = true;
        fraction = 0.0;
      } else if (a > 1e-12) {
        const double discriminant = b * b - 4.0 * a * c_goal;
        if (discriminant >= 0.0) {
          const double entry = (-b - std::sqrt(discriminant)) / (2.0 * a);
          if (entry >= 0.0 && entry <= 1.0) {
            goal_reached = true;
            fraction = entry;
          }
        }
      }
      if (goal_reached) {
        // 截断在首次进入目标容差的位置，不跳到目标，也不忽略停车车体。
        robot_x = previous_x + fraction * dx;
        robot_y = previous_y + fraction * dy;
        yaw = previous_yaw +
              velocity.angular.z * (time - previous_time) * fraction;
      }
    }
    previous_time = time;
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);

    // Costmap边界外没有可验证的安全空间，按碰撞处理。
    geometry_msgs::msg::Pose collision_pose;
    collision_pose.position.x = robot_x;
    collision_pose.position.y = robot_y;
    collision_pose.position.z = robot_pose.pose.position.z;
    collision_pose.orientation.z = std::sin(yaw * 0.5);
    collision_pose.orientation.w = std::cos(yaw * 0.5);
    for (const double local_x : {-half_length, half_length}) {
      for (const double local_y : {-half_width, half_width}) {
        const double wx = robot_x + local_x * c - local_y * s;
        const double wy = robot_y + local_x * s + local_y * c;
        int gx = 0;
        int gy = 0;
        if (!world_to_grid(wx, wy, gx, gy)) {
          geometry_msgs::msg::Point point;
          point.x = wx;
          point.y = wy;
          point.z = robot_pose.pose.position.z;
          publishCollisionPoint(robot_pose, check_name, &point,
                                "out_of_map", -1, time, &collision_pose,
                                half_length, half_width);
          return false;
        }
      }
    }

    int min_gx = 0;
    int min_gy = 0;
    int max_gx = 0;
    int max_gy = 0;
    world_to_grid(robot_x - radius, robot_y - radius, min_gx, min_gy);
    world_to_grid(robot_x + radius, robot_y + radius, max_gx, max_gy);
    min_gx = std::clamp(min_gx, 0, width - 1);
    min_gy = std::clamp(min_gy, 0, height - 1);
    max_gx = std::clamp(max_gx, 0, width - 1);
    max_gy = std::clamp(max_gy, 0, height - 1);

    for (int gy = min_gy; gy <= max_gy; ++gy) {
      for (int gx = min_gx; gx <= max_gx; ++gx) {
        const double wx = map.info.origin.position.x +
                          (static_cast<double>(gx) + 0.5) * resolution;
        const double wy = map.info.origin.position.y +
                          (static_cast<double>(gy) + 0.5) * resolution;
        const double dx = wx - robot_x;
        const double dy = wy - robot_y;
        const double local_x = dx * c + dy * s;
        const double local_y = -dx * s + dy * c;
        // 将栅格半对角线纳入检查，覆盖与 footprint 边界相交的栅格。
        const double cell_padding = resolution * std::sqrt(0.5);
        if (std::abs(local_x) > half_length + cell_padding ||
            std::abs(local_y) > half_width + cell_padding) {
          continue;
        }
        const int cost = map.data[static_cast<std::size_t>(gy) *
                                      map.info.width +
                                  static_cast<std::size_t>(gx)];
        if (cost < 0 || cost >= obstacle_threshold_) {
          geometry_msgs::msg::Point point;
          point.x = wx;
          point.y = wy;
          point.z = robot_pose.pose.position.z;
          publishCollisionPoint(robot_pose, check_name, &point,
                                cost < 0 ? "unknown" : "obstacle", cost, time,
                                &collision_pose, half_length, half_width);
          return false;
        }
      }
    }
    // 上面的完整footprint检查已覆盖停车姿态；之后保持静止，不再向墙延伸。
    if (goal_reached) {
      publishCollisionPoint(robot_pose, check_name, nullptr, "", 0, time);
      return true;
    }
  }
  publishCollisionPoint(robot_pose, check_name, nullptr, "", 0, prediction_time);
  return true;
}

void FrontObstacleDetector::publishCollisionPoint(
    const geometry_msgs::msg::PoseStamped &robot_pose,
    const std::string &check_name, const geometry_msgs::msg::Point *point,
    const std::string &reason, int cost, double prediction_time,
    const geometry_msgs::msg::Pose *collision_pose,
    double half_length, double half_width) const {
  if (!collision_point_pub_) return;
  visualization_msgs::msg::Marker marker;
  marker.header = robot_pose.header;
  marker.header.stamp = node_->now();
  marker.ns = "collision_" + check_name;
  marker.id = 0;
  marker.pose.orientation.w = 1.0;
  marker.action = point ? visualization_msgs::msg::Marker::ADD
                        : visualization_msgs::msg::Marker::DELETE;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.scale.x = marker.scale.y = marker.scale.z = 0.18;
  marker.color = check_name == "output" ? color(1.0F, 0.6F, 0.0F)
                 : check_name == "recovery" ? color(1.0F, 0.0F, 1.0F)
                                           : color(1.0F, 0.0F, 0.0F);
  marker.lifetime = rclcpp::Duration::from_seconds(1.0);
  if (point) {
    marker.pose.position = *point;
    marker.pose.position.z += 0.30;
  }
  visualization_msgs::msg::MarkerArray geometry;
  geometry.markers.push_back(marker);
  // 保持原Marker话题兼容，同时用MarkerArray原子发布点和轮廓。
  collision_point_pub_->publish(marker);
  marker.id = 1;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.scale.z = 0.14;
  marker.pose.position.z += 0.22;
  marker.text = check_name + ": " + reason + " cost=" + std::to_string(cost) +
                " t=" + std::to_string(prediction_time) + "s";
  collision_point_pub_->publish(marker);

  // 独立构建轮廓，避免复用文字Marker的位姿、缩放等状态。
  visualization_msgs::msg::Marker outline;
  outline.header = marker.header;
  outline.ns = marker.ns;
  outline.id = 2;
  outline.type = visualization_msgs::msg::Marker::LINE_STRIP;
  outline.action = point && collision_pose
                      ? visualization_msgs::msg::Marker::ADD
                      : visualization_msgs::msg::Marker::DELETE;
  outline.scale.x = 0.05;
  outline.color = marker.color;
  outline.color.a = 1.0;
  outline.lifetime = rclcpp::Duration::from_seconds(1.0);
  outline.pose.orientation.w = 1.0;
  if (collision_pose) {
    outline.pose = *collision_pose;
    // 提高到机器人参考高度上方，减少被Costmap、地面点云遮挡。
    outline.pose.position.z += 0.30;
    for (int corner = 0; corner < 5; ++corner) {
      geometry_msgs::msg::Point vertex;
      vertex.x = (corner == 0 || corner == 3 || corner == 4)
                     ? half_length : -half_length;
      vertex.y = (corner == 0 || corner == 1 || corner == 4)
                     ? half_width : -half_width;
      outline.points.push_back(vertex);
    }
  }
  collision_point_pub_->publish(outline);
  geometry.markers.push_back(outline);
  if (collision_geometry_pub_) collision_geometry_pub_->publish(geometry);
}

NarrowPassageResult FrontObstacleDetector::detectNarrowPassage(
    const geometry_msgs::msg::PoseStamped &robot_pose) {
  std::lock_guard<std::mutex> costmap_lock(costmap_mutex_);
  if (!narrow_passage_enabled_ || !has_costmap_) {
    NarrowPassageResult result;
    return result;
  }
  return detectNarrowPassageLocked(robot_pose);
}

NarrowPassageResult FrontObstacleDetector::detectNarrowPassageLocked(
    const geometry_msgs::msg::PoseStamped &robot_pose) {
  NarrowPassageResult result;
  result.center_path.header.frame_id = robot_pose.header.frame_id;
  result.center_path.header.stamp = node_->now();
  result.reference_heading = tf2::getYaw(robot_pose.pose.orientation);
  if (!costmap_ || costmap_->info.resolution <= 0.0 ||
      costmap_->info.width == 0 || costmap_->info.height == 0) {
    return result;
  }

  nav_msgs::msg::Path path_snapshot;
  {
    std::lock_guard<std::mutex> path_lock(global_path_mutex_);
    if (has_narrow_passage_reference_path_) {
      path_snapshot = narrow_passage_reference_path_;
    } else if (has_global_path_ && global_path_.poses.size() >= 2U) {
      path_snapshot = global_path_;
    } else {
      return result;
    }
  }

  // 选择距机器人最近的有效路径段，以该段方向建立(s,d)局部坐标系。
  std::size_t nearest_segment = path_snapshot.poses.size();
  double nearest_distance = std::numeric_limits<double>::max();
  for (std::size_t index = 0; index + 1U < path_snapshot.poses.size(); ++index) {
    const auto &start = path_snapshot.poses[index].pose.position;
    const auto &end = path_snapshot.poses[index + 1U].pose.position;
    if (std::hypot(end.x - start.x, end.y - start.y) <= 1e-6) continue;
    const double distance = pointToSegmentDistance(
        robot_pose.pose.position.x, robot_pose.pose.position.y, start, end);
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest_segment = index;
    }
  }
  if (nearest_segment >= path_snapshot.poses.size() - 1U) return result;

  const auto &path_start =
      path_snapshot.poses[nearest_segment].pose.position;
  const auto &path_end =
      path_snapshot.poses[nearest_segment + 1U].pose.position;
  const double path_heading =
      std::atan2(path_end.y - path_start.y, path_end.x - path_start.x);
  result.reference_heading = path_heading;
  const double cos_heading = std::cos(path_heading);
  const double sin_heading = std::sin(path_heading);
  const double robot_x = robot_pose.pose.position.x;
  const double robot_y = robot_pose.pose.position.y;
  const int bin_count = std::max(
      1, static_cast<int>(std::ceil(narrow_passage_forward_range_ /
                                    narrow_passage_bin_size_)));
  result.total_bins = static_cast<std::size_t>(bin_count);
  std::vector<NarrowPassageBin> bins(static_cast<std::size_t>(bin_count));
  for (int index = 0; index < bin_count; ++index) {
    bins[static_cast<std::size_t>(index)].s =
        (static_cast<double>(index) + 0.5) * narrow_passage_bin_size_;
  }

  // 将旋转检测矩形转换为地图轴对齐包围盒，只扫描前方局部区域。
  double min_world_x = std::numeric_limits<double>::max();
  double min_world_y = std::numeric_limits<double>::max();
  double max_world_x = std::numeric_limits<double>::lowest();
  double max_world_y = std::numeric_limits<double>::lowest();
  for (const double s : {0.0, narrow_passage_forward_range_}) {
    for (const double d : {-narrow_passage_lateral_range_,
                           narrow_passage_lateral_range_}) {
      const double world_x = robot_x + s * cos_heading - d * sin_heading;
      const double world_y = robot_y + s * sin_heading + d * cos_heading;
      min_world_x = std::min(min_world_x, world_x);
      min_world_y = std::min(min_world_y, world_y);
      max_world_x = std::max(max_world_x, world_x);
      max_world_y = std::max(max_world_y, world_y);
    }
  }
  const auto worldToMapX = [&](double world_x) {
    return static_cast<int>(std::floor(
        (world_x - costmap_->info.origin.position.x) /
        costmap_->info.resolution));
  };
  const auto worldToMapY = [&](double world_y) {
    return static_cast<int>(std::floor(
        (world_y - costmap_->info.origin.position.y) /
        costmap_->info.resolution));
  };
  const int min_map_x = std::max(0, worldToMapX(min_world_x));
  const int min_map_y = std::max(0, worldToMapY(min_world_y));
  const int max_map_x = std::min(
      static_cast<int>(costmap_->info.width) - 1, worldToMapX(max_world_x) + 1);
  const int max_map_y = std::min(
      static_cast<int>(costmap_->info.height) - 1, worldToMapY(max_world_y) + 1);

  if (min_map_x > max_map_x || min_map_y > max_map_y) return result;
  for (int map_x = min_map_x; map_x <= max_map_x; ++map_x) {
    for (int map_y = min_map_y; map_y <= max_map_y; ++map_y) {
      if (getCostAtMapCoordinate(map_x, map_y) < obstacle_threshold_) continue;
      const double world_x = costmap_->info.origin.position.x +
                             (static_cast<double>(map_x) + 0.5) *
                                 costmap_->info.resolution;
      const double world_y = costmap_->info.origin.position.y +
                             (static_cast<double>(map_y) + 0.5) *
                                 costmap_->info.resolution;
      const double dx = world_x - robot_x;
      const double dy = world_y - robot_y;
      const double s = dx * cos_heading + dy * sin_heading;
      const double d = -dx * sin_heading + dy * cos_heading;
      if (s < 0.0 || s >= narrow_passage_forward_range_ ||
          std::abs(d) > narrow_passage_lateral_range_) {
        continue;
      }
      const int bin_index = std::clamp(
          static_cast<int>(s / narrow_passage_bin_size_), 0, bin_count - 1);
      auto &bin = bins[static_cast<std::size_t>(bin_index)];
      if (d > 0.0) {
        bin.left_obstacles.push_back(d);
      } else if (d < 0.0) {
        bin.right_obstacles.push_back(d);
      }
    }
  }

  // 先使用每个bin中距中心最近的障碍拟合左右边界，再按拟合边界重选。
  // 这样通道中央短箱子的点不会替代贯穿多个bin的墙体边界。
  std::vector<std::pair<double, double>> left_seeds;
  std::vector<std::pair<double, double>> right_seeds;
  for (const auto &bin : bins) {
    if (!bin.left_obstacles.empty()) {
      left_seeds.emplace_back(
          bin.s, *std::min_element(bin.left_obstacles.begin(),
                                   bin.left_obstacles.end()));
    }
    if (!bin.right_obstacles.empty()) {
      right_seeds.emplace_back(
          bin.s, *std::max_element(bin.right_obstacles.begin(),
                                   bin.right_obstacles.end()));
    }
  }
  const auto left_seed_line = fitLine(left_seeds);
  const auto right_seed_line = fitLine(right_seeds);
  const double minimum_width = robot_width_ + 2.0 * narrow_passage_margin_;
  std::vector<std::pair<double, double>> fitted_left;
  std::vector<std::pair<double, double>> fitted_right;
  std::vector<double> widths;
  for (auto &bin : bins) {
    if (bin.left_obstacles.empty() || bin.right_obstacles.empty()) continue;
    const double predicted_left =
        left_seed_line.first * bin.s + left_seed_line.second;
    const double predicted_right =
        right_seed_line.first * bin.s + right_seed_line.second;
    const auto left_it = std::min_element(
        bin.left_obstacles.begin(), bin.left_obstacles.end(),
        [&](double lhs, double rhs) {
          return std::abs(lhs - predicted_left) <
                 std::abs(rhs - predicted_left);
        });
    const auto right_it = std::min_element(
        bin.right_obstacles.begin(), bin.right_obstacles.end(),
        [&](double lhs, double rhs) {
          return std::abs(lhs - predicted_right) <
                 std::abs(rhs - predicted_right);
        });
    if (std::abs(*left_it - predicted_left) >
            narrow_passage_boundary_continuity_tolerance_ ||
        std::abs(*right_it - predicted_right) >
            narrow_passage_boundary_continuity_tolerance_) {
      continue;
    }
    bin.left_boundary = *left_it;
    bin.right_boundary = *right_it;
    bin.width = *left_it - *right_it;
    bin.center = 0.5 * (*left_it + *right_it);
    bin.valid = bin.width >= minimum_width;
    if (bin.valid) {
      fitted_left.emplace_back(bin.s, *left_it);
      fitted_right.emplace_back(bin.s, *right_it);
      widths.push_back(bin.width);
      ++result.valid_bins;
    }
  }

  std::size_t best_start = 0;
  std::size_t best_count = 0;
  std::size_t run_start = 0;
  std::size_t run_count = 0;
  for (std::size_t index = 0; index < bins.size(); ++index) {
    if (bins[index].valid) {
      if (run_count == 0) run_start = index;
      ++run_count;
      if (run_count > best_count) {
        best_start = run_start;
        best_count = run_count;
      }
    } else {
      run_count = 0;
    }
  }

  const auto left_line = fitLine(fitted_left);
  const auto right_line = fitLine(fitted_right);
  const double left_angle = std::atan(left_line.first);
  const double right_angle = std::atan(right_line.first);
  const double direction_tolerance =
      narrow_passage_direction_tolerance_degrees_ * M_PI / 180.0;
  const bool direction_consistent =
      std::abs(left_angle) <= direction_tolerance &&
      std::abs(right_angle) <= direction_tolerance &&
      std::abs(left_angle - right_angle) <= direction_tolerance;
  const double valid_ratio = bins.empty()
                                 ? 0.0
                                 : static_cast<double>(result.valid_bins) /
                                       static_cast<double>(bins.size());
  const double continuous_length =
      static_cast<double>(best_count) * narrow_passage_bin_size_;
  const double estimated_width = widths.empty() ? 0.0 : median(widths);
  double width_stddev = 0.0;
  if (!widths.empty()) {
    for (const double width : widths) {
      const double error = width - estimated_width;
      width_stddev += error * error;
    }
    width_stddev =
        std::sqrt(width_stddev / static_cast<double>(widths.size()));
  }
  const bool width_stable =
      !widths.empty() &&
      width_stddev <= narrow_passage_width_stddev_tolerance_;

  const double coverage_score = std::clamp(
      valid_ratio / std::max(0.01, narrow_passage_valid_bin_ratio_), 0.0, 1.0);
  const double length_score = std::clamp(
      continuous_length / std::max(0.01, narrow_passage_min_length_), 0.0,
      1.0);
  const bool has_boundary_evidence =
      fitted_left.size() >= 2U && fitted_right.size() >= 2U;
  const double width_score = widths.empty()
                                 ? 0.0
                                 : std::clamp(
                                       1.0 - width_stddev /
                                                 std::max(
                                                     0.01,
                                                     narrow_passage_width_stddev_tolerance_),
                                       0.0, 1.0);
  const double parallel_score =
      has_boundary_evidence
          ? std::clamp(1.0 - std::abs(left_angle - right_angle) /
                                 direction_tolerance,
                       0.0, 1.0)
          : 0.0;
  const double heading_score =
      has_boundary_evidence
          ? std::clamp(
                1.0 - std::max(std::abs(left_angle), std::abs(right_angle)) /
                          direction_tolerance,
                0.0, 1.0)
          : 0.0;
  result.confidence = 0.35 * coverage_score + 0.20 * length_score +
                      0.15 * width_score + 0.15 * parallel_score +
                      0.15 * heading_score;
  result.estimated_width = estimated_width;
  result.center_offset = 0.5 * (left_line.second + right_line.second);
  result.detected =
      best_count > 0U && continuous_length >= narrow_passage_min_length_ &&
      valid_ratio >= narrow_passage_valid_bin_ratio_ && width_stable &&
      direction_consistent &&
      result.confidence >= narrow_passage_min_confidence_;

  if (result.detected) {
    std::vector<double> centers;
    centers.reserve(best_count);
    for (std::size_t index = best_start; index < best_start + best_count;
         ++index) {
      centers.push_back(bins[index].center);
    }
    std::vector<double> smoothed_centers(centers.size(), 0.0);
    const int half_window = narrow_passage_center_smoothing_window_ / 2;
    for (std::size_t index = 0; index < centers.size(); ++index) {
      const std::size_t begin = index > static_cast<std::size_t>(half_window)
                                    ? index - half_window
                                    : 0U;
      const std::size_t end = std::min(
          centers.size(), index + static_cast<std::size_t>(half_window) + 1U);
      smoothed_centers[index] =
          std::accumulate(centers.begin() + begin, centers.begin() + end, 0.0) /
          static_cast<double>(end - begin);
    }

    result.center_path.header.frame_id =
        costmap_->header.frame_id.empty() ? robot_pose.header.frame_id
                                         : costmap_->header.frame_id;
    result.center_path.header.stamp = node_->now();
    result.center_path.poses.reserve(best_count);
    result.left_boundary.reserve(best_count);
    result.right_boundary.reserve(best_count);
    std::size_t center_blocked_bins = 0;
    const double center_half_width = minimum_width * 0.5;
    const double expected_cells_per_bin = std::max(
        1.0, narrow_passage_bin_size_ * minimum_width /
                 (costmap_->info.resolution * costmap_->info.resolution));

    for (std::size_t local_index = 0; local_index < best_count; ++local_index) {
      const std::size_t bin_index = best_start + local_index;
      const auto &bin = bins[bin_index];
      const double center_d = smoothed_centers[local_index];
      geometry_msgs::msg::Point left_point;
      geometry_msgs::msg::Point right_point;
      left_point.x = robot_x + bin.s * cos_heading -
                     *bin.left_boundary * sin_heading;
      left_point.y = robot_y + bin.s * sin_heading +
                     *bin.left_boundary * cos_heading;
      left_point.z = robot_pose.pose.position.z;
      right_point.x = robot_x + bin.s * cos_heading -
                      *bin.right_boundary * sin_heading;
      right_point.y = robot_y + bin.s * sin_heading +
                      *bin.right_boundary * cos_heading;
      right_point.z = robot_pose.pose.position.z;
      result.left_boundary.push_back(left_point);
      result.right_boundary.push_back(right_point);

      geometry_msgs::msg::PoseStamped center_pose;
      center_pose.header = result.center_path.header;
      center_pose.pose.position.x =
          robot_x + bin.s * cos_heading - center_d * sin_heading;
      center_pose.pose.position.y =
          robot_y + bin.s * sin_heading + center_d * cos_heading;
      center_pose.pose.position.z = robot_pose.pose.position.z;
      result.center_path.poses.push_back(center_pose);

      std::size_t center_obstacles = 0;
      for (const double d : bin.left_obstacles) {
        if (std::abs(d - center_d) < center_half_width) ++center_obstacles;
      }
      for (const double d : bin.right_obstacles) {
        if (std::abs(d - center_d) < center_half_width) ++center_obstacles;
      }
      const double center_obstacle_ratio =
          static_cast<double>(center_obstacles) / expected_cells_per_bin;
      if (center_obstacle_ratio >
          narrow_passage_center_obstacle_ratio_threshold_) {
        ++center_blocked_bins;
      }
    }

    for (std::size_t index = 0; index < result.center_path.poses.size();
         ++index) {
      const std::size_t other =
          index + 1U < result.center_path.poses.size() ? index + 1U
                                                      : index - (index > 0U);
      const auto &point = result.center_path.poses[index].pose.position;
      const auto &other_point = result.center_path.poses[other].pose.position;
      const double yaw =
          index + 1U < result.center_path.poses.size()
              ? std::atan2(other_point.y - point.y, other_point.x - point.x)
              : std::atan2(point.y - other_point.y, point.x - other_point.x);
      tf2::Quaternion orientation;
      orientation.setRPY(0.0, 0.0, yaw);
      result.center_path.poses[index].pose.orientation =
          tf2::toMsg(orientation);
    }
    result.center_clear = center_blocked_bins == 0U;
  }

  RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "[NARROW_PASSAGE] detected=%d center_clear=%d estimated_width=%.3f "
      "min_width=%.3f center_offset=%.3f valid_bins=%zu total_bins=%zu "
      "continuous_length=%.2f width_stddev=%.3f confidence=%.3f",
      result.detected ? 1 : 0, result.center_clear ? 1 : 0,
      result.estimated_width, minimum_width, result.center_offset,
      result.valid_bins, result.total_bins, continuous_length, width_stddev,
      result.confidence);
  return result;
}

void FrontObstacleDetector::publishNarrowPassageVisualization(
    const geometry_msgs::msg::PoseStamped &robot_pose,
    const NarrowPassageResult &result) {
  if (!narrow_passage_center_path_pub_ ||
      !narrow_passage_debug_marker_pub_) {
    return;
  }
  narrow_passage_center_path_pub_->publish(result.center_path);

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = result.center_path.header.frame_id.empty()
                               ? robot_pose.header.frame_id
                               : result.center_path.header.frame_id;
  marker.header.stamp = node_->now();
  marker.ns = "narrow_passage_shadow";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.035;
  marker.color = color(1.0F, 1.0F, 1.0F, 1.0F);

  const auto addSegment = [&](const geometry_msgs::msg::Point &start,
                              const geometry_msgs::msg::Point &end,
                              const std_msgs::msg::ColorRGBA &segment_color) {
    marker.points.push_back(start);
    marker.points.push_back(end);
    marker.colors.push_back(segment_color);
    marker.colors.push_back(segment_color);
  };
  const auto boundary_color = color(1.0F, 0.2F, 0.1F, 1.0F);
  const auto width_color = color(1.0F, 0.8F, 0.0F, 0.45F);
  const auto center_color = result.center_clear
                                ? color(0.1F, 1.0F, 0.2F, 1.0F)
                                : color(1.0F, 0.1F, 0.1F, 1.0F);
  for (std::size_t index = 1; index < result.left_boundary.size(); ++index) {
    addSegment(result.left_boundary[index - 1U], result.left_boundary[index],
               boundary_color);
    addSegment(result.right_boundary[index - 1U], result.right_boundary[index],
               boundary_color);
  }
  for (std::size_t index = 0; index < result.left_boundary.size(); ++index) {
    addSegment(result.left_boundary[index], result.right_boundary[index],
               width_color);
  }
  for (std::size_t index = 1; index < result.center_path.poses.size(); ++index) {
    addSegment(result.center_path.poses[index - 1U].pose.position,
               result.center_path.poses[index].pose.position, center_color);
  }

  // 检测区域与实际使用的全局路径局部方向保持一致。
  const double area_yaw = result.reference_heading;
  const double cos_yaw = std::cos(area_yaw);
  const double sin_yaw = std::sin(area_yaw);
  const auto areaPoint = [&](double s, double d) {
    geometry_msgs::msg::Point point;
    point.x = robot_pose.pose.position.x + s * cos_yaw - d * sin_yaw;
    point.y = robot_pose.pose.position.y + s * sin_yaw + d * cos_yaw;
    point.z = robot_pose.pose.position.z;
    return point;
  };
  const auto area_color = color(0.1F, 0.7F, 1.0F, 0.65F);
  const auto near_left = areaPoint(0.0, narrow_passage_lateral_range_);
  const auto far_left = areaPoint(narrow_passage_forward_range_,
                                  narrow_passage_lateral_range_);
  const auto far_right = areaPoint(narrow_passage_forward_range_,
                                   -narrow_passage_lateral_range_);
  const auto near_right = areaPoint(0.0, -narrow_passage_lateral_range_);
  addSegment(near_left, far_left, area_color);
  addSegment(far_left, far_right, area_color);
  addSegment(far_right, near_right, area_color);
  addSegment(near_right, near_left, area_color);
  narrow_passage_debug_marker_pub_->publish(marker);
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
    double x_max, double y_min, double y_max, double ratio_threshold) {
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
                          ratio_threshold);
  }

  return obstacle_ratio > ratio_threshold;
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
  double nearest_obstacle_distance = std::numeric_limits<double>::max();
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
        nearest_obstacle_distance = std::min(
            nearest_obstacle_distance, std::hypot(dx, dy));
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
  info.obstacle_distance = nearest_obstacle_distance;
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
  return !detectObstacleInRectLocal(
      robot_pose, x_min, x_max, y_min, y_max,
      side_passage_obstacle_ratio_threshold_);
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
