#include "pure_pursuit/local_avoidance_manager.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iterator>
#include <limits>
#include <utility>
#include <tf2/utils.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "pure_pursuit/param_utils.hpp"

namespace pure_pursuit_local_planner {

namespace {

using TimingClock = std::chrono::steady_clock;

double elapsedMs(TimingClock::time_point start) {
  return std::chrono::duration<double, std::milli>(TimingClock::now() - start)
      .count();
}

template <typename Function>
bool timedCheck(double &elapsed_ms, Function &&function) {
  const auto start = TimingClock::now();
  const bool result = function();
  elapsed_ms += elapsedMs(start);
  return result;
}

struct CandidateTiming {
  rclcpp::Logger logger;
  bool enabled;
  std::uint64_t generation;
  std::size_t batch;
  std::size_t index;
  const char *side;
  TimingClock::time_point start{TimingClock::now()};
  double astar_ms{0.0};
  double return_astar_ms{0.0};
  double safety_ms{0.0};
  double smoothing_ms{0.0};
  const char *result{"rejected"};

  ~CandidateTiming() {
    if (!enabled) return;
    RCLCPP_INFO(logger,
                "[AVOID_TIMING_CANDIDATE] task=%lu batch=%zu index=%zu "
                "side=%s result=%s total_ms=%.1f astar_ms=%.1f "
                "return_astar_ms=%.1f safety_ms=%.1f smoothing_ms=%.1f",
                static_cast<unsigned long>(generation), batch, index, side,
                result, elapsedMs(start), astar_ms, return_astar_ms,
                safety_ms, smoothing_ms);
  }
};

struct LocalPathProjection {
  nav_msgs::msg::Path remaining_path;
  std::size_t segment_index{0U};
  double distance{std::numeric_limits<double>::infinity()};
};

double pathLength(const nav_msgs::msg::Path &path) {
  double length = 0.0;
  for (std::size_t index = 1U; index < path.poses.size(); ++index) {
    const auto &previous = path.poses[index - 1U].pose.position;
    const auto &current = path.poses[index].pose.position;
    length += std::hypot(current.x - previous.x, current.y - previous.y);
  }
  return length;
}

double pathTurnAmount(const nav_msgs::msg::Path &path) {
  bool has_previous_yaw = false;
  double previous_yaw = 0.0;
  double turn = 0.0;
  for (std::size_t index = 1U; index < path.poses.size(); ++index) {
    const auto &previous = path.poses[index - 1U].pose.position;
    const auto &current = path.poses[index].pose.position;
    if (std::hypot(current.x - previous.x, current.y - previous.y) <= 1e-6) {
      continue;
    }
    const double yaw = std::atan2(current.y - previous.y,
                                  current.x - previous.x);
    if (has_previous_yaw) {
      turn += std::abs(std::atan2(std::sin(yaw - previous_yaw),
                                 std::cos(yaw - previous_yaw)));
    }
    previous_yaw = yaw;
    has_previous_yaw = true;
  }
  return turn;
}

// 从机器人在局部路径上的最近投影开始构造待复检路径。不能把机器人
// 当前位置直接回接到旧路径起点：机器人只要在对准过程中产生厘米级位移，
// 这条极短回接线的方向就可能与首段完全不同，进而被误认为需要在旧起点
// 原地锐角旋转，导致刚锁定的安全A*路径立即失效。
bool projectRemainingLocalPath(
    const geometry_msgs::msg::PoseStamped &robot_pose,
    const nav_msgs::msg::Path &selected_path,
    LocalPathProjection &result) {
  result.remaining_path = nav_msgs::msg::Path();
  result.segment_index = 0U;
  result.distance = std::numeric_limits<double>::infinity();
  result.remaining_path.header = selected_path.header;
  if (selected_path.poses.empty()) return false;

  if (selected_path.poses.size() == 1U) {
    result.remaining_path.poses.push_back(selected_path.poses.front());
    const auto &point = selected_path.poses.front().pose.position;
    result.distance = std::hypot(robot_pose.pose.position.x - point.x,
                                 robot_pose.pose.position.y - point.y);
    return std::isfinite(result.distance);
  }

  double best_ratio = 0.0;
  geometry_msgs::msg::Point best_projection;
  for (std::size_t index = 0U; index + 1U < selected_path.poses.size();
       ++index) {
    const auto &start = selected_path.poses[index].pose.position;
    const auto &end = selected_path.poses[index + 1U].pose.position;
    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const double length_squared = dx * dx + dy * dy;
    if (!std::isfinite(length_squared) || length_squared <= 1e-12) continue;

    const double ratio = std::clamp(
        ((robot_pose.pose.position.x - start.x) * dx +
         (robot_pose.pose.position.y - start.y) * dy) /
            length_squared,
        0.0, 1.0);
    geometry_msgs::msg::Point projection;
    projection.x = start.x + ratio * dx;
    projection.y = start.y + ratio * dy;
    projection.z = start.z + ratio * (end.z - start.z);
    const double distance = std::hypot(
        robot_pose.pose.position.x - projection.x,
        robot_pose.pose.position.y - projection.y);
    if (!std::isfinite(distance)) continue;

    // 自交或平行贴近路径出现等距投影时优先后面的线段，避免复检已经走过
    // 的路径；正常无回环A*路径不会受此分支影响。
    if (distance + 1e-9 < result.distance ||
        (std::abs(distance - result.distance) <= 1e-9 &&
         index > result.segment_index)) {
      result.distance = distance;
      result.segment_index = index;
      best_ratio = ratio;
      best_projection = projection;
    }
  }

  if (!std::isfinite(result.distance)) return false;

  geometry_msgs::msg::PoseStamped projection_pose =
      selected_path.poses[result.segment_index];
  projection_pose.pose.position = best_projection;
  projection_pose.pose.orientation =
      selected_path.poses[result.segment_index + 1U].pose.orientation;
  result.remaining_path.poses.push_back(std::move(projection_pose));

  // 投影恰好落在线段末端时无需重复该端点；重复点虽然几何上无害，但会
  // 让终点姿态参与不必要的旋转判断。
  std::size_t copy_start = result.segment_index + 1U;
  if (best_ratio >= 1.0 - 1e-6) ++copy_start;
  if (copy_start < selected_path.poses.size()) {
    result.remaining_path.poses.insert(
        result.remaining_path.poses.end(),
        selected_path.poses.begin() +
            static_cast<std::ptrdiff_t>(copy_start),
        selected_path.poses.end());
  }
  return true;
}

} // namespace

LocalAvoidanceManager::LocalAvoidanceManager()
    : running_(false), global_path_received_(false), costmap_received_(false),
      local_path_valid_(false), dynamic_path_mode_(false),
      path_stability_counter_(0),
      path_similarity_threshold_(0.8), planning_frequency_(1.0),
      avoidance_reentry_cooldown_(2.0), tracking_local_(false),
      failure_latched_(false), has_last_avoidance_complete_time_(false),
      avoidance_active_(false), avoidance_completed_(false),
      avoidance_stage_(AvoidanceStage::IDLE), avoidance_lateral_clearance_(0.8),
      avoidance_max_entry_angle_(40.0 * M_PI / 180.0),
      avoidance_max_return_angle_(30.0 * M_PI / 180.0),
      avoidance_goal_tolerance_(0.35),
      avoidance_return_goal_clearance_(0.8),
      avoidance_return_obstacle_clearance_(2.0),
      avoidance_obstacle_threshold_(50.0) {}

LocalAvoidanceManager::~LocalAvoidanceManager() { stop(); }

bool LocalAvoidanceManager::initialize(
    rclcpp::Node::SharedPtr node, std::shared_ptr<tf2_ros::Buffer> tf_buffer) {
  node_ = node;
  tf_buffer_ = tf_buffer;

  // 加载参数
  planning_frequency_ =
      declareOrGetParameter(node_, "local_planning_frequency", 1.0);
  avoidance_timing_enabled_ =
      declareOrGetParameter(node_, "avoidance_timing_enabled", true);
  path_similarity_threshold_ =
      declareOrGetParameter(node_, "path_similarity_threshold", 0.8);
  global_frame_ =
      declareOrGetParameter<std::string>(node_, "global_frame", "map");
  robot_frame_ =
      declareOrGetParameter<std::string>(node_, "robot_frame", "base_link");
  transform_tolerance_ =
      declareOrGetParameter(node_, "transform_tolerance", 0.1);
  avoidance_reentry_cooldown_ =
      declareOrGetParameter(node_, "avoidance_reentry_cooldown", 2.0);
  if (avoidance_reentry_cooldown_ < 0.0) {
    RCLCPP_ERROR(node_->get_logger(),
                 "avoidance_reentry_cooldown must not be negative");
    return false;
  }
  avoidance_failure_retry_cooldown_ = declareOrGetParameter(
      node_, "avoidance_failure_retry_cooldown", 2.0);
  avoidance_max_planning_failures_ = std::max(
      1, declareOrGetParameter(node_, "avoidance_max_planning_failures", 15));
  avoidance_side_goal_retry_forward_step_ = std::max(
      0.0, declareOrGetParameter(
               node_, "avoidance_side_goal_retry_forward_step", 0.25));
  avoidance_side_goal_retry_max_forward_offset_ = std::max(
      0.0, declareOrGetParameter(
               node_, "avoidance_side_goal_retry_max_forward_offset", 0.75));
  avoidance_stall_timeout_ = std::max(
      0.2, declareOrGetParameter(node_, "avoidance_stall_timeout", 1.5));
  avoidance_stall_distance_ = std::max(
      0.01, declareOrGetParameter(node_, "avoidance_stall_distance", 0.05));
  avoidance_stall_yaw_change_ = std::clamp(
      declareOrGetParameter(node_, "avoidance_stall_yaw_degrees", 5.0),
      1.0, 45.0) * M_PI / 180.0;
  local_avoidance_lookahead_distance_ = std::max(
      0.05, declareOrGetParameter(
                node_, "local_avoidance_lookahead_distance", 0.25));
  avoidance_path_blocked_confirmations_ = std::clamp(
      declareOrGetParameter(node_, "avoidance_path_blocked_confirmations", 3),
      0, 10);
  if (avoidance_failure_retry_cooldown_ < 0.0 ||
      avoidance_max_planning_failures_ < 1) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Invalid avoidance retry configuration");
    return false;
  }
  avoidance_lateral_clearance_ =
      declareOrGetParameter(node_, "avoidance_lateral_clearance", 0.8);
  avoidance_max_entry_angle_ = declareOrGetParameter(
      node_, "avoidance_max_entry_angle", 40.0) * M_PI / 180.0;
  avoidance_max_return_angle_ = declareOrGetParameter(
      node_, "avoidance_max_return_angle", 30.0) * M_PI / 180.0;
  avoidance_near_stop_clearance_ = declareOrGetParameter(
      node_, "front_obstacle_detector.avoid_min_distance", 1.0);
  avoidance_near_stop_margin_ = declareOrGetParameter(
      node_, "avoidance_near_stop_margin", 0.0);
  avoidance_path_clearance_ = declareOrGetParameter(
      node_, "avoidance_path_clearance", 0.5);
  avoidance_initial_escape_max_heading_error_ = std::clamp(
      declareOrGetParameter(
          node_, "avoidance_initial_escape_max_heading_error_degrees", 10.0),
      0.0, 30.0) * M_PI / 180.0;
  avoidance_lateral_clearance_ = std::max(0.2, avoidance_lateral_clearance_);
  avoidance_max_entry_angle_ = std::clamp(
      avoidance_max_entry_angle_, 1.0 * M_PI / 180.0, 89.0 * M_PI / 180.0);
  avoidance_max_return_angle_ = std::clamp(
      avoidance_max_return_angle_, 1.0 * M_PI / 180.0, 89.0 * M_PI / 180.0);
  avoidance_near_stop_clearance_ =
      std::max(0.0, avoidance_near_stop_clearance_);
  avoidance_near_stop_margin_ = std::max(0.0, avoidance_near_stop_margin_);
  avoidance_path_clearance_ = std::max(0.0, avoidance_path_clearance_);
  avoidance_goal_tolerance_ =
      declareOrGetParameter(node_, "avoidance_goal_tolerance", 0.35);
  const double robot_width = declareOrGetParameter(node_, "robot_width", 0.80);
  const double robot_length =
      declareOrGetParameter(node_, "robot_length", 1.10);
  const double robot_safety_margin = declareOrGetParameter(
      node_, "front_obstacle_detector.safety_margin", 0.10);
  const double default_return_goal_clearance =
      std::hypot(robot_width * 0.5, robot_length * 0.5) + robot_safety_margin;
  avoidance_return_goal_clearance_ = declareOrGetParameter(
      node_, "avoidance_return_goal_clearance", default_return_goal_clearance);
  avoidance_return_obstacle_clearance_ =
      declareOrGetParameter(node_, "avoidance_return_obstacle_clearance", 2.0);
  avoidance_obstacle_threshold_ = static_cast<double>(declareOrGetParameter(
      node_, "obstacle_cost_threshold", 50));
  avoidance_temporary_goal_retry_period_ = std::max(
      0.1, declareOrGetParameter(
               node_, "avoidance_temporary_goal_retry_period", 0.5));
  avoidance_clear_resume_path_length_ = std::max(
      0.1, declareOrGetParameter(
               node_, "avoidance_clear_resume_path_length", 2.0));
  avoidance_dynamic_side_goal_fallback_enabled_ = declareOrGetParameter(
      node_, "avoidance_dynamic_side_goal_fallback_enabled", true);
  avoidance_static_side_transition_enabled_ = declareOrGetParameter(
      node_, "avoidance_static_side_transition_enabled", true);
  avoidance_side_score_path_length_weight_ = std::max(
      0.0, declareOrGetParameter(
               node_, "avoidance_side_score_path_length_weight", 1.0));
  avoidance_side_score_heading_weight_ = std::max(
      0.0, declareOrGetParameter(
               node_, "avoidance_side_score_heading_weight", 1.5));
  avoidance_side_score_turn_weight_ = std::max(
      0.0, declareOrGetParameter(
               node_, "avoidance_side_score_turn_weight", 0.5));
  avoidance_side_score_clearance_weight_ = std::max(
      0.0, declareOrGetParameter(
               node_, "avoidance_side_score_clearance_weight", 2.0));
  avoidance_side_score_raw_path_penalty_ = std::max(
      0.0, declareOrGetParameter(
               node_, "avoidance_side_score_raw_path_penalty", 2.0));
  avoidance_side_score_switch_side_penalty_ = std::max(
      0.0, declareOrGetParameter(
               node_, "avoidance_side_score_switch_side_penalty", 0.5));
  avoidance_side_astar_candidate_limit_ = std::clamp(
      declareOrGetParameter(node_, "avoidance_side_astar_candidate_limit", 5),
      1, 20);
  avoidance_replan_max_initial_heading_error_ =
      std::clamp(
          declareOrGetParameter(
              node_, "avoidance_replan_max_initial_heading_error_degrees",
              25.0),
          1.0, 89.0) *
      M_PI / 180.0;
  avoidance_replan_heading_score_multiplier_ = std::max(
      1.0, declareOrGetParameter(
               node_, "avoidance_replan_heading_score_multiplier", 3.0));

  AvoidancePathPlannerConfig path_planner_config;
  path_planner_config.lateral_clearance = avoidance_lateral_clearance_;
  path_planner_config.return_goal_clearance = avoidance_return_goal_clearance_;
  path_planner_config.return_obstacle_clearance =
      avoidance_return_obstacle_clearance_;
  path_planner_config.temporary_goal_search_distance = std::max(
      0.1, declareOrGetParameter(
               node_, "avoidance_temporary_goal_search_distance", 6.0));
  avoidance_static_return_extended_search_distance_ = std::max(
      path_planner_config.temporary_goal_search_distance,
      declareOrGetParameter(
          node_, "avoidance_static_return_extended_search_distance", 10.0));
  path_planner_config.temporary_goal_clearance = std::max(
      0.0,
      declareOrGetParameter(node_, "avoidance_temporary_goal_clearance", 0.8));
  path_planner_config.temporary_goal_safe_path_length = std::max(
      0.05, declareOrGetParameter(
                node_, "avoidance_temporary_goal_safe_path_length", 1.0));
  path_planner_config.min_temporary_goal_distance_from_obstacle = std::max(
      0.0, declareOrGetParameter(
               node_, "avoidance_min_temporary_goal_distance_from_obstacle", 0.5));
  path_planner_config.temporary_goal_sample_spacing = std::max(
      0.005, declareOrGetParameter(
                 node_, "avoidance_temporary_goal_sample_spacing", 0.05));
  path_planner_config.side_goal_boundary_search_radius = std::max(
      0.2, declareOrGetParameter(
               node_, "avoidance_side_goal_boundary_search_radius", 1.5));
  path_planner_config.side_goal_boundary_longitudinal_band = std::max(
      0.05, declareOrGetParameter(
                node_, "avoidance_side_goal_boundary_longitudinal_band", 0.5));
  path_planner_config.side_goal_boundary_lateral_band = std::max(
      0.05, declareOrGetParameter(
                node_, "avoidance_side_goal_boundary_lateral_band", 0.5));
  path_planner_config.side_goal_min_boundary_clearance = std::max(
      0.0, declareOrGetParameter(
               node_, "avoidance_side_goal_min_boundary_clearance", 0.65));
  path_planner_config.side_goal_lateral_sample_step = std::max(
      0.0, declareOrGetParameter(
               node_, "avoidance_side_goal_lateral_sample_step", 0.15));
  path_planner_config.side_goal_lateral_sample_count = std::clamp(
      declareOrGetParameter(node_, "avoidance_side_goal_lateral_sample_count", 3),
      1, 10);
  path_planner_config.side_goal_min_forward_clearance = std::max(
      0.0, declareOrGetParameter(
               node_, "avoidance_side_goal_min_forward_clearance", 0.35));
  path_planner_config.side_goal_forward_sample_step = std::max(
      0.0, declareOrGetParameter(
               node_, "avoidance_side_goal_forward_sample_step", 0.30));
  path_planner_config.side_goal_forward_sample_count = std::clamp(
      declareOrGetParameter(node_, "avoidance_side_goal_forward_sample_count", 4),
      1, 10);
  path_planner_config.obstacle_threshold = avoidance_obstacle_threshold_;
  path_planner_config.smoothing_method = declareOrGetParameter(
      node_, "local_astar_smoothing_method", std::string("bezier"));
  path_planner_config.bezier_smoothing_enabled = declareOrGetParameter(
      node_, "local_astar_bezier_enabled", true);
  path_planner_config.bezier_corner_distance = declareOrGetParameter(
      node_, "local_astar_bezier_corner_distance", 0.80);
  path_planner_config.bezier_sample_spacing = declareOrGetParameter(
      node_, "local_astar_bezier_sample_spacing", 0.05);
  path_planner_config.bezier_simplify_tolerance = declareOrGetParameter(
      node_, "local_astar_bezier_simplify_tolerance", 0.10);
  path_planner_config.bezier_target_turn_radius = declareOrGetParameter(
      node_, "local_astar_bezier_target_turn_radius", 1.15);
  path_planner_config.bezier_min_turn_radius = declareOrGetParameter(
      node_, "local_astar_bezier_min_turn_radius", 0.30);
  path_planner_config.bezier_max_deviation = declareOrGetParameter(
      node_, "local_astar_bezier_max_deviation", 0.25);
  path_planner_config.bezier_visualization_enabled = declareOrGetParameter(
      node_, "local_astar_bezier_visualization", true);
  path_planner_config.bspline_smoothing_enabled = declareOrGetParameter(
      node_, "local_astar_bspline_enabled", true);
  path_planner_config.bspline_degree = declareOrGetParameter(
      node_, "local_astar_bspline_degree", 3);
  path_planner_config.bspline_sample_spacing = declareOrGetParameter(
      node_, "local_astar_bspline_sample_spacing", 0.05);
  path_planner_config.bspline_simplify_tolerance = declareOrGetParameter(
      node_, "local_astar_bspline_simplify_tolerance", 0.10);
  path_planner_config.bspline_control_point_spacing = declareOrGetParameter(
      node_, "local_astar_bspline_control_point_spacing", 0.70);
  path_planner_config.bspline_min_control_point_spacing = declareOrGetParameter(
      node_, "local_astar_bspline_min_control_point_spacing", 0.15);
  path_planner_config.bspline_adaptive_attempts = declareOrGetParameter(
      node_, "local_astar_bspline_adaptive_attempts", 4);
  path_planner_config.bspline_max_deviation = declareOrGetParameter(
      node_, "local_astar_bspline_max_deviation", 0.25);
  path_planner_config.bspline_min_turn_radius = declareOrGetParameter(
      node_, "local_astar_bspline_min_turn_radius", 0.30);
  path_planner_config.smoothing_start_trend_distance = declareOrGetParameter(
      node_, "local_astar_smoothing_start_trend_distance", 0.30);
  path_planner_config.bspline_start_control_blend = declareOrGetParameter(
      node_, "local_astar_bspline_start_control_blend", 0.20);
  path_planner_config.bezier_corner_distance =
      std::max(0.005, path_planner_config.bezier_corner_distance);
  path_planner_config.bezier_sample_spacing =
      std::max(0.01, path_planner_config.bezier_sample_spacing);
  path_planner_config.bezier_simplify_tolerance =
      std::max(0.0, path_planner_config.bezier_simplify_tolerance);
  path_planner_config.bezier_min_turn_radius =
      std::max(0.0, path_planner_config.bezier_min_turn_radius);
  path_planner_config.bezier_target_turn_radius = std::max(
      path_planner_config.bezier_min_turn_radius,
      path_planner_config.bezier_target_turn_radius);
  path_planner_config.bezier_max_deviation =
      std::max(0.0, path_planner_config.bezier_max_deviation);
  path_planner_config.bspline_degree =
      std::clamp(path_planner_config.bspline_degree, 2, 3);
  path_planner_config.bspline_sample_spacing =
      std::max(0.01, path_planner_config.bspline_sample_spacing);
  path_planner_config.bspline_simplify_tolerance =
      std::max(0.0, path_planner_config.bspline_simplify_tolerance);
  path_planner_config.bspline_min_control_point_spacing =
      std::max(0.05, path_planner_config.bspline_min_control_point_spacing);
  path_planner_config.bspline_control_point_spacing = std::max(
      path_planner_config.bspline_min_control_point_spacing,
      path_planner_config.bspline_control_point_spacing);
  path_planner_config.bspline_adaptive_attempts =
      std::clamp(path_planner_config.bspline_adaptive_attempts, 1, 8);
  path_planner_config.bspline_max_deviation =
      std::max(0.0, path_planner_config.bspline_max_deviation);
  path_planner_config.bspline_min_turn_radius =
      std::max(0.0, path_planner_config.bspline_min_turn_radius);
  avoidance_path_planner_ = std::make_unique<AvoidancePathPlanner>();
  if (!avoidance_path_planner_->initialize(node_, path_planner_config)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Failed to initialize avoidance path planner");
    return false;
  }

  // 初始化发布器
  local_path_pub_ =
      node_->create_publisher<nav_msgs::msg::Path>("local_path_astar", 1);
  goal_marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
      "local_goal_marker", 1);
  side_goal_candidates_pub_ =
      node_->create_publisher<visualization_msgs::msg::Marker>(
          "side_goal_candidates", rclcpp::QoS(1).transient_local());

  // 启动规划线程
  running_ = true;
  planning_thread_ = std::thread(&LocalAvoidanceManager::planningLoop, this);

  RCLCPP_INFO(node_->get_logger(),
              "Local Path Planner initialized with frequency: %.1f Hz",
              planning_frequency_);
  RCLCPP_INFO(node_->get_logger(),
              "Obstacle-based avoidance: clearance=%.2f, entry=%.1fdeg, "
              "return=%.1fdeg, "
              "tolerance=%.2f, return_goal_clearance=%.2f, "
              "return_obstacle_clearance=%.2f, near_stop_clearance=%.2f",
              avoidance_lateral_clearance_,
              avoidance_max_entry_angle_ * 180.0 / M_PI,
              avoidance_max_return_angle_ * 180.0 / M_PI,
              avoidance_goal_tolerance_, avoidance_return_goal_clearance_,
              avoidance_return_obstacle_clearance_,
              avoidance_near_stop_clearance_ + avoidance_near_stop_margin_);
  RCLCPP_INFO(node_->get_logger(),
              "Local A* smoothing: method=%s; Bezier enabled=%d corner=%.3f "
              "spacing=%.3f simplify=%.3f radius_target/min=%.2f/%.2fm "
              "max_deviation=%.3f",
              path_planner_config.smoothing_method.c_str(),
              path_planner_config.bezier_smoothing_enabled ? 1 : 0,
              path_planner_config.bezier_corner_distance,
              path_planner_config.bezier_sample_spacing,
              path_planner_config.bezier_simplify_tolerance,
              path_planner_config.bezier_target_turn_radius,
              path_planner_config.bezier_min_turn_radius,
              path_planner_config.bezier_max_deviation);
  RCLCPP_INFO(node_->get_logger(),
              "Local A* B-spline: enabled=%d degree=%d spacing=%.3f "
              "simplify=%.3f control_spacing=%.3f/%.3f attempts=%d "
              "min_radius=%.2f max_deviation=%.3f",
              path_planner_config.bspline_smoothing_enabled ? 1 : 0,
              path_planner_config.bspline_degree,
              path_planner_config.bspline_sample_spacing,
              path_planner_config.bspline_simplify_tolerance,
              path_planner_config.bspline_control_point_spacing,
              path_planner_config.bspline_min_control_point_spacing,
              path_planner_config.bspline_adaptive_attempts,
              path_planner_config.bspline_min_turn_radius,
              path_planner_config.bspline_max_deviation);
  RCLCPP_INFO(node_->get_logger(),
              "Post-obstacle temporary goal: search=%.2fm "
              "clearance=%.2fm safe_length=%.2fm min_after_obstacle=%.2fm "
              "configured_max_spacing=%.3fm retry=%.2fs "
              "static_extended_search=%.2fm",
              path_planner_config.temporary_goal_search_distance,
              path_planner_config.temporary_goal_clearance,
              path_planner_config.temporary_goal_safe_path_length,
              path_planner_config.min_temporary_goal_distance_from_obstacle,
              path_planner_config.temporary_goal_sample_spacing,
              avoidance_temporary_goal_retry_period_,
              avoidance_static_return_extended_search_distance_);
  RCLCPP_INFO(node_->get_logger(),
              "Side-goal prefilter: top %d candidates enter full A* "
              "validation; grid=%dx%d per side dynamic_fallback=%d "
              "static_transition=%d",
              avoidance_side_astar_candidate_limit_,
              path_planner_config.side_goal_lateral_sample_count,
              path_planner_config.side_goal_forward_sample_count,
              avoidance_dynamic_side_goal_fallback_enabled_ ? 1 : 0,
              avoidance_static_side_transition_enabled_ ? 1 : 0);
  return true;
}

void LocalAvoidanceManager::setGlobalPath(
    const nav_msgs::msg::Path &global_path) {
  // 该公开接口自身保证任务切换原子语义，不能依赖调用方必须先reset。
  // reset会递增generation并清除旧临时点/局部路径，使正在运行的旧搜索
  // 或A*结果无法与新全局路径组合后提交。
  reset();
  {
    std::lock_guard<std::mutex> lock(global_path_mutex_);
    global_path_ = global_path;
    global_path_received_ = !global_path.poses.empty();
  }

  if (avoidance_path_planner_ && global_path_received_) {
    avoidance_path_planner_->setGlobalPath(global_path);
    RCLCPP_DEBUG(node_->get_logger(),
                 "Global path passed to A* planner with %zu poses",
                 global_path.poses.size());
  }

  if (global_path_received_) {
    RCLCPP_DEBUG(node_->get_logger(),
                 "Local planner received global path with %zu poses",
                 global_path.poses.size());
  }

  // 动态目标发生显著变化时，立即让规划线程观察新的任务generation。
  // reset已经清除了旧局部路径，因此这里只唤醒，不会沿用旧临时点。
  requestPlanningWakeup();
}

void LocalAvoidanceManager::setDynamicPathMode(bool enabled) {
  dynamic_path_mode_.store(enabled);
}

void LocalAvoidanceManager::updateCostmap(
    CostmapConstPtr costmap) {
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    costmap_ = costmap;
    costmap_received_ = static_cast<bool>(costmap_);
  }

  if (avoidance_path_planner_) {
    avoidance_path_planner_->updateCostmap(std::move(costmap));

    // 仅在规划器尚无可用地图时立即准备首张Costmap。已有地图后，普通
    // Costmap帧仍按local_planning_frequency复核，避免跟踪中反复唤醒。
    if (!avoidance_path_planner_->hasPreparedCostmap()) {
      requestPlanningWakeup();
    }
  }
}

nav_msgs::msg::Path LocalAvoidanceManager::getLocalPath() const {
  std::lock_guard<std::mutex> lock(path_mutex_);
  return local_path_;
}

std::optional<nav_msgs::msg::Path>
LocalAvoidanceManager::getValidLocalPath() const {
  std::lock_guard<std::mutex> lock(path_mutex_);
  if (!local_path_valid_ || local_path_.poses.empty()) {
    return std::nullopt;
  }
  return local_path_;
}

LocalAvoidanceOutput LocalAvoidanceManager::update(
    const geometry_msgs::msg::PoseStamped &robot_pose,
    const AvoidanceObservation &observation, bool avoidance_enabled,
    bool global_controller_aligning, bool safety_stop_blocking,
    std::size_t global_path_progress_index) {
  std::lock_guard<std::mutex> decision_lock(decision_mutex_);
  current_global_path_progress_index_.store(global_path_progress_index);
  LocalAvoidanceOutput output;

  RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 500,
      "[AVOID_TRACE] decision=%d enabled=%d tracking=%d aligning=%d "
      "obstacle_dist=%.3f left=%d right=%d costmap=%d prepared=%d "
      "failure_latched=%d",
      static_cast<int>(observation.decision), avoidance_enabled,
      tracking_local_, global_controller_aligning, observation.obstacle_distance,
      observation.left_clear, observation.right_clear,
      costmap_received_.load(),
      avoidance_path_planner_ && avoidance_path_planner_->hasPreparedCostmap(),
      failure_latched_);

  auto finish = [this, &output]() {
    output.realign_local_path = stage_alignment_requested_.exchange(false);
    output.tracking_local = tracking_local_;
    output.failure_latched = failure_latched_;
    {
      // 与规划线程使用相同的avoidance->path锁顺序，一次取得同代状态和
      // 路径，避免输出一帧“新stage配旧path”或相反的交叉快照。
      std::lock_guard<std::mutex> avoidance_lock(avoidance_mutex_);
      output.stage = avoidance_stage_;
      output.task_generation = avoidance_task_generation_;
      if (avoidance_temporary_goal_locked_) {
        if (avoidance_temporary_goal_on_global_path_) {
          output.temporary_goal_index = avoidance_temporary_goal_index_;
        }
        output.temporary_goal = avoidance_temporary_goal_;
      }
      if (tracking_local_ &&
          output.mode == AvoidanceTrackingMode::GLOBAL_PATH) {
        std::lock_guard<std::mutex> path_lock(path_mutex_);
        if (local_path_valid_ && !local_path_.poses.empty()) {
          output.local_path = local_path_;
          output.mode = AvoidanceTrackingMode::LOCAL_PATH;
        } else {
          output.mode = AvoidanceTrackingMode::WAITING_LOCAL_PATH;
        }
      }
    }
    return output;
  };

  if (observation.decision != ObstacleDecision::CLEAR) {
    search_clear_seen_time_ = TimingClock::time_point{};
  }

  if (!avoidance_enabled) {
    stall_monitoring_ = false;
    RCLCPP_DEBUG_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "[AVOID_SKIP] obstacle avoidance disabled at current path index");
    if (tracking_local_ || isWaitingForDynamicPath()) {
      tracking_local_ = false;
      output.exited_local_tracking = true;
      resetAvoidance();
    }
    return finish();
  }

  // 动态路径已经绕过障碍后保持停车，直到上层收到一条更新鲜的动态路径
  // 并把它作为新任务设置。不能在此阶段重新检测并启动绕障。
  if (isWaitingForDynamicPath()) {
    stall_monitoring_ = false;
    if (tracking_local_) {
      tracking_local_ = false;
      output.exited_local_tracking = true;
    }
    output.mode = AvoidanceTrackingMode::WAITING_DYNAMIC_PATH;
    return finish();
  }

  // 没有安全绕障临时点时保持任务和全局路径不变，显式停车等待地图刷新。
  // 只有检测已经CLEAR才取消等待并恢复全局路径。
  if (isWaitingForTemporaryGoal()) {
    stall_monitoring_ = false;
    if (observation.decision == ObstacleDecision::CLEAR) {
      const std::uint64_t prepared_version =
          avoidance_path_planner_
              ? avoidance_path_planner_->preparedCostmapVersion()
              : 0U;
      if (prepared_version != 0U &&
          prepared_version != last_waiting_clear_costmap_version_) {
        last_waiting_clear_costmap_version_ = prepared_version;
        ++consecutive_waiting_clear_checks_;
      }
      if (consecutive_waiting_clear_checks_ >=
          avoidance_path_blocked_confirmations_) {
        tracking_local_ = false;
        output.exited_local_tracking = true;
        if (dynamic_path_mode_.load()) {
          enterWaitingForDynamicPath();
          output.mode = AvoidanceTrackingMode::WAITING_DYNAMIC_PATH;
          RCLCPP_INFO(
              node_->get_logger(),
              "[TEMP_GOAL_WAIT] obstacle clear on %d distinct Costmaps; "
              "waiting for a fresh dynamic path",
              avoidance_path_blocked_confirmations_);
        } else {
          resetAvoidance();
          RCLCPP_INFO(
              node_->get_logger(),
              "[TEMP_GOAL_WAIT] obstacle clear on %d distinct Costmaps; "
              "resume global path",
              avoidance_path_blocked_confirmations_);
        }
        return finish();
      }
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 500,
          "[TEMP_GOAL_CLEAR_WAIT] clear=%d/%d version=%lu; keep stopped",
          consecutive_waiting_clear_checks_,
          avoidance_path_blocked_confirmations_,
          static_cast<unsigned long>(prepared_version));
    } else {
      consecutive_waiting_clear_checks_ = 0;
      last_waiting_clear_costmap_version_ = 0U;

      // 前方检测仍明确有障碍时，不能因为全局路径栅格扫描暂时返回
      // NO_BLOCKED_PATH而停在WAITING状态。用最新障碍位置重新生成左右
      // 侧方候选，候选仍需通过完整A*、footprint和平滑后碰撞检查。
      const bool side_escape_supported =
          (dynamic_path_mode_.load() &&
           avoidance_dynamic_side_goal_fallback_enabled_) ||
          (!dynamic_path_mode_.load() &&
           avoidance_static_side_transition_enabled_);
      bool side_escape_requested = false;
      if (side_escape_supported) {
        {
          std::lock_guard<std::mutex> avoidance_lock(avoidance_mutex_);
          if (avoidance_active_ &&
              avoidance_stage_ == AvoidanceStage::WAITING_TEMPORARY_GOAL &&
              std::chrono::steady_clock::now() >=
                  next_temporary_goal_retry_time_) {
            avoidance_obstacle_center_ = observation.obstacle_center;
            avoidance_left_clear_at_trigger_ = observation.left_clear;
            avoidance_right_clear_at_trigger_ = observation.right_clear;
            if (observation.left_clear && !observation.right_clear) {
              avoidance_preferred_side_ = AvoidanceSide::LEFT;
            } else if (observation.right_clear && !observation.left_clear) {
              avoidance_preferred_side_ = AvoidanceSide::RIGHT;
            } else {
              // 两侧快速判定都不通过时仍允许两侧进入候选生成；最终
              // 是否可走由footprint和A*决定。优先选择障碍物反侧。
              avoidance_preferred_side_ =
                  observation.obstacle_local_y >= 0.0
                      ? AvoidanceSide::RIGHT
                      : AvoidanceSide::LEFT;
            }
            avoidance_alternate_side_enabled_ = true;
            avoidance_force_side_fallback_ = true;
            avoidance_stage_ = AvoidanceStage::SEARCHING_TEMPORARY_GOAL;
            next_temporary_goal_retry_time_ =
                std::chrono::steady_clock::now();
            ++avoidance_task_generation_;
            side_escape_requested = true;
          }
        }
        if (side_escape_requested) {
          RCLCPP_WARN(
              node_->get_logger(),
              "[TEMP_GOAL_SIDE_ESCAPE] detector remains blocked at "
              "(%.3f,%.3f), leave WAITING and search both sides",
              observation.obstacle_center.x, observation.obstacle_center.y);
          requestPlanningWakeup();
          output.mode = AvoidanceTrackingMode::WAITING_LOCAL_PATH;
          return finish();
        }
      }
    }
    output.mode = AvoidanceTrackingMode::STOPPED_BLOCKED;
    return finish();
  }

  // 搜索线程可能还在逐批尝试侧方候选。原障碍已消失且原全局路径的
  // footprint走廊重新安全时，直接撤销搜索任务；generation会使旧A*结果
  // 无法提交，避免继续等待剩余候选全部失败。
  if (tracking_local_ && observation.decision == ObstacleDecision::CLEAR) {
    bool searching_temporary_goal = false;
    {
      std::lock_guard<std::mutex> lock(avoidance_mutex_);
      searching_temporary_goal =
          avoidance_active_ &&
          avoidance_stage_ == AvoidanceStage::SEARCHING_TEMPORARY_GOAL;
    }
    if (searching_temporary_goal) {
      if (search_clear_seen_time_ == TimingClock::time_point{}) {
        search_clear_seen_time_ = TimingClock::now();
      }
      const auto clear_check_start = TimingClock::now();
      const bool original_path_clear = isGlobalPathClearForResume(
          robot_pose, global_path_progress_index);
      if (avoidance_timing_enabled_) {
        RCLCPP_INFO_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 500,
            "[AVOID_TIMING_CLEAR_CHECK] safe=%d elapsed_ms=%.1f "
            "since_clear_ms=%.1f",
            original_path_clear, elapsedMs(clear_check_start),
            elapsedMs(search_clear_seen_time_));
      }
      if (original_path_clear) {
        const double clear_to_resume_ms = elapsedMs(search_clear_seen_time_);
        tracking_local_ = false;
        output.exited_local_tracking = true;
        resetAvoidance();
        RCLCPP_INFO(node_->get_logger(),
                    "[TEMP_GOAL_CLEAR_CANCEL] original path is "
                    "collision-free for up to %.2fm; cancel side-goal search "
                    "and resume global path (clear_to_resume_ms=%.1f)",
                    avoidance_clear_resume_path_length_,
                    clear_to_resume_ms);
        search_clear_seen_time_ = TimingClock::time_point{};
        return finish();
      }
      RCLCPP_INFO_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "[TEMP_GOAL_CLEAR_CHECK] detector clear, but original path "
          "is not yet collision-free; keep searching");
    }
  }

  // 失败后短暂停车，让Costmap有时间刷新。即使障碍没有完全消失也会
  // 自动重新尝试，避免机器人静止后永远等不到CLEAR的状态机死锁。
  if (failure_latched_) {
    if (observation.decision == ObstacleDecision::CLEAR) {
      failure_latched_ = false;
      has_failure_latch_time_ = false;
      retry_attempt_ = 0;
      output.failure_changed = true;
      RCLCPP_INFO(node_->get_logger(),
                  "[OBSTACLE_REPLAN] obstacle cleared, failure latch reset");
    } else if (has_failure_latch_time_ &&
               (node_->now() - failure_latch_time_).seconds() >=
                   avoidance_failure_retry_cooldown_) {
      failure_latched_ = false;
      has_failure_latch_time_ = false;
      ++retry_attempt_;
      output.failure_changed = true;
      RCLCPP_WARN(node_->get_logger(),
                  "[AVOID_RETRY] failure cooldown elapsed, retry=%d",
                  retry_attempt_);
    } else {
      output.mode = AvoidanceTrackingMode::FAILED;
      return finish();
    }
  }

  // 全局路径预对准期间仍允许前方检测启动A*。动态目标大角度变化时，
  // 原地旋转可能把近障碍扫入车头；若在此处跳过，通常要到硬停距离内
  // 才会开始绕障。
  (void)global_controller_aligning;

  // obstacle_stop持续拦截且机器人没有有效移动时，当前局部路径不可执行。
  // 若正在执行明确的侧绕路径，开放另一侧并优先重选；否则保留锁定临时
  // 点并要求规划线程复核。所有新路径仍必须通过完整A*和footprint校验。
  if (!tracking_local_ || !safety_stop_blocking) {
    stall_monitoring_ = false;
  } else if (!stall_monitoring_) {
    stall_monitoring_ = true;
    stall_start_time_ = node_->now();
    stall_start_position_ = robot_pose.pose.position;
    stall_start_yaw_ = tf2::getYaw(robot_pose.pose.orientation);
  } else {
    const double moved = calculateDistance(
        robot_pose.pose.position, stall_start_position_);
    const double current_yaw = tf2::getYaw(robot_pose.pose.orientation);
    const double yaw_change = std::abs(std::atan2(
        std::sin(current_yaw - stall_start_yaw_),
        std::cos(current_yaw - stall_start_yaw_)));
    if (moved >= avoidance_stall_distance_ ||
        yaw_change >= avoidance_stall_yaw_change_) {
      stall_start_time_ = node_->now();
      stall_start_position_ = robot_pose.pose.position;
      stall_start_yaw_ = current_yaw;
    } else if ((node_->now() - stall_start_time_).seconds() >=
               avoidance_stall_timeout_) {
      bool replan_requested = false;
      bool opposite_side_reselection = false;
      AvoidanceSide old_side = AvoidanceSide::NONE;
      AvoidanceSide next_side = AvoidanceSide::NONE;
      {
        std::lock_guard<std::mutex> lock(avoidance_mutex_);
        if (avoidance_active_ && avoidance_temporary_goal_locked_ &&
            avoidance_stage_ == AvoidanceStage::TRACKING_TEMPORARY_GOAL) {
          ++avoidance_task_generation_;
          avoidance_corridor_selected_ = false;
          avoidance_temporary_path_.poses.clear();
          old_side = avoidance_selected_side_;
          const bool side_reselection_supported =
              old_side != AvoidanceSide::NONE &&
              ((dynamic_path_mode_.load() &&
                avoidance_dynamic_side_goal_fallback_enabled_) ||
               (!dynamic_path_mode_.load() &&
                avoidance_static_side_transition_enabled_));
          if (side_reselection_supported) {
            avoidance_alternate_side_enabled_ = true;
            next_side = old_side == AvoidanceSide::LEFT
                            ? AvoidanceSide::RIGHT
                            : AvoidanceSide::LEFT;
            avoidance_preferred_side_ = next_side;
            avoidance_heading_continuity_required_ = true;
            if (avoidance_temporary_goal_on_global_path_) {
              avoidance_retained_static_return_valid_ = true;
              avoidance_retained_static_return_goal_ =
                  avoidance_temporary_goal_;
              avoidance_retained_static_return_index_ =
                  avoidance_temporary_goal_index_;
            }
            avoidance_temporary_goal_locked_ = false;
            avoidance_temporary_goal_ = geometry_msgs::msg::PoseStamped();
            avoidance_temporary_goal_index_ =
                std::numeric_limits<std::size_t>::max();
            avoidance_temporary_goal_on_global_path_ = false;
            avoidance_force_side_fallback_ = true;
            avoidance_stage_ = AvoidanceStage::SEARCHING_TEMPORARY_GOAL;
            temporary_goal_revalidation_required_ = false;
            opposite_side_reselection = true;
          } else {
            temporary_goal_revalidation_required_ = true;
          }
          consecutive_planning_failures_ = 0;
          next_temporary_goal_retry_time_ =
              std::chrono::steady_clock::now();
          replan_requested = true;
        }
      }
      RCLCPP_WARN(
          node_->get_logger(),
          "[TEMP_PATH_STALL_REPLAN] obstacle_stop blocked local motion for "
          "%.2fs (moved=%.3fm yaw_change=%.1fdeg); "
          "replan_same_temporary_goal=%d switch_side=%d old_side=%s "
          "next_preferred=%s",
          avoidance_stall_timeout_, moved, yaw_change * 180.0 / M_PI,
          replan_requested && !opposite_side_reselection ? 1 : 0,
          opposite_side_reselection ? 1 : 0,
          avoidanceSideToString(old_side), avoidanceSideToString(next_side));
      stall_monitoring_ = false;
      if (replan_requested) {
        clearLocalPath();
        requestPlanningWakeup();
        output.mode = AvoidanceTrackingMode::WAITING_LOCAL_PATH;
        return finish();
      }
    }
  }

  if (tracking_local_) {
    // 规划线程可能异步进入FAILED，必须优先于BLOCKED处理并向上层锁存。
    if (isAvoidanceFailed()) {
      failure_latched_ = true;
      failure_latch_time_ = node_->now();
      has_failure_latch_time_ = true;
      tracking_local_ = false;
      output.failure_changed = true;
      output.exited_local_tracking = true;
      output.mode = AvoidanceTrackingMode::FAILED;
      resetAvoidance();
      RCLCPP_ERROR(node_->get_logger(),
                   "[OBSTACLE_REPLAN] avoidance failed, navigation stopped");
      return finish();
    }

    // 完成状态必须先于BLOCKED处理。规划线程到达回归点后会清空局部
    // 路径；如果此时原障碍仍在检测扇区内，先检查BLOCKED会把状态永远
    // 留在WAITING_LOCAL_PATH。
    if (isAvoidanceCompleted()) {
      if (dynamic_path_mode_.load()) {
        last_avoidance_complete_time_ = node_->now();
        has_last_avoidance_complete_time_ = true;
        retry_attempt_ = 0;
        tracking_local_ = false;
        output.exited_local_tracking = true;
        output.mode = AvoidanceTrackingMode::WAITING_DYNAMIC_PATH;
        enterWaitingForDynamicPath();
        RCLCPP_INFO(node_->get_logger(),
                    "[OBSTACLE_REPLAN] dynamic avoidance completed; "
                    "waiting for a fresh UWB path");
        return finish();
      }
      {
        std::lock_guard<std::mutex> lock(avoidance_mutex_);
        if (avoidance_temporary_goal_on_global_path_ &&
            avoidance_temporary_goal_index_ !=
            std::numeric_limits<std::size_t>::max()) {
          output.resume_global_path_index = avoidance_temporary_goal_index_;
        }
      }
      last_avoidance_complete_time_ = node_->now();
      has_last_avoidance_complete_time_ = true;
      retry_attempt_ = 0;
      tracking_local_ = false;
      output.exited_local_tracking = true;
      resetAvoidance();
      RCLCPP_INFO(node_->get_logger(),
                  "[OBSTACLE_REPLAN] avoidance completed, back to global path");
      return finish();
    }

    if (observation.decision == ObstacleDecision::BLOCKED) {
      // 局部路径已经通过A*的footprint和连续碰撞检查；此时检测器看到的
      // 仍可能是绕障任务中的原障碍物，不能用全局前方停车半径打断侧向路径。
      // 仅当局部路径已经失效时才停车，避免机器人永远到不了障碍侧方。
      const auto local_path = getValidLocalPath();
      if (!local_path || local_path->poses.empty()) {
        output.mode = AvoidanceTrackingMode::WAITING_LOCAL_PATH;
        RCLCPP_DEBUG_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 500,
            "[AVOID_WAIT] obstacle BLOCKED while local path is being planned; "
            "dist=%.3f",
            observation.obstacle_distance);
        return finish();
      }

      output.local_path = local_path;
      output.mode = AvoidanceTrackingMode::LOCAL_PATH;
      RCLCPP_DEBUG_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 500,
          "[AVOID_CONTINUE] local path remains active while front detector "
          "reports BLOCKED, dist=%.3f",
          observation.obstacle_distance);
    }

  }

  if (!tracking_local_) {
    const bool in_cooldown =
        has_last_avoidance_complete_time_ &&
        (node_->now() - last_avoidance_complete_time_).seconds() <
            avoidance_reentry_cooldown_;
    if (in_cooldown) {
      return finish();
    }

    if (observation.decision == ObstacleDecision::AVOIDABLE ||
        observation.decision == ObstacleDecision::BLOCKED) {
      // 首张Costmap可能已经到达检测器，但A*的完整地图/footprint快照仍在
      // 规划线程中准备。此时只停车等待，不退回左右侧方点。
      if (!avoidance_path_planner_ ||
          !avoidance_path_planner_->hasPreparedCostmap()) {
        output.mode = AvoidanceTrackingMode::WAITING_COSTMAP;
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "[AVOID_WAIT] A* not started: planner=%d costmap_received=%d "
            "prepared_costmap=%d",
            avoidance_path_planner_ ? 1 : 0, costmap_received_.load(),
            avoidance_path_planner_ &&
                avoidance_path_planner_->hasPreparedCostmap());
        return finish();
      }

      RCLCPP_INFO(
          node_->get_logger(),
          "[TEMP_GOAL_START] trigger accepted: decision=%d dist=%.3f "
          "left=%d right=%d (dynamic short-path fallback may use clear side)",
          static_cast<int>(observation.decision),
          observation.obstacle_distance,
          observation.left_clear, observation.right_clear);

      if (!startAvoidance(robot_pose, observation)) {
        output.mode = AvoidanceTrackingMode::STOPPED_BLOCKED;
        RCLCPP_WARN(node_->get_logger(),
                    "[TEMP_GOAL_WAIT] could not start search; stopping");
        return finish();
      }

      tracking_local_ = true;
      output.entered_local_tracking = true;
      RCLCPP_INFO(node_->get_logger(),
                  "[TEMP_GOAL_SEARCH] task started; waiting for one-segment "
                  "local A* path");
      return finish();
    }
  }

  return finish();
}

bool LocalAvoidanceManager::isGlobalPathClearForResume(
    const geometry_msgs::msg::PoseStamped &robot_pose,
    std::size_t global_path_progress_index) {
  if (!avoidance_path_planner_ ||
      !avoidance_path_planner_->hasPreparedCostmap()) {
    return false;
  }

  nav_msgs::msg::Path global_path_copy;
  {
    std::lock_guard<std::mutex> lock(global_path_mutex_);
    global_path_copy = global_path_;
  }
  if (global_path_copy.poses.size() < 2U) return false;

  nav_msgs::msg::Path resume_path;
  resume_path.header = global_path_copy.header;
  resume_path.poses.push_back(robot_pose);
  double checked_length = 0.0;
  const std::size_t first_index =
      std::min(global_path_progress_index,
               global_path_copy.poses.size() - 2U) + 1U;
  for (std::size_t index = first_index;
       index < global_path_copy.poses.size() &&
       checked_length < avoidance_clear_resume_path_length_;
       ++index) {
    const auto &previous = resume_path.poses.back().pose.position;
    const auto &next_pose = global_path_copy.poses[index];
    const auto &next = next_pose.pose.position;
    const double segment_length =
        std::hypot(next.x - previous.x, next.y - previous.y);
    if (!std::isfinite(segment_length)) return false;
    if (segment_length < 1e-3) continue;

    if (checked_length + segment_length >
        avoidance_clear_resume_path_length_) {
      const double ratio =
          (avoidance_clear_resume_path_length_ - checked_length) /
          segment_length;
      auto end_pose = next_pose;
      end_pose.pose.position.x = previous.x + ratio * (next.x - previous.x);
      end_pose.pose.position.y = previous.y + ratio * (next.y - previous.y);
      end_pose.pose.position.z =
          previous.z + ratio * (next.z - previous.z);
      resume_path.poses.push_back(std::move(end_pose));
      break;
    }
    resume_path.poses.push_back(next_pose);
    checked_length += segment_length;
  }
  if (resume_path.poses.size() < 2U) return false;

  const auto &start = resume_path.poses[0].pose.position;
  const auto &first = resume_path.poses[1].pose.position;
  const double first_yaw = std::atan2(first.y - start.y, first.x - start.x);
  if (!avoidance_path_planner_->isRotationCollisionFree(
          start, tf2::getYaw(robot_pose.pose.orientation), first_yaw)) {
    return false;
  }
  return avoidance_path_planner_->isPathMotionCollisionFree(resume_path);
}

void LocalAvoidanceManager::reset() {
  std::lock_guard<std::mutex> decision_lock(decision_mutex_);
  tracking_local_ = false;
  failure_latched_ = false;
  has_failure_latch_time_ = false;
  retry_attempt_ = 0;
  has_last_avoidance_complete_time_ = false;
  stall_monitoring_ = false;
  search_clear_seen_time_ = TimingClock::time_point{};
  consecutive_waiting_clear_checks_ = 0;
  last_waiting_clear_costmap_version_ = 0U;
  stage_alignment_requested_.store(false);
  resetAvoidance();
}

bool LocalAvoidanceManager::isTrackingLocal() const {
  std::lock_guard<std::mutex> decision_lock(decision_mutex_);
  return tracking_local_;
}

bool LocalAvoidanceManager::startAvoidance(
    const geometry_msgs::msg::PoseStamped &robot_pose,
    const AvoidanceObservation &observation) {
  if (!global_path_received_ || !costmap_received_ ||
      !avoidance_path_planner_ ||
      !avoidance_path_planner_->hasPreparedCostmap()) {
    RCLCPP_WARN(
        node_->get_logger(),
        "[AVOID_FAIL] A* not started: global_path=%d costmap_received=%d "
        "planner=%d prepared_costmap=%d",
        global_path_received_.load(), costmap_received_.load(),
        avoidance_path_planner_ ? 1 : 0,
        avoidance_path_planner_ &&
            avoidance_path_planner_->hasPreparedCostmap());
    return false;
  }

  std::size_t global_path_size = 0U;
  {
    std::lock_guard<std::mutex> lock(global_path_mutex_);
    global_path_size = global_path_.poses.size();
  }
  if (global_path_size < 2U) {
    RCLCPP_WARN(node_->get_logger(),
                "[TEMP_GOAL_SEARCH] global path has fewer than two poses");
    return false;
  }

  const std::size_t search_start = std::min(
      current_global_path_progress_index_.load(), global_path_size - 1U);
  search_clear_seen_time_ = TimingClock::time_point{};
  consecutive_waiting_clear_checks_ = 0;
  last_waiting_clear_costmap_version_ = 0U;
  {
    std::lock_guard<std::mutex> lock(avoidance_mutex_);
    ++avoidance_task_generation_;
    avoidance_active_ = true;
    avoidance_trigger_time_ = TimingClock::now();
    avoidance_completed_ = false;
    avoidance_stage_ = AvoidanceStage::SEARCHING_TEMPORARY_GOAL;
    avoidance_obstacle_center_ = observation.obstacle_center;
    avoidance_temporary_goal_ = geometry_msgs::msg::PoseStamped();
    avoidance_temporary_goal_index_ =
        std::numeric_limits<std::size_t>::max();
    avoidance_search_start_index_ = search_start;
    avoidance_obstacle_search_start_index_ = search_start;
    avoidance_minimum_goal_index_ = 0U;
    avoidance_temporary_goal_locked_ = false;
    avoidance_temporary_goal_on_global_path_ = false;
    avoidance_force_side_fallback_ = false;
    avoidance_retained_static_return_valid_ = false;
    avoidance_retained_static_return_goal_ =
        geometry_msgs::msg::PoseStamped();
    avoidance_retained_static_return_index_ =
        std::numeric_limits<std::size_t>::max();
    avoidance_heading_continuity_required_ = false;
    avoidance_selected_side_ = AvoidanceSide::NONE;
    avoidance_alternate_side_enabled_ = false;
    avoidance_left_clear_at_trigger_ = observation.left_clear;
    avoidance_right_clear_at_trigger_ = observation.right_clear;
    if (observation.left_clear && !observation.right_clear) {
      avoidance_preferred_side_ = AvoidanceSide::LEFT;
    } else if (observation.right_clear && !observation.left_clear) {
      avoidance_preferred_side_ = AvoidanceSide::RIGHT;
    } else if (observation.left_clear && observation.right_clear) {
      avoidance_preferred_side_ = observation.obstacle_local_y >= 0.0
                                      ? AvoidanceSide::RIGHT
                                      : AvoidanceSide::LEFT;
    } else {
      avoidance_preferred_side_ = AvoidanceSide::NONE;
    }
    temporary_goal_revalidation_required_ = false;
    avoidance_corridor_selected_ = false;
    consecutive_planning_failures_ = 0;
    temporary_goal_reselection_count_ = 0;
    avoidance_side_candidate_retry_batch_ = 0U;
    consecutive_temporary_goal_invalid_checks_ = 0;
    consecutive_blocked_path_checks_ = 0;
    avoidance_temporary_path_.poses.clear();
    last_validated_costmap_version_ = 0U;
    last_path_checked_costmap_version_ = 0U;
    last_retry_costmap_version_ = 0U;
    temporary_goal_last_validation_valid_ = false;
    next_temporary_goal_retry_time_ = std::chrono::steady_clock::now();
  }

  clearLocalPath();
  clearAvoidanceMarkers();
  RCLCPP_INFO(node_->get_logger(),
              "[TEMP_GOAL_SEARCH] started: progress=%zu robot=(%.3f,%.3f) "
              "obstacle=(%.3f,%.3f)",
              search_start, robot_pose.pose.position.x,
              robot_pose.pose.position.y, observation.obstacle_center.x,
              observation.obstacle_center.y);
  requestPlanningWakeup();
  return true;
}

void LocalAvoidanceManager::resetAvoidance() {
  stage_alignment_requested_.store(false);
  {
    std::lock_guard<std::mutex> avoidance_lock(avoidance_mutex_);
    avoidance_active_ = false;
    avoidance_trigger_time_ = TimingClock::time_point{};
    avoidance_completed_ = false;
    avoidance_stage_ = AvoidanceStage::IDLE;
    avoidance_obstacle_center_ = geometry_msgs::msg::Point();
    avoidance_temporary_goal_ = geometry_msgs::msg::PoseStamped();
    avoidance_temporary_goal_index_ =
        std::numeric_limits<std::size_t>::max();
    avoidance_search_start_index_ = 0U;
    avoidance_obstacle_search_start_index_ = 0U;
    avoidance_minimum_goal_index_ = 0U;
    avoidance_temporary_goal_locked_ = false;
    avoidance_temporary_goal_on_global_path_ = false;
    avoidance_force_side_fallback_ = false;
    avoidance_retained_static_return_valid_ = false;
    avoidance_retained_static_return_goal_ =
        geometry_msgs::msg::PoseStamped();
    avoidance_retained_static_return_index_ =
        std::numeric_limits<std::size_t>::max();
    avoidance_heading_continuity_required_ = false;
    avoidance_preferred_side_ = AvoidanceSide::NONE;
    avoidance_selected_side_ = AvoidanceSide::NONE;
    avoidance_alternate_side_enabled_ = false;
    avoidance_left_clear_at_trigger_ = false;
    avoidance_right_clear_at_trigger_ = false;
    temporary_goal_revalidation_required_ = false;
    avoidance_corridor_selected_ = false;
    consecutive_planning_failures_ = 0;
    temporary_goal_reselection_count_ = 0;
    avoidance_side_candidate_retry_batch_ = 0U;
    consecutive_temporary_goal_invalid_checks_ = 0;
    consecutive_blocked_path_checks_ = 0;
    avoidance_temporary_path_.poses.clear();
    last_validated_costmap_version_ = 0U;
    last_path_checked_costmap_version_ = 0U;
    last_retry_costmap_version_ = 0U;
    temporary_goal_last_validation_valid_ = false;
    next_temporary_goal_retry_time_ = std::chrono::steady_clock::time_point{};
    ++avoidance_task_generation_;
  }
  clearLocalPath();
  if (local_path_pub_) {
    nav_msgs::msg::Path empty_path;
    empty_path.header.frame_id = global_frame_;
    empty_path.header.stamp = node_->now();
    local_path_pub_->publish(empty_path);
  }
  clearAvoidanceMarkers();
}

void LocalAvoidanceManager::enterWaitingForDynamicPath() {
  stage_alignment_requested_.store(false);
  {
    std::lock_guard<std::mutex> avoidance_lock(avoidance_mutex_);
    avoidance_active_ = false;
    avoidance_completed_ = false;
    avoidance_stage_ = AvoidanceStage::WAITING_DYNAMIC_PATH;
    avoidance_temporary_goal_ = geometry_msgs::msg::PoseStamped();
    avoidance_temporary_goal_index_ =
        std::numeric_limits<std::size_t>::max();
    avoidance_search_start_index_ = 0U;
    avoidance_obstacle_search_start_index_ = 0U;
    avoidance_minimum_goal_index_ = 0U;
    avoidance_temporary_goal_locked_ = false;
    avoidance_temporary_goal_on_global_path_ = false;
    avoidance_force_side_fallback_ = false;
    avoidance_retained_static_return_valid_ = false;
    avoidance_retained_static_return_goal_ =
        geometry_msgs::msg::PoseStamped();
    avoidance_retained_static_return_index_ =
        std::numeric_limits<std::size_t>::max();
    avoidance_heading_continuity_required_ = false;
    avoidance_preferred_side_ = AvoidanceSide::NONE;
    avoidance_selected_side_ = AvoidanceSide::NONE;
    avoidance_alternate_side_enabled_ = false;
    avoidance_left_clear_at_trigger_ = false;
    avoidance_right_clear_at_trigger_ = false;
    temporary_goal_revalidation_required_ = false;
    avoidance_corridor_selected_ = false;
    consecutive_planning_failures_ = 0;
    temporary_goal_reselection_count_ = 0;
    avoidance_side_candidate_retry_batch_ = 0U;
    consecutive_temporary_goal_invalid_checks_ = 0;
    consecutive_blocked_path_checks_ = 0;
    avoidance_temporary_path_.poses.clear();
    last_validated_costmap_version_ = 0U;
    last_path_checked_costmap_version_ = 0U;
    last_retry_costmap_version_ = 0U;
    temporary_goal_last_validation_valid_ = false;
    next_temporary_goal_retry_time_ =
        std::chrono::steady_clock::time_point{};
    ++avoidance_task_generation_;
    std::lock_guard<std::mutex> path_lock(path_mutex_);
    local_path_valid_ = false;
    local_path_.poses.clear();
    previous_local_path_.poses.clear();
    path_stability_counter_ = 0;
  }
  if (local_path_pub_) {
    nav_msgs::msg::Path empty_path;
    empty_path.header.frame_id = global_frame_;
    empty_path.header.stamp = node_->now();
    local_path_pub_->publish(empty_path);
  }
  clearAvoidanceMarkers();
}

bool LocalAvoidanceManager::isAvoidanceCompleted() const {
  std::lock_guard<std::mutex> lock(avoidance_mutex_);
  return avoidance_completed_;
}

bool LocalAvoidanceManager::isAvoidanceFailed() const {
  std::lock_guard<std::mutex> lock(avoidance_mutex_);
  return avoidance_stage_ == AvoidanceStage::FAILED;
}

bool LocalAvoidanceManager::isWaitingForDynamicPath() const {
  std::lock_guard<std::mutex> lock(avoidance_mutex_);
  return avoidance_stage_ == AvoidanceStage::WAITING_DYNAMIC_PATH;
}

bool LocalAvoidanceManager::isWaitingForTemporaryGoal() const {
  std::lock_guard<std::mutex> lock(avoidance_mutex_);
  return avoidance_stage_ == AvoidanceStage::WAITING_TEMPORARY_GOAL;
}

void LocalAvoidanceManager::stop() {
  running_ = false;
  planning_condition_.notify_all();
  if (planning_thread_.joinable()) {
    planning_thread_.join();
  }
}

void LocalAvoidanceManager::requestPlanningWakeup() {
  {
    std::lock_guard<std::mutex> lock(planning_wait_mutex_);
    planning_wakeup_requested_ = true;
  }
  planning_condition_.notify_one();
}

void LocalAvoidanceManager::planningLoop() {
  const auto periodic_interval =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(
              1.0 / std::max(0.01, planning_frequency_)));

  while (running_ && rclcpp::ok()) {
    try {
      // Costmap回调只提交最新指针；耗时的footprint构建固定在规划线程执行。
      const auto prepare_start = TimingClock::now();
      if (avoidance_path_planner_) {
        avoidance_path_planner_->prepareLatestCostmap();
      }
      const double prepare_ms = elapsedMs(prepare_start);
      std::uint64_t timing_generation = 0U;
      AvoidanceStage timing_stage = AvoidanceStage::IDLE;
      TimingClock::time_point trigger_time;
      bool timing_active = false;
      {
        std::lock_guard<std::mutex> lock(avoidance_mutex_);
        timing_active = avoidance_active_;
        timing_generation = avoidance_task_generation_;
        timing_stage = avoidance_stage_;
        trigger_time = avoidance_trigger_time_;
      }
      const auto compute_start = TimingClock::now();
      const bool planning_result = computeLocalPath();
      if (avoidance_timing_enabled_ && timing_active) {
        const double compute_ms = elapsedMs(compute_start);
        const double since_trigger_ms =
            trigger_time == TimingClock::time_point{}
                ? -1.0
                : std::chrono::duration<double, std::milli>(
                      compute_start - trigger_time).count();
        RCLCPP_INFO(node_->get_logger(),
                    "[AVOID_TIMING_CYCLE] task=%lu stage=%d map=%lu "
                    "since_trigger_ms=%.1f prepare_ms=%.1f "
                    "compute_ms=%.1f result=%d local_valid=%d",
                    static_cast<unsigned long>(timing_generation),
                    static_cast<int>(timing_stage),
                    static_cast<unsigned long>(
                        avoidance_path_planner_->preparedCostmapVersion()),
                    since_trigger_ms, prepare_ms, compute_ms,
                    planning_result, local_path_valid_.load());
      }
      RCLCPP_DEBUG_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "[AVOID_THREAD] computeLocalPath result=%d local_valid=%d",
          planning_result, local_path_valid_.load());
    } catch (const std::exception &e) {
      RCLCPP_ERROR(node_->get_logger(), "Exception in local path planning: %s",
                   e.what());
    }

    std::unique_lock<std::mutex> wait_lock(planning_wait_mutex_);
    planning_condition_.wait_for(
        wait_lock, periodic_interval,
        [this]() { return !running_.load() || planning_wakeup_requested_; });
    planning_wakeup_requested_ = false;
  }
}

bool LocalAvoidanceManager::computeLocalPath() {
  // 绕障临时点搜索、目标复核和A*都在规划线程执行。耗时计算期间不持有
  // Manager互斥量，提交时再用task generation拒绝已经取消的旧结果。
  if (!global_path_received_ || !costmap_received_ ||
      !avoidance_path_planner_ ||
      !avoidance_path_planner_->hasPreparedCostmap()) {
    RCLCPP_DEBUG_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "[AVOID_THREAD_SKIP] A* not called: global_path=%d costmap=%d "
      "planner=%d prepared=%d",
      global_path_received_.load(), costmap_received_.load(),
      avoidance_path_planner_ ? 1 : 0,
      avoidance_path_planner_ &&
        avoidance_path_planner_->hasPreparedCostmap());
    // 输入或准备状态的短暂抖动不应撤销已经提交给控制器的安全路径。
    // 只有明确的状态迁移、路径失效确认或reset才清除local_path_valid_。
    return local_path_valid_.load();
  }

  bool avoidance_active = false;
  {
    std::lock_guard<std::mutex> lock(avoidance_mutex_);
    avoidance_active = avoidance_active_;
  }
  if (!avoidance_active) {
    bool should_publish_empty = false;
    {
      std::lock_guard<std::mutex> path_lock(path_mutex_);
      should_publish_empty = local_path_valid_ || !local_path_.poses.empty() ||
                             !previous_local_path_.poses.empty();
    }
    clearLocalPath();
    if (should_publish_empty && local_path_pub_) {
      nav_msgs::msg::Path empty_path;
      empty_path.header.frame_id = global_frame_;
      empty_path.header.stamp = node_->now();
      local_path_pub_->publish(empty_path);
    }
    return false;
  }

  geometry_msgs::msg::PoseStamped robot_pose;
  if (!getRobotPose(robot_pose)) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "[AVOID_THREAD_SKIP] A* not called: failed to get robot pose");
    return local_path_valid_.load();
  }

  nav_msgs::msg::Path global_path_copy;
  {
    std::lock_guard<std::mutex> lock(global_path_mutex_);
    global_path_copy = global_path_;
  }
  if (global_path_copy.poses.size() < 2U) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "[TEMP_GOAL_WAIT] global path is unavailable");
    return local_path_valid_.load();
  }

  const auto now = std::chrono::steady_clock::now();
  const auto retry_delay =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(avoidance_temporary_goal_retry_period_));
  const std::uint64_t costmap_version =
      avoidance_path_planner_->preparedCostmapVersion();

  // 到达锁定临时点后不再生成第二段回归路径，直接从该原路径索引继续。
  // 动态路径也不能无限等待发布方重发：UWB目标未超过更新阈值或UWB丢失
  // 后继续旧目标时，发布方可能不会产生新路径。显著变化的新动态路径仍会
  // 通过setGlobalPath随时取消并覆盖这个旧任务。
  bool goal_reached = false;
  bool reached_dynamic_goal = false;
  bool reached_goal_on_global_path = false;
  geometry_msgs::msg::Point reached_goal;
  std::size_t reached_goal_index = std::numeric_limits<std::size_t>::max();
  {
    std::lock_guard<std::mutex> avoidance_lock(avoidance_mutex_);
    if (avoidance_active_ && avoidance_temporary_goal_locked_ &&
        avoidance_stage_ == AvoidanceStage::TRACKING_TEMPORARY_GOAL) {
      const double distance = calculateDistance(
          robot_pose.pose.position, avoidance_temporary_goal_.pose.position);
      RCLCPP_DEBUG_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 500,
          "[TEMP_GOAL_PROGRESS] robot=(%.3f,%.3f) goal=(%.3f,%.3f) "
          "index=%zu distance=%.3f tolerance=%.3f",
          robot_pose.pose.position.x, robot_pose.pose.position.y,
          avoidance_temporary_goal_.pose.position.x,
          avoidance_temporary_goal_.pose.position.y,
          avoidance_temporary_goal_index_, distance,
          avoidance_goal_tolerance_);
      if (distance <= avoidance_goal_tolerance_) {
        goal_reached = true;
        reached_dynamic_goal = dynamic_path_mode_.load();
        reached_goal_on_global_path =
            avoidance_temporary_goal_on_global_path_;
        reached_goal = avoidance_temporary_goal_.pose.position;
        reached_goal_index = avoidance_temporary_goal_index_;
        avoidance_active_ = false;
        avoidance_completed_ = true;
        avoidance_stage_ = AvoidanceStage::COMPLETED;
        avoidance_corridor_selected_ = false;
        avoidance_temporary_path_.poses.clear();
        ++avoidance_task_generation_;
        std::lock_guard<std::mutex> path_lock(path_mutex_);
        local_path_valid_ = false;
        local_path_.poses.clear();
        previous_local_path_.poses.clear();
        path_stability_counter_ = 0;
      }
    }
  }
  if (goal_reached) {
    clearAvoidanceMarkers();
    RCLCPP_INFO(node_->get_logger(),
                "[TEMP_GOAL_REACHED] source=%s index=%zu goal=(%.3f,%.3f) "
                "next=%s",
                reached_goal_on_global_path ? "GLOBAL_PATH" : "SIDE_FALLBACK",
                reached_goal_index, reached_goal.x, reached_goal.y,
                reached_dynamic_goal ? "GLOBAL_PATH_LAST_DYNAMIC_TARGET"
                                     : "GLOBAL_PATH");
    return false;
  }

  AvoidanceStage planning_stage = AvoidanceStage::IDLE;
  std::uint64_t task_generation = 0U;
  TimingClock::time_point task_trigger_time;
  std::size_t search_start_index = 0U;
  std::size_t obstacle_search_start_index = 0U;
  std::size_t minimum_goal_index = 0U;
  bool goal_locked = false;
  bool goal_on_global_path = false;
  bool force_side_fallback = false;
  bool retained_static_return_valid = false;
  geometry_msgs::msg::PoseStamped retained_static_return_goal;
  std::size_t retained_static_return_index =
      std::numeric_limits<std::size_t>::max();
  bool heading_continuity_required = false;
  AvoidanceSide preferred_side = AvoidanceSide::NONE;
  AvoidanceSide selected_side = AvoidanceSide::NONE;
  bool alternate_side_enabled = false;
  bool left_clear_at_trigger = false;
  bool right_clear_at_trigger = false;
  std::size_t side_candidate_retry_batch = 0U;
  bool revalidation_required = false;
  bool corridor_selected = false;
  bool last_goal_validation_valid = false;
  std::uint64_t last_validated_version = 0U;
  std::uint64_t last_path_checked_version = 0U;
  std::uint64_t last_retry_version = 0U;
  std::chrono::steady_clock::time_point next_retry;
  geometry_msgs::msg::PoseStamped temporary_goal;
  std::size_t temporary_goal_index =
      std::numeric_limits<std::size_t>::max();
  geometry_msgs::msg::Point obstacle_center;
  nav_msgs::msg::Path selected_path;
  {
    std::lock_guard<std::mutex> lock(avoidance_mutex_);
    if (!avoidance_active_) return false;
    planning_stage = avoidance_stage_;
    task_generation = avoidance_task_generation_;
    task_trigger_time = avoidance_trigger_time_;
    search_start_index = avoidance_search_start_index_;
    obstacle_search_start_index = avoidance_obstacle_search_start_index_;
    minimum_goal_index = avoidance_minimum_goal_index_;
    goal_locked = avoidance_temporary_goal_locked_;
    goal_on_global_path = avoidance_temporary_goal_on_global_path_;
    force_side_fallback = avoidance_force_side_fallback_;
    retained_static_return_valid =
        avoidance_retained_static_return_valid_;
    retained_static_return_goal =
        avoidance_retained_static_return_goal_;
    retained_static_return_index =
        avoidance_retained_static_return_index_;
    heading_continuity_required =
        avoidance_heading_continuity_required_;
    preferred_side = avoidance_preferred_side_;
    selected_side = avoidance_selected_side_;
    alternate_side_enabled = avoidance_alternate_side_enabled_;
    left_clear_at_trigger = avoidance_left_clear_at_trigger_;
    right_clear_at_trigger = avoidance_right_clear_at_trigger_;
    side_candidate_retry_batch = avoidance_side_candidate_retry_batch_;
    revalidation_required = temporary_goal_revalidation_required_;
    corridor_selected = avoidance_corridor_selected_;
    last_goal_validation_valid = temporary_goal_last_validation_valid_;
    last_validated_version = last_validated_costmap_version_;
    last_path_checked_version = last_path_checked_costmap_version_;
    last_retry_version = last_retry_costmap_version_;
    next_retry = next_temporary_goal_retry_time_;
    temporary_goal = avoidance_temporary_goal_;
    temporary_goal_index = avoidance_temporary_goal_index_;
    obstacle_center = avoidance_obstacle_center_;
    selected_path = avoidance_temporary_path_;
  }

  const auto generation_is_current_while_locked = [this, task_generation]() {
    return avoidance_active_ &&
           avoidance_task_generation_ == task_generation;
  };

  // 已锁定的临时点只在新Costmap或stall强制请求时复核。一次无效只记账，
  // 连续多张地图无效后才解锁，避免点云闪烁导致目标跳变。
  if (goal_locked) {
    const bool new_costmap_version =
        costmap_version != last_validated_version;
    const bool validation_due =
        revalidation_required || new_costmap_version;
    if (validation_due) {
      const bool goal_valid = goal_on_global_path
          ? temporary_goal_index < global_path_copy.poses.size() &&
                avoidance_path_planner_->isTemporaryGoalValid(
                    global_path_copy, temporary_goal, temporary_goal_index)
          : !avoidance_path_planner_->isPointInObstacle(
                temporary_goal.pose.position,
                tf2::getYaw(temporary_goal.pose.orientation));
      bool goal_invalidated = false;
      int invalid_checks = 0;
      {
        std::lock_guard<std::mutex> avoidance_lock(avoidance_mutex_);
        if (!generation_is_current_while_locked() ||
            !avoidance_temporary_goal_locked_ ||
            avoidance_temporary_goal_index_ != temporary_goal_index) {
          return false;
        }
        last_validated_costmap_version_ = costmap_version;
        temporary_goal_revalidation_required_ = false;
        temporary_goal_last_validation_valid_ = goal_valid;
        if (goal_valid) {
          consecutive_temporary_goal_invalid_checks_ = 0;
        } else {
          // stall等事件可要求在当前地图上立即复核，但同一份地图不能被
          // 多次采样成“连续确认”。只有新的prepared costmap版本才计数。
          if (new_costmap_version) {
            invalid_checks = ++consecutive_temporary_goal_invalid_checks_;
          } else {
            invalid_checks = consecutive_temporary_goal_invalid_checks_;
          }
          if (invalid_checks >= avoidance_path_blocked_confirmations_) {
            ++avoidance_task_generation_;
            if (avoidance_selected_side_ != AvoidanceSide::NONE) {
              avoidance_alternate_side_enabled_ = true;
              avoidance_preferred_side_ =
                  avoidance_selected_side_ == AvoidanceSide::LEFT
                      ? AvoidanceSide::RIGHT
                      : AvoidanceSide::LEFT;
              avoidance_heading_continuity_required_ = true;
              avoidance_force_side_fallback_ = true;
            }
            avoidance_temporary_goal_locked_ = false;
            avoidance_temporary_goal_ = geometry_msgs::msg::PoseStamped();
            avoidance_temporary_goal_index_ =
                std::numeric_limits<std::size_t>::max();
            avoidance_temporary_goal_on_global_path_ = false;
            avoidance_corridor_selected_ = false;
            avoidance_temporary_path_.poses.clear();
            avoidance_stage_ = AvoidanceStage::SEARCHING_TEMPORARY_GOAL;
            // 临时点失效不代表原障碍已经消失。仍从本次绕障启动时的
            // 障碍扫描锚点识别阻塞段，只跳过这个已失效候选。
            avoidance_search_start_index_ = std::min(
                avoidance_obstacle_search_start_index_,
                global_path_copy.poses.size() - 1U);
            avoidance_minimum_goal_index_ = std::max(
                avoidance_minimum_goal_index_,
                temporary_goal_index < global_path_copy.poses.size()
                    ? temporary_goal_index + 1U
                    : global_path_copy.poses.size());
            consecutive_temporary_goal_invalid_checks_ = 0;
            consecutive_blocked_path_checks_ = 0;
            next_temporary_goal_retry_time_ = now;
            goal_invalidated = true;
            std::lock_guard<std::mutex> path_lock(path_mutex_);
            local_path_valid_ = false;
            local_path_.poses.clear();
            previous_local_path_.poses.clear();
            path_stability_counter_ = 0;
          } else if (!avoidance_corridor_selected_) {
            avoidance_stage_ = AvoidanceStage::WAITING_TEMPORARY_GOAL;
            next_temporary_goal_retry_time_ = now + retry_delay;
          }
        }
      }
      if (goal_invalidated) {
        clearAvoidanceMarkers();
        RCLCPP_WARN(node_->get_logger(),
                    "[TEMP_GOAL_INVALIDATED] index=%zu invalid=%d/%d; "
                    "searching a new post-obstacle goal old_side=%s "
                    "alternate_side=%d",
                    temporary_goal_index, invalid_checks,
                    avoidance_path_blocked_confirmations_,
                    avoidanceSideToString(selected_side),
                    selected_side != AvoidanceSide::NONE ? 1 : 0);
        requestPlanningWakeup();
        return false;
      }
      if (!goal_valid) {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 500,
            "[TEMP_GOAL_UNSTABLE] index=%zu invalid=%d/%d; goal retained",
            temporary_goal_index, invalid_checks,
            avoidance_path_blocked_confirmations_);
        if (!corridor_selected) return false;
      } else {
        last_goal_validation_valid = true;
      }
    } else if (!last_goal_validation_valid) {
      // 同一张地图已经判定目标不安全，等待新地图而不是在已知不安全目标上
      // 重复运行A*。
      return false;
    }
  }

  // 锁定路径只在Costmap版本变化时检查剩余段；同一版本不能累计多次确认。
  if (goal_locked && corridor_selected && !selected_path.poses.empty()) {
    if (costmap_version == last_path_checked_version) return true;

    LocalPathProjection projection;
    const bool projection_valid =
        projectRemainingLocalPath(robot_pose, selected_path, projection);
    // PP使用固定局部前瞻跟踪。若机器人已经横向偏离路径超过一个前瞻
    // 距离，继续复用旧路径不再可靠，应正常重规划；厘米级定位/运动偏差
    // 则从前向投影处复检，不能回头连接旧起点。
    const bool close_to_path =
        projection_valid &&
        projection.distance <= local_avoidance_lookahead_distance_ + 1e-9;
    bool remaining_path_safe =
        close_to_path &&
        avoidance_path_planner_->isInitialEscapePathCollisionFree(
            projection.remaining_path, obstacle_center,
            avoidance_path_clearance_);

    // 在最新Costmap上重新确认从机器人当前航向转向剩余路径首段是安全
    // 的。该检查替代旧的“机器人位置 -> 旧路径点”伪线段，同时保留原有
    // 起点footprint重叠时仅允许小角度向外脱离的安全约束。
    if (remaining_path_safe && projection.remaining_path.poses.size() >= 2U) {
      const auto &first = projection.remaining_path.poses.front().pose.position;
      const auto &second = projection.remaining_path.poses[1U].pose.position;
      const double dx = second.x - first.x;
      const double dy = second.y - first.y;
      if (std::hypot(dx, dy) >= 1e-8) {
        const double path_yaw = std::atan2(dy, dx);
        const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
        if (!avoidance_path_planner_->isRotationCollisionFree(
                robot_pose.pose.position, robot_yaw, path_yaw)) {
          const double heading_error = std::abs(std::atan2(
              std::sin(path_yaw - robot_yaw),
              std::cos(path_yaw - robot_yaw)));
          const bool starts_in_footprint_overlap =
              avoidance_path_planner_->isPointInObstacle(
                  robot_pose.pose.position, robot_yaw);
          remaining_path_safe =
              starts_in_footprint_overlap &&
              heading_error <= avoidance_initial_escape_max_heading_error_;
        }
      }
    }
    if (!remaining_path_safe) {
      int blocked_checks = 0;
      bool replan_same_goal = false;
      bool reselect_side_goal = false;
      bool reselect_static_transition = false;
      {
        std::lock_guard<std::mutex> avoidance_lock(avoidance_mutex_);
        if (!generation_is_current_while_locked() ||
            !avoidance_temporary_goal_locked_ ||
            avoidance_temporary_goal_index_ != temporary_goal_index) {
          return false;
        }
        last_path_checked_costmap_version_ = costmap_version;
        blocked_checks = consecutive_blocked_path_checks_.fetch_add(1) + 1;
        if (blocked_checks >= avoidance_path_blocked_confirmations_) {
          ++avoidance_task_generation_;
          avoidance_corridor_selected_ = false;
          avoidance_temporary_path_.poses.clear();
          temporary_goal_revalidation_required_ = false;
          avoidance_heading_continuity_required_ = true;
          // 当前侧绕路径已被不同Costmap连续确认失效。下一轮不再被首次
          // 触发时的单侧快照锁死：优先尝试实际执行侧的对侧，同时保留
          // 两侧候选，最终仍由A*和完整footprint校验决定是否可行。
          avoidance_alternate_side_enabled_ = true;
          if (avoidance_selected_side_ == AvoidanceSide::LEFT) {
            avoidance_preferred_side_ = AvoidanceSide::RIGHT;
          } else if (avoidance_selected_side_ == AvoidanceSide::RIGHT) {
            avoidance_preferred_side_ = AvoidanceSide::LEFT;
          }
          consecutive_blocked_path_checks_ = 0;
          consecutive_planning_failures_ = 0;
          next_temporary_goal_retry_time_ = now;
          // 侧方路径已经在最新Costmap上连续失效时，不再固守原侧方点。
          // 直接重选候选，避免A*到旧点生成一条需要贴障原地转向的路径。
          reselect_side_goal =
              !avoidance_temporary_goal_on_global_path_ &&
              dynamic_path_mode_.load() &&
              avoidance_dynamic_side_goal_fallback_enabled_;
          reselect_static_transition =
              avoidance_temporary_goal_on_global_path_ &&
              !dynamic_path_mode_.load() &&
              avoidance_static_side_transition_enabled_ &&
              (avoidance_alternate_side_enabled_ ||
               avoidance_left_clear_at_trigger_ ||
               avoidance_right_clear_at_trigger_);
          if (reselect_static_transition) {
            // 已执行路径失效后，目标本身仍通过安全窗检查。保留它作为
            // 回归目标，直接重选侧后方过渡路径，避免先运行一次必然会
            // 被起始旋转扫掠拒绝的“机器人->原回归点”直达规划。
            avoidance_retained_static_return_valid_ = true;
            avoidance_retained_static_return_goal_ =
                avoidance_temporary_goal_;
            avoidance_retained_static_return_index_ =
                avoidance_temporary_goal_index_;
            avoidance_minimum_goal_index_ = std::max(
                avoidance_minimum_goal_index_,
                avoidance_temporary_goal_index_ < global_path_copy.poses.size()
                    ? avoidance_temporary_goal_index_ + 1U
                    : global_path_copy.poses.size());
          }
          if (reselect_side_goal || reselect_static_transition) {
            avoidance_temporary_goal_locked_ = false;
            avoidance_temporary_goal_ = geometry_msgs::msg::PoseStamped();
            avoidance_temporary_goal_index_ =
                std::numeric_limits<std::size_t>::max();
            avoidance_temporary_goal_on_global_path_ = false;
            avoidance_force_side_fallback_ = true;
            avoidance_stage_ = AvoidanceStage::SEARCHING_TEMPORARY_GOAL;
          } else {
            avoidance_stage_ = AvoidanceStage::TRACKING_TEMPORARY_GOAL;
            replan_same_goal = true;
          }
          std::lock_guard<std::mutex> path_lock(path_mutex_);
          local_path_valid_ = false;
          local_path_.poses.clear();
          previous_local_path_.poses.clear();
          path_stability_counter_ = 0;
        }
      }
      if (!replan_same_goal) {
        if (reselect_side_goal || reselect_static_transition) {
          clearAvoidanceMarkers();
          RCLCPP_WARN(
              node_->get_logger(),
              "[TEMP_PATH_RESELECT] path blocked=%d/%d; searching "
              "heading-continuous side candidates mode=%s old_side=%s "
              "next_preferred=%s both_sides=1",
              blocked_checks, avoidance_path_blocked_confirmations_,
              reselect_static_transition ? "STATIC_RETURN" : "DYNAMIC",
              avoidanceSideToString(selected_side),
              avoidanceSideToString(
                  selected_side == AvoidanceSide::LEFT
                      ? AvoidanceSide::RIGHT
                      : (selected_side == AvoidanceSide::RIGHT
                             ? AvoidanceSide::LEFT
                             : preferred_side)));
          requestPlanningWakeup();
          return false;
        }
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 500,
            "[TEMP_PATH_UNSTABLE] blocked=%d/%d; retaining goal and path",
            blocked_checks, avoidance_path_blocked_confirmations_);
        return true;
      }
      RCLCPP_WARN(node_->get_logger(),
                  "[TEMP_PATH_REPLAN] path blocked=%d/%d segment=%zu "
                  "projection_distance=%.3f projection_valid=%d; "
                  "replanning same goal index=%zu",
                  blocked_checks, avoidance_path_blocked_confirmations_,
                  projection.segment_index, projection.distance,
                  projection_valid ? 1 : 0, temporary_goal_index);
      requestPlanningWakeup();
      return false;
    }

    {
      std::lock_guard<std::mutex> lock(avoidance_mutex_);
      if (!generation_is_current_while_locked()) {
        return false;
      }
      consecutive_blocked_path_checks_ = 0;
      last_path_checked_costmap_version_ = costmap_version;
    }
    return true;
  }

  // 没有锁定目标时，按重试周期或新Costmap触发一次搜索。搜索失败只进入
  // 显式等待，不创建左右侧目标，也不锁存为永久失败。
  if (!goal_locked) {
    const bool search_due =
        planning_stage == AvoidanceStage::SEARCHING_TEMPORARY_GOAL ||
        now >= next_retry || costmap_version != last_retry_version;
    if (!search_due) return false;
    const auto goal_search_start = TimingClock::now();

    search_start_index =
        std::min(search_start_index, global_path_copy.poses.size() - 1U);
    obstacle_search_start_index = std::min(
        obstacle_search_start_index, global_path_copy.poses.size() - 1U);
    TemporaryGoalSearchResult search_result;
    bool using_retained_static_return = false;
    const bool retained_return_requested =
        !dynamic_path_mode_.load() &&
        avoidance_static_side_transition_enabled_ && force_side_fallback &&
        retained_static_return_valid &&
        retained_static_return_index < global_path_copy.poses.size();
    if (retained_return_requested &&
        avoidance_path_planner_->isTemporaryGoalValid(
            global_path_copy, retained_static_return_goal,
            retained_static_return_index)) {
      search_result.status = TemporaryGoalSearchStatus::FOUND;
      search_result.search_start_index = search_start_index;
      search_result.temporary_goal_index = retained_static_return_index;
      search_result.goal = retained_static_return_goal;
      search_result.goal_yaw =
          tf2::getYaw(retained_static_return_goal.pose.orientation);
      using_retained_static_return = true;
      RCLCPP_INFO(
          node_->get_logger(),
          "[STATIC_RETURN_RETAINED] retry side transition to index=%zu "
          "goal=(%.3f,%.3f); direct path will not be retried",
          retained_static_return_index,
          retained_static_return_goal.pose.position.x,
          retained_static_return_goal.pose.position.y);
    } else {
      if (retained_return_requested) {
        std::lock_guard<std::mutex> avoidance_lock(avoidance_mutex_);
        if (!generation_is_current_while_locked()) {
          return false;
        }
        avoidance_retained_static_return_valid_ = false;
        avoidance_retained_static_return_goal_ =
            geometry_msgs::msg::PoseStamped();
        avoidance_retained_static_return_index_ =
            std::numeric_limits<std::size_t>::max();
        RCLCPP_WARN(node_->get_logger(),
                    "[STATIC_RETURN_DROPPED] retained return index=%zu is "
                    "no longer safe on the latest Costmap",
                    retained_static_return_index);
      }
      search_result = avoidance_path_planner_->findTemporaryGoal(
          global_path_copy, robot_pose.pose.position, search_start_index,
          minimum_goal_index);
    }

    // 普通导航的基础搜索窗内没有安全回归点时，先沿剩余全局路径扩大
    // 弧长搜索。它仍要求候选点、后续安全窗和完整footprint全部有效，
    // 只是避免6m窗口把实际存在的障碍后回归位置裁掉后永久停车。
    if (!using_retained_static_return &&
        search_result.status ==
            TemporaryGoalSearchStatus::NO_SAFE_TEMPORARY_GOAL &&
        !dynamic_path_mode_.load() &&
        avoidance_static_side_transition_enabled_) {
      const auto extended_result =
          avoidance_path_planner_->findTemporaryGoal(
              global_path_copy, robot_pose.pose.position, search_start_index,
              minimum_goal_index,
              avoidance_static_return_extended_search_distance_);
      if (extended_result.status == TemporaryGoalSearchStatus::FOUND) {
        search_result = extended_result;
        RCLCPP_INFO(
            node_->get_logger(),
            "[STATIC_RETURN_SEARCH_EXTENDED] found index=%zu goal=(%.3f,%.3f) "
            "within %.2fm",
            search_result.temporary_goal_index,
            search_result.goal.pose.position.x,
            search_result.goal.pose.position.y,
            avoidance_static_return_extended_search_distance_);
      }
    }

    // 兼容旧状态或其他重规划分支留下的前移搜索起点：已进入绕障后，
    // 从前移位置得到NO_BLOCKED_PATH不能作为等待依据，必须在同一周期
    // 回到原障碍区重新识别并选择其后的临时点。
    if (!using_retained_static_return &&
        search_result.status == TemporaryGoalSearchStatus::NO_BLOCKED_PATH &&
        search_start_index != obstacle_search_start_index) {
      RCLCPP_WARN(node_->get_logger(),
                  "[TEMP_GOAL_SEARCH_REWIND] NO_BLOCKED_PATH at index=%zu; "
                  "retrying original obstacle anchor=%zu min_goal=%zu",
                  search_start_index, obstacle_search_start_index,
                  minimum_goal_index);
      search_start_index = obstacle_search_start_index;
      search_result = avoidance_path_planner_->findTemporaryGoal(
          global_path_copy, robot_pose.pose.position, search_start_index,
          minimum_goal_index);
      std::lock_guard<std::mutex> avoidance_lock(avoidance_mutex_);
      if (!generation_is_current_while_locked()) {
        return false;
      }
      avoidance_search_start_index_ = search_start_index;
    }
    if (avoidance_timing_enabled_) {
      RCLCPP_INFO(node_->get_logger(),
                  "[AVOID_TIMING_GOAL] task=%lu map=%lu elapsed_ms=%.1f "
                  "status=%d retained=%d start_index=%zu min_index=%zu",
                  static_cast<unsigned long>(task_generation),
                  static_cast<unsigned long>(costmap_version),
                  elapsedMs(goal_search_start),
                  static_cast<int>(search_result.status),
                  using_retained_static_return, search_start_index,
                  minimum_goal_index);
    }

    // UWB动态路径可能在障碍物附近结束，因而没有足够的“障碍后安全段”。
    // 此时可采用侧方临时点。普通导航直达全局回归点失败后，也允许把
    // 侧后方候选作为中间过渡点，但最终目标仍必须回到原全局路径。
    bool side_fallback_found = false;
    AvoidanceSide selected_fallback_side = AvoidanceSide::NONE;
    geometry_msgs::msg::PoseStamped side_fallback_goal;
    nav_msgs::msg::Path side_fallback_path;
    double side_fallback_minimum_clearance = 0.0;
    double side_fallback_score = std::numeric_limits<double>::infinity();
    bool side_fallback_suppress_alignment = false;
    bool side_fallback_used_smoothed_path = false;
    const auto side_batch_start = TimingClock::now();
    struct RankedSideCandidate {
      SideGoalCandidate candidate;
      double quick_score{std::numeric_limits<double>::infinity()};
      double direct_heading_error{0.0};
      double estimated_endpoint_clearance{0.0};
    };
    std::vector<RankedSideCandidate> ranked_side_candidates;
    const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
    const bool dynamic_side_fallback_requested =
        dynamic_path_mode_.load() &&
        avoidance_dynamic_side_goal_fallback_enabled_ &&
        (force_side_fallback ||
         search_result.status ==
             TemporaryGoalSearchStatus::NO_SAFE_TEMPORARY_GOAL);
    const bool static_side_transition_requested =
        !dynamic_path_mode_.load() &&
        avoidance_static_side_transition_enabled_ && force_side_fallback &&
        search_result.status == TemporaryGoalSearchStatus::FOUND &&
        search_result.temporary_goal_index < global_path_copy.poses.size();
    const bool static_side_escape_requested =
        !dynamic_path_mode_.load() &&
        avoidance_static_side_transition_enabled_ && force_side_fallback &&
        search_result.status != TemporaryGoalSearchStatus::FOUND;
    auto collect_side_candidates = [&](AvoidanceSide side) {
      if (side == AvoidanceSide::NONE) return;
      const bool side_was_clear =
          side == AvoidanceSide::LEFT ? left_clear_at_trigger
                                      : right_clear_at_trigger;
      // 首次规划仍只信任检测器确认过的空旷侧；已有侧绕路径被连续多张
      // Costmap判定失效后开放另一侧，让候选终点、完整A*和footprint
      // 校验重新判断，而不是永远受旧快照约束。
      if (!side_was_clear && !alternate_side_enabled &&
          !static_side_escape_requested) {
        return;
      }

      const auto candidates =
          avoidance_path_planner_->buildSideGoalCandidates(
              robot_pose, obstacle_center, side, global_path_copy,
              static_cast<int>(search_start_index),
              avoidance_max_entry_angle_);
      ranked_side_candidates.reserve(ranked_side_candidates.size() +
                                     candidates.size());
      for (const auto &candidate : candidates) {
        const double dx = candidate.point.x - robot_pose.pose.position.x;
        const double dy = candidate.point.y - robot_pose.pose.position.y;
        const double direct_distance = std::hypot(dx, dy);
        const double direct_yaw = std::atan2(dy, dx);
        const double direct_heading_error = std::abs(std::atan2(
            std::sin(direct_yaw - robot_yaw),
            std::cos(direct_yaw - robot_yaw)));
        // 候选点位于估计障碍物侧边界和前边界之外。两方向净空的合成值
        // 可在不运行A*的情况下近似候选终点到障碍物矩形角点的距离。
        const double estimated_endpoint_clearance = std::hypot(
            std::max(0.0, candidate.boundary_clearance),
            std::max(0.0, candidate.forward_clearance));
        const double heading_weight_multiplier =
            heading_continuity_required
                ? avoidance_replan_heading_score_multiplier_
                : 1.0;
        const double quick_score =
            avoidance_side_score_path_length_weight_ * direct_distance +
            avoidance_side_score_heading_weight_ *
                heading_weight_multiplier * direct_heading_error +
            avoidance_side_score_clearance_weight_ /
                std::max(0.05, estimated_endpoint_clearance) +
            (side == preferred_side
                 ? 0.0
                 : avoidance_side_score_switch_side_penalty_);
        ranked_side_candidates.push_back(
            RankedSideCandidate{candidate, quick_score,
                                direct_heading_error,
                                estimated_endpoint_clearance});
      }
    };
    if (dynamic_side_fallback_requested ||
        static_side_transition_requested || static_side_escape_requested) {
      collect_side_candidates(preferred_side);
      if (alternate_side_enabled ||
          (left_clear_at_trigger && right_clear_at_trigger)) {
        const AvoidanceSide alternate =
            preferred_side == AvoidanceSide::LEFT ? AvoidanceSide::RIGHT
                                                  : AvoidanceSide::LEFT;
        if (preferred_side == AvoidanceSide::NONE) {
          collect_side_candidates(AvoidanceSide::LEFT);
          collect_side_candidates(AvoidanceSide::RIGHT);
        } else {
          collect_side_candidates(alternate);
        }
      }
    }

    std::stable_sort(
        ranked_side_candidates.begin(), ranked_side_candidates.end(),
        [](const RankedSideCandidate &left,
           const RankedSideCandidate &right) {
          return left.quick_score < right.quick_score;
        });
    const std::size_t astar_candidate_count = std::min(
        ranked_side_candidates.size(),
        static_cast<std::size_t>(avoidance_side_astar_candidate_limit_));

    // 首批完整A*不能只由全局quick score决定，否则距离/航向相近的
    // 三个候选可能全部落在同一侧、同一扩展层。优先覆盖首选侧、另一
    // 侧和至少一个扩展候选；上一批全失败后从后续位置继续选择。
    if (astar_candidate_count > 0U) {
      std::vector<std::size_t> selected_indices;
      selected_indices.reserve(astar_candidate_count);
      std::vector<bool> selected(ranked_side_candidates.size(), false);
      const std::size_t selection_offset =
          (side_candidate_retry_batch * astar_candidate_count) %
          ranked_side_candidates.size();
      auto select_first_matching = [&](const auto &predicate) {
        if (selected_indices.size() >= astar_candidate_count) return;
        for (std::size_t offset = 0U;
             offset < ranked_side_candidates.size(); ++offset) {
          const std::size_t index =
              (selection_offset + offset) % ranked_side_candidates.size();
          if (!selected[index] && predicate(ranked_side_candidates[index])) {
            selected[index] = true;
            selected_indices.push_back(index);
            return;
          }
        }
      };

      if (preferred_side != AvoidanceSide::NONE) {
        select_first_matching([&](const RankedSideCandidate &ranked) {
          return ranked.candidate.side == preferred_side;
        });
      }
      if (selected_indices.empty()) {
        select_first_matching(
            [](const RankedSideCandidate &) { return true; });
      }

      if (!selected_indices.empty()) {
        const AvoidanceSide first_side =
            ranked_side_candidates[selected_indices.front()].candidate.side;
        select_first_matching([&](const RankedSideCandidate &ranked) {
          return ranked.candidate.side != AvoidanceSide::NONE &&
                 ranked.candidate.side != first_side;
        });
      }

      while (selected_indices.size() < astar_candidate_count) {
        const std::size_t before = selected_indices.size();
        select_first_matching(
            [](const RankedSideCandidate &) { return true; });
        if (selected_indices.size() == before) break;
      }

      std::vector<RankedSideCandidate> ordered_candidates;
      ordered_candidates.reserve(ranked_side_candidates.size());
      for (const std::size_t index : selected_indices) {
        ordered_candidates.push_back(ranked_side_candidates[index]);
      }
      for (std::size_t index = 0U;
           index < ranked_side_candidates.size(); ++index) {
        if (!selected[index]) {
          ordered_candidates.push_back(ranked_side_candidates[index]);
        }
      }
      ranked_side_candidates = std::move(ordered_candidates);
    }

    std::vector<SideGoalCandidate> side_candidates_for_visualization;
    side_candidates_for_visualization.reserve(ranked_side_candidates.size());
    for (const auto &ranked_candidate : ranked_side_candidates) {
      side_candidates_for_visualization.push_back(ranked_candidate.candidate);
    }
    publishSideGoalCandidates(side_candidates_for_visualization,
                              astar_candidate_count);
    if (!ranked_side_candidates.empty()) {
      RCLCPP_INFO(
          node_->get_logger(),
          "[SIDE_CANDIDATE_PREFILTER] generated=%zu selected=%zu limit=%d "
          "batch=%zu first_score=%.3f first_side=%s "
          "first_heading=%.1fdeg first_endpoint_clearance=%.3f",
          ranked_side_candidates.size(), astar_candidate_count,
          avoidance_side_astar_candidate_limit_, side_candidate_retry_batch,
          ranked_side_candidates.front().quick_score,
          avoidanceSideToString(
              ranked_side_candidates.front().candidate.side),
          ranked_side_candidates.front().direct_heading_error * 180.0 / M_PI,
          ranked_side_candidates.front().estimated_endpoint_clearance);
    }
    if (avoidance_timing_enabled_ && !ranked_side_candidates.empty()) {
      RCLCPP_INFO(node_->get_logger(),
                  "[AVOID_TIMING_PREFILTER] task=%lu batch=%zu "
                  "generated=%zu selected=%zu elapsed_ms=%.1f",
                  static_cast<unsigned long>(task_generation),
                  side_candidate_retry_batch, ranked_side_candidates.size(),
                  astar_candidate_count, elapsedMs(side_batch_start));
    }

    for (std::size_t candidate_index = 0U;
         candidate_index < astar_candidate_count; ++candidate_index) {
      {
        std::lock_guard<std::mutex> lock(avoidance_mutex_);
        if (!avoidance_active_ ||
            avoidance_task_generation_ != task_generation) {
          return false;
        }
      }
      const auto &ranked_candidate = ranked_side_candidates[candidate_index];
      const auto &candidate = ranked_candidate.candidate;
      const AvoidanceSide side = candidate.side;
      CandidateTiming candidate_timing{
          node_->get_logger(), avoidance_timing_enabled_, task_generation,
          side_candidate_retry_batch, candidate_index,
          avoidanceSideToString(side)};
      AStarAvoidanceHint avoidance_hint;
      avoidance_hint.enabled = side != AvoidanceSide::NONE;
      avoidance_hint.obstacle_center = obstacle_center;
      avoidance_hint.start_yaw = robot_yaw;
      avoidance_hint.side_sign =
          side == AvoidanceSide::LEFT
              ? 1
              : (side == AvoidanceSide::RIGHT ? -1 : 0);
      nav_msgs::msg::Path raw_path;
      const bool path_found = timedCheck(candidate_timing.astar_ms, [&] {
        return avoidance_path_planner_->planPath(
            robot_pose.pose.position, candidate.point, raw_path,
            candidate.yaw, avoidance_hint);
      });
      {
        std::lock_guard<std::mutex> lock(avoidance_mutex_);
        if (!avoidance_active_ ||
            avoidance_task_generation_ != task_generation) {
          return false;
        }
      }
      if (!path_found || raw_path.poses.size() < 2U) {
          RCLCPP_INFO(node_->get_logger(),
                      "[SIDE_CANDIDATE_REJECT] side=%s "
                      "reason=ASTAR goal=(%.3f,%.3f)",
                      avoidanceSideToString(side),
                      candidate.point.x, candidate.point.y);
          continue;
        }

        double raw_clearance = 0.0;
        if (!timedCheck(candidate_timing.safety_ms, [&] {
              return avoidance_path_planner_->isPathOutsideObstacleRadius(
                  raw_path, obstacle_center, avoidance_path_clearance_,
                  &raw_clearance, true);
            })) {
          RCLCPP_INFO(node_->get_logger(),
                      "[SIDE_CANDIDATE_REJECT] side=%s reason=CLEARANCE "
                      "goal=(%.3f,%.3f) minimum=%.3f",
                      avoidanceSideToString(side), candidate.point.x,
                      candidate.point.y, raw_clearance);
          continue;
        }

        const bool raw_motion_safe = timedCheck(
            candidate_timing.safety_ms, [&] {
              return avoidance_path_planner_->isPathMotionCollisionFree(
                  raw_path, true);
            });
        nav_msgs::msg::Path candidate_path = raw_path;
        double candidate_clearance = raw_clearance;
        bool smoothed_path_selected = false;
        nav_msgs::msg::Path smoothed_path;
        if (timedCheck(candidate_timing.smoothing_ms, [&] {
              return avoidance_path_planner_->smoothPath(raw_path,
                                                         smoothed_path);
            })) {
          double smoothed_clearance = 0.0;
          if (timedCheck(candidate_timing.safety_ms, [&] {
                return avoidance_path_planner_->isPathOutsideObstacleRadius(
                    smoothed_path, obstacle_center, avoidance_path_clearance_,
                    &smoothed_clearance, true);
              }) &&
              timedCheck(candidate_timing.safety_ms, [&] {
                return avoidance_path_planner_->isPathMotionCollisionFree(
                    smoothed_path, true);
              })) {
            candidate_path = std::move(smoothed_path);
            candidate_clearance = smoothed_clearance;
            smoothed_path_selected = true;
          }
        }
        if (!smoothed_path_selected && !raw_motion_safe) {
          RCLCPP_INFO(node_->get_logger(),
                      "[SIDE_CANDIDATE_REJECT] side=%s "
                      "reason=PATH_MOTION_COLLISION goal=(%.3f,%.3f)",
                      avoidanceSideToString(side), candidate.point.x,
                      candidate.point.y);
          continue;
        }

        // 普通导航的侧方点只是中间航路点，不能像UWB短路径兜底那样在
        // 此处结束任务。继续规划到已经通过安全窗检查的全局回归点，
        // 并对两段拼接后的转向扫掠再做一次完整footprint校验。
        if (static_side_transition_requested) {
          AStarAvoidanceHint return_hint = avoidance_hint;
          return_hint.start_yaw = candidate.yaw;
          nav_msgs::msg::Path return_raw_path;
          if (!timedCheck(candidate_timing.return_astar_ms, [&] {
                return avoidance_path_planner_->planPath(
                    candidate.point, search_result.goal.pose.position,
                    return_raw_path, search_result.goal_yaw, return_hint);
              }) ||
              return_raw_path.poses.size() < 2U) {
            RCLCPP_INFO(
                node_->get_logger(),
                "[SIDE_CANDIDATE_REJECT] side=%s "
                "reason=RETURN_ASTAR transition=(%.3f,%.3f) "
                "return=(%.3f,%.3f)",
                avoidanceSideToString(side), candidate.point.x,
                candidate.point.y, search_result.goal.pose.position.x,
                search_result.goal.pose.position.y);
            continue;
          }

          double return_raw_clearance = 0.0;
          if (!timedCheck(candidate_timing.safety_ms, [&] {
                return avoidance_path_planner_->isPathOutsideObstacleRadius(
                    return_raw_path, obstacle_center,
                    avoidance_path_clearance_, &return_raw_clearance, true);
              })) {
            RCLCPP_INFO(
                node_->get_logger(),
                "[SIDE_CANDIDATE_REJECT] side=%s "
                "reason=RETURN_CLEARANCE transition=(%.3f,%.3f) "
                "return=(%.3f,%.3f) minimum=%.3f",
                avoidanceSideToString(side), candidate.point.x,
                candidate.point.y, search_result.goal.pose.position.x,
                search_result.goal.pose.position.y, return_raw_clearance);
            continue;
          }

          const bool return_raw_motion_safe = timedCheck(
              candidate_timing.safety_ms, [&] {
                return avoidance_path_planner_->isPathMotionCollisionFree(
                    return_raw_path, true);
              });
          nav_msgs::msg::Path return_path = return_raw_path;
          double return_clearance = return_raw_clearance;
          bool return_smoothed_selected = false;
          nav_msgs::msg::Path return_smoothed_path;
          if (timedCheck(candidate_timing.smoothing_ms, [&] {
                return avoidance_path_planner_->smoothPath(
                    return_raw_path, return_smoothed_path);
              })) {
            double return_smoothed_clearance = 0.0;
            if (timedCheck(candidate_timing.safety_ms, [&] {
                  return avoidance_path_planner_->isPathOutsideObstacleRadius(
                      return_smoothed_path, obstacle_center,
                      avoidance_path_clearance_, &return_smoothed_clearance,
                      true);
                }) &&
                timedCheck(candidate_timing.safety_ms, [&] {
                  return avoidance_path_planner_->isPathMotionCollisionFree(
                      return_smoothed_path, true);
                })) {
              return_path = std::move(return_smoothed_path);
              return_clearance = return_smoothed_clearance;
              return_smoothed_selected = true;
            }
          }
          if (!return_smoothed_selected && !return_raw_motion_safe) {
            RCLCPP_INFO(
                node_->get_logger(),
                "[SIDE_CANDIDATE_REJECT] side=%s "
                "reason=RETURN_PATH_MOTION_COLLISION "
                "transition=(%.3f,%.3f) return=(%.3f,%.3f)",
                avoidanceSideToString(side), candidate.point.x,
                candidate.point.y, search_result.goal.pose.position.x,
                search_result.goal.pose.position.y);
            continue;
          }

          nav_msgs::msg::Path combined_path = candidate_path;
          for (std::size_t pose_index = 1U;
               pose_index < return_path.poses.size(); ++pose_index) {
            combined_path.poses.push_back(return_path.poses[pose_index]);
          }
          if (!timedCheck(candidate_timing.safety_ms, [&] {
                return avoidance_path_planner_->isPathMotionCollisionFree(
                    combined_path, true);
              })) {
            RCLCPP_INFO(
                node_->get_logger(),
                "[SIDE_CANDIDATE_REJECT] side=%s "
                "reason=TRANSITION_JOIN_MOTION_COLLISION "
                "transition=(%.3f,%.3f) return=(%.3f,%.3f)",
                avoidanceSideToString(side), candidate.point.x,
                candidate.point.y, search_result.goal.pose.position.x,
                search_result.goal.pose.position.y);
            continue;
          }
          candidate_path = std::move(combined_path);
          candidate_clearance =
              std::min(candidate_clearance, return_clearance);
          smoothed_path_selected =
              smoothed_path_selected && return_smoothed_selected;
        }

        geometry_msgs::msg::Point initial_target =
            candidate_path.poses.back().pose.position;
        for (const auto &pose : candidate_path.poses) {
          if (calculateDistance(robot_pose.pose.position,
                                pose.pose.position) >=
              local_avoidance_lookahead_distance_) {
            initial_target = pose.pose.position;
            break;
          }
        }
        const double initial_path_yaw = std::atan2(
            initial_target.y - robot_pose.pose.position.y,
            initial_target.x - robot_pose.pose.position.x);
        const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
        const double heading_error = std::abs(std::atan2(
            std::sin(initial_path_yaw - robot_yaw),
            std::cos(initial_path_yaw - robot_yaw)));
        if (heading_continuity_required &&
            heading_error > avoidance_replan_max_initial_heading_error_) {
          RCLCPP_INFO(node_->get_logger(),
                      "[SIDE_CANDIDATE_REJECT] side=%s "
                      "reason=HEADING_DISCONTINUITY goal=(%.3f,%.3f) "
                      "heading=%.1fdeg limit=%.1fdeg",
                      avoidanceSideToString(side), candidate.point.x,
                      candidate.point.y, heading_error * 180.0 / M_PI,
                      avoidance_replan_max_initial_heading_error_ *
                          180.0 / M_PI);
          continue;
        }
        bool suppress_alignment = false;
        if (!timedCheck(candidate_timing.safety_ms, [&] {
              return avoidance_path_planner_->isRotationCollisionFree(
                  robot_pose.pose.position, robot_yaw, initial_path_yaw);
            })) {
          suppress_alignment =
              avoidance_path_planner_->isPointInObstacle(
                  robot_pose.pose.position, robot_yaw) &&
              heading_error <= avoidance_initial_escape_max_heading_error_;
          if (!suppress_alignment) {
            RCLCPP_INFO(node_->get_logger(),
                        "[SIDE_CANDIDATE_REJECT] side=%s "
                        "reason=INITIAL_ROTATION goal=(%.3f,%.3f)",
                        avoidanceSideToString(side), candidate.point.x,
                        candidate.point.y);
            continue;
          }
        } else if (heading_continuity_required) {
          // 路径首段已经与当前航向连续且完整旋转扫掠安全，直接交给PP
          // 边走边收敛，避免局部路径失效后再次原地转向。
          suppress_alignment = true;
        }

        const double heading_weight_multiplier =
            heading_continuity_required
                ? avoidance_replan_heading_score_multiplier_
                : 1.0;
        const double score =
            avoidance_side_score_path_length_weight_ *
                pathLength(candidate_path) +
            avoidance_side_score_heading_weight_ *
                heading_weight_multiplier * heading_error +
            avoidance_side_score_turn_weight_ *
                pathTurnAmount(candidate_path) +
            avoidance_side_score_clearance_weight_ /
                std::max(0.05, candidate_clearance) +
            (smoothed_path_selected
                 ? 0.0
                 : avoidance_side_score_raw_path_penalty_) +
            (side == preferred_side
                 ? 0.0
                 : avoidance_side_score_switch_side_penalty_);
        RCLCPP_INFO(node_->get_logger(),
                    "[SIDE_CANDIDATE_VALID] side=%s goal=(%.3f,%.3f) "
                    "edge_lat=%.3f edge_forward=%.3f clearance=%.3f "
                    "heading=%.1fdeg smooth=%d score=%.3f",
                    avoidanceSideToString(side), candidate.point.x,
                    candidate.point.y,
                    candidate.boundary_lateral,
                    candidate.boundary_forward, candidate_clearance,
                    heading_error * 180.0 / M_PI,
                    smoothed_path_selected ? 1 : 0, score);
        candidate_timing.result = "valid_not_selected";
        if (score >= side_fallback_score) continue;

        side_fallback_score = score;
        side_fallback_found = true;
        selected_fallback_side = side;
        side_fallback_path = std::move(candidate_path);
        side_fallback_minimum_clearance = candidate_clearance;
        side_fallback_suppress_alignment = suppress_alignment;
        side_fallback_used_smoothed_path = smoothed_path_selected;
        side_fallback_goal.header = global_path_copy.header;
        side_fallback_goal.header.stamp = node_->now();
        side_fallback_goal.pose.position = candidate.point;
        tf2::Quaternion orientation;
        orientation.setRPY(0.0, 0.0, candidate.yaw);
        side_fallback_goal.pose.orientation = tf2::toMsg(orientation);
        candidate_timing.result = "best_so_far";
    }

    if (avoidance_timing_enabled_ && astar_candidate_count > 0U) {
      RCLCPP_INFO(node_->get_logger(),
                  "[AVOID_TIMING_BATCH] task=%lu batch=%zu attempts=%zu "
                  "valid=%d total_ms=%.1f",
                  static_cast<unsigned long>(task_generation),
                  side_candidate_retry_batch, astar_candidate_count,
                  side_fallback_found, elapsedMs(side_batch_start));
    }

    bool search_committed = false;
    bool preplanned_side_path_selected = false;
    const bool static_side_transition_exhausted =
        static_side_transition_requested && !side_fallback_found;
    bool continue_after_exhausted_static_return = false;
    {
      std::lock_guard<std::mutex> avoidance_lock(avoidance_mutex_);
      if (!generation_is_current_while_locked() ||
          avoidance_temporary_goal_locked_) {
        return false;
      }
      if (!side_fallback_found && astar_candidate_count > 0U) {
        ++avoidance_side_candidate_retry_batch_;
        RCLCPP_WARN(
            node_->get_logger(),
            "[SIDE_CANDIDATE_BATCH_FAILED] batch=%zu selected=%zu; next "
            "retry will use later candidates",
            side_candidate_retry_batch, astar_candidate_count);
      }
      const bool prefer_side_candidate =
          force_side_fallback && side_fallback_found;
      if (prefer_side_candidate) {
        if (static_side_transition_requested) {
          avoidance_temporary_goal_ = search_result.goal;
          avoidance_temporary_goal_index_ =
              search_result.temporary_goal_index;
          avoidance_temporary_goal_on_global_path_ = true;
        } else {
          avoidance_temporary_goal_ = side_fallback_goal;
          avoidance_temporary_goal_index_ =
              std::numeric_limits<std::size_t>::max();
          avoidance_temporary_goal_on_global_path_ = false;
        }
        avoidance_temporary_goal_locked_ = true;
        avoidance_selected_side_ = selected_fallback_side;
        avoidance_force_side_fallback_ = false;
        avoidance_side_candidate_retry_batch_ = 0U;
        avoidance_retained_static_return_valid_ = false;
        avoidance_retained_static_return_goal_ =
            geometry_msgs::msg::PoseStamped();
        avoidance_retained_static_return_index_ =
            std::numeric_limits<std::size_t>::max();
        temporary_goal_revalidation_required_ = false;
        temporary_goal_last_validation_valid_ = true;
        last_validated_costmap_version_ = costmap_version;
        consecutive_temporary_goal_invalid_checks_ = 0;
        avoidance_stage_ = AvoidanceStage::SEARCHING_TEMPORARY_GOAL;
        temporary_goal = avoidance_temporary_goal_;
        temporary_goal_index = avoidance_temporary_goal_index_;
        goal_on_global_path = avoidance_temporary_goal_on_global_path_;
        goal_locked = true;
        search_committed = true;
        preplanned_side_path_selected = true;
      } else if (!static_side_transition_requested &&
          search_result.status == TemporaryGoalSearchStatus::FOUND &&
          search_result.temporary_goal_index < global_path_copy.poses.size()) {
        avoidance_temporary_goal_ = search_result.goal;
        avoidance_temporary_goal_index_ = search_result.temporary_goal_index;
        avoidance_temporary_goal_locked_ = true;
        avoidance_temporary_goal_on_global_path_ = true;
        avoidance_selected_side_ = AvoidanceSide::NONE;
        avoidance_force_side_fallback_ = false;
        avoidance_side_candidate_retry_batch_ = 0U;
        avoidance_retained_static_return_valid_ = false;
        avoidance_retained_static_return_goal_ =
            geometry_msgs::msg::PoseStamped();
        avoidance_retained_static_return_index_ =
            std::numeric_limits<std::size_t>::max();
        avoidance_heading_continuity_required_ = false;
        temporary_goal_revalidation_required_ = false;
        temporary_goal_last_validation_valid_ = true;
        last_validated_costmap_version_ = costmap_version;
        consecutive_temporary_goal_invalid_checks_ = 0;
        avoidance_stage_ = AvoidanceStage::SEARCHING_TEMPORARY_GOAL;
        temporary_goal = avoidance_temporary_goal_;
        temporary_goal_index = avoidance_temporary_goal_index_;
        goal_on_global_path = true;
        goal_locked = true;
        search_committed = true;
      } else if (side_fallback_found) {
        avoidance_temporary_goal_ = side_fallback_goal;
        avoidance_temporary_goal_index_ =
            std::numeric_limits<std::size_t>::max();
        avoidance_temporary_goal_locked_ = true;
        avoidance_temporary_goal_on_global_path_ = false;
        avoidance_selected_side_ = selected_fallback_side;
        avoidance_force_side_fallback_ = false;
        avoidance_side_candidate_retry_batch_ = 0U;
        avoidance_retained_static_return_valid_ = false;
        avoidance_retained_static_return_goal_ =
            geometry_msgs::msg::PoseStamped();
        avoidance_retained_static_return_index_ =
            std::numeric_limits<std::size_t>::max();
        temporary_goal_revalidation_required_ = false;
        temporary_goal_last_validation_valid_ = true;
        last_validated_costmap_version_ = costmap_version;
        consecutive_temporary_goal_invalid_checks_ = 0;
        avoidance_stage_ = AvoidanceStage::SEARCHING_TEMPORARY_GOAL;
        temporary_goal = avoidance_temporary_goal_;
        temporary_goal_index = avoidance_temporary_goal_index_;
        goal_on_global_path = false;
        goal_locked = true;
        search_committed = true;
        preplanned_side_path_selected = true;
      } else if (search_result.status == TemporaryGoalSearchStatus::INVALID_INPUT) {
        avoidance_active_ = false;
        avoidance_completed_ = false;
        avoidance_stage_ = AvoidanceStage::FAILED;
        ++avoidance_task_generation_;
      } else {
        if (static_side_transition_exhausted) {
          // 直达失败后，本周期已尝试把侧方候选连接到该回归点。全部
          // 失败时才正式淘汰这个回归点并继续向后搜索，不能退回同一
          // 目标的直达A*，也不能永久重复相同的一批侧方候选。
          avoidance_retained_static_return_valid_ = false;
          avoidance_retained_static_return_goal_ =
              geometry_msgs::msg::PoseStamped();
          avoidance_retained_static_return_index_ =
              std::numeric_limits<std::size_t>::max();
          avoidance_minimum_goal_index_ = std::max(
              avoidance_minimum_goal_index_,
              search_result.temporary_goal_index < global_path_copy.poses.size()
                  ? search_result.temporary_goal_index + 1U
                  : global_path_copy.poses.size());
          avoidance_force_side_fallback_ = true;
          avoidance_stage_ = AvoidanceStage::SEARCHING_TEMPORARY_GOAL;
          last_retry_costmap_version_ = costmap_version;
          next_temporary_goal_retry_time_ = now;
          ++avoidance_task_generation_;
          continue_after_exhausted_static_return = true;
        } else {
          avoidance_stage_ = AvoidanceStage::WAITING_TEMPORARY_GOAL;
          last_retry_costmap_version_ = costmap_version;
          next_temporary_goal_retry_time_ = now + retry_delay;
        }
        avoidance_corridor_selected_ = false;
        avoidance_temporary_path_.poses.clear();
        std::lock_guard<std::mutex> path_lock(path_mutex_);
        local_path_valid_ = false;
        local_path_.poses.clear();
        previous_local_path_.poses.clear();
        path_stability_counter_ = 0;
      }
    }
    {
      // marker发布也受generation保护。持有状态锁直到发布完成，使reset
      // 后的DELETE不会再被旧搜索结果覆盖。
      std::lock_guard<std::mutex> avoidance_lock(avoidance_mutex_);
      if (generation_is_current_while_locked()) {
        if (search_result.blocked_start_index <
            global_path_copy.poses.size()) {
          publishBlockedPathMarker(
              global_path_copy.poses[search_result.blocked_start_index]
                  .pose.position);
        }
        if (search_committed && avoidance_temporary_goal_locked_ &&
            avoidance_temporary_goal_index_ == temporary_goal_index) {
          // 临时点一旦锁定就立即显示；即便 A* 暂时失败进入等待，RViz
          // 也能区分“目标已选中”和“尚未找到目标”两种状态。
          publishGoalMarker(temporary_goal.pose.position);
        }
      }
    }
    if (!search_committed) {
      if (continue_after_exhausted_static_return) {
        RCLCPP_WARN(
            node_->get_logger(),
            "[STATIC_SIDE_RETURN_EXHAUSTED] return_index=%zu; all selected "
            "side candidates failed, continue searching after index=%zu",
            search_result.temporary_goal_index,
            search_result.temporary_goal_index);
        requestPlanningWakeup();
        return false;
      }
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 500,
          "[TEMP_GOAL_WAIT] search=%s progress=%zu blocked_start=%zu "
          "blocked_end=%zu; waiting for a newer Costmap or retry period",
          temporaryGoalSearchStatusToString(search_result.status),
          search_start_index, search_result.blocked_start_index,
          search_result.blocked_end_index);
      if (avoidance_timing_enabled_) {
        RCLCPP_INFO(node_->get_logger(),
                    "[AVOID_TIMING_WAIT] task=%lu reason=%s "
                    "trigger_to_wait_ms=%.1f batch=%zu",
                    static_cast<unsigned long>(task_generation),
                    temporaryGoalSearchStatusToString(search_result.status),
                    elapsedMs(task_trigger_time), side_candidate_retry_batch);
      }
      return false;
    }
    if (preplanned_side_path_selected) {
      if (goal_on_global_path) {
        RCLCPP_INFO(
            node_->get_logger(),
            "[STATIC_SIDE_RETURN_SELECTED] side=%s transition=(%.3f,%.3f) "
            "return_index=%zu return=(%.3f,%.3f) poses=%zu "
            "clearance=%.3f score=%.3f smooth=%d",
            avoidanceSideToString(selected_fallback_side),
            side_fallback_goal.pose.position.x,
            side_fallback_goal.pose.position.y, temporary_goal_index,
            temporary_goal.pose.position.x, temporary_goal.pose.position.y,
            side_fallback_path.poses.size(),
            side_fallback_minimum_clearance, side_fallback_score,
            side_fallback_used_smoothed_path ? 1 : 0);
      } else {
        RCLCPP_INFO(node_->get_logger(),
                    "[TEMP_GOAL_LOCKED] source=SIDE_FALLBACK side=%s "
                    "goal=(%.3f,%.3f) original_search=%s blocked=[%zu,%zu] "
                    "score=%.3f smooth=%d",
                    avoidanceSideToString(selected_fallback_side),
                    temporary_goal.pose.position.x,
                    temporary_goal.pose.position.y,
                    temporaryGoalSearchStatusToString(search_result.status),
                    search_result.blocked_start_index,
                    search_result.blocked_end_index, side_fallback_score,
                    side_fallback_used_smoothed_path ? 1 : 0);
      }

      // 候选评选阶段已经完成两段A*、平滑、净空和拼接转向检查，直接
      // 原子提交完整路径，不能再丢掉过渡点后重新直达回归目标。
      side_fallback_path.header.stamp = node_->now();
      {
        std::lock_guard<std::mutex> avoidance_lock(avoidance_mutex_);
        if (!generation_is_current_while_locked() ||
            !avoidance_temporary_goal_locked_ ||
            avoidance_temporary_goal_index_ != temporary_goal_index) {
          return false;
        }
        avoidance_temporary_path_ = side_fallback_path;
        avoidance_corridor_selected_ = true;
        avoidance_force_side_fallback_ = false;
        avoidance_heading_continuity_required_ = false;
        avoidance_stage_ = AvoidanceStage::TRACKING_TEMPORARY_GOAL;
        last_path_checked_costmap_version_ = costmap_version;
        last_retry_costmap_version_ = costmap_version;
        consecutive_planning_failures_ = 0;
        temporary_goal_reselection_count_ = 0;
        avoidance_side_candidate_retry_batch_ = 0U;
        consecutive_blocked_path_checks_ = 0;
        std::lock_guard<std::mutex> path_lock(path_mutex_);
        previous_local_path_ = local_path_;
        local_path_ = side_fallback_path;
        local_path_valid_ = true;
        path_stability_counter_ = 0;
        stage_alignment_requested_.store(
            !side_fallback_suppress_alignment);
      }
      {
        std::lock_guard<std::mutex> avoidance_lock(avoidance_mutex_);
        if (!generation_is_current_while_locked()) return false;
        if (local_path_pub_) local_path_pub_->publish(side_fallback_path);
        publishGoalMarker(temporary_goal.pose.position);
        avoidance_path_planner_->publishVisualization(
            robot_pose.pose.position, temporary_goal.pose.position,
            side_fallback_path);
      }
      RCLCPP_INFO(node_->get_logger(),
                  "[SIDE_PATH_SELECTED] mode=%s side=%s goal=(%.3f,%.3f) "
                  "poses=%zu clearance=%.3f score=%.3f",
                  goal_on_global_path ? "STATIC_RETURN" : "DYNAMIC_FALLBACK",
                  avoidanceSideToString(selected_fallback_side),
                  temporary_goal.pose.position.x,
                  temporary_goal.pose.position.y,
                  side_fallback_path.poses.size(),
                  side_fallback_minimum_clearance, side_fallback_score);
      if (avoidance_timing_enabled_) {
        RCLCPP_INFO(node_->get_logger(),
                    "[AVOID_TIMING_READY] task=%lu source=SIDE "
                    "trigger_to_path_ms=%.1f poses=%zu",
                    static_cast<unsigned long>(task_generation),
                    elapsedMs(task_trigger_time),
                    side_fallback_path.poses.size());
      }
      return true;
    }
    if (goal_on_global_path) {
      RCLCPP_INFO(
          node_->get_logger(),
          "[TEMP_GOAL_LOCKED] source=GLOBAL_PATH index=%zu "
          "goal=(%.3f,%.3f) blocked=[%zu,%zu] "
          "temporary_goal_distance=%.3f safe_length=%.3f",
          temporary_goal_index, temporary_goal.pose.position.x,
          temporary_goal.pose.position.y, search_result.blocked_start_index,
          search_result.blocked_end_index,
          search_result.temporary_goal_path_distance,
          search_result.verified_safe_length);
    }
  }

  // WAITING状态中若目标已经锁定，仅在新地图或重试周期到来时对同一目标
  // 再规划；不会重新运行selector并把目标换到另一个路径点。
  if (planning_stage == AvoidanceStage::WAITING_TEMPORARY_GOAL &&
      now < next_retry && costmap_version == last_retry_version) {
    return false;
  }

  nav_msgs::msg::Path planned_path;
  AStarAvoidanceHint avoidance_hint;
  avoidance_hint.enabled = preferred_side != AvoidanceSide::NONE;
  avoidance_hint.obstacle_center = obstacle_center;
  avoidance_hint.start_yaw = tf2::getYaw(robot_pose.pose.orientation);
  avoidance_hint.side_sign =
      preferred_side == AvoidanceSide::LEFT
          ? 1
          : (preferred_side == AvoidanceSide::RIGHT ? -1 : 0);
  const auto global_plan_start = TimingClock::now();
  double global_astar_ms = 0.0;
  double global_safety_ms = 0.0;
  double global_smoothing_ms = 0.0;
  const bool astar_ok = timedCheck(global_astar_ms, [&] {
    return avoidance_path_planner_->planPath(
        robot_pose.pose.position, temporary_goal.pose.position, planned_path,
        tf2::getYaw(temporary_goal.pose.orientation), avoidance_hint);
  });
  bool usable_path = astar_ok && planned_path.poses.size() >= 2U;
  const char *unusable_reason =
      astar_ok ? "ASTAR_PATH_TOO_SHORT" : "ASTAR_SEARCH_FAILED";
  double minimum_obstacle_clearance = 0.0;
  bool raw_motion_safe = false;
  bool suppress_initial_alignment = false;
  if (usable_path) {
    usable_path = timedCheck(global_safety_ms, [&] {
      return avoidance_path_planner_->isPathOutsideObstacleRadius(
          planned_path, obstacle_center, avoidance_path_clearance_,
          &minimum_obstacle_clearance, true);
    });
    if (!usable_path) unusable_reason = "OBSTACLE_CLEARANCE";
    if (usable_path) {
      raw_motion_safe = timedCheck(global_safety_ms, [&] {
        return avoidance_path_planner_->isPathMotionCollisionFree(
            planned_path, true);
      });
    }
  }

  if (usable_path) {
    bool smoothed_path_selected = false;
    nav_msgs::msg::Path smoothed_path;
    if (timedCheck(global_smoothing_ms, [&] {
          return avoidance_path_planner_->smoothPath(planned_path,
                                                     smoothed_path);
        })) {
      double smoothed_clearance = 0.0;
      if (timedCheck(global_safety_ms, [&] {
            return avoidance_path_planner_->isPathOutsideObstacleRadius(
                smoothed_path, obstacle_center, avoidance_path_clearance_,
                &smoothed_clearance, true);
          }) &&
          timedCheck(global_safety_ms, [&] {
            return avoidance_path_planner_->isPathMotionCollisionFree(
                smoothed_path, true);
          })) {
        planned_path = std::move(smoothed_path);
        minimum_obstacle_clearance = smoothed_clearance;
        smoothed_path_selected = true;
      } else {
        RCLCPP_WARN(node_->get_logger(),
                    "[SMOOTH_CLEARANCE_FALLBACK] temporary path "
                    "smooth_min=%.3f required=%.3f; checking raw A*",
                    smoothed_clearance, avoidance_path_clearance_);
      }
    }

    // 平滑路径不可用时，回退到通过完整折点/终点旋转扫掠检查的原始
    // A*；不能仅因各条线段平移无碰撞就把锐角折线路径交给机器人。
    if (!smoothed_path_selected && !raw_motion_safe) {
      usable_path = false;
      unusable_reason = "PATH_MOTION_COLLISION";
      RCLCPP_WARN(node_->get_logger(),
                  "[ASTAR_ROTATION_REJECTED] raw path corner or terminal "
                  "rotation sweeps the footprint into an obstacle");
    }

    if (usable_path) {
      geometry_msgs::msg::Point initial_target =
          planned_path.poses.back().pose.position;
      for (const auto &pose : planned_path.poses) {
        if (calculateDistance(robot_pose.pose.position, pose.pose.position) >=
            local_avoidance_lookahead_distance_) {
          initial_target = pose.pose.position;
          break;
        }
      }
      const double initial_path_yaw = std::atan2(
          initial_target.y - robot_pose.pose.position.y,
          initial_target.x - robot_pose.pose.position.x);
      const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
      const bool initial_rotation_free = timedCheck(global_safety_ms, [&] {
        return avoidance_path_planner_->isRotationCollisionFree(
            robot_pose.pose.position, robot_yaw, initial_path_yaw);
      });
      if (!initial_rotation_free) {
        const double heading_error = std::abs(std::atan2(
            std::sin(initial_path_yaw - robot_yaw),
            std::cos(initial_path_yaw - robot_yaw)));
        const bool starts_in_footprint_overlap =
            avoidance_path_planner_->isPointInObstacle(
                robot_pose.pose.position, robot_yaw);
        // 起点已被保守footprint覆盖时，原地旋转必然无法通过扫掠检查。
        // 仅在局部路径几乎沿当前朝向离开时允许直接平移脱困，并禁止控制
        // 层再次原地对准；大角度转向仍拒绝，不能借脱困逻辑贴障旋转。
        suppress_initial_alignment =
            starts_in_footprint_overlap &&
            heading_error <= avoidance_initial_escape_max_heading_error_;
        usable_path = suppress_initial_alignment;
        if (suppress_initial_alignment) {
          RCLCPP_WARN(node_->get_logger(),
                      "[AVOID_INITIAL_ESCAPE] footprint overlaps at start; "
                      "skip in-place alignment and translate out "
                      "(heading_error=%.1fdeg limit=%.1fdeg)",
                      heading_error * 180.0 / M_PI,
                      avoidance_initial_escape_max_heading_error_ *
                          180.0 / M_PI);
        }
      }
      if (!usable_path) {
        unusable_reason = "INITIAL_ROTATION_BLOCKED";
        RCLCPP_WARN(node_->get_logger(),
                    "[AVOID_ROTATION_BLOCKED] initial turn %.1fdeg -> %.1fdeg "
                    "sweeps the footprint into an obstacle",
                    robot_yaw * 180.0 / M_PI,
                    initial_path_yaw * 180.0 / M_PI);
      }
    }
  }

  if (avoidance_timing_enabled_) {
    RCLCPP_INFO(node_->get_logger(),
                "[AVOID_TIMING_GLOBAL] task=%lu map=%lu result=%s "
                "total_ms=%.1f astar_ms=%.1f safety_ms=%.1f "
                "smoothing_ms=%.1f",
                static_cast<unsigned long>(task_generation),
                static_cast<unsigned long>(costmap_version),
                usable_path ? "valid" : unusable_reason,
                elapsedMs(global_plan_start), global_astar_ms,
                global_safety_ms, global_smoothing_ms);
  }

  if (!usable_path) {
    int failure_count = 0;
    const bool side_fallback_available =
        goal_on_global_path &&
        (alternate_side_enabled || left_clear_at_trigger ||
         right_clear_at_trigger) &&
        ((dynamic_path_mode_.load() &&
          avoidance_dynamic_side_goal_fallback_enabled_) ||
         (!dynamic_path_mode_.load() &&
          avoidance_static_side_transition_enabled_));
    // 全局路径候选已经在当前Costmap上完成终点和后续安全窗验证；若到
    // 该点的完整A*或运动安全校验失败，继续用同一输入重试不会产生新
    // 结果。一次失败后立即丢弃并向后搜索；存在明确空旷侧时，动态跟随
    // 可选择侧方临时点，普通导航则生成“侧后方过渡点->全局回归点”。
    constexpr int failure_limit = 1;
    int reselection_count = 0;
    bool reselect_goal = false;
    AvoidanceSide next_side = preferred_side;
    std::size_t next_search_start = search_start_index;
    std::size_t next_minimum_goal_index = minimum_goal_index;
    {
      std::lock_guard<std::mutex> avoidance_lock(avoidance_mutex_);
      if (!generation_is_current_while_locked() ||
          !avoidance_temporary_goal_locked_ ||
          avoidance_temporary_goal_index_ != temporary_goal_index) {
        return false;
      }
      failure_count = ++consecutive_planning_failures_;
      if (failure_count >= failure_limit) {
        reselect_goal = true;
        ++temporary_goal_reselection_count_;
        reselection_count = temporary_goal_reselection_count_;
        if (avoidance_temporary_goal_on_global_path_ &&
            avoidance_temporary_goal_index_ !=
                std::numeric_limits<std::size_t>::max()) {
          const bool retain_for_static_side_transition =
              !dynamic_path_mode_.load() &&
              avoidance_static_side_transition_enabled_ &&
              side_fallback_available;
          if (retain_for_static_side_transition) {
            avoidance_retained_static_return_valid_ = true;
            avoidance_retained_static_return_goal_ =
                avoidance_temporary_goal_;
            avoidance_retained_static_return_index_ =
                avoidance_temporary_goal_index_;
          } else {
            avoidance_retained_static_return_valid_ = false;
            avoidance_retained_static_return_goal_ =
                geometry_msgs::msg::PoseStamped();
            avoidance_retained_static_return_index_ =
                std::numeric_limits<std::size_t>::max();
          }
          // A*失败只淘汰这个候选，不允许把障碍扫描起点推进到候选后方；
          // 否则下一轮会跳过已经确认的障碍并误报NO_BLOCKED_PATH。普通
          // 导航仍保留目标几何位置，先尝试经侧后方过渡到达该点。
          avoidance_search_start_index_ = std::min(
              avoidance_obstacle_search_start_index_,
              global_path_copy.poses.size() - 1U);
          avoidance_minimum_goal_index_ = std::max(
              avoidance_minimum_goal_index_,
              avoidance_temporary_goal_index_ < global_path_copy.poses.size()
                  ? avoidance_temporary_goal_index_ + 1U
                  : global_path_copy.poses.size());
          if (side_fallback_available) {
            avoidance_force_side_fallback_ = true;
          }
        } else if (avoidance_alternate_side_enabled_ ||
                   (avoidance_left_clear_at_trigger_ &&
                    avoidance_right_clear_at_trigger_)) {
          avoidance_preferred_side_ =
              avoidance_preferred_side_ == AvoidanceSide::LEFT
                  ? AvoidanceSide::RIGHT
                  : AvoidanceSide::LEFT;
        }
        next_search_start = avoidance_search_start_index_;
        next_minimum_goal_index = avoidance_minimum_goal_index_;
        next_side = avoidance_preferred_side_;
        avoidance_temporary_goal_ = geometry_msgs::msg::PoseStamped();
        avoidance_temporary_goal_index_ =
            std::numeric_limits<std::size_t>::max();
        avoidance_temporary_goal_locked_ = false;
        avoidance_temporary_goal_on_global_path_ = false;
        temporary_goal_revalidation_required_ = false;
        temporary_goal_last_validation_valid_ = false;
        consecutive_planning_failures_ = 0;
        consecutive_temporary_goal_invalid_checks_ = 0;
        avoidance_stage_ = AvoidanceStage::SEARCHING_TEMPORARY_GOAL;
        ++avoidance_task_generation_;
        next_temporary_goal_retry_time_ = now;
      } else {
        avoidance_stage_ = AvoidanceStage::WAITING_TEMPORARY_GOAL;
        next_temporary_goal_retry_time_ = now + retry_delay;
      }
      avoidance_corridor_selected_ = false;
      avoidance_temporary_path_.poses.clear();
      last_retry_costmap_version_ = costmap_version;
      std::lock_guard<std::mutex> path_lock(path_mutex_);
      local_path_valid_ = false;
      local_path_.poses.clear();
      previous_local_path_.poses.clear();
      path_stability_counter_ = 0;
    }
    if (reselect_goal) {
      clearAvoidanceMarkers();
      RCLCPP_WARN(
          node_->get_logger(),
          "[TEMP_GOAL_RESELECT] reason=%s failures=%d/%d "
          "reselection=%d obstacle_anchor=%zu min_goal=%zu next_side=%s; "
          "side_fallback=%d heading_continuity=%d; "
          "discarding unreachable goal index=%zu",
          unusable_reason, failure_count, failure_limit,
          reselection_count, next_search_start, next_minimum_goal_index,
          avoidanceSideToString(next_side), side_fallback_available ? 1 : 0,
          heading_continuity_required ? 1 : 0, temporary_goal_index);
      requestPlanningWakeup();
    } else {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 500,
          "[TEMP_GOAL_WAIT] reason=%s A* to locked goal index=%zu failed "
          "(%d/%d); keeping goal until retry limit",
          unusable_reason, temporary_goal_index, failure_count,
          failure_limit);
    }
    return false;
  }

  // 搜索/A*期间可能收到新UWB路径、reset或stall重规划请求。目标、路径和
  // 控制可见快照在固定avoidance->path锁顺序下原子提交。
  planned_path.header.stamp = node_->now();
  {
    std::lock_guard<std::mutex> avoidance_lock(avoidance_mutex_);
    if (!generation_is_current_while_locked() ||
        !avoidance_temporary_goal_locked_ ||
        avoidance_temporary_goal_index_ != temporary_goal_index) {
      return false;
    }
    avoidance_temporary_path_ = planned_path;
    avoidance_corridor_selected_ = true;
    avoidance_force_side_fallback_ = false;
    avoidance_heading_continuity_required_ = false;
    avoidance_stage_ = AvoidanceStage::TRACKING_TEMPORARY_GOAL;
    temporary_goal_revalidation_required_ = false;
    temporary_goal_last_validation_valid_ = true;
    last_validated_costmap_version_ = costmap_version;
    last_path_checked_costmap_version_ = costmap_version;
    last_retry_costmap_version_ = costmap_version;
    consecutive_planning_failures_ = 0;
    temporary_goal_reselection_count_ = 0;
    avoidance_side_candidate_retry_batch_ = 0U;
    consecutive_temporary_goal_invalid_checks_ = 0;
    consecutive_blocked_path_checks_ = 0;
    std::lock_guard<std::mutex> path_lock(path_mutex_);
    previous_local_path_ = local_path_;
    local_path_ = planned_path;
    local_path_valid_ = true;
    path_stability_counter_ = 0;
    stage_alignment_requested_.store(!suppress_initial_alignment);
  }

  {
    // 状态提交后再次核对generation，并在发布期间保持状态锁。这样新全局
    // 路径/reset要么发生在本次发布之前并令其失效，要么发生在发布之后并
    // 紧接着发布空路径和DELETE marker，不会让旧可视化残留在新任务上。
    std::lock_guard<std::mutex> avoidance_lock(avoidance_mutex_);
    if (!generation_is_current_while_locked() ||
        !avoidance_temporary_goal_locked_ ||
        avoidance_temporary_goal_index_ != temporary_goal_index) {
      return false;
    }
    if (local_path_pub_) {
      local_path_pub_->publish(planned_path);
    }
    publishGoalMarker(temporary_goal.pose.position);
    avoidance_path_planner_->publishVisualization(
        robot_pose.pose.position, temporary_goal.pose.position, planned_path);
  }
  RCLCPP_INFO(node_->get_logger(),
              "[TEMP_PATH_LOCKED] generation=%lu goal_index=%zu "
              "goal=(%.3f,%.3f) poses=%zu minimum_clearance=%.3f",
              static_cast<unsigned long>(task_generation),
              temporary_goal_index, temporary_goal.pose.position.x,
              temporary_goal.pose.position.y, planned_path.poses.size(),
              minimum_obstacle_clearance);
  if (avoidance_timing_enabled_) {
    RCLCPP_INFO(node_->get_logger(),
                "[AVOID_TIMING_READY] task=%lu source=GLOBAL "
                "trigger_to_path_ms=%.1f poses=%zu",
                static_cast<unsigned long>(task_generation),
                elapsedMs(task_trigger_time), planned_path.poses.size());
  }
  return true;
}

void LocalAvoidanceManager::markAvoidanceFailed(const char *reason) {
  int failure_count = 0;
  {
    std::lock_guard<std::mutex> lock(avoidance_mutex_);
    if (!avoidance_active_) {
      return;
    }
    failure_count = ++consecutive_planning_failures_;
    if (failure_count < avoidance_max_planning_failures_) {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 500,
          "[AVOID_RETRY] planning failure %d/%d: %s",
          failure_count, avoidance_max_planning_failures_, reason);
      return;
    }
    avoidance_active_ = false;
    avoidance_completed_ = false;
    avoidance_stage_ = AvoidanceStage::FAILED;
  }

  clearLocalPath();
  RCLCPP_ERROR(node_->get_logger(),
               "[AVOID_PLAN] avoidance failed after %d attempts: %s",
               failure_count, reason);
}

void LocalAvoidanceManager::clearLocalPath() {
  std::lock_guard<std::mutex> lock(path_mutex_);
  local_path_valid_ = false;
  local_path_.poses.clear();
  previous_local_path_.poses.clear();
  path_stability_counter_ = 0;
}

bool LocalAvoidanceManager::getRobotPose(
    geometry_msgs::msg::PoseStamped &robot_pose) {
  try {
    geometry_msgs::msg::TransformStamped transform =
        tf_buffer_->lookupTransform(global_frame_, robot_frame_,
                                    tf2::TimePointZero,
                                    tf2::durationFromSec(transform_tolerance_));

    robot_pose.header.frame_id = global_frame_;
    robot_pose.header.stamp = transform.header.stamp;
    robot_pose.pose.position.x = transform.transform.translation.x;
    robot_pose.pose.position.y = transform.transform.translation.y;
    robot_pose.pose.position.z = transform.transform.translation.z;
    robot_pose.pose.orientation = transform.transform.rotation;

    return true;
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "Failed to get robot pose: %s", ex.what());
    return false;
  }
}

const char *
LocalAvoidanceManager::avoidanceSideToString(AvoidanceSide side) const {
  switch (side) {
  case AvoidanceSide::LEFT:
    return "LEFT";
  case AvoidanceSide::RIGHT:
    return "RIGHT";
  default:
    return "NONE";
  }
}

int LocalAvoidanceManager::findClosestPointOnGlobalPath(
    const geometry_msgs::msg::PoseStamped &robot_pose) {
  // 复制全局路径（线程安全）
  nav_msgs::msg::Path global_path_copy;
  {
    std::lock_guard<std::mutex> lock(global_path_mutex_);
    global_path_copy = global_path_;
  }

  if (global_path_copy.poses.empty())
    return -1;

  // 方法1：找到机器人在路径上的投影点
  int best_index = findProjectionOnPath(robot_pose, global_path_copy);

  // 方法2：如果投影失败，使用改进的最近点算法
  if (best_index < 0) {
    best_index = findBestPathPoint(robot_pose, global_path_copy);
  }

  RCLCPP_DEBUG(node_->get_logger(), "Selected path point index: %d",
               best_index);
  return best_index;
}

double
LocalAvoidanceManager::calculateDistance(const geometry_msgs::msg::Point &p1,
                                         const geometry_msgs::msg::Point &p2) {
  return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) +
                   (p1.y - p2.y) * (p1.y - p2.y));
}

int LocalAvoidanceManager::findProjectionOnPath(
    const geometry_msgs::msg::PoseStamped &robot_pose,
    const nav_msgs::msg::Path &path) {
  if (path.poses.size() < 2)
    return -1;

  double min_distance_to_line = std::numeric_limits<double>::max();
  int best_segment_index = -1;

  // 找到机器人到路径线段的最短距离
  for (size_t i = 0; i < path.poses.size() - 1; ++i) {
    double distance = pointToLineDistance(robot_pose.pose.position,
                                          path.poses[i].pose.position,
                                          path.poses[i + 1].pose.position);

    if (distance < min_distance_to_line) {
      min_distance_to_line = distance;
      best_segment_index = i;
    }
  }

  if (best_segment_index < 0)
    return -1;

  // 在最佳线段上找投影点，返回更合适的端点
  geometry_msgs::msg::Point projection = projectPointOnLine(
      robot_pose.pose.position, path.poses[best_segment_index].pose.position,
      path.poses[best_segment_index + 1].pose.position);

  // 判断投影点更接近哪个端点
  double dist_to_start = calculateDistance3D(
      projection, path.poses[best_segment_index].pose.position);
  double dist_to_end = calculateDistance3D(
      projection, path.poses[best_segment_index + 1].pose.position);

  // 考虑路径方向：优先选择前进方向的点
  return (dist_to_end <= dist_to_start) ? best_segment_index + 1
                                        : best_segment_index;
}

int LocalAvoidanceManager::findBestPathPoint(
    const geometry_msgs::msg::PoseStamped &robot_pose,
    const nav_msgs::msg::Path &path) {
  double min_distance = std::numeric_limits<double>::max();
  int closest_index = 0;

  // 考虑路径连续性的权重最近点算法
  for (size_t i = 0; i < path.poses.size(); ++i) {
    // 基础距离（3D）
    double distance_3d = calculateDistance3D(robot_pose.pose.position,
                                             path.poses[i].pose.position);

    // 路径方向性权重：优先选择前进方向的点
    double direction_weight = 1.0;
    if (i > 0) {
      // 计算机器人到当前点的方向向量
      double robot_to_point_x =
          path.poses[i].pose.position.x - robot_pose.pose.position.x;
      double robot_to_point_y =
          path.poses[i].pose.position.y - robot_pose.pose.position.y;

      // 计算路径前进方向向量
      double path_direction_x =
          path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x;
      double path_direction_y =
          path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y;

      // 计算方向一致性
      double dot_product = robot_to_point_x * path_direction_x +
                           robot_to_point_y * path_direction_y;

      if (dot_product > 0) {
        direction_weight = 0.8; // 前进方向，降低权重（优先）
      } else {
        direction_weight = 1.2; // 后退方向，增加权重（惩罚）
      }
    }

    double weighted_distance = distance_3d * direction_weight;

    if (weighted_distance < min_distance) {
      min_distance = weighted_distance;
      closest_index = i;
    }
  }

  return closest_index;
}

double LocalAvoidanceManager::calculateDistance3D(
    const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2) {
  return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) +
                   (p1.y - p2.y) * (p1.y - p2.y) +
                   (p1.z - p2.z) * (p1.z - p2.z));
}

double LocalAvoidanceManager::pointToLineDistance(
    const geometry_msgs::msg::Point &point,
    const geometry_msgs::msg::Point &line_start,
    const geometry_msgs::msg::Point &line_end) {
  // 计算点到线段的最短距离（3D）
  double A = point.x - line_start.x;
  double B = point.y - line_start.y;
  double C = point.z - line_start.z;

  double D = line_end.x - line_start.x;
  double E = line_end.y - line_start.y;
  double F = line_end.z - line_start.z;

  double dot = A * D + B * E + C * F;
  double len_sq = D * D + E * E + F * F;

  if (len_sq == 0) {
    // 线段退化为点
    return calculateDistance3D(point, line_start);
  }

  double param = dot / len_sq;

  geometry_msgs::msg::Point closest;
  if (param < 0) {
    closest = line_start;
  } else if (param > 1) {
    closest = line_end;
  } else {
    closest.x = line_start.x + param * D;
    closest.y = line_start.y + param * E;
    closest.z = line_start.z + param * F;
  }

  return calculateDistance3D(point, closest);
}

geometry_msgs::msg::Point LocalAvoidanceManager::projectPointOnLine(
    const geometry_msgs::msg::Point &point,
    const geometry_msgs::msg::Point &line_start,
    const geometry_msgs::msg::Point &line_end) {
  double A = point.x - line_start.x;
  double B = point.y - line_start.y;
  double C = point.z - line_start.z;

  double D = line_end.x - line_start.x;
  double E = line_end.y - line_start.y;
  double F = line_end.z - line_start.z;

  double dot = A * D + B * E + C * F;
  double len_sq = D * D + E * E + F * F;

  geometry_msgs::msg::Point projection;
  if (len_sq == 0) {
    projection = line_start;
  } else {
    double param = std::max(0.0, std::min(1.0, dot / len_sq));
    projection.x = line_start.x + param * D;
    projection.y = line_start.y + param * E;
    projection.z = line_start.z + param * F;
  }

  return projection;
}

void LocalAvoidanceManager::publishGoalMarker(
    const geometry_msgs::msg::Point &goal_point) {
  if (!goal_marker_pub_) return;
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = global_frame_;
  marker.header.stamp = node_->now();
  marker.ns = "local_goal";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position = goal_point;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = marker.scale.y = marker.scale.z = 0.4;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0; // 蓝色表示局部目标点

  goal_marker_pub_->publish(marker);
}

void LocalAvoidanceManager::publishBlockedPathMarker(
    const geometry_msgs::msg::Point &blocked_point) {
  if (!goal_marker_pub_) return;
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = global_frame_;
  marker.header.stamp = node_->now();
  marker.ns = "blocked_global_path";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position = blocked_point;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = marker.scale.y = marker.scale.z = 0.3;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  goal_marker_pub_->publish(marker);
}

void LocalAvoidanceManager::publishSideGoalCandidates(
    const std::vector<SideGoalCandidate> &candidates,
    std::size_t astar_candidate_count) {
  if (!side_goal_candidates_pub_) return;

  const auto stamp = node_->now();
  visualization_msgs::msg::Marker all_candidates;
  all_candidates.header.frame_id = global_frame_;
  all_candidates.header.stamp = stamp;
  all_candidates.ns = "side_goal_candidates_all";
  all_candidates.id = 0;
  all_candidates.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  all_candidates.action = visualization_msgs::msg::Marker::ADD;
  all_candidates.pose.orientation.w = 1.0;
  all_candidates.scale.x = 0.14;
  all_candidates.scale.y = 0.14;
  all_candidates.scale.z = 0.14;
  all_candidates.points.reserve(candidates.size());
  all_candidates.colors.reserve(candidates.size());
  for (const auto &candidate : candidates) {
    all_candidates.points.push_back(candidate.point);
    std_msgs::msg::ColorRGBA color;
    color.a = 0.85F;
    if (candidate.side == AvoidanceSide::LEFT) {
      color.r = 0.10F;
      color.g = 1.00F;
      color.b = 0.10F;
    } else {
      color.r = 0.10F;
      color.g = 0.40F;
      color.b = 1.00F;
    }
    all_candidates.colors.push_back(color);
  }
  side_goal_candidates_pub_->publish(all_candidates);

  visualization_msgs::msg::Marker astar_candidates;
  astar_candidates.header.frame_id = global_frame_;
  astar_candidates.header.stamp = stamp;
  astar_candidates.ns = "side_goal_candidates_astar";
  astar_candidates.id = 0;
  astar_candidates.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  astar_candidates.action = visualization_msgs::msg::Marker::ADD;
  astar_candidates.pose.orientation.w = 1.0;
  astar_candidates.scale.x = 0.22;
  astar_candidates.scale.y = 0.22;
  astar_candidates.scale.z = 0.22;
  astar_candidates.color.a = 1.0F;
  astar_candidates.color.r = 1.0F;
  astar_candidates.color.g = 0.85F;
  astar_candidates.color.b = 0.0F;
  const std::size_t selected_count =
      std::min(astar_candidate_count, candidates.size());
  astar_candidates.points.reserve(selected_count);
  for (std::size_t index = 0U; index < selected_count; ++index) {
    geometry_msgs::msg::Point point = candidates[index].point;
    point.z += 0.03;
    astar_candidates.points.push_back(point);
  }
  side_goal_candidates_pub_->publish(astar_candidates);
}

void LocalAvoidanceManager::clearAvoidanceMarkers() {
  const auto stamp = node_->now();
  if (goal_marker_pub_) {
    for (const char *marker_namespace : {"local_goal",
                                         "blocked_global_path"}) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = global_frame_;
      marker.header.stamp = stamp;
      marker.ns = marker_namespace;
      marker.id = 0;
      marker.action = visualization_msgs::msg::Marker::DELETE;
      goal_marker_pub_->publish(marker);
    }
  }
  if (side_goal_candidates_pub_) {
    for (const char *marker_namespace : {"side_goal_candidates_all",
                                         "side_goal_candidates_astar"}) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = global_frame_;
      marker.header.stamp = stamp;
      marker.ns = marker_namespace;
      marker.id = 0;
      marker.action = visualization_msgs::msg::Marker::DELETE;
      side_goal_candidates_pub_->publish(marker);
    }
  }
}

double LocalAvoidanceManager::calculatePathSimilarity(
    const nav_msgs::msg::Path &path1, const nav_msgs::msg::Path &path2) {
  if (path1.poses.empty() || path2.poses.empty()) {
    return 0.0;
  }

  // 计算路径相似度基于关键点的距离
  size_t min_size = std::min(path1.poses.size(), path2.poses.size());
  if (min_size == 0)
    return 0.0;

  double total_distance = 0.0;
  double max_allowed_distance = 2.0; // 最大允许距离

  // 采样关键点进行比较（避免计算量过大）
  size_t step =
      std::max(1, static_cast<int>(min_size / 10)); // 最多取10个采样点
  size_t comparison_points = 0;

  for (size_t i = 0; i < min_size; i += step) {
    double dx = path1.poses[i].pose.position.x - path2.poses[i].pose.position.x;
    double dy = path1.poses[i].pose.position.y - path2.poses[i].pose.position.y;
    double distance = sqrt(dx * dx + dy * dy);

    total_distance += distance;
    comparison_points++;

    // 如果某点距离过大，直接返回低相似度
    if (distance > max_allowed_distance) {
      return 0.0;
    }
  }

  if (comparison_points == 0)
    return 0.0;

  double average_distance = total_distance / comparison_points;
  // 距离越小，相似度越高（归一化到0-1）
  double similarity =
      std::max(0.0, 1.0 - (average_distance / max_allowed_distance));

  return similarity;
}

bool LocalAvoidanceManager::shouldKeepPreviousPath(
    const nav_msgs::msg::Path &new_path) {
  nav_msgs::msg::Path comparison_path;
  nav_msgs::msg::Path retained_path;
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    if (previous_local_path_.poses.empty() || !local_path_valid_) {
      path_stability_counter_ = 0;
      return false;
    }
    comparison_path = previous_local_path_;
    retained_path = local_path_;
  }

  if (retained_path.poses.empty()) {
    path_stability_counter_ = 0;
    return false;
  }

  // 计算路径相似度
  double similarity = calculatePathSimilarity(new_path, comparison_path);

  RCLCPP_DEBUG(node_->get_logger(), "Path similarity: %.3f, threshold: %.3f",
               similarity, path_similarity_threshold_);

  // 如果相似度高于阈值，增加稳定性计数器
  if (similarity > path_similarity_threshold_) {
    path_stability_counter_++;

    // 连续多次路径相似，保持之前路径以减少抖动
    if (path_stability_counter_ >= 3) // 连续3次相似就保持稳定
    {
      // 动态障碍可能已经进入旧路径；只有最新Costmap确认安全时才复用。
      if (!avoidance_path_planner_ ||
          !avoidance_path_planner_->isPathCollisionFree(retained_path)) {
        path_stability_counter_ = 0;
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Previous local path is no longer collision-free; "
                             "using replanned path");
        return false;
      }
      path_stability_counter_ = 3; // 避免计数器无限增长
      return true;
    }
  } else {
    // 相似度低，重置计数器
    path_stability_counter_ = 0;
  }

  return false;
}

} // namespace pure_pursuit_local_planner
