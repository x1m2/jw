#include "pure_pursuit/local_avoidance_manager.hpp"

#include <limits>
#include <iterator>
#include <tf2/utils.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "pure_pursuit/param_utils.hpp"

namespace pure_pursuit_local_planner {

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
  avoidance_max_planning_failures_ = declareOrGetParameter(
      node_, "avoidance_max_planning_failures", 15);
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
      1, 10);
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

  AvoidancePathPlannerConfig path_planner_config;
  path_planner_config.lateral_clearance = avoidance_lateral_clearance_;
  path_planner_config.return_goal_clearance = avoidance_return_goal_clearance_;
  path_planner_config.return_obstacle_clearance =
      avoidance_return_obstacle_clearance_;
  path_planner_config.obstacle_threshold = avoidance_obstacle_threshold_;
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
              "Local A* Bezier smoothing: enabled=%d corner=%.3f "
              "spacing=%.3f simplify=%.3f radius_target/min=%.2f/%.2fm "
              "max_deviation=%.3f",
              path_planner_config.bezier_smoothing_enabled ? 1 : 0,
              path_planner_config.bezier_corner_distance,
              path_planner_config.bezier_sample_spacing,
              path_planner_config.bezier_simplify_tolerance,
              path_planner_config.bezier_target_turn_radius,
              path_planner_config.bezier_min_turn_radius,
              path_planner_config.bezier_max_deviation);
  return true;
}

void LocalAvoidanceManager::setGlobalPath(
    const nav_msgs::msg::Path &global_path) {
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
    bool global_controller_aligning, bool safety_stop_blocking) {
  std::lock_guard<std::mutex> decision_lock(decision_mutex_);
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
    if (tracking_local_ && output.mode == AvoidanceTrackingMode::GLOBAL_PATH) {
      output.local_path = getValidLocalPath();
      output.mode = output.local_path ? AvoidanceTrackingMode::LOCAL_PATH
                                      : AvoidanceTrackingMode::WAITING_LOCAL_PATH;
    }
    return output;
  };

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

  // 失败后短暂停车，让Costmap有时间刷新。即使障碍没有完全消失也会
  // 自动重新尝试，避免机器人静止后永远等不到CLEAR的状态机死锁。
  if (failure_latched_) {
    if (observation.decision == ObstacleDecision::CLEAR) {
      failure_latched_ = false;
      has_failure_latch_time_ = false;
      recovery_attempt_ = 0;
      output.failure_changed = true;
      RCLCPP_INFO(node_->get_logger(),
                  "[OBSTACLE_REPLAN] obstacle cleared, failure latch reset");
    } else if (has_failure_latch_time_ &&
               (node_->now() - failure_latch_time_).seconds() >=
                   avoidance_failure_retry_cooldown_) {
      failure_latched_ = false;
      has_failure_latch_time_ = false;
      ++recovery_attempt_;
      output.failure_changed = true;
      RCLCPP_WARN(node_->get_logger(),
                  "[AVOID_RECOVERY] failure cooldown elapsed, retry=%d",
                  recovery_attempt_);
    } else {
      output.mode = AvoidanceTrackingMode::FAILED;
      return finish();
    }
  }

  // 全局路径预对准期间仍允许前方检测启动A*。动态目标大角度变化时，
  // 原地旋转可能把近障碍扫入车头；若在此处跳过，通常要到硬停距离内
  // 才会开始绕障。
  (void)global_controller_aligning;

  // obstacle_stop是最终安全门，之前PP只看到自己发布的规划速度，不知道
  // 实际速度已被清零。绕障指令持续被安全层拦截且位姿没有产生足够变化
  // 时，旧路径已经不可执行：主动切到相反侧重新生成A*，避免无限输出同一
  // 条不可执行路径。
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
      AvoidanceSide stalled_side = AvoidanceSide::NONE;
      geometry_msgs::msg::Point obstacle_center;
      {
        std::lock_guard<std::mutex> lock(avoidance_mutex_);
        stalled_side = avoidance_side_;
        obstacle_center = avoidance_obstacle_center_;
      }
      AvoidanceSide recovery_side =
          stalled_side == AvoidanceSide::LEFT ? AvoidanceSide::RIGHT
                                              : AvoidanceSide::LEFT;
      if (stalled_side == AvoidanceSide::NONE) {
        recovery_side = observation.obstacle_local_y > 0.0
                            ? AvoidanceSide::RIGHT
                            : AvoidanceSide::LEFT;
      }

      RCLCPP_WARN(
          node_->get_logger(),
          "[AVOID_STALL_RECOVERY] obstacle_stop blocked local motion for "
          "%.2fs (moved=%.3fm yaw_change=%.1fdeg); switching side %d -> %d",
          avoidance_stall_timeout_, moved, yaw_change * 180.0 / M_PI,
          static_cast<int>(stalled_side), static_cast<int>(recovery_side));
      stall_monitoring_ = false;
      tracking_local_ = false;
      ++recovery_attempt_;
      resetAvoidance();
      if (startAvoidance(robot_pose, obstacle_center, recovery_side)) {
        tracking_local_ = true;
        output.realign_local_path = true;
        output.entered_local_tracking = true;
        return finish();
      }

      failure_latched_ = true;
      failure_latch_time_ = node_->now();
      has_failure_latch_time_ = true;
      output.failure_changed = true;
      output.exited_local_tracking = true;
      output.mode = AvoidanceTrackingMode::FAILED;
      return finish();
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
      last_avoidance_complete_time_ = node_->now();
      has_last_avoidance_complete_time_ = true;
      recovery_attempt_ = 0;
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

    if (observation.decision == ObstacleDecision::BLOCKED) {
      if (avoidance_path_planner_ &&
          avoidance_path_planner_->hasPreparedCostmap()) {
        AvoidanceSide recovery_side = AvoidanceSide::LEFT;
        if (observation.left_clear != observation.right_clear) {
          recovery_side = observation.left_clear ? AvoidanceSide::LEFT
                                                  : AvoidanceSide::RIGHT;
        } else if (observation.obstacle_local_y > 0.05) {
          recovery_side = AvoidanceSide::RIGHT;
        } else if (observation.obstacle_local_y < -0.05) {
          recovery_side = AvoidanceSide::LEFT;
        } else if (recovery_attempt_ % 2 != 0) {
          recovery_side = AvoidanceSide::RIGHT;
        }
        RCLCPP_WARN(
            node_->get_logger(),
            "[AVOID_RECOVERY] BLOCKED at dist=%.3f, trying side=%d "
            "despite detector passage result left=%d right=%d local_y=%.3f",
            observation.obstacle_distance, static_cast<int>(recovery_side),
            observation.left_clear, observation.right_clear,
            observation.obstacle_local_y);

        bool recovery_started = startAvoidance(
            robot_pose, observation.obstacle_center, recovery_side);
        if (!recovery_started) {
          const AvoidanceSide alternate_side =
              recovery_side == AvoidanceSide::LEFT ? AvoidanceSide::RIGHT
                                                    : AvoidanceSide::LEFT;
          RCLCPP_WARN(node_->get_logger(),
                      "[AVOID_RECOVERY] side=%d failed, trying alternate "
                      "side=%d",
                      static_cast<int>(recovery_side),
                      static_cast<int>(alternate_side));
          recovery_started = startAvoidance(
              robot_pose, observation.obstacle_center, alternate_side);
        }

        if (recovery_started) {
          tracking_local_ = true;
          output.entered_local_tracking = true;
          RCLCPP_INFO(node_->get_logger(),
                      "[AVOID_RECOVERY] local A* recovery started");
          return finish();
        }
      }

      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 500,
          "[AVOID_STOP] obstacle BLOCKED; no A* start, dist=%.3f "
          "left=%d right=%d",
          observation.obstacle_distance, observation.left_clear,
          observation.right_clear);
      output.mode = AvoidanceTrackingMode::STOPPED_BLOCKED;
      return finish();
    }

    if (observation.decision == ObstacleDecision::AVOIDABLE) {
      // 首张Costmap可能已经到达检测器，但A*的完整地图/footprint快照仍在
      // 规划线程中准备。此时只停车等待，不能误锁存为规划失败。
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

      // 两侧均可通行时，恢复尝试交替选择左右侧，避免重复撞在同一个
      // 不可行的固定走廊上。
      AvoidanceSide side = AvoidanceSide::NONE;
      if (observation.left_clear && observation.right_clear) {
        side = recovery_attempt_ % 2 == 0 ? AvoidanceSide::LEFT
                                         : AvoidanceSide::RIGHT;
      } else if (observation.left_clear) {
        side = AvoidanceSide::LEFT;
      } else if (observation.right_clear) {
        side = AvoidanceSide::RIGHT;
      }

      RCLCPP_INFO(
          node_->get_logger(),
          "[AVOID_START] A* trigger accepted: dist=%.3f side=%d "
          "left=%d right=%d",
          observation.obstacle_distance, static_cast<int>(side),
          observation.left_clear, observation.right_clear);

      bool avoidance_started =
          startAvoidance(robot_pose, observation.obstacle_center, side);
      if (!avoidance_started) {
        AvoidanceSide alternate_side = AvoidanceSide::NONE;
        if (side == AvoidanceSide::LEFT && observation.right_clear) {
          alternate_side = AvoidanceSide::RIGHT;
        } else if (side == AvoidanceSide::RIGHT && observation.left_clear) {
          alternate_side = AvoidanceSide::LEFT;
        }

        if (alternate_side != AvoidanceSide::NONE) {
          RCLCPP_WARN(node_->get_logger(),
                      "[AVOID_RETRY] preferred side=%d failed, retrying "
                      "alternate side=%d",
                      static_cast<int>(side),
                      static_cast<int>(alternate_side));
          avoidance_started = startAvoidance(
              robot_pose, observation.obstacle_center, alternate_side);
        }
      }

      if (!avoidance_started) {
        failure_latched_ = true;
        failure_latch_time_ = node_->now();
        has_failure_latch_time_ = true;
        output.failure_changed = true;
        output.mode = AvoidanceTrackingMode::FAILED;
        resetAvoidance();
        RCLCPP_WARN(node_->get_logger(),
              "[AVOID_FAIL] A* not started: could not build side goal");
        return finish();
      }

      tracking_local_ = true;
      output.entered_local_tracking = true;
      RCLCPP_INFO(node_->get_logger(),
                  "[OBSTACLE_REPLAN] active, tracking local A* path");
    }
  }

  return finish();
}

void LocalAvoidanceManager::reset() {
  std::lock_guard<std::mutex> decision_lock(decision_mutex_);
  tracking_local_ = false;
  failure_latched_ = false;
  has_failure_latch_time_ = false;
  recovery_attempt_ = 0;
  has_last_avoidance_complete_time_ = false;
  stall_monitoring_ = false;
  stage_alignment_requested_.store(false);
  resetAvoidance();
}

bool LocalAvoidanceManager::isTrackingLocal() const {
  std::lock_guard<std::mutex> decision_lock(decision_mutex_);
  return tracking_local_;
}

bool LocalAvoidanceManager::startAvoidance(
    const geometry_msgs::msg::PoseStamped &robot_pose,
    const geometry_msgs::msg::Point &obstacle_center, AvoidanceSide side) {
  if (side == AvoidanceSide::NONE) {
    RCLCPP_WARN(node_->get_logger(),
                "[AVOID_FAIL] A* not started: no clear avoidance side");
    return false;
  }
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

  geometry_msgs::msg::Point first_side_goal;
  double side_goal_yaw = 0.0;
  // Manager只准备本次任务快照，侧向目标的几何计算交给路径计算层。
  nav_msgs::msg::Path global_path_copy;
  {
    std::lock_guard<std::mutex> lock(global_path_mutex_);
    global_path_copy = global_path_;
  }
  // 在全局路径上找到机器人最近点索引，作为侧向目标计算的参考。
  const int closest_index = findClosestPointOnGlobalPath(robot_pose);
  RCLCPP_INFO(
      node_->get_logger(),
      "[AVOID_START] side=%d obstacle=(%.3f,%.3f) closest_index=%d "
      "global_poses=%zu",
      static_cast<int>(side), obstacle_center.x, obstacle_center.y,
      closest_index, global_path_copy.poses.size());
  // 每次绕障只生成一个侧向临时点，不扩展候选、不做评分。
  const double base_lateral_clearance = std::max(
      avoidance_lateral_clearance_,
      avoidance_near_stop_clearance_ + avoidance_near_stop_margin_) +
      std::min(0.45, 0.15 * static_cast<double>(recovery_attempt_ / 2));
  const bool have_seed = avoidance_path_planner_->buildSideGoal(
      robot_pose, obstacle_center, side, global_path_copy, closest_index,
      first_side_goal, side_goal_yaw, base_lateral_clearance,
      avoidance_max_entry_angle_);
  if (!have_seed) {
    std::lock_guard<std::mutex> lock(avoidance_mutex_);
    avoidance_active_ = false;
    avoidance_completed_ = false;
    avoidance_stage_ = AvoidanceStage::FAILED;
    avoidance_obstacle_center_ = obstacle_center;
    local_path_valid_ = false;
    RCLCPP_WARN(node_->get_logger(),
                "[AVOID_FAIL] A* not started: side goal generation failed");
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(avoidance_mutex_);
    avoidance_active_ = true;
    avoidance_completed_ = false;
    avoidance_stage_ = AvoidanceStage::FIRST_SIDE_GOAL;
    avoidance_obstacle_center_ = obstacle_center;
    avoidance_first_side_goal_ = first_side_goal;
    avoidance_side_goal_yaw_ = side_goal_yaw;
    avoidance_return_goal_ = geometry_msgs::msg::Point();
    avoidance_return_goal_precomputed_ = false;
    avoidance_side_ = side;
    avoidance_corridor_selected_ = false;
    consecutive_planning_failures_ = 0;
    consecutive_blocked_path_checks_ = 0;
    avoidance_side_path_.poses.clear();
    avoidance_return_path_.poses.clear();
  }

  clearLocalPath();

  RCLCPP_INFO(node_->get_logger(),
              "[AVOID_PLAN] Started %s avoidance: obstacle=(%.3f, %.3f), "
              "side_goal=(%.3f, %.3f)",
              avoidanceSideToString(side), obstacle_center.x, obstacle_center.y,
              first_side_goal.x, first_side_goal.y);
  return true;
}

void LocalAvoidanceManager::resetAvoidance() {
  stage_alignment_requested_.store(false);
  {
    std::lock_guard<std::mutex> avoidance_lock(avoidance_mutex_);
    avoidance_active_ = false;
    avoidance_completed_ = false;
    avoidance_stage_ = AvoidanceStage::IDLE;
    avoidance_obstacle_center_ = geometry_msgs::msg::Point();
    avoidance_return_goal_precomputed_ = false;
    avoidance_side_ = AvoidanceSide::NONE;
    avoidance_corridor_selected_ = false;
    consecutive_planning_failures_ = 0;
    consecutive_blocked_path_checks_ = 0;
    avoidance_side_path_.poses.clear();
    avoidance_return_path_.poses.clear();
  }
  clearLocalPath();
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

void LocalAvoidanceManager::stop() {
  running_ = false;
  if (planning_thread_.joinable()) {
    planning_thread_.join();
  }
}

void LocalAvoidanceManager::planningLoop() {
  rclcpp::Rate rate(planning_frequency_);

  while (running_ && rclcpp::ok()) {
    try {
      // Costmap回调只提交最新指针；耗时的footprint构建固定在规划线程执行。
      if (avoidance_path_planner_) {
        avoidance_path_planner_->prepareLatestCostmap();
      }
      const bool planning_result = computeLocalPath();
      RCLCPP_DEBUG_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "[AVOID_THREAD] computeLocalPath result=%d local_valid=%d",
          planning_result, local_path_valid_.load());
    } catch (const std::exception &e) {
      RCLCPP_ERROR(node_->get_logger(), "Exception in local path planning: %s",
                   e.what());
    }

    rate.sleep();
  }
}

bool LocalAvoidanceManager::computeLocalPath() {
  // 本函数保留原有状态推进顺序；耗时的路径计算通过
  // AvoidancePathPlanner完成，Manager只提交结果和更新状态。
  // 检查必要的数据是否可用
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
    local_path_valid_ = false;
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(avoidance_mutex_);
    if (!avoidance_active_) {
      RCLCPP_DEBUG_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "[AVOID_THREAD_SKIP] A* not called: avoidance_active=false");
      bool should_publish_empty = false;
      {
        std::lock_guard<std::mutex> path_lock(path_mutex_);
        should_publish_empty = local_path_valid_ ||
                               !local_path_.poses.empty() ||
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
  }

  // 获取机器人当前位姿
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!getRobotPose(robot_pose)) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "[AVOID_THREAD_SKIP] A* not called: failed to get robot pose");
    local_path_valid_ = false;
    return false;
  }

  geometry_msgs::msg::Point goal_point;
  bool using_avoidance_goal = false;
  bool local_segment_changed = false;
  // Costmap更新可能使当前阶段已经锁定的路径失效。此时新A*路径仍属于
  // 同一阶段，但控制层保存的旧路径索引同样必须清零。
  bool same_stage_path_replanned = false;
  bool plan_return_path = false;
  {
    std::lock_guard<std::mutex> lock(avoidance_mutex_);

    if (avoidance_active_ &&
        avoidance_stage_ == AvoidanceStage::FIRST_SIDE_GOAL) {
      const double first_goal_distance = calculateDistance(
          robot_pose.pose.position, avoidance_first_side_goal_);
      RCLCPP_DEBUG_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 500,
          "[AVOID_PROGRESS] stage=FIRST_SIDE_GOAL, robot=(%.3f, %.3f), "
          "goal=(%.3f, %.3f), dist=%.3f, tolerance=%.3f",
          robot_pose.pose.position.x, robot_pose.pose.position.y,
          avoidance_first_side_goal_.x, avoidance_first_side_goal_.y,
          first_goal_distance, avoidance_goal_tolerance_);

      // 第一阶段只按到临时点的欧氏距离完成。禁止使用X值或投影越过判断，
      // 避免机器人横向尚未到位就提前切换到回归段。
      if (first_goal_distance <= avoidance_goal_tolerance_) {
        if (dynamic_path_mode_.load()) {
          avoidance_active_ = false;
          avoidance_completed_ = false;
          avoidance_stage_ = AvoidanceStage::WAITING_DYNAMIC_PATH;
          clearLocalPath();
          RCLCPP_INFO(node_->get_logger(),
                      "[AVOID_PLAN] Side goal reached, waiting for "
                      "a fresh dynamic path");
          return false;
        }
        // 静态路径的回归段到达障碍侧方后再规划。此时机器人视野和
        // Costmap已经前移，比在障碍前预估整条回归走廊可靠得多。
        avoidance_stage_ = AvoidanceStage::RETURN_PATH_PLANNING;
        avoidance_return_goal_precomputed_ = false;
        avoidance_corridor_selected_ = false;
        avoidance_return_path_.poses.clear();
        local_segment_changed = true;
        plan_return_path = true;
        RCLCPP_INFO(node_->get_logger(),
                    "[AVOID_PLAN] Side goal reached, planning return path "
                    "from the latest robot pose");
      }
    }

    if (avoidance_active_ && avoidance_stage_ == AvoidanceStage::RETURN_GOAL) {
      const double return_goal_distance =
          calculateDistance(robot_pose.pose.position, avoidance_return_goal_);
      RCLCPP_DEBUG_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 500,
          "[AVOID_PROGRESS] stage=RETURN_GOAL, robot=(%.3f, %.3f), "
          "return_goal=(%.3f, %.3f), dist=%.3f, tolerance=%.3f",
          robot_pose.pose.position.x, robot_pose.pose.position.y,
          avoidance_return_goal_.x, avoidance_return_goal_.y,
          return_goal_distance, avoidance_goal_tolerance_);

      if (return_goal_distance <= avoidance_goal_tolerance_) {
        avoidance_active_ = false;
        avoidance_completed_ = true;
        avoidance_stage_ = AvoidanceStage::COMPLETED;
        clearLocalPath();
        RCLCPP_INFO(node_->get_logger(),
                    "[AVOID_PLAN] Return goal reached, avoidance completed");
        return false;
      }
    }

    if (avoidance_active_ &&
        avoidance_stage_ == AvoidanceStage::FIRST_SIDE_GOAL) {
      goal_point = avoidance_first_side_goal_;
      using_avoidance_goal = true;
    } else if (avoidance_active_ &&
               avoidance_stage_ == AvoidanceStage::RETURN_GOAL) {
      goal_point = avoidance_return_goal_;
      using_avoidance_goal = true;
    } else if (avoidance_active_ &&
               avoidance_stage_ == AvoidanceStage::RETURN_PATH_PLANNING) {
      plan_return_path = true;
    }
  }

  if (plan_return_path) {
    if (local_segment_changed) clearLocalPath();

    nav_msgs::msg::Path global_path_copy;
    geometry_msgs::msg::Point obstacle_center;
    {
      std::lock_guard<std::mutex> lock(global_path_mutex_);
      global_path_copy = global_path_;
    }
    {
      std::lock_guard<std::mutex> lock(avoidance_mutex_);
      obstacle_center = avoidance_obstacle_center_;
    }

    const int closest_index = findClosestPointOnGlobalPath(robot_pose);
    geometry_msgs::msg::Point return_goal;
    nav_msgs::msg::Path return_path;
    bool used_direct_path = false;
    const bool return_ok = avoidance_path_planner_->buildReturnPath(
        robot_pose.pose.position, closest_index, obstacle_center,
        global_path_copy, return_goal, return_path,
        avoidance_max_return_angle_, avoidance_path_clearance_,
        used_direct_path);
    if (!return_ok || return_path.poses.empty()) {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 500,
          "[AVOID_RETRY] return path unavailable from current Costmap");
      markAvoidanceFailed("return path unavailable");
      local_path_valid_ = false;
      return false;
    }

    geometry_msgs::msg::Point initial_return_target =
        return_path.poses.back().pose.position;
    for (const auto &pose : return_path.poses) {
      if (calculateDistance(robot_pose.pose.position, pose.pose.position) >=
          local_avoidance_lookahead_distance_) {
        initial_return_target = pose.pose.position;
        break;
      }
    }
    const double initial_return_yaw = std::atan2(
        initial_return_target.y - robot_pose.pose.position.y,
        initial_return_target.x - robot_pose.pose.position.x);
    const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
    if (!avoidance_path_planner_->isRotationCollisionFree(
            robot_pose.pose.position, robot_yaw, initial_return_yaw)) {
      RCLCPP_WARN(node_->get_logger(),
                  "[AVOID_ROTATION_BLOCKED] return turn %.1fdeg -> %.1fdeg "
                  "sweeps the footprint into an obstacle",
                  robot_yaw * 180.0 / M_PI,
                  initial_return_yaw * 180.0 / M_PI);
      markAvoidanceFailed("return rotation blocked");
      local_path_valid_ = false;
      return false;
    }

    {
      std::lock_guard<std::mutex> lock(avoidance_mutex_);
      if (!avoidance_active_ ||
          avoidance_stage_ != AvoidanceStage::RETURN_PATH_PLANNING) {
        return false;
      }
      avoidance_return_goal_ = return_goal;
      avoidance_return_goal_precomputed_ = true;
      avoidance_return_path_ = return_path;
      avoidance_corridor_selected_ = true;
      avoidance_stage_ = AvoidanceStage::RETURN_GOAL;
      consecutive_planning_failures_ = 0;
    }
    goal_point = return_goal;
    using_avoidance_goal = true;
    local_segment_changed = true;
    RCLCPP_INFO(node_->get_logger(),
                "[AVOID_PLAN] return path selected from side position: "
                "goal=(%.3f, %.3f), mode=%s",
                return_goal.x, return_goal.y,
                used_direct_path ? "DIRECT" : "ASTAR");
  }

  if (!using_avoidance_goal) {
    clearLocalPath();
    return false;
  }

  auto get_costmap_cost =
      [this](const geometry_msgs::msg::Point &point) -> int {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    if (!costmap_received_ || !costmap_ || costmap_->info.resolution <= 0.0) {
      return -999;
    }

    const int grid_x = static_cast<int>(
        (point.x - costmap_->info.origin.position.x) / costmap_->info.resolution);
    const int grid_y = static_cast<int>(
        (point.y - costmap_->info.origin.position.y) / costmap_->info.resolution);

    if (grid_x < 0 || grid_y < 0 ||
        grid_x >= static_cast<int>(costmap_->info.width) ||
        grid_y >= static_cast<int>(costmap_->info.height)) {
      return -998;
    }

    const int index = grid_y * static_cast<int>(costmap_->info.width) + grid_x;
    if (index < 0 || index >= static_cast<int>(costmap_->data.size())) {
      return -997;
    }

    return static_cast<int>(costmap_->data[index]);
  };

  const bool start_in_obstacle =
      !avoidance_path_planner_ ||
      avoidance_path_planner_->isPointInObstacle(robot_pose.pose.position);
  const bool goal_in_obstacle =
      !avoidance_path_planner_ ||
      avoidance_path_planner_->isPointInObstacle(goal_point);
  const int start_cost = get_costmap_cost(robot_pose.pose.position);
  const int goal_cost = get_costmap_cost(goal_point);
  RCLCPP_DEBUG_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "[LOCAL_PLAN_DEBUG] mode=%s, robot=(%.3f, %.3f), goal=(%.3f, %.3f), "
      "start_obstacle=%d, goal_obstacle=%d, start_cost=%d, goal_cost=%d",
      using_avoidance_goal ? "AVOIDANCE" : "GLOBAL_LOOKAHEAD",
      robot_pose.pose.position.x, robot_pose.pose.position.y, goal_point.x,
      goal_point.y, start_in_obstacle ? 1 : 0, goal_in_obstacle ? 1 : 0,
      start_cost, goal_cost);

  // 两段A*只规划一次并分别锁定，控制器一次只接收当前阶段对应的路径。
  nav_msgs::msg::Path new_local_path;
  bool planning_success = false;
  geometry_msgs::msg::Point published_goal = goal_point;
  AvoidanceStage planning_stage = AvoidanceStage::IDLE;
  geometry_msgs::msg::Point first_side_goal;
  geometry_msgs::msg::Point return_goal;
  double side_goal_yaw = 0.0;
  geometry_msgs::msg::Point obstacle_center;
  nav_msgs::msg::Path selected_segment;
  bool corridor_selected = false;
  {
    std::lock_guard<std::mutex> lock(avoidance_mutex_);
    planning_stage = avoidance_stage_;
    first_side_goal = avoidance_first_side_goal_;
    return_goal = avoidance_return_goal_;
    side_goal_yaw = avoidance_side_goal_yaw_;
    obstacle_center = avoidance_obstacle_center_;
    selected_segment = planning_stage == AvoidanceStage::RETURN_GOAL
                           ? avoidance_return_path_
                           : avoidance_side_path_;
    corridor_selected = avoidance_corridor_selected_;
  }

  // A*段虽然在生成时通过了碰撞检查，但Costmap仍会持续变化。只检查从
  // 机器人当前位置开始的剩余段，避免已经驶过的路径被后方障碍误判；
  // 同时把机器人当前位姿作为首点，确保“机器人已偏离旧路径且前方被
  // 安全层挡住”的情况也能触发重新规划。
  if (avoidance_path_planner_ && corridor_selected &&
      !selected_segment.poses.empty()) {
    std::size_t closest_pose = 0U;
    double closest_distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < selected_segment.poses.size(); ++i) {
      const double distance = calculateDistance(
          robot_pose.pose.position, selected_segment.poses[i].pose.position);
      if (distance < closest_distance) {
        closest_distance = distance;
        closest_pose = i;
      }
    }

    nav_msgs::msg::Path remaining_path;
    remaining_path.header = selected_segment.header;
    remaining_path.poses.push_back(robot_pose);
    const std::size_t copy_start = closest_pose > 0U ? closest_pose - 1U : 0U;
    remaining_path.poses.insert(
        remaining_path.poses.end(),
        selected_segment.poses.begin() + static_cast<std::ptrdiff_t>(copy_start),
        selected_segment.poses.end());

    const bool initial_escape =
        planning_stage == AvoidanceStage::FIRST_SIDE_GOAL;
    const bool remaining_path_safe =
        initial_escape
            ? avoidance_path_planner_->isInitialEscapePathCollisionFree(
                  remaining_path, obstacle_center, avoidance_path_clearance_)
            : avoidance_path_planner_->isPathCollisionFree(remaining_path);
    if (!remaining_path_safe) {
      const int blocked_checks =
          consecutive_blocked_path_checks_.fetch_add(1) + 1;
      if (blocked_checks < avoidance_path_blocked_confirmations_) {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 500,
            "[AVOID_PATH_UNSTABLE] latest Costmap blocked local path "
            "%d/%d; retaining path pending confirmation",
            blocked_checks,
            avoidance_path_blocked_confirmations_);
        return true;
      }
      consecutive_blocked_path_checks_ = 0;
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 500,
          "[AVOID_REPLAN_LOCAL] locked local path is blocked on latest "
          "Costmap; stage=%d closest=%zu distance=%.3f",
          static_cast<int>(planning_stage), closest_pose, closest_distance);
      {
        std::lock_guard<std::mutex> lock(avoidance_mutex_);
        if (avoidance_active_ && avoidance_stage_ == planning_stage) {
          avoidance_corridor_selected_ = false;
          if (planning_stage == AvoidanceStage::FIRST_SIDE_GOAL) {
            avoidance_side_path_.poses.clear();
          } else if (planning_stage == AvoidanceStage::RETURN_GOAL) {
            avoidance_return_path_.poses.clear();
            avoidance_return_goal_precomputed_ = false;
            avoidance_stage_ = AvoidanceStage::RETURN_PATH_PLANNING;
          }
        }
      }
      clearLocalPath();
      selected_segment.poses.clear();
      corridor_selected = false;
      if (planning_stage == AvoidanceStage::RETURN_GOAL) {
        return false;
      }
      same_stage_path_replanned = true;
    } else {
      consecutive_blocked_path_checks_ = 0;
    }
  }

  // 阶段1只发布当前位置到临时点；阶段2只发布临时点到回归点。
  if (avoidance_path_planner_ && corridor_selected &&
      !selected_segment.poses.empty()) {
    new_local_path = selected_segment;
    new_local_path.header.stamp = node_->now();
    planning_success = true;
    published_goal = planning_stage == AvoidanceStage::RETURN_GOAL
                         ? return_goal
                         : first_side_goal;
  }

  if (!planning_success && avoidance_path_planner_ &&
      planning_stage == AvoidanceStage::FIRST_SIDE_GOAL) {
    if (!corridor_selected) {
      nav_msgs::msg::Path first_path;
      const bool first_ok = avoidance_path_planner_->planPath(
          robot_pose.pose.position, first_side_goal, first_path,
          side_goal_yaw);
      RCLCPP_INFO(
          node_->get_logger(),
          "[AVOID_ASTAR] first segment called: result=%d path_poses=%zu "
          "start=(%.3f,%.3f) goal=(%.3f,%.3f)",
          first_ok, first_path.poses.size(), robot_pose.pose.position.x,
          robot_pose.pose.position.y, first_side_goal.x, first_side_goal.y);
      if (first_ok && first_path.poses.size() >= 2) {
        double minimum_obstacle_clearance = 0.0;
        const bool raw_path_safe =
            avoidance_path_planner_->isPathOutsideObstacleRadius(
                first_path, obstacle_center, avoidance_path_clearance_,
                &minimum_obstacle_clearance, true);
        if (!raw_path_safe) {
          RCLCPP_WARN(node_->get_logger(),
                      "[AVOID_PLAN] side path rejected: "
                      "minimum_clearance=%.2f required=%.2f",
                      minimum_obstacle_clearance, avoidance_path_clearance_);
        } else {
          nav_msgs::msg::Path smoothed_first_path;
          if (avoidance_path_planner_->smoothPathWithBezier(
                  first_path, smoothed_first_path)) {
            double smoothed_minimum_clearance = 0.0;
            if (avoidance_path_planner_->isPathOutsideObstacleRadius(
                    smoothed_first_path, obstacle_center,
                    avoidance_path_clearance_, &smoothed_minimum_clearance,
                    true)) {
              first_path = std::move(smoothed_first_path);
              minimum_obstacle_clearance = smoothed_minimum_clearance;
            } else {
              RCLCPP_WARN(
                  node_->get_logger(),
                  "[BEZIER_CLEARANCE_FALLBACK] side path smooth_min=%.3f "
                  "required=%.3f; using raw A*",
                  smoothed_minimum_clearance, avoidance_path_clearance_);
            }
          }
          // 控制器刚接管局部路径时会朝固定短前瞻点转向。非全向长车体
          // 必须在规划阶段检查这次原地转向的完整角点扫掠，而不只检查
          // A*平移路径上的起止朝向。
          geometry_msgs::msg::Point initial_target =
              first_path.poses.back().pose.position;
          for (const auto &pose : first_path.poses) {
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
          if (!avoidance_path_planner_->isRotationCollisionFree(
                  robot_pose.pose.position, robot_yaw, initial_path_yaw)) {
            RCLCPP_WARN(
                node_->get_logger(),
                "[AVOID_ROTATION_BLOCKED] initial turn %.1fdeg -> %.1fdeg "
                "sweeps the footprint into an obstacle; rejecting path",
                robot_yaw * 180.0 / M_PI,
                initial_path_yaw * 180.0 / M_PI);
            planning_success = false;
          } else {
            const double start_clearance = std::hypot(
                first_path.poses.front().pose.position.x - obstacle_center.x,
                first_path.poses.front().pose.position.y - obstacle_center.y);
            if (start_clearance < avoidance_path_clearance_) {
              RCLCPP_WARN(node_->get_logger(),
                          "[AVOID_PLAN] accepting outward escape path: "
                          "start_clearance=%.2f minimum_clearance=%.2f "
                          "target_clearance=%.2f",
                          start_clearance, minimum_obstacle_clearance,
                          avoidance_path_clearance_);
            }
            new_local_path = first_path;
            published_goal = first_side_goal;
            planning_success = true;
          }
        }
      } else {
        RCLCPP_WARN(node_->get_logger(),
                    "[AVOID_PLAN] side A* failed: result=%d poses=%zu",
                    first_ok ? 1 : 0, first_path.poses.size());
      }

      if (planning_success) {
        new_local_path.header.stamp = node_->now();
        {
          std::lock_guard<std::mutex> lock(avoidance_mutex_);
          if (!avoidance_active_ ||
              avoidance_stage_ != AvoidanceStage::FIRST_SIDE_GOAL) {
            return false;
          }
          avoidance_side_path_ = first_path;
          avoidance_corridor_selected_ = true;
          consecutive_planning_failures_ = 0;
        }
        RCLCPP_INFO(node_->get_logger(),
                    "[AVOID_PLAN] side path locked: goal=(%.3f,%.3f); "
                    "return path will be planned after reaching it",
                    first_side_goal.x, first_side_goal.y);
      }
    }
  }

  if (planning_success && !new_local_path.poses.empty()) {
    // 路径稳定性检查：如果新路径与之前路径相似，保持之前的路径以减少抖动
    if (!local_segment_changed && shouldKeepPreviousPath(new_local_path)) {
      RCLCPP_DEBUG(node_->get_logger(),
                   "Keeping previous path to reduce oscillation");
      const nav_msgs::msg::Path retained_path = getLocalPath();
      if (!retained_path.poses.empty()) {
        local_path_pub_->publish(retained_path);
        publishGoalMarker(published_goal);
        return true;
      }
      return false;
    }

    // 路径已在生成阶段完成贝塞尔平滑、footprint检查和净空检查；
    // 任一检查失败时生成阶段会保留原始A*路径。
    nav_msgs::msg::Path planned_path = new_local_path;

    // 更新局部路径。状态检查与路径提交按avoidance->path的固定锁顺序
    // 原子完成，防止暂停取消任务时规划线程又提交一条旧临时路径。
    {
      std::lock_guard<std::mutex> avoidance_lock(avoidance_mutex_);
      if (!avoidance_active_ || avoidance_stage_ != planning_stage) {
        return false;
      }
      std::lock_guard<std::mutex> path_lock(path_mutex_);
      previous_local_path_ = local_path_; // 保存之前的路径
      local_path_ = planned_path;
      local_path_valid_ = true;
      // 新路径已经可见后再通知控制层清零索引。除侧向段/回归段切换外，
      // 同一阶段因Costmap变化重规划也会改变路径长度和点序，不能继续使用
      // 旧路径索引。
      if (local_segment_changed || same_stage_path_replanned) {
        stage_alignment_requested_.store(true);
      }
    }

    // 重置稳定性计数器
    path_stability_counter_ = 0;

    // 发布局部路径和可视化
    local_path_pub_->publish(planned_path);
    publishGoalMarker(published_goal);
    avoidance_path_planner_->publishVisualization(robot_pose.pose.position,
                                                  published_goal, planned_path);

    RCLCPP_DEBUG(node_->get_logger(),
                 "Local path computed successfully with %zu poses",
                 planned_path.poses.size());
    return true;
  } else {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
                         "[AVOID_RETRY] A* returned no usable local path");
    markAvoidanceFailed("A* returned no path");
    local_path_valid_ = false;
    return false;
  }
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
