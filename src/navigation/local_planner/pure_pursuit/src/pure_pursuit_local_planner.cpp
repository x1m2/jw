#include "pure_pursuit/pure_pursuit_local_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "pure_pursuit/param_utils.hpp"

namespace pure_pursuit_local_planner {

PurePursuitLocalPlanner::PurePursuitLocalPlanner()
    : smoothed_local_path_received_(false),
      path_received_(false),
      goal_reached_(false),
      goal_position_reached_(false),
      target_pose_received_(false),
      current_path_index_(0),
      path_progress_index_(0) {}

PurePursuitLocalPlanner::~PurePursuitLocalPlanner() { stop(); }

void PurePursuitLocalPlanner::stop() {
  if (control_timer_) {
    control_timer_->cancel();
  }
  if (cmd_vel_pub_) {
    geometry_msgs::msg::Twist stop_command;
    cmd_vel_pub_->publish(stop_command);
    last_cmd_vel_ = stop_command;
  }
}

bool PurePursuitLocalPlanner::setPaused(bool paused) {
  navigation_paused_.store(paused);
  if (paused) {
    // 暂停代表人工或上层接管。全局任务继续保留，但局部绕障路径是基于
    // 接管前位姿生成的一次性轨迹，恢复后不能再追逐旧临时点。
    if (local_avoidance_manager_) {
      const uint8_t previous_avoidance_state = avoidance_state_.load();
      local_avoidance_manager_->reset();
      smoothed_local_path_received_ = false;
      current_path_index_ = static_cast<int>(path_progress_index_);
      goal_position_reached_ = false;
      if (controller_) controller_->resetTrackingState();
      publishAvoidanceFailure(false);
      LocalAvoidanceOutput idle_output;
      publishAvoidanceState(idle_output);
      if (previous_avoidance_state !=
          navigation_interfaces::msg::AvoidanceState::IDLE) {
        RCLCPP_INFO(node_->get_logger(),
                    "[AVOID_CANCEL] navigation paused; discarded local "
                    "avoidance path so tracking resumes from a fresh path");
      }
    }

    // 立即发送一次零速度，并要求控制循环持续发送，防止底盘沿用旧指令。
    reset_velocity_on_next_control_.store(true);
    if (cmd_vel_pub_) {
      cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
    }
    RCLCPP_INFO(node_->get_logger(), "Navigation velocity output paused");
  } else {
    // 不恢复暂停前的速度；下一周期以零速度为基准重新计算。
    reset_velocity_on_next_control_.store(true);
    RCLCPP_INFO(node_->get_logger(), "Navigation velocity output resumed");
  }
  return true;
}

bool PurePursuitLocalPlanner::initialize(rclcpp::Node::SharedPtr node) {
  node_ = node;

  // 加载参数
  loadParameters();

  // 初始化TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // 初始化组件
  PathTrackerConfig path_tracker_config;
  path_tracker_config.lookahead_distance_min = config_.lookahead_distance_min;
  path_tracker_config.lookahead_distance_max = config_.lookahead_distance_max;
  path_tracker_config.max_linear_velocity = config_.max_linear_velocity;
  path_tracker_config.output_frame = config_.global_frame;
  path_tracker_ = std::make_unique<PathTracker>();
  if (!path_tracker_->initialize(node_, path_tracker_config)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to initialize path tracker");
    return false;
  }

  PurePursuitControllerConfig controller_config;
  controller_config.lookahead_distance_max = config_.lookahead_distance_max;
  controller_config.max_linear_velocity = config_.max_linear_velocity;
  controller_config.max_linear_velocity_y = config_.max_linear_velocity_y;
  controller_config.max_angular_velocity = config_.max_angular_velocity;
  controller_config.acceleration_limit = config_.acceleration_limit;
  controller_config.deceleration_limit = config_.deceleration_limit;
  controller_config.yaw_tolerance = config_.yaw_tolerance;
  controller_config.yaw_adjustment_gain = config_.yaw_adjustment_gain;
  controller_config.final_yaw_min_angular_velocity =
      config_.final_yaw_min_angular_velocity;
  controller_config.local_avoidance_align_enter_angle =
      config_.local_avoidance_align_enter_angle;
  controller_config.local_avoidance_align_exit_angle =
      config_.local_avoidance_align_exit_angle;
  controller_config.local_avoidance_min_align_angular_velocity =
      config_.local_avoidance_min_align_angular_velocity;
  controller_config.local_avoidance_min_linear_velocity =
      config_.local_avoidance_min_linear_velocity;
  controller_ = std::make_unique<PurePursuitController>();
  if (!controller_->initialize(node_, controller_config)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Failed to initialize pure pursuit controller");
    return false;
  }

  // 绕障组件始终初始化，运行时由当前拓扑边策略决定是否使用。
  local_avoidance_manager_ = std::make_unique<LocalAvoidanceManager>();
  if (!local_avoidance_manager_->initialize(node_, tf_buffer_)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Failed to initialize local path planner");
    return false;
  }
  RCLCPP_INFO(node_->get_logger(),
              "Local path planner (A* replanning) initialized");

  front_obstacle_detector_ = std::make_unique<FrontObstacleDetector>();
  if (!front_obstacle_detector_->initialize(node_)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Failed to initialize front obstacle detector");
    return false;
  }

  // 初始化发布器
  cmd_vel_pub_ =
      node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_planned", 1);
  lookahead_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
      "lookahead_point", 1);
  local_trajectory_pub_ =
      node_->create_publisher<nav_msgs::msg::Path>("local_trajectory", 1);
  velocity_arrow_pub_ =
      node_->create_publisher<visualization_msgs::msg::Marker>("velocity_arrow",
                                                               1);
  nav_state_pub_ =
      node_->create_publisher<std_msgs::msg::Bool>("/nav_state", 1);
  nav_failed_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
      "/nav_failed", rclcpp::QoS(1).reliable().transient_local());
  publishAvoidanceFailure(false);
  obstacle_stop_enabled_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
      "/obstacle_stop/enabled", rclcpp::QoS(1).reliable().transient_local());
  avoidance_state_pub_ =
      node_->create_publisher<navigation_interfaces::msg::AvoidanceState>(
          "/local_avoidance/state",
          rclcpp::QoS(1).reliable().transient_local());
  LocalAvoidanceOutput initial_avoidance_output;
  publishAvoidanceState(initial_avoidance_output);

  // SlamManager内部控制接口，不作为APP直接调用的业务API。
  pause_service_ = node_->create_service<std_srvs::srv::SetBool>(
      "/pure_pursuit/set_paused",
      [this](const std_srvs::srv::SetBool::Request::SharedPtr request,
             std_srvs::srv::SetBool::Response::SharedPtr response) {
        response->success = setPaused(request->data);
        response->message =
            request->data ? "navigation paused" : "navigation resumed";
      });

  // 初始化订阅器
  global_path_with_policy_sub_ =
      node_->create_subscription<navigation_interfaces::msg::PathWithPolicy>(
          "/smoothed_global_path_with_policy", rclcpp::QoS(10),
          [this](
              navigation_interfaces::msg::PathWithPolicy::ConstSharedPtr msg) {
            this->globalPathWithPolicyCallback(msg);
          });

  costmap_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/costmap", rclcpp::QoS(10),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
        this->costmapCallback(msg);
      });

  if (config_.use_smoothed_path) {
    smoothed_local_path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
        "/pure_pursuit/smoothed_local_path", rclcpp::QoS(10),
        [this](nav_msgs::msg::Path::ConstSharedPtr msg) {
          this->smoothedLocalPathCallback(msg);
        });
  }

  task_target_sub_ = node_->create_subscription<slam_manager_interfaces::msg::TaskTarget>(
      "/navigation/task_target", 10,
      [this](slam_manager_interfaces::msg::TaskTarget::ConstSharedPtr msg) {
        auto pose = std::make_shared<geometry_msgs::msg::Pose>();
        pose->position.x = msg->target.x;
        pose->position.y = msg->target.y;
        pose->position.z = msg->target.z;
        pose->orientation.z = std::sin(msg->target.theta * 0.5);
        pose->orientation.w = std::cos(msg->target.theta * 0.5);
        targetPointsCallback(pose);
      });
  obstacle_stop_active_sub_ =
      node_->create_subscription<std_msgs::msg::Bool>(
          "/obstacle_stop/active", rclcpp::QoS(1).reliable().transient_local(),
          [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
            obstacle_stop_active_.store(msg->data);
          });

  // 初始化控制定时器
  control_timer_ = node_->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(1.0 / config_.control_frequency)),
      std::bind(&PurePursuitLocalPlanner::controlCallback, this));

  last_control_time_ = node_->now();

  RCLCPP_INFO(node_->get_logger(), "Pure Pursuit Local Planner initialized");
  return true;
}

void PurePursuitLocalPlanner::publishAvoidanceFailure(bool failed) {
  if (nav_failed_pub_) {
    std_msgs::msg::Bool message;
    message.data = failed;
    nav_failed_pub_->publish(message);
  }
}

void PurePursuitLocalPlanner::publishAvoidanceState(
    const LocalAvoidanceOutput &output) {
  if (!avoidance_state_pub_) return;

  uint8_t state = navigation_interfaces::msg::AvoidanceState::IDLE;
  switch (output.mode) {
    case AvoidanceTrackingMode::WAITING_COSTMAP:
    case AvoidanceTrackingMode::WAITING_LOCAL_PATH:
    case AvoidanceTrackingMode::LOCAL_PATH:
      state = navigation_interfaces::msg::AvoidanceState::AVOIDING;
      break;
    case AvoidanceTrackingMode::STOPPED_BLOCKED:
      state = navigation_interfaces::msg::AvoidanceState::BLOCKED;
      break;
    case AvoidanceTrackingMode::WAITING_DYNAMIC_PATH:
      state = navigation_interfaces::msg::AvoidanceState::WAITING_DYNAMIC_PATH;
      break;
    case AvoidanceTrackingMode::FAILED:
      state = navigation_interfaces::msg::AvoidanceState::FAILED;
      break;
    case AvoidanceTrackingMode::GLOBAL_PATH:
      break;
  }

  const uint8_t previous = avoidance_state_.exchange(state);
  if (previous == state) return;
  if (state ==
          navigation_interfaces::msg::AvoidanceState::WAITING_DYNAMIC_PATH &&
      previous != state) {
    // 只接受进入等待状态之后生成的动态路径，过滤平滑器和DDS队列中的旧帧。
    dynamic_path_accept_after_ns_.store(node_->now().nanoseconds());
  }

  navigation_interfaces::msg::AvoidanceState message;
  message.state = state;
  avoidance_state_pub_->publish(message);
}

void PurePursuitLocalPlanner::loadParameters() {
  // Pure Pursuit parameters
  config_.lookahead_distance_min =
      declareOrGetParameter(node_, "lookahead_distance_min", 0.4);
  config_.lookahead_distance_max =
      declareOrGetParameter(node_, "lookahead_distance_max", 0.8);
  config_.goal_tolerance = declareOrGetParameter(node_, "goal_tolerance", 0.2);

  // Speed parameters
  config_.max_linear_velocity =
      declareOrGetParameter(node_, "max_linear_velocity", 0.8);
  config_.max_linear_velocity_y =
      declareOrGetParameter(node_, "max_linear_velocity_y", 0.0);
  config_.max_angular_velocity =
      declareOrGetParameter(node_, "max_angular_velocity", 0.8);
  config_.acceleration_limit =
      declareOrGetParameter(node_, "acceleration_limit", 0.5);
  config_.deceleration_limit =
      declareOrGetParameter(node_, "deceleration_limit", 0.8);
  // Control parameters
  config_.control_frequency =
      declareOrGetParameter(node_, "control_frequency", 20.0);
  config_.transform_tolerance =
      declareOrGetParameter(node_, "transform_tolerance", 0.1);

  // Orientation adjustment parameters
  config_.yaw_tolerance = declareOrGetParameter(node_, "yaw_tolerance", 0.1);
  config_.yaw_adjustment_gain =
      declareOrGetParameter(node_, "yaw_adjustment_gain", 1.0);
  config_.final_yaw_min_angular_velocity =
      declareOrGetParameter(node_, "final_yaw_min_angular_velocity", 0.28);
  config_.local_avoidance_align_enter_angle =
      declareOrGetParameter(node_, "local_avoidance_align_enter_angle", 30.0) *
      M_PI / 180.0;
  config_.local_avoidance_align_exit_angle =
      declareOrGetParameter(node_, "local_avoidance_align_exit_angle", 6.0) *
      M_PI / 180.0;
  config_.local_avoidance_align_enter_angle = std::clamp(
      config_.local_avoidance_align_enter_angle, 1.0 * M_PI / 180.0,
      89.0 * M_PI / 180.0);
  config_.local_avoidance_align_exit_angle = std::clamp(
      config_.local_avoidance_align_exit_angle, 1.0 * M_PI / 180.0,
      config_.local_avoidance_align_enter_angle);
  config_.local_avoidance_min_align_angular_velocity = std::clamp(
      declareOrGetParameter(
          node_, "local_avoidance_min_align_angular_velocity", 0.35),
      0.0, config_.max_angular_velocity);
  config_.local_avoidance_lookahead_distance = std::clamp(
      declareOrGetParameter(node_, "local_avoidance_lookahead_distance", 0.25),
      0.05, config_.lookahead_distance_max);
  config_.local_avoidance_max_linear_velocity = std::clamp(
      declareOrGetParameter(node_, "local_avoidance_max_linear_velocity", 0.5),
      0.0, config_.max_linear_velocity);
  config_.local_avoidance_min_linear_velocity = std::clamp(
      declareOrGetParameter(node_, "local_avoidance_min_linear_velocity", 0.4),
      0.0, config_.local_avoidance_max_linear_velocity);
  config_.dynamic_goal_change_distance = std::max(
      0.0, declareOrGetParameter(node_, "dynamic_goal_change_distance", 0.7));
  config_.dynamic_goal_change_angle =
      std::clamp(declareOrGetParameter(
                     node_, "dynamic_goal_change_angle_degrees", 25.0),
                 0.0, 180.0) *
      M_PI / 180.0;
  // Local planning parameters
  config_.use_smoothed_path =
      declareOrGetParameter(node_, "use_smoothed_path", false);
  // Frame IDs
  config_.global_frame =
      declareOrGetParameter<std::string>(node_, "global_frame", "map");
  config_.robot_frame =
      declareOrGetParameter<std::string>(node_, "robot_frame", "base_link");
}

bool PurePursuitLocalPlanner::setGlobalPath(
    const nav_msgs::msg::Path &global_path, bool dynamic_path) {
  if (global_path.poses.empty()) {
    RCLCPP_WARN(node_->get_logger(), "Received empty global path");
    return false;
  }

  global_path_ = global_path;
  path_received_ = true;
  goal_reached_ = false;
  goal_position_reached_ = false;  // 新增：新路径开始，重置到点锁存
  if (controller_) controller_->resetTrackingState();
  current_path_index_ = 0;
  path_progress_index_ = 0;  // 新增：重置路径进度，防止回头
  smoothed_local_path_received_ = false;
  if (local_avoidance_manager_) {
    local_avoidance_manager_->reset();
    local_avoidance_manager_->setDynamicPathMode(dynamic_path);
  }
  publishAvoidanceFailure(false);

  // 发布导航开始状态
  std_msgs::msg::Bool nav_state_msg;
  nav_state_msg.data = false;
  nav_state_pub_->publish(nav_state_msg);

  // 将路径传递给路径跟踪器
  if (path_tracker_) {
    path_tracker_->setPath(global_path_);
  }

  if (front_obstacle_detector_) {
    front_obstacle_detector_->setGlobalPath(global_path_);
  } else {
    RCLCPP_ERROR(node_->get_logger(),
                 "Front obstacle detector is not initialized");
  }

  if (local_avoidance_manager_) {
    local_avoidance_manager_->setGlobalPath(global_path_);
  }

  LocalAvoidanceOutput idle_output;
  publishAvoidanceState(idle_output);

  RCLCPP_INFO(node_->get_logger(), "Global path set with %zu poses",
              global_path_.poses.size());
  return true;
}

bool PurePursuitLocalPlanner::computeVelocityCommands(
    geometry_msgs::msg::Twist &cmd_vel) {
  if (!path_received_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "No global path received");
    cmd_vel = geometry_msgs::msg::Twist();
    return false;
  }

  if (goal_reached_) {
    cmd_vel = geometry_msgs::msg::Twist();
    return true;
  }

  // 获取机器人当前位姿
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!getRobotPose(robot_pose)) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "Failed to get robot pose");
    cmd_vel = geometry_msgs::msg::Twist();
    return false;
  }

  // 执行避障决策前先更新当前拓扑段策略。
  updatePathProgress(robot_pose);
  const auto edge_policy = currentEdgePolicy();
  const double edge_linear_velocity = edge_policy.velocity_limit_enabled
                                          ? edge_policy.linear_velocity
                                          : config_.max_linear_velocity;
  const double edge_angular_velocity = edge_policy.velocity_limit_enabled
                                           ? edge_policy.angular_velocity
                                           : config_.max_angular_velocity;
  const int motion_direction = edge_policy.motion_direction;
  const bool avoidance_enabled = isObstacleAvoidanceEnabled();
  bool local_tracking =
      local_avoidance_manager_ && local_avoidance_manager_->isTrackingLocal();
  LocalAvoidanceOutput avoidance_output;
  bool avoidance_updated = false;

  auto apply_avoidance_transition = [this](const LocalAvoidanceOutput &output) {
    if (output.failure_changed) {
      publishAvoidanceFailure(output.failure_latched);
    }
    if (output.exited_local_tracking) {
      smoothed_local_path_received_ = false;
      current_path_index_ = static_cast<int>(path_progress_index_);
      if (controller_) controller_->resetTrackingState();
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "[OBSTACLE_REPLAN] inactive, tracking global path");
    }
    if (output.entered_local_tracking) {
      current_path_index_ = 0;
      if (controller_) controller_->startLocalAvoidanceAlignment();
      goal_position_reached_ = false;
      last_cmd_vel_ = geometry_msgs::msg::Twist();
    }
    if (output.realign_local_path && !output.entered_local_tracking) {
      current_path_index_ = 0;
      smoothed_local_path_received_ = false;
      // 新路径要重新投影和检查对齐，但不要先清空上一帧速度。若新路径
      // 航向连续，限速器可平滑接续；若航向变化较大，控制器会输出纯旋转，
      // 下方逻辑仍会在转向前清掉线速度。
      if (controller_) controller_->startLocalAvoidanceAlignment();
      RCLCPP_INFO(node_->get_logger(),
                  "[AVOID_PATH] local path replaced, tracking index reset");
    }
  };

  // 策略切到关闭时立即让Manager退出局部路径状态，再进行终点判断。
  if (!avoidance_enabled && local_tracking) {
    const AvoidanceObservation clear_observation;
    avoidance_output = local_avoidance_manager_->update(
        robot_pose, clear_observation, false, false, false);
    avoidance_updated = true;
    apply_avoidance_transition(avoidance_output);
    local_tracking = avoidance_output.tracking_local;
  }

  if (local_tracking && goal_position_reached_) {
    goal_position_reached_ = false;
    last_cmd_vel_ = geometry_msgs::msg::Twist();
  }

  // 位置到达锁存：第一次进入 goal_tolerance 后，就锁住“位置已到达”
  if (!local_tracking && !goal_position_reached_ && isGoalReached(robot_pose)) {
    goal_position_reached_ = true;

    // 到点后先清零上一帧速度，避免最终调角度时还带线速度
    last_cmd_vel_ = geometry_msgs::msg::Twist();

    RCLCPP_INFO(node_->get_logger(),
                "Goal position reached! Start final yaw adjustment...");
  }

  // 如果位置已经到达过，则只做最终角度调整，不再回 Pure Pursuit
  if (!local_tracking && goal_position_reached_) {
    if (target_pose_received_ && controller_ &&
        !controller_->isTargetOrientationReached(robot_pose, target_pose_)) {
      if (controller_->computeOrientationAdjustmentCommand(
              robot_pose, target_pose_, edge_angular_velocity, cmd_vel)) {
        return true;
      } else {
        cmd_vel = geometry_msgs::msg::Twist();
        return false;
      }
    }

    // 位置和角度都完成
    cmd_vel = geometry_msgs::msg::Twist();
    goal_reached_ = true;
    goal_position_reached_ = false;

    std_msgs::msg::Bool nav_state_msg;
    nav_state_msg.data = true;
    nav_state_pub_->publish(nav_state_msg);

    RCLCPP_INFO(node_->get_logger(), "Goal reached!");
    return true;
  }

  if (local_avoidance_manager_ && !avoidance_updated) {
    AvoidanceObservation observation;
    if (avoidance_enabled && front_obstacle_detector_) {
      observation = front_obstacle_detector_->analyzeAvoidance(robot_pose);
    }
    const bool planned_motion =
        std::abs(last_cmd_vel_.linear.x) > 0.02 ||
        std::abs(last_cmd_vel_.linear.y) > 0.02 ||
        std::abs(last_cmd_vel_.angular.z) > 0.05;
    avoidance_output = local_avoidance_manager_->update(
        robot_pose, observation, avoidance_enabled,
        controller_ && controller_->isAligning(),
        obstacle_stop_active_.load() && planned_motion);
    apply_avoidance_transition(avoidance_output);
    local_tracking = avoidance_output.tracking_local;
  }

  if (local_avoidance_manager_) {
    publishAvoidanceState(avoidance_output);
  }

  // 绕障期间路径走廊必须跟随当前A*段。若继续使用旧全局路径做过滤，
  // 局部路径正前方的障碍可能被判为“全局走廊外”而返回CLEAR。
  if (front_obstacle_detector_) {
    if (local_tracking && avoidance_output.local_path &&
        !avoidance_output.local_path->poses.empty()) {
      front_obstacle_detector_->setGlobalPath(*avoidance_output.local_path);
    } else if (avoidance_output.exited_local_tracking) {
      front_obstacle_detector_->setGlobalPath(global_path_);
    }
  }

  if (avoidance_output.mode == AvoidanceTrackingMode::FAILED ||
      avoidance_output.mode == AvoidanceTrackingMode::WAITING_COSTMAP ||
      avoidance_output.mode == AvoidanceTrackingMode::STOPPED_BLOCKED ||
      avoidance_output.mode == AvoidanceTrackingMode::WAITING_DYNAMIC_PATH) {
    cmd_vel = geometry_msgs::msg::Twist();
    last_cmd_vel_ = cmd_vel;
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 500,
        "[OBSTACLE_REPLAN] avoidance requested stop, cmd_vel forced to zero");
    return true;
  }

  // 计算前瞻点
  geometry_msgs::msg::PointStamped lookahead_point;
  if (!computeLookaheadPoint(robot_pose, local_tracking,
                             avoidance_output.local_path, lookahead_point)) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "Failed to compute lookahead point");
    cmd_vel = geometry_msgs::msg::Twist();
    if (local_tracking) {
      last_cmd_vel_ = cmd_vel;
      return true;
    }
    return false;
  }

  double target_velocity =
      local_tracking
          ? std::min(edge_linear_velocity,
                     config_.local_avoidance_max_linear_velocity)
          : edge_linear_velocity;
  if (path_tracker_) {
    target_velocity = path_tracker_->computeTargetVelocity(
        robot_pose, lookahead_point, target_velocity, motion_direction);
  }
  if (!controller_ ||
      !controller_->computeCommand(
          robot_pose, lookahead_point, target_velocity, edge_angular_velocity,
          motion_direction,
          local_tracking, path_progress_index_, current_path_index_, cmd_vel)) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "Failed to compute pure pursuit command");
    cmd_vel = geometry_msgs::msg::Twist();
    return false;
  }

  if (std::abs(cmd_vel.linear.x) < 1e-6 && std::abs(cmd_vel.linear.y) < 1e-6 &&
      std::abs(cmd_vel.angular.z) < 1e-6) {
    last_cmd_vel_ = geometry_msgs::msg::Twist();
  }
  // 新增：原地旋转时清掉上一帧线速度
  if (std::abs(cmd_vel.linear.x) < 1e-6 && std::abs(cmd_vel.angular.z) > 1e-6) {
    last_cmd_vel_.linear.x = 0.0;
    last_cmd_vel_.linear.y = 0.0;
  }
  // 应用速度限制
  controller_->applyVelocityLimits(
      last_cmd_vel_, (node_->now() - last_control_time_).seconds(),
      edge_linear_velocity, edge_angular_velocity, cmd_vel);

  // 发布可视化信息
  publishVisualization(lookahead_point);

  // 发布局部轨迹和速度可视化
  publishLocalTrajectory(robot_pose, cmd_vel);
  publishVelocityArrow(robot_pose, cmd_vel);

  last_cmd_vel_ = cmd_vel;
  last_control_time_ = node_->now();

  return true;
}

bool PurePursuitLocalPlanner::isGoalReached(
    const geometry_msgs::msg::PoseStamped &robot_pose) const {
  if (!path_received_ || global_path_.poses.empty()) return false;

  const auto &goal_pose = global_path_.poses.back();
  // 导航到点只判断水平面位置，忽略 SLAM 高度与路径高度之间的偏差。
  // XY 到达后，computeVelocityCommands() 会继续单独判断并调整最终 yaw。
  const double distance =
      std::hypot(robot_pose.pose.position.x - goal_pose.pose.position.x,
                 robot_pose.pose.position.y - goal_pose.pose.position.y);

  return distance < config_.goal_tolerance;
}

bool PurePursuitLocalPlanner::getRobotPose(
    geometry_msgs::msg::PoseStamped &robot_pose) {
  try {
    const geometry_msgs::msg::TransformStamped transform =
        tf_buffer_->lookupTransform(
            config_.global_frame, config_.robot_frame, tf2::TimePointZero,
            tf2::durationFromSec(config_.transform_tolerance));

    robot_pose.header.frame_id = config_.global_frame;
    robot_pose.header.stamp = transform.header.stamp;
    robot_pose.pose.position.x = transform.transform.translation.x;
    robot_pose.pose.position.y = transform.transform.translation.y;
    robot_pose.pose.position.z = transform.transform.translation.z;
    robot_pose.pose.orientation = transform.transform.rotation;

    return true;
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "Failed to get robot pose: %s", ex.what());
    return false;
  }
}

void PurePursuitLocalPlanner::controlCallback() {
  if (reset_velocity_on_next_control_.exchange(false)) {
    last_cmd_vel_ = geometry_msgs::msg::Twist();
    last_control_time_ = node_->now();
  }

  if (navigation_paused_.load()) {
    // 暂停期间保留全局任务，但局部绕障状态已在setPaused(true)中清除。
    // 每个控制周期持续发送零速度，避免底盘沿用接管前的速度。
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "[PP_PAUSED] navigation paused, publishing zero "
                         "velocity");
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
    return;
  }

  geometry_msgs::msg::Twist cmd_vel;
  if (computeVelocityCommands(cmd_vel)) {
    cmd_vel_pub_->publish(cmd_vel);
  } else {
    // 位姿、路径或前瞻点计算失败时立即停止，避免底盘继续执行上一帧速度。
    const geometry_msgs::msg::Twist stop_command;
    cmd_vel_pub_->publish(stop_command);
    last_cmd_vel_ = stop_command;
    last_control_time_ = node_->now();
  }
}

void PurePursuitLocalPlanner::globalPathWithPolicyCallback(
    navigation_interfaces::msg::PathWithPolicy::ConstSharedPtr msg) {
  if (msg->dynamic_path) {
    const uint8_t state = avoidance_state_.load();
    if (state == navigation_interfaces::msg::AvoidanceState::AVOIDING ||
        state == navigation_interfaces::msg::AvoidanceState::BLOCKED ||
        state == navigation_interfaces::msg::AvoidanceState::FAILED) {
      double goal_shift = 0.0;
      double direction_change = 0.0;
      if (!hasDynamicGoalChanged(msg->path, goal_shift, direction_change)) {
        RCLCPP_DEBUG_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "Ignoring dynamic path while local avoidance state=%u; "
            "goal_shift=%.2fm direction_change=%.1fdeg",
            state, goal_shift, direction_change * 180.0 / M_PI);
        return;
      }

      // setGlobalPath()会统一复位Manager、控制器和局部路径。新路径在下一
      // 控制周期重新做障碍检测：新路线已避开障碍就直接跟踪，否则重新规划。
      RCLCPP_INFO(node_->get_logger(),
                  "[AVOID_CANCEL_TARGET] dynamic target changed; discarding "
                  "old avoidance path (shift=%.2fm, direction=%.1fdeg, "
                  "state=%u)",
                  goal_shift, direction_change * 180.0 / M_PI, state);
    }
    if (state ==
        navigation_interfaces::msg::AvoidanceState::WAITING_DYNAMIC_PATH) {
      const int64_t path_stamp_ns =
          rclcpp::Time(msg->path.header.stamp).nanoseconds();
      if (path_stamp_ns <= dynamic_path_accept_after_ns_.load()) {
        RCLCPP_DEBUG_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "Ignoring stale dynamic path while waiting for a fresh target");
        return;
      }
    }
  }

  edge_policies_ = msg->edge_policies;
  if (edge_policies_.size() != msg->path.poses.size()) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Path policy size mismatch: poses=%zu, policies=%zu; "
                 "falling back to safe defaults",
                 msg->path.poses.size(), edge_policies_.size());
    const auto default_policy =
        navigation_interfaces::makeDefaultEdgePolicy(false);
    edge_policies_.assign(msg->path.poses.size(), default_policy);
  }
  for (auto &policy : edge_policies_) {
    if (policy.velocity_limit_enabled &&
        (!std::isfinite(policy.linear_velocity) ||
         policy.linear_velocity < 0.0 ||
         !std::isfinite(policy.angular_velocity) ||
         policy.angular_velocity < 0.0)) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Invalid EdgePolicy velocity limits; using configured "
                   "maximums");
      policy.velocity_limit_enabled = false;
    }
    if (policy.velocity_limit_enabled) {
      policy.linear_velocity =
          std::min(policy.linear_velocity, config_.max_linear_velocity);
      policy.angular_velocity =
          std::min(policy.angular_velocity, config_.max_angular_velocity);
    }
    if (policy.motion_direction != 1 && policy.motion_direction != -1) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Invalid EdgePolicy motion_direction=%d; using forward",
                   static_cast<int>(policy.motion_direction));
      policy.motion_direction = 1;
    }
    if (!std::isfinite(policy.obstacle_avoidance_distance_m) ||
        policy.obstacle_avoidance_distance_m < 0.0) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Invalid EdgePolicy obstacle avoidance distance; using "
                   "0.5m");
      policy.obstacle_avoidance_distance_m = 0.5;
    }
  }
  setGlobalPath(msg->path, msg->dynamic_path);
  publishCurrentObstacleStopPolicy();
}

bool PurePursuitLocalPlanner::hasDynamicGoalChanged(
    const nav_msgs::msg::Path &new_path, double &goal_shift,
    double &direction_change) {
  goal_shift = 0.0;
  direction_change = 0.0;
  if (new_path.poses.empty() || global_path_.poses.empty()) {
    return false;
  }

  const std::string &old_frame = global_path_.header.frame_id;
  const std::string &new_frame = new_path.header.frame_id;
  if (!old_frame.empty() && !new_frame.empty() && old_frame != new_frame) {
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "Cannot compare dynamic goals in different frames: old=%s new=%s",
        old_frame.c_str(), new_frame.c_str());
    return false;
  }

  // 防止平滑器或DDS队列中的旧路径把机器人拉回旧目标。时间戳为0时保留
  // 兼容性，只用目标几何变化判断。
  const int64_t old_stamp_ns =
      rclcpp::Time(global_path_.header.stamp).nanoseconds();
  const int64_t new_stamp_ns = rclcpp::Time(new_path.header.stamp).nanoseconds();
  if (old_stamp_ns > 0 && new_stamp_ns > 0 && new_stamp_ns <= old_stamp_ns) {
    return false;
  }

  const auto &old_goal = global_path_.poses.back().pose.position;
  const auto &new_goal = new_path.poses.back().pose.position;
  goal_shift = std::hypot(new_goal.x - old_goal.x, new_goal.y - old_goal.y);

  geometry_msgs::msg::PoseStamped robot_pose;
  if (config_.dynamic_goal_change_angle > 0.0 && getRobotPose(robot_pose) &&
      (new_frame.empty() || robot_pose.header.frame_id == new_frame)) {
    const double old_dx = old_goal.x - robot_pose.pose.position.x;
    const double old_dy = old_goal.y - robot_pose.pose.position.y;
    const double new_dx = new_goal.x - robot_pose.pose.position.x;
    const double new_dy = new_goal.y - robot_pose.pose.position.y;
    const double old_norm = std::hypot(old_dx, old_dy);
    const double new_norm = std::hypot(new_dx, new_dy);
    if (old_norm > 1e-3 && new_norm > 1e-3) {
      const double cosine = std::clamp(
          (old_dx * new_dx + old_dy * new_dy) / (old_norm * new_norm), -1.0,
          1.0);
      direction_change = std::acos(cosine);
    }
  }

  const bool distance_changed =
      config_.dynamic_goal_change_distance > 0.0 &&
      goal_shift >= config_.dynamic_goal_change_distance;
  const bool direction_changed = config_.dynamic_goal_change_angle > 0.0 &&
                                 direction_change >=
                                     config_.dynamic_goal_change_angle;
  return distance_changed || direction_changed;
}

bool PurePursuitLocalPlanner::isObstacleAvoidanceEnabled() const {
  return currentEdgePolicy().obstacle_avoidance_enabled;
}

navigation_interfaces::msg::EdgePolicy
PurePursuitLocalPlanner::currentEdgePolicy() const {
  if (path_progress_index_ < edge_policies_.size()) {
    return edge_policies_[path_progress_index_];
  }
  return navigation_interfaces::makeDefaultEdgePolicy(false);
}

void PurePursuitLocalPlanner::publishCurrentObstacleStopPolicy() {
  const bool enabled = currentEdgePolicy().obstacle_stop_enabled;
  if (has_published_obstacle_stop_policy_ &&
      enabled == published_obstacle_stop_enabled_) {
    return;
  }
  std_msgs::msg::Bool message;
  message.data = enabled;
  obstacle_stop_enabled_pub_->publish(message);
  published_obstacle_stop_enabled_ = enabled;
  has_published_obstacle_stop_policy_ = true;
  RCLCPP_INFO(node_->get_logger(), "Obstacle stop policy: %s at path index %zu",
              enabled ? "enabled" : "disabled", path_progress_index_);
}

void PurePursuitLocalPlanner::costmapCallback(
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
  if (local_avoidance_manager_) {
    local_avoidance_manager_->updateCostmap(msg);
  }

  // 更新前方障碍物检测器的代价地图
  if (front_obstacle_detector_) {
    front_obstacle_detector_->updateCostmap(msg);
  }
}

void PurePursuitLocalPlanner::smoothedLocalPathCallback(
    nav_msgs::msg::Path::ConstSharedPtr msg) {
  // 只接收当前绕障任务的平滑结果，避免下一次任务误用迟到的旧路径。
  if (config_.use_smoothed_path && local_avoidance_manager_ &&
      local_avoidance_manager_->isTrackingLocal()) {
    smoothed_local_path_ = *msg;
    smoothed_local_path_received_ = !msg->poses.empty();

    if (smoothed_local_path_received_) {
      RCLCPP_DEBUG(node_->get_logger(),
                   "Received smoothed local path with %zu poses",
                   msg->poses.size());
    }
  } else {
    RCLCPP_DEBUG(
        node_->get_logger(),
        "Smoothed path callback received but smoothed path usage is disabled");
  }
}

void PurePursuitLocalPlanner::targetPointsCallback(
    geometry_msgs::msg::Pose::ConstSharedPtr msg) {
  target_pose_ = *msg;
  target_pose_received_ = true;
  RCLCPP_DEBUG(node_->get_logger(),
               "Received target pose: position(%.3f, %.3f, %.3f), "
               "orientation(%.3f, %.3f, %.3f, %.3f)",
               target_pose_.position.x, target_pose_.position.y,
               target_pose_.position.z, target_pose_.orientation.x,
               target_pose_.orientation.y, target_pose_.orientation.z,
               target_pose_.orientation.w);
}

bool PurePursuitLocalPlanner::computeLookaheadPoint(
    const geometry_msgs::msg::PoseStamped &robot_pose, bool local_tracking,
    const std::optional<nav_msgs::msg::Path> &local_path,
    geometry_msgs::msg::PointStamped &lookahead_point) {
  // 根据参数决定路径选择优先级
  const nav_msgs::msg::Path *path_to_use = nullptr;
  TrackingPathType path_type = TrackingPathType::kGlobal;

  if (local_tracking) {
    // 启用局部重规划时的优先级：平滑后局部路径 > 原始局部路径 > 全局路径
    if (config_.use_smoothed_path && smoothed_local_path_received_ &&
        !smoothed_local_path_.poses.empty()) {
      path_to_use = &smoothed_local_path_;
      path_type = TrackingPathType::kSmoothedLocal;
      RCLCPP_DEBUG(node_->get_logger(),
                   "Using smoothed local path for lookahead point computation");
    } else if (local_path && !local_path->poses.empty()) {
      // 使用Manager在本控制周期返回的单一快照，避免分开查询“有效”和“路径”
      // 时读到两个不同规划周期的数据。
      path_to_use = &*local_path;
      path_type = TrackingPathType::kLocal;
      RCLCPP_DEBUG(node_->get_logger(),
                   "Using original local path for lookahead point computation");
    } else {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 500,
          "[OBSTACLE_REPLAN] waiting for valid local avoidance path");
      return false;
    }
  } else {
    // 禁用局部重规划时，直接使用全局路径
    if (!global_path_.poses.empty()) {
      path_to_use = &global_path_;
      path_type = TrackingPathType::kGlobal;
      RCLCPP_DEBUG(node_->get_logger(),
                   "Local replanning disabled, using global path directly");
    } else {
      return false;
    }
  }

  if (!path_to_use || path_to_use->poses.empty() || !path_tracker_)
    return false;
  const double current_velocity =
      std::hypot(last_cmd_vel_.linear.x, last_cmd_vel_.linear.y);
  const TrackingContext tracking_context{
      path_type,
      path_type == TrackingPathType::kGlobal
          ? path_progress_index_
          : static_cast<std::size_t>(std::max(0, current_path_index_)),
      current_velocity,
      local_tracking ? config_.local_avoidance_lookahead_distance : 0.0,
  };
  const auto tracking_result = path_tracker_->computeTrackingLookahead(
      robot_pose, *path_to_use, tracking_context);
  if (!tracking_result) {
    return false;
  }
  lookahead_point = tracking_result->lookahead_point;
  if (path_type == TrackingPathType::kGlobal) {
    current_path_index_ = static_cast<int>(tracking_result->path_index);
  } else {
    // 局部路径的控制进度使用机器人投影段，不能使用前瞻点段；否则每个
    // 控制周期都会把搜索起点向前推一个前瞻距离，机器人不动索引也会跑完。
    // PathTracker正常情况下只从当前索引向前搜索，因此直接赋值仍保持单调；
    // 当局部路径被异步替换且旧索引越界时，它会从新路径重新投影，此处
    // 必须允许索引回到新路径的有效范围。
    current_path_index_ = static_cast<int>(tracking_result->progress_index);
  }
  return true;
}

void PurePursuitLocalPlanner::publishVisualization(
    const geometry_msgs::msg::PointStamped &lookahead_point) {
  // 发布前瞻点标记
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = config_.global_frame;
  marker.header.stamp = node_->now();
  marker.ns = "pure_pursuit";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position = lookahead_point.point;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;

  lookahead_pub_->publish(marker);

  // 发布局部路径
  if (path_tracker_) {
    path_tracker_->publishVisualization(lookahead_point);
  }
}

void PurePursuitLocalPlanner::publishLocalTrajectory(
    const geometry_msgs::msg::PoseStamped &robot_pose,
    const geometry_msgs::msg::Twist &cmd_vel) {
  nav_msgs::msg::Path trajectory = generateLocalTrajectory(robot_pose, cmd_vel);
  local_trajectory_pub_->publish(trajectory);
}

void PurePursuitLocalPlanner::publishVelocityArrow(
    const geometry_msgs::msg::PoseStamped &robot_pose,
    const geometry_msgs::msg::Twist &cmd_vel) {
  visualization_msgs::msg::Marker arrow;
  arrow.header.frame_id = config_.global_frame;
  arrow.header.stamp = node_->now();
  arrow.ns = "velocity";
  arrow.id = 0;
  arrow.type = visualization_msgs::msg::Marker::ARROW;
  arrow.action = visualization_msgs::msg::Marker::ADD;

  // 设置箭头起点
  arrow.pose.position = robot_pose.pose.position;
  arrow.pose.position.z += 0.2;  // 稍微抬高避免与地面重叠

  // 全向机器人：计算实际速度方向（结合X和Y方向速度）
  const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);

  // 将机器人坐标系下的速度转换到全局坐标系
  const double global_vel_x = cmd_vel.linear.x * std::cos(robot_yaw) -
                              cmd_vel.linear.y * std::sin(robot_yaw);
  const double global_vel_y = cmd_vel.linear.x * std::sin(robot_yaw) +
                              cmd_vel.linear.y * std::cos(robot_yaw);

  // 计算实际运动方向
  const double arrow_yaw = std::atan2(global_vel_y, global_vel_x);

  tf2::Quaternion q;
  q.setRPY(0, 0, arrow_yaw);
  arrow.pose.orientation = tf2::toMsg(q);

  // 根据速度大小设置箭头长度
  const double velocity_magnitude =
      std::hypot(cmd_vel.linear.x, cmd_vel.linear.y);
  const double arrow_length =
      std::max(0.2, velocity_magnitude * 0.5);  // 最小长度0.2m

  arrow.scale.x = arrow_length;  // 箭头长度
  arrow.scale.y = 0.1;           // 箭头宽度
  arrow.scale.z = 0.1;           // 箭头高度

  // 根据速度大小设置颜色（绿色到红色渐变）
  arrow.color.a = 0.8;
  const double max_combined_velocity =
      std::hypot(config_.max_linear_velocity, config_.max_linear_velocity_y);
  const double speed_ratio =
      std::clamp(velocity_magnitude / max_combined_velocity, 0.0, 1.0);
  arrow.color.r = speed_ratio;
  arrow.color.g = 1.0 - speed_ratio;
  arrow.color.b = 0.0;

  velocity_arrow_pub_->publish(arrow);
}

nav_msgs::msg::Path PurePursuitLocalPlanner::generateLocalTrajectory(
    const geometry_msgs::msg::PoseStamped &robot_pose,
    const geometry_msgs::msg::Twist &cmd_vel, double prediction_time,
    double dt) {
  nav_msgs::msg::Path trajectory;
  trajectory.header.frame_id = config_.global_frame;
  const auto base_stamp = node_->now();
  trajectory.header.stamp = base_stamp;

  if (prediction_time < 0.0 || dt <= 0.0) {
    return trajectory;
  }

  // 获取机器人当前状态
  const double x = robot_pose.pose.position.x;
  const double y = robot_pose.pose.position.y;
  const double theta = tf2::getYaw(robot_pose.pose.orientation);

  const double linear_vel = cmd_vel.linear.x;
  const double angular_vel = cmd_vel.angular.z;

  // 生成预测轨迹点
  const int num_steps = static_cast<int>(prediction_time / dt);
  trajectory.poses.reserve(static_cast<size_t>(num_steps) + 1);
  for (int i = 0; i <= num_steps; ++i) {
    const double t = i * dt;

    geometry_msgs::msg::PoseStamped pose;
    pose.header = trajectory.header;
    pose.header.stamp = base_stamp + rclcpp::Duration::from_seconds(t);

    if (std::abs(angular_vel) < 1e-6) {
      // 直线运动
      pose.pose.position.x = x + linear_vel * t * std::cos(theta);
      pose.pose.position.y = y + linear_vel * t * std::sin(theta);
      pose.pose.position.z = robot_pose.pose.position.z;

      tf2::Quaternion q;
      q.setRPY(0, 0, theta);
      pose.pose.orientation = tf2::toMsg(q);
    } else {
      // 圆弧运动
      const double radius = linear_vel / angular_vel;
      const double delta_theta = angular_vel * t;

      pose.pose.position.x =
          x + radius * (std::sin(theta + delta_theta) - std::sin(theta));
      pose.pose.position.y =
          y - radius * (std::cos(theta + delta_theta) - std::cos(theta));
      pose.pose.position.z = robot_pose.pose.position.z;

      tf2::Quaternion q;
      q.setRPY(0, 0, theta + delta_theta);
      pose.pose.orientation = tf2::toMsg(q);
    }

    trajectory.poses.push_back(pose);
  }

  return trajectory;
}

void PurePursuitLocalPlanner::updatePathProgress(
    const geometry_msgs::msg::PoseStamped &robot_pose) {
  if (global_path_.poses.empty()) return;

  const double robot_x = robot_pose.pose.position.x;
  const double robot_y = robot_pose.pose.position.y;

  double min_distance = std::numeric_limits<double>::max();
  size_t closest_index = path_progress_index_;

  // 从当前进度开始向前搜索最近点（允许少量向前搜索）
  const size_t search_start = path_progress_index_;
  const size_t search_end =
      std::min(path_progress_index_ + 20, global_path_.poses.size());

  for (size_t i = search_start; i < search_end; ++i) {
    const auto &waypoint = global_path_.poses[i].pose.position;
    const double distance =
        std::hypot(robot_x - waypoint.x, robot_y - waypoint.y);

    if (distance < min_distance) {
      min_distance = distance;
      closest_index = i;
    }
  }

  // 只允许进度向前更新，防止回头
  if (closest_index > path_progress_index_) {
    path_progress_index_ = closest_index;
    publishCurrentObstacleStopPolicy();
    RCLCPP_DEBUG(node_->get_logger(), "Path progress updated to index %zu",
                 path_progress_index_);
  }
}

}  // namespace pure_pursuit_local_planner
