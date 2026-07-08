#include "pure_pursuit_local_planner/pure_pursuit_local_planner.hpp"

#include <limits>

#include "pure_pursuit_local_planner/param_utils.hpp"

namespace pure_pursuit_local_planner {

PurePursuitLocalPlanner::PurePursuitLocalPlanner()
    : smoothed_local_path_received_(false),
      path_received_(false),
      goal_reached_(false),
      goal_position_reached_(false),
      yaw_adjusting_(false),
      target_pose_received_(false),
      current_path_index_(0),
      path_progress_index_(0),
      has_valid_lookahead_(false),
      pp_aligning_(false),
      pp_correcting_(false),
      local_replanning_active_(false),
      local_avoidance_aligning_(false),
      has_last_avoidance_complete_time_(false),
      latest_costmap_received_(
          false)  // 新增：路径进度索引，防止回头，平滑局部路径标志，朝向调整标志，目标位姿标志
{}

PurePursuitLocalPlanner::~PurePursuitLocalPlanner() {}

bool PurePursuitLocalPlanner::initialize(rclcpp::Node::SharedPtr node) {
  node_ = node;

  // 加载参数
  loadParameters();

  // 初始化TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // 初始化组件
  obstacle_avoidance_ = std::make_unique<ObstacleAvoidance>();
  if (!obstacle_avoidance_->initialize(node_)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Failed to initialize obstacle avoidance");
    return false;
  }

  path_tracker_ = std::make_unique<PathTracker>();
  if (!path_tracker_->initialize(node_)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to initialize path tracker");
    return false;
  }

  // 只有在启用局部重规划时才初始化局部路径规划器
  if (config_.enable_local_replanning) {
    local_path_planner_ = std::make_unique<LocalPathPlanner>();
    if (!local_path_planner_->initialize(node_, tf_buffer_)) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Failed to initialize local path planner");
      return false;
    }
    RCLCPP_INFO(node_->get_logger(),
                "Local path planner (A* replanning) enabled");
  } else {
    RCLCPP_INFO(node_->get_logger(),
                "Local path planner (A* replanning) disabled");
  }

  // 初始化前方障碍物检测器
  front_obstacle_detector_ = std::make_unique<FrontObstacleDetector>();
  if (!front_obstacle_detector_->initialize(node_, tf_buffer_)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Failed to initialize front obstacle detector");
    return false;
  }

  // 初始化发布器
  cmd_vel_pub_ =
      node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
  lookahead_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
      "lookahead_point", 1);
  path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("local_path", 1);
  local_trajectory_pub_ =
      node_->create_publisher<nav_msgs::msg::Path>("local_trajectory", 1);
  velocity_arrow_pub_ =
      node_->create_publisher<visualization_msgs::msg::Marker>("velocity_arrow",
                                                               1);
  nav_state_pub_ =
      node_->create_publisher<std_msgs::msg::Bool>("/nav_state", 1);

  // 初始化订阅器
  global_path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
      "/global_path", rclcpp::QoS(10),
      [this](nav_msgs::msg::Path::ConstSharedPtr msg) {
        this->globalPathCallback(msg);
      });

  costmap_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/local_costmap", rclcpp::QoS(10),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
        this->costmapCallback(msg);
      });

  // scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
  //     config_.scan_topic,
  //     rclcpp::QoS(10),
  //     [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
  //         this->scanCallback(msg);
  //     });

  smoothed_local_path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
      "/pure_pursuit_local_planner/smoothed_local_path", rclcpp::QoS(10),
      [this](nav_msgs::msg::Path::ConstSharedPtr msg) {
        this->smoothedLocalPathCallback(msg);
      });

  target_points_sub_ = node_->create_subscription<geometry_msgs::msg::Pose>(
      "/target_points", rclcpp::QoS(10),
      [this](geometry_msgs::msg::Pose::ConstSharedPtr msg) {
        this->targetPointsCallback(msg);
      });

  emergency_stop_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/emergency_stop", rclcpp::QoS(10),
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        this->emergencyStopCallback(msg);
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

bool PurePursuitLocalPlanner::isLocalReplanningEnabled() const {
  return config_.enable_local_replanning && local_path_planner_ != nullptr;
}

void PurePursuitLocalPlanner::setLocalReplanningActive(bool active) {
  if (!isLocalReplanningEnabled()) {
    local_replanning_active_ = false;
    smoothed_local_path_received_ = false;
    return;
  }

  if (local_replanning_active_ == active) {
    return;
  }

  local_replanning_active_ = active;
  if (!active) {
    smoothed_local_path_received_ = false;
    local_avoidance_aligning_ = false;
    pp_aligning_ = false;
    pp_correcting_ = false;
    if (local_path_planner_) {
      local_path_planner_->resetAvoidance();
    }
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "[OBSTACLE_REPLAN] inactive, tracking global path");
    return;
  }

  if (path_received_) {
    local_path_planner_->setGlobalPath(global_path_);
  }
  if (latest_costmap_received_) {
    local_path_planner_->updateCostmap(latest_costmap_);
  }
  local_avoidance_aligning_ = true;
  pp_aligning_ = false;
  pp_correcting_ = false;
  goal_position_reached_ = false;
  yaw_adjusting_ = false;
  last_cmd_vel_ = geometry_msgs::msg::Twist();
  RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
                       "[OBSTACLE_REPLAN] active, tracking local A* path");
}

void PurePursuitLocalPlanner::emergencyStopCallback(
    std_msgs::msg::Bool::ConstSharedPtr msg) {
  emergency_stop_active_ = msg->data;
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
      declareOrGetParameter(node_, "max_linear_velocity_y", 0.5);
  config_.max_angular_velocity =
      declareOrGetParameter(node_, "max_angular_velocity", 0.8);
  config_.min_linear_velocity =
      declareOrGetParameter(node_, "min_linear_velocity", 0.1);
  config_.acceleration_limit =
      declareOrGetParameter(node_, "acceleration_limit", 0.5);
  config_.deceleration_limit =
      declareOrGetParameter(node_, "deceleration_limit", 0.8);

  // Obstacle avoidance parameters
  config_.obstacle_detection_range =
      declareOrGetParameter(node_, "obstacle_detection_range", 2.0);
  config_.safety_distance =
      declareOrGetParameter(node_, "safety_distance", 0.3);

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
  // Local planning parameters
  config_.enable_obstacle_replanning =
      declareOrGetParameter(node_, "enable_obstacle_replanning", false);
  config_.enable_local_replanning =
      declareOrGetParameter(node_, "enable_local_replanning", false);
  config_.use_smoothed_path =
      declareOrGetParameter(node_, "use_smoothed_path", false);
  config_.avoidance_reentry_cooldown =
      declareOrGetParameter(node_, "avoidance_reentry_cooldown", 2.0);

  if (config_.enable_obstacle_replanning && !config_.enable_local_replanning) {
    RCLCPP_WARN(node_->get_logger(),
                "enable_obstacle_replanning=true but enable_local_replanning=false; obstacle avoidance will fall back to stop.");
  }

  // Frame IDs
  config_.global_frame =
      declareOrGetParameter<std::string>(node_, "global_frame", "map");
  config_.robot_frame =
      declareOrGetParameter<std::string>(node_, "robot_frame", "base_link");
}

bool PurePursuitLocalPlanner::setGlobalPath(
    const nav_msgs::msg::Path &global_path) {
  if (global_path.poses.empty()) {
    RCLCPP_WARN(node_->get_logger(), "Received empty global path");
    return false;
  }

  global_path_ = global_path;
  path_received_ = true;
  goal_reached_ = false;
  goal_position_reached_ = false;  // 新增：新路径开始，重置到点锁存
  pp_aligning_ = false;
  pp_correcting_ = false;
  yaw_adjusting_ = false;  // 新增：重置朝向调整状态
  current_path_index_ = 0;
  path_progress_index_ = 0;      // 新增：重置路径进度，防止回头
  has_valid_lookahead_ = false;  // 重置前瞻点状态
  setLocalReplanningActive(false);

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
    RCLCPP_WARN(node_->get_logger(),
                "[AVOID_PATH_FILTER] passed global path to front detector: poses=%zu",
                global_path_.poses.size());
  } else {
    RCLCPP_WARN(node_->get_logger(),
                "[AVOID_PATH_FILTER] front detector is null, cannot pass global path");
  }

  // 只有在启用局部重规划时才将路径传递给局部路径规划器
  if (config_.enable_local_replanning && local_path_planner_) {
    local_path_planner_->setGlobalPath(global_path_);
    RCLCPP_INFO(node_->get_logger(),
                "Global path passed to local planner for A* replanning");
  } else {
    RCLCPP_INFO(node_->get_logger(),
                "Local replanning disabled, using global path directly");
  }

  RCLCPP_INFO(node_->get_logger(), "Global path set with %zu poses",
              global_path_.poses.size());
  for (size_t i = 0; i < std::min<size_t>(global_path_.poses.size(), 20); ++i) {
    const auto &p = global_path_.poses[i].pose.position;
    RCLCPP_INFO(node_->get_logger(), "[GLOBAL_PATH] idx=%zu x=%.3f y=%.3f", i,
                p.x, p.y);
  }
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

  if (local_replanning_active_ && goal_position_reached_) {
    goal_position_reached_ = false;
    yaw_adjusting_ = false;
    last_cmd_vel_ = geometry_msgs::msg::Twist();
  }

  // 位置到达锁存：第一次进入 goal_tolerance 后，就锁住“位置已到达”
  if (!local_replanning_active_ && !goal_position_reached_ && isGoalReached()) {
    goal_position_reached_ = true;
    yaw_adjusting_ = true;

    // 到点后先清零上一帧速度，避免最终调角度时还带线速度
    last_cmd_vel_ = geometry_msgs::msg::Twist();

    RCLCPP_INFO(node_->get_logger(),
                "Goal position reached! Start final yaw adjustment...");
  }

  // 如果位置已经到达过，则只做最终角度调整，不再回 Pure Pursuit
  if (!local_replanning_active_ && goal_position_reached_) {
    if (target_pose_received_ && !isTargetOrientationReached(robot_pose)) {
      if (computeOrientationAdjustmentCommand(robot_pose, cmd_vel)) {
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
    yaw_adjusting_ = false;

    std_msgs::msg::Bool nav_state_msg;
    nav_state_msg.data = true;
    nav_state_pub_->publish(nav_state_msg);

    RCLCPP_INFO(node_->get_logger(), "Goal reached!");
    return true;
  }

  // emergency stop 逻辑
  if (emergency_stop_active_) {
    cmd_vel = geometry_msgs::msg::Twist();
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "Emergency stop active, cmd_vel forced to zero");
    return true;
  }

  if (config_.enable_obstacle_replanning && front_obstacle_detector_) {
    const auto avoidance_info =
        front_obstacle_detector_->analyzeAvoidance(robot_pose);

    if (avoidance_info.decision ==
        FrontObstacleDetector::ObstacleDecision::HARD_STOP) {
      cmd_vel = geometry_msgs::msg::Twist();
      last_cmd_vel_ = cmd_vel;
      if (local_replanning_active_) {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 500,
            "[OBSTACLE_REPLAN] hard stop during avoidance, waiting and keeping local plan");
        return true;
      }

      setLocalReplanningActive(false);
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
                           "[OBSTACLE_REPLAN] hard stop, cmd_vel forced to zero");
      return true;
    }

    if (!local_replanning_active_ && pp_aligning_) {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 500,
          "[OBSTACLE_REPLAN] ignored obstacle replanning during global PP_ALIGN");
    } else {
      if (isLocalReplanningEnabled() && local_replanning_active_ &&
          local_path_planner_) {
        if (avoidance_info.decision ==
            FrontObstacleDetector::ObstacleDecision::CLEAR) {
          RCLCPP_WARN_THROTTLE(
              node_->get_logger(), *node_->get_clock(), 1000,
              "[OBSTACLE_REPLAN] obstacle cleared during avoidance, keeping local plan until return goal");
        } else if (avoidance_info.decision ==
                   FrontObstacleDetector::ObstacleDecision::BLOCKED) {
          cmd_vel = geometry_msgs::msg::Twist();
          last_cmd_vel_ = cmd_vel;
          RCLCPP_WARN_THROTTLE(
              node_->get_logger(), *node_->get_clock(), 500,
              "[OBSTACLE_REPLAN] blocked during avoidance, waiting and keeping local plan");
          return true;
        }

        if (local_replanning_active_ &&
            local_path_planner_->isAvoidanceCompleted()) {
          const bool skip_current_goal =
              local_path_planner_->shouldSkipCurrentGoal();
          last_avoidance_complete_time_ = node_->now();
          has_last_avoidance_complete_time_ = true;
          setLocalReplanningActive(false);

          if (skip_current_goal) {
            cmd_vel = geometry_msgs::msg::Twist();
            last_cmd_vel_ = cmd_vel;
            goal_reached_ = true;
            goal_position_reached_ = false;
            yaw_adjusting_ = false;

            std_msgs::msg::Bool nav_state_msg;
            nav_state_msg.data = true;
            nav_state_pub_->publish(nav_state_msg);

            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 1000,
                "[OBSTACLE_REPLAN] no reachable forward path point, current goal skipped");
            return true;
          }

          RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                               "[OBSTACLE_REPLAN] avoidance completed, back to global path");
        } else if (local_path_planner_->isAvoidanceFailed()) {
          setLocalReplanningActive(false);
          cmd_vel = geometry_msgs::msg::Twist();
          last_cmd_vel_ = cmd_vel;
          RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
                               "[OBSTACLE_REPLAN] avoidance failed, cmd_vel forced to zero");
          return true;
        }
      }

      if (!local_replanning_active_) {
        if (has_last_avoidance_complete_time_ &&
            (node_->now() - last_avoidance_complete_time_).seconds() <
                config_.avoidance_reentry_cooldown) {
          // 冷却期内继续正常跟踪全局路径，避免刚回归就反复触发绕障。
        } else {
          if (avoidance_info.decision ==
              FrontObstacleDetector::ObstacleDecision::BLOCKED) {
            setLocalReplanningActive(false);
            cmd_vel = geometry_msgs::msg::Twist();
            last_cmd_vel_ = cmd_vel;
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
                                 "[OBSTACLE_REPLAN] blocked/hard stop, cmd_vel forced to zero");
            return true;
          }

          if (avoidance_info.decision ==
              FrontObstacleDetector::ObstacleDecision::AVOIDABLE) {
            if (!isLocalReplanningEnabled()) {
              cmd_vel = geometry_msgs::msg::Twist();
              last_cmd_vel_ = cmd_vel;
              RCLCPP_WARN_THROTTLE(
                  node_->get_logger(), *node_->get_clock(), 500,
                  "[OBSTACLE_REPLAN] avoidable but local replanning disabled");
              return true;
            }

            auto side = LocalPathPlanner::AvoidanceSide::NONE;
            if (avoidance_info.left_clear) {
              side = LocalPathPlanner::AvoidanceSide::LEFT;
            } else if (avoidance_info.right_clear) {
              side = LocalPathPlanner::AvoidanceSide::RIGHT;
            }

            if (path_received_) {
              local_path_planner_->setGlobalPath(global_path_);
            }
            if (latest_costmap_received_) {
              local_path_planner_->updateCostmap(latest_costmap_);
            }

            if (!local_path_planner_->startAvoidance(
                    robot_pose, avoidance_info.obstacle_center, side)) {
              setLocalReplanningActive(false);
              cmd_vel = geometry_msgs::msg::Twist();
              last_cmd_vel_ = cmd_vel;
              RCLCPP_WARN_THROTTLE(
                  node_->get_logger(), *node_->get_clock(), 500,
                  "[OBSTACLE_REPLAN] could not build obstacle-based avoidance goals");
              return true;
            }

            setLocalReplanningActive(true);
          }
        }
      }
    }
  }

  // 计算前瞻点
  geometry_msgs::msg::PointStamped lookahead_point;
  if (!computeLookaheadPoint(robot_pose, lookahead_point)) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "Failed to compute lookahead point");
    cmd_vel = geometry_msgs::msg::Twist();
    if (local_replanning_active_) {
      last_cmd_vel_ = cmd_vel;
      return true;
    }
    return false;
  }

  // 计算Pure Pursuit控制指令
  if (!computePurePursuitCommand(robot_pose, lookahead_point, cmd_vel)) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "Failed to compute pure pursuit command");
    cmd_vel = geometry_msgs::msg::Twist();
    return false;
  }

  // 人工势场避障
  // if (obstacle_avoidance_)
  // {
  //     cmd_vel = obstacle_avoidance_->computePotentialFieldAvoidance(cmd_vel,
  //     robot_pose, lookahead_point);
  // }

  // 前方障碍物检测和速度调整：主动绕障关闭时保留旧停障/减速逻辑。
  if (!config_.enable_obstacle_replanning && front_obstacle_detector_) {
    cmd_vel = front_obstacle_detector_->adjustVelocityForObstacles(cmd_vel,
                                                                   robot_pose);
  }
  if (std::abs(cmd_vel.linear.x) < 1e-6 &&
      std::abs(cmd_vel.linear.y) < 1e-6 &&
      std::abs(cmd_vel.angular.z) < 1e-6) {
    last_cmd_vel_ = geometry_msgs::msg::Twist();
  }
  // 新增：原地旋转时清掉上一帧线速度
  if (std::abs(cmd_vel.linear.x) < 1e-6 && std::abs(cmd_vel.angular.z) > 1e-6) {
    last_cmd_vel_.linear.x = 0.0;
    last_cmd_vel_.linear.y = 0.0;
  }
  // 应用速度限制
  applyVelocityLimits(cmd_vel);

  // 发布可视化信息
  publishVisualization(robot_pose, lookahead_point);

  // 发布局部轨迹和速度可视化
  publishLocalTrajectory(robot_pose, cmd_vel);
  publishVelocityArrow(robot_pose, cmd_vel);

  last_cmd_vel_ = cmd_vel;
  last_control_time_ = node_->now();

  return true;
}

bool PurePursuitLocalPlanner::isGoalReached() {
  if (!path_received_ || global_path_.poses.empty()) return false;

  geometry_msgs::msg::PoseStamped robot_pose;
  if (!getRobotPose(robot_pose)) return false;

  const auto &goal_pose = global_path_.poses.back();
  double distance =
      calculateDistance(robot_pose.pose.position, goal_pose.pose.position);

  return distance < config_.goal_tolerance;
}

bool PurePursuitLocalPlanner::getRobotPose(
    geometry_msgs::msg::PoseStamped &robot_pose) {
  try {
    geometry_msgs::msg::TransformStamped transform =
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
  geometry_msgs::msg::Twist cmd_vel;
  if (computeVelocityCommands(cmd_vel)) {
    cmd_vel_pub_->publish(cmd_vel);
  }
}

void PurePursuitLocalPlanner::globalPathCallback(
    nav_msgs::msg::Path::ConstSharedPtr msg) {
  setGlobalPath(*msg);
}

void PurePursuitLocalPlanner::costmapCallback(
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
  latest_costmap_ = *msg;
  latest_costmap_received_ = true;

  if (obstacle_avoidance_) {
    obstacle_avoidance_->updateCostmap(*msg);
  }

  // 只有在启用局部重规划时才更新局部路径规划器的代价地图
  if (config_.enable_local_replanning && local_path_planner_) {
    local_path_planner_->updateCostmap(*msg);
  }

  // 更新前方障碍物检测器的代价地图
  if (front_obstacle_detector_) {
    front_obstacle_detector_->updateCostmap(*msg);
  }
}

void PurePursuitLocalPlanner::scanCallback(
    sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
  // 人工势场方法不使用激光雷达数据，仅保留回调函数以避免编译错误
  // 可以在这里添加其他激光雷达相关的处理逻辑（如果需要）
}

void PurePursuitLocalPlanner::smoothedLocalPathCallback(
    nav_msgs::msg::Path::ConstSharedPtr msg) {
  // 只有在启用平滑路径时才处理
  if (config_.use_smoothed_path && local_replanning_active_) {
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
    const geometry_msgs::msg::PoseStamped &robot_pose,
    geometry_msgs::msg::PointStamped &lookahead_point) {
  // 根据参数决定路径选择优先级
  nav_msgs::msg::Path path_to_use;
  bool use_local_path = false;
  std::string path_type;

  if (config_.enable_local_replanning && local_replanning_active_) {
    // 启用局部重规划时的优先级：平滑后局部路径 > 原始局部路径 > 全局路径
    if (config_.use_smoothed_path && smoothed_local_path_received_ &&
        !smoothed_local_path_.poses.empty()) {
      path_to_use = smoothed_local_path_;
      use_local_path = true;
      path_type = "smoothed local path";
      RCLCPP_DEBUG(node_->get_logger(),
                   "Using smoothed local path for lookahead point computation");
    } else if (local_path_planner_ &&
               local_path_planner_->hasValidLocalPath()) {
      path_to_use = local_path_planner_->getLocalPath();
      use_local_path = true;
      path_type = "original local path";
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
      path_to_use = global_path_;
      use_local_path = false;
      path_type = "global path";
      RCLCPP_DEBUG(node_->get_logger(),
                   "Local replanning disabled, using global path directly");
    } else {
      return false;
    }
  }

  if (path_to_use.poses.empty()) return false;

  // 计算前瞻距离
  double current_velocity =
      std::hypot(last_cmd_vel_.linear.x, last_cmd_vel_.linear.y);

  // 计算速度因子：将当前速度映射到 [0, 1] 范围
  double velocity_factor = 0.0;
  if (config_.max_linear_velocity > 0.0) {
    velocity_factor =
        std::min(current_velocity / config_.max_linear_velocity, 1.0);
  }

  // 根据速度因子在最小和最大前瞻距离之间插值
  double lookahead_distance =
      config_.lookahead_distance_min +
      velocity_factor *
          (config_.lookahead_distance_max - config_.lookahead_distance_min);

  // 机器人当前位置
  double robot_x = robot_pose.pose.position.x;
  double robot_y = robot_pose.pose.position.y;

  // 对于全局路径，需要更新路径进度防止回头；对于局部路径则不需要
  size_t start_index = 0;
  if (!use_local_path) {
    // 使用全局路径时，更新路径进度防止回头
    updatePathProgress(robot_pose);
    start_index = path_progress_index_;
  }

  // 查找以机器人为中心、前瞻距离为半径的圆与路径的交点
  std::vector<int> intersection_indices;
  std::vector<geometry_msgs::msg::Point> intersection_points;

  for (size_t i = start_index; i < path_to_use.poses.size() - 1; ++i) {
    const auto &p1 = path_to_use.poses[i].pose.position;
    const auto &p2 = path_to_use.poses[i + 1].pose.position;

    // 计算线段与圆的交点
    std::vector<geometry_msgs::msg::Point> segment_intersections =
        computeCircleLineIntersection(robot_x, robot_y, lookahead_distance, p1,
                                      p2);

    for (const auto &intersection : segment_intersections) {
      intersection_indices.push_back(i);
      intersection_points.push_back(intersection);
    }
  }

  // 如果有交点，选择最靠近终点的交点
  if (!intersection_points.empty()) {
    int best_index = 0;
    double max_progress = -1.0;

    for (size_t i = 0; i < intersection_points.size(); ++i) {
      // 对于局部路径，直接使用索引作为进度；对于全局路径，计算累积距离
      double progress;
      if (use_local_path) {
        progress = intersection_indices[i];  // 局部路径直接使用索引
      } else {
        progress = calculatePathProgress(intersection_indices[i],
                                         intersection_points[i]);
      }

      if (progress > max_progress) {
        max_progress = progress;
        best_index = i;
      }
    }

    lookahead_point.header.frame_id = config_.global_frame;
    lookahead_point.header.stamp = node_->now();
    lookahead_point.point = intersection_points[best_index];

    // 更新当前路径索引
    current_path_index_ = intersection_indices[best_index];

    // 保存当前前瞻点作为下次的备选
    last_lookahead_point_ = lookahead_point;
    has_valid_lookahead_ = true;

    return true;
  }

  // 如果没有交点，选择从当前进度开始的最远可见点作为前瞻点
  if (use_local_path) {
    // 对于局部路径，选择路径终点或距离起点一定距离的点
    size_t target_index =
        std::min(start_index + 10, path_to_use.poses.size() - 1);

    const auto &target_pose = path_to_use.poses[target_index];
    lookahead_point.header.frame_id = config_.global_frame;
    lookahead_point.header.stamp = node_->now();
    lookahead_point.point = target_pose.pose.position;

    current_path_index_ = target_index;

    // 保存当前前瞻点
    last_lookahead_point_ = lookahead_point;
    has_valid_lookahead_ = true;

    RCLCPP_DEBUG(node_->get_logger(),
                 "No intersection found on %s, using target point at index %zu",
                 path_type.c_str(), target_index);
    return true;
  } else if (path_progress_index_ < global_path_.poses.size()) {
    // 对于全局路径，选择距离当前进度一定距离的点，或者路径终点
    size_t target_index =
        std::min(path_progress_index_ + 10, global_path_.poses.size() - 1);

    const auto &target_pose = global_path_.poses[target_index];
    lookahead_point.header.frame_id = config_.global_frame;
    lookahead_point.header.stamp = node_->now();
    lookahead_point.point = target_pose.pose.position;

    current_path_index_ = target_index;

    // 保存当前前瞻点
    last_lookahead_point_ = lookahead_point;
    has_valid_lookahead_ = true;

    RCLCPP_DEBUG(node_->get_logger(),
                 "No intersection found on %s, using target point at index %zu",
                 path_type.c_str(), target_index);
    return true;
  }

  return false;
}

bool PurePursuitLocalPlanner::computePurePursuitCommand(
    const geometry_msgs::msg::PoseStamped &robot_pose,
    const geometry_msgs::msg::PointStamped &lookahead_point,
    geometry_msgs::msg::Twist &cmd_vel) {
  // 计算机器人到前瞻点的向量
  double dx = lookahead_point.point.x - robot_pose.pose.position.x;
  double dy = lookahead_point.point.y - robot_pose.pose.position.y;
  double distance = std::hypot(dx, dy);

  if (distance < 1e-6) {
    cmd_vel = geometry_msgs::msg::Twist();
    return false;
  }

  // 获取机器人朝向
  double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);

  // 计算目标线速度（基于机器人方向和前瞻点方向的点乘）
  double target_velocity = config_.max_linear_velocity;
  if (path_tracker_) {
    target_velocity = path_tracker_->computeTargetVelocity(
        robot_pose, lookahead_point, config_.max_linear_velocity);
  }

  // 计算距离比例因子：距离前瞻点越近，速度越小
  // 使用当前前瞻距离范围进行归一化
  double min_distance_factor = 0.3;  // 最小速度比例（距离很近时）
  double max_distance_factor = 1.0;  // 最大速度比例（距离较远时）

  // 将距离映射到 [min_distance_factor, max_distance_factor] 范围
  double distance_factor =
      min_distance_factor +
      (max_distance_factor - min_distance_factor) *
          std::min(distance / config_.lookahead_distance_max, 1.0);

  // 应用距离比例因子
  target_velocity *= distance_factor;

  // 计算X方向的速度分量（Y方向设为0）
  // cmd_vel.linear.x = target_velocity;
  // RCLCPP_DEBUG(node_->get_logger(), "target_velocity: %f\r\n",
  // target_velocity); cmd_vel.linear.y = 0.0;  // 固定设为0，不使用全向运动
  // cmd_vel.linear.z = 0.0;

  // 确保最小速度（仅对前进方向）
  // if (std::abs(cmd_vel.linear.x) > 0 && std::abs(cmd_vel.linear.x) <
  // config_.min_linear_velocity)
  // {
  //     cmd_vel.linear.x = std::copysign(config_.min_linear_velocity,
  //     cmd_vel.linear.x);
  // }

  // 计算前瞻点方向
  double target_yaw = std::atan2(dy, dx);
  double yaw_error = normalizeAngle(target_yaw - robot_yaw);
  double yaw_error_abs = std::abs(yaw_error);

  // 大角度先原地转，不前进
  double rotate_enter_threshold = 35.0 * M_PI / 180.0;  // >35° 原地旋转
  double rotate_exit_threshold = 10.0 * M_PI / 180.0;  // <10° 退出原地旋转

  double correct_enter_threshold = 8.0 * M_PI / 180.0;  // >8° 进入边走修正
  double correct_exit_threshold = 4.0 * M_PI / 180.0;  // <4° 退出边走修正

  double angular_gain = 0.7;

  // 默认清零
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.linear.z = 0.0;
  cmd_vel.angular.x = 0.0;
  cmd_vel.angular.y = 0.0;
  cmd_vel.angular.z = 0.0;

  if (local_replanning_active_ && local_avoidance_aligning_) {
    const double avoid_align_exit_threshold = 8.0 * M_PI / 180.0;
    if (yaw_error_abs < avoid_align_exit_threshold) {
      local_avoidance_aligning_ = false;
      pp_aligning_ = false;
      pp_correcting_ = false;
      RCLCPP_WARN(node_->get_logger(),
                  "[AVOID_ALIGN] aligned to local path, yaw_error=%.2f deg",
                  yaw_error * 180.0 / M_PI);
    } else {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = angular_gain * yaw_error;

      const double min_avoid_align_wz = 0.22;
      if (std::abs(cmd_vel.angular.z) < min_avoid_align_wz) {
        cmd_vel.angular.z = std::copysign(min_avoid_align_wz, yaw_error);
      }

      cmd_vel.angular.z =
          std::max(cmd_vel.angular.z, -config_.max_angular_velocity);
      cmd_vel.angular.z =
          std::min(cmd_vel.angular.z, config_.max_angular_velocity);

      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 300,
          "[AVOID_ALIGN] rotate in place before local path, yaw_error=%.2f deg, wz=%.2f",
          yaw_error * 180.0 / M_PI, cmd_vel.angular.z);
      return true;
    }
  }

  // 原地旋转滞回逻辑：
  if (!pp_aligning_ && yaw_error_abs > rotate_enter_threshold) {
    pp_aligning_ = true;
  }

  if (pp_aligning_ && yaw_error_abs < rotate_exit_threshold) {
    pp_aligning_ = false;
  }

  if (pp_aligning_) {
    // 原地旋转阶段：只转，不走
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = angular_gain * yaw_error;

    // 原地旋转最小角速度，避免小角速度转不动导致卡在对齐阶段。
    double min_rotate_wz = 0.35;
    if (std::abs(cmd_vel.angular.z) < min_rotate_wz) {
      cmd_vel.angular.z = std::copysign(min_rotate_wz, yaw_error);
    }

    RCLCPP_INFO_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 500,
        "[PP_ALIGN] yaw_error=%.2f deg, rotate in place, wz=%.2f",
        yaw_error * 180.0 / M_PI, cmd_vel.angular.z);

  } else {
    // 边走修正滞回逻辑：
    if (!pp_correcting_ && yaw_error_abs > correct_enter_threshold) {
      pp_correcting_ = true;
    }

    if (pp_correcting_ && yaw_error_abs < correct_exit_threshold) {
      pp_correcting_ = false;
    }

    if (pp_correcting_) {
      // 边走边修正
      target_velocity *= 0.9;
      cmd_vel.linear.x = target_velocity;
      cmd_vel.angular.z = angular_gain * yaw_error;

      double min_walk_wz = 0.08;
      if (std::abs(cmd_vel.angular.z) < min_walk_wz) {
        cmd_vel.angular.z = std::copysign(min_walk_wz, yaw_error);
      }

      double max_walk_wz = 0.35;
      cmd_vel.angular.z = std::max(cmd_vel.angular.z, -max_walk_wz);
      cmd_vel.angular.z = std::min(cmd_vel.angular.z, max_walk_wz);

    } else {
      double yaw_deg = yaw_error_abs * 180.0 / M_PI;

      cmd_vel.linear.x = target_velocity;

      // 小误差轻微修正，避免一直斜着走
      if (yaw_deg > 4.0) {
        cmd_vel.angular.z = 0.30 * yaw_error;

        double max_small_wz = 0.08;
        cmd_vel.angular.z = std::max(cmd_vel.angular.z, -max_small_wz);
        cmd_vel.angular.z = std::min(cmd_vel.angular.z, max_small_wz);
      } else {
        cmd_vel.angular.z = 0.0;
      }
    }
  }

  // 限制角速度
  cmd_vel.angular.z =
      std::max(cmd_vel.angular.z, -config_.max_angular_velocity);
  cmd_vel.angular.z = std::min(cmd_vel.angular.z, config_.max_angular_velocity);
  RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 300,
      "[PP_CMD] robot=(%.3f, %.3f, yaw=%.2f deg), "
      "lookahead=(%.3f, %.3f), dx=%.3f, dy=%.3f, "
      "target_yaw=%.2f deg, yaw_error=%.2f deg, vx=%.2f, wz=%.2f, "
      "progress=%zu, path_idx=%d",
      robot_pose.pose.position.x, robot_pose.pose.position.y,
      robot_yaw * 180.0 / M_PI, lookahead_point.point.x,
      lookahead_point.point.y, dx, dy, target_yaw * 180.0 / M_PI,
      yaw_error * 180.0 / M_PI, cmd_vel.linear.x, cmd_vel.angular.z,
      path_progress_index_, current_path_index_);
  return true;
}

void PurePursuitLocalPlanner::applyVelocityLimits(
    geometry_msgs::msg::Twist &cmd_vel) {
  double dt = (node_->now() - last_control_time_).seconds();
  if (dt <= 0.0) return;

  // 限制X方向线速度
  cmd_vel.linear.x = std::max(cmd_vel.linear.x, -config_.max_linear_velocity);
  cmd_vel.linear.x = std::min(cmd_vel.linear.x, config_.max_linear_velocity);

  // 限制Y方向线速度
  cmd_vel.linear.y = std::max(cmd_vel.linear.y, -config_.max_linear_velocity_y);
  cmd_vel.linear.y = std::min(cmd_vel.linear.y, config_.max_linear_velocity_y);

  // 限制加速度（X方向）
  double velocity_diff_x = cmd_vel.linear.x - last_cmd_vel_.linear.x;
  double max_velocity_change_x = (velocity_diff_x > 0)
                                     ? config_.acceleration_limit * dt
                                     : config_.deceleration_limit * dt;

  if (std::abs(velocity_diff_x) > max_velocity_change_x) {
    cmd_vel.linear.x = last_cmd_vel_.linear.x +
                       std::copysign(max_velocity_change_x, velocity_diff_x);
  }

  // 限制加速度（Y方向）
  double velocity_diff_y = cmd_vel.linear.y - last_cmd_vel_.linear.y;
  double max_velocity_change_y = (velocity_diff_y > 0)
                                     ? config_.acceleration_limit * dt
                                     : config_.deceleration_limit * dt;

  if (std::abs(velocity_diff_y) > max_velocity_change_y) {
    cmd_vel.linear.y = last_cmd_vel_.linear.y +
                       std::copysign(max_velocity_change_y, velocity_diff_y);
  }

  // 限制角速度
  cmd_vel.angular.z =
      std::max(cmd_vel.angular.z, -config_.max_angular_velocity);
  cmd_vel.angular.z = std::min(cmd_vel.angular.z, config_.max_angular_velocity);
}

void PurePursuitLocalPlanner::publishVisualization(
    const geometry_msgs::msg::PoseStamped &robot_pose,
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
    path_tracker_->publishVisualization(robot_pose, lookahead_point,
                                        current_path_index_);
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
  double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);

  // 将机器人坐标系下的速度转换到全局坐标系
  double global_vel_x = cmd_vel.linear.x * std::cos(robot_yaw) -
                        cmd_vel.linear.y * std::sin(robot_yaw);
  double global_vel_y = cmd_vel.linear.x * std::sin(robot_yaw) +
                        cmd_vel.linear.y * std::cos(robot_yaw);

  // 计算实际运动方向
  double arrow_yaw = std::atan2(global_vel_y, global_vel_x);

  tf2::Quaternion q;
  q.setRPY(0, 0, arrow_yaw);
  arrow.pose.orientation = tf2::toMsg(q);

  // 根据速度大小设置箭头长度
  double velocity_magnitude = std::hypot(cmd_vel.linear.x, cmd_vel.linear.y);
  double arrow_length =
      std::max(0.2, velocity_magnitude * 0.5);  // 最小长度0.2m

  arrow.scale.x = arrow_length;  // 箭头长度
  arrow.scale.y = 0.1;           // 箭头宽度
  arrow.scale.z = 0.1;           // 箭头高度

  // 根据速度大小设置颜色（绿色到红色渐变）
  arrow.color.a = 0.8;
  double max_combined_velocity =
      std::hypot(config_.max_linear_velocity, config_.max_linear_velocity_y);
  double speed_ratio = velocity_magnitude / max_combined_velocity;
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
  trajectory.header.stamp = node_->now();

  // 获取机器人当前状态
  double x = robot_pose.pose.position.x;
  double y = robot_pose.pose.position.y;
  double theta = tf2::getYaw(robot_pose.pose.orientation);

  double linear_vel = cmd_vel.linear.x;
  double angular_vel = cmd_vel.angular.z;

  // 生成预测轨迹点
  int num_steps = static_cast<int>(prediction_time / dt);
  for (int i = 0; i <= num_steps; ++i) {
    double t = i * dt;

    geometry_msgs::msg::PoseStamped pose;
    pose.header = trajectory.header;
    pose.header.stamp = node_->now() + rclcpp::Duration::from_seconds(t);

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
      double radius = linear_vel / angular_vel;
      double delta_theta = angular_vel * t;

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

double PurePursuitLocalPlanner::calculateDistance(
    const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2) {
  return std::hypot(std::hypot(p1.x - p2.x, p1.y - p2.y), p1.z - p2.z);
}

double PurePursuitLocalPlanner::normalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

std::vector<geometry_msgs::msg::Point>
PurePursuitLocalPlanner::computeCircleLineIntersection(
    double circle_x, double circle_y, double radius,
    const geometry_msgs::msg::Point &line_start,
    const geometry_msgs::msg::Point &line_end) {
  std::vector<geometry_msgs::msg::Point> intersections;

  // 线段向量
  double dx = line_end.x - line_start.x;
  double dy = line_end.y - line_start.y;

  // 从线段起点到圆心的向量
  double fx = line_start.x - circle_x;
  double fy = line_start.y - circle_y;

  // 求解二次方程 a*t^2 + b*t + c = 0
  double a = dx * dx + dy * dy;
  double b = 2.0 * (fx * dx + fy * dy);
  double c = fx * fx + fy * fy - radius * radius;

  double discriminant = b * b - 4.0 * a * c;

  if (discriminant < 0.0 || a < 1e-8) {
    // 无交点或线段长度为0
    return intersections;
  }

  double sqrt_discriminant = std::sqrt(discriminant);
  double t1 = (-b - sqrt_discriminant) / (2.0 * a);
  double t2 = (-b + sqrt_discriminant) / (2.0 * a);

  // 检查交点是否在线段上 (0 <= t <= 1)
  if (t1 >= 0.0 && t1 <= 1.0) {
    geometry_msgs::msg::Point intersection;
    intersection.x = line_start.x + t1 * dx;
    intersection.y = line_start.y + t1 * dy;
    intersection.z = line_start.z;
    intersections.push_back(intersection);
  }

  if (t2 >= 0.0 && t2 <= 1.0 && std::abs(t2 - t1) > 1e-8) {
    geometry_msgs::msg::Point intersection;
    intersection.x = line_start.x + t2 * dx;
    intersection.y = line_start.y + t2 * dy;
    intersection.z = line_start.z;
    intersections.push_back(intersection);
  }

  return intersections;
}

double PurePursuitLocalPlanner::calculatePathProgress(
    int segment_index, const geometry_msgs::msg::Point &point) {
  if (segment_index < 0 ||
      segment_index >= static_cast<int>(global_path_.poses.size() - 1))
    return -1.0;

  // 计算累积路径长度到该段的起点
  double cumulative_length = 0.0;
  for (int i = 0; i < segment_index; ++i) {
    cumulative_length +=
        calculateDistance(global_path_.poses[i].pose.position,
                          global_path_.poses[i + 1].pose.position);
  }

  // 计算点在当前段上的距离
  const auto &segment_start = global_path_.poses[segment_index].pose.position;
  double segment_progress = calculateDistance(segment_start, point);

  return cumulative_length + segment_progress;
}

void PurePursuitLocalPlanner::updatePathProgress(
    const geometry_msgs::msg::PoseStamped &robot_pose) {
  if (global_path_.poses.empty()) return;

  double robot_x = robot_pose.pose.position.x;
  double robot_y = robot_pose.pose.position.y;

  double min_distance = std::numeric_limits<double>::max();
  size_t closest_index = path_progress_index_;

  // 从当前进度开始向前搜索最近点（允许少量向前搜索）
  size_t search_start = path_progress_index_;
  size_t search_end =
      std::min(path_progress_index_ + 20, global_path_.poses.size());

  for (size_t i = search_start; i < search_end; ++i) {
    const auto &waypoint = global_path_.poses[i].pose.position;
    double distance = std::hypot(robot_x - waypoint.x, robot_y - waypoint.y);

    if (distance < min_distance) {
      min_distance = distance;
      closest_index = i;
    }
  }

  // 只允许进度向前更新，防止回头
  if (closest_index > path_progress_index_) {
    path_progress_index_ = closest_index;
    RCLCPP_DEBUG(node_->get_logger(), "Path progress updated to index %zu",
                 path_progress_index_);
  }
}

bool PurePursuitLocalPlanner::isTargetOrientationReached(
    const geometry_msgs::msg::PoseStamped &robot_pose) {
  if (!target_pose_received_) return true;

  double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
  double target_yaw = tf2::getYaw(target_pose_.orientation);

  double yaw_error = std::abs(normalizeAngle(target_yaw - robot_yaw));

  return yaw_error < config_.yaw_tolerance;
}

bool PurePursuitLocalPlanner::computeOrientationAdjustmentCommand(
    const geometry_msgs::msg::PoseStamped &robot_pose,
    geometry_msgs::msg::Twist &cmd_vel) {
  if (!target_pose_received_) return false;

  double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
  double target_yaw = tf2::getYaw(target_pose_.orientation);

  double yaw_error = normalizeAngle(target_yaw - robot_yaw);

  // 设置线速度为0，只调整角速度
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.linear.z = 0.0;
  cmd_vel.angular.x = 0.0;
  cmd_vel.angular.y = 0.0;

  // 计算角速度，使用比例控制
  cmd_vel.angular.z = config_.yaw_adjustment_gain * yaw_error;
  // 最终调角度给一个最小角速度，避免机械狗小角速度不动
  double min_final_yaw_wz = config_.final_yaw_min_angular_velocity;
  if (std::abs(cmd_vel.angular.z) < min_final_yaw_wz) {
    cmd_vel.angular.z = std::copysign(min_final_yaw_wz, yaw_error);
  }
  // 限制角速度
  cmd_vel.angular.z =
      std::max(cmd_vel.angular.z, -config_.max_angular_velocity);
  cmd_vel.angular.z = std::min(cmd_vel.angular.z, config_.max_angular_velocity);

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
                       "[FINAL_YAW] robot_yaw=%.2f deg, target_yaw=%.2f deg, "
                       "yaw_error=%.2f deg, wz=%.2f",
                       robot_yaw * 180.0 / M_PI, target_yaw * 180.0 / M_PI,
                       yaw_error * 180.0 / M_PI, cmd_vel.angular.z);

  return true;
}

}  // namespace pure_pursuit_local_planner
