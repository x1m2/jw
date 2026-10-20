#include "pure_pursuit/pure_pursuit_controller.hpp"

#include <algorithm>
#include <cmath>
#include <tf2/LinearMath/Scalar.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace pure_pursuit_local_planner {

bool PurePursuitController::initialize(
    rclcpp::Node::SharedPtr node, const PurePursuitControllerConfig &config) {
  node_ = std::move(node);
  config_ = config;
  resetTrackingState();
  return true;
}

void PurePursuitController::resetTrackingState() {
  aligning_ = false;
  correcting_ = false;
  local_avoidance_aligning_ = false;
  reverse_terminal_active_ = false;
  reverse_terminal_failed_ = false;
  reverse_terminal_failure_reason_ = "";
  reverse_terminal_progress_since_ = {};
  reverse_terminal_goal_capture_ = false;
  reverse_terminal_approach_yaw_ = 0.0;
}

void PurePursuitController::startLocalAvoidanceAlignment() {
  resetTrackingState();
  local_avoidance_aligning_ = true;
  aligning_ = false;
  correcting_ = false;
}

bool PurePursuitController::computeCommand(
    const geometry_msgs::msg::PoseStamped &robot_pose,
    const geometry_msgs::msg::PointStamped &lookahead_point,
    double target_velocity, double angular_velocity_limit,
    int motion_direction,
    bool local_replanning_active, std::size_t path_progress_index,
    int current_path_index, geometry_msgs::msg::Twist &cmd_vel,
    const ReverseTerminalContext *reverse_terminal) {
  const double effective_angular_velocity =
      std::clamp(angular_velocity_limit, 0.0, config_.max_angular_velocity);
  const double velocity_ceiling = std::abs(target_velocity);
  const bool reverse_global = motion_direction == -1 &&
                              !local_replanning_active && reverse_terminal;
  if (!reverse_global && reverse_terminal_active_) {
    // 模式变化不携带末段锁存；局部路径首次对齐状态仍由其入口独立管理。
    reverse_terminal_active_ = false;
    reverse_terminal_failed_ = false;
    reverse_terminal_progress_since_ = {};
    reverse_terminal_goal_capture_ = false;
    aligning_ = false;
    correcting_ = false;
  }
  if (reverse_global && !reverse_terminal_active_ &&
      reverse_terminal->remaining_path_length <= config_.reverse_terminal_distance &&
      std::hypot(reverse_terminal->goal.x - robot_pose.pose.position.x,
                 reverse_terminal->goal.y - robot_pose.pose.position.y) <=
          config_.reverse_terminal_distance) {
    reverse_terminal_active_ = true;
    reverse_terminal_failed_ = false;
    reverse_terminal_path_yaw_ = reverse_terminal->path_yaw;
    reverse_terminal_progress_since_ = {};
    reverse_terminal_goal_capture_ = false;
    aligning_ = false;
    correcting_ = false;
    RCLCPP_INFO(node_->get_logger(),
                "[PP_REVERSE_TERMINAL] entered, path_yaw=%.2f deg",
                tf2Degrees(reverse_terminal_path_yaw_));
  }
  if (reverse_terminal_active_) {
    return computeReverseTerminalCommand(robot_pose, *reverse_terminal,
                                         velocity_ceiling,
                                         effective_angular_velocity, cmd_vel);
  }
  const double dx = lookahead_point.point.x - robot_pose.pose.position.x;
  const double dy = lookahead_point.point.y - robot_pose.pose.position.y;
  const double distance = std::hypot(dx, dy);
  if (distance < 1e-6) {
    cmd_vel = geometry_msgs::msg::Twist();
    return false;
  }

  const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
  const double min_distance_factor = 0.3;
  const double max_distance_factor = 1.0;
  const double distance_factor =
      min_distance_factor +
      (max_distance_factor - min_distance_factor) *
          std::clamp(distance / config_.lookahead_distance_max, 0.0, 1.0);
  target_velocity *= distance_factor;

  const int direction = motion_direction == -1 ? -1 : 1;
  const double path_yaw = std::atan2(dy, dx);
  const double target_yaw = tf2NormalizeAngle(
      path_yaw + (direction == -1 ? M_PI : 0.0));
  const double yaw_error = tf2NormalizeAngle(target_yaw - robot_yaw);
  const double yaw_error_abs = std::abs(yaw_error);
  // 局部A*路径靠近障碍物时不能用全局路径的大角度边走边转策略，否则
  // Pure Pursuit会切过折角。超过局部进入阈值时重新原地对准。
  const double rotate_enter_threshold =
      local_replanning_active ? config_.local_avoidance_align_enter_angle
                              : tf2Radians(35.0);
  const double rotate_exit_threshold =
      local_replanning_active ? config_.local_avoidance_align_exit_angle
                              : tf2Radians(10.0);
  const double correct_enter_threshold = tf2Radians(8.0);
  const double correct_exit_threshold = tf2Radians(4.0);
  const double angular_gain = 0.7;

  cmd_vel = geometry_msgs::msg::Twist();

  // 刚切入局部路径时优先完成一次方向对齐，避免带线速度横切局部路径。
  if (local_replanning_active && local_avoidance_aligning_) {
    // 每一段新的局部路径都必须先对准到退出阈值内。不能因为初始误差
    // 小于进入阈值就直接前进，否则车体会切向障碍物而不是贴合A*路径。
    if (yaw_error_abs < config_.local_avoidance_align_exit_angle) {
      local_avoidance_aligning_ = false;
      aligning_ = false;
      correcting_ = false;
      RCLCPP_DEBUG(node_->get_logger(),
                   "[AVOID_ALIGN] aligned to local path, yaw_error=%.2f deg",
                   tf2Degrees(yaw_error));
    } else {
      cmd_vel.angular.z = angular_gain * yaw_error;
      const double min_avoid_align_wz =
          config_.local_avoidance_min_align_angular_velocity;
      if (std::abs(cmd_vel.angular.z) < min_avoid_align_wz) {
        cmd_vel.angular.z = std::copysign(min_avoid_align_wz, yaw_error);
      }
      cmd_vel.angular.z =
          std::clamp(cmd_vel.angular.z, -effective_angular_velocity,
                     effective_angular_velocity);
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
                           "[AVOID_ALIGN] rotate in place before local path, "
                           "robot_yaw=%.2f deg, target_yaw=%.2f deg, "
                           "yaw_error=%.2f deg, vx=%.2f, wz=%.2f",
                           tf2Degrees(robot_yaw), tf2Degrees(target_yaw),
                           tf2Degrees(yaw_error), cmd_vel.linear.x,
                           cmd_vel.angular.z);
      return true;
    }
  }

  // 大角度原地旋转采用进入/退出双阈值，避免在临界角附近频繁切换。
  if (!aligning_ && yaw_error_abs > rotate_enter_threshold) aligning_ = true;
  if (aligning_ && yaw_error_abs < rotate_exit_threshold) aligning_ = false;

  if (aligning_) {
    cmd_vel.angular.z = angular_gain * yaw_error;
    const double min_rotate_wz = 0.35;
    if (std::abs(cmd_vel.angular.z) < min_rotate_wz) {
      cmd_vel.angular.z = std::copysign(min_rotate_wz, yaw_error);
    }
    RCLCPP_INFO_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 500,
        "[PP_ALIGN] robot_yaw=%.2f deg, target_yaw=%.2f deg, "
        "yaw_error=%.2f deg, rotate in place, vx=%.2f, wz=%.2f",
        tf2Degrees(robot_yaw), tf2Degrees(target_yaw), tf2Degrees(yaw_error),
        cmd_vel.linear.x, cmd_vel.angular.z);
  } else {
    // 中等角度误差同样使用滞回，保持原来的边走边修正行为。
    if (!correcting_ && yaw_error_abs > correct_enter_threshold) {
      correcting_ = true;
    }
    if (correcting_ && yaw_error_abs < correct_exit_threshold) {
      correcting_ = false;
    }

    if (correcting_) {
      cmd_vel.linear.x = direction * target_velocity;
      cmd_vel.angular.z = angular_gain * yaw_error;
      const double min_walk_wz = 0.08;
      if (std::abs(cmd_vel.angular.z) < min_walk_wz) {
        cmd_vel.angular.z = std::copysign(min_walk_wz, yaw_error);
      }
      const double max_walk_wz = 0.35;
      cmd_vel.angular.z = std::clamp(
          cmd_vel.angular.z, -std::min(max_walk_wz, effective_angular_velocity),
          std::min(max_walk_wz, effective_angular_velocity));
    } else {
      const double yaw_deg = tf2Degrees(yaw_error_abs);
      cmd_vel.linear.x = direction * target_velocity;
      if (yaw_deg > 4.0) {
        cmd_vel.angular.z = 0.30 * yaw_error;
        const double max_small_wz = 0.08;
        cmd_vel.angular.z =
            std::clamp(cmd_vel.angular.z, -max_small_wz, max_small_wz);
      }
    }
  }

  cmd_vel.angular.z = std::clamp(cmd_vel.angular.z, -effective_angular_velocity,
                                 effective_angular_velocity);
  // 局部绕障进入行走阶段后保持至少0.4m/s巡航；原地对准时vx为0，
  // 边策略给出的速度上限更低时仍优先服从上限。
  if (local_replanning_active && std::abs(cmd_vel.linear.x) > 1e-6) {
    const double minimum_cruise = std::min(
        config_.local_avoidance_min_linear_velocity, velocity_ceiling);
    if (std::abs(cmd_vel.linear.x) < minimum_cruise) {
      cmd_vel.linear.x = std::copysign(minimum_cruise, cmd_vel.linear.x);
    }
  }
  RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 500,
      "[PP_CMD] robot=(%.3f, %.3f, yaw=%.2f deg), "
      "lookahead=(%.3f, %.3f), dx=%.3f, dy=%.3f, "
      "target_yaw=%.2f deg, yaw_error=%.2f deg, vx=%.2f, wz=%.2f, "
      "progress=%zu, path_idx=%d, direction=%d, max_v=%.2f",
      robot_pose.pose.position.x, robot_pose.pose.position.y,
      tf2Degrees(robot_yaw), lookahead_point.point.x, lookahead_point.point.y,
      dx, dy, tf2Degrees(target_yaw), tf2Degrees(yaw_error), cmd_vel.linear.x,
      cmd_vel.angular.z, path_progress_index, current_path_index,
      direction,
      config_.max_linear_velocity);
  return true;
}

bool PurePursuitController::computeReverseTerminalCommand(
    const geometry_msgs::msg::PoseStamped &robot_pose,
    const ReverseTerminalContext &context, double velocity_ceiling,
    double angular_velocity_limit, geometry_msgs::msg::Twist &cmd_vel) {
  cmd_vel = geometry_msgs::msg::Twist();
  const double tx = std::cos(reverse_terminal_path_yaw_);
  const double ty = std::sin(reverse_terminal_path_yaw_);
  const double dx = context.goal.x - robot_pose.pose.position.x;
  const double dy = context.goal.y - robot_pose.pose.position.y;
  const double distance = std::hypot(dx, dy);
  const double remaining = dx * tx + dy * ty;
  const double lateral = dx * ty - dy * tx;
  const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
  const double heading_error = tf2NormalizeAngle(
      reverse_terminal_path_yaw_ + M_PI - robot_yaw);
  // 主规划器负责锁存成功；此处也避免已在容差内时继续产生运动指令。
  if (distance < config_.goal_tolerance) return true;

  // 当前车尾方向的射线：投影为正且横向距离小于半径，才可能进入到点圆。
  // 这里只判断接近几何，完整车体碰撞仍由主规划器检查。
  const double rear_along = -dx * std::cos(robot_yaw) - dy * std::sin(robot_yaw);
  const double rear_lateral = dx * std::sin(robot_yaw) - dy * std::cos(robot_yaw);
  const auto rear_reaches_goal = [&](double radius_scale) {
    return rear_along > 0.0 &&
           std::abs(rear_lateral) < radius_scale * config_.goal_tolerance;
  };
  const bool was_aligning = aligning_;
  if (!reverse_terminal_goal_capture_ &&
      distance <= std::max(0.30, 3.0 * config_.goal_tolerance)) {
    reverse_terminal_goal_capture_ = true;
    reverse_terminal_progress_since_ = {};
    aligning_ = !rear_reaches_goal(0.90);
    reverse_terminal_approach_yaw_ = aligning_
        ? tf2NormalizeAngle(std::atan2(dy, dx) + M_PI) : robot_yaw;
    RCLCPP_INFO(node_->get_logger(),
                "[PP_REVERSE_CAPTURE] entered distance=%.3f rear_along=%.3f "
                "rear_lateral=%.3f aligning=%d", distance, rear_along,
                rear_lateral, aligning_ ? 1 : 0);
  }

  const double correction_length = std::clamp(remaining, 0.40, 0.60);
  const double lateral_angle = std::clamp(
      std::atan2(lateral, correction_length), -tf2Radians(30.0), tf2Radians(30.0));
  double steering_error = tf2NormalizeAngle(heading_error - lateral_angle);
  if (reverse_terminal_goal_capture_) {
    if (!reverse_terminal_failed_) {
      if (!aligning_ && !rear_reaches_goal(0.95)) {
        // 行走方向已不能进入到点圆，停车后锁定一次新的对准方向。
        aligning_ = true;
        reverse_terminal_approach_yaw_ =
            tf2NormalizeAngle(std::atan2(dy, dx) + M_PI);
      }
      steering_error = tf2NormalizeAngle(reverse_terminal_approach_yaw_ - robot_yaw);
      if (aligning_ && std::abs(steering_error) < tf2Radians(10.0)) {
        if (rear_reaches_goal(0.90)) {
          aligning_ = false;
          reverse_terminal_approach_yaw_ = robot_yaw;
        } else {
          // 转向导致base_link位置变化时，完成本次对准后再计算参考；
          // 旋转期间不逐周期追逐方位。此处不重置位置进展超时。
          reverse_terminal_approach_yaw_ =
              tf2NormalizeAngle(std::atan2(dy, dx) + M_PI);
        }
      }
    }
    steering_error = tf2NormalizeAngle(reverse_terminal_approach_yaw_ - robot_yaw);
  } else {
    if (!aligning_ && std::abs(steering_error) > tf2Radians(35.0)) aligning_ = true;
    if (aligning_ && std::abs(steering_error) < tf2Radians(10.0)) aligning_ = false;
  }

  const auto now = std::chrono::steady_clock::now();
  // 近终点始终检查二维位置进展；切换对准/行走或更新方向不能刷新计时，
  // 防止参考点随转向移动时不断重对准、永远不能退出。
  const bool check_heading = !reverse_terminal_goal_capture_ && aligning_;
  const double progress_error = check_heading ? std::abs(steering_error) : distance;
  const double improvement = check_heading ? tf2Radians(3.0)
      : std::min(0.02, 0.2 * config_.goal_tolerance);
  if (reverse_terminal_progress_since_ == std::chrono::steady_clock::time_point{} ||
      (!reverse_terminal_goal_capture_ && was_aligning != aligning_) ||
      context.motion_blocked || velocity_ceiling <= 1e-6 ||
      progress_error < reverse_terminal_best_error_ - improvement) {
    reverse_terminal_best_error_ = progress_error;
    reverse_terminal_progress_since_ = now;
  }
  if (!reverse_terminal_failed_ &&
      std::chrono::duration<double>(now - reverse_terminal_progress_since_).count() >=
      config_.reverse_terminal_progress_timeout) {
    reverse_terminal_failed_ = true;
    reverse_terminal_failure_reason_ = "progress_timeout";
  }
  // 到点仍由主规划器按二维容差锁存，投影到终点不等于位置到达。
  if (reverse_terminal_failed_) {
    RCLCPP_ERROR_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "[PP_REVERSE_TERMINAL_STOP] reason=%s distance=%.3f along=%.3f "
        "lateral=%.3f heading_error=%.2f deg; new path or task reset required",
        reverse_terminal_failure_reason_, distance, remaining, lateral,
        tf2Degrees(heading_error));
    return true;
  }

  if (velocity_ceiling > 1e-6) {
    if (aligning_) {
      cmd_vel.angular.z = std::clamp(
          std::copysign(std::max(0.35, 0.7 * std::abs(steering_error)), steering_error),
          -angular_velocity_limit, angular_velocity_limit);
    } else {
      double speed;
      if (reverse_terminal_goal_capture_) {
        // 按二维距离减速，不因路径纵向投影为零而失去后退速度。
        // 制动空间取当前后退射线到0.8倍容差圆的入口；未穿过该内圆时
        // 取最近点，避免在到点边界外提前把目标速度压到零。
        const double inner_radius = 0.8 * config_.goal_tolerance;
        const double braking_distance = std::max(0.0, rear_along - std::sqrt(
            std::max(0.0, inner_radius * inner_radius - rear_lateral * rear_lateral)));
        const double braking_speed = std::sqrt(
            2.0 * std::max(0.0, config_.deceleration_limit) * braking_distance);
        speed = std::min({velocity_ceiling, config_.reverse_terminal_max_velocity,
                          0.8 * distance, braking_speed});
      } else {
        speed = std::min({velocity_ceiling, config_.reverse_terminal_max_velocity,
                          0.8 * std::max(0.0, remaining)});
        if (std::abs(lateral) > 0.8 * config_.goal_tolerance) {
          speed *= std::clamp(remaining / (2.0 * std::abs(lateral) + 0.05), 0.2, 1.0);
        }
      }
      speed *= std::max(0.0, std::cos(
          reverse_terminal_goal_capture_ ? steering_error : heading_error));
      double angular = 1.2 * steering_error;
      const double angular_limit = std::min(0.35, angular_velocity_limit);
      if (std::abs(angular) > angular_limit) {
        const double scale = angular_limit / std::abs(angular);
        speed *= scale;
        angular *= scale;
      }
      cmd_vel.linear.x = -speed;
      cmd_vel.angular.z = angular;
    }
  }
  RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 500,
      "[PP_REVERSE_TERMINAL] distance=%.3f along=%.3f lateral=%.3f "
      "heading_error=%.2f deg aligning=%d vx=%.3f wz=%.3f "
      "capture=%d rear_along=%.3f rear_lateral=%.3f steering_error=%.2f deg",
      distance, remaining, lateral, tf2Degrees(heading_error), aligning_ ? 1 : 0,
      cmd_vel.linear.x, cmd_vel.angular.z, reverse_terminal_goal_capture_ ? 1 : 0,
      rear_along, rear_lateral, tf2Degrees(steering_error));
  return true;
}

void PurePursuitController::applyVelocityLimits(
    const geometry_msgs::msg::Twist &last_cmd_vel, double dt,
    double linear_velocity_limit, double angular_velocity_limit,
    geometry_msgs::msg::Twist &cmd_vel) const {
  // 即使控制时钟尚未推进或发生回拨，也不能让横移分量绕过限速出口。
  cmd_vel.linear.y = 0.0;
  if (dt <= 0.0) return;
  const double target_linear_velocity = cmd_vel.linear.x;
  const double target_angular_velocity = cmd_vel.angular.z;
  const double effective_linear_velocity =
      std::clamp(linear_velocity_limit, 0.0, config_.max_linear_velocity);
  const double effective_angular_velocity =
      std::clamp(angular_velocity_limit, 0.0, config_.max_angular_velocity);
  cmd_vel.linear.x = std::clamp(cmd_vel.linear.x, -effective_linear_velocity,
                                effective_linear_velocity);
  // 当前机器为非全向四轮足，底盘不执行linear.y。无论上层配置如何，
  // 都在最终限速出口强制清零，防止再次出现“规划器以为正在横移、实车不动”。
  cmd_vel.linear.y = 0.0;

  const double velocity_diff_x = cmd_vel.linear.x - last_cmd_vel.linear.x;
  const bool accelerating_x =
      std::abs(cmd_vel.linear.x) > std::abs(last_cmd_vel.linear.x);
  const double max_velocity_change_x =
      (accelerating_x ? config_.acceleration_limit
                      : config_.deceleration_limit) *
      dt;
  if (std::abs(velocity_diff_x) > max_velocity_change_x) {
    cmd_vel.linear.x = last_cmd_vel.linear.x +
                       std::copysign(max_velocity_change_x, velocity_diff_x);
  }

  cmd_vel.angular.z = std::clamp(cmd_vel.angular.z, -effective_angular_velocity,
                                 effective_angular_velocity);
  cmd_vel.linear.x = std::clamp(cmd_vel.linear.x, -effective_linear_velocity,
                                effective_linear_velocity);
  cmd_vel.linear.y = 0.0;
  // 起步/加速限幅时沿候选曲线降低速度，避免只降低线速度导致急转弯。
  // 纯旋转、减速以及换向过零前保留原有处理。
  if (std::abs(target_linear_velocity) > 1e-6 &&
      cmd_vel.linear.x * target_linear_velocity >= 0.0 &&
      std::abs(cmd_vel.linear.x) < std::abs(target_linear_velocity)) {
    const double scale = cmd_vel.linear.x / target_linear_velocity;
    cmd_vel.angular.z = std::clamp(
        target_angular_velocity * scale,
        -effective_angular_velocity, effective_angular_velocity);
  }
}

bool PurePursuitController::isTargetOrientationReached(
    const geometry_msgs::msg::PoseStamped &robot_pose,
    const geometry_msgs::msg::Pose &target_pose) const {
  const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
  const double target_yaw = tf2::getYaw(target_pose.orientation);
  return std::abs(tf2NormalizeAngle(target_yaw - robot_yaw)) <
         config_.yaw_tolerance;
}

bool PurePursuitController::computeOrientationAdjustmentCommand(
    const geometry_msgs::msg::PoseStamped &robot_pose,
    const geometry_msgs::msg::Pose &target_pose, double angular_velocity_limit,
    geometry_msgs::msg::Twist &cmd_vel) const {
  const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
  const double target_yaw = tf2::getYaw(target_pose.orientation);
  const double yaw_error = tf2NormalizeAngle(target_yaw - robot_yaw);

  cmd_vel = geometry_msgs::msg::Twist();
  cmd_vel.angular.z = config_.yaw_adjustment_gain * yaw_error;
  if (std::abs(cmd_vel.angular.z) < config_.final_yaw_min_angular_velocity) {
    cmd_vel.angular.z =
        std::copysign(config_.final_yaw_min_angular_velocity, yaw_error);
  }
  cmd_vel.angular.z = std::clamp(
      cmd_vel.angular.z,
      -std::min(angular_velocity_limit, config_.max_angular_velocity),
      std::min(angular_velocity_limit, config_.max_angular_velocity));
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
                       "[FINAL_YAW] robot_yaw=%.2f deg, target_yaw=%.2f deg, "
                       "yaw_error=%.2f deg, vx=%.2f, wz=%.2f",
                       tf2Degrees(robot_yaw), tf2Degrees(target_yaw),
                       tf2Degrees(yaw_error), cmd_vel.linear.x,
                       cmd_vel.angular.z);
  return true;
}

}  // namespace pure_pursuit_local_planner
