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
    int current_path_index, geometry_msgs::msg::Twist &cmd_vel) {
  const double effective_angular_velocity =
      std::clamp(angular_velocity_limit, 0.0, config_.max_angular_velocity);
  const double velocity_ceiling = std::abs(target_velocity);
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

  // 前后退共用跟踪状态机：倒车以车尾为运动朝向，线速度取负；角速度不反号。
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
      const double min_walk_wz = 0.1;
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
