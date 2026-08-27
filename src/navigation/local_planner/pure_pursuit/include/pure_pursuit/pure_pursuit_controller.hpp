#ifndef PURE_PURSUIT_CONTROLLER_HPP
#define PURE_PURSUIT_CONTROLLER_HPP

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

namespace pure_pursuit_local_planner {

struct PurePursuitControllerConfig {
  double lookahead_distance_max{1.0};
  double max_linear_velocity{0.5};
  double max_linear_velocity_y{0.0};
  double max_angular_velocity{0.5};
  double acceleration_limit{0.5};
  double deceleration_limit{0.5};
  double yaw_tolerance{0.1};
  double yaw_adjustment_gain{1.0};
  double final_yaw_min_angular_velocity{0.28};
  double local_avoidance_align_enter_angle{0.524};
  double local_avoidance_align_exit_angle{0.105};
  double local_avoidance_min_linear_velocity{0.4};
};

// 有状态的速度控制层：只把前瞻目标转换成Twist，不负责选择路径或触发绕障。
// aligning/correcting等滞回状态集中在这里，避免泄漏到主编排器。
class PurePursuitController {
public:
  bool initialize(rclcpp::Node::SharedPtr node,
                  const PurePursuitControllerConfig &config);

  // 新全局路径或退出局部绕障时，清除控制器的对齐/修正锁存。
  void resetTrackingState();
  // 切入局部绕障路径时，保持原逻辑：先原地对齐局部路径方向。
  void startLocalAvoidanceAlignment();
  bool isAligning() const { return aligning_; }

  bool computeCommand(const geometry_msgs::msg::PoseStamped &robot_pose,
                      const geometry_msgs::msg::PointStamped &lookahead_point,
                      double target_velocity, double angular_velocity_limit,
                      int motion_direction,
                      bool local_replanning_active,
                      std::size_t path_progress_index, int current_path_index,
                      geometry_msgs::msg::Twist &cmd_vel);

  // 根据上一控制周期的速度执行限速和加减速约束。
  void applyVelocityLimits(const geometry_msgs::msg::Twist &last_cmd_vel,
                           double dt, double linear_velocity_limit,
                           double angular_velocity_limit,
                           geometry_msgs::msg::Twist &cmd_vel) const;

  bool
  isTargetOrientationReached(const geometry_msgs::msg::PoseStamped &robot_pose,
                             const geometry_msgs::msg::Pose &target_pose) const;
  bool computeOrientationAdjustmentCommand(
      const geometry_msgs::msg::PoseStamped &robot_pose,
      const geometry_msgs::msg::Pose &target_pose,
      double angular_velocity_limit,
      geometry_msgs::msg::Twist &cmd_vel) const;

private:
  rclcpp::Node::SharedPtr node_;
  PurePursuitControllerConfig config_;
  bool aligning_{false};   // 大角度误差时的原地旋转锁存
  bool correcting_{false}; // 中等角度误差时的行走修正锁存
  bool local_avoidance_aligning_{false}; // 切入局部路径后的首次对齐锁存
};

} // namespace pure_pursuit_local_planner

#endif // PURE_PURSUIT_CONTROLLER_HPP
