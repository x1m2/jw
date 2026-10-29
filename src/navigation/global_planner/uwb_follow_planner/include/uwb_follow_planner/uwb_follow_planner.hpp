#pragma once

#include <atomic>
#include <deque>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <navigation_interfaces/msg/avoidance_state.hpp>
#include <navigation_interfaces/msg/path_with_policy.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "uwb_aoa_pkg/msg/lib_aoa_robot_msg.hpp"

namespace uwb_follow_planner {

class UwbFollowPlanner : public rclcpp::Node {
 public:
  explicit UwbFollowPlanner(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

 private:
  struct TrailPoint {
    geometry_msgs::msg::Point point;
    rclcpp::Time stamp;
  };

  enum class FollowState {
    kWaitingForPose,
    kWaitingForUwb,
    kHold,
    kFollowing,
    kTargetLost,
    kCommandStopped,
    kCommandPaused,
  };

  enum class FollowCommand : int8_t {
    kStart = 0,
    kPause = 1,
    kStop = 2,
  };

  void odometryCallback(nav_msgs::msg::Odometry::ConstSharedPtr message);
  void uwbCallback(
      uwb_aoa_pkg::msg::LibAoaRobotMsg::ConstSharedPtr message);
  void followCommandCallback(std_msgs::msg::Int8::ConstSharedPtr message);
  void avoidanceStateCallback(
      navigation_interfaces::msg::AvoidanceState::ConstSharedPtr message);
  void planningTimerCallback();

  bool uwbMeasurementToGlobal(
      const uwb_aoa_pkg::msg::LibAoaRobotMsg &message,
      geometry_msgs::msg::Point &person_global, double &range_m) const;
  bool acceptMeasurement(const geometry_msgs::msg::Point &person_map,
                         const rclcpp::Time &stamp) const;
  void updateTrail(const geometry_msgs::msg::Point &person_map,
                   const rclcpp::Time &stamp);
  void pruneTrail(const rclcpp::Time &now);
  void publishPersonPose(const rclcpp::Time &stamp);
  void publishTrail(const rclcpp::Time &stamp);
  bool buildFollowPath(const rclcpp::Time &stamp,
                       navigation_interfaces::msg::PathWithPolicy &output,
                       bool &target_updated);
  std::vector<geometry_msgs::msg::Point> resamplePath(
      const std::vector<geometry_msgs::msg::Point> &input) const;

  void setState(FollowState state);
  static const char *stateName(FollowState state);
  void requestPurePursuitPause(bool paused);

  static double distance2d(const geometry_msgs::msg::Point &first,
                           const geometry_msgs::msg::Point &second);

  // Topics and frames.
  std::string uwb_topic_;
  std::string follow_command_topic_;
  std::string odometry_topic_;
  std::string path_topic_;
  std::string global_frame_;
  bool enable_follow_command_topic_;

  // UWB interpretation and sensor mounting.
  double range_scale_;
  double bearing_offset_degrees_;
  double sensor_offset_x_;
  double sensor_offset_y_;
  double minimum_range_;
  double maximum_range_;

  // Filtering and trail generation.
  double filter_alpha_;
  double maximum_person_speed_;
  double jump_margin_;
  double trail_sample_distance_;
  double trail_keep_seconds_;
  int maximum_trail_points_;
  double short_loop_merge_radius_;
  double short_loop_max_age_;
  double path_point_spacing_;

  // Follow behavior.
  double follow_distance_;
  double start_follow_distance_;
  double stop_follow_distance_;
  double hold_radius_;
  double start_move_distance_;
  double movement_confirm_time_;
  double target_angle_update_threshold_degrees_;
  double target_position_update_threshold_;
  double uwb_timeout_;
  double pose_timeout_;
  double planning_frequency_;
  // UWB短时丢失时，继续执行最近一次已经发布的跟随目标；到达该目标后
  // 由Pure Pursuit的目标到达逻辑停车。BLE暂停/停止和定位超时不受影响。
  bool continue_to_last_goal_on_uwb_timeout_;
  bool manage_pure_pursuit_pause_;
  bool obstacle_stop_enabled_;
  bool obstacle_avoidance_enabled_;
  double obstacle_avoidance_distance_m_;

  rclcpp::Subscription<uwb_aoa_pkg::msg::LibAoaRobotMsg>::SharedPtr
      uwb_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr
      follow_command_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
      odometry_subscriber_;
  rclcpp::Subscription<navigation_interfaces::msg::AvoidanceState>::SharedPtr
      avoidance_state_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      person_pose_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trail_publisher_;
  rclcpp::Publisher<navigation_interfaces::msg::PathWithPolicy>::SharedPtr
      path_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_publisher_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr pause_client_;
  rclcpp::TimerBase::SharedPtr planning_timer_;

  geometry_msgs::msg::PoseStamped robot_pose_;
  geometry_msgs::msg::Point person_map_;
  geometry_msgs::msg::Point hold_center_;
  geometry_msgs::msg::Point follow_goal_;
  std::deque<TrailPoint> trail_;
  rclcpp::Time last_pose_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_uwb_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time movement_candidate_since_{0, 0, RCL_ROS_TIME};
  bool has_pose_{false};
  bool has_uwb_{false};
  bool has_filtered_person_{false};
  bool has_hold_center_{false};
  bool has_follow_goal_{false};
  bool movement_candidate_active_{false};
  std::atomic<uint8_t> avoidance_state_{
      navigation_interfaces::msg::AvoidanceState::IDLE};
  std::atomic<bool> fresh_dynamic_path_requested_{false};
  // BLE仅在操作时发布一次，因此命令必须锁存；节点启动默认不允许跟随。
  std::atomic<int8_t> follow_command_{
      static_cast<int8_t>(FollowCommand::kStop)};
  FollowState state_{FollowState::kWaitingForPose};
  std::optional<bool> last_pause_request_;
};

}  // namespace uwb_follow_planner
