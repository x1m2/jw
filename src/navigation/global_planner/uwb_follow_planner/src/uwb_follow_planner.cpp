#include "uwb_follow_planner/uwb_follow_planner.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <stdexcept>

namespace uwb_follow_planner {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kPointEpsilon = 1e-6;
}

UwbFollowPlanner::UwbFollowPlanner(const rclcpp::NodeOptions &options)
    : rclcpp::Node("uwb_follow_planner_node", options) {
  uwb_topic_ = declare_parameter<std::string>(
      "uwb_topic", "/libAoa_robot_publisher");
  odometry_topic_ =
      declare_parameter<std::string>("odometry_topic", "/Odometry");
  path_topic_ = declare_parameter<std::string>(
      "path_topic", "/global_path_with_policy");
  global_frame_ = declare_parameter<std::string>("global_frame", "odom");

  range_scale_ = declare_parameter<double>("range_scale", 1.0);
  bearing_offset_degrees_ =
      declare_parameter<double>("bearing_offset_degrees", 0.0);
  sensor_offset_x_ = declare_parameter<double>("sensor_offset_x", 0.0);
  sensor_offset_y_ = declare_parameter<double>("sensor_offset_y", 0.0);
  minimum_range_ = declare_parameter<double>("minimum_range", 0.2);
  maximum_range_ = declare_parameter<double>("maximum_range", 30.0);

  filter_alpha_ = declare_parameter<double>("filter_alpha", 0.35);
  maximum_person_speed_ =
      declare_parameter<double>("maximum_person_speed", 4.0);
  jump_margin_ = declare_parameter<double>("jump_margin", 0.8);
  trail_sample_distance_ =
      declare_parameter<double>("trail_sample_distance", 0.25);
  trail_keep_seconds_ =
      declare_parameter<double>("trail_keep_seconds", 45.0);
  maximum_trail_points_ =
      declare_parameter<int>("maximum_trail_points", 400);
  short_loop_merge_radius_ =
      declare_parameter<double>("short_loop_merge_radius", 0.4);
  short_loop_max_age_ =
      declare_parameter<double>("short_loop_max_age", 4.0);
  path_point_spacing_ =
      declare_parameter<double>("path_point_spacing", 0.2);

  follow_distance_ = declare_parameter<double>("follow_distance", 1.5);
  start_follow_distance_ =
      declare_parameter<double>("start_follow_distance", 1.9);
  stop_follow_distance_ =
      declare_parameter<double>("stop_follow_distance", 1.6);
  hold_radius_ = declare_parameter<double>("hold_radius", 0.35);
  start_move_distance_ =
      declare_parameter<double>("start_move_distance", 0.7);
  movement_confirm_time_ =
      declare_parameter<double>("movement_confirm_time", 0.6);
  target_angle_update_threshold_degrees_ =
      declare_parameter<double>("target_angle_update_threshold_degrees", 10.0);
  target_position_update_threshold_ =
      declare_parameter<double>("target_position_update_threshold", 0.4);
  uwb_timeout_ = declare_parameter<double>("uwb_timeout", 0.5);
  pose_timeout_ = declare_parameter<double>("pose_timeout", 0.5);
  planning_frequency_ =
      declare_parameter<double>("planning_frequency", 3.0);
  manage_pure_pursuit_pause_ =
      declare_parameter<bool>("manage_pure_pursuit_pause", true);
  obstacle_stop_enabled_ =
      declare_parameter<bool>("obstacle_stop_enabled", true);
  obstacle_avoidance_enabled_ =
      declare_parameter<bool>("obstacle_avoidance_enabled", true);

  filter_alpha_ = std::clamp(filter_alpha_, 0.0, 1.0);
  maximum_trail_points_ = std::max(maximum_trail_points_, 2);
  planning_frequency_ = std::max(planning_frequency_, 0.1);
  path_point_spacing_ = std::max(path_point_spacing_, 0.02);

  if (stop_follow_distance_ >= start_follow_distance_) {
    throw std::invalid_argument(
        "stop_follow_distance must be smaller than start_follow_distance");
  }
  if (follow_distance_ <= 0.0 || minimum_range_ < 0.0 ||
      maximum_range_ <= minimum_range_ || uwb_timeout_ <= 0.0 ||
      pose_timeout_ <= 0.0 || !std::isfinite(range_scale_) ||
      range_scale_ <= 0.0 || !std::isfinite(bearing_offset_degrees_) ||
      !std::isfinite(target_angle_update_threshold_degrees_) ||
      target_angle_update_threshold_degrees_ <= 0.0 ||
      target_angle_update_threshold_degrees_ > 180.0 ||
      !std::isfinite(target_position_update_threshold_) ||
      target_position_update_threshold_ <= 0.0) {
    throw std::invalid_argument("invalid UWB follow planner parameters");
  }

  person_pose_publisher_ =
      create_publisher<geometry_msgs::msg::PoseStamped>(
          "/uwb_follow/person_pose", rclcpp::QoS(10));
  trail_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "/uwb_follow/person_trail", rclcpp::QoS(1).transient_local());
  path_publisher_ =
      create_publisher<navigation_interfaces::msg::PathWithPolicy>(
          path_topic_, rclcpp::QoS(1).transient_local().reliable());
  state_publisher_ = create_publisher<std_msgs::msg::String>(
      "/uwb_follow/state", rclcpp::QoS(1).transient_local());
  valid_publisher_ = create_publisher<std_msgs::msg::Bool>(
      "/uwb_follow/target_valid", rclcpp::QoS(1).transient_local());

  odometry_subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic_, rclcpp::QoS(20),
      std::bind(&UwbFollowPlanner::odometryCallback, this,
                std::placeholders::_1));
  uwb_subscriber_ =
      create_subscription<uwb_aoa_pkg::msg::LibAoaRobotMsg>(
          uwb_topic_, rclcpp::QoS(20),
          std::bind(&UwbFollowPlanner::uwbCallback, this,
                    std::placeholders::_1));
  avoidance_state_subscriber_ =
      create_subscription<navigation_interfaces::msg::AvoidanceState>(
          "/local_avoidance/state",
          rclcpp::QoS(1).reliable().transient_local(),
          std::bind(&UwbFollowPlanner::avoidanceStateCallback, this,
                    std::placeholders::_1));

  pause_client_ = create_client<std_srvs::srv::SetBool>(
      "/pure_pursuit/set_paused");

  const auto timer_period = std::chrono::duration<double>(
      1.0 / planning_frequency_);
  planning_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
      std::bind(&UwbFollowPlanner::planningTimerCallback, this));

  setState(FollowState::kWaitingForPose);
  RCLCPP_INFO(get_logger(),
              "UWB follow planner ready: input=%s, odometry=%s, frame=%s, "
              "output=%s",
              uwb_topic_.c_str(), odometry_topic_.c_str(),
              global_frame_.c_str(), path_topic_.c_str());
}

void UwbFollowPlanner::avoidanceStateCallback(
    navigation_interfaces::msg::AvoidanceState::ConstSharedPtr message) {
  if (message->state > navigation_interfaces::msg::AvoidanceState::FAILED) {
    RCLCPP_WARN(get_logger(), "Ignoring unknown local avoidance state: %u",
                message->state);
    return;
  }
  avoidance_state_.store(message->state);
}

void UwbFollowPlanner::odometryCallback(
    nav_msgs::msg::Odometry::ConstSharedPtr message) {
  if (message->header.frame_id != global_frame_) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring FAST-LIO odometry in frame '%s'; expected '%s'",
        message->header.frame_id.c_str(), global_frame_.c_str());
    return;
  }
  const auto &position = message->pose.pose.position;
  const auto &orientation = message->pose.pose.orientation;
  if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
      !std::isfinite(orientation.x) || !std::isfinite(orientation.y) ||
      !std::isfinite(orientation.z) || !std::isfinite(orientation.w)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Ignoring non-finite FAST-LIO odometry");
    return;
  }
  robot_pose_.header = message->header;
  robot_pose_.pose = message->pose.pose;
  last_pose_time_ = now();
  has_pose_ = true;
}

void UwbFollowPlanner::uwbCallback(
    uwb_aoa_pkg::msg::LibAoaRobotMsg::ConstSharedPtr message) {
  RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "UWB callback: r=%.3f, a=%.3f deg, pose=%s",
      message->r, message->a, has_pose_ ? "ready" : "waiting");

  if (!has_pose_) {
    return;
  }

  geometry_msgs::msg::Point measurement;
  double range_m = 0.0;
  if (!uwbMeasurementToGlobal(*message, measurement, range_m)) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring invalid UWB measurement: r=%.3f, a=%.3f deg",
        message->r, message->a);
    return;
  }

  const rclcpp::Time stamp = now();
  if (!acceptMeasurement(measurement, stamp)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Rejected implausible UWB person-position jump");
    return;
  }

  if (!has_filtered_person_) {
    person_map_ = measurement;
    has_filtered_person_ = true;
  } else {
    person_map_.x += filter_alpha_ * (measurement.x - person_map_.x);
    person_map_.y += filter_alpha_ * (measurement.y - person_map_.y);
    person_map_.z = 0.0;
  }

  last_uwb_time_ = stamp;
  has_uwb_ = true;
  if (!has_hold_center_) {
    hold_center_ = person_map_;
    has_hold_center_ = true;
  }

  updateTrail(person_map_, stamp);
  publishPersonPose(stamp);
  publishTrail(stamp);

  const double robot_yaw_degrees =
      tf2::getYaw(robot_pose_.pose.orientation) * 180.0 / kPi;
  const double person_bearing_degrees =
      std::atan2(person_map_.y - robot_pose_.pose.position.y,
                 person_map_.x - robot_pose_.pose.position.x) *
      180.0 / kPi;
  RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "UWB accepted: range=%.3f m, target=(%.3f, %.3f), "
      "filtered=(%.3f, %.3f), robot_yaw=%.1f deg, "
      "person_bearing=%.1f deg",
      range_m, measurement.x, measurement.y, person_map_.x, person_map_.y,
      robot_yaw_degrees, person_bearing_degrees);
}

bool UwbFollowPlanner::uwbMeasurementToGlobal(
    const uwb_aoa_pkg::msg::LibAoaRobotMsg &message,
    geometry_msgs::msg::Point &person_global, double &range_m) const {
  if (!std::isfinite(message.r) || !std::isfinite(message.a)) {
    return false;
  }

  range_m = message.r * range_scale_;
  if (!std::isfinite(range_m) || range_m < minimum_range_ ||
      range_m > maximum_range_) {
    return false;
  }

  // UWB convention matches REP-103 base_link axes: +x is forward and +y is
  // left. Therefore positive (counter-clockwise) angles produce positive y.
  const double normalized_angle_degrees = std::remainder(message.a, 360.0);
  const double bearing =
      (normalized_angle_degrees + bearing_offset_degrees_) * kPi / 180.0;
  const double base_x = sensor_offset_x_ + range_m * std::cos(bearing);
  const double base_y = sensor_offset_y_ + range_m * std::sin(bearing);

  const double robot_yaw = tf2::getYaw(robot_pose_.pose.orientation);
  const double yaw_cos = std::cos(robot_yaw);
  const double yaw_sin = std::sin(robot_yaw);
  person_global.x = robot_pose_.pose.position.x + yaw_cos * base_x -
                    yaw_sin * base_y;
  person_global.y = robot_pose_.pose.position.y + yaw_sin * base_x +
                    yaw_cos * base_y;
  person_global.z = 0.0;
  return true;
}

bool UwbFollowPlanner::acceptMeasurement(
    const geometry_msgs::msg::Point &person_map,
    const rclcpp::Time &stamp) const {
  if (!has_filtered_person_ || last_uwb_time_.nanoseconds() == 0) {
    return true;
  }
  const double elapsed = std::max((stamp - last_uwb_time_).seconds(), 0.0);
  const double maximum_jump = jump_margin_ + maximum_person_speed_ * elapsed;
  return distance2d(person_map_, person_map) <= maximum_jump;
}

void UwbFollowPlanner::updateTrail(
    const geometry_msgs::msg::Point &person_map, const rclcpp::Time &stamp) {
  if (trail_.empty()) {
    trail_.push_back({person_map, stamp});
    return;
  }

  if (distance2d(trail_.back().point, person_map) < trail_sample_distance_) {
    return;
  }

  // Collapse only recent, short loops caused by pacing or UWB noise. Long loops
  // are preserved because they may represent an intentional return route.
  if (trail_.size() > 2U) {
    for (std::size_t index = trail_.size() - 2U; index > 0U; --index) {
      const double age = (stamp - trail_[index].stamp).seconds();
      if (age > short_loop_max_age_) {
        break;
      }
      if (distance2d(trail_[index].point, person_map) <=
          short_loop_merge_radius_) {
        trail_.erase(trail_.begin() + static_cast<std::ptrdiff_t>(index + 1U),
                     trail_.end());
        trail_.push_back({person_map, stamp});
        pruneTrail(stamp);
        return;
      }
    }
  }

  trail_.push_back({person_map, stamp});
  pruneTrail(stamp);
}

void UwbFollowPlanner::pruneTrail(const rclcpp::Time &now_time) {
  while (trail_.size() > 2U &&
         ((now_time - trail_.front().stamp).seconds() > trail_keep_seconds_ ||
          static_cast<int>(trail_.size()) > maximum_trail_points_)) {
    trail_.pop_front();
  }
}

void UwbFollowPlanner::publishPersonPose(const rclcpp::Time &stamp) {
  geometry_msgs::msg::PoseStamped message;
  message.header.stamp = stamp;
  message.header.frame_id = global_frame_;
  message.pose.position = person_map_;
  message.pose.orientation.w = 1.0;
  person_pose_publisher_->publish(message);
}

void UwbFollowPlanner::publishTrail(const rclcpp::Time &stamp) {
  nav_msgs::msg::Path path;
  path.header.stamp = stamp;
  path.header.frame_id = global_frame_;
  path.poses.reserve(trail_.size());
  for (const auto &sample : trail_) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position = sample.point;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  trail_publisher_->publish(path);
}

void UwbFollowPlanner::planningTimerCallback() {
  const rclcpp::Time current_time = now();
  std_msgs::msg::Bool valid_message;

  const uint8_t avoidance_state = avoidance_state_.load();
  const bool local_avoidance_owns_motion =
      avoidance_state ==
          navigation_interfaces::msg::AvoidanceState::AVOIDING ||
      avoidance_state == navigation_interfaces::msg::AvoidanceState::BLOCKED ||
      avoidance_state == navigation_interfaces::msg::AvoidanceState::FAILED;
  if (local_avoidance_owns_motion) {
    // 绕障期间继续更新人员位置和历史轨迹，但不再发布新的控制参考路径。
    // Pure Pursuit控制循环必须保持运行，由局部规划器负责绕行或停车。
    valid_message.data =
        has_uwb_ &&
        (current_time - last_uwb_time_).seconds() <= uwb_timeout_;
    valid_publisher_->publish(valid_message);
    // The person may move significantly while avoidance owns the controller.
    // Discard the old target so the first post-avoidance path uses the latest
    // person position instead of resuming toward a stale goal.
    has_follow_goal_ = false;
    requestPurePursuitPause(false);
    return;
  }

  if (!has_pose_ ||
      (current_time - last_pose_time_).seconds() > pose_timeout_) {
    has_follow_goal_ = false;
    setState(FollowState::kWaitingForPose);
    valid_message.data = false;
    valid_publisher_->publish(valid_message);
    requestPurePursuitPause(true);
    return;
  }

  if (!has_uwb_) {
    has_follow_goal_ = false;
    setState(FollowState::kWaitingForUwb);
    valid_message.data = false;
    valid_publisher_->publish(valid_message);
    requestPurePursuitPause(true);
    return;
  }

  if ((current_time - last_uwb_time_).seconds() > uwb_timeout_) {
    has_follow_goal_ = false;
    setState(FollowState::kTargetLost);
    valid_message.data = false;
    valid_publisher_->publish(valid_message);
    requestPurePursuitPause(true);
    movement_candidate_active_ = false;
    return;
  }

  valid_message.data = true;
  valid_publisher_->publish(valid_message);

  const double map_range = distance2d(robot_pose_.pose.position, person_map_);
  const double displacement =
      has_hold_center_ ? distance2d(hold_center_, person_map_) : 0.0;

  const bool waiting_for_dynamic_path =
      avoidance_state ==
      navigation_interfaces::msg::AvoidanceState::WAITING_DYNAMIC_PATH;
  if (waiting_for_dynamic_path) {
    // 已经位于障碍后侧，不再等待人员移动确认，立即尝试生成最新参考路径。
    movement_candidate_active_ = false;
    setState(FollowState::kFollowing);
  } else if (state_ != FollowState::kFollowing) {
    const bool movement_requires_follow =
        map_range >= start_follow_distance_ ||
        displacement >= start_move_distance_;
    if (!movement_requires_follow) {
      has_follow_goal_ = false;
      movement_candidate_active_ = false;
      if (displacement <= hold_radius_) {
        hold_center_.x = 0.9 * hold_center_.x + 0.1 * person_map_.x;
        hold_center_.y = 0.9 * hold_center_.y + 0.1 * person_map_.y;
      }
      setState(FollowState::kHold);
      requestPurePursuitPause(true);
      return;
    }

    if (!movement_candidate_active_) {
      movement_candidate_since_ = current_time;
      movement_candidate_active_ = true;
      setState(FollowState::kHold);
      requestPurePursuitPause(true);
      return;
    }
    if ((current_time - movement_candidate_since_).seconds() <
        movement_confirm_time_) {
      return;
    }
    movement_candidate_active_ = false;
    setState(FollowState::kFollowing);
  }

  if (map_range <= stop_follow_distance_) {
    has_follow_goal_ = false;
    hold_center_ = person_map_;
    has_hold_center_ = true;
    setState(FollowState::kHold);
    requestPurePursuitPause(true);
    return;
  }

  navigation_interfaces::msg::PathWithPolicy path;
  bool target_updated = false;
  if (!buildFollowPath(current_time, path, target_updated)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Unable to build a usable UWB follow path");
    requestPurePursuitPause(true);
    return;
  }
  // Keep tracking the current path while its locked goal is still valid.
  // Publishing only on a target change avoids resetting Pure Pursuit on every
  // planning tick.
  if (target_updated) {
    path_publisher_->publish(path);
  }
  requestPurePursuitPause(false);
}

bool UwbFollowPlanner::buildFollowPath(
    const rclcpp::Time &stamp,
    navigation_interfaces::msg::PathWithPolicy &output,
    bool &target_updated) {
  target_updated = false;
  std::vector<geometry_msgs::msg::Point> raw_path;
  const geometry_msgs::msg::Point dog = robot_pose_.pose.position;
  raw_path.push_back(dog);

  const double dog_to_person = distance2d(dog, person_map_);
  if (dog_to_person <= follow_distance_ + kPointEpsilon) {
    return false;
  }
  const double candidate_ratio =
      (dog_to_person - follow_distance_) / dog_to_person;
  geometry_msgs::msg::Point candidate_goal;
  candidate_goal.x = dog.x + candidate_ratio * (person_map_.x - dog.x);
  candidate_goal.y = dog.y + candidate_ratio * (person_map_.y - dog.y);
  candidate_goal.z = 0.0;

  double angle_error_degrees = 0.0;
  double goal_shift = 0.0;
  const char *update_reason = "initial";
  bool update_goal = !has_follow_goal_;
  const geometry_msgs::msg::Point previous_goal = follow_goal_;
  const bool had_previous_goal = has_follow_goal_;
  if (has_follow_goal_) {
    const double old_goal_distance = distance2d(dog, follow_goal_);
    if (old_goal_distance <= kPointEpsilon) {
      angle_error_degrees = 180.0;
    } else {
      const double person_angle =
          std::atan2(person_map_.y - dog.y, person_map_.x - dog.x);
      const double old_goal_angle =
          std::atan2(follow_goal_.y - dog.y, follow_goal_.x - dog.x);
      angle_error_degrees =
          std::abs(std::remainder(person_angle - old_goal_angle, 2.0 * kPi)) *
          180.0 / kPi;
    }
    goal_shift = distance2d(candidate_goal, follow_goal_);
    const bool angle_changed =
        angle_error_degrees > target_angle_update_threshold_degrees_;
    const bool position_changed =
        goal_shift > target_position_update_threshold_;
    update_goal = angle_changed || position_changed;
    if (angle_changed && position_changed) {
      update_reason = "angle+position";
    } else if (angle_changed) {
      update_reason = "angle";
    } else if (position_changed) {
      update_reason = "position";
    } else {
      update_reason = "keep";
    }
  }

  if (update_goal) {
    follow_goal_ = candidate_goal;
    has_follow_goal_ = true;
    target_updated = true;
    if (had_previous_goal) {
      RCLCPP_INFO(
          get_logger(),
          "UWB follow target updated [%s]: angle_error=%.1f deg, "
          "goal_shift=%.3f m, old=(%.3f, %.3f), new=(%.3f, %.3f)",
          update_reason, angle_error_degrees, goal_shift, previous_goal.x,
          previous_goal.y, follow_goal_.x, follow_goal_.y);
    } else {
      RCLCPP_INFO(get_logger(),
                  "UWB follow target initialized: goal=(%.3f, %.3f)",
                  follow_goal_.x, follow_goal_.y);
    }
  } else {
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "UWB follow target kept: angle_error=%.1f/%.1f deg, "
        "goal_shift=%.3f/%.3f m, goal=(%.3f, %.3f)",
        angle_error_degrees, target_angle_update_threshold_degrees_, goal_shift,
        target_position_update_threshold_, follow_goal_.x, follow_goal_.y);
  }
  raw_path.push_back(follow_goal_);

  auto sampled_path = resamplePath(raw_path);
  if (sampled_path.size() < 2U) {
    return false;
  }

  output.path.header.stamp = stamp;
  output.path.header.frame_id = global_frame_;
  output.path.poses.reserve(sampled_path.size());
  for (std::size_t index = 0U; index < sampled_path.size(); ++index) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = output.path.header;
    pose.pose.position = sampled_path[index];
    double yaw = 0.0;
    if (index + 1U < sampled_path.size()) {
      yaw = std::atan2(sampled_path[index + 1U].y - sampled_path[index].y,
                       sampled_path[index + 1U].x - sampled_path[index].x);
    } else if (!output.path.poses.empty()) {
      pose.pose.orientation = output.path.poses.back().pose.orientation;
      output.path.poses.push_back(pose);
      continue;
    }
    pose.pose.orientation.z = std::sin(yaw * 0.5);
    pose.pose.orientation.w = std::cos(yaw * 0.5);
    output.path.poses.push_back(pose);
  }

  output.obstacle_stop_enabled.assign(output.path.poses.size(),
                                      obstacle_stop_enabled_);
  output.obstacle_avoidance_enabled.assign(output.path.poses.size(),
                                           obstacle_avoidance_enabled_);
  output.dynamic_path = true;

  const auto &path_goal = output.path.poses.back().pose.position;
  const double robot_yaw_degrees =
      tf2::getYaw(robot_pose_.pose.orientation) * 180.0 / kPi;
  const double person_bearing_degrees =
      std::atan2(person_map_.y - dog.y, person_map_.x - dog.x) * 180.0 / kPi;
  const double goal_bearing_degrees =
      std::atan2(path_goal.y - dog.y, path_goal.x - dog.x) * 180.0 / kPi;
  const double goal_yaw_error_degrees =
      std::remainder((goal_bearing_degrees - robot_yaw_degrees) * kPi / 180.0,
                     2.0 * kPi) *
      180.0 / kPi;
  RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "UWB follow path goal [%s]: frame=%s, robot=(%.3f, %.3f), "
      "person=(%.3f, %.3f), goal=(%.3f, %.3f), points=%zu, "
      "robot_yaw=%.1f deg, person_bearing=%.1f deg, "
      "goal_bearing=%.1f deg, goal_yaw_error=%.1f deg",
      update_goal ? "updated" : "locked", global_frame_.c_str(), dog.x,
      dog.y, person_map_.x, person_map_.y, path_goal.x, path_goal.y,
      output.path.poses.size(), robot_yaw_degrees, person_bearing_degrees,
      goal_bearing_degrees, goal_yaw_error_degrees);
  return true;
}

std::vector<geometry_msgs::msg::Point> UwbFollowPlanner::resamplePath(
    const std::vector<geometry_msgs::msg::Point> &input) const {
  if (input.size() < 2U) {
    return input;
  }
  std::vector<geometry_msgs::msg::Point> output;
  output.push_back(input.front());
  for (std::size_t index = 0U; index + 1U < input.size(); ++index) {
    const auto &start = input[index];
    const auto &finish = input[index + 1U];
    const double length = distance2d(start, finish);
    if (length <= kPointEpsilon) {
      continue;
    }
    const int divisions =
        std::max(1, static_cast<int>(std::ceil(length / path_point_spacing_)));
    for (int step = 1; step <= divisions; ++step) {
      const double ratio = static_cast<double>(step) / divisions;
      geometry_msgs::msg::Point point;
      point.x = start.x + ratio * (finish.x - start.x);
      point.y = start.y + ratio * (finish.y - start.y);
      point.z = start.z + ratio * (finish.z - start.z);
      output.push_back(point);
    }
  }
  return output;
}

void UwbFollowPlanner::setState(FollowState state) {
  if (state_ == state && state_publisher_->get_subscription_count() == 0U) {
    return;
  }
  const bool changed = state_ != state;
  state_ = state;
  std_msgs::msg::String message;
  message.data = stateName(state_);
  state_publisher_->publish(message);
  if (changed) {
    RCLCPP_INFO(get_logger(), "UWB follow state: %s", message.data.c_str());
  }
}

const char *UwbFollowPlanner::stateName(FollowState state) {
  switch (state) {
    case FollowState::kWaitingForPose:
      return "WAITING_FOR_POSE";
    case FollowState::kWaitingForUwb:
      return "WAITING_FOR_UWB";
    case FollowState::kHold:
      return "HOLD";
    case FollowState::kFollowing:
      return "FOLLOWING";
    case FollowState::kTargetLost:
      return "TARGET_LOST";
  }
  return "UNKNOWN";
}

void UwbFollowPlanner::requestPurePursuitPause(bool paused) {
  if (!manage_pure_pursuit_pause_ ||
      (last_pause_request_.has_value() &&
       last_pause_request_.value() == paused)) {
    return;
  }
  if (!pause_client_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Pure Pursuit pause service is not ready");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = paused;
  pause_client_->async_send_request(request);
  last_pause_request_ = paused;
}

double UwbFollowPlanner::distance2d(
    const geometry_msgs::msg::Point &first,
    const geometry_msgs::msg::Point &second) {
  return std::hypot(first.x - second.x, first.y - second.y);
}

}  // namespace uwb_follow_planner
