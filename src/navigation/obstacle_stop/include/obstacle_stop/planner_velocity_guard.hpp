#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstddef>
#include <geometry_msgs/msg/twist.hpp>
#include <navigation_interfaces/msg/planner_motion_mode.hpp>

namespace obstacle_stop {

struct PlannerVelocityGuardConfig {
  double velocity{0.05};
  double check_distance{0.50};
};

struct MotionPermission {
  bool allowed{false};
  double velocity_limit{0.0};
  const char *reason{"motion unavailable"};
};

struct PlannerVelocityDecision {
  bool allowed{false};
  double velocity{0.0};
  const char *reason{"invalid planner motion mode"};
};

inline std::size_t motionDirection(std::uint8_t mode) {
  using Mode = navigation_interfaces::msg::PlannerMotionMode;
  return mode == Mode::LATERAL_LEFT ? 2U : mode == Mode::LATERAL_RIGHT ? 3U : 1U;
}

// Caller has already checked all components are finite and destination is safe.
// This helper only validates pure motion/direction and limits the requested speed.
inline PlannerVelocityDecision evaluatePlannerVelocityDecision(std::uint8_t mode,
    const geometry_msgs::msg::Twist &command, double velocity_limit) {
  using Mode = navigation_interfaces::msg::PlannerMotionMode;
  if (std::abs(command.linear.z) > 1e-6 || std::abs(command.angular.x) > 1e-6 ||
      std::abs(command.angular.y) > 1e-6)
    return {false, 0.0, "planner motion requires a pure planar command"};
  if (std::abs(command.angular.z) > 1e-6)
    return {false, 0.0, "planner motion requires zero angular velocity"};
  const bool backward = mode == Mode::BACKWARD;
  if (mode != Mode::LATERAL_LEFT && mode != Mode::LATERAL_RIGHT && !backward) return {};
  if (std::abs(backward ? command.linear.y : command.linear.x) > 1e-6)
    return {false, 0.0, "planner motion requires a pure direction command"};
  const double velocity = backward ? command.linear.x : command.linear.y;
  const bool positive = mode == Mode::LATERAL_RIGHT; // Chassis +y is right.
  if ((positive && velocity < -1e-6) || (!positive && velocity > 1e-6))
    return {false, 0.0, "planned velocity disagrees with planner motion mode"};
  // Clamp tiny wrong-sign values to zero as well.
  return {true, positive ? std::clamp(velocity, 0.0, velocity_limit) :
      std::clamp(velocity, -velocity_limit, 0.0), "planner velocity passed"};
}

}  // namespace obstacle_stop
