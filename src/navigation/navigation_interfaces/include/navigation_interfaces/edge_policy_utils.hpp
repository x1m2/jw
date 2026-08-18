#pragma once

#include "navigation_interfaces/msg/edge_policy.hpp"

namespace navigation_interfaces {

inline msg::EdgePolicy makeDefaultEdgePolicy(
    bool velocity_limit_enabled = true) {
  msg::EdgePolicy policy;
  policy.velocity_limit_enabled = velocity_limit_enabled;
  policy.linear_velocity = 0.5;
  policy.angular_velocity = 0.5;
  policy.motion_direction = 1;
  policy.obstacle_stop_enabled = true;
  policy.obstacle_avoidance_enabled = false;
  policy.obstacle_avoidance_distance_m = 0.5;
  return policy;
}

}  // namespace navigation_interfaces
