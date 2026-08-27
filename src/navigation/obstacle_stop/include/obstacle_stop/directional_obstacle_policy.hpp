#pragma once

#include <algorithm>
#include <cmath>

namespace obstacle_stop {

// 固定检测矩形按机器人横向划分为左、中央和右三个区域。
struct DirectionalZoneOccupancy {
  bool left{false};
  bool center{false};
  bool right{false};
};

struct DirectionalObstacleDecision {
  bool active{false};
  bool released_left{false};
  bool released_right{false};
};

// lateral_velocity是检测区域中心在base_link中的瞬时横向速度：
// vy + wz * x。正值向左，负值向右。
inline DirectionalObstacleDecision evaluateDirectionalObstacle(
    const DirectionalZoneOccupancy &occupancy, double lateral_velocity,
    double minimum_escape_velocity, bool directional_release_enabled) {
  DirectionalObstacleDecision decision;
  if (!occupancy.left && !occupancy.center && !occupancy.right) {
    return decision;
  }

  // 中央障碍或两侧夹障没有安全的单侧脱离方向。
  if (occupancy.center || (occupancy.left && occupancy.right)) {
    decision.active = true;
    return decision;
  }

  const double threshold = std::max(0.0, minimum_escape_velocity);
  if (directional_release_enabled && occupancy.left &&
      lateral_velocity <= -threshold) {
    decision.released_left = true;
    return decision;
  }
  if (directional_release_enabled && occupancy.right &&
      lateral_velocity >= threshold) {
    decision.released_right = true;
    return decision;
  }

  decision.active = true;
  return decision;
}

}  // namespace obstacle_stop
