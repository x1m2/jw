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
  bool released_center{false};
  bool released_right{false};
};

struct DirectionalEscapeMotion {
  double forward_velocity{0.0};
  double lateral_velocity{0.0};
  double angular_velocity{0.0};
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

// 未完成，未测试。
// 全向脱困专用策略。中央近障碍只允许“零前进、低角速度”的纯横移，
// 防止把普通前进或原地旋转误放行。中央同时延伸到单侧时，横移方向还
// 必须远离该侧；左右同时占用仍无条件停车。
inline DirectionalObstacleDecision evaluateDirectionalEscape(
    const DirectionalZoneOccupancy &occupancy,
    const DirectionalEscapeMotion &motion, double minimum_escape_velocity,
    double maximum_forward_velocity, double maximum_angular_velocity,
    bool directional_release_enabled, bool center_lateral_release_enabled) {
  if (!occupancy.center) {
    return evaluateDirectionalObstacle(
        occupancy,
        motion.lateral_velocity + motion.angular_velocity,
        minimum_escape_velocity, directional_release_enabled);
  }

  DirectionalObstacleDecision decision;
  decision.active = true;
  if (!directional_release_enabled || !center_lateral_release_enabled ||
      (occupancy.left && occupancy.right)) {
    return decision;
  }

  const double threshold = std::max(0.0, minimum_escape_velocity);
  if (std::abs(motion.forward_velocity) >
          std::max(0.0, maximum_forward_velocity) ||
      std::abs(motion.angular_velocity) >
          std::max(0.0, maximum_angular_velocity) ||
      std::abs(motion.lateral_velocity) < threshold) {
    return decision;
  }

  if (occupancy.left && motion.lateral_velocity > -threshold) {
    return decision;
  }
  if (occupancy.right && motion.lateral_velocity < threshold) {
    return decision;
  }

  decision.active = false;
  decision.released_left = occupancy.left;
  decision.released_center = true;
  decision.released_right = occupancy.right;
  return decision;
}

}  // namespace obstacle_stop
