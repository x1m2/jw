#ifndef AVOIDANCE_TYPES_HPP
#define AVOIDANCE_TYPES_HPP

#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/path.hpp>

#include <optional>

namespace pure_pursuit_local_planner {

// 障碍物检测层只给出事实，不负责启动、停止或锁存绕障任务。
enum class ObstacleDecision { CLEAR, AVOIDABLE, BLOCKED };

struct AvoidanceObservation {
  ObstacleDecision decision{ObstacleDecision::CLEAR};
  geometry_msgs::msg::Point obstacle_center;
  double obstacle_distance{0.0};
  // 障碍中心在base_link中的横向坐标：左正、右负。Manager需要该值
  // 在左右通道检测都失败时仍能优先向障碍相反侧脱困。
  double obstacle_local_y{0.0};
  bool left_clear{false};
  bool right_clear{false};
};

// 相对于全局路径前进方向的绕行侧。
enum class AvoidanceSide { NONE, LEFT, RIGHT };

// 静态路径分两阶段跟踪；动态路径到达临时点后等待新的参考路径。
enum class AvoidanceStage {
  IDLE,
  FIRST_SIDE_GOAL,
  RETURN_PATH_PLANNING,
  RETURN_GOAL,
  WAITING_DYNAMIC_PATH,
  COMPLETED,
  FAILED
};

// Manager对控制层暴露的单周期结果。控制层不再保存一套绕障状态。
enum class AvoidanceTrackingMode {
  GLOBAL_PATH,
  WAITING_COSTMAP,
  WAITING_LOCAL_PATH,
  LOCAL_PATH,
  STOPPED_BLOCKED,
  WAITING_DYNAMIC_PATH,
  FAILED
};

struct LocalAvoidanceOutput {
  AvoidanceTrackingMode mode{AvoidanceTrackingMode::GLOBAL_PATH};
  std::optional<nav_msgs::msg::Path> local_path;
  bool tracking_local{false};
  bool entered_local_tracking{false};
  bool realign_local_path{false};
  bool exited_local_tracking{false};
  bool failure_changed{false};
  bool failure_latched{false};
};

} // namespace pure_pursuit_local_planner

#endif // AVOIDANCE_TYPES_HPP
