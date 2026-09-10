#ifndef AVOIDANCE_TYPES_HPP
#define AVOIDANCE_TYPES_HPP

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

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

// 窄通道第一阶段只提供几何检测结果和调试中心线，不参与控制决策。
// center_offset 使用当前全局路径的局部横向坐标：左正、右负。
struct NarrowPassageResult {
  bool detected{false};
  bool center_clear{false};
  double estimated_width{0.0};
  double center_offset{0.0};
  double confidence{0.0};
  double reference_heading{0.0};
  std::size_t valid_bins{0};
  std::size_t total_bins{0};
  std::vector<geometry_msgs::msg::Point> left_boundary;
  std::vector<geometry_msgs::msg::Point> right_boundary;
  nav_msgs::msg::Path center_path;
};

// 相对于全局路径前进方向的绕行侧。
enum class AvoidanceSide { NONE, LEFT, RIGHT };

// 沿原始全局路径搜索“障碍后绕障临时点”的结果；找到后会将该点锁定为
// 本轮局部A*的唯一目标。
enum class TemporaryGoalSearchStatus {
  FOUND,
  NO_BLOCKED_PATH,
  NO_SAFE_TEMPORARY_GOAL,
  INVALID_INPUT,
  NO_COSTMAP
};

inline const char *temporaryGoalSearchStatusToString(TemporaryGoalSearchStatus status) {
  switch (status) {
  case TemporaryGoalSearchStatus::FOUND:
    return "FOUND";
  case TemporaryGoalSearchStatus::NO_BLOCKED_PATH:
    return "NO_BLOCKED_PATH";
  case TemporaryGoalSearchStatus::NO_SAFE_TEMPORARY_GOAL:
    return "NO_SAFE_TEMPORARY_GOAL";
  case TemporaryGoalSearchStatus::INVALID_INPUT:
    return "INVALID_INPUT";
  case TemporaryGoalSearchStatus::NO_COSTMAP:
    return "NO_COSTMAP";
  }
  return "UNKNOWN";
}

struct TemporaryGoalSearchResult {
  TemporaryGoalSearchStatus status{TemporaryGoalSearchStatus::INVALID_INPUT};
  geometry_msgs::msg::PoseStamped goal;
  double goal_yaw{0.0};
  std::size_t search_start_index{std::numeric_limits<std::size_t>::max()};
  std::size_t search_start_segment_index{
      std::numeric_limits<std::size_t>::max()};
  std::size_t blocked_start_index{std::numeric_limits<std::size_t>::max()};
  std::size_t blocked_end_index{std::numeric_limits<std::size_t>::max()};
  // 临时点可位于路径线段内部；该索引表示临时点之后用于恢复全局跟踪的
  // 第一个原始路径点，而不是要求goal必须等于该索引处的路径点。
  std::size_t temporary_goal_index{std::numeric_limits<std::size_t>::max()};
  // 以下距离均沿全局路径弧长计算，而不是使用点间欧式直线距离。
  double scanned_path_length{0.0};
  double blocked_start_path_distance{0.0};
  double blocked_end_path_distance{0.0};
  double temporary_goal_path_distance{0.0};
  double distance_from_last_unsafe{0.0};
  double verified_safe_length{0.0};
};

enum class AvoidanceStage {
  IDLE,
  SEARCHING_TEMPORARY_GOAL,
  TRACKING_TEMPORARY_GOAL,
  WAITING_TEMPORARY_GOAL,
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
  // 绕障临时点完成时通知PP推进到插值临时点之后的首个原路径索引，避免
  // 重新追向障碍前方。
  std::optional<std::size_t> resume_global_path_index;
  std::optional<std::size_t> temporary_goal_index;
  std::optional<geometry_msgs::msg::PoseStamped> temporary_goal;
  AvoidanceStage stage{AvoidanceStage::IDLE};
  std::uint64_t task_generation{0};
};

} // namespace pure_pursuit_local_planner

#endif // AVOIDANCE_TYPES_HPP
