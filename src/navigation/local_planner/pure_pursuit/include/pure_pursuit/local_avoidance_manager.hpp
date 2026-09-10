#ifndef LOCAL_AVOIDANCE_MANAGER_HPP
#define LOCAL_AVOIDANCE_MANAGER_HPP

#include <tf2/time.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <limits>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <thread>

#include "pure_pursuit/avoidance_path_planner.hpp"
#include "pure_pursuit/costmap_types.hpp"

namespace pure_pursuit_local_planner {

// 局部绕障运行管理层。
// 只管理绕障阶段、规划线程和当前局部路径；具体目标生成与A*计算下沉到
// AvoidancePathPlanner，速度控制由上层控制器负责。
class LocalAvoidanceManager {
public:
  using AvoidanceSide = ::pure_pursuit_local_planner::AvoidanceSide;
  using AvoidanceStage = ::pure_pursuit_local_planner::AvoidanceStage;

  LocalAvoidanceManager();
  ~LocalAvoidanceManager();

  // 初始化
  bool initialize(rclcpp::Node::SharedPtr node,
                  std::shared_ptr<tf2_ros::Buffer> tf_buffer);

  // 设置全局路径
  void setGlobalPath(const nav_msgs::msg::Path &global_path);

  // 设置当前参考路径是否为动态路径。动态路径绕过障碍后不回到旧路径，
  // 而是在障碍后侧停车等待新的参考路径。
  void setDynamicPathMode(bool enabled);

  // 更新代价地图
  void updateCostmap(CostmapConstPtr costmap);

  // 绕障状态机的唯一入口。检测结果、失败锁存、完成冷却和路径快照都在
  // Manager内部统一处理，上层只消费本周期输出。
  LocalAvoidanceOutput update(
      const geometry_msgs::msg::PoseStamped &robot_pose,
      const AvoidanceObservation &observation, bool avoidance_enabled,
      bool global_controller_aligning, bool safety_stop_blocking = false,
      std::size_t global_path_progress_index = 0);
  void reset();
  bool isTrackingLocal() const;

  // 停止规划器
  void stop();

private:
  // ROS相关
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  // 发布器
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      goal_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      side_goal_candidates_pub_;

  // 同步绕障路径计算层；状态和工作线程仍由本类管理。
  std::unique_ptr<AvoidancePathPlanner> avoidance_path_planner_;

  // 规划线程只周期性计算局部路径；ROS控制循环通过下面的互斥量读取快照。
  std::thread planning_thread_;
  std::atomic<bool> running_;
  // local_planning_frequency只作为周期复核频率。首次触发绕障、停障重规划
  // 等事件通过条件变量立即唤醒规划线程，避免0.5Hz带来最长2秒首规划延迟。
  std::condition_variable planning_condition_;
  std::mutex planning_wait_mutex_;
  bool planning_wakeup_requested_{false};
  mutable std::mutex path_mutex_;
  mutable std::mutex costmap_mutex_;
  mutable std::mutex global_path_mutex_;
  mutable std::mutex avoidance_mutex_;
  mutable std::mutex decision_mutex_;

  // 输入快照和当前输出路径分别加锁，避免Costmap回调阻塞路径读取。
  nav_msgs::msg::Path global_path_;
  nav_msgs::msg::Path local_path_;
  CostmapConstPtr costmap_;
  std::atomic<bool> global_path_received_;
  std::atomic<bool> costmap_received_;
  std::atomic<bool> local_path_valid_;
  std::atomic<bool> dynamic_path_mode_;
  std::atomic<bool> stage_alignment_requested_{false};

  std::atomic<std::size_t> current_global_path_progress_index_{0};

  // 路径稳定性相关
  nav_msgs::msg::Path previous_local_path_; // 上一次的局部路径
  std::atomic<int> path_stability_counter_; // 路径稳定性计数器
  double path_similarity_threshold_;        // 路径相似度阈值

  // 参数
  double planning_frequency_;  // 规划频率 (Hz)
  std::string global_frame_;   // 全局坐标系
  std::string robot_frame_;    // 机器人坐标系
  double transform_tolerance_; // TF变换容忍度
  double avoidance_reentry_cooldown_;
  double avoidance_failure_retry_cooldown_{2.0};
  int avoidance_max_planning_failures_{15};
  double avoidance_stall_timeout_{1.5};
  double avoidance_stall_distance_{0.05};
  double avoidance_stall_yaw_change_{0.0872664626};
  double local_avoidance_lookahead_distance_{0.25};
  int avoidance_path_blocked_confirmations_{3};
  std::atomic<int> consecutive_blocked_path_checks_{0};
  double avoidance_temporary_goal_retry_period_{0.5};
  double avoidance_side_goal_retry_forward_step_{0.25};
  double avoidance_side_goal_retry_max_forward_offset_{0.75};
  double avoidance_side_score_path_length_weight_{1.0};
  double avoidance_side_score_heading_weight_{1.5};
  double avoidance_side_score_turn_weight_{0.5};
  double avoidance_side_score_clearance_weight_{2.0};
  double avoidance_side_score_raw_path_penalty_{2.0};
  double avoidance_side_score_switch_side_penalty_{0.5};
  int avoidance_side_astar_candidate_limit_{5};
  double avoidance_replan_max_initial_heading_error_{25.0 * M_PI / 180.0};
  double avoidance_replan_heading_score_multiplier_{3.0};

  // 控制层状态也由Manager唯一持有，避免PurePursuit再维护镜像布尔量。
  bool tracking_local_;
  bool failure_latched_;
  bool has_failure_latch_time_{false};
  rclcpp::Time failure_latch_time_;
  int retry_attempt_{0};
  bool has_last_avoidance_complete_time_;
  rclcpp::Time last_avoidance_complete_time_;
  bool stall_monitoring_{false};
  rclcpp::Time stall_start_time_;
  geometry_msgs::msg::Point stall_start_position_;
  double stall_start_yaw_{0.0};
  int consecutive_waiting_clear_checks_{0};
  std::uint64_t last_waiting_clear_costmap_version_{0};

  // 单段绕障任务：搜索并锁定原全局路径上的障碍后临时点，然后只跟踪
  // “当前位置 -> 绕障临时点”的一段局部A*。
  bool avoidance_active_;
  bool avoidance_completed_;
  AvoidanceStage avoidance_stage_;
  geometry_msgs::msg::Point avoidance_obstacle_center_;
  geometry_msgs::msg::PoseStamped avoidance_temporary_goal_;
  std::size_t avoidance_temporary_goal_index_{
      std::numeric_limits<std::size_t>::max()};
  std::size_t avoidance_search_start_index_{0};
  // 障碍扫描基准与候选下限必须分离：A*失败后仍从原位置识别同一障碍，
  // 但跳过已经确定不可达的全局临时点。
  std::size_t avoidance_obstacle_search_start_index_{0};
  std::size_t avoidance_minimum_goal_index_{0};
  bool avoidance_temporary_goal_locked_{false};
  bool avoidance_temporary_goal_on_global_path_{false};
  // 全局路径临时点不可达后，下一轮必须先评估侧方候选，不能继续在
  // 后续全局路径点上消耗多次A*重试。
  bool avoidance_force_side_fallback_{false};
  // 已执行的局部路径失效后，只接受初始航向与当前机器人朝向连续的
  // 侧方路径，并允许这类小角度路径直接进入跟踪而不再次原地对准。
  bool avoidance_heading_continuity_required_{false};
  AvoidanceSide avoidance_preferred_side_{AvoidanceSide::NONE};
  bool avoidance_left_clear_at_trigger_{false};
  bool avoidance_right_clear_at_trigger_{false};
  bool temporary_goal_revalidation_required_{false};
  bool avoidance_corridor_selected_{false};
  int consecutive_planning_failures_{0};
  int temporary_goal_reselection_count_{0};
  int consecutive_temporary_goal_invalid_checks_{0};
  nav_msgs::msg::Path avoidance_temporary_path_;
  std::uint64_t avoidance_task_generation_{0};
  std::uint64_t last_validated_costmap_version_{0};
  std::uint64_t last_path_checked_costmap_version_{0};
  std::uint64_t last_retry_costmap_version_{0};
  bool temporary_goal_last_validation_valid_{false};
  std::chrono::steady_clock::time_point next_temporary_goal_retry_time_{};
  double avoidance_lateral_clearance_;
  double avoidance_max_entry_angle_;
  double avoidance_max_return_angle_;
  double avoidance_near_stop_clearance_{1.0};
  double avoidance_near_stop_margin_{0.0};
  double avoidance_path_clearance_{0.5};
  double avoidance_initial_escape_max_heading_error_{10.0 * M_PI / 180.0};
  double avoidance_goal_tolerance_;
  double avoidance_return_goal_clearance_;
  double avoidance_return_obstacle_clearance_;
  double avoidance_obstacle_threshold_;
  bool avoidance_dynamic_side_goal_fallback_enabled_{true};

  // 规划线程主循环和单周期状态推进。
  void planningLoop();
  void requestPlanningWakeup();
  bool computeLocalPath();
  bool startAvoidance(const geometry_msgs::msg::PoseStamped &robot_pose,
                      const AvoidanceObservation &observation);
  void resetAvoidance();
  bool isAvoidanceCompleted() const;
  bool isAvoidanceFailed() const;
  bool isWaitingForDynamicPath() const;
  bool isWaitingForTemporaryGoal() const;
  void enterWaitingForDynamicPath();
  nav_msgs::msg::Path getLocalPath() const;
  std::optional<nav_msgs::msg::Path> getValidLocalPath() const;
  void markAvoidanceFailed(const char *reason);
  void clearLocalPath();

  // 辅助函数
  bool getRobotPose(geometry_msgs::msg::PoseStamped &robot_pose);
  const char *avoidanceSideToString(AvoidanceSide side) const;
  int findClosestPointOnGlobalPath(
      const geometry_msgs::msg::PoseStamped &robot_pose);
  double calculateDistance(const geometry_msgs::msg::Point &p1,
                           const geometry_msgs::msg::Point &p2);

  // 新增的路径分析函数
  int findProjectionOnPath(const geometry_msgs::msg::PoseStamped &robot_pose,
                           const nav_msgs::msg::Path &path);
  int findBestPathPoint(const geometry_msgs::msg::PoseStamped &robot_pose,
                        const nav_msgs::msg::Path &path);
  double calculateDistance3D(const geometry_msgs::msg::Point &p1,
                             const geometry_msgs::msg::Point &p2);
  double pointToLineDistance(const geometry_msgs::msg::Point &point,
                             const geometry_msgs::msg::Point &line_start,
                             const geometry_msgs::msg::Point &line_end);
  geometry_msgs::msg::Point
  projectPointOnLine(const geometry_msgs::msg::Point &point,
                     const geometry_msgs::msg::Point &line_start,
                     const geometry_msgs::msg::Point &line_end);

  // 路径稳定性函数
  double calculatePathSimilarity(const nav_msgs::msg::Path &path1,
                                 const nav_msgs::msg::Path &path2);
  bool shouldKeepPreviousPath(const nav_msgs::msg::Path &new_path);

  // 可视化
  void publishGoalMarker(const geometry_msgs::msg::Point &goal_point);
  void publishBlockedPathMarker(
      const geometry_msgs::msg::Point &blocked_point);
  void publishSideGoalCandidates(
      const std::vector<SideGoalCandidate> &candidates,
      std::size_t astar_candidate_count);
  void clearAvoidanceMarkers();
};

} // namespace pure_pursuit_local_planner

#endif // LOCAL_AVOIDANCE_MANAGER_HPP
