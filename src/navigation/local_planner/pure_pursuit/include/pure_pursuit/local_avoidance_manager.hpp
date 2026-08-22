#ifndef LOCAL_AVOIDANCE_MANAGER_HPP
#define LOCAL_AVOIDANCE_MANAGER_HPP

#include <tf2/time.h>

#include <atomic>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
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
      bool global_controller_aligning);
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

  // 同步绕障路径计算层；状态和工作线程仍由本类管理。
  std::unique_ptr<AvoidancePathPlanner> avoidance_path_planner_;

  // 规划线程只周期性计算局部路径；ROS控制循环通过下面的互斥量读取快照。
  std::thread planning_thread_;
  std::atomic<bool> running_;
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

  // 控制层状态也由Manager唯一持有，避免PurePursuit再维护镜像布尔量。
  bool tracking_local_;
  bool failure_latched_;
  bool has_last_avoidance_complete_time_;
  rclcpp::Time last_avoidance_complete_time_;

  // 静态路径：单个侧向临时点 -> 回归点；动态路径在临时点等待新路径。
  bool avoidance_active_;
  bool avoidance_completed_;
  AvoidanceStage avoidance_stage_;
  geometry_msgs::msg::Point avoidance_obstacle_center_;
  geometry_msgs::msg::Point avoidance_first_side_goal_;
  double avoidance_side_goal_yaw_{0.0};
  geometry_msgs::msg::Point avoidance_return_goal_;
  bool avoidance_return_goal_precomputed_{false};
  AvoidanceSide avoidance_side_{AvoidanceSide::NONE};
  // 两段路径在本次绕障期间保持不变，但控制器一次只能看到当前阶段。
  bool avoidance_corridor_selected_{false};
  nav_msgs::msg::Path avoidance_side_path_;
  nav_msgs::msg::Path avoidance_return_path_;
  double avoidance_lateral_clearance_;
  double avoidance_max_entry_angle_;
  double avoidance_max_return_angle_;
  double avoidance_near_stop_clearance_{1.0};
  double avoidance_near_stop_margin_{0.0};
  double avoidance_goal_tolerance_;
  double avoidance_return_goal_clearance_;
  double avoidance_return_obstacle_clearance_;
  double avoidance_obstacle_threshold_;

  // 规划线程主循环和单周期状态推进。
  void planningLoop();
  bool computeLocalPath();
  bool startAvoidance(const geometry_msgs::msg::PoseStamped &robot_pose,
                      const geometry_msgs::msg::Point &obstacle_center,
                      AvoidanceSide side);
  void resetAvoidance();
  bool isAvoidanceCompleted() const;
  bool isAvoidanceFailed() const;
  bool isWaitingForDynamicPath() const;
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
};

} // namespace pure_pursuit_local_planner

#endif // LOCAL_AVOIDANCE_MANAGER_HPP
