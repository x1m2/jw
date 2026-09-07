#ifndef PURE_PURSUIT_LOCAL_PLANNER_HPP
#define PURE_PURSUIT_LOCAL_PLANNER_HPP

#include <slam_manager_interfaces/msg/task_target.hpp>
#include <tf2/time.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <limits>
#include <memory>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <navigation_interfaces/msg/avoidance_state.hpp>
#include <navigation_interfaces/edge_policy_utils.hpp>
#include <navigation_interfaces/msg/edge_policy.hpp>
#include <navigation_interfaces/msg/path_with_policy.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2/LinearMath/Scalar.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "pure_pursuit/front_obstacle_detector.hpp"
#include "pure_pursuit/local_avoidance_manager.hpp"
#include "pure_pursuit/path_tracker.hpp"
#include "pure_pursuit/pure_pursuit_controller.hpp"

namespace pure_pursuit_local_planner {

struct PlannerConfig {
  // Pure Pursuit parameters
  double lookahead_distance_min; // 最小前瞻距离
  double lookahead_distance_max; // 最大前瞻距离
  double goal_tolerance;         // 目标点容忍距离

  // Speed parameters
  double max_linear_velocity;   // 最大线速度 (X方向)
  double max_linear_velocity_y; // Y方向最大线速度 (全向机器人)
  double max_angular_velocity;  // 最大角速度
  double acceleration_limit;    // 加速度限制
  double deceleration_limit;    // 减速度限制

  // Obstacle avoidance parameters
  // Control parameters
  double control_frequency;   // 控制频率
  double transform_tolerance; // TF变换容忍时间

  // Orientation adjustment parameters
  double yaw_tolerance;       // 角度容忍度
  double yaw_adjustment_gain; // 角度调整增益

  // Local planning parameters
  bool use_smoothed_path; // 是否使用平滑后的路径

  // Frame IDs
  std::string global_frame; // 全局坐标系
  std::string robot_frame;  // 机器人坐标系

  double final_yaw_min_angular_velocity = 0.28;
  double local_avoidance_align_enter_angle = 0.524;
  double local_avoidance_align_exit_angle = 0.105;
  double local_avoidance_min_align_angular_velocity = 0.35;
  double local_avoidance_lookahead_distance = 0.25;
  double local_avoidance_min_linear_velocity = 0.4;
  double local_avoidance_max_linear_velocity = 0.5;
  // 动态目标明显变化时，放弃基于旧目标生成的局部绕障路径。
  double dynamic_goal_change_distance = 0.7;
  double dynamic_goal_change_angle = 0.436;
};

class PurePursuitLocalPlanner {
public:
  PurePursuitLocalPlanner();
  ~PurePursuitLocalPlanner();

  /**
   * @brief 初始化局部规划器
   * @param nh ROS节点句柄
   * @return 初始化是否成功
   */
  bool initialize(rclcpp::Node::SharedPtr node);

  /** Stop control updates and publish a zero velocity command. */
  void stop();

  /** 暂停时保留全局任务、丢弃瞬态绕障路径并持续输出零速度；继续后基于最新路径控制。 */
  bool setPaused(bool paused);

  /**
   * @brief 设置全局路径
   * @param global_path 全局路径
   * @return 设置是否成功
   */
  bool setGlobalPath(const nav_msgs::msg::Path &global_path,
                     bool dynamic_path = false);

  /**
   * @brief 计算控制指令
   * @param cmd_vel 输出的控制指令
   * @return 计算是否成功
   */
  bool computeVelocityCommands(geometry_msgs::msg::Twist &cmd_vel);

  /**
   * @brief 判断是否到达目标点
   * @return 是否到达目标点
   */
  bool isGoalReached(const geometry_msgs::msg::PoseStamped &robot_pose) const;

  /**
   * @brief 获取机器人在全局坐标系下的位姿
   * @param robot_pose 输出的机器人位姿
   * @return 获取是否成功
   */
  bool getRobotPose(geometry_msgs::msg::PoseStamped &robot_pose);

private:
  rclcpp::Subscription<slam_manager_interfaces::msg::TaskTarget>::SharedPtr
      task_target_sub_;
  // ROS相关
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lookahead_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr
      local_trajectory_pub_; // 局部轨迹发布器
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      velocity_arrow_pub_; // 速度箭头发布器
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
      nav_state_pub_; // 导航状态发布器
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr nav_failed_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obstacle_stop_enabled_pub_;
  rclcpp::Publisher<navigation_interfaces::msg::AvoidanceState>::SharedPtr
      avoidance_state_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr pause_service_;
  rclcpp::Subscription<navigation_interfaces::msg::PathWithPolicy>::SharedPtr
      global_path_with_policy_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr
      smoothed_local_path_sub_; // 新增：平滑后局部路径订阅器
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
      obstacle_stop_active_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // 配置参数
  PlannerConfig config_;

  // 路径和状态
  nav_msgs::msg::Path global_path_;
  nav_msgs::msg::Path smoothed_local_path_; // 新增：平滑后的局部路径
  bool smoothed_local_path_received_; // 新增：平滑后局部路径接收标志
  bool path_received_;
  bool goal_reached_;
  bool goal_position_reached_;           // 新增：目标位置到达锁存
  geometry_msgs::msg::Pose target_pose_; // 新增：目标位姿
  bool target_pose_received_;            // 新增：目标位姿接收标志
  int current_path_index_;
  size_t path_progress_index_; // 路径进度索引，防止回头
  std::vector<navigation_interfaces::msg::EdgePolicy> edge_policies_;
  bool published_obstacle_stop_enabled_{true};
  bool has_published_obstacle_stop_policy_{false};
  std::atomic<uint8_t> avoidance_state_{255};
  std::atomic<int64_t> dynamic_path_accept_after_ns_{0};
  // 组件
  std::unique_ptr<PathTracker> path_tracker_;
  std::unique_ptr<PurePursuitController> controller_;
  std::unique_ptr<LocalAvoidanceManager>
      local_avoidance_manager_; // 新增：局部路径规划器
  std::unique_ptr<FrontObstacleDetector>
      front_obstacle_detector_; // 新增：前方障碍物检测器

  // 控制相关
  geometry_msgs::msg::Twist last_cmd_vel_;
  rclcpp::Time last_control_time_;
  std::atomic_bool navigation_paused_{false};
  std::atomic_bool reset_velocity_on_next_control_{false};
  std::atomic_bool obstacle_stop_active_{false};

  /**
   * @brief 加载参数
   */
  void loadParameters();

  void publishAvoidanceFailure(bool failed);
  void publishAvoidanceState(const LocalAvoidanceOutput &output);

  /**
   * @brief 控制回调函数
   */
  void controlCallback();

  /**
   * @brief 全局路径回调函数
   */
  void globalPathWithPolicyCallback(
      navigation_interfaces::msg::PathWithPolicy::ConstSharedPtr msg);
  bool hasDynamicGoalChanged(const nav_msgs::msg::Path &new_path,
                             double &goal_shift,
                             double &direction_change);
  void publishCurrentObstacleStopPolicy();
  bool isObstacleAvoidanceEnabled() const;
  navigation_interfaces::msg::EdgePolicy currentEdgePolicy() const;

  /**
   * @brief 代价地图回调函数
   */
  void costmapCallback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);

  /**
   * @brief 平滑后局部路径回调函数
   */
  void smoothedLocalPathCallback(nav_msgs::msg::Path::ConstSharedPtr msg);

  /**
   * @brief 目标点回调函数
   */
  void targetPointsCallback(geometry_msgs::msg::Pose::ConstSharedPtr msg);

  /**
   * @brief 计算前瞻点
   * @param robot_pose 机器人当前位姿
   * @param lookahead_point 输出的前瞻点
   * @return 计算是否成功
   */
  bool computeLookaheadPoint(const geometry_msgs::msg::PoseStamped &robot_pose,
                             bool local_tracking,
                             const std::optional<nav_msgs::msg::Path> &local_path,
                             geometry_msgs::msg::PointStamped &lookahead_point);

  /**
   * @brief 计算Pure Pursuit控制指令
   * @param robot_pose 机器人当前位姿
   * @param lookahead_point 前瞻点
   * @param cmd_vel 输出的控制指令
   * @return 计算是否成功
   */
  /**
   * @brief 发布可视化信息
   */
  void publishVisualization(
      const geometry_msgs::msg::PointStamped &lookahead_point);

  /**
   * @brief 发布局部轨迹可视化
   */
  void publishLocalTrajectory(const geometry_msgs::msg::PoseStamped &robot_pose,
                              const geometry_msgs::msg::Twist &cmd_vel);

  /**
   * @brief 发布速度箭头可视化
   */
  void publishVelocityArrow(const geometry_msgs::msg::PoseStamped &robot_pose,
                            const geometry_msgs::msg::Twist &cmd_vel);

  /**
   * @brief 生成局部轨迹预测路径
   */
  nav_msgs::msg::Path
  generateLocalTrajectory(const geometry_msgs::msg::PoseStamped &robot_pose,
                          const geometry_msgs::msg::Twist &cmd_vel,
                          double prediction_time = 2.0, double dt = 0.1);

  /**
   * @brief 更新路径进度索引（防止回头）
   * @param robot_pose 机器人当前位姿
   */
  void updatePathProgress(const geometry_msgs::msg::PoseStamped &robot_pose);

  /**
   * @brief 检查是否已达到目标朝向
   * @param robot_pose 机器人当前位姿
   * @return 是否达到目标朝向
   */
};

} // namespace pure_pursuit_local_planner

#endif // PURE_PURSUIT_LOCAL_PLANNER_H
