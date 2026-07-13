#ifndef PURE_PURSUIT_LOCAL_PLANNER_HPP
#define PURE_PURSUIT_LOCAL_PLANNER_HPP

#include <tf2/time.h>

#include <algorithm>
#include <chrono>
#include <cmath>
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
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "pure_pursuit_local_planner/front_obstacle_detector.hpp"
#include "pure_pursuit_local_planner/local_path_planner.hpp"
#include "pure_pursuit_local_planner/obstacle_avoidance.hpp"
#include "pure_pursuit_local_planner/path_tracker.hpp"

namespace pure_pursuit_local_planner {

struct PlannerConfig {
  // Pure Pursuit parameters
  double lookahead_distance_min;  // 最小前瞻距离
  double lookahead_distance_max;  // 最大前瞻距离
  double goal_tolerance;          // 目标点容忍距离

  // Speed parameters
  double max_linear_velocity;    // 最大线速度 (X方向)
  double max_linear_velocity_y;  // Y方向最大线速度 (全向机器人)
  double max_angular_velocity;   // 最大角速度
  double min_linear_velocity;    // 最小线速度
  double acceleration_limit;     // 加速度限制
  double deceleration_limit;     // 减速度限制
  double stairs_velocity_boost;  // 楼梯状态下额外增加的X方向最大线速度

  // Obstacle avoidance parameters
  double obstacle_detection_range;  // 障碍物检测范围
  double safety_distance;           // 安全距离
  double avoidance_weight;          // 避障权重
  double path_following_weight;     // 路径跟踪权重

  // Control parameters
  double control_frequency;    // 控制频率
  double transform_tolerance;  // TF变换容忍时间

  // Orientation adjustment parameters
  double yaw_tolerance;        // 角度容忍度
  double yaw_adjustment_gain;  // 角度调整增益

  // Local planning parameters
  bool enable_obstacle_replanning;  // 是否启用基于障碍物中心的主动绕障
  bool enable_local_replanning;  // 是否启用A*局部重规划
  bool use_smoothed_path;        // 是否使用平滑后的路径
  double avoidance_reentry_cooldown;  // 绕障完成后重新触发冷却时间

  // Frame IDs
  std::string global_frame;  // 全局坐标系
  std::string robot_frame;   // 机器人坐标系

  double final_yaw_min_angular_velocity = 0.28;
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

  /**
   * @brief 设置全局路径
   * @param global_path 全局路径
   * @return 设置是否成功
   */
  bool setGlobalPath(const nav_msgs::msg::Path& global_path);

  /**
   * @brief 计算控制指令
   * @param cmd_vel 输出的控制指令
   * @return 计算是否成功
   */
  bool computeVelocityCommands(geometry_msgs::msg::Twist& cmd_vel);

  /**
   * @brief 判断是否到达目标点
   * @return 是否到达目标点
   */
  bool isGoalReached();

  /**
   * @brief 获取机器人在全局坐标系下的位姿
   * @param robot_pose 输出的机器人位姿
   * @return 获取是否成功
   */
  bool getRobotPose(geometry_msgs::msg::PoseStamped& robot_pose);

 private:
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stairs_state_sub_;
  bool emergency_stop_active_ = false;
  bool stairs_state_active_ = false;
  // ROS相关
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lookahead_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr
      local_trajectory_pub_;  // 局部轨迹发布器
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      velocity_arrow_pub_;  // 速度箭头发布器
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
      nav_state_pub_;  // 导航状态发布器
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr
      smoothed_local_path_sub_;  // 新增：平滑后局部路径订阅器
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr
      target_points_sub_;  // 新增：目标点订阅器
  rclcpp::TimerBase::SharedPtr control_timer_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // 配置参数
  PlannerConfig config_;

  // 路径和状态
  nav_msgs::msg::Path global_path_;
  nav_msgs::msg::Path smoothed_local_path_;  // 新增：平滑后的局部路径
  bool smoothed_local_path_received_;  // 新增：平滑后局部路径接收标志
  bool path_received_;
  bool goal_reached_;
  bool goal_position_reached_;            // 新增：目标位置到达锁存
  bool yaw_adjusting_;                    // 新增：是否正在调整朝向
  geometry_msgs::msg::Pose target_pose_;  // 新增：目标位姿
  bool target_pose_received_;             // 新增：目标位姿接收标志
  int current_path_index_;
  size_t path_progress_index_;  // 路径进度索引，防止回头
  geometry_msgs::msg::PointStamped last_lookahead_point_;  // 上一次的前瞻点
  bool has_valid_lookahead_;  // 是否有有效的前瞻点
  bool pp_aligning_;
  bool pp_correcting_;
  bool local_replanning_active_;  // 是否正在跟踪局部绕障路径
  bool local_avoidance_aligning_;  // 局部绕障起步时先原地对齐
  bool has_last_avoidance_complete_time_;
  rclcpp::Time last_avoidance_complete_time_;
  bool latest_costmap_received_;
  nav_msgs::msg::OccupancyGrid latest_costmap_;
  // 组件
  std::unique_ptr<ObstacleAvoidance> obstacle_avoidance_;
  std::unique_ptr<PathTracker> path_tracker_;
  std::unique_ptr<LocalPathPlanner>
      local_path_planner_;  // 新增：局部路径规划器
  std::unique_ptr<FrontObstacleDetector>
      front_obstacle_detector_;  // 新增：前方障碍物检测器

  // 控制相关
  geometry_msgs::msg::Twist last_cmd_vel_;
  rclcpp::Time last_control_time_;

  /**
   * @brief 加载参数
   */
  void loadParameters();

  void setLocalReplanningActive(bool active);
  bool isLocalReplanningEnabled() const;

  /**
   * @brief 控制回调函数
   */
  void controlCallback();

  /**
   * @brief 全局路径回调函数
   */
  void globalPathCallback(nav_msgs::msg::Path::ConstSharedPtr msg);

  /**
   * @brief 代价地图回调函数
   */
  void costmapCallback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);

  /**
   * @brief 激光雷达回调函数
   */
  void scanCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg);

  /**
   * @brief emergency_stop回调函数
   */
  void emergencyStopCallback(std_msgs::msg::Bool::ConstSharedPtr msg);
  void stairsStateCallback(std_msgs::msg::Bool::ConstSharedPtr msg);
  double effectiveMaxLinearVelocity() const;

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
  bool computeLookaheadPoint(const geometry_msgs::msg::PoseStamped& robot_pose,
                             geometry_msgs::msg::PointStamped& lookahead_point);

  /**
   * @brief 计算Pure Pursuit控制指令
   * @param robot_pose 机器人当前位姿
   * @param lookahead_point 前瞻点
   * @param cmd_vel 输出的控制指令
   * @return 计算是否成功
   */
  bool computePurePursuitCommand(
      const geometry_msgs::msg::PoseStamped& robot_pose,
      const geometry_msgs::msg::PointStamped& lookahead_point,
      geometry_msgs::msg::Twist& cmd_vel);

  /**
   * @brief 应用速度限制
   * @param cmd_vel 控制指令
   */
  void applyVelocityLimits(geometry_msgs::msg::Twist& cmd_vel);

  /**
   * @brief 发布可视化信息
   */
  void publishVisualization(
      const geometry_msgs::msg::PoseStamped& robot_pose,
      const geometry_msgs::msg::PointStamped& lookahead_point);

  /**
   * @brief 发布局部轨迹可视化
   */
  void publishLocalTrajectory(const geometry_msgs::msg::PoseStamped& robot_pose,
                              const geometry_msgs::msg::Twist& cmd_vel);

  /**
   * @brief 发布速度箭头可视化
   */
  void publishVelocityArrow(const geometry_msgs::msg::PoseStamped& robot_pose,
                            const geometry_msgs::msg::Twist& cmd_vel);

  /**
   * @brief 生成局部轨迹预测路径
   */
  nav_msgs::msg::Path generateLocalTrajectory(
      const geometry_msgs::msg::PoseStamped& robot_pose,
      const geometry_msgs::msg::Twist& cmd_vel, double prediction_time = 2.0,
      double dt = 0.1);

  /**
   * @brief 计算两点间距离
   */
  double calculateDistance(const geometry_msgs::msg::Point& p1,
                           const geometry_msgs::msg::Point& p2);

  /**
   * @brief 标准化角度到[-π, π]
   */
  double normalizeAngle(double angle);

  /**
   * @brief 计算圆与线段的交点
   * @param circle_x 圆心x坐标
   * @param circle_y 圆心y坐标
   * @param radius 圆半径
   * @param line_start 线段起点
   * @param line_end 线段终点
   * @return 交点列表
   */
  std::vector<geometry_msgs::msg::Point> computeCircleLineIntersection(
      double circle_x, double circle_y, double radius,
      const geometry_msgs::msg::Point& line_start,
      const geometry_msgs::msg::Point& line_end);

  /**
   * @brief 计算点在路径上的进度
   * @param segment_index 线段索引
   * @param point 点坐标
   * @return 累积路径长度
   */
  double calculatePathProgress(int segment_index,
                               const geometry_msgs::msg::Point& point);

  /**
   * @brief 更新路径进度索引（防止回头）
   * @param robot_pose 机器人当前位姿
   */
  void updatePathProgress(const geometry_msgs::msg::PoseStamped& robot_pose);

  /**
   * @brief 检查是否已达到目标朝向
   * @param robot_pose 机器人当前位姿
   * @return 是否达到目标朝向
   */
  bool isTargetOrientationReached(
      const geometry_msgs::msg::PoseStamped& robot_pose);

  /**
   * @brief 计算朝向调整控制指令
   * @param robot_pose 机器人当前位姿
   * @param cmd_vel 输出的控制指令
   * @return 计算是否成功
   */
  bool computeOrientationAdjustmentCommand(
      const geometry_msgs::msg::PoseStamped& robot_pose,
      geometry_msgs::msg::Twist& cmd_vel);
};

}  // namespace pure_pursuit_local_planner

#endif  // PURE_PURSUIT_LOCAL_PLANNER_H
