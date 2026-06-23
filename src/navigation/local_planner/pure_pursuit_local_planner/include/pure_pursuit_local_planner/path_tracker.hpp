#ifndef PATH_TRACKER_HPP
#define PATH_TRACKER_HPP

#include <tf2/utils.h>

#include <cmath>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

namespace pure_pursuit_local_planner {

struct PathPoint {
  geometry_msgs::msg::PoseStamped pose;
  double curvature;        // 路径曲率
  double target_velocity;  // 目标速度
  bool is_valid;           // 是否有效
};

class PathTracker {
 public:
  PathTracker();
  ~PathTracker();

  /**
   * @brief 初始化路径跟踪器
   * @param nh ROS节点句柄
   * @return 初始化是否成功
   */
  bool initialize(rclcpp::Node::SharedPtr node);

  /**
   * @brief 设置全局路径
   * @param path 全局路径
   * @return 设置是否成功
   */
  bool setPath(const nav_msgs::msg::Path& path);

  /**
   * @brief 找到最近的路径点
   * @param robot_pose 机器人当前位姿
   * @return 最近路径点的索引
   */
  int findClosestPathPoint(const geometry_msgs::msg::PoseStamped& robot_pose);

  /**
   * @brief 计算前瞻点
   * @param robot_pose 机器人当前位姿
   * @param lookahead_distance 前瞻距离
   * @param closest_index 最近点索引
   * @param lookahead_point 输出的前瞻点
   * @return 计算是否成功
   */
  bool computeLookaheadPoint(const geometry_msgs::msg::PoseStamped& robot_pose,
                             double lookahead_distance, int closest_index,
                             geometry_msgs::msg::PointStamped& lookahead_point);

  /**
   * @brief 计算路径曲率
   * @param index 路径点索引
   * @return 曲率值
   */
  double computePathCurvature(int index);

  /**
   * @brief 根据机器人方向和前瞻点方向计算目标速度
   * @param robot_pose 机器人当前位姿
   * @param lookahead_point 前瞻点
   * @param max_velocity 最大速度
   * @return 目标速度
   */
  double computeTargetVelocity(
      const geometry_msgs::msg::PoseStamped& robot_pose,
      const geometry_msgs::msg::PointStamped& lookahead_point,
      double max_velocity);

  /**
   * @brief 检查是否到达路径终点
   * @param robot_pose 机器人当前位姿
   * @param tolerance 容忍距离
   * @return 是否到达终点
   */
  bool isPathComplete(const geometry_msgs::msg::PoseStamped& robot_pose,
                      double tolerance);

  /**
   * @brief 获取当前路径长度
   * @return 路径长度
   */
  double getPathLength();

  /**
   * @brief 获取剩余路径长度
   * @param current_index 当前索引
   * @return 剩余路径长度
   */
  double getRemainingPathLength(int current_index);

  /**
   * @brief 发布可视化信息
   */
  void publishVisualization(
      const geometry_msgs::msg::PoseStamped& robot_pose,
      const geometry_msgs::msg::PointStamped& lookahead_point,
      int closest_index);

 private:
  // ROS相关
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      path_markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      lookahead_marker_pub_;

  // 路径数据
  std::vector<PathPoint> path_points_;
  bool path_valid_;

  // 参数
  double curvature_smoothing_window_;  // 曲率平滑窗口
  double max_curvature_;               // 最大曲率
  double min_lookahead_distance_;      // 最小前瞻距离
  double max_lookahead_distance_;      // 最大前瞻距离

  /**
   * @brief 加载参数
   */
  void loadParameters();

  /**
   * @brief 预处理路径（计算曲率、目标速度等）
   */
  void preprocessPath();

  /**
   * @brief 计算两点间距离
   */
  double calculateDistance(const geometry_msgs::msg::Point& p1,
                           const geometry_msgs::msg::Point& p2);

  /**
   * @brief 计算两个位姿间的距离
   */
  double calculateDistance(const geometry_msgs::msg::PoseStamped& pose1,
                           const geometry_msgs::msg::PoseStamped& pose2);

  /**
   * @brief 线性插值计算前瞻点
   */
  geometry_msgs::msg::PointStamped interpolateLookaheadPoint(
      const PathPoint& p1, const PathPoint& p2,
      const geometry_msgs::msg::PoseStamped& robot_pose,
      double lookahead_distance);

  /**
   * @brief 创建路径可视化标记
   */
  visualization_msgs::msg::Marker createPathMarker();

  /**
   * @brief 创建前瞻点可视化标记
   */
  visualization_msgs::msg::Marker createLookaheadMarker(
      const geometry_msgs::msg::PointStamped& lookahead_point);

  /**
   * @brief 创建最近点可视化标记
   */
  visualization_msgs::msg::Marker createClosestPointMarker(int closest_index);
};

}  // namespace pure_pursuit_local_planner

#endif  // PATH_TRACKER_H
