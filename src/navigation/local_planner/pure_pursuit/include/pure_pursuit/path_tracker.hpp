#ifndef PATH_TRACKER_HPP
#define PATH_TRACKER_HPP

#include <array>
#include <cstddef>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <visualization_msgs/msg/marker.hpp>

namespace pure_pursuit_local_planner {

struct PathTrackerConfig {
  double lookahead_distance_min{0.0};
  double lookahead_distance_max{0.0};
  double max_linear_velocity{0.0};
  std::string output_frame;
};

enum class TrackingPathType {
  kGlobal,
  kLocal,
  kSmoothedLocal,
};

struct TrackingContext {
  TrackingPathType path_type{TrackingPathType::kGlobal};
  std::size_t progress_index{0U};
  double current_velocity{0.0};
  // 大于0时使用固定前瞻距离。局部A*路径需要更短前瞻，避免在折角切弯。
  double lookahead_distance_override{0.0};
};

struct TrackingResult {
  geometry_msgs::msg::PointStamped lookahead_point;
  // 机器人当前实际投影所在的路径段。它与前瞻点所在段必须分开保存，
  // 否则把前瞻点索引当作机器人进度会在每个控制周期持续向前跳段。
  std::size_t progress_index{0U};
  std::size_t path_index{0U};
};

class PathTracker {
public:
  /**
   * @brief 初始化路径跟踪器
   * @param nh ROS节点句柄
   * @return 初始化是否成功
   */
  bool initialize(rclcpp::Node::SharedPtr node,
                  const PathTrackerConfig &config);

  /**
   * @brief 设置全局路径
   * @param path 全局路径
   * @return 设置是否成功
   */
  bool setPath(const nav_msgs::msg::Path &path);

  // Tracks the selected global/local path using the existing circle/segment
  // lookahead behavior. Path selection remains in the top-level planner.
  std::optional<TrackingResult> computeTrackingLookahead(
      const geometry_msgs::msg::PoseStamped &robot_pose,
      const nav_msgs::msg::Path &path, const TrackingContext &context);

  /**
   * @brief 根据机器人方向和前瞻点方向计算目标速度
   * @param robot_pose 机器人当前位姿
   * @param lookahead_point 前瞻点
   * @param max_velocity 最大速度
   * @return 目标速度
   */
  double
  computeTargetVelocity(const geometry_msgs::msg::PoseStamped &robot_pose,
                        const geometry_msgs::msg::PointStamped &lookahead_point,
                        double max_velocity, int motion_direction);

  /**
   * @brief 发布可视化信息
   */
  void publishVisualization(
      const geometry_msgs::msg::PointStamped &lookahead_point);

private:
  // ROS相关
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      path_markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      lookahead_marker_pub_;
  PathTrackerConfig config_;

  // 路径数据
  nav_msgs::msg::Path path_;

  /**
   * @brief 计算两点间距离
   */
  double calculateDistance(const geometry_msgs::msg::Point &p1,
                           const geometry_msgs::msg::Point &p2);

  static std::size_t computeCircleLineIntersection(
      double circle_x, double circle_y, double radius,
      const geometry_msgs::msg::Point &line_start,
      const geometry_msgs::msg::Point &line_end,
      std::array<geometry_msgs::msg::Point, 2> &intersections);
  static const char *pathTypeLabel(TrackingPathType path_type);

  /**
   * @brief 创建路径可视化标记
   */
  visualization_msgs::msg::Marker createPathMarker();

  /**
   * @brief 创建前瞻点可视化标记
   */
  visualization_msgs::msg::Marker createLookaheadMarker(
      const geometry_msgs::msg::PointStamped &lookahead_point);

};

} // namespace pure_pursuit_local_planner

#endif // PATH_TRACKER_H
