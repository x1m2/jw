#ifndef FRONT_OBSTACLE_DETECTOR_HPP
#define FRONT_OBSTACLE_DETECTOR_HPP

#include <tf2/time.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

#include "pure_pursuit/costmap_types.hpp"
#include "pure_pursuit/avoidance_types.hpp"

namespace pure_pursuit_local_planner {

class FrontObstacleDetector {
 public:
  using ObstacleDecision = ::pure_pursuit_local_planner::ObstacleDecision;
  using AvoidanceInfo = ::pure_pursuit_local_planner::AvoidanceObservation;

  FrontObstacleDetector();

  /**
   * @brief 初始化障碍物检测器
   * @param nh ROS节点句柄
   * @param tf_buffer TF变换缓冲区
   * @return 是否初始化成功
   */
  bool initialize(rclcpp::Node::SharedPtr node);

  /**
   * @brief 更新代价地图
   * @param costmap 代价地图
   */
  void updateCostmap(CostmapConstPtr costmap);

  /**
   * @brief 更新当前全局路径，用于过滤路径外的绕障候选障碍。
   */
  void setGlobalPath(const nav_msgs::msg::Path& global_path);

  /**
   * @brief 更新窄通道检测使用的原始全局路径，不随局部A*路径切换。
   */
  void setNarrowPassageReferencePath(
      const nav_msgs::msg::Path& global_path);

  /**
   * @brief 绕障决策：检测障碍物中心及左右通道状态
   */
  AvoidanceInfo analyzeAvoidance(
      const geometry_msgs::msg::PoseStamped& robot_pose);

  /**
   * @brief 以当前全局路径方向为局部纵轴检测连续中央通道。
   * @note 第一阶段结果仅用于观测和测试，不改变 AvoidanceInfo 或控制路径。
   */
  NarrowPassageResult detectNarrowPassage(
      const geometry_msgs::msg::PoseStamped& robot_pose);

  /**
   * @brief 发布检测区域可视化
   * @param robot_pose 机器人当前位姿
   * @param obstacle_detected 是否检测到障碍物
   */
  void publishVisualization(const geometry_msgs::msg::PoseStamped& robot_pose,
                            bool obstacle_detected);

 private:
  /**
   * @brief 获取地图指定位置的代价值
   * @param map_x 地图坐标X
   * @param map_y 地图坐标Y
   * @return 代价值，-1表示越界
   */
  int getCostAtMapCoordinate(int map_x, int map_y);

  bool detectObstacleInRectLocal(
      const geometry_msgs::msg::PoseStamped& robot_pose, double x_min,
      double x_max, double y_min, double y_max, double ratio_threshold);
  bool findObstacleInSectorLocal(
      const geometry_msgs::msg::PoseStamped& robot_pose, double min_radius,
      double max_radius, double angle_deg, AvoidanceInfo& info);
  bool isPassageClear(const geometry_msgs::msg::PoseStamped& robot_pose,
                      bool check_left);
  double pointToSegmentDistance(double px, double py,
                                const geometry_msgs::msg::Point& a,
                                const geometry_msgs::msg::Point& b) const;
  bool isPointInSectorLocal(double local_x, double local_y, double min_radius,
                            double max_radius, double angle_deg) const;
  const char* decisionToString(ObstacleDecision decision) const;
  NarrowPassageResult detectNarrowPassageLocked(
      const geometry_msgs::msg::PoseStamped& robot_pose);
  void publishNarrowPassageVisualization(
      const geometry_msgs::msg::PoseStamped& robot_pose,
      const NarrowPassageResult& result);

  /**
   * @brief 生成扇形检测区域的边界点
   * @param robot_pose 机器人位姿
   * @param sector_points 输出的扇形边界点
   */
  void generateSectorAreaPoints(
      const geometry_msgs::msg::PoseStamped& robot_pose,
      std::vector<std::pair<double, double>>& sector_points);

  // ROS相关
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      detection_area_pub_;  // 检测区域可视化发布器
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr
      narrow_passage_center_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      narrow_passage_debug_marker_pub_;

  // 代价地图
  CostmapConstPtr costmap_;
  bool has_costmap_;
  mutable std::mutex costmap_mutex_;
  nav_msgs::msg::Path global_path_;
  bool has_global_path_;
  nav_msgs::msg::Path narrow_passage_reference_path_;
  bool has_narrow_passage_reference_path_;
  mutable std::mutex global_path_mutex_;

  // 参数
  int obstacle_threshold_;           // 障碍物阈值 (代价值)
  double obstacle_ratio_threshold_;  // 障碍物占比触发阈值
  double robot_width_;               // 车体宽度
  double robot_length_;              // 车体长度
  double safety_margin_;             // 车体外扩安全边距
  double avoid_min_distance_;        // 绕障检测起始距离
  double avoid_max_distance_;        // 绕障检测最大距离
  double avoid_sector_angle_;        // 绕障检测扇形角度
  double side_passage_width_;        // 左右通道检测宽度
  double side_passage_obstacle_ratio_threshold_; // 通道障碍占比阈值
  double near_side_obstacle_lateral_threshold_; // 近障碍判为单侧障碍的横向阈值
  bool enable_path_corridor_filter_;  // 是否只处理全局路径走廊内的绕障障碍
  double path_obstacle_margin_;  // 路径走廊额外安全边距
  double path_check_distance_;  // 机器人前方参与绕障判断的路径长度

  // 窄通道影子检测参数；只生成诊断结果，不进入控制链。
  bool narrow_passage_enabled_;
  double narrow_passage_margin_;
  double narrow_passage_forward_range_;
  double narrow_passage_lateral_range_;
  double narrow_passage_bin_size_;
  double narrow_passage_min_length_;
  double narrow_passage_valid_bin_ratio_;
  int narrow_passage_center_smoothing_window_;
  double narrow_passage_min_confidence_;
  double narrow_passage_boundary_continuity_tolerance_;
  double narrow_passage_direction_tolerance_degrees_;
  double narrow_passage_width_stddev_tolerance_;
  double narrow_passage_center_obstacle_ratio_threshold_;

  // 调试和可视化
  bool debug_mode_;
  bool enable_visualization_;  // 是否启用可视化
};

}  // namespace pure_pursuit_local_planner

#endif  // FRONT_OBSTACLE_DETECTOR_H
