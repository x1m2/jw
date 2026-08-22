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
   * @brief 绕障决策：检测障碍物中心及左右通道状态
   */
  AvoidanceInfo analyzeAvoidance(
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
      double x_max, double y_min, double y_max);
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

  // 代价地图
  CostmapConstPtr costmap_;
  bool has_costmap_;
  mutable std::mutex costmap_mutex_;
  nav_msgs::msg::Path global_path_;
  bool has_global_path_;
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
  bool enable_path_corridor_filter_;  // 是否只处理全局路径走廊内的绕障障碍
  double path_obstacle_margin_;  // 路径走廊额外安全边距
  double path_check_distance_;  // 机器人前方参与绕障判断的路径长度

  // 调试和可视化
  bool debug_mode_;
  bool enable_visualization_;  // 是否启用可视化
};

}  // namespace pure_pursuit_local_planner

#endif  // FRONT_OBSTACLE_DETECTOR_H
