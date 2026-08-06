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
#include <tf2_ros/buffer.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

namespace pure_pursuit_local_planner {

class FrontObstacleDetector {
 public:
  enum class ObstacleDecision { CLEAR, AVOIDABLE, BLOCKED };

  struct AvoidanceInfo {
    ObstacleDecision decision = ObstacleDecision::CLEAR;
    geometry_msgs::msg::Point obstacle_center;
    geometry_msgs::msg::Point closest_obstacle;
    double obstacle_distance = 0.0;
    bool left_clear = false;
    bool right_clear = false;
  };

  FrontObstacleDetector();
  ~FrontObstacleDetector();

  /**
   * @brief 初始化障碍物检测器
   * @param nh ROS节点句柄
   * @param tf_buffer TF变换缓冲区
   * @return 是否初始化成功
   */
  bool initialize(rclcpp::Node::SharedPtr node,
                  std::shared_ptr<tf2_ros::Buffer> tf_buffer);

  /**
   * @brief 更新代价地图
   * @param costmap 代价地图
   */
  void updateCostmap(const nav_msgs::msg::OccupancyGrid& costmap);

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
   * @brief 检测指定区域内是否有障碍物
   * @param robot_pose 机器人位姿
   * @return 是否检测到障碍物
   */
  bool detectObstacleInFrontArea(
      const geometry_msgs::msg::PoseStamped& robot_pose);

  /**
   * @brief 将世界坐标转换为代价地图坐标
   * @param world_x 世界坐标X
   * @param world_y 世界坐标Y
   * @param map_x 输出地图坐标X
   * @param map_y 输出地图坐标Y
   * @return 是否转换成功
   */
  bool worldToMap(double world_x, double world_y, int& map_x, int& map_y);

  /**
   * @brief 获取地图指定位置的代价值
   * @param map_x 地图坐标X
   * @param map_y 地图坐标Y
   * @return 代价值，-1表示越界
   */
  int getCostAtMapCoordinate(int map_x, int map_y);

  /**
   * @brief 生成检测区域的角点
   * @param robot_pose 机器人位姿
   * @param corners 输出的角点坐标
   */
  void generateDetectionAreaCorners(
      const geometry_msgs::msg::PoseStamped& robot_pose,
      std::vector<std::pair<double, double>>& corners);

  /**
   * @brief 检测扇形区域内是否有障碍物
   * @param robot_pose 机器人位姿
   * @return 是否检测到障碍物
   */
  bool detectObstacleInSectorArea(
      const geometry_msgs::msg::PoseStamped& robot_pose);

  bool detectObstacleInRectLocal(
      const geometry_msgs::msg::PoseStamped& robot_pose, double x_min,
      double x_max, double y_min, double y_max);
  bool findObstacleInSectorLocal(
      const geometry_msgs::msg::PoseStamped& robot_pose, double min_radius,
      double max_radius, double angle_deg, AvoidanceInfo& info);
  bool isPassageClear(const geometry_msgs::msg::PoseStamped& robot_pose,
                      bool check_left);
  bool isPointInPathCorridor(const geometry_msgs::msg::PoseStamped& robot_pose,
                             double world_x, double world_y) const;
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

  /**
   * @brief 检查点是否在扇形区域内
   * @param point_x 点的X坐标
   * @param point_y 点的Y坐标
   * @param robot_x 机器人X坐标
   * @param robot_y 机器人Y坐标
   * @param robot_yaw 机器人朝向
   * @return 是否在扇形内
   */
  bool isPointInSector(double point_x, double point_y, double robot_x,
                       double robot_y, double robot_yaw);

  // ROS相关
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      detection_area_pub_;  // 检测区域可视化发布器

  // 代价地图
  nav_msgs::msg::OccupancyGrid costmap_;
  bool has_costmap_;
  nav_msgs::msg::Path global_path_;
  bool has_global_path_;
  mutable std::mutex global_path_mutex_;

  // 参数
  double detection_width_;       // 检测区域宽度 (m)
  double detection_length_;      // 检测区域长度 (m)
  double sector_radius_;         // 扇形半径 (m)
  double sector_angle_;          // 扇形角度 (度)
  int obstacle_threshold_;       // 障碍物阈值 (代价值)
  std::string detection_shape_;  // 检测形状 ("rectangle" 或 "sector")

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
