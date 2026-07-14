#ifndef LOCAL_PLANNER_COSTMAP_2D_HPP
#define LOCAL_PLANNER_COSTMAP_2D_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <utility>
#include <vector>

namespace local_planner {

// 代价值定义
static const unsigned char NO_INFORMATION = 255;
static const unsigned char LETHAL_OBSTACLE = 254;
static const unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
static const unsigned char FREE_SPACE = 0;

class CostMap2D {
 public:
  /**
   * @brief 构造函数
   * @param node ROS2节点
   */
  explicit CostMap2D(const rclcpp::Node::SharedPtr& node);

  /**
   * @brief 析构函数
   */
  ~CostMap2D();

  /**
   * @brief 初始化代价地图
   * @return 是否成功初始化
   */
  bool initialize();

  /**
   * @brief 更新代价地图
   */
  void updateCostMap();

  /**
   * @brief 发布代价地图
   */
  void publishCostMap();

 private:
  // ROS2相关
  rclcpp::Node::SharedPtr node_;

  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr
      livox_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;

  // TF相关
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster>
      static_tf_broadcaster_;

  // Livox 原始点云回调及内部二维投影
  void livoxCallback(
      const livox_ros_driver2::msg::CustomMsg::SharedPtr livox_msg);
  sensor_msgs::msg::LaserScan::SharedPtr livoxToLaserScan(
      const livox_ros_driver2::msg::CustomMsg& livox_msg) const;
  void processLaserScan(const sensor_msgs::msg::LaserScan& scan_msg);
  void publishLidarStaticTransform();

  // 参数
  std::string global_frame_;      // 全局坐标系
  std::string robot_base_frame_;  // 机器人基座坐标系
  std::string livox_topic_;       // Livox CustomMsg 话题名
  std::string lidar_frame_;       // 雷达坐标系

  // 雷达在 robot_base_frame 中的固定外参
  double lidar_x_;
  double lidar_y_;
  double lidar_z_;
  double lidar_roll_;
  double lidar_pitch_;
  double lidar_yaw_;
  Eigen::Affine3d lidar_to_base_transform_;

  // 三维点云裁剪及二维投影参数
  double min_height_;
  double max_height_;
  double angle_min_;
  double angle_max_;
  double angle_increment_;
  double range_min_;
  double range_max_;

  // 代价地图参数
  double resolution_;  // 分辨率 (m/pixel)
  int width_;          // 宽度 (像素)
  int height_;         // 高度 (像素)
  double origin_x_;    // 原点x坐标
  double origin_y_;    // 原点y坐标
  double origin_z_;    // 原点z坐标

  // 障碍物参数
  double max_obstacle_range_;  // 最大障碍物检测距离
  double min_obstacle_range_;  // 最小障碍物检测距离
  double inflation_radius_;    // 膨胀半径

  // 时间衰减参数
  double obstacle_max_age_;  // 障碍物最大存在时间 (秒)
  double decay_rate_;        // 衰减速率 (1/秒)
  bool use_time_decay_;      // 是否使用时间衰减

  // 俯仰角检测参数
  bool enable_pitch_detection_;  // 是否启用俯仰角检测
  double max_pitch_angle_;       // 最大俯仰角 (度)

  // 俯仰角状态管理
  bool pitch_exceeded_state_;            // 当前俯仰角超限状态
  rclcpp::Time last_normal_pitch_time_;  // 最后一次俯仰角正常的时间
  double pitch_recovery_time_;  // 俯仰角恢复正常的等待时间 (秒)

  // 代价地图数据
  std::vector<unsigned char> costmap_;
  std::vector<rclcpp::Time> obstacle_timestamps_;
  nav_msgs::msg::OccupancyGrid costmap_msg_;

  // 互斥锁
  std::mutex costmap_mutex_;

  /**
   * @brief 初始化代价地图数据
   */
  void initializeCostMap();

  /**
   * @brief 将激光数据转换为点云
   * @param scan 激光数据
   * @param cloud 输出点云
   */
  void scanToPointCloud(const sensor_msgs::msg::LaserScan& scan,
                        pcl::PointCloud<pcl::PointXYZ>& cloud);

  /**
   * @brief 将点云转换到地图坐标系
   * @param cloud_in 输入点云
   * @param cloud_out 输出点云
   * @param target_frame 目标坐标系
   * @param stamp 时间戳
   */
  bool transformPointCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud_in,
                           pcl::PointCloud<pcl::PointXYZ>& cloud_out,
                           const std::string& target_frame,
                           const rclcpp::Time& stamp);

  /**
   * @brief 清除代价地图
   */
  void clearCostMap();

  /**
   * @brief 使用时间衰减机制清理过时障碍物
   */
  void decayObstacles();

  /**
   * @brief 射线追踪清理自由空间
   */
  void raytraceFreespace(const sensor_msgs::msg::LaserScan& scan,
                         double robot_x, double robot_y);

  /**
   * @brief 清理射线路径上的障碍物
   */
  void clearRayPath(const std::vector<std::pair<int, int>>& ray_cells,
                    bool obstacle_detected);

  /**
   * @brief Bresenham直线算法
   */
  void bresenhamLine(int x0, int y0, int x1, int y1,
                     std::vector<std::pair<int, int>>& cells);

  /**
   * @brief 在代价地图上标记障碍物
   */
  void markObstacles(const pcl::PointCloud<pcl::PointXYZ>& cloud);

  /**
   * @brief 膨胀障碍物
   */
  void inflateObstacles();

  /**
   * @brief 世界坐标转换为地图坐标
   */
  bool worldToMap(double wx, double wy, int& mx, int& my);

  /**
   * @brief 地图坐标转换为世界坐标
   */
  void mapToWorld(int mx, int my, double& wx, double& wy);

  /**
   * @brief 设置代价值
   */
  void setCost(int mx, int my, unsigned char cost);

  /**
   * @brief 获取代价值
   */
  unsigned char getCost(int mx, int my);

  /**
   * @brief 获取机器人当前位置
   */
  bool getRobotPose(double& x, double& y, double& z, double& yaw);

  /**
   * @brief 更新代价地图原点位置
   */
  void updateMapOrigin();

  /**
   * @brief 滚动地图数据
   */
  void rollMap(int offset_x, int offset_y);

  /**
   * @brief 计算两点间的距离
   */
  double distance(double x1, double y1, double x2, double y2);

  /**
   * @brief 检查机器人俯仰角是否超过阈值
   */
  bool isPitchAngleExceeded();

  /**
   * @brief 加载参数
   */
  void loadParameters();
};

}  // namespace local_planner

#endif  // LOCAL_PLANNER_COSTMAP_2D_HPP
