#ifndef LOCAL_PLANNER_COSTMAP_2D_HPP
#define LOCAL_PLANNER_COSTMAP_2D_HPP

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <memory>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

namespace local_planner {

// 代价值定义
static const unsigned char LETHAL_OBSTACLE = 254;
static const unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
static const unsigned char FREE_SPACE = 0;

class CostMap2D {
 public:
  explicit CostMap2D(const rclcpp::Node::SharedPtr& node);

  ~CostMap2D() = default;

  /**
   * @brief 初始化代价地图
   * @return 是否成功初始化
   */
  bool initialize();

 private:
  struct InflationOffset {
    int dx;
    int dy;
    unsigned char cost;
  };

  // 三维点云裁剪并投影为二维射线时使用的参数集合。
  struct PointCloudProjectionConfig {
    double min_height;
    double max_height;
    double angle_min;
    double angle_max;
    double angle_increment;
    double range_min;
    double range_max;
  };

  // ROS2相关
  rclcpp::Node::SharedPtr node_;

  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr
      livox_custom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      pointcloud2_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;

  // TF相关
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Livox 原始点云回调及内部二维投影
  void livoxCallback(
      const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr livox_msg);
  void pointCloud2Callback(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud_msg);
  sensor_msgs::msg::LaserScan::SharedPtr livoxToLaserScan(
      const livox_ros_driver2::msg::CustomMsg& livox_msg) const;
  sensor_msgs::msg::LaserScan::SharedPtr pointCloud2ToLaserScan(
      const sensor_msgs::msg::PointCloud2& pointcloud_msg) const;
  sensor_msgs::msg::LaserScan::SharedPtr makeEmptyLaserScan(
      const builtin_interfaces::msg::Time& stamp) const;
  void projectLidarPoint(double x, double y, double z,
                         sensor_msgs::msg::LaserScan& scan) const;
  void processLaserScan(const sensor_msgs::msg::LaserScan& scan_msg);

  // 参数
  std::string global_frame_;
  std::string robot_base_frame_;  // 机器人基座坐标系
  std::string livox_topic_;
  std::string lidar_frame_;
  int lidar_type_;  // 1: Livox CustomMsg，其他: PointCloud2
  // PointCloud2 已经在 robot_base_frame 下时跳过固定雷达外参。
  bool pointcloud2_in_base_frame_;
  // 语义障碍点云已完成地面剔除时，可跳过重复的绝对Z高度裁剪。
  bool pointcloud2_apply_height_filter_;
  // 点云投影后的无穷距离仅表示该角度本帧没有点，不代表传感器确认了
  // 一条完整的自由射线。语义障碍点云必须关闭这种清除方式。
  bool clear_unobserved_rays_;

  // 雷达在 robot_base_frame 中的固定外参
  Eigen::Affine3d lidar_to_base_transform_;

  // 三维点云裁剪及二维投影参数
  PointCloudProjectionConfig projection_config_;

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
  // 滚动地图复用缓冲区，避免机器人每移动一个栅格都重新申请整图内存。
  std::vector<unsigned char> costmap_scratch_;
  std::vector<rclcpp::Time> timestamp_scratch_;
  nav_msgs::msg::OccupancyGrid costmap_msg_;
  std::vector<float> ray_cosines_;
  std::vector<float> ray_sines_;
  std::vector<InflationOffset> inflation_offsets_;

  /**
   * @brief 初始化代价地图数据
   */
  void initializeCostMap();

  /**
   * @brief 预计算固定射线方向和膨胀模板
   */
  void initializeLookupTables();

  /**
   * @brief 清除代价地图
   */
  void clearCostMap();

  /**
   * @brief 删除上一帧的膨胀代价，保留致命障碍后重新膨胀
   */
  void clearInflatedCosts();

  /**
   * @brief 使用时间衰减机制清理过时障碍物
   */
  void decayObstacles();

  /**
   * @brief 射线追踪清理自由空间
   */
  void raytraceFreespace(const sensor_msgs::msg::LaserScan& scan,
                         const tf2::Transform& map_from_base);

  /**
   * @brief 清理射线路径上的障碍物
   */
  void clearRayPath(int x0, int y0, int x1, int y1, bool obstacle_detected,
                    const rclcpp::Time& frame_time);

  /**
   * @brief 在代价地图上标记障碍物
   */
  void markObstacles(const sensor_msgs::msg::LaserScan& scan,
                     const tf2::Transform& map_from_base);

  /**
   * @brief 膨胀障碍物
   */
  void inflateObstacles();

  /**
   * @brief 世界坐标转换为地图坐标
   */
  bool worldToMap(double wx, double wy, int& mx, int& my);

  /**
   * @brief 更新代价地图原点位置
   */
  void updateMapOrigin(
      const geometry_msgs::msg::TransformStamped& robot_transform);

  /**
   * @brief 滚动地图数据
   */
  void rollMap(int offset_x, int offset_y);

  /**
   * @brief 发布代价地图
   */
  void publishCostMap();

  /**
   * @brief 检查机器人俯仰角是否超过阈值
   */
  bool isPitchAngleExceeded(
      const geometry_msgs::msg::TransformStamped& robot_transform);

  /**
   * @brief 加载参数
   */
  void loadParameters();
};

}  // namespace local_planner

#endif  // LOCAL_PLANNER_COSTMAP_2D_HPP
