#pragma once

#include <Eigen/Geometry>
#include <cstddef>
#include <cstdint>
#include <geometry_msgs/msg/twist.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

namespace obstacle_stop {

// 将MID360点云投影到base_link水平面，并作为所有导航速度的最终停障门。
class ObstacleStop {
 public:
  explicit ObstacleStop(rclcpp::Node::SharedPtr node);

  bool initialize();
  void stop();

 private:
  // 节点当前的安全状态。
  enum class State {
    kWaitingForCommand,
    kWaitingForLidar,
    kClear,
    kObstacleSlowdown,
    kObstacleStop,
    kEmergencyStop,
    kSensorFault,
  };

  struct ObstacleStopConfig {
    // 点云过滤参数。
    double min_height{0.2};
    double max_height{1.0};
    double angle_min{-3.141592653589793};
    double angle_max{3.141592653589793};
    double range_min{0.4};
    double range_max{8.0};

    // 内部栅格参数。
    double grid_resolution{0.05};
    int grid_width{80};
    int grid_height{50};
    double inflation_radius{0.10};

    // 矩形检测和减速参数。
    double robot_width{0.80};
    double robot_length{1.10};
    double safety_margin{0.10};
    double hard_stop_distance{0.40};
    double slowdown_distance{0.60};
    double obstacle_ratio_threshold{0.10};
    double linear_velocity_factor{0.40};
    double angular_velocity_factor{1.0};
    double hard_stop_clear_delay{0.30};
    double slowdown_clear_delay{0.50};

    // 数据时效和调试参数。
    double lidar_timeout_sec{0.30};
    double command_timeout_sec{0.30};
    double control_frequency{30.0};
    int minimum_filtered_points{10};
    bool publish_visualization{true};
  };

  struct GridCell {
    int x{0};
    int y{0};
  };

  void loadParameters();
  void initializeGridLookup();
  void lidarCallback(
      const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr message);
  void plannedVelocityCallback(
      const geometry_msgs::msg::Twist::ConstSharedPtr message);
  void emergencyStopCallback(const std_msgs::msg::Bool::ConstSharedPtr message);
  void obstacleStopEnabledCallback(
      const std_msgs::msg::Bool::ConstSharedPtr message);
  // 周期检查传感器、指令和障碍物状态，发布最终安全速度。
  void controlCallback();

  // 过滤MID360点云并直接构建临时局部障碍栅格。
  std::size_t buildObstacleGrid(
      const livox_ros_driver2::msg::CustomMsg &message,
      std::vector<uint8_t> &grid, std::vector<GridCell> &occupied_cells) const;
  void inflateGrid(std::vector<uint8_t> &grid,
                   const std::vector<GridCell> &occupied_cells) const;
  std::vector<std::size_t> buildRectangleCellIndices(double x_min, double x_max,
                                                     double y_min,
                                                     double y_max) const;
  bool obstacleInCells(const std::vector<uint8_t> &grid,
                       const std::vector<std::size_t> &cells) const;
  bool localToGrid(double x, double y, int &mx, int &my) const;
  void updateObstacleState(bool raw_hard_stop, bool raw_slowdown,
                           const rclcpp::Time &now);
  void publishDetectionMarkers(bool hard_stop, bool slowdown);
  void publishZeroVelocity();
  void logStateChange(State state, const std::string &reason);
  static const char *stateName(State state);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr planned_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
      obstacle_stop_enabled_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr safe_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_active_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  std::string lidar_topic_;
  std::string planned_cmd_topic_;
  std::string safe_cmd_topic_;
  std::string base_frame_;
  // MID360坐标系到机器人base_link坐标系的固定外参。
  Eigen::Isometry3d lidar_to_base_{Eigen::Isometry3d::Identity()};
  ObstacleStopConfig config_;
  std::vector<GridCell> inflation_offsets_;
  std::vector<std::size_t> hard_stop_cells_;
  std::vector<std::size_t> slowdown_cells_;

  // 回调与控制定时器可能并发访问以下状态。
  mutable std::mutex mutex_;
  geometry_msgs::msg::Twist planned_cmd_;
  rclcpp::Time last_lidar_time_;
  rclcpp::Time last_command_time_;
  rclcpp::Time hard_stop_clear_deadline_;
  rclcpp::Time slowdown_clear_deadline_;
  bool has_lidar_{false};
  bool lidar_valid_{false};
  bool has_command_{false};
  bool hard_stop_active_{false};
  bool slowdown_active_{false};
  bool emergency_stop_active_{false};
  bool obstacle_stop_enabled_{true};
  State last_state_{State::kWaitingForCommand};
  bool has_logged_state_{false};
};

}  // namespace obstacle_stop
