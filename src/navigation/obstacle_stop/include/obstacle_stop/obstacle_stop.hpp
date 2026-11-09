#pragma once

#include <Eigen/Geometry>
#include <array>
#include <cstddef>
#include <cstdint>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "imu_stair_detector/imu_stair_detector.hpp"
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>
#include <utility>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "obstacle_stop/directional_obstacle_policy.hpp"
#include "obstacle_stop/height_cell.hpp"
#include "obstacle_stop/detection_zone.hpp"

namespace obstacle_stop {

// 将原始雷达点云变换到base_link，并作为所有导航速度的最终停障门。
class ObstacleStop {
 public:
  explicit ObstacleStop(rclcpp::Node::SharedPtr node);

  bool initialize(std::string &message);
  void stop();

 private:
  static constexpr char kBaseFrame[] = "base_link";
  static constexpr uint8_t kFree = 0U;
  static constexpr uint8_t kOccupied = 1U;
  static constexpr std::array<const char *, 4> kZoneNames{
      "front", "rear", "left", "right"};

  // 节点当前的安全状态。
  enum class State {
    kWaitingForCommand,
    kWaitingForLidar,
    kClear,
    kObstacleSlowdown,
    kObstacleStop,
    kPostureStop,
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
    std::size_t min_points_per_cell{1U};
    int grid_width{80};
    int grid_height{50};

    // 矩形检测和减速参数。
    double robot_width{0.80};
    double robot_length{1.10};
    double linear_velocity_factor{0.40};
    double angular_velocity_factor{1.0};
    double hard_stop_clear_delay{0.30};
    double slowdown_clear_delay{0.50};
    bool directional_release_enabled{false};
    double directional_center_half_width{0.20};
    double directional_escape_velocity_threshold{0.05};
    bool center_lateral_escape_enabled{false};
    double center_escape_max_forward_velocity{0.02};
    double center_escape_max_angular_velocity{0.10};

    // 数据时效和调试参数。
    double lidar_timeout_sec{0.30};
    double command_timeout_sec{0.30};
    double control_frequency{30.0};
    bool publish_visualization{true};
    std::array<DetectionZoneConfig, 4> zones;
  };

  struct SlowZoneRuntime {
    ZoneRectangle box;
    std::vector<std::size_t> indices, occupied;
    bool active{false};
  };

  struct ZoneRuntime {
    ZoneRectangle hard_box, display_box;
    std::vector<std::size_t> hard_indices, hard_occupied;
    bool hard_active{false};
    std::optional<SlowZoneRuntime> slow;  // constructed only for front/rear
  };

  struct RegionProfile {
    int id{0};
    bool enabled{true};
    std::vector<std::array<double, 2>> polygon;
    ObstacleStopConfig config;
    bool has_custom_config{false};
  };

  struct ZoneVisualization {
    ZoneRectangle hard_box;
    std::optional<ZoneRectangle> slow_box;
    std::size_t hard_count{0U}, slow_count{0U};
    bool hard_active{false}, slow_active{false};
  };

  struct VisualizationSnapshot {
    ObstacleStopConfig config;
    std::array<ZoneVisualization, 4> zones;
    std::vector<std::pair<std::size_t, HeightCell>> cells;
  };

  // 主流程：配置、地图区域切换与停障控制（obstacle_stop.cpp）。
  void loadParameters();
  bool loadRegionProfiles(std::string &message);
  bool initializeImuStairDetection(std::string &message);
  void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr message);
  void localizationPoseCallback(
      const geometry_msgs::msg::PoseStamped::ConstSharedPtr message);
  const RegionProfile *findRegion(double x, double y) const;
  void applyRegionProfile(const RegionProfile *region);
  void plannedVelocityCallback(
      const geometry_msgs::msg::Twist::ConstSharedPtr message);
  void emergencyStopCallback(const std_msgs::msg::Bool::ConstSharedPtr message);
  void postureStopCallback(const std_msgs::msg::Bool::ConstSharedPtr message);
  void obstacleStopEnabledCallback(
      const std_msgs::msg::Bool::ConstSharedPtr message);
  void updateZoneState();
  void updateObstacleState(bool raw_hard_stop, bool raw_slowdown,
                           bool longitudinal_hard_stop, const rclcpp::Time &now);
  // 周期检查传感器、指令和障碍物状态，发布最终安全速度。
  void controlCallback();
  void publishZeroVelocity();
  void logStateChange(State state, const std::string &reason);
  static const char *stateName(State state);

  // 点云变换、过滤与栅格占用统计（obstacle_stop_detection.cpp）。
  bool sourceRequired(unsigned source) const;
  void initializeGridLookup();
  void lidarCallback(
      const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr message);
  void pointCloud2Callback(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr message);
  void rearPointCloud2Callback(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr message);
  void processPointCloud2(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr &message,
      bool rear_lidar);
  void processObstacleGrid(std::vector<HeightCell> &grid, bool input_valid,
                           bool rear_lidar = false);
  void refreshZoneOccupancy(const std::array<bool, 2> &usable);
  void buildObstacleGrid(const livox_ros_driver2::msg::CustomMsg &message,
                         std::vector<HeightCell> &grid,
                         std::vector<Eigen::Vector3f> *base_points) const;
  bool buildObstacleGrid(const sensor_msgs::msg::PointCloud2 &message,
                         std::vector<HeightCell> &grid,
                         const Eigen::Isometry3d &lidar_to_base,
                         std::vector<Eigen::Vector3f> *base_points) const;
  bool passesDetectionFilters(double x, double y, double z) const;
  void addBasePointToGrid(double x, double y, double z,
                          std::vector<HeightCell> &grid) const;
  bool localToGrid(double x, double y, int &mx, int &my) const;

  // 调试点云与RViz显示；快照构造后在控制锁外发布Marker和栅格。
  // 点云发布由雷达回调调用，沿用调用方持有的配置锁。
  void initializeVisualization();
  void publishUnfilteredBaseCloud(const std::vector<Eigen::Vector3f> &points,
                                  const std_msgs::msg::Header &header, bool rear_lidar);
  void publishBaseCloud(const std::vector<Eigen::Vector3f> &points,
                        const std_msgs::msg::Header &header,
                        bool rear_lidar);
  void visualizationCallback();
  void publishDetectionMarkers(const VisualizationSnapshot &snapshot);
  void publishOccupiedCells(const VisualizationSnapshot &snapshot);
  void publishOccupiedColumns(const VisualizationSnapshot &snapshot);

  // IMU detector is independent of map-region overrides; guarded by mutex_.
  bool imu_stairs_enabled_{true};
  std::string imu_frame_;
  imu_stair_detector::ImuStairDetector imu_stair_detector_;
  bool last_stairs_active_{false};
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stairs_active_pub_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      pointcloud2_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      rear_pointcloud2_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr planned_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr posture_stop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
      obstacle_stop_enabled_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      localization_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr safe_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_active_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obstacle_stopped_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      occupied_columns_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      occupied_cells_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      front_base_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      rear_base_cloud_pub_;
  std::array<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr, 2>
      unfiltered_base_cloud_pubs_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::CallbackGroup::SharedPtr visualization_callback_group_;
  rclcpp::TimerBase::SharedPtr visualization_timer_;

  std::string lidar_topic_;
  std::string rear_lidar_topic_;
  std::string lidar_frame_;
  std::string rear_lidar_frame_;
  std::string planned_cmd_topic_;
  std::string safe_cmd_topic_;
  std::string global_frame_;
  std::string pose_topic_;
  std::string topology_map_file_;
  int lidar_type_{1};  // 1: Livox CustomMsg，其他: PointCloud2
  bool front_lidar_enabled_{true};
  bool rear_lidar_enabled_{true};
  // 雷达坐标系到机器人base_link坐标系的固定外参。
  Eigen::Isometry3d lidar_to_base_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d rear_lidar_to_base_{Eigen::Isometry3d::Identity()};
  ObstacleStopConfig config_;
  ObstacleStopConfig default_config_;
  std::vector<RegionProfile> region_profiles_;
  std::optional<int> active_region_id_;
  mutable std::mutex config_mutex_;
  std::array<ZoneRuntime, 4> zone_runtime_;
  std::array<std::vector<uint8_t>, 2> detection_masks_;
  std::vector<HeightCell> front_height_grid_, rear_height_grid_;
  std::vector<HeightCell> fused_height_grid_;
  bool occupancy_dirty_{true};
  std::array<bool, 2> cached_usable_{};
  std::vector<HeightCell> visual_height_grid_;
  std::vector<std::size_t> visual_occupied_;

  // 回调与控制定时器可能并发访问以下状态。
  mutable std::mutex mutex_;
  geometry_msgs::msg::Twist planned_cmd_;
  rclcpp::Time last_lidar_time_;
  rclcpp::Time rear_last_lidar_time_;
  rclcpp::Time last_command_time_;
  rclcpp::Time hard_stop_clear_deadline_;
  rclcpp::Time slowdown_clear_deadline_;
  bool has_lidar_{false};
  bool lidar_valid_{false};
  bool rear_has_lidar_{false};
  bool rear_lidar_valid_{false};
  bool has_command_{false};
  bool hard_stop_active_{false};
  bool slowdown_active_{false};
  bool directional_escape_release_active_{false};
  bool emergency_stop_active_{false};
  bool posture_stop_active_{false};
  bool obstacle_stop_enabled_{true};
  State last_state_{State::kWaitingForCommand};
  bool has_logged_state_{false};
};

}  // namespace obstacle_stop
