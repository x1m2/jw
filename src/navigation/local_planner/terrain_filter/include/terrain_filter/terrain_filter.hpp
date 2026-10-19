#ifndef TERRAIN_FILTER__TERRAIN_FILTER_HPP_
#define TERRAIN_FILTER__TERRAIN_FILTER_HPP_

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>
#include <cstdint>
#include <deque>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <map>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>
#include <tuple>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <unordered_map>
#include <vector>

#include "patchwork/pcl_ground_segmenter.hpp"

using PointType = pcl::PointXYZI;

enum class TerrainLabel : std::uint8_t {
  Unknown = 0,
  Ground = 1,
  Obstacle = 2,
  TraversableStair = 3,
};

class TerrainFilter {
 public:
  explicit TerrainFilter(const rclcpp::Node::SharedPtr& node);
  ~TerrainFilter();

 private:
  rclcpp::Node::SharedPtr node_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr radar_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      obstacle_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr stair_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      combined_cloud_pub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  std::shared_ptr<patchwork::PclGroundSegmenter> patchworkpp_;

  std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> lidar_queue_;
  std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> radar_queue_;
  std::deque<double> lidar_time_queue_;
  std::deque<double> radar_time_queue_;

  std::mutex lidar_mutex_;
  std::mutex radar_mutex_;

  std::string lidar_topic_;
  std::string radar_topic_;
  std::string base_frame_;
  std::string radar_frame_;

  bool use_dual_radar_;
  bool use_patchworkpp_;
  double refined_height_threshold_;
  double time_sync_threshold_;
  double voxel_leaf_size_;
  double height_threshold_;
  double robot_radius_;
  double max_range_;

  bool use_statistical_filter_;
  int statistical_filter_k_;
  double statistical_filter_std_;

  bool use_obstacle_clustering_;
  double cluster_tolerance_;
  int min_cluster_size_;
  int max_cluster_size_;

  bool use_stair_region_growing_;
  bool use_stair_riser_detection_;
  bool use_stair_tread_fallback_;
  double stair_grid_resolution_;
  double stair_max_step_height_;
  double stair_max_cell_height_span_;
  int stair_max_gap_cells_;
  int stair_hysteresis_frames_;
  int stair_hysteresis_neighbor_cells_;
  double stair_surface_tolerance_;
  double stair_min_step_height_;
  int stair_min_step_count_;
  double stair_min_total_rise_;
  double stair_min_tread_depth_;
  double stair_max_tread_depth_;
  double stair_step_height_tolerance_;
  double stair_model_tread_height_tolerance_;
  double stair_model_riser_position_tolerance_;
  int stair_model_max_extrapolated_steps_;
  double stair_direction_tolerance_deg_;
  double stair_min_width_;
  int stair_confirmation_frames_{2};
  double stair_confirmation_max_gap_{1.0};
  double stair_model_observation_timeout_{3.0};
  // 候选结构用固定坐标系追踪；同一帧的全局/局部重试不累计确认次数，
  // 并允许稀疏点云短时漏检后继续累计实际观测次数。
  struct StairConfirmation {
    Eigen::Vector3d anchor{Eigen::Vector3d::Zero()};
    Eigen::Vector3d direction{Eigen::Vector3d::UnitX()};
    Eigen::Vector3d up{Eigen::Vector3d::UnitZ()};
    double step_height{0.0};
    double tread_depth{0.0};
    int frames{1};
    double last_seen_stamp{0.0};
  };
  std::vector<StairConfirmation> stair_confirmation_tracks_;
  std::string stair_confirmation_frame_;
  double stair_confirmation_stamp_{0.0};
  int stair_ground_seed_min_points_;
  double stair_ground_seed_max_span_;
  double stair_ground_reference_max_abs_height_;
  std::unordered_map<std::uint64_t, int> stair_cell_hold_frames_;
  std::unordered_map<std::uint64_t, double> stair_cell_surface_heights_;
  std::unordered_map<std::uint64_t, double> stair_cell_height_spans_;
  bool stair_model_valid_{false};
  int stair_model_hold_frames_remaining_{0};
  double stair_model_observation_expires_at_{0.0};
  Eigen::Vector2d stair_model_direction_{Eigen::Vector2d::UnitX()};
  Eigen::Vector2d stair_model_width_direction_{Eigen::Vector2d::UnitY()};
  double stair_model_start_forward_{0.0};
  double stair_model_start_height_{0.0};
  double stair_model_step_height_{0.0};
  double stair_model_tread_depth_{0.0};
  double stair_model_min_width_{0.0};
  double stair_model_max_width_{0.0};
  int stair_model_last_level_{0};
  Eigen::Matrix4d current_cloud_to_base_{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d stair_model_cloud_to_base_{Eigen::Matrix4d::Identity()};
  bool current_cloud_to_base_valid_{false};
  std::string current_cloud_frame_;
  std::string stair_model_cloud_frame_;

  // 已确认表面保存在配准点云坐标系中，避免 base_link 运动导致历史错位。
  struct StairSurfaceSample {
    PointType point;
    double expires_at{0.0};
  };
  std::map<std::tuple<int, int, int>, StairSurfaceSample> stair_surface_history_;
  std::string stair_surface_history_frame_;
  double stair_surface_history_stamp_{0.0};
  double current_cloud_stamp_{0.0};
  double stair_surface_history_duration_{3.0};
  double stair_surface_history_distance_{0.04};

  std::size_t retainStairSurfaces(
      const pcl::PointCloud<PointType>::Ptr& cloud,
      const std::vector<TerrainLabel>& observed_labels,
      std::vector<TerrainLabel>& labels);

  pcl::VoxelGrid<PointType> voxel_filter_;
  pcl::StatisticalOutlierRemoval<PointType> stat_filter_;

  Eigen::Matrix4d radar_to_base_matrix_;
  bool radar_transform_ready_;

  void lidarCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void radarCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  void processData();
  void loadParameters();

  bool getTransform(const std::string& from_frame, const std::string& to_frame,
                    Eigen::Matrix4d& transform_matrix,
                    const rclcpp::Time& stamp = rclcpp::Time(0));

  pcl::PointCloud<PointType>::Ptr transformPointCloud(
      const pcl::PointCloud<PointType>::Ptr& cloud,
      const Eigen::Matrix4d& transform);

  pcl::PointCloud<PointType>::Ptr combinePointClouds(
      const pcl::PointCloud<PointType>::Ptr& lidar_cloud,
      const pcl::PointCloud<PointType>::Ptr& radar_cloud);

  pcl::PointCloud<PointType>::Ptr preprocessPointCloud(
      const pcl::PointCloud<PointType>::Ptr& input_cloud);

  pcl::PointCloud<PointType>::Ptr filterObstacleCloud(
      const pcl::PointCloud<PointType>::Ptr& obstacle_cloud);

  bool segmentGround(const pcl::PointCloud<PointType>::Ptr& input_cloud,
                     pcl::PointCloud<PointType>::Ptr& ground_cloud,
                     pcl::PointCloud<PointType>::Ptr& obstacle_cloud,
                     std::vector<TerrainLabel>* labels = nullptr);

  pcl::PointCloud<PointType>::Ptr detectTraversableStairs(
      const pcl::PointCloud<PointType>::Ptr& input_cloud,
      std::vector<TerrainLabel>& labels);

  void buildClassifiedClouds(
      const pcl::PointCloud<PointType>::Ptr& input_cloud,
      const std::vector<TerrainLabel>& labels,
      pcl::PointCloud<PointType>::Ptr& ground_cloud,
      pcl::PointCloud<PointType>::Ptr& obstacle_cloud,
      pcl::PointCloud<PointType>::Ptr& stair_cloud) const;

  void publishResults(const pcl::PointCloud<PointType>::Ptr& combined_cloud,
                      const pcl::PointCloud<PointType>::Ptr& ground_cloud,
                      const pcl::PointCloud<PointType>::Ptr& obstacle_cloud,
                      const pcl::PointCloud<PointType>::Ptr& stair_cloud,
                      const std_msgs::msg::Header& header);
};

#endif  // TERRAIN_FILTER__TERRAIN_FILTER_HPP_
