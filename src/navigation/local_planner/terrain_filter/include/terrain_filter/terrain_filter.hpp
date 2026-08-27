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
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <unordered_map>

#include "patchwork/pcl_ground_segmenter.hpp"

using PointType = pcl::PointXYZI;

class TerrainFilter {
 public:
  explicit TerrainFilter(const rclcpp::Node::SharedPtr& node);
  ~TerrainFilter();

 private:
  rclcpp::Node::SharedPtr node_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr radar_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      obstacle_cloud_pub_;
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
  std::string odom_topic_;
  std::string base_frame_;
  std::string lidar_frame_;
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

  bool use_ground_expansion_;
  double expansion_radius_;
  double expansion_height_;
  bool use_stair_region_growing_;
  double stair_grid_resolution_;
  double stair_max_step_height_;
  double stair_max_cell_height_span_;
  int stair_max_gap_cells_;
  int stair_hysteresis_frames_;
  int stair_hysteresis_neighbor_cells_;
  std::unordered_map<std::uint64_t, int> stair_cell_hold_frames_;

  pcl::VoxelGrid<PointType> voxel_filter_;
  pcl::StatisticalOutlierRemoval<PointType> stat_filter_;

  Eigen::Matrix4d lidar_to_base_matrix_;
  Eigen::Matrix4d radar_to_base_matrix_;
  bool lidar_transform_ready_;
  bool radar_transform_ready_;

  void lidarCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void radarCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void odomCallback(nav_msgs::msg::Odometry::ConstSharedPtr msg);

  void processData();
  void loadParameters();

  bool getTransform(const std::string& from_frame, const std::string& to_frame,
                    Eigen::Matrix4d& transform_matrix);

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
                     pcl::PointCloud<PointType>::Ptr& obstacle_cloud);

  void expandGroundCloud(const pcl::PointCloud<PointType>::Ptr& original_cloud,
                         pcl::PointCloud<PointType>::Ptr& ground_cloud,
                         pcl::PointCloud<PointType>::Ptr& obstacle_cloud);

  void publishResults(const pcl::PointCloud<PointType>::Ptr& combined_cloud,
                      const pcl::PointCloud<PointType>::Ptr& ground_cloud,
                      const pcl::PointCloud<PointType>::Ptr& obstacle_cloud,
                      const std_msgs::msg::Header& header);
};

#endif  // TERRAIN_FILTER__TERRAIN_FILTER_HPP_
