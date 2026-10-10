#pragma once

#include <atomic>
#include <cstdio>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <slam_manager_interfaces/msg/localization_pose.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "system/DataDef.h"

class System;

class FastlioLocalizationNode : public rclcpp::Node {
 public:
  explicit FastlioLocalizationNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~FastlioLocalizationNode() override;

  void initialize();
  void requestStop();

 private:
  void publish_cloud(
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &publisher,
      PointCloudType::Ptr cloud, const double &lidar_end_time,
      const std::string &frame_id);
  void publish_cloud_world(
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &publisher,
      PointCloudType::Ptr cloud, const state_ikfom &state,
      const double &lidar_end_time);
  void publish_cloud_body(
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &publisher,
      PointCloudType::Ptr cloud, const double &lidar_end_time);
  template <typename T>
  void set_posestamp(T &out, const QD &rotation, const V3D &position);
  void publish_tf(const QD &rotation, const V3D &position,
                 const double &lidar_end_time);
  void compute_baselink_pose(const state_ikfom &state,
                             QD &baselink_rotation,
                             V3D &baselink_position) const;
  void publish_geometry_msgs(
      rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr &publisher,
      const state_ikfom &state, const double &lidar_end_time, QD &baselink_rotation,
      V3D &baselink_position);
  void publish_odometry(
      rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr &publisher,
      const state_ikfom &state, const double &lidar_end_time, QD &baselink_rotation,
      V3D &baselink_position);
  void publish_module_status(bool valid);
  void publish_imu_odometry(
      rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr &publisher,
      const state_ikfom &state, const double &imu_time);
  void publish_imu_path(
      rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &publisher,
      const QD &rotation, const V3D &position,
      const double &lidar_end_time);

  void gnss_cbk(const sensor_msgs::msg::NavSatFix::SharedPtr message);
  bool load_last_pose(const PointCloudType::Ptr &scan);
  void sensor_data_process();
  void standard_pcl_cbk(
      const sensor_msgs::msg::PointCloud2::SharedPtr message);
  void livox_pcl_cbk(
      const livox_ros_driver2::msg::CustomMsg::SharedPtr message);
  void imu_cbk(const sensor_msgs::msg::Imu::SharedPtr message);
  void publish_global_map();
  void initialPoseCallback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message);

  int lidar_type{0};
  std::unique_ptr<System> slam_owner;
  System &slam;
  std::string map_frame;
  std::string lidar_frame;
  std::string baselink_frame;
  FILE *last_pose_record{nullptr};

  std::vector<double> lidar_to_base_T;
  std::vector<double> lidar_to_base_R;
  Eigen::Matrix4d T_base_lidar{Eigen::Matrix4d::Identity()};
  bool path_en{true};
  bool scan_pub_en{false};
  bool dense_pub_en{false};
  bool lidar_tf_broadcast{false};
  bool imu_tf_broadcast{false};
  bool use_gnss{false};
  std::atomic_bool stopping{false};

  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr
      sub_pcl1;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      sub_pcl2;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr
      sub_gnss;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      sub_initpose;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pubLaserCloudFull;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pubLaserCloudBody;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr
      pubLidarOdom;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr
      pubImuOdom;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubImuPath;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pubrelocalizationDebug;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pubGlobalMap;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
      pubOccGrid;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
      reloc_status_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_robot_pose;
  rclcpp::Publisher<slam_manager_interfaces::msg::LocalizationPose>::SharedPtr
      pubAppLocalizationPose;
  rclcpp::TimerBase::SharedPtr timer;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
};
