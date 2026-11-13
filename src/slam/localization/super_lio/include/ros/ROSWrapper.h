
#ifndef ROSWRAPPER_HPP_
#define ROSWRAPPER_HPP_

#include <map>
#include <tuple>
#include <deque>
#include <chrono>
#include <vector>
#include <execution>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/callback_group.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <slam_manager_interfaces/msg/localization_pose.hpp>
#include "common/localization_geometry.h"
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include "lio/params.h"
#include "basic/alias.h"
#include "basic/logs.h"
#include "basic/Manifold.h"
#include "common/ds.h"

#include "lio/ESKF.h"
#include "OctVoxMap/OctVoxMap.hpp"

#include "backend/backend/Backend.h"

namespace LI2Sup{

void LoadParamFromRos(rclcpp::Node& node);

void livox2pcl(const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg, BASIC::CloudPtr& point_cloud);

class ROSWrapper : public rclcpp::Node {
public:
  explicit ROSWrapper(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~ROSWrapper(){};
  using Ptr = std::shared_ptr<ROSWrapper>;
  bool sync_measure(MeasureGroup&);

  void setESKF(ESKF::Ptr& eskf) { eskf_ = eskf;}

  void clear(){
    lidar_buffer_.clear();
    imu_buffer_.clear();
    lidar_pushed_ = false;
    last_timestamp_imu_ = -1.0;
    last_timestamp_lidar_ = -1.0;
  }

  void pub_odom(const NavState&, double match_score = 0.0);
  void publish_status(bool valid);
  void publish_imu_localization(const DynamicState &state);
  void pub_cloud_body(const BASIC::CloudPtr &pc, double time);
  Backend &backend() { return backend_; }
  bool initial_pose_requested() const { return manual_pose_requested_; }
  void consume_initial_pose_request() { manual_pose_requested_ = false; }

  void pub_cloud_world(const BASIC::CloudPtr& pc, double time);

  void pub_processing_time(double time, double current_time, double mean_time, double std_time);

  void set_global_map(const BASIC::CloudPtr& global_map);

  void set_initial_data(bool& flg_get_init_guess, bool flg_finish_init = false);

  /// Optional GNSS prior feed for relocalization (relocalization.use_gnss).
  void set_gnss_data();

  void load_backend_param(Backend& backend);

  rclcpp::CallbackGroup::SharedPtr getSensorCallbackGroup() {
    return cb_sensor_;
  }

private:
  void imuHandler(const sensor_msgs::msg::Imu::SharedPtr msg);
  void livoxHandler(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg);
  void stdMsgHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  void setupParams();
  void setupIO();

private:
  void setup_localization_interface();
  bool sensors_fresh() const;
  Eigen::Matrix4d base_pose(const Eigen::Matrix4d &map_imu) const;
  void publish_base_tf(const Eigen::Matrix4d &pose, double time);
  Backend backend_;
  std::string map_frame_, lidar_frame_, baselink_frame_;
  Eigen::Matrix4d T_base_lidar_ = Eigen::Matrix4d::Identity();
  bool path_en_ = true, lidar_tf_broadcast_ = true, imu_tf_broadcast_ = false;
  bool manual_pose_requested_ = false;
  std::atomic<bool> localization_valid_{false};
  double sensor_timeout_sec_ = 1.0;
  std::chrono::steady_clock::time_point last_imu_received_{}, last_lidar_received_{};
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reloc_status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_pub_;
  rclcpp::Publisher<slam_manager_interfaces::msg::LocalizationPose>::SharedPtr app_pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr imu_localization_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_body_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr processing_time_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_sub_;
  rclcpp::TimerBase::SharedPtr global_map_timer_, status_timer_;
  rclcpp::CallbackGroup::SharedPtr cb_sensor_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_lidar_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_std_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_gnss_;

  /// relocalization
  bool use_gnss_ = false;
  std::string gnss_topic_ = "/gnss/odom";
  bool print_pose_ = false;   /// publish.print_pose: echo the world pose to the terminal

  std::deque<IMUData>   imu_buffer_;
  std::deque<LidarData> lidar_buffer_;
  bool lidar_pushed_ = false;
  double last_timestamp_imu_ = -1.0;
  double last_timestamp_lidar_ = -1.0;

  ESKF::Ptr eskf_{nullptr};

  nav_msgs::msg::Path path_;
  geometry_msgs::msg::PoseStamped msg2uav_;
  sensor_msgs::msg::PointCloud2 global_map_msg_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  BASIC::V3 last_path_point_ = BASIC::V3(0, 0, -100);

/// output.
private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;       /// lidar fre --> IMU frame
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_lidar_localization_; /// world -> base_link (reference-compatible)
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_imu_odom_;   /// IMU fre   --> IMU frame
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_robo_odom_;  /// IMU fre   --> Robot frame
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_world_;

};

} // namespace END.

#endif
