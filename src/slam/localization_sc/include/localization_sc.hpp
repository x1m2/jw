#ifndef LOCALIZATION_SC_MAIN_H
#define LOCALIZATION_SC_MAIN_H

#include <angles/angles.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <quatro/quatro_module.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <memory>
#include <mutex>
#include <nano_gicp/nano_gicp.hpp>
#include <nano_gicp/point_type_nano_gicp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>
#include <utility>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

#include "map_matcher.hpp"
#include "pose_pcd.hpp"
#include "utilities.hpp"
using namespace std::chrono;
typedef message_filters::sync_policies::ApproximateTime<
    nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2>
    odom_pcd_sync_pol;

class FastLioLocalizationScQn : public rclcpp::Node {
 private:
  enum class LocalizationState { IDLE, GLOBAL_RELOCALIZING, LOCAL_TRACKING };

  std::string map_frame_;
  std::mutex keyframes_mutex_, vis_mutex_;
  bool is_initialized_ = false;
  int current_keyframe_idx_ = 0;
  PosePcd last_keyframe_;
  std::vector<PosePcdReduced> saved_map_from_bag_;
  Eigen::Matrix4d last_corrected_TF_ = Eigen::Matrix4d::Identity();
  double keyframe_dist_thr_;
  double voxel_res_;
  bool saved_map_vis_switch_ = true;
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  nav_msgs::msg::Path raw_odom_path_, corrected_odom_path_;
  std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> matched_pairs_xyz_;
  pcl::PointCloud<pcl::PointXYZ> raw_odoms_, corrected_odoms_;
  pcl::PointCloud<PointType> saved_map_pcd_;

  // ROS 2 publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      corrected_odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr corrected_path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      corrected_current_pcd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      realtime_pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr map_match_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr saved_map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_src_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_dst_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      debug_coarse_aligned_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      debug_fine_aligned_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr map_to_odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr localization_odom_pub_;
  rclcpp::TimerBase::SharedPtr match_timer_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reloc_success_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      sub_initial_pose_;

  // Message filters
  std::shared_ptr<message_filters::Synchronizer<odom_pcd_sync_pol>>
      sub_odom_pcd_sync_;
  std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>>
      sub_odom_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>
      sub_pcd_;

  // Map matcher
  std::shared_ptr<MapMatcher> map_matcher_;
  bool global_reloc_done_ = false;
  bool need_apply_initial_pose_ = false;
  int last_matched_keyframe_idx_ = -1;
  // -----------------------
  // reloc stable check
  // -----------------------
  int reloc_stable_required_ = 3;
  double reloc_trans_diff_threshold_ = 0.15;
  double reloc_yaw_diff_threshold_deg_ = 5.0;
  bool reloc_use_xy_yaw_only_ = true;
  bool reloc_use_initial_pose_ = true;

  int reloc_stable_count_ = 0;
  bool has_last_reloc_candidate_ = false;
  Eigen::Matrix4d last_reloc_candidate_tf_ = Eigen::Matrix4d::Identity();

  Eigen::Matrix4d initial_pose_eig_ = Eigen::Matrix4d::Identity();

  LocalizationState localization_state_ = LocalizationState::IDLE;

 public:
  explicit FastLioLocalizationScQn(const std::string &node_name);

 private:
  geometry_msgs::msg::Transform poseEigToROSTf2(const Eigen::Matrix4d &pose);

  void updateOdomsAndPaths(const PosePcd &pose_pcd_in);
  bool checkIfKeyframe(const PosePcd &pose_pcd_in,
                       const PosePcd &latest_pose_pcd);
  visualization_msgs::msg::Marker getMatchMarker(
      const std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>>
          &match_xyz_pairs);
  void loadMap(const std::string &saved_map_path);
  void publishMapToOdom(const rclcpp::Time &stamp);
  void odomPcdCallback(
      const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg,
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pcd_msg);
  void matchingTimerFunc();

  void initialPoseCallback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  std::vector<std::pair<int, double>> findCandidateKeyframesByPose(
      const Eigen::Matrix4d &query_pose,
      const std::vector<PosePcdReduced> &saved_keyframes);
  bool isRelocalizationCandidateStable(const Eigen::Matrix4d &last_tf,
                                       const Eigen::Matrix4d &new_tf);
  void handleGlobalRelocalization(const PosePcd &last_keyframe_copy);
  void handleLocalTracking(const PosePcd &last_keyframe_copy);

  int findNearestKeyframeByPose(
      const Eigen::Matrix4d &query_pose,
      const std::vector<PosePcdReduced> &saved_keyframes);

  void publishMatchedResult(const PosePcd &last_keyframe_copy,
                            const Eigen::Matrix4d &tfed_pose);
  void resetRelocStableState();
};

#endif  // LOCALIZATION_SC_MAIN_H