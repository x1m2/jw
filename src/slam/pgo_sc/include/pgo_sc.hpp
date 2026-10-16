#ifndef PGO_SC_MAIN_HPP
#define PGO_SC_MAIN_HPP

///// common headers
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>
///// ROS2
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf2_ros/transform_broadcaster.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <slam_manager_interfaces/msg/map_keyframe.hpp>
#include <slam_manager_interfaces/msg/z_constraint_state.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <visualization_msgs/msg/marker.hpp>
///// GTSAM
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

///// Coded headers
#include "loop_closure.hpp"
#include "pose_pcd.hpp"
#include "utilities.hpp"

namespace fs = std::filesystem;
using namespace std::chrono;

// Message filter sync policy for ROS2
typedef message_filters::sync_policies::ApproximateTime<
    nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2>
    odom_pcd_sync_pol;

class PgoSc : public rclcpp::Node {
 public:
  explicit PgoSc(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  ~PgoSc() override;

  // finish_mapping通过Worker同步保存当前地图。
  bool saveMap(const std::string &map_path);

 private:
  struct LoopConstraint {
    size_t source_index;
    size_t target_index;
    gtsam::Pose3 relative_pose;
    double score;
  };

  ///// Basic params
  std::string map_frame_;
  std::string package_path_;
  std::string seq_name_;
  std::mutex realtime_pose_mutex_, keyframes_mutex_;
  std::mutex graph_mutex_, vis_mutex_;
  std::mutex z_constraint_mutex_;
  std::mutex loop_results_mutex_;
  Eigen::Matrix4d last_corrected_pose_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d odom_delta_ = Eigen::Matrix4d::Identity();
  PosePcd current_frame_;
  std::vector<PosePcd> keyframes_;
  int current_keyframe_idx_ = 0;

  ///// Graph and values
  std::atomic_bool is_initialized_;
  bool loop_added_flag_vis_;
  std::shared_ptr<gtsam::ISAM2> isam_handler_;
  gtsam::NonlinearFactorGraph gtsam_graph_;
  gtsam::Values init_esti_;
  gtsam::Values corrected_esti_;
  double keyframe_thr_;
  double keyframe_angle_thr_;
  double loop_robust_kernel_scale_;
  double voxel_res_;
  double keyframe_voxel_res_;
  bool ikdtree_rebuild_enable_;
  double ikdtree_rebuild_voxel_res_;
  int ikdtree_reconstruct_keyframe_num_;
  int sub_key_num_;
  int last_loop_src_idx_ = -1;
  std::deque<LoopConstraint> pending_loop_constraints_;
  std::vector<std::pair<size_t, size_t>> loop_idx_pairs_;  // for vis

  ///// Z-axis constraint
  bool z_axis_locked_ = false;
  std::optional<double> locked_height_;
  double add_height_factor_threshold_ = 0.1;

  ///// Visualize
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  pcl::PointCloud<pcl::PointXYZ> odoms_, corrected_odoms_;
  nav_msgs::msg::Path odom_path_, corrected_path_;
  bool global_map_vis_switch_ = true;

  ///// Results
  bool save_map_bag_, save_map_pcd_, save_in_kitti_format_;

  ///// ROS2 Publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      corrected_odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr corrected_path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      corrected_current_pcd_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      corrected_pcd_map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      ikdtree_rebuild_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      loop_detection_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      realtime_pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_src_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_dst_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      debug_coarse_aligned_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      debug_fine_aligned_pub_;
  rclcpp::Publisher<slam_manager_interfaces::msg::MapKeyframe>::SharedPtr
      keyframe_cloud_pub_;
  rclcpp::Publisher<slam_manager_interfaces::msg::ZConstraintState>::SharedPtr
      z_constraint_state_pub_;
  ///// ROS2 Subscribers
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_save_flag_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr z_lock_cmd_sub_;
  rclcpp::TimerBase::SharedPtr loop_timer_;
  rclcpp::TimerBase::SharedPtr vis_timer_;
  rclcpp::CallbackGroup::SharedPtr loop_callback_group_;

  ///// Odom/PCD sync
  std::shared_ptr<message_filters::Synchronizer<odom_pcd_sync_pol>>
      sub_odom_pcd_sync_;
  std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>>
      sub_odom_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>
      sub_pcd_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  ///// Loop closure
  std::shared_ptr<LoopClosure> loop_closure_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr save_done_pub_;

 private:
  // Methods
  void updateOdomsAndPaths(const PosePcd &pose_pcd_in);
  bool checkIfKeyframe(const PosePcd &pose_pcd_in,
                       const PosePcd &latest_pose_pcd);
  visualization_msgs::msg::Marker getLoopMarkers(
      const gtsam::Values &corrected_esti_in);
  bool addHeightFactor(const gtsam::Pose3 &pose, size_t index);
  void publishIkdtreeRebuildMap();
  void publishZConstraintState(bool locked, double height);
  void optimizeGraph(bool is_loop, bool is_height_constrained = false);
  void publishKeyframe(const PosePcd &keyframe);

  // Callbacks
  void odomPcdCallback(
      const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg,
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pcd_msg);
  void zLockCmdCallback(const std_msgs::msg::Bool::ConstSharedPtr msg);
  bool saveFlagCallback();
  void loopTimerFunc();
  void visTimerFunc();
};

#endif  // PGO_SC_MAIN_H
