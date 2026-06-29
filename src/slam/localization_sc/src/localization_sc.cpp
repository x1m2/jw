#include "localization_sc.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <rclcpp/rclcpp.hpp>
using namespace std::chrono_literals;
namespace {

double getYawFromMatrix(const Eigen::Matrix4d &T) {
  Eigen::Quaterniond q(T.block<3, 3>(0, 0));
  q.normalize();

  tf2::Quaternion tf_q(q.x(), q.y(), q.z(), q.w());
  tf_q.normalize();

  return tf2::getYaw(tf_q);
}

Eigen::Matrix4d keepXYYawOnly(const Eigen::Matrix4d &T) {
  Eigen::Matrix4d out = Eigen::Matrix4d::Identity();

  const double yaw = getYawFromMatrix(T);

  out.block<3, 3>(0, 0) =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  out(0, 3) = T(0, 3);
  out(1, 3) = T(1, 3);
  out(2, 3) = 0.0;

  return out;
}
struct RelocMatchCandidate {
  int idx = -1;
  double pose_dist = 0.0;
  double score = std::numeric_limits<double>::max();
  Eigen::Matrix4d candidate_tf = Eigen::Matrix4d::Identity();
  RegistrationOutput reg_output;
};

}  // namespace

FastLioLocalizationScQn::FastLioLocalizationScQn(const std::string &node_name)
    : Node(node_name) {
  // -----------------------
  // 1. ROS2 参数
  // -----------------------
  std::string saved_map_path;
  double map_match_hz;
  MapMatcherConfig mm_config;
  auto &gc = mm_config.gicp_config_;
  auto &qc = mm_config.quatro_config_;
  // declare/get parameters
  this->declare_parameter<std::string>("basic.map_frame", "map");
  this->get_parameter("basic.map_frame", map_frame_);
  this->declare_parameter<std::string>("basic.saved_map", "/map/result_bag");
  this->get_parameter("basic.saved_map", saved_map_path);
  this->declare_parameter<double>("basic.map_match_hz", 1.0);
  this->get_parameter("basic.map_match_hz", map_match_hz);
  this->declare_parameter<double>("basic.visualize_voxel_size", 1.0);
  this->get_parameter("basic.visualize_voxel_size", voxel_res_);
  this->declare_parameter<double>("keyframe.keyframe_threshold", 1.0);
  this->get_parameter("keyframe.keyframe_threshold", keyframe_dist_thr_);
  this->declare_parameter<int>("keyframe.num_submap_keyframes", 5);
  this->get_parameter("keyframe.num_submap_keyframes",
                      mm_config.num_submap_keyframes_);
  this->declare_parameter<double>(
      "match.scancontext_max_correspondence_distance", 15.0);
  this->get_parameter("match.scancontext_max_correspondence_distance",
                      mm_config.scancontext_max_correspondence_distance_);
  this->declare_parameter<double>("match.quatro_nano_gicp_voxel_resolution",
                                  0.3);
  this->get_parameter("match.quatro_nano_gicp_voxel_resolution",
                      mm_config.voxel_res_);
  this->declare_parameter<int>("nano_gicp.thread_number", 0);
  this->get_parameter("nano_gicp.thread_number", gc.nano_thread_number_);
  this->declare_parameter<double>("nano_gicp.icp_score_threshold", 10.0);
  this->get_parameter("nano_gicp.icp_score_threshold", gc.icp_score_thr_);

  this->declare_parameter<int>("nano_gicp.correspondences_number", 15);
  this->get_parameter("nano_gicp.correspondences_number",
                      gc.nano_correspondences_number_);
  this->declare_parameter<double>("nano_gicp.max_correspondence_distance",
                                  0.01);
  this->get_parameter("nano_gicp.max_correspondence_distance",
                      gc.max_corr_dist_);
  this->declare_parameter<int>("nano_gicp.max_iter", 32);
  this->get_parameter("nano_gicp.max_iter", gc.nano_max_iter_);
  this->declare_parameter<double>("nano_gicp.transformation_epsilon", 0.01);
  this->get_parameter("nano_gicp.transformation_epsilon",
                      gc.transformation_epsilon_);
  this->declare_parameter<double>("nano_gicp.euclidean_fitness_epsilon", 0.01);
  this->get_parameter("nano_gicp.euclidean_fitness_epsilon",
                      gc.euclidean_fitness_epsilon_);
  this->declare_parameter<int>("nano_gicp.ransac.max_iter", 5);
  this->get_parameter("nano_gicp.ransac.max_iter", gc.nano_ransac_max_iter_);
  this->declare_parameter<double>(
      "nano_gicp.ransac.outlier_rejection_threshold", 1.0);
  this->get_parameter("nano_gicp.ransac.outlier_rejection_threshold",
                      gc.ransac_outlier_rejection_threshold_);
  this->declare_parameter<bool>("quatro.enable", false);
  this->get_parameter("quatro.enable", mm_config.enable_quatro_);
  this->declare_parameter<bool>("quatro.optimize_matching", true);
  this->get_parameter("quatro.optimize_matching", qc.use_optimized_matching_);
  this->declare_parameter<double>("quatro.distance_threshold", 30.0);
  this->get_parameter("quatro.distance_threshold",
                      qc.quatro_distance_threshold_);
  this->declare_parameter<int>("quatro.max_correspondences", 200);
  this->get_parameter("quatro.max_correspondences", qc.quatro_max_num_corres_);
  this->declare_parameter<double>("quatro.fpfh_normal_radius", 0.02);
  this->get_parameter("quatro.fpfh_normal_radius", qc.fpfh_normal_radius_);
  this->declare_parameter<double>("quatro.fpfh_radius", 0.04);
  this->get_parameter("quatro.fpfh_radius", qc.fpfh_radius_);
  this->declare_parameter<bool>("quatro.estimating_scale", false);
  this->get_parameter("quatro.estimating_scale", qc.estimat_scale_);
  this->declare_parameter<double>("quatro.noise_bound", 0.25);
  this->get_parameter("quatro.noise_bound", qc.noise_bound_);
  this->declare_parameter<double>("quatro.rotation.gnc_factor", 0.25);
  this->get_parameter("quatro.rotation.gnc_factor", qc.rot_gnc_factor_);
  this->declare_parameter<double>("quatro.rotation.rot_cost_diff_threshold",
                                  0.25);
  this->get_parameter("quatro.rotation.rot_cost_diff_threshold",
                      qc.rot_cost_diff_thr_);
  this->declare_parameter<int>("quatro.rotation.num_max_iter", 50);
  this->get_parameter("quatro.rotation.num_max_iter", qc.quatro_max_iter_);

  this->declare_parameter<double>("nano_gicp.reloc_icp_score_threshold", 10.0);
  this->get_parameter("nano_gicp.reloc_icp_score_threshold",
                      gc.reloc_icp_score_thr_);
  this->declare_parameter<int>("reloc.stable_required", 3);
  this->get_parameter("reloc.stable_required", reloc_stable_required_);

  this->declare_parameter<double>("reloc.trans_diff_threshold", 0.15);
  this->get_parameter("reloc.trans_diff_threshold",
                      reloc_trans_diff_threshold_);

  this->declare_parameter<double>("reloc.yaw_diff_threshold_deg", 5.0);
  this->get_parameter("reloc.yaw_diff_threshold_deg",
                      reloc_yaw_diff_threshold_deg_);

  this->declare_parameter<bool>("reloc.use_xy_yaw_only", true);
  this->get_parameter("reloc.use_xy_yaw_only", reloc_use_xy_yaw_only_);
  this->declare_parameter<bool>("reloc.use_initial_pose", true);
  this->get_parameter("reloc.use_initial_pose", reloc_use_initial_pose_);
  // -----------------------
  // 2. Initialize map matcher
  // -----------------------
  map_matcher_ = std::make_shared<MapMatcher>(mm_config);

  // -----------------------
  // 3. Load saved map
  // -----------------------
  loadMap(saved_map_path);

  // -----------------------
  // 4. ROS2 publishers
  // -----------------------
  odom_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/ori_odom_localization", 10);
  path_pub_ =
      this->create_publisher<nav_msgs::msg::Path>("/ori_path_localization", 10);
  corrected_odom_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/corrected_odom_localization", 10);
  corrected_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      "/corrected_path_localization", 10);
  corrected_current_pcd_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/corrected_current_pcd_localization", 10);
  map_match_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/map_match_localization", 10);
  realtime_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/pose_stamped_localization", 10);
  saved_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/saved_map_localization", 10);
  debug_src_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/src_localization", 10);
  debug_dst_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/dst_localization", 10);
  debug_coarse_aligned_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/coarse_aligned_quatro_localization", 10);
  debug_fine_aligned_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/fine_aligned_nano_gicp_localization", 10);
  map_to_odom_pub_ =
      this->create_publisher<nav_msgs::msg::Odometry>("/map_to_odom", 10);
  localization_odom_pub_ =
      this->create_publisher<nav_msgs::msg::Odometry>("/localization", 10);
  // add reloc success publisher
  reloc_success_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/reloc_success", rclcpp::QoS(1).reliable().transient_local());

  /* subscribers */
  sub_odom_ =
      std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>();
  sub_pcd_ = std::make_shared<
      message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>();

  sub_odom_->subscribe(this, "/Odometry");
  sub_pcd_->subscribe(this, "/cloud_registered");

  sub_odom_pcd_sync_ =
      std::make_shared<message_filters::Synchronizer<odom_pcd_sync_pol>>(
          odom_pcd_sync_pol(10), *sub_odom_, *sub_pcd_);

  sub_odom_pcd_sync_->registerCallback(
      std::bind(&FastLioLocalizationScQn::odomPcdCallback, this,
                std::placeholders::_1, std::placeholders::_2));
  sub_initial_pose_ =
      this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/initialpose", 10,
          std::bind(&FastLioLocalizationScQn::initialPoseCallback, this,
                    std::placeholders::_1));
  // -----------------------
  // 5. Timer
  // -----------------------
  match_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / map_match_hz),
      std::bind(&FastLioLocalizationScQn::matchingTimerFunc, this));

  // -----------------------
  // 6. TF Broadcaster
  // -----------------------
  broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

  RCLCPP_WARN(this->get_logger(), "FastLioLocalizationScQn node started...");
}

void FastLioLocalizationScQn::initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  global_reloc_done_ = false;
  resetRelocStableState();
  last_matched_keyframe_idx_ = -1;
  Eigen::Quaterniond q(
      msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

  q.normalize();

  initial_pose_eig_ = Eigen::Matrix4d::Identity();
  initial_pose_eig_.block<3, 3>(0, 0) = q.toRotationMatrix();
  initial_pose_eig_(0, 3) = msg->pose.pose.position.x;
  initial_pose_eig_(1, 3) = msg->pose.pose.position.y;
  initial_pose_eig_(2, 3) = msg->pose.pose.position.z;

  need_apply_initial_pose_ = true;
  localization_state_ = LocalizationState::GLOBAL_RELOCALIZING;
  RCLCPP_WARN(this->get_logger(),
              "Received initial pose from RViz: x=%.3f, y=%.3f, z=%.3f",
              initial_pose_eig_(0, 3), initial_pose_eig_(1, 3),
              initial_pose_eig_(2, 3));
}

// ----------------------
// odom+pcd 同步回调
// ----------------------
void FastLioLocalizationScQn::odomPcdCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pcd_msg) {
  if (reloc_use_initial_pose_ &&
      localization_state_ == LocalizationState::IDLE) {
    return;
  }
  PosePcd current_frame(*odom_msg, *pcd_msg, current_keyframe_idx_);
  // 1. 打印原始里程计位姿 (pose_eig_)
  {
    Eigen::Matrix3d rot = current_frame.pose_eig_.block<3, 3>(0, 0);
    Eigen::Quaterniond q(rot);

    tf2::Quaternion tf_q(q.x(), q.y(), q.z(), q.w());

    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);

    RCLCPP_INFO(this->get_logger(),
                "[BEFORE] pose_eig_ - "
                "x: %.3f, y: %.3f, z: %.3f, "
                "roll: %.2f deg, pitch: %.2f deg, yaw: %.2f deg",
                current_frame.pose_eig_(0, 3), current_frame.pose_eig_(1, 3),
                current_frame.pose_eig_(2, 3), roll * 180.0 / M_PI,
                pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);
  }

  // 2. 打印 last_corrected_TF_
  {
    Eigen::Matrix3d rot = last_corrected_TF_.block<3, 3>(0, 0);
    Eigen::Quaterniond q(rot);

    tf2::Quaternion tf_q(q.x(), q.y(), q.z(), q.w());

    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);

    RCLCPP_INFO(this->get_logger(),
                "last_corrected_TF_ - "
                "x: %.3f, y: %.3f, z: %.3f, "
                "roll: %.2f deg, pitch: %.2f deg, yaw: %.2f deg",
                last_corrected_TF_(0, 3), last_corrected_TF_(1, 3),
                last_corrected_TF_(2, 3), roll * 180.0 / M_PI,
                pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);
  }
  if (reloc_use_initial_pose_ && need_apply_initial_pose_) {
    last_corrected_TF_ = initial_pose_eig_ * current_frame.pose_eig_.inverse();
    need_apply_initial_pose_ = false;

    RCLCPP_WARN(this->get_logger(),
                "Applied initial pose to map->odom transform.");
  }
  // 1. 实时 pose = last TF * odom
  current_frame.pose_corrected_eig_ =
      last_corrected_TF_ * current_frame.pose_eig_;
  geometry_msgs::msg::PoseStamped current_pose_stamped =
      poseEigToPoseStamped(current_frame.pose_corrected_eig_, map_frame_);
  // 4. 打印修正后的 Current pose
  {
    const auto &q_msg = current_pose_stamped.pose.orientation;

    tf2::Quaternion tf_q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);

    RCLCPP_INFO(this->get_logger(),
                "Current pose - Position: "
                "[x: %.3f, y: %.3f, z: %.3f], "
                "Orientation: [roll: %.2f deg, pitch: %.2f deg, yaw: %.2f deg]",
                current_pose_stamped.pose.position.x,
                current_pose_stamped.pose.position.y,
                current_pose_stamped.pose.position.z, roll * 180.0 / M_PI,
                pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);
  }

  realtime_pose_pub_->publish(current_pose_stamped);
  {
    geometry_msgs::msg::TransformStamped tf_msg;

    tf_msg.header.stamp = this->get_clock()->now();
    tf_msg.header.frame_id = map_frame_;
    tf_msg.child_frame_id = "odom";

    tf_msg.transform = poseEigToROSTf2(last_corrected_TF_);

    broadcaster_->sendTransform(tf_msg);
  }

  nav_msgs::msg::Odometry corrected_odom;
  corrected_odom.header.stamp = odom_msg->header.stamp;
  corrected_odom.header.frame_id = map_frame_;
  corrected_odom.child_frame_id = odom_msg->child_frame_id;

  corrected_odom.pose.pose = current_pose_stamped.pose;
  corrected_odom.twist = odom_msg->twist;

  // 设置协方差
  for (int i = 0; i < 36; i++) corrected_odom.pose.covariance[i] = 0.0;
  corrected_odom.pose.covariance[0] = 0.01;
  corrected_odom.pose.covariance[7] = 0.01;
  corrected_odom.pose.covariance[14] = 0.01;
  corrected_odom.pose.covariance[21] = 0.01;
  corrected_odom.pose.covariance[28] = 0.01;
  corrected_odom.pose.covariance[35] = 0.01;

  for (int i = 0; i < 36; i++)
    corrected_odom.twist.covariance[i] = odom_msg->twist.covariance[i];

  localization_odom_pub_->publish(corrected_odom);

  // 发布点云
  corrected_current_pcd_pub_->publish(pclToPclRos(
      transformPcd(current_frame.pcd_, current_frame.pose_corrected_eig_),
      map_frame_));

  if (!is_initialized_) {
    std::lock_guard<std::mutex> lock(keyframes_mutex_);
    last_keyframe_ = current_frame;
    current_keyframe_idx_++;

    {
      std::lock_guard<std::mutex> lock2(vis_mutex_);
      updateOdomsAndPaths(current_frame);
    }

    publishMapToOdom(this->now());
    is_initialized_ = true;
    return;
  }

  // 检查 keyframe
  if (checkIfKeyframe(current_frame, last_keyframe_) || !global_reloc_done_) {
    std::lock_guard<std::mutex> lock(keyframes_mutex_);
    last_keyframe_ = current_frame;
    current_keyframe_idx_++;

    std::lock_guard<std::mutex> lock2(vis_mutex_);
    updateOdomsAndPaths(current_frame);
  }
}

// ----------------------
// map match 定时器函数
// ----------------------
void FastLioLocalizationScQn::matchingTimerFunc() {
  if (!is_initialized_) return;

  auto t1 = high_resolution_clock::now();

  PosePcd last_keyframe_copy;
  {
    std::lock_guard<std::mutex> lock(keyframes_mutex_);
    last_keyframe_copy = last_keyframe_;
    last_keyframe_.processed_ = true;
  }
  if (last_keyframe_copy.idx_ == 0 || last_keyframe_copy.processed_) return;
  if (!reloc_use_initial_pose_ &&
      localization_state_ == LocalizationState::IDLE) {
    localization_state_ = LocalizationState::GLOBAL_RELOCALIZING;
  }
  switch (localization_state_) {
    case LocalizationState::GLOBAL_RELOCALIZING:
      handleGlobalRelocalization(last_keyframe_copy);
      break;

    case LocalizationState::LOCAL_TRACKING:
      handleLocalTracking(last_keyframe_copy);
      break;
    default:
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Localization state is IDLE, skip map matching.");
      return;
  }

  // 发布 debug 点云
  debug_src_pub_->publish(
      pclToPclRos(map_matcher_->getSourceCloud(), map_frame_));
  debug_dst_pub_->publish(
      pclToPclRos(map_matcher_->getTargetCloud(), map_frame_));
  debug_coarse_aligned_pub_->publish(
      pclToPclRos(map_matcher_->getCoarseAlignedCloud(), map_frame_));
  debug_fine_aligned_pub_->publish(
      pclToPclRos(map_matcher_->getFinalAlignedCloud(), map_frame_));

  std::lock_guard<std::mutex> lock(vis_mutex_);
  corrected_odom_pub_->publish(pclToPclRos(corrected_odoms_, map_frame_));
  corrected_path_pub_->publish(corrected_odom_path_);
  odom_pub_->publish(pclToPclRos(raw_odoms_, map_frame_));
  path_pub_->publish(raw_odom_path_);

  if (saved_map_vis_switch_ && saved_map_pub_->get_subscription_count() > 0) {
    saved_map_pub_->publish(pclToPclRos(saved_map_pcd_, map_frame_));
    saved_map_vis_switch_ = false;
  }
  if (!saved_map_vis_switch_ && saved_map_pub_->get_subscription_count() == 0) {
    saved_map_vis_switch_ = true;
  }

  auto t2 = high_resolution_clock::now();
  RCLCPP_INFO(this->get_logger(), "Matching: %.1fms",
              duration_cast<microseconds>(t2 - t1).count() / 1e3);
}

void FastLioLocalizationScQn::handleGlobalRelocalization(
    const PosePcd &last_keyframe_copy) {
  if (reloc_use_initial_pose_ &&
      localization_state_ == LocalizationState::IDLE) {
    resetRelocStableState();
    return;
  }

  int closest_keyframe_idx = -1;
  RegistrationOutput reg_output;
  Eigen::Matrix4d candidate_tf = Eigen::Matrix4d::Identity();

  if (reloc_use_initial_pose_) {
    //找附近所有候选帧
    auto pose_candidates =
        findCandidateKeyframesByPose(initial_pose_eig_, saved_map_from_bag_);

    if (pose_candidates.empty()) {
      resetRelocStableState();
      RCLCPP_WARN(this->get_logger(),
                  "[RELOCALIZATION] failed: no keyframe near initial pose.");
      return;
    }

    std::vector<RelocMatchCandidate> valid_matches;

    for (const auto &[candidate_idx, pose_dist] : pose_candidates) {
      RegistrationOutput output = map_matcher_->performMapMatcher(
          last_keyframe_copy, saved_map_from_bag_, candidate_idx, false);
      RCLCPP_INFO(this->get_logger(),
                  "[RELOCALIZATION] candidate idx=%d, pose_dist=%.3f, "
                  "valid=%d, converged=%d, score=%.3f",
                  candidate_idx, pose_dist, output.is_valid_,
                  output.is_converged_, output.score_);
      if (!output.is_valid_) continue;

      Eigen::Matrix4d tf = output.pose_between_eig_ * last_corrected_TF_;

      if (reloc_use_xy_yaw_only_) {
        tf = keepXYYawOnly(tf);
      }

      valid_matches.push_back(
          {candidate_idx, pose_dist, output.score_, tf, output});
    }

    if (valid_matches.empty()) {
      resetRelocStableState();
      RCLCPP_WARN(this->get_logger(),
                  "[RELOCALIZATION] failed: no valid registration.");
      return;
    }

    auto chosen_it = std::min_element(
        valid_matches.begin(), valid_matches.end(),
        [](const RelocMatchCandidate &a, const RelocMatchCandidate &b) {
          return a.score < b.score;
        });

    closest_keyframe_idx = chosen_it->idx;
    reg_output = chosen_it->reg_output;
    candidate_tf = chosen_it->candidate_tf;
    RCLCPP_INFO(
        this->get_logger(),
        "[RELOCALIZATION] chosen candidate idx=%d, pose_dist=%.3f, score=%.3f",
        chosen_it->idx, chosen_it->pose_dist, chosen_it->score);
  } else {
    closest_keyframe_idx = map_matcher_->fetchClosestKeyframeIdx(
        last_keyframe_copy, saved_map_from_bag_);

    if (closest_keyframe_idx < 0) {
      resetRelocStableState();
      RCLCPP_WARN(this->get_logger(),
                  "[RELOCALIZATION] failed: no ScanContext candidate.");
      return;
    }

    reg_output = map_matcher_->performMapMatcher(
        last_keyframe_copy, saved_map_from_bag_, closest_keyframe_idx, false);

    if (!reg_output.is_valid_) {
      RCLCPP_WARN(this->get_logger(),
                  "[RELOCALIZATION] rejected: registration invalid.");
      resetRelocStableState();
      return;
    }

    candidate_tf = reg_output.pose_between_eig_ * last_corrected_TF_;

    if (reloc_use_xy_yaw_only_) {
      candidate_tf = keepXYYawOnly(candidate_tf);
    }
  }

  if (!has_last_reloc_candidate_) {
    last_reloc_candidate_tf_ = candidate_tf;
    has_last_reloc_candidate_ = true;
    reloc_stable_count_ = 1;
    RCLCPP_WARN(this->get_logger(),
                "[RELOCALIZATION] candidate 1/%d, waiting stable matches. "
                "idx=%d, score=%.3f",
                reloc_stable_required_, closest_keyframe_idx,
                reg_output.score_);
    return;
  }

  bool stable =
      isRelocalizationCandidateStable(last_reloc_candidate_tf_, candidate_tf);

  if (!stable) {
    last_reloc_candidate_tf_ = candidate_tf;
    reloc_stable_count_ = 1;
    RCLCPP_WARN(this->get_logger(),
                "[RELOCALIZATION] candidate unstable, reset stable count. "
                "idx=%d, score=%.3f",
                closest_keyframe_idx, reg_output.score_);
    return;
  }

  reloc_stable_count_++;
  RCLCPP_WARN(this->get_logger(),
              "[RELOCALIZATION] stable candidate %d/%d. idx=%d, score=%.3f",
              reloc_stable_count_, reloc_stable_required_, closest_keyframe_idx,
              reg_output.score_);
  if (reloc_stable_count_ < reloc_stable_required_) {
    return;
  }

  last_corrected_TF_ = candidate_tf;
  last_matched_keyframe_idx_ = closest_keyframe_idx;
  global_reloc_done_ = true;
  localization_state_ = LocalizationState::LOCAL_TRACKING;
  RCLCPP_WARN(this->get_logger(),
              "[RELOCALIZATION] success. idx=%d, score=%.3f, "
              "map_to_odom=[x=%.3f, y=%.3f, z=%.3f, yaw=%.2f deg]",
              closest_keyframe_idx, reg_output.score_, last_corrected_TF_(0, 3),
              last_corrected_TF_(1, 3), last_corrected_TF_(2, 3),
              getYawFromMatrix(last_corrected_TF_) * 180.0 / M_PI);
  Eigen::Matrix4d tfed_pose = last_corrected_TF_ * last_keyframe_copy.pose_eig_;

  std_msgs::msg::Bool msg;
  msg.data = true;
  reloc_success_pub_->publish(msg);
  publishMatchedResult(last_keyframe_copy, tfed_pose);
}

void FastLioLocalizationScQn::handleLocalTracking(
    const PosePcd &last_keyframe_copy) {
  int closest_keyframe_idx = findNearestKeyframeByPose(
      last_keyframe_copy.pose_corrected_eig_, saved_map_from_bag_);
  RCLCPP_INFO(this->get_logger(), "Local tracking nearest keyframe idx: %d",
              closest_keyframe_idx);

  if (closest_keyframe_idx < 0) {
    RCLCPP_WARN(this->get_logger(),
                "[LOCAL_TRACKING] failed: no nearest keyframe.");
    return;
  }

  const RegistrationOutput &reg_output = map_matcher_->performMapMatcher(
      last_keyframe_copy, saved_map_from_bag_, closest_keyframe_idx, true);

  if (!reg_output.is_valid_) {
    double delta_yaw_deg =
        getYawFromMatrix(reg_output.pose_between_eig_) * 180.0 / M_PI;

    RCLCPP_WARN(this->get_logger(),
                "[LOCAL_TRACKING] rejected: registration invalid. "
                "idx=%d, converged=%d, score=%.3f, "
                "delta=[x=%.3f, y=%.3f, z=%.3f, yaw=%.2f deg]",
                closest_keyframe_idx, reg_output.is_converged_,
                reg_output.score_, reg_output.pose_between_eig_(0, 3),
                reg_output.pose_between_eig_(1, 3),
                reg_output.pose_between_eig_(2, 3), delta_yaw_deg);
    return;
  }

  last_corrected_TF_ = reg_output.pose_between_eig_ * last_corrected_TF_;

  Eigen::Matrix4d tfed_pose =
      reg_output.pose_between_eig_ * last_keyframe_copy.pose_corrected_eig_;

  last_matched_keyframe_idx_ = closest_keyframe_idx;
  publishMatchedResult(last_keyframe_copy, tfed_pose);
}

bool FastLioLocalizationScQn::isRelocalizationCandidateStable(
    const Eigen::Matrix4d &last_tf, const Eigen::Matrix4d &new_tf) {
  const Eigen::Vector2d last_xy(last_tf(0, 3), last_tf(1, 3));
  const Eigen::Vector2d new_xy(new_tf(0, 3), new_tf(1, 3));

  double trans_diff = (new_xy - last_xy).norm();

  const double last_yaw = getYawFromMatrix(last_tf);
  const double new_yaw = getYawFromMatrix(new_tf);

  double yaw_diff_deg =
      std::fabs(angles::normalize_angle(new_yaw - last_yaw)) * 180.0 / M_PI;
  bool stable = trans_diff < reloc_trans_diff_threshold_ &&
                yaw_diff_deg < reloc_yaw_diff_threshold_deg_;
  RCLCPP_WARN(this->get_logger(),
              "[RELOCALIZATION] stable check: stable=%d, "
              "trans_diff=%.3f/%.3f m, yaw_diff=%.2f/%.2f deg",
              stable, trans_diff, reloc_trans_diff_threshold_, yaw_diff_deg,
              reloc_yaw_diff_threshold_deg_);
  return stable;
}

void FastLioLocalizationScQn::resetRelocStableState() {
  reloc_stable_count_ = 0;
  has_last_reloc_candidate_ = false;
}

std::vector<std::pair<int, double>>
FastLioLocalizationScQn::findCandidateKeyframesByPose(
    const Eigen::Matrix4d &query_pose,
    const std::vector<PosePcdReduced> &saved_keyframes) {
  constexpr double search_radius = 3.0;
  constexpr int max_candidates = 10;
  std::vector<std::pair<int, double>> candidates;

  if (saved_keyframes.empty()) {
    return candidates;
  }

  Eigen::Vector3d query_pos = query_pose.block<3, 1>(0, 3);

  for (int i = 0; i < static_cast<int>(saved_keyframes.size()); ++i) {
    Eigen::Vector3d map_pos = saved_keyframes[i].pose_eig_.block<3, 1>(0, 3);

    double dx = query_pos.x() - map_pos.x();
    double dy = query_pos.y() - map_pos.y();
    double dist = std::hypot(dx, dy);

    if (dist <= search_radius) {
      candidates.emplace_back(i, dist);
    }
  }

  std::sort(candidates.begin(), candidates.end(),
            [](const auto &a, const auto &b) { return a.second < b.second; });

  if (max_candidates > 0 &&
      static_cast<int>(candidates.size()) > max_candidates) {
    candidates.resize(max_candidates);
  }
  return candidates;
}

int FastLioLocalizationScQn::findNearestKeyframeByPose(
    const Eigen::Matrix4d &query_pose,
    const std::vector<PosePcdReduced> &saved_keyframes) {
  if (saved_keyframes.empty()) {
    return -1;
  }

  Eigen::Vector3d query_pos = query_pose.block<3, 1>(0, 3);

  int start_idx = 0;
  int end_idx = static_cast<int>(saved_keyframes.size()) - 1;
  constexpr int ksearch_range = 3;
  if (last_matched_keyframe_idx_ >= 0) {
    start_idx = std::max(0, last_matched_keyframe_idx_ - ksearch_range);
    end_idx = std::min(static_cast<int>(saved_keyframes.size()) - 1,
                       last_matched_keyframe_idx_ + ksearch_range);
  }

  double min_dist = std::numeric_limits<double>::max();
  int nearest_idx = -1;
  //找距离 query_pose 最近的地图关键帧
  for (int i = start_idx; i <= end_idx; ++i) {
    Eigen::Vector3d map_pos = saved_keyframes[i].pose_eig_.block<3, 1>(0, 3);
    double dist = (query_pos - map_pos).norm();

    if (dist < min_dist) {
      min_dist = dist;
      nearest_idx = i;
    }
  }

  double max_nearest_dist = 6.0;
  if (min_dist > max_nearest_dist) {
    RCLCPP_WARN(this->get_logger(), "Nearest keyframe too far: %.3f m",
                min_dist);
    return -1;
  }

  return nearest_idx;
}

void FastLioLocalizationScQn::publishMatchedResult(
    const PosePcd &last_keyframe_copy, const Eigen::Matrix4d &tfed_pose) {
  std::lock_guard<std::mutex> lock(vis_mutex_);

  corrected_odoms_.points[last_keyframe_copy.idx_] =
      pcl::PointXYZ(tfed_pose(0, 3), tfed_pose(1, 3), tfed_pose(2, 3));

  corrected_odom_path_.poses[last_keyframe_copy.idx_] =
      poseEigToPoseStamped(tfed_pose, map_frame_);

  matched_pairs_xyz_.push_back(
      {corrected_odoms_.points[last_keyframe_copy.idx_],
       raw_odoms_.points[last_keyframe_copy.idx_]});

  map_match_pub_->publish(getMatchMarker(matched_pairs_xyz_));

  publishMapToOdom(this->now());
}
// ----------------------
// 发布 map->odom
// ----------------------
void FastLioLocalizationScQn::publishMapToOdom(const rclcpp::Time &stamp) {
  nav_msgs::msg::Odometry map_to_odom;

  // header
  map_to_odom.header.stamp = stamp;
  map_to_odom.header.frame_id = map_frame_;
  map_to_odom.child_frame_id = "odom";

  // 将Eigen变换矩阵转换为Pose消息
  Eigen::Affine3d affine(last_corrected_TF_.cast<double>());
  geometry_msgs::msg::Pose pose_msg = tf2::toMsg(affine);

  map_to_odom.pose.pose = pose_msg;

  // 设置协方差矩阵
  for (int i = 0; i < 36; i++) {
    map_to_odom.pose.covariance[i] = 0.0;
  }

  // 设置对角线元素为较小的不确定性
  map_to_odom.pose.covariance[0] = 0.01;   // x
  map_to_odom.pose.covariance[7] = 0.01;   // y
  map_to_odom.pose.covariance[14] = 0.01;  // z
  map_to_odom.pose.covariance[21] = 0.01;  // roll
  map_to_odom.pose.covariance[28] = 0.01;  // pitch
  map_to_odom.pose.covariance[35] = 0.01;  // yaw

  // 发布
  map_to_odom_pub_->publish(map_to_odom);

  RCLCPP_INFO(
      this->get_logger(), "Published map->odom transform: [%.3f, %.3f, %.3f]",
      map_to_odom.pose.pose.position.x, map_to_odom.pose.pose.position.y,
      map_to_odom.pose.pose.position.z);
}

// ----------------------
// 更新 odom 和 path
// ----------------------
void FastLioLocalizationScQn::updateOdomsAndPaths(const PosePcd &pose_pcd_in) {
  raw_odoms_.points.emplace_back(pose_pcd_in.pose_eig_(0, 3),
                                 pose_pcd_in.pose_eig_(1, 3),
                                 pose_pcd_in.pose_eig_(2, 3));
  corrected_odoms_.points.emplace_back(pose_pcd_in.pose_corrected_eig_(0, 3),
                                       pose_pcd_in.pose_corrected_eig_(1, 3),
                                       pose_pcd_in.pose_corrected_eig_(2, 3));

  raw_odom_path_.poses.emplace_back(
      poseEigToPoseStamped(pose_pcd_in.pose_eig_, map_frame_));
  corrected_odom_path_.poses.emplace_back(
      poseEigToPoseStamped(pose_pcd_in.pose_corrected_eig_, map_frame_));
}

// ----------------------
// 匹配 marker
// ----------------------
visualization_msgs::msg::Marker FastLioLocalizationScQn::getMatchMarker(
    const std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>>
        &match_xyz_pairs) {
  visualization_msgs::msg::Marker edges;
  edges.type = visualization_msgs::msg::Marker::LINE_LIST;
  edges.scale.x = 0.2f;
  edges.header.frame_id = map_frame_;
  edges.pose.orientation.w = 1.0f;
  edges.color.r = 1.0f;
  edges.color.g = 1.0f;
  edges.color.b = 1.0f;
  edges.color.a = 1.0f;

  for (const auto &p : match_xyz_pairs) {
    geometry_msgs::msg::Point pt1, pt2;
    pt1.x = p.first.x;
    pt1.y = p.first.y;
    pt1.z = p.first.z;
    pt2.x = p.second.x;
    pt2.y = p.second.y;
    pt2.z = p.second.z;
    edges.points.push_back(pt1);
    edges.points.push_back(pt2);
  }
  return edges;
}

// ----------------------
// 判断 keyframe
// ----------------------
bool FastLioLocalizationScQn::checkIfKeyframe(const PosePcd &pose_pcd_in,
                                              const PosePcd &latest_pose_pcd) {
  Eigen::Vector3d diff = latest_pose_pcd.pose_corrected_eig_.block<3, 1>(0, 3) -
                         pose_pcd_in.pose_corrected_eig_.block<3, 1>(0, 3);
  return diff.norm() > keyframe_dist_thr_;
}

geometry_msgs::msg::Transform FastLioLocalizationScQn::poseEigToROSTf2(
    const Eigen::Matrix4d &pose) {
  geometry_msgs::msg::Transform transform;

  transform.translation.x = pose(0, 3);
  transform.translation.y = pose(1, 3);
  transform.translation.z = pose(2, 3);

  Eigen::Matrix3d rotation = pose.block<3, 3>(0, 0);
  Eigen::Quaterniond q(rotation);
  q.normalize();

  transform.rotation.x = q.x();
  transform.rotation.y = q.y();
  transform.rotation.z = q.z();
  transform.rotation.w = q.w();

  return transform;
}

// ----------------------
// 加载 map
// ----------------------
void FastLioLocalizationScQn::loadMap(const std::string &saved_map_path) {
  // 创建 rosbag2 reader
  rosbag2_cpp::Reader reader;
  reader.open(saved_map_path);  // ROS 2 bag 文件通常是 *.db3

  std::vector<sensor_msgs::msg::PointCloud2> load_pcd_vec;
  std::vector<geometry_msgs::msg::PoseStamped> load_pose_vec;

  // rclcpp 序列化对象，用于反序列化消息
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pcd_serializer;
  rclcpp::Serialization<geometry_msgs::msg::PoseStamped> pose_serializer;

  // 遍历 bag 中的所有消息
  while (reader.has_next()) {
    auto bag_msg = reader.read_next();

    if (bag_msg->topic_name == "/keyframe_pcd") {
      sensor_msgs::msg::PointCloud2 msg;
      rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);
      pcd_serializer.deserialize_message(&serialized_msg, &msg);
      load_pcd_vec.push_back(msg);
    } else if (bag_msg->topic_name == "/keyframe_pose") {
      geometry_msgs::msg::PoseStamped msg;
      rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);
      pose_serializer.deserialize_message(&serialized_msg, &msg);
      load_pose_vec.push_back(msg);
    }
  }

  if (load_pcd_vec.size() != load_pose_vec.size()) {
    RCLCPP_ERROR(this->get_logger(), "WRONG BAG FILE!!!!!");
    return;
  }

  // 将 Pose 和 PCD 结合成 saved_map_from_bag_ map下
  for (size_t i = 0; i < load_pose_vec.size(); ++i) {
    saved_map_from_bag_.push_back(
        PosePcdReduced(load_pose_vec[i], load_pcd_vec[i], i));

    // transform 点云到全局坐标
    saved_map_pcd_ += saved_map_from_bag_[i].pcd_;

    // 更新 Scan Context，用于 loop candidate detection
    map_matcher_->updateScancontext(saved_map_from_bag_[i].pcd_);
  }

  // voxelize 降采样
  saved_map_pcd_ = *voxelizePcd(saved_map_pcd_, voxel_res_);

  RCLCPP_INFO(this->get_logger(), "Loaded map with %zu keyframes",
              load_pose_vec.size());
}
