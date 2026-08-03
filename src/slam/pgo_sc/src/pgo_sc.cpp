#include "pgo_sc.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Geometry>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

using namespace std::chrono_literals;
namespace fs = std::filesystem;
PgoSc::PgoSc(const rclcpp::NodeOptions &options)
    : Node("pgo_sc", options),
      is_initialized_(false),
      loop_added_flag_vis_(false) {
  ////// ROS2 params
  double loop_update_hz, vis_hz;
  LoopClosureConfig lc_config;
  auto &gc = lc_config.gicp_config_;
  auto &qc = lc_config.quatro_config_;

  /* declare parameters */
  this->declare_parameter<std::string>("basic.map_frame", "map");
  this->declare_parameter<double>("basic.loop_update_hz", 1.0);
  this->declare_parameter<double>("basic.vis_hz", 0.5);
  this->declare_parameter<std::string>("basic.package_path", "");
  this->declare_parameter<double>("save_voxel_resolution", 0.3);
  this->declare_parameter<double>("quatro_nano_gicp_voxel_resolution", 0.3);

  this->declare_parameter<double>("keyframe.keyframe_threshold", 1.0);
  this->declare_parameter<int>("keyframe.num_submap_keyframes", 5);
  this->declare_parameter<bool>("keyframe.enable_submap_matching", false);

  this->declare_parameter<double>("loop.robust_kernel_scale", 1.0);
  this->declare_parameter<double>("loop.history_keyframe_search_xy_radius",
                                  15.0);
  this->declare_parameter<double>("loop.history_keyframe_search_z_tolerance",
                                  1.5);
  this->declare_parameter<int>("loop.history_keyframe_min_index_diff", 30);

  this->declare_parameter<double>("scancontext_max_correspondence_distance",
                                  35.0);

  this->declare_parameter<int>("nano_gicp.thread_number", 0);
  this->declare_parameter<double>("nano_gicp.icp_score_threshold", 10.0);
  this->declare_parameter<int>("nano_gicp.correspondences_number", 15);
  this->declare_parameter<double>("nano_gicp.max_correspondence_distance",
                                  0.01);
  this->declare_parameter<int>("nano_gicp.max_iter", 32);
  this->declare_parameter<double>("nano_gicp.transformation_epsilon", 0.01);
  this->declare_parameter<double>("nano_gicp.euclidean_fitness_epsilon", 0.01);
  this->declare_parameter<int>("nano_gicp.ransac.max_iter", 5);
  this->declare_parameter<double>(
      "nano_gicp.ransac.outlier_rejection_threshold", 1.0);

  this->declare_parameter<bool>("quatro.enable", false);
  this->declare_parameter<bool>("quatro.optimize_matching", true);
  this->declare_parameter<double>("quatro.distance_threshold", 30.0);
  this->declare_parameter<int>("quatro.max_correspondences", 200);
  this->declare_parameter<double>("quatro.fpfh_normal_radius", 0.3);
  this->declare_parameter<double>("quatro.fpfh_radius", 0.5);
  this->declare_parameter<bool>("quatro.estimating_scale", false);
  this->declare_parameter<double>("quatro.noise_bound", 0.3);
  this->declare_parameter<double>("quatro.rotation.gnc_factor", 1.4);
  this->declare_parameter<double>("quatro.rotation.rot_cost_diff_threshold",
                                  0.0001);
  this->declare_parameter<int>("quatro.rotation.num_max_iter", 50);

  this->declare_parameter<bool>("result.save_map_bag", false);
  this->declare_parameter<bool>("result.save_map_pcd", false);
  this->declare_parameter<bool>("result.save_in_kitti_format", false);
  this->declare_parameter<std::string>("result.seq_name", "");

  /* get parameters */
  this->get_parameter("basic.map_frame", map_frame_);
  this->get_parameter("basic.package_path", package_path_);
  this->get_parameter("basic.loop_update_hz", loop_update_hz);
  this->get_parameter("basic.vis_hz", vis_hz);

  this->get_parameter("save_voxel_resolution", voxel_res_);
  this->get_parameter("quatro_nano_gicp_voxel_resolution",
                      lc_config.voxel_res_);

  this->get_parameter("keyframe.keyframe_threshold", keyframe_thr_);
  this->get_parameter("keyframe.num_submap_keyframes",
                      lc_config.num_submap_keyframes_);
  this->get_parameter("keyframe.enable_submap_matching",
                      lc_config.enable_submap_matching_);

  this->get_parameter("scancontext_max_correspondence_distance",
                      lc_config.scancontext_max_correspondence_distance_);
  this->get_parameter("loop.robust_kernel_scale", loop_robust_kernel_scale_);
  this->get_parameter("loop.history_keyframe_search_xy_radius",
                      lc_config.history_keyframe_search_xy_radius_);
  this->get_parameter("loop.history_keyframe_search_z_tolerance",
                      lc_config.history_keyframe_search_z_tolerance_);
  this->get_parameter("loop.history_keyframe_min_index_diff",
                      lc_config.history_keyframe_min_index_diff_);

  this->get_parameter("nano_gicp.thread_number", gc.nano_thread_number_);
  this->get_parameter("nano_gicp.icp_score_threshold", gc.icp_score_thr_);
  this->get_parameter("nano_gicp.correspondences_number",
                      gc.nano_correspondences_number_);
  this->get_parameter("nano_gicp.max_correspondence_distance",
                      gc.max_corr_dist_);
  this->get_parameter("nano_gicp.max_iter", gc.nano_max_iter_);
  this->get_parameter("nano_gicp.transformation_epsilon",
                      gc.transformation_epsilon_);
  this->get_parameter("nano_gicp.euclidean_fitness_epsilon",
                      gc.euclidean_fitness_epsilon_);
  this->get_parameter("nano_gicp.ransac.max_iter", gc.nano_ransac_max_iter_);
  this->get_parameter("nano_gicp.ransac.outlier_rejection_threshold",
                      gc.ransac_outlier_rejection_threshold_);

  this->get_parameter("quatro.enable", lc_config.enable_quatro_);
  this->get_parameter("quatro.optimize_matching", qc.use_optimized_matching_);
  this->get_parameter("quatro.distance_threshold",
                      qc.quatro_distance_threshold_);
  this->get_parameter("quatro.max_correspondences", qc.quatro_max_num_corres_);
  this->get_parameter("quatro.fpfh_normal_radius", qc.fpfh_normal_radius_);
  this->get_parameter("quatro.fpfh_radius", qc.fpfh_radius_);
  this->get_parameter("quatro.estimating_scale", qc.estimat_scale_);
  this->get_parameter("quatro.noise_bound", qc.noise_bound_);
  this->get_parameter("quatro.rotation.gnc_factor", qc.rot_gnc_factor_);
  this->get_parameter("quatro.rotation.rot_cost_diff_threshold",
                      qc.rot_cost_diff_thr_);
  this->get_parameter("quatro.rotation.num_max_iter", qc.quatro_max_iter_);

  this->get_parameter("result.save_map_bag", save_map_bag_);
  this->get_parameter("result.save_map_pcd", save_map_pcd_);
  this->get_parameter("result.save_in_kitti_format", save_in_kitti_format_);
  this->get_parameter("result.seq_name", seq_name_);

  /* loop closure */
  loop_closure_ = std::make_shared<LoopClosure>(lc_config);

  /* Initialization of GTSAM */
  gtsam::ISAM2Params isam_params;
  isam_params.relinearizeThreshold = 0.01;
  isam_params.relinearizeSkip = 1;
  isam_handler_ = std::make_shared<gtsam::ISAM2>(isam_params);

  /* ROS things */
  odom_path_.header.frame_id = map_frame_;
  corrected_path_.header.frame_id = map_frame_;

  /* QoS */
  auto qos_latched =
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
  auto qos_normal = rclcpp::QoS(rclcpp::KeepLast(10));

  /* publishers */
  broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  odom_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/ori_odom", qos_latched);
  path_pub_ =
      this->create_publisher<nav_msgs::msg::Path>("/ori_path", qos_latched);
  corrected_odom_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/corrected_odom", qos_latched);
  corrected_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      "/corrected_path", qos_latched);
  corrected_pcd_map_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("/corrected_map",
                                                            qos_latched);
  corrected_current_pcd_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/corrected_current_pcd", qos_latched);
  loop_detection_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/loop_detection", qos_latched);
  realtime_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/pose_stamped", qos_normal);
  debug_src_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/src", qos_latched);
  debug_dst_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/dst", qos_latched);
  debug_coarse_aligned_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/coarse_aligned_quatro", qos_latched);
  debug_fine_aligned_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/fine_aligned_nano_gicp", qos_latched);
  keyframe_cloud_pub_ =
      this->create_publisher<robot_interfaces::msg::MapKeyframe>(
          "/pgo/keyframe_cloud", rclcpp::QoS(100).reliable());

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

  sub_odom_pcd_sync_->registerCallback(std::bind(&PgoSc::odomPcdCallback, this,
                                                 std::placeholders::_1,
                                                 std::placeholders::_2));

  sub_save_flag_ = this->create_subscription<std_msgs::msg::Empty>(
      "/save_dir", rclcpp::QoS(1),
      [this](std_msgs::msg::Empty::SharedPtr /*msg*/) {
        RCLCPP_INFO(this->get_logger(),
                    "Receive save trigger, start saving...");
        this->saveFlagCallback();
      });
  save_done_pub_ = this->create_publisher<std_msgs::msg::Empty>(
      "/save_done", rclcpp::QoS(1).transient_local().reliable());

  /* timers */
  loop_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / loop_update_hz),
      std::bind(&PgoSc::loopTimerFunc, this));

  vis_timer_ =
      this->create_wall_timer(std::chrono::duration<double>(1.0 / vis_hz),
                              std::bind(&PgoSc::visTimerFunc, this));

  RCLCPP_INFO(this->get_logger(), "Main class, starting node...");
}

PgoSc::~PgoSc() {}

// ====================== odom + pcd callback ======================
// 功能：前端位姿估计 + 关键帧管理 + 后端图优化
void PgoSc::odomPcdCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pcd_msg) {
  // 保存上一帧 odom，用于计算 delta
  Eigen::Matrix4d last_odom_tf = current_frame_.pose_eig_;
  current_frame_ =
      PosePcd(*odom_msg, *pcd_msg, current_keyframe_idx_);  // 构建当前帧
  // keyframes_[i].pcd_是局部点云，pose_eig_是odom坐标系下的位姿
  {
    // 提取旋转矩阵
    Eigen::Matrix3d rot = current_frame_.pose_eig_.block<3, 3>(0, 0);
    Eigen::Quaterniond q(rot);

    // Eigen 四元数转 tf2 四元数
    tf2::Quaternion tf_q(q.x(), q.y(), q.z(), q.w());

    // 转 roll pitch yaw
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);

    // 弧度转角度
    const double rad2deg = 180.0 / M_PI;

    RCLCPP_INFO(this->get_logger(),
                "[BEFORE] current_frame_ - "
                "x: %.3f, y: %.3f, z: %.3f, "
                "roll: %.2f deg, pitch: %.2f deg, yaw: %.2f deg",
                current_frame_.pose_eig_(0, 3), current_frame_.pose_eig_(1, 3),
                current_frame_.pose_eig_(2, 3), roll * rad2deg, pitch * rad2deg,
                yaw * rad2deg);
  }

  auto t1 = std::chrono::high_resolution_clock::now();

  // ==== 1. 实时位姿计算 ====
  {
    std::lock_guard<std::mutex> lock(realtime_pose_mutex_);
    odom_delta_ =
        odom_delta_ * last_odom_tf.inverse() * current_frame_.pose_eig_;
    current_frame_.pose_corrected_eig_ = last_corrected_pose_ * odom_delta_;

    // ROS2 发布 PoseStamped
    geometry_msgs::msg::PoseStamped pose_msg =
        poseEigToPoseStamped(current_frame_.pose_corrected_eig_, map_frame_);
    {
      // 四元数转 RPY
      tf2::Quaternion q(
          pose_msg.pose.orientation.x, pose_msg.pose.orientation.y,
          pose_msg.pose.orientation.z, pose_msg.pose.orientation.w);

      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

      const double rad2deg = 180.0 / M_PI;

      RCLCPP_INFO(this->get_logger(),
                  "Current pose - "
                  "Position: [x: %.3f, y: %.3f, z: %.3f], "
                  "RPY: [roll: %.2f deg, pitch: %.2f deg, yaw: %.2f deg]",
                  pose_msg.pose.position.x, pose_msg.pose.position.y,
                  pose_msg.pose.position.z, roll * rad2deg, pitch * rad2deg,
                  yaw * rad2deg);
    }
    realtime_pose_pub_->publish(pose_msg);
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = odom_msg->header.stamp;
    tf_msg.header.frame_id = map_frame_;
    tf_msg.child_frame_id = "odom";
    Eigen::Matrix4d map_to_odom =
        current_frame_.pose_corrected_eig_ * current_frame_.pose_eig_.inverse();

    tf_msg.transform = poseEigToROSTf(map_to_odom);

    broadcaster_->sendTransform(tf_msg);
  }
  // 发布当前校正点云
  corrected_current_pcd_pub_->publish(pclToPclRos(
      transformPcd(current_frame_.pcd_, current_frame_.pose_corrected_eig_),
      map_frame_));

  // ==== 初始化关键帧（只执行一次） ====
  if (!is_initialized_) {
    {
      std::lock_guard<std::mutex> lock(keyframes_mutex_);
      keyframes_.push_back(current_frame_);
    }
    publishKeyframe(current_frame_);
    updateOdomsAndPaths(current_frame_);

    // 添加 PriorFactor 到图
    auto variance_vector =
        (gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2).finished();
    auto prior_noise = gtsam::noiseModel::Diagonal::Variances(variance_vector);
    gtsam_graph_.add(gtsam::PriorFactor<gtsam::Pose3>(
        0, poseEigToGtsamPose(current_frame_.pose_eig_), prior_noise));
    init_esti_.insert(current_keyframe_idx_,
                      poseEigToGtsamPose(current_frame_.pose_eig_));
    current_keyframe_idx_++;

    // ScanContext候选检测已停用，故不再生成描述子。
    // loop_closure_->updateScancontext(current_frame_.pcd_);

    is_initialized_ = true;
    return;  // 初始化完成直接返回
  }

  // ==== 2. 判断是否是新关键帧 ====
  // auto t2 = std::chrono::high_resolution_clock::now();
  if (checkIfKeyframe(current_frame_, keyframes_.back())) {
    {
      std::lock_guard<std::mutex> lock(keyframes_mutex_);
      keyframes_.push_back(current_frame_);
    }
    publishKeyframe(current_frame_);

    // ==== 3. 添加到图优化 ====
    auto variance_vector =
        (gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2).finished();
    auto odom_noise = gtsam::noiseModel::Diagonal::Variances(variance_vector);

    gtsam::Pose3 pose_from = poseEigToGtsamPose(
        keyframes_[current_keyframe_idx_ - 1].pose_corrected_eig_);
    gtsam::Pose3 pose_to =
        poseEigToGtsamPose(current_frame_.pose_corrected_eig_);
    {
      std::lock_guard<std::mutex> lock(graph_mutex_);
      gtsam_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
          current_keyframe_idx_ - 1, current_keyframe_idx_,
          pose_from.between(pose_to), odom_noise));
      init_esti_.insert(current_keyframe_idx_, pose_to);
    }

    current_keyframe_idx_++;

    // ==== ScanContext候选检测已停用 ====
    // loop_closure_->updateScancontext(current_frame_.pcd_);

    // ==== 4. 更新可视化 ====
    // auto t3 = std::chrono::high_resolution_clock::now();
    {
      std::lock_guard<std::mutex> lock(vis_mutex_);
      updateOdomsAndPaths(current_frame_);
    }

    // ==== 5. 图优化（ISAM2） ====
    // auto t4 = std::chrono::high_resolution_clock::now();
    optimizeGraph(false);
    // ==== 7. ROS2 中可以省略详细时间统计，保留简单日志 ====
    auto t6 = std::chrono::high_resolution_clock::now();
    RCLCPP_INFO(
        this->get_logger(), "Keyframe added, total time: %.1f ms",
        std::chrono::duration_cast<std::chrono::milliseconds>(t6 - t1).count());
  }
}

// ====================== loop closure timer ======================
void PgoSc::loopTimerFunc() {
  if (!is_initialized_ || keyframes_.empty()) return;

  auto &latest_keyframe = keyframes_.back();
  if (latest_keyframe.processed_) return;
  latest_keyframe.processed_ = true;

  constexpr int kLoopCooldownKeyframes = 20;
  const int latest_idx = latest_keyframe.idx_;

  if (last_loop_src_idx_ >= 0 &&
      latest_idx - last_loop_src_idx_ < kLoopCooldownKeyframes) {
    return;
  }

  auto t1 = std::chrono::high_resolution_clock::now();

  // ==== 1. XY半径＋高度差＋关键帧索引间隔搜索 ====
  // int closest_keyframe_idx =
  //     loop_closure_->fetchCandidateKeyframeIdx(latest_keyframe, keyframes_);
  int closest_keyframe_idx =
      loop_closure_->fetchRadiusSearchCandidateIdx(latest_keyframe, keyframes_);
  if (closest_keyframe_idx < 0) return;
  RegistrationOutput reg_output = loop_closure_->performLoopClosure(
      latest_keyframe, keyframes_, closest_keyframe_idx);

  // ==== 2. 如果 loop closure 有效，加入图优化 ====
  if (reg_output.is_valid_) {
    RCLCPP_INFO(this->get_logger(),
                "\033[1;32mLoop closure accepted. Score: %.3f\033[0m",
                reg_output.score_);

    gtsam::Pose3 pose_from = poseEigToGtsamPose(
        reg_output.pose_between_eig_ * latest_keyframe.pose_corrected_eig_);
    gtsam::Pose3 pose_to = poseEigToGtsamPose(
        keyframes_[closest_keyframe_idx].pose_corrected_eig_);

    auto variance_vector =
        (gtsam::Vector(6) << reg_output.score_, reg_output.score_,
         reg_output.score_, reg_output.score_, reg_output.score_,
         reg_output.score_)
            .finished();

    auto base_loop_noise =
        gtsam::noiseModel::Diagonal::Variances(variance_vector);
    auto loop_noise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(
            loop_robust_kernel_scale_),
        base_loop_noise);

    {
      std::lock_guard<std::mutex> lock(graph_mutex_);
      gtsam_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
          latest_keyframe.idx_, closest_keyframe_idx,
          pose_from.between(pose_to), loop_noise));
    }
    optimizeGraph(true);
    // 保存 loop closure 信息用于可视化
    loop_idx_pairs_.push_back({latest_keyframe.idx_, closest_keyframe_idx});
    loop_added_flag_vis_ = true;
    last_loop_src_idx_ = latest_keyframe.idx_;
  } else {
    RCLCPP_WARN(this->get_logger(), "Loop closure rejected. Score: %.3f",
                reg_output.score_);
    const double reject_detail_score_thr = 2.0;

    if (reg_output.score_ < reject_detail_score_thr) {
      // 只有 score < 2.0 的 rejected 才打印详细原因

      const auto &cand_keyframe = keyframes_[closest_keyframe_idx];

      Eigen::Vector3d t_latest =
          latest_keyframe.pose_corrected_eig_.block<3, 1>(0, 3);
      Eigen::Vector3d t_cand =
          cand_keyframe.pose_corrected_eig_.block<3, 1>(0, 3);

      double xy_diff = (t_latest.head<2>() - t_cand.head<2>()).norm();
      double z_diff = t_latest.z() - t_cand.z();
      double trans_diff = (t_latest - t_cand).norm();

      Eigen::Matrix3d R_latest =
          latest_keyframe.pose_corrected_eig_.block<3, 3>(0, 0);
      Eigen::Matrix3d R_cand =
          cand_keyframe.pose_corrected_eig_.block<3, 3>(0, 0);

      double rot_diff_deg =
          Eigen::AngleAxisd(R_cand.transpose() * R_latest).angle() * 180.0 /
          M_PI;

      auto getRPYDeg = [](const Eigen::Matrix4d &T) {
        Eigen::Matrix3d R = T.block<3, 3>(0, 0);
        Eigen::Quaterniond q(R);
        tf2::Quaternion tf_q(q.x(), q.y(), q.z(), q.w());

        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);

        const double rad2deg = 180.0 / M_PI;
        return std::array<double, 3>{roll * rad2deg, pitch * rad2deg,
                                     yaw * rad2deg};
      };

      auto latest_rpy = getRPYDeg(latest_keyframe.pose_corrected_eig_);
      auto cand_rpy = getRPYDeg(cand_keyframe.pose_corrected_eig_);

      RCLCPP_WARN(this->get_logger(),
                  "[LOOP_REJECT_DETAIL] latest=%d cand=%d "
                  "score=%.3f < %.3f "
                  "xy_diff=%.3f m, z_diff=%.3f m, trans_diff=%.3f m, "
                  "rot_diff=%.2f deg | "
                  "latest_xyz=[%.3f %.3f %.3f], cand_xyz=[%.3f %.3f %.3f] | "
                  "latest_rpy=[%.2f %.2f %.2f], cand_rpy=[%.2f %.2f %.2f]",
                  latest_keyframe.idx_, closest_keyframe_idx, reg_output.score_,
                  reject_detail_score_thr, xy_diff, z_diff, trans_diff,
                  rot_diff_deg, t_latest.x(), t_latest.y(), t_latest.z(),
                  t_cand.x(), t_cand.y(), t_cand.z(), latest_rpy[0],
                  latest_rpy[1], latest_rpy[2], cand_rpy[0], cand_rpy[1],
                  cand_rpy[2]);
    }
  }

  auto t2 = std::chrono::high_resolution_clock::now();

  // ==== 3. 发布调试点云（debug） ====
  debug_src_pub_->publish(
      pclToPclRos(loop_closure_->getSourceCloud(), map_frame_));
  debug_dst_pub_->publish(
      pclToPclRos(loop_closure_->getTargetCloud(), map_frame_));
  debug_fine_aligned_pub_->publish(
      pclToPclRos(loop_closure_->getFinalAlignedCloud(), map_frame_));
  debug_coarse_aligned_pub_->publish(
      pclToPclRos(loop_closure_->getCoarseAlignedCloud(), map_frame_));

  // ==== 4. 输出循环耗时信息 ====
  RCLCPP_INFO(
      this->get_logger(), "loop: %.1f ms",
      std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() /
          1e3);
}

// ====================== visualization timer ======================
void PgoSc::visTimerFunc() {
  if (!is_initialized_) return;

  auto tv1 = std::chrono::high_resolution_clock::now();

  //// 1. 如果 loop closure 更新，处理可视化数据
  if (loop_added_flag_vis_) {
    gtsam::Values corrected_esti_copied;
    pcl::PointCloud<pcl::PointXYZ> corrected_odoms;
    nav_msgs::msg::Path corrected_path;  // ROS2 使用 msg 命名空间

    // 拷贝实时估计，避免多线程冲突
    {
      std::lock_guard<std::mutex> lock(realtime_pose_mutex_);
      corrected_esti_copied = corrected_esti_;
    }

    // 遍历估计结果生成校正轨迹
    for (size_t i = 0; i < corrected_esti_copied.size(); ++i) {
      gtsam::Pose3 pose_ = corrected_esti_copied.at<gtsam::Pose3>(i);
      corrected_odoms.points.emplace_back(pose_.translation().x(),
                                          pose_.translation().y(),
                                          pose_.translation().z());
      corrected_path.poses.push_back(gtsamPoseToPoseStamped(pose_, map_frame_));
    }

    // 发布 loop closure 约束 Marker
    if (!loop_idx_pairs_.empty()) {
      loop_detection_pub_->publish(getLoopMarkers(corrected_esti_copied));
    }

    // 更新全局可视化数据
    {
      std::lock_guard<std::mutex> lock(vis_mutex_);
      corrected_odoms_ = corrected_odoms;
      corrected_path_.poses = corrected_path.poses;
    }

    loop_added_flag_vis_ = false;
  }

  //// 2. 发布 odom、路径
  {
    std::lock_guard<std::mutex> lock(vis_mutex_);
    odom_pub_->publish(pclToPclRos(odoms_, map_frame_));
    path_pub_->publish(odom_path_);
    corrected_odom_pub_->publish(pclToPclRos(corrected_odoms_, map_frame_));
    corrected_path_pub_->publish(corrected_path_);
  }

  //// 3. 发布全局地图
  if (global_map_vis_switch_ &&
      corrected_pcd_map_pub_->get_subscription_count() >
          0)  // ROS2 获取订阅者数量
  {
    pcl::PointCloud<PointType>::Ptr corrected_map(
        new pcl::PointCloud<PointType>());
    corrected_map->reserve(keyframes_[0].pcd_.size() * keyframes_.size());

    {
      std::lock_guard<std::mutex> lock(keyframes_mutex_);
      for (size_t i = 0; i < keyframes_.size(); ++i) {
        *corrected_map +=
            transformPcd(keyframes_[i].pcd_, keyframes_[i].pose_corrected_eig_);
      }
    }

    const auto &voxelized_map = voxelizePcd(corrected_map, voxel_res_);
    corrected_pcd_map_pub_->publish(pclToPclRos(*voxelized_map, map_frame_));
    global_map_vis_switch_ = false;
  }

  if (!global_map_vis_switch_ &&
      corrected_pcd_map_pub_->get_subscription_count() == 0) {
    global_map_vis_switch_ = true;
  }

  auto tv2 = std::chrono::high_resolution_clock::now();
  // ROS2 日志输出
  RCLCPP_INFO(
      this->get_logger(), "vis: %.1f ms",
      std::chrono::duration_cast<std::chrono::microseconds>(tv2 - tv1).count() /
          1e3);
}

// ====================== optimizeGraph ======================
void PgoSc::optimizeGraph(bool is_loop) {
  gtsam::Values new_estimate;

  // 1. 更新 iSAM2
  {
    std::lock_guard<std::mutex> lock(graph_mutex_);

    isam_handler_->update(gtsam_graph_, init_esti_);
    isam_handler_->update();

    if (is_loop) {
      // 回环约束加入后，多更新几次促进重线性化
      isam_handler_->update();
      isam_handler_->update();
      isam_handler_->update();
    }

    gtsam_graph_.resize(0);
    init_esti_.clear();

    new_estimate = isam_handler_->calculateEstimate();
  }

  if (new_estimate.empty()) {
    RCLCPP_WARN(this->get_logger(), "[optimizeGraph] new_estimate is empty");
    return;
  }

  // 2. 更新实时位姿基准
  {
    std::lock_guard<std::mutex> lock(realtime_pose_mutex_);

    corrected_esti_ = new_estimate;
    last_corrected_pose_ = gtsamPoseToPoseEig(
        corrected_esti_.at<gtsam::Pose3>(corrected_esti_.size() - 1));

    if (is_loop) {
      odom_delta_ =
          keyframes_.back().pose_eig_.inverse() * current_frame_.pose_eig_;
    } else {
      odom_delta_ = Eigen::Matrix4d::Identity();
    }
  }

  // 3. 回环后更新全部关键帧
  if (is_loop) {
    std::lock_guard<std::mutex> lock(keyframes_mutex_);

    for (size_t i = 0; i < new_estimate.size(); ++i) {
      keyframes_[i].pose_corrected_eig_ =
          gtsamPoseToPoseEig(new_estimate.at<gtsam::Pose3>(i));
    }
  }
}

void PgoSc::publishKeyframe(const PosePcd &keyframe) {
  if (keyframe.idx_ < 0) {
    RCLCPP_ERROR(this->get_logger(),
                 "Cannot publish keyframe with negative id: %d", keyframe.idx_);
    return;
  }

  robot_interfaces::msg::MapKeyframe message;
  message.keyframe_id = static_cast<uint32_t>(keyframe.idx_);
  message.cloud = pclToPclRos(
      transformPcd(keyframe.pcd_, keyframe.pose_corrected_eig_), map_frame_);
  const int64_t timestamp_nanoseconds =
      static_cast<int64_t>(keyframe.timestamp_ * 1e9);
  message.cloud.header.stamp.sec =
      static_cast<int32_t>(timestamp_nanoseconds / 1000000000LL);
  message.cloud.header.stamp.nanosec =
      static_cast<uint32_t>(timestamp_nanoseconds % 1000000000LL);
  keyframe_cloud_pub_->publish(message);
}
// ====================== save callback ======================
void PgoSc::saveFlagCallback() {
  if (package_path_.empty()) {
    throw std::runtime_error("basic.package_path is empty");
  }
  std::error_code directory_error;
  fs::create_directories(package_path_, directory_error);
  if (directory_error) {
    throw std::runtime_error("failed to create map directory: " +
                             directory_error.message());
  }

  std::string seq_directory = package_path_ + "/" + seq_name_;
  std::string scans_directory = seq_directory + "/scans";

  //// 1. 保存点云和位姿，KITTI / TUM 格式
  if (save_in_kitti_format_) {
    RCLCPP_INFO(this->get_logger(),
                "\033[32;1mScans are saved in %s, following the KITTI and TUM "
                "format\033[0m",
                scans_directory.c_str());

    // 删除已有目录并创建新目录
    std::error_code ec;

    if (fs::exists(seq_directory, ec)) {
      fs::remove_all(seq_directory, ec);
      if (ec) {
        RCLCPP_ERROR(this->get_logger(), "Failed to remove directory %s: %s",
                     seq_directory.c_str(), ec.message().c_str());
        return;
      }
    }
    fs::create_directories(scans_directory, ec);
    if (ec) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create directory %s: %s",
                   scans_directory.c_str(), ec.message().c_str());
      return;
    }

    std::ofstream kitti_pose_file(seq_directory + "/poses_kitti.txt");
    std::ofstream tum_pose_file(seq_directory + "/poses_tum.txt");
    if (!kitti_pose_file.is_open() || !tum_pose_file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open pose files in %s",
                   seq_directory.c_str());
      return;
    }
    tum_pose_file << "#timestamp x y z qx qy qz qw\n";
    // 遍历关键帧保存点云和位姿
    {
      std::lock_guard<std::mutex> lock(keyframes_mutex_);
      if (keyframes_.empty()) {
        RCLCPP_WARN(this->get_logger(),
                    "No keyframes, skip saving KITTI/TUM scans.");
        return;
      }
      for (size_t i = 0; i < keyframes_.size(); ++i) {
        // 保存 PCD
        std::stringstream ss_;
        ss_ << scans_directory << "/" << std::setw(6) << std::setfill('0') << i
            << ".pcd";
        // RCLCPP_INFO(this->get_logger(), "Saving %s...", ss_.str().c_str());
        if (pcl::io::savePCDFileBinary<PointType>(ss_.str(),
                                                  keyframes_[i].pcd_) != 0) {
          RCLCPP_WARN(this->get_logger(), "Failed to save %s",
                      ss_.str().c_str());
        }

        // 保存 KITTI 格式位姿
        const auto &pose_ = keyframes_[i].pose_corrected_eig_;
        kitti_pose_file << pose_(0, 0) << " " << pose_(0, 1) << " "
                        << pose_(0, 2) << " " << pose_(0, 3) << " "
                        << pose_(1, 0) << " " << pose_(1, 1) << " "
                        << pose_(1, 2) << " " << pose_(1, 3) << " "
                        << pose_(2, 0) << " " << pose_(2, 1) << " "
                        << pose_(2, 2) << " " << pose_(2, 3) << "\n";

        // 保存 TUM 格式位姿
        const auto &lidar_optim_pose_ =
            poseEigToPoseStamped(keyframes_[i].pose_corrected_eig_);
        tum_pose_file << std::fixed << std::setprecision(8)
                      << keyframes_[i].timestamp_ << " "
                      << lidar_optim_pose_.pose.position.x << " "
                      << lidar_optim_pose_.pose.position.y << " "
                      << lidar_optim_pose_.pose.position.z << " "
                      << lidar_optim_pose_.pose.orientation.x << " "
                      << lidar_optim_pose_.pose.orientation.y << " "
                      << lidar_optim_pose_.pose.orientation.z << " "
                      << lidar_optim_pose_.pose.orientation.w << "\n";
      }
    }

    kitti_pose_file.close();
    tum_pose_file.close();
    RCLCPP_INFO(
        this->get_logger(),
        "\033[32;1mScans and poses saved in .pcd and KITTI format\033[0m");
  }

  if (save_map_bag_) {
    std::string bag_path = package_path_ + "/result_bag";
    std::error_code ec;
    if (fs::exists(bag_path, ec)) {
      fs::remove_all(bag_path, ec);
      if (ec) {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to remove existing bag directory: %s",
                     ec.message().c_str());
        return;
      }
    }
    rosbag2_cpp::Writer writer;
    writer.open(bag_path);

    writer.create_topic({"/keyframe_pcd", "sensor_msgs/msg/PointCloud2",
                         rmw_get_serialization_format(), ""});

    writer.create_topic({"/keyframe_pose", "geometry_msgs/msg/PoseStamped",
                         rmw_get_serialization_format(), ""});

    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> cloud_serializer;
    rclcpp::Serialization<geometry_msgs::msg::PoseStamped> pose_serializer;

    {
      std::lock_guard<std::mutex> lock(keyframes_mutex_);

      for (size_t i = 0; i < keyframes_.size(); ++i) {
        auto cloud_msg = pclToPclRos(
            transformPcd(keyframes_[i].pcd_, keyframes_[i].pose_corrected_eig_),
            map_frame_);
        auto pose_msg =
            poseEigToPoseStamped(keyframes_[i].pose_corrected_eig_, map_frame_);

        const auto stamp_ns = static_cast<rcutils_time_point_value_t>(
            keyframes_[i].timestamp_ * 1e9);

        rclcpp::Time stamp(stamp_ns);
        cloud_msg.header.stamp = stamp;
        pose_msg.header.stamp = stamp;

        auto serialized_cloud = std::make_shared<rclcpp::SerializedMessage>();
        auto serialized_pose = std::make_shared<rclcpp::SerializedMessage>();

        cloud_serializer.serialize_message(&cloud_msg, serialized_cloud.get());
        pose_serializer.serialize_message(&pose_msg, serialized_pose.get());

        auto bag_cloud_msg =
            std::make_shared<rosbag2_storage::SerializedBagMessage>();
        bag_cloud_msg->topic_name = "/keyframe_pcd";
        bag_cloud_msg->time_stamp = stamp_ns;
        bag_cloud_msg->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
            new rcutils_uint8_array_t, [](rcutils_uint8_array_t *msg) {
              rcutils_uint8_array_fini(msg);
              delete msg;
            });
        *bag_cloud_msg->serialized_data =
            serialized_cloud->release_rcl_serialized_message();

        auto bag_pose_msg =
            std::make_shared<rosbag2_storage::SerializedBagMessage>();
        bag_pose_msg->topic_name = "/keyframe_pose";
        bag_pose_msg->time_stamp = stamp_ns;
        bag_pose_msg->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
            new rcutils_uint8_array_t, [](rcutils_uint8_array_t *msg) {
              rcutils_uint8_array_fini(msg);
              delete msg;
            });
        *bag_pose_msg->serialized_data =
            serialized_pose->release_rcl_serialized_message();

        writer.write(bag_cloud_msg);
        writer.write(bag_pose_msg);
      }
    }

    RCLCPP_INFO(this->get_logger(),
                "\033[36;1mResult saved in ROS2 bag format!!!\033[0m");
  }

  //// 2. 保存累积地图 PCD
  if (save_map_pcd_) {
    pcl::PointCloud<PointType>::Ptr corrected_map(
        new pcl::PointCloud<PointType>());

    {
      std::lock_guard<std::mutex> lock(keyframes_mutex_);
      if (keyframes_.empty()) {
        RCLCPP_WARN(this->get_logger(),
                    "No keyframes, skip saving globalmap.pcd.");
        return;
      }
      size_t reserve_size = 0;
      for (const auto &kf : keyframes_) {
        reserve_size += kf.pcd_.size();
      }
      corrected_map->reserve(reserve_size);
      for (size_t i = 0; i < keyframes_.size(); ++i) {
        *corrected_map +=
            transformPcd(keyframes_[i].pcd_, keyframes_[i].pose_corrected_eig_);
      }
    }

    auto voxelized_map = voxelizePcd(corrected_map, voxel_res_);
    const std::string result_path = package_path_ + "/globalmap.pcd";

    if (pcl::io::savePCDFileBinary<PointType>(result_path, *voxelized_map) !=
        0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to save result map: %s",
                   result_path.c_str());
      return;
    }

    RCLCPP_INFO(this->get_logger(),
                "\033[32;1mResult saved in .pcd format: %s, points: %zu\033[0m",
                result_path.c_str(), voxelized_map->size());
  }

  //发布保存完成
  std_msgs::msg::Empty done_msg;
  save_done_pub_->publish(done_msg);

  RCLCPP_INFO(this->get_logger(), "Map save finished, published /save_done.");
}

void PgoSc::saveMap() { saveFlagCallback(); }

// ====================== Keyframe Utilities ======================
void PgoSc::updateOdomsAndPaths(const PosePcd &pose_pcd_in) {
  odoms_.points.emplace_back(pose_pcd_in.pose_eig_(0, 3),
                             pose_pcd_in.pose_eig_(1, 3),
                             pose_pcd_in.pose_eig_(2, 3));
  corrected_odoms_.points.emplace_back(pose_pcd_in.pose_corrected_eig_(0, 3),
                                       pose_pcd_in.pose_corrected_eig_(1, 3),
                                       pose_pcd_in.pose_corrected_eig_(2, 3));
  odom_path_.poses.push_back(
      poseEigToPoseStamped(pose_pcd_in.pose_eig_, map_frame_));
  corrected_path_.poses.push_back(
      poseEigToPoseStamped(pose_pcd_in.pose_corrected_eig_, map_frame_));
}

visualization_msgs::msg::Marker PgoSc::getLoopMarkers(
    const gtsam::Values &corrected_esti_in) {
  visualization_msgs::msg::Marker edges;
  edges.type = visualization_msgs::msg::Marker::LINE_LIST;
  edges.scale.x = 0.12f;
  edges.header.frame_id = map_frame_;
  edges.pose.orientation.w = 1.0f;
  edges.color.r = edges.color.g = edges.color.b = edges.color.a = 1.0f;

  for (auto &pair : loop_idx_pairs_) {
    if (pair.first >= corrected_esti_in.size() ||
        pair.second >= corrected_esti_in.size())
      continue;

    gtsam::Pose3 p1 = corrected_esti_in.at<gtsam::Pose3>(pair.first);
    gtsam::Pose3 p2 = corrected_esti_in.at<gtsam::Pose3>(pair.second);
    geometry_msgs::msg::Point pt1, pt2;
    pt1.x = p1.translation().x();
    pt1.y = p1.translation().y();
    pt1.z = p1.translation().z();
    pt2.x = p2.translation().x();
    pt2.y = p2.translation().y();
    pt2.z = p2.translation().z();
    edges.points.push_back(pt1);
    edges.points.push_back(pt2);
  }
  return edges;
}

bool PgoSc::checkIfKeyframe(const PosePcd &pose_pcd_in,
                            const PosePcd &latest_pose_pcd) {
  return keyframe_thr_ <
         (latest_pose_pcd.pose_corrected_eig_.block<3, 1>(0, 3) -
          pose_pcd_in.pose_corrected_eig_.block<3, 1>(0, 3))
             .norm();
}
