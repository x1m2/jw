#include "map_matcher.hpp"

#include <rclcpp/rclcpp.hpp>
MapMatcher::MapMatcher(const MapMatcherConfig &config) {
  config_ = config;
  const auto &gc = config_.gicp_config_;
  const auto &qc = config_.quatro_config_;
  ////// nano_gicp init
  nano_gicp_.setNumThreads(gc.nano_thread_number_);
  nano_gicp_.setCorrespondenceRandomness(gc.nano_correspondences_number_);
  nano_gicp_.setMaximumIterations(gc.nano_max_iter_);
  nano_gicp_.setRANSACIterations(gc.nano_ransac_max_iter_);
  nano_gicp_.setMaxCorrespondenceDistance(gc.max_corr_dist_);
  nano_gicp_.setTransformationEpsilon(gc.transformation_epsilon_);
  nano_gicp_.setEuclideanFitnessEpsilon(gc.euclidean_fitness_epsilon_);
  nano_gicp_.setRANSACOutlierRejectionThreshold(
      gc.ransac_outlier_rejection_threshold_);
  ////// quatro init
  quatro_handler_ = std::make_shared<quatro<PointType>>(
      qc.fpfh_normal_radius_, qc.fpfh_radius_, qc.noise_bound_,
      qc.rot_gnc_factor_, qc.rot_cost_diff_thr_, qc.quatro_max_iter_,
      qc.estimat_scale_, qc.use_optimized_matching_,
      qc.quatro_distance_threshold_, qc.quatro_max_num_corres_);
  src_cloud_.reset(new pcl::PointCloud<PointType>);
  dst_cloud_.reset(new pcl::PointCloud<PointType>);
}

MapMatcher::~MapMatcher() {}

void MapMatcher::updateScancontext(pcl::PointCloud<PointType> cloud) {
  sc_manager_.makeAndSaveScancontextAndKeys(cloud);
}

int MapMatcher::fetchClosestKeyframeIdx(
    const PosePcd &front_keyframe,
    const std::vector<PosePcdReduced> &saved_map) {
  // from ScanContext, get the loop candidate
  std::pair<int, float> sc_detected_ = sc_manager_.detectLoopClosureIDGivenScan(
      front_keyframe.pcd_);  // int: nearest node index,
                             // float: relative yaw
  int candidate_keyframe_idx = sc_detected_.first;
  RCLCPP_INFO(rclcpp::get_logger("MapMatcher"),
              "ScanContext detected candidate keyframe idx: %d, relative yaw: "
              "%f",
              candidate_keyframe_idx, sc_detected_.second);
  if (candidate_keyframe_idx >= 0)  // if exists
  {
    // if close enough
    if ((saved_map[candidate_keyframe_idx].pose_eig_.block<3, 1>(0, 3) -
         front_keyframe.pose_corrected_eig_.block<3, 1>(0, 3))
            .norm() < config_.scancontext_max_correspondence_distance_) {
      return candidate_keyframe_idx;
    }
  }
  return -1;
}

PcdPair MapMatcher::setSrcAndDstCloud(
    const PosePcd &query_keyframe,
    const std::vector<PosePcdReduced> &saved_keyframes, const int dst_idx,
    const int submap_range, const double voxel_res, const bool enable_quatro) {
  pcl::PointCloud<PointType> dst_accum, src_out;
  int num_approx = saved_keyframes[dst_idx].pcd_.size() * 2 * submap_range;
  dst_accum.reserve(num_approx);

  src_out =
      transformPcd(query_keyframe.pcd_, query_keyframe.pose_corrected_eig_);
  if (enable_quatro) {
    dst_accum = saved_keyframes[dst_idx].pcd_;
  } else {
    // For ICP matching,
    // empirically scan-to-submap matching works better
    for (int i = dst_idx - submap_range; i < dst_idx + submap_range + 1; ++i) {
      if (i >= 0 && i < static_cast<int>(saved_keyframes.size())) {
        dst_accum += saved_keyframes[i].pcd_;
      }
    }
  }
  return {*voxelizePcd(src_out, voxel_res), *voxelizePcd(dst_accum, voxel_res)};
}

RegistrationOutput MapMatcher::icpAlignment(
    const pcl::PointCloud<PointType> &src,
    const pcl::PointCloud<PointType> &dst, bool global_reloc_done) {
  RegistrationOutput reg_output;
  aligned_.clear();
  // merge subkeyframes before ICP
  pcl::PointCloud<PointType>::Ptr src_cloud(new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr dst_cloud(new pcl::PointCloud<PointType>());
  *src_cloud = src;
  *dst_cloud = dst;
  nano_gicp_.setInputSource(src_cloud);
  nano_gicp_.calculateSourceCovariances();
  nano_gicp_.setInputTarget(dst_cloud);
  nano_gicp_.calculateTargetCovariances();
  nano_gicp_.align(aligned_);

  // handle results
  reg_output.score_ = nano_gicp_.getFitnessScore();
  // if matchness score is lower than threshold, (lower is better)
  float score_threshold = config_.gicp_config_.icp_score_thr_;
  if (!global_reloc_done) {
    score_threshold = config_.gicp_config_.reloc_icp_score_thr_;
  }
  RCLCPP_INFO(rclcpp::get_logger("MapMatcher"),
              "nano_gicp hasConverged: %d, score: %f, threshold: %f ",
              nano_gicp_.hasConverged(), reg_output.score_, score_threshold);
  if (nano_gicp_.hasConverged() && reg_output.score_ < score_threshold) {
    reg_output.is_valid_ = true;
    reg_output.is_converged_ = true;
    reg_output.pose_between_eig_ =
        nano_gicp_.getFinalTransformation().cast<double>();
  }
  return reg_output;
}

RegistrationOutput MapMatcher::coarseToFineAlignment(
    const pcl::PointCloud<PointType> &src,
    const pcl::PointCloud<PointType> &dst, bool global_reloc_done) {
  RegistrationOutput reg_output;
  coarse_aligned_.clear();
  // Quatro 做粗配准
  reg_output.pose_between_eig_ =
      (quatro_handler_->align(src, dst, reg_output.is_converged_));
  // 如果 Quatro 没有收敛，直接返回失败
  if (!reg_output.is_converged_) {
    RCLCPP_WARN(rclcpp::get_logger("MapMatcher"),
                "Quatro did not converge. Reject this match.");
    return reg_output;
  } else {  // if valid,
    // coarse align with the result of Quatro
    {
      Eigen::Matrix4d q_tf = reg_output.pose_between_eig_;

      Eigen::Vector3d q_t = q_tf.block<3, 1>(0, 3);
      Eigen::Vector3d q_rpy =
          q_tf.block<3, 3>(0, 0).eulerAngles(0, 1, 2) * 180.0 / M_PI;

      RCLCPP_WARN(rclcpp::get_logger("MapMatcher"),
                  "[QUATRO_DEBUG_RESULT] converged=%d, "
                  "trans=[%.3f %.3f %.3f], rpy_deg=[%.2f %.2f %.2f]",
                  reg_output.is_converged_, q_t.x(), q_t.y(), q_t.z(),
                  q_rpy.x(), q_rpy.y(), q_rpy.z());
    }
    coarse_aligned_ = transformPcd(src, reg_output.pose_between_eig_);
    // icp refine
    const auto &fine_output =
        icpAlignment(coarse_aligned_, dst, global_reloc_done);
    {
      Eigen::Matrix4d fine_tf = fine_output.pose_between_eig_;

      Eigen::Vector3d f_t = fine_tf.block<3, 1>(0, 3);
      Eigen::Vector3d f_rpy =
          fine_tf.block<3, 3>(0, 0).eulerAngles(0, 1, 2) * 180.0 / M_PI;

      RCLCPP_WARN(rclcpp::get_logger("MapMatcher"),
                  "[FINE_GICP_RESULT] valid=%d, converged=%d, score=%.6f, "
                  "fine_trans=[%.3f %.3f %.3f], fine_rpy_deg=[%.2f %.2f %.2f]",
                  fine_output.is_valid_, fine_output.is_converged_,
                  fine_output.score_, f_t.x(), f_t.y(), f_t.z(), f_rpy.x(),
                  f_rpy.y(), f_rpy.z());
    }
    const auto quatro_tf_ = reg_output.pose_between_eig_;
    reg_output = fine_output;
    reg_output.pose_between_eig_ = fine_output.pose_between_eig_ * quatro_tf_;
    {
      Eigen::Vector3d total_t = reg_output.pose_between_eig_.block<3, 1>(0, 3);
      Eigen::Vector3d total_rpy =
          reg_output.pose_between_eig_.block<3, 3>(0, 0).eulerAngles(0, 1, 2) *
          180.0 / M_PI;

      RCLCPP_WARN(rclcpp::get_logger("MapMatcher"),
                  "[COARSE_FINE_TOTAL_TF] valid=%d, score=%.6f, "
                  "total_trans=[%.3f %.3f %.3f], "
                  "total_rpy_deg=[%.2f %.2f %.2f]",
                  reg_output.is_valid_, reg_output.score_, total_t.x(),
                  total_t.y(), total_t.z(), total_rpy.x(), total_rpy.y(),
                  total_rpy.z());
    }
  }
  return reg_output;
}

RegistrationOutput MapMatcher::performMapMatcher(
    const PosePcd &query_keyframe,
    const std::vector<PosePcdReduced> &saved_keyframes,
    const int closest_keyframe_idx, bool global_reloc_done) {
  RegistrationOutput reg_output;
  closest_keyframe_idx_ = closest_keyframe_idx;
  if (closest_keyframe_idx_ >= 0) {
    // Quatro + NANO-GICP to check loop (from front_keyframe to closest
    // keyframe's neighbor)
    const auto &[src_cloud, dst_cloud] =
        setSrcAndDstCloud(query_keyframe, saved_keyframes,
                          closest_keyframe_idx_, config_.num_submap_keyframes_,
                          config_.voxel_res_, config_.enable_quatro_);
    // Only for visualization
    *src_cloud_ = src_cloud;
    *dst_cloud_ = dst_cloud;

    if (config_.enable_quatro_ && !global_reloc_done) {
      RCLCPP_INFO(rclcpp::get_logger("MapMatcher"),
                  "Execute quatro alignment: %d vs %d", src_cloud.size(),
                  dst_cloud.size());
      return coarseToFineAlignment(src_cloud, dst_cloud, global_reloc_done);
    } else {
      RCLCPP_INFO(rclcpp::get_logger("MapMatcher"),
                  "Execute GICP alignment: %d vs %d", src_cloud.size(),
                  dst_cloud.size());
      return icpAlignment(src_cloud, dst_cloud);
    }
  } else {
    return reg_output;  // dummy output whose `is_valid` is false
  }
}

pcl::PointCloud<PointType> MapMatcher::getSourceCloud() { return *src_cloud_; }

pcl::PointCloud<PointType> MapMatcher::getTargetCloud() { return *dst_cloud_; }

pcl::PointCloud<PointType> MapMatcher::getCoarseAlignedCloud() {
  return coarse_aligned_;
}

// NOTE(hlim): To cover ICP-only mode, I just set `Final`, not `Fine`
pcl::PointCloud<PointType> MapMatcher::getFinalAlignedCloud() {
  return aligned_;
}

int MapMatcher::getClosestKeyframeidx() { return closest_keyframe_idx_; }
