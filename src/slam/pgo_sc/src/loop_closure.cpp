#include "loop_closure.hpp"

#include <pcl/kdtree/kdtree_flann.h>

LoopClosure::LoopClosure(const LoopClosureConfig &config) {
  config_ = config;
  const auto &gc = config_.gicp_config_;
  const auto &qc = config_.quatro_config_;
  // ---------- Nano-GICP 初始化 ----------
  nano_gicp_.setNumThreads(gc.nano_thread_number_);
  nano_gicp_.setCorrespondenceRandomness(gc.nano_correspondences_number_);
  nano_gicp_.setMaximumIterations(gc.nano_max_iter_);
  nano_gicp_.setRANSACIterations(gc.nano_ransac_max_iter_);
  nano_gicp_.setMaxCorrespondenceDistance(gc.max_corr_dist_);
  nano_gicp_.setTransformationEpsilon(gc.transformation_epsilon_);
  nano_gicp_.setEuclideanFitnessEpsilon(gc.euclidean_fitness_epsilon_);
  nano_gicp_.setRANSACOutlierRejectionThreshold(
      gc.ransac_outlier_rejection_threshold_);
  // ---------- Quatro 粗配准初始化 ----------
  quatro_handler_ = std::make_shared<quatro<PointType>>(
      qc.fpfh_normal_radius_, qc.fpfh_radius_, qc.noise_bound_,
      qc.rot_gnc_factor_, qc.rot_cost_diff_thr_, qc.quatro_max_iter_,
      qc.estimat_scale_, qc.use_optimized_matching_,
      qc.quatro_distance_threshold_, qc.quatro_max_num_corres_);
  // 初始化源点云和目标点云指针
  src_cloud_.reset(new pcl::PointCloud<PointType>);
  dst_cloud_.reset(new pcl::PointCloud<PointType>);
}

LoopClosure::~LoopClosure() {}

// 原ScanContext候选检测实现保留，但RS模式下不参与编译。
#if 0
void LoopClosure::updateScancontext(pcl::PointCloud<PointType> cloud) {
  // 将当前帧点云生成并保存 ScanContext，用于后续回环检测
  sc_manager_.makeAndSaveScancontextAndKeys(cloud);
}
// -------------------------
// 获取回环候选关键帧索引
// -------------------------
int LoopClosure::fetchCandidateKeyframeIdx(
    const PosePcd &query_keyframe, const std::vector<PosePcd> &keyframes) {
  // 使用 ScanContext 检测回环候选帧
  std::pair<int, float> sc_detected_ = sc_manager_.detectLoopClosureIDGivenScan(
      query_keyframe.pcd_);  // int: nearest node index,
                             // float: relative yaw
  int candidate_keyframe_idx = sc_detected_.first;
  RCLCPP_INFO(rclcpp::get_logger("LoopClosure"),
              "ScanContext candidate keyframe index: %d, relative yaw: %.2f "
              "degrees",
              candidate_keyframe_idx, sc_detected_.second * 180.0 / M_PI);
  if (candidate_keyframe_idx >= 0)  // if exists
  {
    // if close enough
    if ((keyframes[candidate_keyframe_idx].pose_corrected_eig_.block<3, 1>(0,
                                                                           3) -
         query_keyframe.pose_corrected_eig_.block<3, 1>(0, 3))
            .norm() < config_.scancontext_max_correspondence_distance_) {
      return candidate_keyframe_idx;
    } else {
      RCLCPP_WARN(
          rclcpp::get_logger("LoopClosure"),
          "Candidate keyframe %d rejected by distance check. Distance: %.3f, "
          "Threshold: %.3f",
          candidate_keyframe_idx,
          (keyframes[candidate_keyframe_idx].pose_corrected_eig_.block<3, 1>(
               0, 3) -
           query_keyframe.pose_corrected_eig_.block<3, 1>(0, 3))
              .norm(),
          config_.scancontext_max_correspondence_distance_);
    }
  }
  return -1;  // 未找到合法候选帧
}
#endif

int LoopClosure::fetchRadiusSearchCandidateIdx(
    const PosePcd &query_keyframe,
    const std::vector<PosePcd> &keyframes) const {
  if (keyframes.size() < 2) return -1;

  pcl::PointCloud<pcl::PointXYZ>::Ptr key_positions(
      new pcl::PointCloud<pcl::PointXYZ>);
  key_positions->reserve(keyframes.size());
  for (const auto &keyframe : keyframes) {
    const Eigen::Vector3d position =
        keyframe.pose_corrected_eig_.block<3, 1>(0, 3);
    key_positions->emplace_back(static_cast<float>(position.x()),
                                static_cast<float>(position.y()), 0.0F);
  }
  // 将关键帧投影到 XY 平面建立 KD-Tree，高度差单独过滤。
  pcl::KdTreeFLANN<pcl::PointXYZ> position_tree;
  position_tree.setInputCloud(key_positions);

  // 查询点同样使用优化位姿并投影到 XY 平面。
  const Eigen::Vector3d query_position =
      query_keyframe.pose_corrected_eig_.block<3, 1>(0, 3);
  pcl::PointXYZ query_point(static_cast<float>(query_position.x()),
                            static_cast<float>(query_position.y()), 0.0F);
  std::vector<int> nearby_indices;
  std::vector<float> nearby_squared_distances;
  position_tree.radiusSearch(query_point,
                             config_.history_keyframe_search_xy_radius_,
                             nearby_indices, nearby_squared_distances);

  int best_candidate_idx = -1;
  float best_squared_distance = 0.0F;
  int best_keyframe_gap = 0;
  int old_keyframes_within_radius = 0;
  int nearest_z_rejected_idx = -1;
  double nearest_z_rejected_abs_dz = std::numeric_limits<double>::max();
  for (size_t i = 0; i < nearby_indices.size(); ++i) {
    const int candidate_idx = nearby_indices[i];
    if (candidate_idx < 0 ||
        candidate_idx >= static_cast<int>(keyframes.size()) ||
        candidate_idx == query_keyframe.idx_) {
      continue;
    }

    const int keyframe_gap = query_keyframe.idx_ - candidate_idx;
    if (keyframe_gap < config_.history_keyframe_min_index_diff_) continue;

    ++old_keyframes_within_radius;
    const double z_difference =
        std::abs(query_position.z() -
                 keyframes[candidate_idx].pose_corrected_eig_(2, 3));
    if (z_difference <= config_.history_keyframe_search_z_tolerance_) {
      best_candidate_idx = candidate_idx;
      best_squared_distance = nearby_squared_distances[i];
      best_keyframe_gap = keyframe_gap;
      break;
    }

    if (z_difference < nearest_z_rejected_abs_dz) {
      nearest_z_rejected_idx = candidate_idx;
      nearest_z_rejected_abs_dz = z_difference;
    }
  }

  // 找到候选时记录各项筛选数据；该阶段只表示“位置上可能回环”，
  // 最终是否接受仍由后续 Quatro/Nano-GICP 的配准结果决定。
  if (best_candidate_idx >= 0) {
    RCLCPP_INFO(
        rclcpp::get_logger("LoopClosure"),
        "[RS_LOOP_CANDIDATE] current=%d, history=%d, xy_distance=%.3f m "
        "< xy_radius=%.3f m, abs_dz=%.3f m <= z_tolerance=%.3f m, "
        "keyframe_gap=%d >= required=%d",
        query_keyframe.idx_, best_candidate_idx,
        std::sqrt(best_squared_distance),
        config_.history_keyframe_search_xy_radius_,
        std::abs(query_position.z() -
                 keyframes[best_candidate_idx].pose_corrected_eig_(2, 3)),
        config_.history_keyframe_search_z_tolerance_, best_keyframe_gap,
        config_.history_keyframe_min_index_diff_);
    return best_candidate_idx;
  }

  // 只记录接近 XY/Z 阈值的失败，便于现场调参。
  constexpr double kXyNearMissMargin = 5.0;
  constexpr double kZNearMissMargin = 1.0;
  int nearest_same_floor_idx = -1;
  double nearest_same_floor_xy = std::numeric_limits<double>::max();
  for (const auto &keyframe : keyframes) {
    const int keyframe_gap = query_keyframe.idx_ - keyframe.idx_;
    if (keyframe_gap < config_.history_keyframe_min_index_diff_) continue;

    const Eigen::Vector3d history_position =
        keyframe.pose_corrected_eig_.block<3, 1>(0, 3);
    const double abs_dz = std::abs(query_position.z() - history_position.z());
    if (abs_dz > config_.history_keyframe_search_z_tolerance_) continue;
    const double xy_distance =
        (query_position.head<2>() - history_position.head<2>()).norm();
    if (xy_distance < nearest_same_floor_xy) {
      nearest_same_floor_idx = keyframe.idx_;
      nearest_same_floor_xy = xy_distance;
    }
  }

  const double xy_radius = config_.history_keyframe_search_xy_radius_;
  const double z_tolerance = config_.history_keyframe_search_z_tolerance_;
  const bool xy_near_miss =
      nearest_same_floor_idx >= 0 && nearest_same_floor_xy > xy_radius &&
      nearest_same_floor_xy <= xy_radius + kXyNearMissMargin;
  const bool z_near_miss =
      old_keyframes_within_radius > 0 && nearest_z_rejected_idx >= 0 &&
      nearest_z_rejected_abs_dz > z_tolerance &&
      nearest_z_rejected_abs_dz <= z_tolerance + kZNearMissMargin;
  if (!xy_near_miss && !z_near_miss) return -1;

  const bool report_xy =
      xy_near_miss &&
      (!z_near_miss ||
       (nearest_same_floor_xy - xy_radius) / kXyNearMissMargin <=
           (nearest_z_rejected_abs_dz - z_tolerance) / kZNearMissMargin);
  const int diagnostic_history_idx =
      report_xy ? nearest_same_floor_idx : nearest_z_rejected_idx;
  const Eigen::Vector3d diagnostic_history_position =
      keyframes[diagnostic_history_idx].pose_corrected_eig_.block<3, 1>(0, 3);
  const double diagnostic_xy_distance =
      (query_position.head<2>() - diagnostic_history_position.head<2>()).norm();
  const double diagnostic_abs_dz =
      std::abs(query_position.z() - diagnostic_history_position.z());

  RCLCPP_WARN(rclcpp::get_logger("LoopClosure"),
              "[RS_LOOP_NEAR_MISS] current=%d, reason=%s, history=%d, "
              "xy_distance=%.3f m, xy_radius=%.3f m, abs_dz=%.3f m, "
              "z_tolerance=%.3f m",
              query_keyframe.idx_,
              report_xy ? "xy_distance_slightly_too_large"
                        : "z_difference_slightly_too_large",
              diagnostic_history_idx, diagnostic_xy_distance, xy_radius,
              diagnostic_abs_dz, z_tolerance);
  return -1;
}

PcdPair LoopClosure::setSrcAndDstCloud(const std::vector<PosePcd> &keyframes,
                                       const int src_idx, const int dst_idx,
                                       const int submap_range,
                                       const double voxel_res,
                                       const bool enable_quatro,
                                       const bool enable_submap_matching) {
  pcl::PointCloud<PointType> dst_accum, src_accum;
  int num_approx = keyframes[src_idx].pcd_.size() * 2 * submap_range;
  src_accum.reserve(num_approx);
  dst_accum.reserve(num_approx);
  if (enable_submap_matching) {
    // 当前子地图 ↔ 历史子地图
    for (int i = src_idx - submap_range; i < src_idx + submap_range + 1; ++i) {
      if (i >= 0 && i < static_cast<int>(keyframes.size())) {
        src_accum +=
            transformPcd(keyframes[i].pcd_, keyframes[i].pose_corrected_eig_);
      }
    }
    for (int i = dst_idx - submap_range; i < dst_idx + submap_range + 1; ++i) {
      if (i >= 0 && i < static_cast<int>(keyframes.size())) {
        dst_accum +=
            transformPcd(keyframes[i].pcd_, keyframes[i].pose_corrected_eig_);
      }
    }
  } else {
    // 仅使用单帧或 Quatro 对齐
    src_accum = transformPcd(keyframes[src_idx].pcd_,
                             keyframes[src_idx].pose_corrected_eig_);
    // if (enable_quatro) {
    //   //当前单帧 ↔ 历史单帧
    //   dst_accum = transformPcd(keyframes[dst_idx].pcd_,
    //                            keyframes[dst_idx].pose_corrected_eig_);
    // } else {
    // 当前单帧 ↔ 历史子地图
    for (int i = dst_idx - submap_range; i < dst_idx + submap_range + 1; ++i) {
      if (i >= 0 && i < static_cast<int>(keyframes.size())) {
        dst_accum +=
            transformPcd(keyframes[i].pcd_, keyframes[i].pose_corrected_eig_);
      }
    }
    // }
  }
  // Voxel 化点云，减小点云密度，提高配准效率
  return {*voxelizePcd(src_accum, voxel_res),
          *voxelizePcd(dst_accum, voxel_res)};
}

RegistrationOutput LoopClosure::icpAlignment(
    const pcl::PointCloud<PointType> &src,
    const pcl::PointCloud<PointType> &dst,
    const double fitness_score_threshold) {
  RegistrationOutput reg_output;
  aligned_.clear();
  // merge subkeyframes before ICP
  pcl::PointCloud<PointType>::Ptr src_cloud(new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr dst_cloud(new pcl::PointCloud<PointType>());
  *src_cloud = src;
  *dst_cloud = dst;
  // 设置 Nano-GICP 输入
  nano_gicp_.setInputSource(src_cloud);
  nano_gicp_.calculateSourceCovariances();
  nano_gicp_.setInputTarget(dst_cloud);
  nano_gicp_.calculateTargetCovariances();
  nano_gicp_.align(aligned_);  // 执行配准

  // 处理配准结果
  reg_output.score_ = nano_gicp_.getFitnessScore();
  // if matchness score is lower than threshold, (lower is better)
  if (nano_gicp_.hasConverged() &&
      reg_output.score_ < fitness_score_threshold) {
    reg_output.is_valid_ = true;
    reg_output.is_converged_ = true;
    reg_output.pose_between_eig_ =
        nano_gicp_.getFinalTransformation().cast<double>();
  }
  RCLCPP_WARN(rclcpp::get_logger("LoopClosure"),
              "[ICP RESULT] converged=%d, score=%.6f, threshold=%.6f, valid=%d",
              nano_gicp_.hasConverged(), reg_output.score_,
              fitness_score_threshold, reg_output.is_valid_);
  return reg_output;
}

// -------------------------
// Quatro 粗配准 + ICP 精细对齐
// -------------------------
RegistrationOutput LoopClosure::coarseToFineAlignment(
    const pcl::PointCloud<PointType> &src,
    const pcl::PointCloud<PointType> &dst,
    const double fitness_score_threshold) {
  RegistrationOutput reg_output;
  coarse_aligned_.clear();

  {
    auto printCloudInfo = [&](const char *name,
                              const pcl::PointCloud<PointType> &cloud) {
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(cloud, centroid);

      PointType min_pt, max_pt;
      pcl::getMinMax3D(cloud, min_pt, max_pt);

      RCLCPP_WARN(rclcpp::get_logger("MapMatcher"),
                  "[CLOUD_INFO] %s size=%zu centroid=[%.3f %.3f %.3f] "
                  "range_x=[%.3f %.3f] range_y=[%.3f %.3f]",
                  name, cloud.size(), centroid.x(), centroid.y(), centroid.z(),
                  min_pt.x, max_pt.x, min_pt.y, max_pt.y);
    };

    printCloudInfo("src", src);
    printCloudInfo("dst", dst);
  }
  constexpr double kRad2Deg = 180.0 / M_PI;
  auto logPose = [&](const char *tag, const auto &T) {
    const Eigen::Matrix3d R = T.template block<3, 3>(0, 0);

    const double roll = std::atan2(R(2, 1), R(2, 2)) * kRad2Deg;

    const double pitch =
        std::atan2(-R(2, 0), std::sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0))) *
        kRad2Deg;

    const double yaw = std::atan2(R(1, 0), R(0, 0)) * kRad2Deg;

    RCLCPP_INFO(rclcpp::get_logger("MapMatcher"),
                "[LOOP_REG_POSE] %s "
                "t=[%.3f %.3f %.3f], "
                "rpy=[%.2f %.2f %.2f] deg",
                tag, T(0, 3), T(1, 3), T(2, 3), roll, pitch, yaw);
  };

  // ==========================================================
  // 1. Quatro 粗配准
  // ==========================================================
  reg_output.pose_between_eig_ =
      quatro_handler_->align(src, dst, reg_output.is_converged_);

  if (!reg_output.is_converged_) {
    return reg_output;
  }

  const auto quatro_tf = reg_output.pose_between_eig_;

  logPose("quatro_tf_src_to_dst", quatro_tf);

  // ==========================================================
  // 2. 使用 Quatro 输出作为 ICP 初值
  // ==========================================================
  coarse_aligned_ = transformPcd(src, quatro_tf);

  const auto &fine_output =
      icpAlignment(coarse_aligned_, dst, fitness_score_threshold);

  logPose("icp_delta_coarse_to_dst", fine_output.pose_between_eig_);

  const auto final_tf = fine_output.pose_between_eig_ * quatro_tf;

  logPose("final_tf_src_to_dst", final_tf);

  reg_output = fine_output;
  reg_output.pose_between_eig_ = final_tf;

  return reg_output;
}

RegistrationOutput LoopClosure::performLoopClosure(
    const PosePcd &query_keyframe, const std::vector<PosePcd> &keyframes,
    const int closest_keyframe_idx) {
  RegistrationOutput reg_output;
  closest_keyframe_idx_ = closest_keyframe_idx;
  if (closest_keyframe_idx_ >= 0) {
    const double max_threshold = config_.gicp_config_.icp_score_thr_;
    double fitness_score_threshold = max_threshold;
    double elapsed_since_last_loop = -1.0;
    if (last_successful_loop_time_) {
      elapsed_since_last_loop =
          std::max(0.0, query_keyframe.timestamp_ - *last_successful_loop_time_);
      const double adaptation_time =
          std::max(config_.fitness_adaptation_time_sec_, 1e-6);
      const double ratio =
          std::min(elapsed_since_last_loop / adaptation_time, 1.0);
      fitness_score_threshold =
          config_.fitness_score_thr_min_ +
          (max_threshold - config_.fitness_score_thr_min_) * ratio;
    }

    RCLCPP_INFO(rclcpp::get_logger("LoopClosure"),
                "[LOOP_FITNESS_GATE] current=%d, elapsed_since_last=%.3f s, "
                "threshold=%.3f (min=%.3f, max=%.3f)",
                query_keyframe.idx_, elapsed_since_last_loop,
                fitness_score_threshold, config_.fitness_score_thr_min_,
                max_threshold);

    // Quatro + NANO-GICP to check loop (from front_keyframe to closest
    // keyframe's neighbor)
    const auto &[src_cloud, dst_cloud] = setSrcAndDstCloud(
        keyframes, query_keyframe.idx_, closest_keyframe_idx_,
        config_.num_submap_keyframes_, config_.voxel_res_,
        config_.enable_quatro_, config_.enable_submap_matching_);
    // Only for visualization
    *src_cloud_ = src_cloud;
    *dst_cloud_ = dst_cloud;

    if (config_.enable_quatro_) {
      RCLCPP_INFO(rclcpp::get_logger("MapMatcher"),
                  "Execute quatro alignment: %zu vs %zu", src_cloud.size(),
                  dst_cloud.size());
      reg_output = coarseToFineAlignment(src_cloud, dst_cloud,
                                         fitness_score_threshold);
    } else {
      RCLCPP_INFO(rclcpp::get_logger("MapMatcher"), "Execute GICP: %zu vs %zu",
                  src_cloud.size(), dst_cloud.size());
      reg_output =
          icpAlignment(src_cloud, dst_cloud, fitness_score_threshold);
    }
    if (reg_output.is_valid_) {
      last_successful_loop_time_ = query_keyframe.timestamp_;
    }
    return reg_output;
  } else {
    return reg_output;  // dummy output whose `is_valid` is false
  }
}

pcl::PointCloud<PointType> LoopClosure::getSourceCloud() { return *src_cloud_; }

pcl::PointCloud<PointType> LoopClosure::getTargetCloud() { return *dst_cloud_; }

pcl::PointCloud<PointType> LoopClosure::getCoarseAlignedCloud() {
  return coarse_aligned_;
}

// NOTE(hlim): To cover ICP-only mode, I just set `Final`, not `Fine`
pcl::PointCloud<PointType> LoopClosure::getFinalAlignedCloud() {
  return aligned_;
}

int LoopClosure::getClosestKeyframeidx() { return closest_keyframe_idx_; }
