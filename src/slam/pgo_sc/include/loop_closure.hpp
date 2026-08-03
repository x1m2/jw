#ifndef PGO_SC_LOOP_CLOSURE_HPP_
#define PGO_SC_LOOP_CLOSURE_HPP_
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <quatro/quatro_module.h>

// ScanContext候选检测已停用；保留源码，恢复时同时启用对应接口和实现。
#if 0
#include <scancontext/Scancontext.h>
#endif

#include <Eigen/Eigen>
#include <iostream>
#include <limits>
#include <memory>
#include <nano_gicp/nano_gicp.hpp>
#include <nano_gicp/point_type_nano_gicp.hpp>
#include <tuple>
#include <utility>
#include <vector>

#include "pose_pcd.hpp"
#include "utilities.hpp"

using PcdPair =
    std::tuple<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>>;

struct NanoGICPConfig {
  int nano_thread_number_ = 0;
  int nano_correspondences_number_ = 15;
  int nano_max_iter_ = 32;
  int nano_ransac_max_iter_ = 5;
  double max_corr_dist_ = 2.0;
  double icp_score_thr_ = 10.0;
  double transformation_epsilon_ = 0.01;
  double euclidean_fitness_epsilon_ = 0.01;
  double ransac_outlier_rejection_threshold_ = 1.0;
};

struct QuatroConfig {
  bool use_optimized_matching_ = true;
  bool estimat_scale_ = false;
  int quatro_max_num_corres_ = 500;
  int quatro_max_iter_ = 50;
  double quatro_distance_threshold_ = 30.0;
  double fpfh_normal_radius_ = 0.30;
  double fpfh_radius_ = 0.50;
  double noise_bound_ = 0.30;
  double rot_gnc_factor_ = 1.40;
  double rot_cost_diff_thr_ = 0.0001;
};

struct LoopClosureConfig {
  bool enable_quatro_ = true;
  bool enable_submap_matching_ = true;
  int num_submap_keyframes_ = 10;
  double voxel_res_ = 0.1;
  double scancontext_max_correspondence_distance_;
  double history_keyframe_search_xy_radius_ = 15.0;
  double history_keyframe_search_z_tolerance_ = 1.5;
  int history_keyframe_min_index_diff_ = 30;
  NanoGICPConfig gicp_config_;
  QuatroConfig quatro_config_;
};

struct RegistrationOutput {
  bool is_valid_ = false;
  bool is_converged_ = false;
  double score_ = std::numeric_limits<double>::max();
  Eigen::Matrix4d pose_between_eig_ = Eigen::Matrix4d::Identity();
};

class LoopClosure {
 public:
  explicit LoopClosure(const LoopClosureConfig& config);
  ~LoopClosure();
  // 原ScanContext候选接口，当前不参与编译。
#if 0
  void updateScancontext(pcl::PointCloud<PointType> cloud);
  int fetchCandidateKeyframeIdx(const PosePcd& query_keyframe,
                                const std::vector<PosePcd>& keyframes);
#endif
  // 使用原始前端位姿进行三维半径搜索，并排除时间上过近的历史关键帧。
  int fetchRadiusSearchCandidateIdx(
      const PosePcd& query_keyframe,
      const std::vector<PosePcd>& keyframes) const;
  // 设置源点云和目标点云
  PcdPair setSrcAndDstCloud(const std::vector<PosePcd>& keyframes,
                            const int src_idx, const int dst_idx,
                            const int submap_range, const double voxel_res,
                            const bool enable_quatro,
                            const bool enable_submap_matching);
  // ICP 精细对齐
  RegistrationOutput icpAlignment(const pcl::PointCloud<PointType>& src,
                                  const pcl::PointCloud<PointType>& dst);
  // Quatro 粗配准 + ICP 精细对齐
  RegistrationOutput coarseToFineAlignment(
      const pcl::PointCloud<PointType>& src,
      const pcl::PointCloud<PointType>& dst);
  // 执行闭环检测
  RegistrationOutput performLoopClosure(const PosePcd& query_keyframe,
                                        const std::vector<PosePcd>& keyframes,
                                        const int closest_keyframe_idx);
  // 获取内部点云（用于可视化）
  pcl::PointCloud<PointType> getSourceCloud();
  pcl::PointCloud<PointType> getTargetCloud();
  pcl::PointCloud<PointType> getCoarseAlignedCloud();
  pcl::PointCloud<PointType> getFinalAlignedCloud();
  int getClosestKeyframeidx();

 private:
#if 0
  SCManager sc_manager_;
#endif
  nano_gicp::NanoGICP<PointType, PointType> nano_gicp_;
  std::shared_ptr<quatro<PointType>> quatro_handler_ = nullptr;
  int closest_keyframe_idx_ = -1;
  pcl::PointCloud<PointType>::Ptr src_cloud_;
  pcl::PointCloud<PointType>::Ptr dst_cloud_;
  pcl::PointCloud<PointType> coarse_aligned_;
  pcl::PointCloud<PointType> aligned_;
  LoopClosureConfig config_;
};

#endif  // PGO_SC__LOOP_CLOSURE_HPP_
