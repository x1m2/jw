#pragma once

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <memory>
#include <utility>
#include <vector>

#include "KDTreeVectorOfVectorsAdaptor.h"
#include "nanoflann.hpp"

using namespace Eigen;
using namespace nanoflann;

using std::cout;
using std::endl;
using std::make_pair;

using std::atan2;
using std::cos;
using std::sin;

using SCPointType =
    pcl::PointXYZI;  // using xyz only. but a user can exchange the original bin
                     // encoding function (i.e., max hegiht) to max intensity
                     // (for detail, refer 20 ICRA Intensity Scan Context)
using KeyMat = std::vector<std::vector<float> >;
using InvKeyTree = KDTreeVectorOfVectorsAdaptor<KeyMat, float>;

// namespace SC2
// {

void coreImportTest(void);

// sc param-independent helper functions
float xy2theta(const float &_x, const float &_y);
MatrixXd circshift(MatrixXd &_mat, int _num_shift);
std::vector<float> eig2stdvec(MatrixXd _eigmat);

class SCManager {
 public:
  SCManager() =
      default;  // reserving data space (of std::vector) could be considered.
                // but the descriptor is lightweight so don't care.

  Eigen::MatrixXd makeScancontext(
      const pcl::PointCloud<SCPointType> &_scan_down);
  Eigen::MatrixXd makeRingkeyFromScancontext(Eigen::MatrixXd &_desc);
  Eigen::MatrixXd makeSectorkeyFromScancontext(Eigen::MatrixXd &_desc);

  int fastAlignUsingVkey(MatrixXd &_vkey1, MatrixXd &_vkey2);
  double distDirectSC(
      MatrixXd &_sc1,
      MatrixXd &_sc2);  // "d" (eq 5) in the original paper (IROS 18)
  std::pair<double, int> distanceBtnScanContext(
      MatrixXd &_sc1,
      MatrixXd &_sc2);  // "D" (eq 6) in the original paper (IROS 18)

  // User-side API
  void makeAndSaveScancontextAndKeys(
      const pcl::PointCloud<SCPointType> &_scan_down);
  std::pair<int, float> detectLoopClosureID(
      void);  // int: nearest node index, float: relative yaw
  std::pair<int, float> detectLoopClosureIDGivenScan(
      const pcl::PointCloud<SCPointType>
          &_scan_down);  // int: nearest node index, float: relative yaw

  // for ltslam
  // User-side API for multi-session
  void saveScancontextAndKeys(Eigen::MatrixXd _scd);
  std::pair<int, float> detectLoopClosureIDBetweenSession(
      std::vector<float> &curr_key, Eigen::MatrixXd &curr_desc);

  const Eigen::MatrixXd &getConstRefRecentSCD(void);

 public:
  // ====================== ScanContext 超参数 ======================

  // 雷达安装高度。
  // 如果直接使用雷达自身坐标系下的点云，可以设置为实际雷达高度，
  // 这里会简单地把点云高度补偿到类似机器人坐标系的效果。
  // 如果输入点云已经转换到了 robot/base_link 坐标系，建议设置为 0。
  const double LIDAR_HEIGHT = 2.0;

  // ScanContext 极坐标描述子的环数。
  // 原论文 IROS 2018 中使用 20。
  // 环数越大，径向分辨率越高，但计算量也会增加。
  const int PC_NUM_RING = 20;

  // ScanContext 极坐标描述子的扇区数。
  // 原论文 IROS 2018 中使用 60。
  // 扇区数越大，角度分辨率越高。
  const int PC_NUM_SECTOR = 60;

  // ScanContext 最大感知半径，单位：米。
  // 原论文中使用 80m。
  // 超过该半径的点不会参与 ScanContext 描述子构建。
  double PC_MAX_RADIUS = 80.0;

  // 每个扇区对应的角度，单位：度。
  // 例如 60 个扇区，则每个扇区是 6 度。
  const double PC_UNIT_SECTORANGLE = 360.0 / double(PC_NUM_SECTOR);

  // 每个 ring 的径向宽度，单位：米。
  // 例如最大半径 80m，20 个 ring，则每个 ring 宽度为 4m。
  const double PC_UNIT_RINGGAP = PC_MAX_RADIUS / double(PC_NUM_RING);

  // ====================== KDTree 检索参数 ======================

  // 排除最近的关键帧数量。
  // 作用：避免刚经过的相邻关键帧被误认为回环。
  // 这里的 30 表示最近 30 个关键帧不参与回环候选搜索。
  const int NUM_EXCLUDE_RECENT = 30;

  // KDTree 粗检索返回的候选数量。
  // 原论文中常用 10 个候选。
  const int NUM_CANDIDATES_FROM_TREE = 10;  // 3

  // ====================== ScanContext 回环阈值参数 ======================

  // 快速比较比例。
  // 用于 ScanContext 距离计算时减少暴力搜索量。
  // 0.1 表示搜索约 10% 的列偏移候选，兼顾速度和效果。
  const double SEARCH_RATIO = 0.1;

  // ScanContext 距离阈值。
  // min_dist < SC_DIST_THRES 时，认为检测到回环候选。
  // 注意：ScanContext 距离越小，两个点云越相似。
  //
  // 经验说明：
  // 1. 如果后端有鲁棒核，例如 Cauchy、DCS，
  //    并且后面还有 ICP / GICP fitness score 阈值过滤，
  //    可以适当放宽到 0.4 ~ 0.6。
  // 2. 如果后端没有鲁棒核，或者错误回环风险较高，
  //    建议设置严格一些，例如 0.1 ~ 0.15。
  //
  // 如果你现在出现“位姿距离起点很近，但 ScanContext 返回 -1”，
  // 可以先排查性地试 0.25、0.30、0.35，
  // 但最终是否采用要看 Quatro / GICP 是否稳定通过。
  double SC_DIST_THRES = 0.35;  // 0.2

  // KDTree 重建周期。
  // 为了节省计算量，不需要每次检测回环都重建 KDTree。
  // 这里每 30 次调用重建一次。
  // 在 LeGO-LOAM 集成中，回环检测通常 1Hz，
  // 如果设置为 30，约等于每 30 秒重建一次。
  // 但 KDTree 重建通常较快，约 5~50ms，
  // 如果希望新关键帧更快参与回环检测，可以适当减小该值，例如 5 或 10。
  const int TREE_MAKING_PERIOD_ = 30;

  // KDTree 重建计数器。
  // 每次调用 detectLoopClosureIDGivenScan() 后递增，
  // 当 tree_making_period_conter % TREE_MAKING_PERIOD_ == 0 时重建 KDTree。
  int tree_making_period_conter = 0;

  // setter
  void setSCdistThres(double _new_thres);
  void setMaximumRadius(double _max_r);

  // data
  std::vector<double> polarcontexts_timestamp_;  // optional.
  std::vector<Eigen::MatrixXd> polarcontexts_;
  std::vector<Eigen::MatrixXd> polarcontext_invkeys_;
  std::vector<Eigen::MatrixXd> polarcontext_vkeys_;

  KeyMat polarcontext_invkeys_mat_;
  KeyMat polarcontext_invkeys_to_search_;
  std::unique_ptr<InvKeyTree> polarcontext_tree_;

  bool is_tree_batch_made = false;
  std::unique_ptr<InvKeyTree> polarcontext_tree_batch_;

};  // SCManager

// } // namespace SC2
