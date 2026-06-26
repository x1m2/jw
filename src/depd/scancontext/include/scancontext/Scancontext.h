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
using KeyMat = std::vector<std::vector<float>>;
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
  // hyper parameters ()
  // 当前点云直接使用雷达自身坐标系时，填实际雷达离地高度；
  // 若点云已经转换到机器人基座坐标系，且地面高度已处理，可设为 0。
  const double LIDAR_HEIGHT = 2.0;

  const int PC_NUM_RING = 20;  // Scan Context 极坐标描述子的环数。# 20
  const int PC_NUM_SECTOR = 60;  // Scan Context 极坐标描述子的扇区数 #60
  double PC_MAX_RADIUS = 80.0;  // Scan Context 描述子的最大有效半径 #80
  const double PC_UNIT_SECTORANGLE =
      360.0 / double(PC_NUM_SECTOR);  // 每个扇区对应的角度分辨率（单位：deg）。
  const double PC_UNIT_RINGGAP =
      PC_MAX_RADIUS / double(PC_NUM_RING);  // 每个环对应的半径间隔（单位：m）。

  // tree
  const int NUM_EXCLUDE_RECENT =
      30;  // 最近若干关键帧不参与回环候选检索。  //
           // 作用：避免当前帧和最近轨迹帧被误判为回环。
  const int NUM_CANDIDATES_FROM_TREE =
      3;  // 从 KD-Tree 中取出的候选数量。  // 一般 3~10 都可以，当前 3
          // 个候选速度更快。

  // loop thres
  // 快速搜索时的候选 yaw 搜索比例。
  // 0.1 表示只搜索全部扇区偏移量中的约 10%，
  // 用于避免完整暴力搜索，提高匹配速度。
  const double SEARCH_RATIO = 0.1;

  // 对于 20×60 的极坐标 Scan Context 描述子，经验上阈值设为 0.1~0.2
  // 通常较合适， 误回环较少。 但当阈值大于 0.15
  // 时，为了保证鲁棒性，建议额外使用 DCS、 或 ICP 拟合分数检查（例如 LeGO-LOAM
  // 中的做法）进行二次验证。

  double SC_DIST_THRES = 0.2;
  // 当后端使用鲁棒核函数（例如 Cauchy、DCS）
  // 并且结合 ICP fitness score 阈值进行验证时，
  // 0.4~0.6 是一个较合适的选择；
  // 若没有这些额外的鲁棒验证机制，则建议使用 0.1~0.15。

  // config
  // 当前系统可根据关键帧产生速度和计算量调小，例如 5~15。
  const int TREE_MAKING_PERIOD_ =
      30;  // i.e., remaking tree frequency, to avoid non-mandatory every
           // remaking, to save time cost / in the LeGO-LOAM integration, it is
           // synchronized with the loop detection callback (which is 1Hz) so it
           // means the tree is updated evrey 10 sec. But you can use the
           // smaller value because it is enough fast ~ 5-50ms wrt N.
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
