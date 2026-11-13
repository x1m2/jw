#pragma once
#include <deque>
#include "backend/Header.h"
// #define ENU

struct GnssPose
{
  GnssPose(const double &time = 0, const V3D &pos = ZERO3D, const QD &rot = EYEQD,
           const Eigen::VectorXd &cov = Eigen::VectorXd::Zero(6))
      : timestamp(time), gnss_position(pos), gnss_quat(rot.normalized()), covariance(cov) {}

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double timestamp;
  V3D gnss_position;
  QD gnss_quat;
  Eigen::VectorXd covariance;

  float current_gnss_interval;
  V3D lidar_pos_fix; // utm add extrinsic
};
