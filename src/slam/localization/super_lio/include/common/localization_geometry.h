#pragma once
#include <Eigen/Geometry>
#include <cmath>

namespace localization_geometry {
inline bool gravityFromImu(const Eigen::Vector3d &mean_acc, Eigen::Matrix4d &transform) {
  transform.setIdentity();
  if (!mean_acc.allFinite() || mean_acc.norm() <= 1e-3) return false;
  transform.topLeftCorner<3, 3>() = Eigen::Quaterniond::FromTwoVectors(
      (-mean_acc).normalized(), -Eigen::Vector3d::UnitZ()).normalized().toRotationMatrix();
  return true;
}
inline Eigen::Matrix4d baseFromImuPose(const Eigen::Matrix4d &map_imu,
    const Eigen::Matrix4d &imu_lidar, const Eigen::Matrix4d &base_lidar) {
  return map_imu * imu_lidar * base_lidar.inverse();
}
inline Eigen::Matrix4d imuFromBasePose(const Eigen::Matrix4d &map_base,
    const Eigen::Matrix4d &imu_lidar, const Eigen::Matrix4d &base_lidar) {
  return map_base * base_lidar * imu_lidar.inverse();
}
}
