#pragma once

#include <Eigen/Geometry>
#include <cstddef>
#include <mutex>

// One immutable gravity alignment per System lifetime. Consumers take copies
// so relocalization workers never access the accumulating IMU samples.
class GravityInitialization {
 public:
  struct Result {
    Eigen::Vector3d mean_acc = Eigen::Vector3d::Zero();
    Eigen::Matrix4d gravity_from_imu = Eigen::Matrix4d::Identity();
    std::size_t sample_count = 0;
    bool ready = false;
  };

  bool add_sample(const Eigen::Vector3d &acc, std::size_t required_samples) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (result_.ready) return true;
    if (!acc.allFinite()) return false;
    ++result_.sample_count;
    result_.mean_acc += (acc - result_.mean_acc) /
                        static_cast<double>(result_.sample_count);
    if (result_.sample_count < required_samples) return false;
    return align();
  }

  Result snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return result_;
  }

 private:
  bool align() {
    if (result_.mean_acc.norm() <= 1e-3) return false;
    result_.gravity_from_imu.topLeftCorner<3, 3>() =
        Eigen::Quaterniond::FromTwoVectors(
            (-result_.mean_acc).normalized(), Eigen::Vector3d(0, 0, -1))
            .normalized().toRotationMatrix();
    result_.ready = true;
    return true;
  }

  mutable std::mutex mutex_;
  Result result_;
};
