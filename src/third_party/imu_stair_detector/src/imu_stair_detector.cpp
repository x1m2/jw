#include "imu_stair_detector/imu_stair_detector.hpp"

#include <algorithm>
#include <cmath>

namespace imu_stair_detector {
namespace {
constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;
bool validQuaternion(const Eigen::Quaterniond &q) {
  return q.coeffs().allFinite() && std::isfinite(q.norm()) && q.norm() > 1e-12;
}
}  // namespace

bool ImuStairDetector::configure(const Config &config) {
  reset();
  if (!validQuaternion(config.imu_to_base) ||
      !std::isfinite(config.pitch_baseline_deg) ||
      std::abs(config.pitch_baseline_deg) > 90.0 ||
      !std::isfinite(config.enter_angle_deg) || config.enter_angle_deg <= 0.0 ||
      config.enter_angle_deg > 90.0 || !std::isfinite(config.exit_angle_deg) ||
      config.exit_angle_deg < 0.0 || config.exit_angle_deg >= config.enter_angle_deg ||
      !std::isfinite(config.filter_time_constant_sec) || config.filter_time_constant_sec < 0.0 ||
      !std::isfinite(config.enter_duration_sec) || config.enter_duration_sec < 0.0 ||
      !std::isfinite(config.exit_duration_sec) || config.exit_duration_sec < 0.0 ||
      !std::isfinite(config.timeout_sec) || config.timeout_sec <= 0.0) {
    return false;
  }
  config_ = config;
  config_.imu_to_base.normalize();
  return true;
}

void ImuStairDetector::reset() {
  status_ = {};
  last_sample_time_.reset();
  transition_since_.reset();
}

void ImuStairDetector::update(const Eigen::Quaterniond &world_from_imu,
                            double sample_time) {
  if (!validQuaternion(world_from_imu) || !std::isfinite(sample_time)) {
    reset();
    return;
  }
  if (last_sample_time_ && sample_time <= *last_sample_time_) {
    reset();
    return;
  }
  if (last_sample_time_ && sample_time - *last_sample_time_ > config_.timeout_sec) {
    reset();
  }
  // R_world_base = R_world_imu * R_imu_base.
  const Eigen::Quaterniond world_from_base =
      world_from_imu.normalized() * config_.imu_to_base.conjugate();
  const auto rotation = world_from_base.toRotationMatrix();
  const double pitch = std::atan2(-rotation(2, 0),
      std::hypot(rotation(0, 0), rotation(1, 0))) * kRadiansToDegrees;
  if (!last_sample_time_) {
    status_.pitch_deg = pitch;
  } else {
    const double dt = sample_time - *last_sample_time_;
    const double alpha = config_.filter_time_constant_sec == 0.0 ? 1.0 :
        -std::expm1(-dt / config_.filter_time_constant_sec);
    status_.pitch_deg += alpha * (pitch - status_.pitch_deg);
  }
  status_.valid = true;
  last_sample_time_ = sample_time;
  const double deviation = std::abs(status_.pitch_deg - config_.pitch_baseline_deg);
  const bool transition = status_.on_stairs ? deviation <= config_.exit_angle_deg :
                                            deviation >= config_.enter_angle_deg;
  if (!transition) {
    transition_since_.reset();
    return;
  }
  if (!transition_since_) transition_since_ = sample_time;
  const double duration = status_.on_stairs ? config_.exit_duration_sec : config_.enter_duration_sec;
  if (sample_time - *transition_since_ >= duration) {
    status_.on_stairs = !status_.on_stairs;
    transition_since_.reset();
  }
}

Status ImuStairDetector::status(double now) {
  if (!std::isfinite(now) || !last_sample_time_ || now < *last_sample_time_ ||
      now - *last_sample_time_ > config_.timeout_sec) {
    reset();
  }
  return status_;
}

}  // namespace imu_stair_detector
