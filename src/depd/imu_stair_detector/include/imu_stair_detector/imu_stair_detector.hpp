#pragma once

#include <Eigen/Geometry>
#include <optional>

namespace imu_stair_detector {

struct Config {
  // R_base_imu: rotates vectors from the IMU frame into the robot frame.
  Eigen::Quaterniond imu_to_base{Eigen::Quaterniond::Identity()};
  double pitch_baseline_deg{0.0};
  double enter_angle_deg{14.0};
  double exit_angle_deg{10.0};
  double filter_time_constant_sec{0.15};
  double enter_duration_sec{0.15};
  double exit_duration_sec{0.40};
  double timeout_sec{0.30};
};

struct Status {
  bool valid{false};
  bool on_stairs{false};
  double pitch_deg{0.0};
};

// Pure algorithm; the caller supplies seconds on one clock and synchronizes access.
// Detects pitch inclination, including ramps; it does not identify stair geometry.
class ImuStairDetector {
 public:
  bool configure(const Config &config);
  void reset();
  void update(const Eigen::Quaterniond &world_from_imu, double sample_time);
  // Expired or reversed time clears all history, requiring a new confirmation.
  Status status(double now);

 private:
  Config config_;
  Status status_;
  std::optional<double> last_sample_time_;
  std::optional<double> transition_since_;
};

}  // namespace imu_stair_detector
