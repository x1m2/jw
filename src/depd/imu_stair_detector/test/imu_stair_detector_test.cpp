#include <gtest/gtest.h>
#include <limits>
#include "imu_stair_detector/imu_stair_detector.hpp"

namespace {
using imu_stair_detector::Config;
using imu_stair_detector::ImuStairDetector;
Eigen::Quaterniond pitch(double degrees) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(degrees * 3.14159265358979323846 / 180.0,
                                            Eigen::Vector3d::UnitY()));
}
Config immediateConfig() {
  Config c;
  c.filter_time_constant_sec = 0.0;
  c.enter_duration_sec = 0.0;
  c.exit_duration_sec = 0.0;
  return c;
}
TEST(ImuStairs, DetectsBothPitchSignsAndRetainsHysteresisBand) {
  for (double sign : {-1.0, 1.0}) {
    ImuStairDetector d;
    ASSERT_TRUE(d.configure(immediateConfig()));
    EXPECT_FALSE(d.status(1.0).valid);
    d.update(pitch(15.0 * sign), 1.0);
    EXPECT_TRUE(d.status(1.0).on_stairs);
    d.update(pitch(12.0 * sign), 1.1);
    EXPECT_TRUE(d.status(1.1).on_stairs);
    d.update(pitch(9.0 * sign), 1.2);
    EXPECT_FALSE(d.status(1.2).on_stairs);
    d.update(pitch(12.0 * sign), 1.3);
    EXPECT_FALSE(d.status(1.3).on_stairs);
  }
}
TEST(ImuStairs, RequiresContinuousEntryAndExitDurations) {
  ImuStairDetector d;
  auto c = immediateConfig();
  c.enter_duration_sec = 0.15;
  c.exit_duration_sec = 0.20;
  ASSERT_TRUE(d.configure(c));
  d.update(pitch(20), 1.0);
  d.update(pitch(0), 1.1);  // Short vibration cancels entry.
  d.update(pitch(20), 1.2);
  EXPECT_FALSE(d.status(1.2).on_stairs);
  d.update(pitch(20), 1.4);
  EXPECT_TRUE(d.status(1.4).on_stairs);
  d.update(pitch(0), 1.5);
  d.update(pitch(12), 1.6);  // Hysteresis band cancels exit.
  d.update(pitch(0), 1.7);
  EXPECT_TRUE(d.status(1.7).on_stairs);
  d.update(pitch(0), 1.95);
  EXPECT_FALSE(d.status(1.95).on_stairs);
}
TEST(ImuStairs, TimeoutGapInvalidQuaternionAndReversedTimeClearBypass) {
  ImuStairDetector d;
  auto c = immediateConfig();
  c.enter_duration_sec = 0.1;
  ASSERT_TRUE(d.configure(c));
  d.update(pitch(20), 1.0);
  d.update(pitch(20), 1.2);
  ASSERT_TRUE(d.status(1.2).on_stairs);
  EXPECT_FALSE(d.status(1.6).valid);
  d.update(pitch(20), 1.7);
  EXPECT_FALSE(d.status(1.7).on_stairs);
  d.update(pitch(20), 1.9);
  ASSERT_TRUE(d.status(1.9).on_stairs);
  d.update(pitch(20), 2.3); // Gap without an intervening status call.
  EXPECT_FALSE(d.status(2.3).on_stairs);
  d.update(pitch(20), 2.5);
  d.update(pitch(20), 2.4);
  EXPECT_FALSE(d.status(2.5).valid);
  d.update(Eigen::Quaterniond(0, 0, 0, 0), 2.6);
  EXPECT_FALSE(d.status(2.6).valid);
  d.update(pitch(20), 3.0);
  EXPECT_FALSE(d.status(2.9).valid); // Clock jumps backwards.
}
TEST(ImuStairs, CompensatesGeneralMountRotationAndQuaternionScale) {
  ImuStairDetector d;
  auto c = immediateConfig();
  c.imu_to_base = Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitX());
  ASSERT_TRUE(d.configure(c));
  auto q = pitch(-20) * c.imu_to_base;
  q.coeffs() *= 2.0;
  d.update(q, 1.0);
  EXPECT_NEAR(d.status(1.0).pitch_deg, -20, 1e-9);
  EXPECT_TRUE(d.status(1.0).on_stairs);
  d.update(c.imu_to_base, 1.1);
  EXPECT_NEAR(d.status(1.1).pitch_deg, 0, 1e-9);
  EXPECT_FALSE(d.status(1.1).on_stairs);
}
TEST(ImuStairs, FilteringUsesElapsedTimeAndBaseline) {
  ImuStairDetector a, b;
  auto c = immediateConfig();
  c.filter_time_constant_sec = 0.2;
  c.pitch_baseline_deg = 5.0;
  ASSERT_TRUE(a.configure(c));
  ASSERT_TRUE(b.configure(c));
  a.update(pitch(5), 1.0);
  b.update(pitch(5), 1.0);
  a.update(pitch(25), 1.1);
  a.update(pitch(25), 1.2);
  b.update(pitch(25), 1.2);
  EXPECT_NEAR(a.status(1.2).pitch_deg, b.status(1.2).pitch_deg, 1e-9);
  EXPECT_FALSE(a.status(1.2).on_stairs);
}
TEST(ImuStairs, InvalidConfigurationAndNonFiniteInput) {
  ImuStairDetector d;
  auto c = immediateConfig();
  c.exit_angle_deg = c.enter_angle_deg;
  EXPECT_FALSE(d.configure(c));
  c = immediateConfig();
  c.timeout_sec = 0;
  EXPECT_FALSE(d.configure(c));
  ASSERT_TRUE(d.configure(immediateConfig()));
  d.update(pitch(20), std::numeric_limits<double>::quiet_NaN());
  EXPECT_FALSE(d.status(1).valid);
  d.update(Eigen::Quaterniond(std::numeric_limits<double>::quiet_NaN(), 0, 0, 0), 1);
  EXPECT_FALSE(d.status(1).valid);
}
}  // namespace
