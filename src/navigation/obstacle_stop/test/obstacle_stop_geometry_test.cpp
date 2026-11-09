#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <unistd.h>

#include "obstacle_stop/detection_zone.hpp"
#include "obstacle_stop/region_profile_loader.hpp"

namespace {
using namespace obstacle_stop;

TEST(FourZoneGeometry, FrontRearUseSensorOriginsAndRobotWidth) {
  const auto front = zoneRectangle(0, 0.47, 0.02, 0.0, 0.3, 0.6);
  EXPECT_TRUE(front.contains(0.60, 0.25));
  EXPECT_FALSE(front.contains(0.46, 0.02));
  EXPECT_FALSE(front.contains(0.60, 0.33));
  const auto rear = zoneRectangle(1, -0.47, -0.02, 0.0, 0.3, 0.6);
  EXPECT_TRUE(rear.contains(-0.60, -0.25));
  EXPECT_FALSE(rear.contains(-0.46, -0.02));
  EXPECT_FALSE(rear.contains(-0.78, -0.02));
}

TEST(FourZoneGeometry, SidesExtendOutwardFromBodyEdgesUsingRobotLength) {
  const auto left = zoneRectangle(2, 0.0, 0.3, 0.0, 0.1, 1.1);
  const auto right = zoneRectangle(3, 0.0, -0.3, 0.0, 0.1, 1.1);
  EXPECT_TRUE(left.contains(0.5, 0.35));
  EXPECT_FALSE(left.contains(0.56, 0.35));
  EXPECT_FALSE(left.contains(0.0, 0.29));
  EXPECT_TRUE(right.contains(-0.5, -0.35));
  EXPECT_FALSE(right.contains(0.0, -0.29));
}

TEST(FourZoneGeometry, SlowdownStartsAtOuterEdgeOfFrontStopZone) {
  const auto stop = zoneRectangle(0, 0.47, 0.0, 0.0, 0.3, 0.6);
  const auto slow = zoneRectangle(0, 0.47, 0.0, 0.3, 0.2, 0.6);
  EXPECT_TRUE(stop.contains(0.70, 0.0));
  EXPECT_FALSE(slow.contains(0.70, 0.0));
  EXPECT_TRUE(slow.contains(0.90, 0.0));
  EXPECT_FALSE(stop.contains(0.90, 0.0));
}

TEST(FourZoneOccupancy, EachSensorMustIndependentlyMeetPointThreshold) {
  HeightCell a, b, merged;
  a.add(0.0);
  b.add(0.1);
  mergeOccupiedCell(merged, a, 2);
  mergeOccupiedCell(merged, b, 2);
  EXPECT_FALSE(merged.occupied(2));
  a.add(0.2);
  mergeOccupiedCell(merged, a, 2);
  EXPECT_TRUE(merged.occupied(2));
  b.add(0.3);
  mergeOccupiedCell(merged, b, 2);
  EXPECT_EQ(merged.point_count, 2U);  // Same cell is not counted twice.
  EXPECT_DOUBLE_EQ(merged.z_min, 0.0);
  EXPECT_DOUBLE_EQ(merged.z_max, 0.3);
}

TEST(FourZoneOccupancy, RejectsNonFinitePoints) {
  HeightCell cell;
  cell.add(std::numeric_limits<double>::quiet_NaN());
  cell.add(std::numeric_limits<double>::infinity());
  EXPECT_FALSE(cell.occupied(1));
  cell.add(0.1);
  cell.add(0.1);
  EXPECT_TRUE(cell.occupied(2));  // Height span is not an occupancy gate.
}

class RegionProfile : public ::testing::Test {
 protected:
  void SetUp() override {
    directory = std::filesystem::temp_directory_path() /
        ("obstacle-stop-region-" + std::to_string(getpid()));
    std::filesystem::create_directories(directory);
    path = directory / "region.yaml";
  }
  void TearDown() override { std::filesystem::remove_all(directory); }
  void write(const std::string &text) { std::ofstream(path) << text; }
  bool load() { return loadRegionObstacleStopOverrides(path, overrides, found, message); }
  std::filesystem::path directory, path;
  RegionObstacleStopOverrides overrides;
  bool found{false};
  std::string message;
};

TEST_F(RegionProfile, MissingFileFallsBackToModelDefaults) {
  EXPECT_TRUE(load());
  EXPECT_FALSE(found);
}

TEST_F(RegionProfile, LoadsPerDirectionOverrides) {
  write("zones:\n  front:\n    stop_distance: 0.4\n    slowdown_distance: 0.2\n"
        "  left:\n    obstacle_cell_threshold: 4\nlinear_velocity_factor: 0.3\n");
  ASSERT_TRUE(load()) << message;
  EXPECT_TRUE(found);
  EXPECT_DOUBLE_EQ(*overrides.zones[0].stop_distance, 0.4);
  EXPECT_EQ(*overrides.zones[2].obstacle_cell_threshold, 4U);
  EXPECT_FALSE(overrides.zones[1].stop_distance.has_value());
}

TEST_F(RegionProfile, RejectsSideSlowdownAndLegacySchema) {
  write("zones:\n  left:\n    slowdown_distance: 0.2\n");
  EXPECT_FALSE(load());
  write("hard_stop_distance: 0.2\n");
  EXPECT_FALSE(load());
}
}  // namespace
