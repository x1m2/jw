#include <gtest/gtest.h>

#include "obstacle_stop/directional_obstacle_policy.hpp"

namespace obstacle_stop {
namespace {

constexpr double kEscapeVelocity = 0.05;

TEST(DirectionalObstaclePolicy, LeftObstacleReleasedWhenMovingRight) {
  DirectionalZoneOccupancy occupancy;
  occupancy.left = true;
  const auto decision = evaluateDirectionalObstacle(
      occupancy, -0.20, kEscapeVelocity, true);
  EXPECT_FALSE(decision.active);
  EXPECT_TRUE(decision.released_left);
  EXPECT_FALSE(decision.released_right);
}

TEST(DirectionalObstaclePolicy, RightObstacleReleasedWhenMovingLeft) {
  DirectionalZoneOccupancy occupancy;
  occupancy.right = true;
  const auto decision = evaluateDirectionalObstacle(
      occupancy, 0.20, kEscapeVelocity, true);
  EXPECT_FALSE(decision.active);
  EXPECT_FALSE(decision.released_left);
  EXPECT_TRUE(decision.released_right);
}

TEST(DirectionalObstaclePolicy, MotionTowardObstacleStillStops) {
  DirectionalZoneOccupancy occupancy;
  occupancy.left = true;
  EXPECT_TRUE(evaluateDirectionalObstacle(
                  occupancy, 0.20, kEscapeVelocity, true)
                  .active);
}

TEST(DirectionalObstaclePolicy, WeakTurnStillStops) {
  DirectionalZoneOccupancy occupancy;
  occupancy.left = true;
  EXPECT_TRUE(evaluateDirectionalObstacle(
                  occupancy, -0.02, kEscapeVelocity, true)
                  .active);
}

TEST(DirectionalObstaclePolicy, CenterObstacleAlwaysStops) {
  DirectionalZoneOccupancy occupancy;
  occupancy.center = true;
  EXPECT_TRUE(evaluateDirectionalObstacle(
                  occupancy, -1.0, kEscapeVelocity, true)
                  .active);
}

TEST(DirectionalObstaclePolicy, ObstaclesOnBothSidesAlwaysStop) {
  DirectionalZoneOccupancy occupancy;
  occupancy.left = true;
  occupancy.right = true;
  EXPECT_TRUE(evaluateDirectionalObstacle(
                  occupancy, -1.0, kEscapeVelocity, true)
                  .active);
}

TEST(DirectionalObstaclePolicy, DisabledReleaseKeepsOriginalStopBehavior) {
  DirectionalZoneOccupancy occupancy;
  occupancy.left = true;
  EXPECT_TRUE(evaluateDirectionalObstacle(
                  occupancy, -1.0, kEscapeVelocity, false)
                  .active);
}

}  // namespace
}  // namespace obstacle_stop
