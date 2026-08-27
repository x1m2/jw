#include <gtest/gtest.h>

#include "pure_pursuit/avoidance_path_planner.hpp"

namespace pure_pursuit_local_planner {
namespace {

nav_msgs::msg::Path makePath(
    std::initializer_list<std::pair<double, double>> points) {
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (const auto &[x, y] : points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}

TEST(AvoidancePathPlannerClearance, StrictModeRejectsPathStartingInside) {
  AvoidancePathPlanner planner;
  geometry_msgs::msg::Point obstacle;
  const auto path = makePath({{0.74, 0.0}, {0.82, 0.1}, {1.2, 0.3}});
  double minimum = 0.0;

  EXPECT_FALSE(planner.isPathOutsideObstacleRadius(
      path, obstacle, 0.80, &minimum));
  EXPECT_NEAR(minimum, 0.74, 1e-6);
}

TEST(AvoidancePathPlannerClearance, AllowsPathEscapingInitialClearance) {
  AvoidancePathPlanner planner;
  geometry_msgs::msg::Point obstacle;
  const auto path = makePath(
      {{0.74, 0.0}, {0.76, 0.05}, {0.84, 0.10}, {1.2, 0.3}});
  double minimum = 0.0;

  EXPECT_TRUE(planner.isPathOutsideObstacleRadius(
      path, obstacle, 0.80, &minimum, true));
  EXPECT_NEAR(minimum, 0.74, 1e-6);
}

TEST(AvoidancePathPlannerClearance, RejectsEscapeThatMovesCloser) {
  AvoidancePathPlanner planner;
  geometry_msgs::msg::Point obstacle;
  const auto path = makePath({{0.74, 0.0}, {0.62, 0.0}, {0.9, 0.1}});

  EXPECT_FALSE(planner.isPathOutsideObstacleRadius(
      path, obstacle, 0.80, nullptr, true));
}

TEST(AvoidancePathPlannerClearance, RejectsEscapeThatNeverLeavesRadius) {
  AvoidancePathPlanner planner;
  geometry_msgs::msg::Point obstacle;
  const auto path = makePath({{0.74, 0.0}, {0.76, 0.02}, {0.78, 0.02}});

  EXPECT_FALSE(planner.isPathOutsideObstacleRadius(
      path, obstacle, 0.80, nullptr, true));
}

TEST(AvoidancePathPlannerClearance, RejectsPathThatReentersAfterEscape) {
  AvoidancePathPlanner planner;
  geometry_msgs::msg::Point obstacle;
  const auto path =
      makePath({{0.74, 0.0}, {0.85, 0.0}, {0.76, 0.0}, {1.0, 0.0}});

  EXPECT_FALSE(planner.isPathOutsideObstacleRadius(
      path, obstacle, 0.80, nullptr, true));
}

}  // namespace
}  // namespace pure_pursuit_local_planner
