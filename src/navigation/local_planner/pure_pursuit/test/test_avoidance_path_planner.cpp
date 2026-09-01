#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

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

nav_msgs::msg::OccupancyGrid makeSmoothingCostmap() {
  nav_msgs::msg::OccupancyGrid costmap;
  costmap.header.frame_id = "map";
  costmap.info.resolution = 0.02;
  costmap.info.width = 200;
  costmap.info.height = 200;
  costmap.info.origin.position.x = -2.0;
  costmap.info.origin.position.y = -2.0;
  costmap.info.origin.orientation.w = 1.0;
  costmap.data.assign(costmap.info.width * costmap.info.height, 0);
  return costmap;
}

double minimumDiscreteTurnRadius(const nav_msgs::msg::Path &path) {
  double minimum_radius = std::numeric_limits<double>::infinity();
  for (std::size_t i = 1; i + 1 < path.poses.size(); ++i) {
    const auto &a = path.poses[i - 1].pose.position;
    const auto &b = path.poses[i].pose.position;
    const auto &c = path.poses[i + 1].pose.position;
    const double ab = std::hypot(b.x - a.x, b.y - a.y);
    const double bc = std::hypot(c.x - b.x, c.y - b.y);
    const double ac = std::hypot(c.x - a.x, c.y - a.y);
    const double twice_area = std::abs(
        (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x));
    if (ab < 1e-6 || bc < 1e-6 || ac < 1e-6 || twice_area < 1e-8) {
      continue;
    }
    minimum_radius = std::min(
        minimum_radius, ab * bc * ac / (2.0 * twice_area));
  }
  return minimum_radius;
}

struct AvoidancePlannerContext {
  rclcpp::Node::SharedPtr node;
  std::unique_ptr<AvoidancePathPlanner> planner;
};

AvoidancePlannerContext makeSmoothingPlanner(
    const std::string &name) {
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("robot_width", 0.04),
       rclcpp::Parameter("robot_length", 0.04),
       rclcpp::Parameter("astar_footprint_padding", 0.0),
       rclcpp::Parameter("astar_footprint_obstacle_threshold", 100)});
  AvoidancePlannerContext context;
  context.node = std::make_shared<rclcpp::Node>(name, options);
  context.planner = std::make_unique<AvoidancePathPlanner>();
  AvoidancePathPlannerConfig config;
  config.bezier_corner_distance = 0.4;
  config.bezier_sample_spacing = 0.02;
  config.bezier_max_deviation = 0.2;
  config.bezier_visualization_enabled = false;
  EXPECT_TRUE(context.planner->initialize(context.node, config));
  return context;
}

class AvoidancePathPlannerSmoothingTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      int argc = 0;
      rclcpp::init(argc, nullptr);
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) rclcpp::shutdown();
  }
};

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

TEST_F(AvoidancePathPlannerSmoothingTest, RoundsRightAngleInOpenSpace) {
  auto context = makeSmoothingPlanner("bezier_open_space_test");
  context.planner->updateCostmap(
      std::make_shared<const nav_msgs::msg::OccupancyGrid>(
          makeSmoothingCostmap()));
  ASSERT_TRUE(context.planner->prepareLatestCostmap());
  const auto raw = makePath(
      {{0.0, 0.0}, {0.2, 0.0}, {0.4, 0.0}, {0.6, 0.0},
       {0.8, 0.0}, {1.0, 0.0}, {1.0, 0.2}, {1.0, 0.4},
       {1.0, 0.6}, {1.0, 0.8}, {1.0, 1.0}});

  nav_msgs::msg::Path smoothed;
  ASSERT_TRUE(context.planner->smoothPathWithBezier(raw, smoothed));
  ASSERT_GE(smoothed.poses.size(), 3U);
  EXPECT_NEAR(smoothed.poses.front().pose.position.x, 0.0, 1e-9);
  EXPECT_NEAR(smoothed.poses.front().pose.position.y, 0.0, 1e-9);
  EXPECT_NEAR(smoothed.poses.back().pose.position.x, 1.0, 1e-9);
  EXPECT_NEAR(smoothed.poses.back().pose.position.y, 1.0, 1e-9);

  bool has_rounded_point = false;
  for (const auto &pose : smoothed.poses) {
    if (pose.pose.position.x < 0.999 && pose.pose.position.y > 0.001) {
      has_rounded_point = true;
      break;
    }
  }
  EXPECT_TRUE(has_rounded_point);
  EXPECT_TRUE(context.planner->isPathCollisionFree(smoothed));
  EXPECT_GE(minimumDiscreteTurnRadius(smoothed), 0.29);
}

TEST_F(AvoidancePathPlannerSmoothingTest,
       RemovesEightConnectedGridStairSteps) {
  auto context = makeSmoothingPlanner("bezier_grid_stair_test");
  context.planner->updateCostmap(
      std::make_shared<const nav_msgs::msg::OccupancyGrid>(
          makeSmoothingCostmap()));
  ASSERT_TRUE(context.planner->prepareLatestCostmap());
  const auto raw = makePath(
      {{0.0, 0.0}, {0.1, 0.0}, {0.2, 0.1}, {0.3, 0.1},
       {0.4, 0.2}, {0.5, 0.2}, {0.6, 0.3}, {0.7, 0.3},
       {0.8, 0.4}});

  nav_msgs::msg::Path smoothed;
  ASSERT_TRUE(context.planner->smoothPathWithBezier(raw, smoothed));
  ASSERT_GE(smoothed.poses.size(), 2U);
  for (const auto &pose : smoothed.poses) {
    EXPECT_NEAR(pose.pose.position.y, 0.5 * pose.pose.position.x, 1e-6);
  }
  EXPECT_TRUE(context.planner->isPathCollisionFree(smoothed));
  EXPECT_TRUE(std::isinf(minimumDiscreteTurnRadius(smoothed)));
}

TEST_F(AvoidancePathPlannerSmoothingTest,
       RejectsCornerThatCannotMeetMinimumTurnRadius) {
  auto context = makeSmoothingPlanner("bezier_minimum_radius_test");
  context.planner->updateCostmap(
      std::make_shared<const nav_msgs::msg::OccupancyGrid>(
          makeSmoothingCostmap()));
  ASSERT_TRUE(context.planner->prepareLatestCostmap());
  const auto raw = makePath({{0.0, 0.0}, {0.2, 0.0}, {0.2, 0.2}});

  nav_msgs::msg::Path result;
  EXPECT_FALSE(context.planner->smoothPathWithBezier(raw, result));
  ASSERT_EQ(result.poses.size(), raw.poses.size());
  EXPECT_NEAR(result.poses[1].pose.position.x, 0.2, 1e-9);
  EXPECT_NEAR(result.poses[1].pose.position.y, 0.0, 1e-9);
}

TEST_F(AvoidancePathPlannerSmoothingTest, LeavesStraightPathUnchanged) {
  auto context = makeSmoothingPlanner("bezier_straight_path_test");
  context.planner->updateCostmap(
      std::make_shared<const nav_msgs::msg::OccupancyGrid>(
          makeSmoothingCostmap()));
  ASSERT_TRUE(context.planner->prepareLatestCostmap());
  const auto raw = makePath({{0.0, 0.0}, {0.5, 0.0}, {1.0, 0.0}});

  nav_msgs::msg::Path result;
  EXPECT_FALSE(context.planner->smoothPathWithBezier(raw, result));
  ASSERT_EQ(result.poses.size(), raw.poses.size());
  EXPECT_NEAR(result.poses[1].pose.position.x, 0.5, 1e-9);
  EXPECT_NEAR(result.poses[1].pose.position.y, 0.0, 1e-9);
}

TEST_F(AvoidancePathPlannerSmoothingTest,
       FallsBackWhenBezierCornerIntersectsObstacle) {
  auto context = makeSmoothingPlanner("bezier_collision_fallback_test");
  auto costmap = makeSmoothingCostmap();
  const int center_x = static_cast<int>(
      (0.9 - costmap.info.origin.position.x) / costmap.info.resolution);
  const int center_y = static_cast<int>(
      (0.1 - costmap.info.origin.position.y) / costmap.info.resolution);
  for (int dy = -1; dy <= 1; ++dy) {
    for (int dx = -1; dx <= 1; ++dx) {
      const int index = (center_y + dy) *
                        static_cast<int>(costmap.info.width) + center_x + dx;
      costmap.data[static_cast<std::size_t>(index)] = 100;
    }
  }
  context.planner->updateCostmap(
      std::make_shared<const nav_msgs::msg::OccupancyGrid>(costmap));
  ASSERT_TRUE(context.planner->prepareLatestCostmap());
  const auto raw = makePath({{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}});
  ASSERT_TRUE(context.planner->isPathCollisionFree(raw));

  nav_msgs::msg::Path result;
  EXPECT_FALSE(context.planner->smoothPathWithBezier(raw, result));
  ASSERT_EQ(result.poses.size(), raw.poses.size());
  EXPECT_NEAR(result.poses[1].pose.position.x, 1.0, 1e-9);
  EXPECT_NEAR(result.poses[1].pose.position.y, 0.0, 1e-9);
}

}  // namespace
}  // namespace pure_pursuit_local_planner
