#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

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

double quaternionYaw(const geometry_msgs::msg::Quaternion &orientation) {
  return std::atan2(
      2.0 * (orientation.w * orientation.z +
             orientation.x * orientation.y),
      1.0 - 2.0 * (orientation.y * orientation.y +
                   orientation.z * orientation.z));
}

geometry_msgs::msg::Quaternion yawOrientation(double yaw) {
  geometry_msgs::msg::Quaternion orientation;
  orientation.z = std::sin(yaw * 0.5);
  orientation.w = std::cos(yaw * 0.5);
  return orientation;
}

struct AvoidancePlannerContext {
  rclcpp::Node::SharedPtr node;
  std::unique_ptr<AvoidancePathPlanner> planner;
};

AvoidancePlannerContext makeSmoothingPlanner(
    const std::string &name,
    const std::string &smoothing_method = "bezier") {
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
  config.smoothing_method = smoothing_method;
  config.bezier_corner_distance = 0.4;
  config.bezier_sample_spacing = 0.02;
  config.bezier_max_deviation = 0.2;
  config.bezier_visualization_enabled = false;
  EXPECT_TRUE(context.planner->initialize(context.node, config));
  return context;
}

nav_msgs::msg::OccupancyGrid makeTemporaryGoalCostmap() {
  nav_msgs::msg::OccupancyGrid costmap;
  costmap.header.frame_id = "map";
  costmap.info.resolution = 0.05;
  costmap.info.width = 240;
  costmap.info.height = 120;
  costmap.info.origin.position.x = -1.0;
  costmap.info.origin.position.y = -3.0;
  costmap.info.origin.orientation.w = 1.0;
  costmap.data.assign(costmap.info.width * costmap.info.height, 0);
  return costmap;
}

void addObstacleRectangle(nav_msgs::msg::OccupancyGrid &costmap,
                          double minimum_x, double maximum_x,
                          double minimum_y, double maximum_y,
                          int8_t cost = 100) {
  for (std::size_t y = 0; y < costmap.info.height; ++y) {
    for (std::size_t x = 0; x < costmap.info.width; ++x) {
      const double world_x = costmap.info.origin.position.x +
                             (static_cast<double>(x) + 0.5) *
                                 costmap.info.resolution;
      const double world_y = costmap.info.origin.position.y +
                             (static_cast<double>(y) + 0.5) *
                                 costmap.info.resolution;
      if (world_x >= minimum_x && world_x <= maximum_x &&
          world_y >= minimum_y && world_y <= maximum_y) {
        costmap.data[y * costmap.info.width + x] = cost;
      }
    }
  }
}

nav_msgs::msg::Path makeStraightPath(double start_x, double end_x,
                                     double spacing) {
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  const int segment_count = std::max(
      1, static_cast<int>(std::ceil((end_x - start_x) / spacing)));
  for (int index = 0; index <= segment_count; ++index) {
    const double ratio = static_cast<double>(index) / segment_count;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = start_x + ratio * (end_x - start_x);
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(std::move(pose));
  }
  return path;
}

AvoidancePlannerContext makeTemporaryGoalPlanner(
    const std::string &name, const AvoidancePathPlannerConfig &config,
    double robot_width = 0.20, double robot_length = 0.30,
    double footprint_padding = 0.0) {
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("robot_width", robot_width),
       rclcpp::Parameter("robot_length", robot_length),
       rclcpp::Parameter("astar_footprint_padding", footprint_padding),
       rclcpp::Parameter("obstacle_cost_threshold", 50),
       rclcpp::Parameter("astar_footprint_obstacle_threshold", 100)});
  AvoidancePlannerContext context;
  context.node = std::make_shared<rclcpp::Node>(name, options);
  context.planner = std::make_unique<AvoidancePathPlanner>();
  EXPECT_TRUE(context.planner->initialize(context.node, config));
  return context;
}

AvoidancePathPlannerConfig makeTemporaryGoalConfig() {
  AvoidancePathPlannerConfig config;
  config.temporary_goal_search_distance = 7.0;
  config.temporary_goal_clearance = 0.20;
  config.temporary_goal_safe_path_length = 0.80;
  config.min_temporary_goal_distance_from_obstacle = 0.20;
  config.temporary_goal_sample_spacing = 0.05;
  config.bezier_visualization_enabled = false;
  return config;
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

TEST_F(AvoidancePathPlannerSmoothingTest,
       BSplineRoundsRightAngleAndPreservesEndpoints) {
  auto context = makeSmoothingPlanner("bspline_open_space_test", "bspline");
  context.planner->updateCostmap(
      std::make_shared<const nav_msgs::msg::OccupancyGrid>(
          makeSmoothingCostmap()));
  ASSERT_TRUE(context.planner->prepareLatestCostmap());
  const auto raw = makePath(
      {{0.0, 0.0}, {0.2, 0.0}, {0.4, 0.0}, {0.6, 0.0},
       {0.8, 0.0}, {1.0, 0.0}, {1.0, 0.2}, {1.0, 0.4},
       {1.0, 0.6}, {1.0, 0.8}, {1.0, 1.0}});

  nav_msgs::msg::Path smoothed;
  ASSERT_TRUE(context.planner->smoothPath(raw, smoothed));
  ASSERT_GE(smoothed.poses.size(), 3U);
  EXPECT_NEAR(smoothed.poses.front().pose.position.x, 0.0, 1e-9);
  EXPECT_NEAR(smoothed.poses.front().pose.position.y, 0.0, 1e-9);
  EXPECT_NEAR(smoothed.poses.back().pose.position.x, 1.0, 1e-9);
  EXPECT_NEAR(smoothed.poses.back().pose.position.y, 1.0, 1e-9);
  EXPECT_TRUE(context.planner->isPathCollisionFree(smoothed));
  EXPECT_GE(minimumDiscreteTurnRadius(smoothed), 0.29);
}

TEST_F(AvoidancePathPlannerSmoothingTest,
       BSplineFallsBackWhenCurveIntersectsObstacle) {
  auto context = makeSmoothingPlanner(
      "bspline_collision_fallback_test", "bspline");
  auto costmap = makeSmoothingCostmap();
  const int center_x = static_cast<int>(
      (0.75 - costmap.info.origin.position.x) / costmap.info.resolution);
  const int center_y = static_cast<int>(
      (0.25 - costmap.info.origin.position.y) / costmap.info.resolution);
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
  EXPECT_FALSE(context.planner->smoothPath(raw, result));
  ASSERT_EQ(result.poses.size(), raw.poses.size());
}

TEST_F(AvoidancePathPlannerSmoothingTest,
       PlanPathPreservesExactWorldEndpointsAndTerminalYaw) {
  auto context = makeSmoothingPlanner("astar_exact_endpoint_test");
  const auto costmap = makeSmoothingCostmap();
  context.planner->updateCostmap(
      std::make_shared<const nav_msgs::msg::OccupancyGrid>(costmap));
  ASSERT_TRUE(context.planner->prepareLatestCostmap());

  geometry_msgs::msg::Point start;
  start.x = -0.613;
  start.y = -0.487;
  start.z = 0.17;
  geometry_msgs::msg::Point goal;
  goal.x = 1.237;
  goal.y = 0.413;
  goal.z = -0.04;
  context.planner->setGlobalPath(
      makePath({{start.x, start.y}, {goal.x, goal.y}}));

  constexpr double kGoalYaw = 1.123;
  nav_msgs::msg::Path result;
  ASSERT_TRUE(context.planner->planPath(start, goal, result, kGoalYaw));
  ASSERT_GE(result.poses.size(), 2U);
  EXPECT_DOUBLE_EQ(result.poses.front().pose.position.x, start.x);
  EXPECT_DOUBLE_EQ(result.poses.front().pose.position.y, start.y);
  EXPECT_DOUBLE_EQ(result.poses.front().pose.position.z, start.z);
  EXPECT_DOUBLE_EQ(result.poses.back().pose.position.x, goal.x);
  EXPECT_DOUBLE_EQ(result.poses.back().pose.position.y, goal.y);
  EXPECT_DOUBLE_EQ(result.poses.back().pose.position.z, goal.z);
  EXPECT_NEAR(
      std::atan2(
          std::sin(quaternionYaw(result.poses.back().pose.orientation) -
                   kGoalYaw),
          std::cos(quaternionYaw(result.poses.back().pose.orientation) -
                   kGoalYaw)),
      0.0, 1e-9);
}

TEST_F(AvoidancePathPlannerSmoothingTest,
       PlanPathFailsWhenObstacleWallSeparatesStartAndGoal) {
  auto config = makeTemporaryGoalConfig();
  auto context = makeTemporaryGoalPlanner("astar_unreachable_goal_test", config,
                                     0.04, 0.04, 0.0);
  auto costmap = makeTemporaryGoalCostmap();
  addObstacleRectangle(costmap, 1.95, 2.25, -3.0, 3.0);
  context.planner->updateCostmap(
      std::make_shared<const nav_msgs::msg::OccupancyGrid>(costmap));
  ASSERT_TRUE(context.planner->prepareLatestCostmap());

  geometry_msgs::msg::Point start;
  start.x = 0.13;
  start.y = 0.11;
  geometry_msgs::msg::Point goal;
  goal.x = 4.17;
  goal.y = 0.09;
  context.planner->setGlobalPath(
      makePath({{start.x, start.y}, {goal.x, goal.y}}));

  nav_msgs::msg::Path result;
  EXPECT_FALSE(context.planner->planPath(start, goal, result, 0.0));
}

TEST_F(AvoidancePathPlannerSmoothingTest,
       PlanPathRejectsNonFiniteInputAndClearsOldOutput) {
  const auto config = makeTemporaryGoalConfig();
  auto context = makeTemporaryGoalPlanner(
      "astar_non_finite_input_test", config, 0.04, 0.04, 0.0);
  const auto costmap = makeTemporaryGoalCostmap();
  context.planner->updateCostmap(
      std::make_shared<const nav_msgs::msg::OccupancyGrid>(costmap));
  ASSERT_TRUE(context.planner->prepareLatestCostmap());
  context.planner->setGlobalPath(makePath({{0.0, 0.0}, {2.0, 0.0}}));

  geometry_msgs::msg::Point start;
  geometry_msgs::msg::Point goal;
  goal.x = 2.0;
  nav_msgs::msg::Path old_output = makePath({{8.0, 8.0}, {9.0, 9.0}});
  start.x = std::numeric_limits<double>::quiet_NaN();

  EXPECT_FALSE(context.planner->planPath(start, goal, old_output));
  EXPECT_TRUE(old_output.poses.empty());

  old_output = makePath({{8.0, 8.0}, {9.0, 9.0}});
  start.x = 0.0;
  EXPECT_FALSE(context.planner->planPath(
      start, goal, old_output, std::numeric_limits<double>::infinity()));
  EXPECT_TRUE(old_output.poses.empty());
}

TEST_F(AvoidancePathPlannerSmoothingTest,
       MotionValidationRejectsInternalRightAngleRotationSweep) {
  auto config = makeTemporaryGoalConfig();
  auto context = makeTemporaryGoalPlanner(
      "astar_internal_rotation_sweep_test", config, 0.33, 0.53, 0.0);
  auto costmap = makeSmoothingCostmap();
  // 相对折点栅格中心约(+0.18,+0.18)m：0°和90°矩形footprint均不碰撞，
  // 但两者之间的姿态会让车体角部扫过该障碍栅格。
  addObstacleRectangle(costmap, 1.185, 1.195, 0.185, 0.195);
  context.planner->updateCostmap(
      std::make_shared<const nav_msgs::msg::OccupancyGrid>(costmap));
  ASSERT_TRUE(context.planner->prepareLatestCostmap());

  auto raw_astar_style_path =
      makePath({{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}});
  raw_astar_style_path.poses.back().pose.orientation =
      yawOrientation(M_PI_2);

  ASSERT_TRUE(context.planner->isPathCollisionFree(raw_astar_style_path));
  ASSERT_FALSE(context.planner->isRotationCollisionFree(
      raw_astar_style_path.poses[1].pose.position, 0.0, M_PI_2));
  EXPECT_FALSE(
      context.planner->isPathMotionCollisionFree(raw_astar_style_path));
}

TEST_F(AvoidancePathPlannerSmoothingTest,
       MotionValidationRejectsTerminalArrivalToGoalYawSweep) {
  auto config = makeTemporaryGoalConfig();
  auto context = makeTemporaryGoalPlanner(
      "astar_terminal_rotation_sweep_test", config, 0.33, 0.53, 0.0);
  auto costmap = makeSmoothingCostmap();
  // 直线路段以0°到达目标；目标要求90°。障碍只落在中间旋转姿态的
  // footprint扫掠区，保证失败不是平移路径或两个端点姿态造成的。
  addObstacleRectangle(costmap, 1.185, 1.195, 0.185, 0.195);
  context.planner->updateCostmap(
      std::make_shared<const nav_msgs::msg::OccupancyGrid>(costmap));
  ASSERT_TRUE(context.planner->prepareLatestCostmap());

  auto path = makePath({{0.0, 0.0}, {1.0, 0.0}});
  path.poses.back().pose.orientation = yawOrientation(M_PI_2);

  ASSERT_TRUE(context.planner->isPathCollisionFree(path));
  ASSERT_FALSE(context.planner->isRotationCollisionFree(
      path.poses.back().pose.position, 0.0, M_PI_2));
  EXPECT_FALSE(context.planner->isPathMotionCollisionFree(path));
}

TEST_F(AvoidancePathPlannerSmoothingTest,
       TemporaryGoalTreatsClearanceOnlyViolationAsBlockedPath) {
  auto config = makeTemporaryGoalConfig();
  config.temporary_goal_clearance = 0.30;
  config.temporary_goal_safe_path_length = 0.80;
  config.min_temporary_goal_distance_from_obstacle = 0.10;
  auto context = makeTemporaryGoalPlanner(
      "temporary_goal_clearance_only_block_test", config, 0.20, 0.30, 0.0);
  auto costmap = makeTemporaryGoalCostmap();
  // 障碍距路径中心约0.20m，不与该测试机器人的矩形footprint相交，
  // 但明确落在0.30m绕障临时点净空内。
  addObstacleRectangle(costmap, 1.80, 2.20, 0.22, 0.23);
  context.planner->updateCostmap(
      std::make_shared<const nav_msgs::msg::OccupancyGrid>(costmap));
  ASSERT_TRUE(context.planner->prepareLatestCostmap());
  const auto path = makeStraightPath(0.0, 8.0, 0.2);

  const auto result = context.planner->findTemporaryGoal(
      path, path.poses.front().pose.position, 0U);

  ASSERT_EQ(result.status, TemporaryGoalSearchStatus::FOUND);
  EXPECT_GT(result.blocked_end_path_distance,
            result.blocked_start_path_distance);
  EXPECT_GT(result.goal.pose.position.x, 2.20);
}

TEST_F(AvoidancePathPlannerSmoothingTest,
       TemporaryGoalClearanceUsesExactSubcellPosition) {
  auto config = makeTemporaryGoalConfig();
  config.temporary_goal_clearance = 0.10;
  config.temporary_goal_safe_path_length = 0.50;
  auto context = makeTemporaryGoalPlanner(
      "temporary_goal_subcell_clearance_test", config, 0.04, 0.04, 0.0);
  auto costmap = makeTemporaryGoalCostmap();
  // 临时点(2.045,0.045)靠近其栅格右上边界。障碍栅格中心相对所属
  // 栅格为(-2,+1)，中心间离散距离约0.112m；但点到障碍栅格方块
  // 的真实最短距离约0.095m，必须按0.10m净空判为不安全。
  addObstacleRectangle(costmap, 1.92, 1.93, 0.07, 0.08);
  context.planner->updateCostmap(
      std::make_shared<const nav_msgs::msg::OccupancyGrid>(costmap));
  ASSERT_TRUE(context.planner->prepareLatestCostmap());
  const auto path =
      makePath({{0.0, 0.045}, {2.045, 0.045}, {4.0, 0.045}});

  EXPECT_FALSE(context.planner->isTemporaryGoalValid(path, 1U));
}

TEST_F(AvoidancePathPlannerSmoothingTest,
       TemporaryGoalValidationRejectsExcessiveSampleRequest) {
  auto config = makeTemporaryGoalConfig();
  config.temporary_goal_sample_spacing = 0.001;
  config.temporary_goal_safe_path_length = 25.0;
  auto context = makeTemporaryGoalPlanner(
      "temporary_goal_sample_cap_test", config, 0.04, 0.04, 0.0);
  const auto costmap = makeTemporaryGoalCostmap();
  context.planner->updateCostmap(
      std::make_shared<const nav_msgs::msg::OccupancyGrid>(costmap));
  ASSERT_TRUE(context.planner->prepareLatestCostmap());
  const auto path = makePath({{0.0, 0.0}, {1.0, 0.0}});

  EXPECT_FALSE(context.planner->isTemporaryGoalValid(path, 0U));
}

TEST_F(AvoidancePathPlannerSmoothingTest,
       TemporaryGoalIsValidWithContinuousSafeWindow) {
  auto config = makeTemporaryGoalConfig();
  config.temporary_goal_clearance = 0.10;
  config.temporary_goal_safe_path_length = 0.80;
  auto context = makeTemporaryGoalPlanner("temporary_goal_valid_test", config);
  const auto costmap = makeTemporaryGoalCostmap();
  context.planner->updateCostmap(
      std::make_shared<const nav_msgs::msg::OccupancyGrid>(costmap));
  ASSERT_TRUE(context.planner->prepareLatestCostmap());
  const auto path = makeStraightPath(0.0, 8.0, 0.2);

  EXPECT_TRUE(context.planner->isTemporaryGoalValid(path, 10U));
}

TEST_F(AvoidancePathPlannerSmoothingTest,
       TemporaryGoalRejectsObstacleAtLockedGoal) {
  auto config = makeTemporaryGoalConfig();
  config.temporary_goal_clearance = 0.10;
  config.temporary_goal_safe_path_length = 0.80;
  auto context = makeTemporaryGoalPlanner("temporary_goal_blocked_target_test",
                                     config);
  auto costmap = makeTemporaryGoalCostmap();
  addObstacleRectangle(costmap, 1.95, 2.05, -0.10, 0.10);
  context.planner->updateCostmap(
      std::make_shared<const nav_msgs::msg::OccupancyGrid>(costmap));
  ASSERT_TRUE(context.planner->prepareLatestCostmap());
  const auto path = makeStraightPath(0.0, 8.0, 0.2);

  EXPECT_FALSE(context.planner->isTemporaryGoalValid(path, 10U));
}

TEST_F(AvoidancePathPlannerSmoothingTest,
       TemporaryGoalRejectsObstacleInFollowingSafeWindow) {
  auto config = makeTemporaryGoalConfig();
  config.temporary_goal_clearance = 0.10;
  config.temporary_goal_safe_path_length = 0.80;
  auto context = makeTemporaryGoalPlanner("temporary_goal_blocked_window_test",
                                     config);
  auto costmap = makeTemporaryGoalCostmap();
  addObstacleRectangle(costmap, 2.45, 2.55, -0.10, 0.10);
  context.planner->updateCostmap(
      std::make_shared<const nav_msgs::msg::OccupancyGrid>(costmap));
  ASSERT_TRUE(context.planner->prepareLatestCostmap());
  const auto path = makeStraightPath(0.0, 8.0, 0.2);

  EXPECT_FALSE(context.planner->isTemporaryGoalValid(path, 10U));
}

TEST_F(AvoidancePathPlannerSmoothingTest,
       TemporaryGoalRejectsInsufficientPathTail) {
  auto config = makeTemporaryGoalConfig();
  config.temporary_goal_clearance = 0.10;
  config.temporary_goal_safe_path_length = 0.80;
  auto context = makeTemporaryGoalPlanner("temporary_goal_short_tail_test", config);
  const auto costmap = makeTemporaryGoalCostmap();
  context.planner->updateCostmap(
      std::make_shared<const nav_msgs::msg::OccupancyGrid>(costmap));
  ASSERT_TRUE(context.planner->prepareLatestCostmap());
  const auto path = makeStraightPath(0.0, 3.0, 0.2);

  ASSERT_EQ(path.poses.size(), 16U);
  EXPECT_FALSE(context.planner->isTemporaryGoalValid(path, 14U));
}

TEST_F(AvoidancePathPlannerSmoothingTest,
       TemporaryGoalUsesPathPositionAfterBlockedRegion) {
  const auto config = makeTemporaryGoalConfig();
  auto context = makeTemporaryGoalPlanner("temporary_goal_success_test", config);
  auto costmap = makeTemporaryGoalCostmap();
  addObstacleRectangle(costmap, 1.8, 2.2, -0.5, 0.5);
  context.planner->updateCostmap(
      std::make_shared<const nav_msgs::msg::OccupancyGrid>(costmap));
  ASSERT_TRUE(context.planner->prepareLatestCostmap());
  const auto path = makeStraightPath(0.0, 8.0, 0.2);

  const auto result = context.planner->findTemporaryGoal(
      path, path.poses.front().pose.position, 0U);

  ASSERT_EQ(result.status, TemporaryGoalSearchStatus::FOUND);
  ASSERT_LT(result.temporary_goal_index, path.poses.size());
  ASSERT_GT(result.temporary_goal_index, 0U);
  EXPECT_GT(result.temporary_goal_index, result.blocked_start_index);
  EXPECT_GE(result.goal.pose.position.x,
            path.poses[result.temporary_goal_index - 1U].pose.position.x);
  EXPECT_LE(result.goal.pose.position.x,
            path.poses[result.temporary_goal_index].pose.position.x);
  EXPECT_NEAR(result.goal.pose.position.y, 0.0, 1e-9);
  EXPECT_GE(result.distance_from_last_unsafe,
            config.min_temporary_goal_distance_from_obstacle - 1e-9);
  EXPECT_GE(result.verified_safe_length,
            config.temporary_goal_safe_path_length - 1e-9);
}

TEST_F(AvoidancePathPlannerSmoothingTest,
       TemporaryGoalSkipsShortGapBetweenObstacles) {
  auto config = makeTemporaryGoalConfig();
  config.temporary_goal_clearance = 0.10;
  config.temporary_goal_safe_path_length = 0.80;
  auto context = makeTemporaryGoalPlanner("temporary_goal_short_gap_test", config);
  auto costmap = makeTemporaryGoalCostmap();
  addObstacleRectangle(costmap, 1.8, 2.2, -0.5, 0.5);
  addObstacleRectangle(costmap, 2.8, 3.2, -0.5, 0.5);
  context.planner->updateCostmap(
      std::make_shared<const nav_msgs::msg::OccupancyGrid>(costmap));
  ASSERT_TRUE(context.planner->prepareLatestCostmap());
  const auto path = makeStraightPath(0.0, 8.0, 0.2);

  const auto result = context.planner->findTemporaryGoal(
      path, path.poses.front().pose.position, 0U);

  ASSERT_EQ(result.status, TemporaryGoalSearchStatus::FOUND);
  EXPECT_GT(result.goal.pose.position.x, 3.4);
  EXPECT_GE(result.verified_safe_length,
            config.temporary_goal_safe_path_length - 1e-9);
}

TEST_F(AvoidancePathPlannerSmoothingTest,
       TemporaryGoalRejectsTailWithoutContinuousSafeLength) {
  auto config = makeTemporaryGoalConfig();
  config.temporary_goal_safe_path_length = 1.0;
  auto context = makeTemporaryGoalPlanner("temporary_goal_short_tail_test", config);
  auto costmap = makeTemporaryGoalCostmap();
  addObstacleRectangle(costmap, 1.8, 2.2, -0.5, 0.5);
  context.planner->updateCostmap(
      std::make_shared<const nav_msgs::msg::OccupancyGrid>(costmap));
  ASSERT_TRUE(context.planner->prepareLatestCostmap());
  const auto path = makeStraightPath(0.0, 3.0, 0.2);

  const auto result = context.planner->findTemporaryGoal(
      path, path.poses.front().pose.position, 0U);

  EXPECT_EQ(result.status, TemporaryGoalSearchStatus::NO_SAFE_TEMPORARY_GOAL);
}

TEST_F(AvoidancePathPlannerSmoothingTest,
       TemporaryGoalDetectsObstacleBetweenSparsePathPoses) {
  auto config = makeTemporaryGoalConfig();
  config.temporary_goal_clearance = 0.10;
  config.temporary_goal_safe_path_length = 1.0;
  auto context = makeTemporaryGoalPlanner("temporary_goal_sparse_path_test", config);
  auto costmap = makeTemporaryGoalCostmap();
  addObstacleRectangle(costmap, 1.8, 2.2, -0.5, 0.5);
  context.planner->updateCostmap(
      std::make_shared<const nav_msgs::msg::OccupancyGrid>(costmap));
  ASSERT_TRUE(context.planner->prepareLatestCostmap());
  const auto path = makePath({{0.0, 0.0}, {4.0, 0.0}, {8.0, 0.0}});

  const auto result = context.planner->findTemporaryGoal(
      path, path.poses.front().pose.position, 0U);

  ASSERT_EQ(result.status, TemporaryGoalSearchStatus::FOUND);
  EXPECT_EQ(result.temporary_goal_index, 1U);
  EXPECT_GT(result.goal.pose.position.x, 2.2);
  EXPECT_LT(result.goal.pose.position.x, 4.0);
  EXPECT_NEAR(result.goal.pose.position.y, 0.0, 1e-9);
}

TEST_F(AvoidancePathPlannerSmoothingTest,
       TemporaryGoalUsesSafeLookaheadBeyondCandidateSearchLimit) {
  auto config = makeTemporaryGoalConfig();
  config.temporary_goal_search_distance = 2.5;
  config.temporary_goal_safe_path_length = 1.0;
  auto context = makeTemporaryGoalPlanner("temporary_goal_safe_lookahead_test", config);
  auto costmap = makeTemporaryGoalCostmap();
  addObstacleRectangle(costmap, 1.0, 1.4, -0.5, 0.5);
  context.planner->updateCostmap(
      std::make_shared<const nav_msgs::msg::OccupancyGrid>(costmap));
  ASSERT_TRUE(context.planner->prepareLatestCostmap());
  const auto path = makeStraightPath(0.0, 8.0, 0.2);

  const auto result = context.planner->findTemporaryGoal(
      path, path.poses.front().pose.position, 0U);

  ASSERT_EQ(result.status, TemporaryGoalSearchStatus::FOUND);
  EXPECT_LE(result.temporary_goal_path_distance,
            config.temporary_goal_search_distance + 1e-9);
  EXPECT_GT(result.scanned_path_length, config.temporary_goal_search_distance);
  EXPECT_GE(result.verified_safe_length,
            config.temporary_goal_safe_path_length - 1e-9);
}

TEST_F(AvoidancePathPlannerSmoothingTest,
       TemporaryGoalReportsNoBlockedPathInOpenSpace) {
  const auto config = makeTemporaryGoalConfig();
  auto context = makeTemporaryGoalPlanner("temporary_goal_open_path_test", config);
  const auto costmap = makeTemporaryGoalCostmap();
  context.planner->updateCostmap(
      std::make_shared<const nav_msgs::msg::OccupancyGrid>(costmap));
  ASSERT_TRUE(context.planner->prepareLatestCostmap());
  const auto path = makeStraightPath(0.0, 8.0, 0.2);

  const auto result = context.planner->findTemporaryGoal(
      path, path.poses.front().pose.position, 0U);

  EXPECT_EQ(result.status, TemporaryGoalSearchStatus::NO_BLOCKED_PATH);
}

TEST_F(AvoidancePathPlannerSmoothingTest,
       TemporaryGoalRejectsMissingCostmapAndInvalidIndex) {
  const auto config = makeTemporaryGoalConfig();
  auto context = makeTemporaryGoalPlanner("temporary_goal_invalid_input_test", config);
  const auto path = makeStraightPath(0.0, 8.0, 0.2);

  EXPECT_EQ(context.planner
                ->findTemporaryGoal(path, path.poses.front().pose.position, 0U)
                .status,
            TemporaryGoalSearchStatus::NO_COSTMAP);

  const auto costmap = makeTemporaryGoalCostmap();
  context.planner->updateCostmap(
      std::make_shared<const nav_msgs::msg::OccupancyGrid>(costmap));
  ASSERT_TRUE(context.planner->prepareLatestCostmap());
  EXPECT_EQ(context.planner
                ->findTemporaryGoal(path, path.poses.front().pose.position,
                                   path.poses.size())
                .status,
            TemporaryGoalSearchStatus::INVALID_INPUT);
}

TEST_F(AvoidancePathPlannerSmoothingTest,
       TemporaryGoalStartsAtRobotProjectionOnSparseActiveSegment) {
  auto config = makeTemporaryGoalConfig();
  config.temporary_goal_search_distance = 4.0;
  config.temporary_goal_clearance = 0.10;
  config.temporary_goal_safe_path_length = 0.8;
  config.min_temporary_goal_distance_from_obstacle = 0.10;
  auto context = makeTemporaryGoalPlanner("temporary_goal_projection_test", config);
  auto costmap = makeTemporaryGoalCostmap();
  addObstacleRectangle(costmap, 2.4, 2.8, -0.5, 0.5);
  context.planner->updateCostmap(
      std::make_shared<const nav_msgs::msg::OccupancyGrid>(costmap));
  ASSERT_TRUE(context.planner->prepareLatestCostmap());
  const auto path = makePath({{0.0, 0.0}, {4.0, 0.0}, {8.0, 0.0}});
  geometry_msgs::msg::Point robot_position;
  robot_position.x = 2.1;

  const auto result =
      context.planner->findTemporaryGoal(path, robot_position, 1U);

  ASSERT_EQ(result.status, TemporaryGoalSearchStatus::FOUND);
  EXPECT_EQ(result.search_start_segment_index, 0U);
  EXPECT_EQ(result.temporary_goal_index, 1U);
  EXPECT_GT(result.goal.pose.position.x, 2.8);
  EXPECT_LT(result.goal.pose.position.x, 4.0);
  EXPECT_NEAR(result.goal.pose.position.y, 0.0, 1e-9);
}

TEST_F(AvoidancePathPlannerSmoothingTest,
       TemporaryGoalRejectsCandidateBeyondPathArcSearchLimit) {
  auto config = makeTemporaryGoalConfig();
  config.temporary_goal_search_distance = 5.0;
  config.temporary_goal_clearance = 0.10;
  config.temporary_goal_safe_path_length = 0.4;
  config.min_temporary_goal_distance_from_obstacle = 4.0;
  auto context = makeTemporaryGoalPlanner("temporary_goal_arc_limit_test", config);
  auto costmap = makeTemporaryGoalCostmap();
  addObstacleRectangle(costmap, 0.8, 1.2, -0.5, 0.5);
  context.planner->updateCostmap(
      std::make_shared<const nav_msgs::msg::OccupancyGrid>(costmap));
  ASSERT_TRUE(context.planner->prepareLatestCostmap());
  const auto path = makePath(
      {{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}, {0.0, 0.5}});

  const auto result = context.planner->findTemporaryGoal(
      path, path.poses.front().pose.position, 0U);

  EXPECT_EQ(result.status, TemporaryGoalSearchStatus::NO_SAFE_TEMPORARY_GOAL);
  EXPECT_LE(result.blocked_start_path_distance,
            config.temporary_goal_search_distance);
  EXPECT_GT(result.scanned_path_length, config.temporary_goal_search_distance);
}

TEST_F(AvoidancePathPlannerSmoothingTest,
       TemporaryGoalCapsLargeConfiguredSpacingAtHalfGrid) {
  auto config = makeTemporaryGoalConfig();
  config.temporary_goal_sample_spacing = 1.0;
  config.temporary_goal_clearance = 0.05;
  auto context = makeTemporaryGoalPlanner("temporary_goal_sample_spacing_test", config);
  auto costmap = makeTemporaryGoalCostmap();
  addObstacleRectangle(costmap, 2.02, 2.03, 0.02, 0.03);
  context.planner->updateCostmap(
      std::make_shared<const nav_msgs::msg::OccupancyGrid>(costmap));
  ASSERT_TRUE(context.planner->prepareLatestCostmap());
  const auto path = makePath({{0.0, 0.0}, {4.0, 0.0}, {8.0, 0.0}});

  const auto result = context.planner->findTemporaryGoal(
      path, path.poses.front().pose.position, 0U);

  ASSERT_EQ(result.status, TemporaryGoalSearchStatus::FOUND);
  EXPECT_EQ(result.temporary_goal_index, 1U);
}

TEST_F(AvoidancePathPlannerSmoothingTest,
       TemporaryGoalChecksRealFootprintAgainstLateralObstacle) {
  auto config = makeTemporaryGoalConfig();
  config.temporary_goal_clearance = 0.0;
  config.min_temporary_goal_distance_from_obstacle = 0.1;
  auto context = makeTemporaryGoalPlanner(
      "temporary_goal_real_footprint_test", config, 0.80, 1.10, 0.15);
  auto costmap = makeTemporaryGoalCostmap();
  addObstacleRectangle(costmap, 1.97, 2.03, 0.42, 0.48);
  context.planner->updateCostmap(
      std::make_shared<const nav_msgs::msg::OccupancyGrid>(costmap));
  ASSERT_TRUE(context.planner->prepareLatestCostmap());
  const auto path = makeStraightPath(0.0, 8.0, 0.2);

  const auto result = context.planner->findTemporaryGoal(
      path, path.poses.front().pose.position, 0U);

  EXPECT_EQ(result.status, TemporaryGoalSearchStatus::FOUND);
  EXPECT_LT(result.blocked_start_path_distance, 2.0);
  EXPECT_GT(result.temporary_goal_path_distance, 2.0);
}

TEST_F(AvoidancePathPlannerSmoothingTest,
       TemporaryGoalRejectsUnsafeFootprintRotationAtPathCorner) {
  auto config = makeTemporaryGoalConfig();
  config.temporary_goal_clearance = 0.0;
  config.temporary_goal_safe_path_length = 0.4;
  config.min_temporary_goal_distance_from_obstacle = 0.2;
  auto context = makeTemporaryGoalPlanner(
      "temporary_goal_corner_rotation_test", config, 0.33, 0.53, 0.0);
  auto costmap = makeTemporaryGoalCostmap();
  addObstacleRectangle(costmap, 0.8, 1.0, -0.3, 0.3);
  // 相对拐角(0.18, 0.18)：位于0°和90°矩形footprint之外，但位于
  // 约45°时的旋转扫掠区内。
  addObstacleRectangle(costmap, 2.17, 2.19, 0.17, 0.19);
  context.planner->updateCostmap(
      std::make_shared<const nav_msgs::msg::OccupancyGrid>(costmap));
  ASSERT_TRUE(context.planner->prepareLatestCostmap());
  const auto path = makePath(
      {{0.0, 0.0}, {2.0, 0.0}, {2.0, 1.5}, {2.0, 2.5}});

  const auto result = context.planner->findTemporaryGoal(
      path, path.poses.front().pose.position, 0U);

  ASSERT_EQ(result.status, TemporaryGoalSearchStatus::FOUND);
  EXPECT_EQ(result.temporary_goal_index, 2U);
  EXPECT_GE(result.blocked_end_path_distance, 2.0 - 1e-9);
}

TEST_F(AvoidancePathPlannerSmoothingTest,
       TemporaryGoalRejectsNonFinitePathAndFrameMismatch) {
  const auto config = makeTemporaryGoalConfig();
  auto context = makeTemporaryGoalPlanner("temporary_goal_validation_test", config);
  const auto costmap = makeTemporaryGoalCostmap();
  context.planner->updateCostmap(
      std::make_shared<const nav_msgs::msg::OccupancyGrid>(costmap));
  ASSERT_TRUE(context.planner->prepareLatestCostmap());

  auto invalid_path = makeStraightPath(0.0, 8.0, 0.2);
  invalid_path.poses[2].pose.position.x =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(context.planner
                ->findTemporaryGoal(invalid_path,
                                   invalid_path.poses.front().pose.position,
                                   0U)
                .status,
            TemporaryGoalSearchStatus::INVALID_INPUT);

  auto wrong_frame_path = makeStraightPath(0.0, 8.0, 0.2);
  wrong_frame_path.header.frame_id = "odom";
  EXPECT_EQ(context.planner
                ->findTemporaryGoal(wrong_frame_path,
                                   wrong_frame_path.poses.front().pose.position,
                                   0U)
                .status,
            TemporaryGoalSearchStatus::INVALID_INPUT);
}

TEST_F(AvoidancePathPlannerSmoothingTest,
       TemporaryGoalTreatsUnknownRegionAsUnsafe) {
  auto config = makeTemporaryGoalConfig();
  config.temporary_goal_clearance = 0.10;
  config.temporary_goal_safe_path_length = 0.5;
  config.min_temporary_goal_distance_from_obstacle = 0.10;
  auto context = makeTemporaryGoalPlanner("temporary_goal_unknown_region_test", config);
  auto costmap = makeTemporaryGoalCostmap();
  addObstacleRectangle(costmap, 1.0, 1.2, -0.4, 0.4);
  addObstacleRectangle(costmap, 2.0, 3.0, -0.4, 0.4, -1);
  context.planner->updateCostmap(
      std::make_shared<const nav_msgs::msg::OccupancyGrid>(costmap));
  ASSERT_TRUE(context.planner->prepareLatestCostmap());
  const auto path = makeStraightPath(0.0, 8.0, 0.2);

  const auto result = context.planner->findTemporaryGoal(
      path, path.poses.front().pose.position, 0U);

  ASSERT_EQ(result.status, TemporaryGoalSearchStatus::FOUND);
  EXPECT_GT(result.goal.pose.position.x, 3.2);
}

TEST_F(AvoidancePathPlannerSmoothingTest,
       TemporaryGoalClearanceMovesCandidatePastSideObstacle) {
  auto low_clearance_config = makeTemporaryGoalConfig();
  low_clearance_config.temporary_goal_clearance = 0.0;
  low_clearance_config.temporary_goal_safe_path_length = 0.5;
  low_clearance_config.min_temporary_goal_distance_from_obstacle = 0.1;
  auto high_clearance_config = low_clearance_config;
  high_clearance_config.temporary_goal_clearance = 0.40;
  auto low_context = makeTemporaryGoalPlanner(
      "temporary_goal_low_clearance_test", low_clearance_config);
  auto high_context = makeTemporaryGoalPlanner(
      "temporary_goal_high_clearance_test", high_clearance_config);
  auto costmap = makeTemporaryGoalCostmap();
  addObstacleRectangle(costmap, 1.0, 1.2, -0.4, 0.4);
  addObstacleRectangle(costmap, 2.0, 2.2, 0.32, 0.38);
  const auto costmap_ptr =
      std::make_shared<const nav_msgs::msg::OccupancyGrid>(costmap);
  low_context.planner->updateCostmap(costmap_ptr);
  high_context.planner->updateCostmap(costmap_ptr);
  ASSERT_TRUE(low_context.planner->prepareLatestCostmap());
  ASSERT_TRUE(high_context.planner->prepareLatestCostmap());
  const auto path = makeStraightPath(0.0, 8.0, 0.2);

  const auto low_result = low_context.planner->findTemporaryGoal(
      path, path.poses.front().pose.position, 0U);
  const auto high_result = high_context.planner->findTemporaryGoal(
      path, path.poses.front().pose.position, 0U);

  ASSERT_EQ(low_result.status, TemporaryGoalSearchStatus::FOUND);
  ASSERT_EQ(high_result.status, TemporaryGoalSearchStatus::FOUND);
  EXPECT_GT(high_result.temporary_goal_path_distance,
            low_result.temporary_goal_path_distance);
}

TEST_F(AvoidancePathPlannerSmoothingTest,
       TemporaryGoalMinimumPostObstacleDistanceMovesCandidateForward) {
  auto short_distance_config = makeTemporaryGoalConfig();
  short_distance_config.temporary_goal_clearance = 0.10;
  short_distance_config.temporary_goal_safe_path_length = 0.5;
  short_distance_config.min_temporary_goal_distance_from_obstacle = 0.1;
  auto long_distance_config = short_distance_config;
  long_distance_config.min_temporary_goal_distance_from_obstacle = 1.0;
  auto short_context = makeTemporaryGoalPlanner(
      "temporary_goal_short_post_obstacle_test", short_distance_config);
  auto long_context = makeTemporaryGoalPlanner(
      "temporary_goal_long_post_obstacle_test", long_distance_config);
  auto costmap = makeTemporaryGoalCostmap();
  addObstacleRectangle(costmap, 1.0, 1.2, -0.4, 0.4);
  const auto costmap_ptr =
      std::make_shared<const nav_msgs::msg::OccupancyGrid>(costmap);
  short_context.planner->updateCostmap(costmap_ptr);
  long_context.planner->updateCostmap(costmap_ptr);
  ASSERT_TRUE(short_context.planner->prepareLatestCostmap());
  ASSERT_TRUE(long_context.planner->prepareLatestCostmap());
  const auto path = makeStraightPath(0.0, 8.0, 0.2);

  const auto short_result = short_context.planner->findTemporaryGoal(
      path, path.poses.front().pose.position, 0U);
  const auto long_result = long_context.planner->findTemporaryGoal(
      path, path.poses.front().pose.position, 0U);

  ASSERT_EQ(short_result.status, TemporaryGoalSearchStatus::FOUND);
  ASSERT_EQ(long_result.status, TemporaryGoalSearchStatus::FOUND);
  EXPECT_GT(long_result.temporary_goal_path_distance,
            short_result.temporary_goal_path_distance);
}

TEST_F(AvoidancePathPlannerSmoothingTest,
       TemporaryGoalHandlesRepeatedPathPoses) {
  auto config = makeTemporaryGoalConfig();
  config.temporary_goal_clearance = 0.10;
  auto context = makeTemporaryGoalPlanner("temporary_goal_repeated_pose_test", config);
  auto costmap = makeTemporaryGoalCostmap();
  addObstacleRectangle(costmap, 1.8, 2.2, -0.5, 0.5);
  context.planner->updateCostmap(
      std::make_shared<const nav_msgs::msg::OccupancyGrid>(costmap));
  ASSERT_TRUE(context.planner->prepareLatestCostmap());
  const auto path = makePath(
      {{0.0, 0.0}, {0.0, 0.0}, {4.0, 0.0}, {8.0, 0.0}});

  const auto result = context.planner->findTemporaryGoal(
      path, path.poses.front().pose.position, 0U);

  ASSERT_EQ(result.status, TemporaryGoalSearchStatus::FOUND);
  EXPECT_EQ(result.temporary_goal_index, 2U);
}

TEST_F(AvoidancePathPlannerSmoothingTest,
       SideGoalExpandsAfterBaseEndpointsCollide) {
  auto config = makeTemporaryGoalConfig();
  config.side_goal_boundary_search_radius = 0.20;
  config.side_goal_boundary_longitudinal_band = 0.12;
  config.side_goal_boundary_lateral_band = 0.12;
  config.side_goal_min_boundary_clearance = 0.20;
  config.side_goal_lateral_sample_step = 0.10;
  config.side_goal_lateral_sample_count = 3;
  config.side_goal_min_forward_clearance = 0.20;
  config.side_goal_forward_sample_step = 0.20;
  config.side_goal_forward_sample_count = 4;
  config.side_goal_expansion_rounds = 2;
  config.side_goal_expansion_lateral_step = 0.55;
  config.side_goal_expansion_forward_step = 0.20;
  auto context = makeTemporaryGoalPlanner(
      "side_goal_endpoint_expansion_test", config, 0.20, 0.30, 0.0);
  auto costmap = makeTemporaryGoalCostmap();
  addObstacleRectangle(costmap, 0.90, 1.10, -0.10, 0.10);
  // 覆盖基础右侧候选的终点footprint，但在更外侧保留空间。
  addObstacleRectangle(costmap, 1.20, 2.50, -0.62, -0.18);
  context.planner->updateCostmap(
      std::make_shared<const nav_msgs::msg::OccupancyGrid>(costmap));
  ASSERT_TRUE(context.planner->prepareLatestCostmap());

  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = "map";
  robot_pose.pose.orientation.w = 1.0;
  geometry_msgs::msg::Point obstacle_center;
  obstacle_center.x = 1.0;
  const auto path = makeStraightPath(0.0, 4.0, 0.2);

  const auto candidates = context.planner->buildSideGoalCandidates(
      robot_pose, obstacle_center, AvoidanceSide::RIGHT, path, 0,
      M_PI / 4.0);

  ASSERT_FALSE(candidates.empty());
  for (const auto &candidate : candidates) {
    EXPECT_GE(candidate.boundary_clearance, 0.75 - 1e-9);
    EXPECT_FALSE(
        context.planner->isPointInObstacle(candidate.point, candidate.yaw));
  }
}

}  // namespace
}  // namespace pure_pursuit_local_planner
