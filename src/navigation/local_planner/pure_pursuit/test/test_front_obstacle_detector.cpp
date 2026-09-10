#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>

#include "pure_pursuit/front_obstacle_detector.hpp"

namespace pure_pursuit_local_planner {
namespace {

nav_msgs::msg::OccupancyGrid makeCostmap() {
  nav_msgs::msg::OccupancyGrid costmap;
  costmap.header.frame_id = "map";
  costmap.info.resolution = 0.05;
  costmap.info.width = 200;
  costmap.info.height = 200;
  costmap.info.origin.position.x = -5.0;
  costmap.info.origin.position.y = -5.0;
  costmap.info.origin.orientation.w = 1.0;
  costmap.data.assign(costmap.info.width * costmap.info.height, 0);
  return costmap;
}

void addObstacleRectangle(nav_msgs::msg::OccupancyGrid &costmap, double x_min,
                          double x_max, double y_min, double y_max) {
  for (std::size_t my = 0; my < costmap.info.height; ++my) {
    const double y = costmap.info.origin.position.y +
                     (static_cast<double>(my) + 0.5) *
                         costmap.info.resolution;
    if (y < y_min || y > y_max) continue;
    for (std::size_t mx = 0; mx < costmap.info.width; ++mx) {
      const double x = costmap.info.origin.position.x +
                       (static_cast<double>(mx) + 0.5) *
                           costmap.info.resolution;
      if (x >= x_min && x <= x_max) {
        costmap.data[my * costmap.info.width + mx] = 100;
      }
    }
  }
}

nav_msgs::msg::Path makeStraightPath() {
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses.resize(2);
  path.poses[0].header = path.header;
  path.poses[0].pose.orientation.w = 1.0;
  path.poses[1] = path.poses[0];
  path.poses[1].pose.position.x = 4.0;
  return path;
}

geometry_msgs::msg::PoseStamped makeRobotPose() {
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.orientation.w = 1.0;
  return pose;
}

struct DetectorContext {
  rclcpp::Node::SharedPtr node;
  std::unique_ptr<FrontObstacleDetector> detector;
};

DetectorContext makeDetector(const std::string &name) {
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("robot_width", 0.7),
       rclcpp::Parameter("robot_length", 1.1),
       rclcpp::Parameter("front_obstacle_detector.avoid_min_distance", 1.0),
       rclcpp::Parameter("front_obstacle_detector.avoid_max_distance", 2.0),
       rclcpp::Parameter("front_obstacle_detector.avoid_sector_angle", 90.0),
       rclcpp::Parameter(
           "front_obstacle_detector.obstacle_ratio_threshold", 0.001),
       rclcpp::Parameter("front_obstacle_detector.side_passage_width", 1.0),
       rclcpp::Parameter(
           "front_obstacle_detector.side_passage_obstacle_ratio_threshold",
           0.02),
       rclcpp::Parameter(
           "front_obstacle_detector.near_side_obstacle_lateral_threshold",
           0.20),
       rclcpp::Parameter("front_obstacle_detector.enable_visualization",
                         false)});
  DetectorContext context;
  context.node = std::make_shared<rclcpp::Node>(name, options);
  context.detector = std::make_unique<FrontObstacleDetector>();
  EXPECT_TRUE(context.detector->initialize(context.node));
  context.detector->setGlobalPath(makeStraightPath());
  return context;
}

class FrontObstacleDetectorTest : public ::testing::Test {
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

TEST_F(FrontObstacleDetectorTest, NearLeftObstacleSelectsRightPassage) {
  auto context = makeDetector("near_left_obstacle_test");
  auto costmap = std::make_shared<nav_msgs::msg::OccupancyGrid>(makeCostmap());
  addObstacleRectangle(*costmap, 0.65, 0.90, 0.28, 0.43);
  context.detector->updateCostmap(costmap);

  const auto result = context.detector->analyzeAvoidance(makeRobotPose());
  EXPECT_EQ(result.decision, ObstacleDecision::AVOIDABLE);
  EXPECT_FALSE(result.left_clear);
  EXPECT_TRUE(result.right_clear);
  EXPECT_GT(result.obstacle_local_y, 0.20);
}

TEST_F(FrontObstacleDetectorTest, NearRightObstacleSelectsLeftPassage) {
  auto context = makeDetector("near_right_obstacle_test");
  auto costmap = std::make_shared<nav_msgs::msg::OccupancyGrid>(makeCostmap());
  addObstacleRectangle(*costmap, 0.65, 0.90, -0.43, -0.28);
  context.detector->updateCostmap(costmap);

  const auto result = context.detector->analyzeAvoidance(makeRobotPose());
  EXPECT_EQ(result.decision, ObstacleDecision::AVOIDABLE);
  EXPECT_TRUE(result.left_clear);
  EXPECT_FALSE(result.right_clear);
  EXPECT_LT(result.obstacle_local_y, -0.20);
}

TEST_F(FrontObstacleDetectorTest, NearCenteredObstacleRemainsBlocked) {
  auto context = makeDetector("near_center_obstacle_test");
  auto costmap = std::make_shared<nav_msgs::msg::OccupancyGrid>(makeCostmap());
  addObstacleRectangle(*costmap, 0.65, 0.90, -0.08, 0.08);
  context.detector->updateCostmap(costmap);

  const auto result = context.detector->analyzeAvoidance(makeRobotPose());
  EXPECT_EQ(result.decision, ObstacleDecision::BLOCKED);
  EXPECT_FALSE(result.left_clear);
  EXPECT_FALSE(result.right_clear);
  EXPECT_NEAR(result.obstacle_local_y, 0.0, 0.05);
}

void addStraightPassage(nav_msgs::msg::OccupancyGrid &costmap) {
  addObstacleRectangle(costmap, 0.0, 2.20, 0.62, 0.72);
  addObstacleRectangle(costmap, 0.0, 2.20, -0.72, -0.62);
}

TEST_F(FrontObstacleDetectorTest, DetectsContinuousCenterPassage) {
  auto context = makeDetector("continuous_center_passage_test");
  auto costmap = std::make_shared<nav_msgs::msg::OccupancyGrid>(makeCostmap());
  addStraightPassage(*costmap);
  context.detector->updateCostmap(costmap);

  const auto result =
      context.detector->detectNarrowPassage(makeRobotPose());
  EXPECT_TRUE(result.detected);
  EXPECT_TRUE(result.center_clear);
  EXPECT_NEAR(result.estimated_width, 1.25, 0.08);
  EXPECT_NEAR(result.center_offset, 0.0, 0.06);
  EXPECT_GE(result.confidence, 0.70);
  EXPECT_FALSE(result.center_path.poses.empty());
}

TEST_F(FrontObstacleDetectorTest, RecoversCenterWhenRobotIsOffset) {
  auto context = makeDetector("offset_center_passage_test");
  auto costmap = std::make_shared<nav_msgs::msg::OccupancyGrid>(makeCostmap());
  addStraightPassage(*costmap);
  context.detector->updateCostmap(costmap);
  auto robot_pose = makeRobotPose();
  robot_pose.pose.position.y = 0.15;

  const auto result = context.detector->detectNarrowPassage(robot_pose);
  ASSERT_TRUE(result.detected);
  ASSERT_FALSE(result.center_path.poses.empty());
  EXPECT_TRUE(result.center_clear);
  EXPECT_NEAR(result.center_offset, -0.15, 0.06);
  EXPECT_NEAR(result.center_path.poses.front().pose.position.y, 0.0, 0.06);

  robot_pose.pose.position.y = -0.15;
  const auto right_offset_result =
      context.detector->detectNarrowPassage(robot_pose);
  ASSERT_TRUE(right_offset_result.detected);
  ASSERT_FALSE(right_offset_result.center_path.poses.empty());
  EXPECT_TRUE(right_offset_result.center_clear);
  EXPECT_NEAR(right_offset_result.center_offset, 0.15, 0.06);
  EXPECT_NEAR(right_offset_result.center_path.poses.front().pose.position.y,
              0.0, 0.06);
}

TEST_F(FrontObstacleDetectorTest, SingleBoxIsNotPassage) {
  auto context = makeDetector("single_box_not_passage_test");
  auto costmap = std::make_shared<nav_msgs::msg::OccupancyGrid>(makeCostmap());
  addObstacleRectangle(*costmap, 0.55, 1.05, -0.30, 0.30);
  context.detector->updateCostmap(costmap);

  const auto result =
      context.detector->detectNarrowPassage(makeRobotPose());
  EXPECT_FALSE(result.detected);
  EXPECT_FALSE(result.center_clear);
  EXPECT_TRUE(result.center_path.poses.empty());
}

TEST_F(FrontObstacleDetectorTest, CenterBoxKeepsPassageButBlocksCenter) {
  auto context = makeDetector("center_box_blocks_passage_test");
  auto costmap = std::make_shared<nav_msgs::msg::OccupancyGrid>(makeCostmap());
  addStraightPassage(*costmap);
  addObstacleRectangle(*costmap, 0.80, 1.20, -0.22, 0.22);
  context.detector->updateCostmap(costmap);

  const auto result =
      context.detector->detectNarrowPassage(makeRobotPose());
  EXPECT_TRUE(result.detected);
  EXPECT_FALSE(result.center_clear);
  EXPECT_NEAR(result.estimated_width, 1.25, 0.08);
}

TEST_F(FrontObstacleDetectorTest, OpenAreaIsNotPassage) {
  auto context = makeDetector("open_area_not_passage_test");
  auto costmap = std::make_shared<nav_msgs::msg::OccupancyGrid>(makeCostmap());
  context.detector->updateCostmap(costmap);

  const auto result =
      context.detector->detectNarrowPassage(makeRobotPose());
  EXPECT_FALSE(result.detected);
  EXPECT_FALSE(result.center_clear);
}

}  // namespace
}  // namespace pure_pursuit_local_planner
