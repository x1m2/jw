#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "pure_pursuit/astar_planner.hpp"

namespace pure_pursuit_local_planner {
namespace {

struct PlannerContext {
  rclcpp::Node::SharedPtr node;
  std::unique_ptr<AStarPlanner> planner;
};

nav_msgs::msg::OccupancyGrid makeCostmap() {
  nav_msgs::msg::OccupancyGrid costmap;
  costmap.header.frame_id = "map";
  costmap.info.resolution = 0.1;
  costmap.info.width = 80;
  costmap.info.height = 80;
  costmap.info.origin.position.x = 0.0;
  costmap.info.origin.position.y = 0.0;
  costmap.info.origin.orientation.w = 1.0;
  costmap.data.assign(costmap.info.width * costmap.info.height, 0);
  return costmap;
}

nav_msgs::msg::Path makeGlobalPath(const geometry_msgs::msg::Point &start,
                                   const geometry_msgs::msg::Point &goal) {
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  geometry_msgs::msg::PoseStamped start_pose;
  start_pose.header = path.header;
  start_pose.pose.position = start;
  start_pose.pose.orientation.w = 1.0;
  path.poses.push_back(start_pose);
  geometry_msgs::msg::PoseStamped goal_pose = start_pose;
  goal_pose.pose.position = goal;
  path.poses.push_back(goal_pose);
  return path;
}

PlannerContext makePlanner(const std::string &name) {
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("robot_width", 0.8),
       rclcpp::Parameter("robot_length", 1.1),
       rclcpp::Parameter("astar_footprint_padding", 0.1),
       rclcpp::Parameter("astar_start_obstacle_threshold", 80),
       rclcpp::Parameter("astar_footprint_obstacle_threshold", 100),
       rclcpp::Parameter("astar_allow_initial_footprint_escape", true),
       rclcpp::Parameter("astar_initial_escape_max_distance", 0.8),
       rclcpp::Parameter("astar_path_deviation_weight", 0.0)});
  PlannerContext context;
  context.node = std::make_shared<rclcpp::Node>(name, options);
  context.planner = std::make_unique<AStarPlanner>();
  EXPECT_TRUE(context.planner->initialize(context.node));
  return context;
}

class AStarPlannerTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      int argc = 0;
      rclcpp::init(argc, nullptr);
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(AStarPlannerTest, PlansAcrossOpenMap) {
  auto context = makePlanner("astar_open_map_test");
  auto costmap = makeCostmap();
  geometry_msgs::msg::Point start;
  start.x = 2.0;
  start.y = 4.0;
  geometry_msgs::msg::Point goal;
  goal.x = 6.0;
  goal.y = 4.0;
  context.planner->setCostmap(costmap);
  context.planner->setGlobalPath(makeGlobalPath(start, goal));

  nav_msgs::msg::Path result;
  EXPECT_TRUE(context.planner->planPath(start, goal, result));
  EXPECT_FALSE(result.poses.empty());
}

TEST_F(AStarPlannerTest, ResetsAndReusesContiguousSearchWorkspace) {
  auto context = makePlanner("astar_workspace_reuse_test");
  context.planner->setCostmap(makeCostmap());

  geometry_msgs::msg::Point start;
  start.x = 2.0;
  start.y = 4.0;
  geometry_msgs::msg::Point goal;
  goal.x = 6.0;
  goal.y = 4.0;

  context.planner->setGlobalPath(makeGlobalPath(start, goal));
  nav_msgs::msg::Path forward_path;
  ASSERT_TRUE(context.planner->planPath(start, goal, forward_path));

  // 第二轮反向搜索会复用相同容量，验证上一轮closed/parent/g没有残留。
  context.planner->setGlobalPath(makeGlobalPath(goal, start));
  nav_msgs::msg::Path reverse_path;
  EXPECT_TRUE(context.planner->planPath(goal, start, reverse_path));
  EXPECT_FALSE(reverse_path.poses.empty());
}

TEST_F(AStarPlannerTest, PublishesCostmapAndFootprintAsOnePreparedSnapshot) {
  auto context = makePlanner("astar_costmap_snapshot_test");
  auto initial_costmap = makeCostmap();
  context.planner->setCostmap(initial_costmap);

  geometry_msgs::msg::Point point;
  point.x = 4.05;
  point.y = 4.05;
  ASSERT_TRUE(context.planner->hasPreparedCostmap());
  EXPECT_FALSE(context.planner->isObstacle(point));

  auto updated_costmap = std::make_shared<nav_msgs::msg::OccupancyGrid>(
      makeCostmap());
  updated_costmap->data[40 * updated_costmap->info.width + 40] = 100;
  context.planner->setCostmap(updated_costmap);

  // 运行时提交只更新pending输入，消费者仍读取上一份完整快照。
  EXPECT_FALSE(context.planner->isObstacle(point));

  ASSERT_TRUE(context.planner->prepareLatestCostmap());
  EXPECT_TRUE(context.planner->isObstacle(point));
}

TEST_F(AStarPlannerTest, FootprintRejectsPathNearHardObstacle) {
  auto context = makePlanner("astar_footprint_test");
  auto costmap = makeCostmap();
  const int obstacle_x = 40;
  const int obstacle_y = 40;
  costmap.data[obstacle_y * costmap.info.width + obstacle_x] = 100;
  context.planner->setCostmap(costmap);

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  geometry_msgs::msg::PoseStamped pose;
  pose.header = path.header;
  pose.pose.position.x = 4.5;
  pose.pose.position.y = 4.05;
  pose.pose.orientation.w = 1.0;
  path.poses.push_back(pose);
  EXPECT_FALSE(context.planner->isPathCollisionFree(path));
}

TEST_F(AStarPlannerTest, RejectsObstacleBetweenSparsePathPoses) {
  auto context = makePlanner("astar_sparse_path_test");
  auto costmap = makeCostmap();
  costmap.data[40 * costmap.info.width + 40] = 100;
  context.planner->setCostmap(costmap);

  geometry_msgs::msg::Point start;
  start.x = 2.0;
  start.y = 4.05;
  geometry_msgs::msg::Point goal;
  goal.x = 6.0;
  goal.y = 4.05;
  EXPECT_FALSE(
      context.planner->isPathCollisionFree(makeGlobalPath(start, goal)));
}

TEST_F(AStarPlannerTest, InitialEscapeAllowsOnlyContinuousBlockedPrefix) {
  auto context = makePlanner("astar_initial_escape_test");
  auto costmap = makeCostmap();
  costmap.data[40 * costmap.info.width + 40] = 100;
  context.planner->setCostmap(costmap);

  geometry_msgs::msg::Point start;
  start.x = 4.65;
  start.y = 4.05;
  geometry_msgs::msg::Point goal;
  goal.x = 6.0;
  goal.y = 4.05;
  const auto escape_path = makeGlobalPath(start, goal);

  EXPECT_FALSE(context.planner->isPathCollisionFree(escape_path));
  EXPECT_TRUE(context.planner->isPathCollisionFree(escape_path, true));

  // 一旦路径进入严格安全区，后续再次撞到硬障碍仍必须失败。
  costmap.data[40 * costmap.info.width + 55] = 100;
  context.planner->setCostmap(costmap);
  EXPECT_FALSE(context.planner->isPathCollisionFree(escape_path, true));
}

TEST_F(AStarPlannerTest, PlansOutOfInitialFootprintOverlap) {
  auto context = makePlanner("astar_plan_initial_escape_test");
  auto costmap = makeCostmap();
  costmap.data[40 * costmap.info.width + 40] = 100;
  context.planner->setCostmap(costmap);

  geometry_msgs::msg::Point start;
  start.x = 4.65;
  start.y = 4.05;
  geometry_msgs::msg::Point goal;
  goal.x = 6.0;
  goal.y = 4.05;
  context.planner->setGlobalPath(makeGlobalPath(start, goal));

  nav_msgs::msg::Path result;
  ASSERT_TRUE(context.planner->planPath(start, goal, result));
  EXPECT_GE(result.poses.size(), 2U);
  EXPECT_TRUE(context.planner->isPathCollisionFree(result, true));
}

TEST_F(AStarPlannerTest, RejectsCoordinateJustOutsideMap) {
  auto context = makePlanner("astar_bounds_test");
  context.planner->setCostmap(makeCostmap());
  geometry_msgs::msg::Point outside;
  outside.x = -0.01;
  outside.y = 4.0;
  EXPECT_TRUE(context.planner->isObstacle(outside));
}

TEST_F(AStarPlannerTest, RotationChecksIntermediateFootprintSweep) {
  auto context = makePlanner("astar_rotation_sweep_test");
  auto costmap = makeCostmap();
  geometry_msgs::msg::Point center;
  center.x = 4.05;
  center.y = 4.05;

  // 障碍位于车体对角方向：0度和90度的矩形均不覆盖它，但旋转过程中的
  // 中间角度会让车体角点扫过该栅格。
  costmap.data[46 * costmap.info.width + 46] = 100;
  context.planner->setCostmap(costmap);
  EXPECT_FALSE(context.planner->isObstacle(center, 0.0));
  EXPECT_FALSE(context.planner->isObstacle(center, M_PI_2));
  EXPECT_FALSE(context.planner->isRotationCollisionFree(center, 0.0, M_PI_2));

  context.planner->setCostmap(makeCostmap());
  EXPECT_TRUE(context.planner->isRotationCollisionFree(center, 0.0, M_PI_2));
}

TEST_F(AStarPlannerTest, RejectsStartAboveConfiguredStartThreshold) {
  auto context = makePlanner("astar_start_threshold_test");
  auto costmap = makeCostmap();
  geometry_msgs::msg::Point start;
  start.x = 4.05;
  start.y = 4.05;
  geometry_msgs::msg::Point goal;
  goal.x = 6.0;
  goal.y = 4.05;
  costmap.data[40 * costmap.info.width + 40] = 90;
  context.planner->setCostmap(costmap);
  context.planner->setGlobalPath(makeGlobalPath(start, goal));

  nav_msgs::msg::Path result;
  EXPECT_FALSE(context.planner->planPath(start, goal, result));
}

} // namespace
} // namespace pure_pursuit_local_planner
