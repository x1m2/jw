#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include "pure_pursuit/path_tracker.hpp"
#include "pure_pursuit/pure_pursuit_controller.hpp"

namespace pure_pursuit_local_planner {
namespace {

class PurePursuitControllerTest : public ::testing::Test {
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

  void SetUp() override {
    node_ = std::make_shared<rclcpp::Node>("pure_pursuit_controller_test");
    PurePursuitControllerConfig config;
    config.lookahead_distance_max = 0.7;
    config.max_linear_velocity = 0.6;
    config.max_angular_velocity = 0.7;
    config.local_avoidance_align_enter_angle = 30.0 * M_PI / 180.0;
    config.local_avoidance_align_exit_angle = 6.0 * M_PI / 180.0;
    config.local_avoidance_min_linear_velocity = 0.4;
    ASSERT_TRUE(controller_.initialize(node_, config));
  }

  geometry_msgs::msg::Twist commandAtAngle(double angle_degrees,
                                            double distance = 1.0) {
    geometry_msgs::msg::PoseStamped robot_pose;
    robot_pose.pose.orientation.w = 1.0;
    geometry_msgs::msg::PointStamped lookahead;
    const double angle = angle_degrees * M_PI / 180.0;
    lookahead.point.x = distance * std::cos(angle);
    lookahead.point.y = distance * std::sin(angle);
    geometry_msgs::msg::Twist command;
    EXPECT_TRUE(controller_.computeCommand(robot_pose, lookahead, 0.5, 0.7,
                                           1, true, 0U, 0, command));
    return command;
  }

  rclcpp::Node::SharedPtr node_;
  PurePursuitController controller_;
};

TEST_F(PurePursuitControllerTest, NewAvoidanceSegmentFullyAlignsBeforeMoving) {
  controller_.startLocalAvoidanceAlignment();
  const auto command = commandAtAngle(30.0);
  EXPECT_DOUBLE_EQ(command.linear.x, 0.0);
  EXPECT_GT(command.angular.z, 0.0);

  const auto exit_command = commandAtAngle(4.0);
  EXPECT_GT(exit_command.linear.x, 0.0);
}

TEST_F(PurePursuitControllerTest, LargeAvoidanceTurnUsesAlignmentHysteresis) {
  controller_.startLocalAvoidanceAlignment();

  const auto enter_command = commandAtAngle(60.0);
  EXPECT_DOUBLE_EQ(enter_command.linear.x, 0.0);
  EXPECT_GT(enter_command.angular.z, 0.0);

  const auto latched_command = commandAtAngle(30.0);
  EXPECT_DOUBLE_EQ(latched_command.linear.x, 0.0);
  EXPECT_GT(latched_command.angular.z, 0.0);

  const auto exit_command = commandAtAngle(4.0);
  EXPECT_GT(exit_command.linear.x, 0.0);
}

TEST_F(PurePursuitControllerTest, SharpLocalCornerRealignsInPlace) {
  controller_.startLocalAvoidanceAlignment();
  EXPECT_GT(commandAtAngle(4.0).linear.x, 0.0);

  const auto corner_command = commandAtAngle(35.0);
  EXPECT_DOUBLE_EQ(corner_command.linear.x, 0.0);
  EXPECT_GT(corner_command.angular.z, 0.0);
}

TEST_F(PurePursuitControllerTest, AvoidanceWalkingKeepsMinimumCruiseSpeed) {
  const auto command = commandAtAngle(25.0, 0.25);
  EXPECT_NEAR(command.linear.x, 0.4, 1e-6);
  EXPECT_GT(command.angular.z, 0.0);
}

TEST_F(PurePursuitControllerTest, LocalLookaheadOverrideAvoidsLongCut) {
  PathTracker tracker;
  PathTrackerConfig config;
  config.lookahead_distance_min = 0.4;
  config.lookahead_distance_max = 0.7;
  config.max_linear_velocity = 0.6;
  config.output_frame = "map";
  ASSERT_TRUE(tracker.initialize(node_, config));

  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.pose.orientation.w = 1.0;
  nav_msgs::msg::Path path;
  path.poses.resize(2);
  path.poses[1].pose.position.x = 2.0;
  const TrackingContext context{TrackingPathType::kLocal, 0U, 0.0, 0.2};
  const auto result =
      tracker.computeTrackingLookahead(robot_pose, path, context);

  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->lookahead_point.point.x, 0.2, 1e-6);
  EXPECT_NEAR(result->lookahead_point.point.y, 0.0, 1e-6);
}

TEST_F(PurePursuitControllerTest,
       StationaryRobotDoesNotAdvanceLocalProgressToPathEnd) {
  PathTracker tracker;
  PathTrackerConfig config;
  config.lookahead_distance_min = 0.4;
  config.lookahead_distance_max = 0.7;
  config.max_linear_velocity = 0.6;
  config.output_frame = "map";
  ASSERT_TRUE(tracker.initialize(node_, config));

  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.pose.orientation.w = 1.0;
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (int i = 0; i <= 40; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = 0.05 * static_cast<double>(i);
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  TrackingContext context{TrackingPathType::kLocal, 0U, 0.0, 0.25};
  for (int cycle = 0; cycle < 30; ++cycle) {
    const auto result =
        tracker.computeTrackingLookahead(robot_pose, path, context);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->progress_index, 0U);
    EXPECT_NEAR(result->lookahead_point.point.x, 0.25, 1e-6);
    context.progress_index = result->progress_index;
  }
}

TEST_F(PurePursuitControllerTest, LocalProjectionDoesNotJumpToNearbyLaterLoop) {
  PathTracker tracker;
  PathTrackerConfig config;
  config.lookahead_distance_min = 0.4;
  config.lookahead_distance_max = 0.7;
  config.max_linear_velocity = 0.6;
  config.output_frame = "map";
  ASSERT_TRUE(tracker.initialize(node_, config));

  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.pose.orientation.w = 1.0;
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  // 前半段沿Y轴离开机器人，后半段绕回机器人附近。局部跟踪必须留在
  // 起始分支，不能因为后续分支空间距离近就跳过去。
  const std::array<std::pair<double, double>, 9> points{{
      {0.0, 0.0}, {0.0, 0.25}, {0.0, 0.50}, {0.0, 0.75},
      {0.0, 1.00}, {0.5, 1.00}, {0.5, 0.50}, {0.1, 0.10},
      {1.0, 0.10}}};
  for (const auto &[x, y] : points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  const TrackingContext context{TrackingPathType::kLocal, 0U, 0.0, 0.25};
  const auto result =
      tracker.computeTrackingLookahead(robot_pose, path, context);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->progress_index, 0U);
  EXPECT_LE(result->path_index, 1U);
  EXPECT_NEAR(result->lookahead_point.point.x, 0.0, 1e-6);
  EXPECT_NEAR(result->lookahead_point.point.y, 0.25, 1e-6);
}

TEST_F(PurePursuitControllerTest,
       ShorterLocalReplanRecoversFromStaleProgressIndex) {
  PathTracker tracker;
  PathTrackerConfig config;
  config.lookahead_distance_min = 0.4;
  config.lookahead_distance_max = 0.7;
  config.max_linear_velocity = 0.6;
  config.output_frame = "map";
  ASSERT_TRUE(tracker.initialize(node_, config));

  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.pose.orientation.w = 1.0;
  nav_msgs::msg::Path replanned_path;
  replanned_path.header.frame_id = "map";
  for (int i = 0; i < 9; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = 0.05 * static_cast<double>(i);
    pose.pose.orientation.w = 1.0;
    replanned_path.poses.push_back(pose);
  }

  // 复现pp.log：旧路径索引已经到15，而替换后的局部路径只有9个点。
  const TrackingContext stale_context{
      TrackingPathType::kLocal, 15U, 0.0, 0.25};
  const auto result = tracker.computeTrackingLookahead(
      robot_pose, replanned_path, stale_context);

  ASSERT_TRUE(result.has_value());
  EXPECT_LT(result->progress_index, replanned_path.poses.size());
  EXPECT_NEAR(result->lookahead_point.point.x, 0.25, 1e-6);
  EXPECT_NEAR(result->lookahead_point.point.y, 0.0, 1e-6);
}

} // namespace
} // namespace pure_pursuit_local_planner
