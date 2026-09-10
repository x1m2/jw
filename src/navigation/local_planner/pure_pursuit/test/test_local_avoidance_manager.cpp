#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include <tf2_ros/buffer.hpp>

#include "pure_pursuit/local_avoidance_manager.hpp"

namespace pure_pursuit_local_planner {
namespace {

using namespace std::chrono_literals;

nav_msgs::msg::OccupancyGrid makeCostmap() {
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
                          double minimum_y, double maximum_y) {
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
        costmap.data[y * costmap.info.width + x] = 100;
      }
    }
  }
}

nav_msgs::msg::Path makeStraightPath() {
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  constexpr int kPoseCount = 41;
  path.poses.reserve(kPoseCount);
  for (int index = 0; index < kPoseCount; ++index) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = 0.2 * index;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(std::move(pose));
  }
  return path;
}

nav_msgs::msg::Path makeShortDynamicPath() {
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  constexpr int kPoseCount = 7;
  path.poses.reserve(kPoseCount);
  for (int index = 0; index < kPoseCount; ++index) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = 0.4 * index;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(std::move(pose));
  }
  return path;
}

geometry_msgs::msg::PoseStamped makePose(double x, double y = 0.0) {
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation.w = 1.0;
  return pose;
}

AvoidanceObservation makeBlockedObservation() {
  AvoidanceObservation observation;
  observation.decision = ObstacleDecision::AVOIDABLE;
  observation.obstacle_center.x = 2.0;
  observation.obstacle_distance = 2.0;
  observation.left_clear = true;
  observation.right_clear = true;
  return observation;
}

struct ManagerContext {
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  std::unique_ptr<LocalAvoidanceManager> manager;
};

void setRobotTransform(const ManagerContext &context, double x,
                       double y = 0.0) {
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.child_frame_id = "base_link";
  transform.header.stamp = context.node->now();
  transform.transform.translation.x = x;
  transform.transform.translation.y = y;
  transform.transform.rotation.w = 1.0;
  ASSERT_TRUE(context.tf_buffer->setTransform(transform, "test", false));
}

ManagerContext makeManager(const std::string &name,
                           int blocked_confirmations = 2) {
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("global_frame", "map"),
       rclcpp::Parameter("robot_frame", "base_link"),
       rclcpp::Parameter("local_planning_frequency", 50.0),
       // 状态机测试不验证具体车体几何；使用近似点机器人，避免测试夹具
       // 中人为构造的贴边 A* 折角被旋转扫掠安全检查正确拒绝。
       rclcpp::Parameter("robot_width", 0.04),
       rclcpp::Parameter("robot_length", 0.04),
       rclcpp::Parameter("astar_footprint_padding", 0.0),
       rclcpp::Parameter("astar_safety_margin", 0),
       rclcpp::Parameter("astar_path_deviation_weight", 0.0),
       rclcpp::Parameter("astar_obstacle_avoidance_weight", 0.0),
       rclcpp::Parameter("obstacle_cost_threshold", 50),
       rclcpp::Parameter("astar_footprint_obstacle_threshold", 100),
       rclcpp::Parameter("avoidance_temporary_goal_search_distance", 6.0),
       rclcpp::Parameter("avoidance_temporary_goal_clearance", 0.10),
       rclcpp::Parameter("avoidance_temporary_goal_safe_path_length", 0.80),
       rclcpp::Parameter(
           "avoidance_min_temporary_goal_distance_from_obstacle", 0.20),
       rclcpp::Parameter("avoidance_temporary_goal_sample_spacing", 0.05),
       rclcpp::Parameter("avoidance_path_clearance", 0.30),
       rclcpp::Parameter("avoidance_goal_tolerance", 0.10),
       rclcpp::Parameter("avoidance_reentry_cooldown", 0.0),
       rclcpp::Parameter("avoidance_path_blocked_confirmations",
                         blocked_confirmations),
       rclcpp::Parameter("avoidance_temporary_goal_retry_period", 0.10),
       rclcpp::Parameter("avoidance_dynamic_side_goal_fallback_enabled", true),
       rclcpp::Parameter("avoidance_stall_timeout", 0.20),
       rclcpp::Parameter("local_astar_bezier_enabled", false),
       rclcpp::Parameter("local_astar_bezier_visualization", false)});

  ManagerContext context;
  context.node = std::make_shared<rclcpp::Node>(name, options);
  context.tf_buffer =
      std::make_shared<tf2_ros::Buffer>(context.node->get_clock());
  context.tf_buffer->setUsingDedicatedThread(true);
  context.manager = std::make_unique<LocalAvoidanceManager>();
  EXPECT_TRUE(context.manager->initialize(context.node, context.tf_buffer));
  setRobotTransform(context, 0.0);
  context.manager->setDynamicPathMode(false);
  context.manager->setGlobalPath(makeStraightPath());
  return context;
}

LocalAvoidanceOutput waitForOutput(
    LocalAvoidanceManager &manager,
    const geometry_msgs::msg::PoseStamped &robot_pose,
    const AvoidanceObservation &observation,
    const std::function<bool(const LocalAvoidanceOutput &)> &predicate,
    std::chrono::milliseconds timeout = 3000ms,
    bool safety_stop_blocking = false) {
  LocalAvoidanceOutput output;
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  do {
    output = manager.update(robot_pose, observation, true, false,
                            safety_stop_blocking, 0U);
    if (predicate(output)) return output;
    std::this_thread::sleep_for(20ms);
  } while (std::chrono::steady_clock::now() < deadline);
  return output;
}

class LocalAvoidanceManagerTest : public ::testing::Test {
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

TEST_F(LocalAvoidanceManagerTest,
       LocksOneTemporaryGoalReplansItAndReturnsResumeIndex) {
  auto context = makeManager("manager_locked_temporary_goal_test");
  auto costmap = makeCostmap();
  addObstacleRectangle(costmap, 1.8, 2.2, -0.30, 0.30);
  context.manager->updateCostmap(
      std::make_shared<const nav_msgs::msg::OccupancyGrid>(costmap));

  const auto robot_pose = makePose(0.0);
  const auto observation = makeBlockedObservation();
  auto output = waitForOutput(
      *context.manager, robot_pose, observation,
      [](const LocalAvoidanceOutput &candidate) {
        return candidate.mode == AvoidanceTrackingMode::LOCAL_PATH &&
               candidate.temporary_goal_index.has_value() &&
               candidate.temporary_goal.has_value() &&
               candidate.local_path.has_value();
      });

  ASSERT_EQ(output.mode, AvoidanceTrackingMode::LOCAL_PATH);
  ASSERT_TRUE(output.temporary_goal_index);
  ASSERT_TRUE(output.temporary_goal);
  ASSERT_TRUE(output.local_path);
  ASSERT_FALSE(output.local_path->poses.empty());
  const std::size_t locked_index = *output.temporary_goal_index;
  const auto locked_goal = *output.temporary_goal;
  const std::uint64_t initial_generation = output.task_generation;
  EXPECT_DOUBLE_EQ(output.local_path->poses.back().pose.position.x,
                   locked_goal.pose.position.x);
  EXPECT_DOUBLE_EQ(output.local_path->poses.back().pose.position.y,
                   locked_goal.pose.position.y);
  ASSERT_GT(locked_index, 0U);
  EXPECT_GT(locked_goal.pose.position.x, 0.2 * (locked_index - 1U));
  EXPECT_LE(locked_goal.pose.position.x, 0.2 * locked_index + 1e-9);

  // 模拟obstacle_stop持续拦截且机器人没有移动。Manager应使当前路径失效，
  // 但必须保留同一个临时点并对它重新规划，不能回到左右换侧逻辑。
  (void)context.manager->update(robot_pose, observation, true, false, true, 0U);
  std::this_thread::sleep_for(250ms);
  const auto stalled_output = context.manager->update(
      robot_pose, observation, true, false, true, 0U);
  EXPECT_EQ(stalled_output.mode, AvoidanceTrackingMode::WAITING_LOCAL_PATH);
  ASSERT_TRUE(stalled_output.temporary_goal_index);
  EXPECT_EQ(*stalled_output.temporary_goal_index, locked_index);
  EXPECT_GT(stalled_output.task_generation, initial_generation);

  output = waitForOutput(
      *context.manager, robot_pose, observation,
      [locked_index](const LocalAvoidanceOutput &candidate) {
        return candidate.mode == AvoidanceTrackingMode::LOCAL_PATH &&
               candidate.temporary_goal_index == locked_index;
      });
  ASSERT_EQ(output.mode, AvoidanceTrackingMode::LOCAL_PATH);
  ASSERT_EQ(output.temporary_goal_index, locked_index);

  // 到达临时点后，静态任务直接恢复全局路径，并返回对应原路径索引。
  setRobotTransform(context, locked_goal.pose.position.x,
                    locked_goal.pose.position.y);
  const auto reached_pose = makePose(locked_goal.pose.position.x,
                                     locked_goal.pose.position.y);
  output = waitForOutput(
      *context.manager, reached_pose, observation,
      [](const LocalAvoidanceOutput &candidate) {
        return candidate.exited_local_tracking &&
               candidate.resume_global_path_index.has_value();
      });
  EXPECT_TRUE(output.exited_local_tracking);
  ASSERT_TRUE(output.resume_global_path_index);
  EXPECT_EQ(*output.resume_global_path_index, locked_index);
  EXPECT_EQ(output.mode, AvoidanceTrackingMode::GLOBAL_PATH);
}

TEST_F(LocalAvoidanceManagerTest,
       WaitsWithoutSideGoalWhenNoSafeTemporaryGoalExists) {
  auto context = makeManager("manager_no_temporary_goal_test", 3);
  auto costmap = makeCostmap();
  addObstacleRectangle(costmap, 1.0, 8.0, -0.30, 0.30);
  context.manager->updateCostmap(
      std::make_shared<const nav_msgs::msg::OccupancyGrid>(costmap));

  auto output = waitForOutput(
      *context.manager, makePose(0.0), makeBlockedObservation(),
      [](const LocalAvoidanceOutput &candidate) {
        return candidate.stage == AvoidanceStage::WAITING_TEMPORARY_GOAL;
      });

  EXPECT_EQ(output.stage, AvoidanceStage::WAITING_TEMPORARY_GOAL);
  EXPECT_EQ(output.mode, AvoidanceTrackingMode::STOPPED_BLOCKED);
  EXPECT_TRUE(output.tracking_local);
  EXPECT_FALSE(output.temporary_goal_index);
  EXPECT_FALSE(output.temporary_goal);
  EXPECT_FALSE(output.local_path);

  AvoidanceObservation clear_observation;
  output = context.manager->update(makePose(0.0), clear_observation, true,
                                   false, false, 0U);
  EXPECT_EQ(output.mode, AvoidanceTrackingMode::STOPPED_BLOCKED);

  // 即使控制循环在同一份空闲地图上多次看到CLEAR，也只能累计一个确认。
  const auto open_costmap = makeCostmap();
  context.manager->updateCostmap(
      std::make_shared<const nav_msgs::msg::OccupancyGrid>(open_costmap));
  std::this_thread::sleep_for(100ms);
  for (int repeat = 0; repeat < 5; ++repeat) {
    output = context.manager->update(makePose(0.0), clear_observation, true,
                                     false, false, 0U);
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_TRUE(output.tracking_local);
  EXPECT_EQ(output.mode, AvoidanceTrackingMode::STOPPED_BLOCKED);

  // 第三个不同prepared Costmap版本仍为CLEAR时，才恢复全局路径。
  context.manager->updateCostmap(
      std::make_shared<const nav_msgs::msg::OccupancyGrid>(open_costmap));
  output = waitForOutput(
      *context.manager, makePose(0.0), clear_observation,
      [](const LocalAvoidanceOutput &candidate) {
        return candidate.exited_local_tracking;
      });
  EXPECT_TRUE(output.exited_local_tracking);
  EXPECT_FALSE(output.tracking_local);
  EXPECT_EQ(output.mode, AvoidanceTrackingMode::GLOBAL_PATH);
}

TEST_F(LocalAvoidanceManagerTest,
       ChangesTemporaryGoalOnlyAfterDistinctCostmapConfirmations) {
  auto context = makeManager("manager_temporary_goal_invalidation_test");
  auto initial_costmap = makeCostmap();
  addObstacleRectangle(initial_costmap, 1.8, 2.2, -0.30, 0.30);
  context.manager->updateCostmap(
      std::make_shared<const nav_msgs::msg::OccupancyGrid>(initial_costmap));

  const auto robot_pose = makePose(0.0);
  const auto observation = makeBlockedObservation();
  auto output = waitForOutput(
      *context.manager, robot_pose, observation,
      [](const LocalAvoidanceOutput &candidate) {
        return candidate.mode == AvoidanceTrackingMode::LOCAL_PATH &&
               candidate.temporary_goal_index.has_value();
      });
  ASSERT_TRUE(output.temporary_goal_index);
  ASSERT_TRUE(output.temporary_goal);
  const std::size_t first_index = *output.temporary_goal_index;
  const double first_goal_x = output.temporary_goal->pose.position.x;

  // 一份新地图把锁定点及其安全窗口占用。规划线程可以多次循环，但同一
  // prepared版本只能计为一次无效确认，因此目标仍应保持不变。
  auto blocked_goal_costmap = initial_costmap;
  addObstacleRectangle(blocked_goal_costmap, first_goal_x - 0.10,
                       first_goal_x + 0.35, -0.20, 0.20);
  context.manager->updateCostmap(
      std::make_shared<const nav_msgs::msg::OccupancyGrid>(
          blocked_goal_costmap));
  std::this_thread::sleep_for(250ms);
  output = context.manager->update(robot_pose, observation, true, false,
                                   false, 0U);
  ASSERT_TRUE(output.temporary_goal_index);
  EXPECT_EQ(*output.temporary_goal_index, first_index);

  // 第二个不同版本确认相同失效后，才允许解锁并沿全局路径搜索更靠后的
  // 临时点。
  context.manager->updateCostmap(
      std::make_shared<const nav_msgs::msg::OccupancyGrid>(
          blocked_goal_costmap));
  output = waitForOutput(
      *context.manager, robot_pose, observation,
      [first_index](const LocalAvoidanceOutput &candidate) {
        return candidate.temporary_goal_index.has_value() &&
               *candidate.temporary_goal_index != first_index;
      });
  ASSERT_TRUE(output.temporary_goal_index);
  EXPECT_GT(*output.temporary_goal_index, first_index);
}

TEST_F(LocalAvoidanceManagerTest,
       KeepsLockedPathAfterCentimeterScaleStartPoseDrift) {
  auto context = makeManager("manager_projected_revalidation_test", 2);
  auto costmap = makeCostmap();
  addObstacleRectangle(costmap, 1.8, 2.2, -0.30, 0.30);
  context.manager->updateCostmap(
      std::make_shared<const nav_msgs::msg::OccupancyGrid>(costmap));

  const auto observation = makeBlockedObservation();
  auto output = waitForOutput(
      *context.manager, makePose(0.0), observation,
      [](const LocalAvoidanceOutput &candidate) {
        return candidate.mode == AvoidanceTrackingMode::LOCAL_PATH &&
               candidate.temporary_goal_index.has_value() &&
               candidate.local_path.has_value();
      });
  ASSERT_EQ(output.mode, AvoidanceTrackingMode::LOCAL_PATH);
  ASSERT_TRUE(output.temporary_goal_index);
  const std::size_t locked_index = *output.temporary_goal_index;
  const std::uint64_t locked_generation = output.task_generation;

  // 局部路径发布与下一张Costmap到达之间，机器人在对准过程中产生几厘米
  // 位姿变化是正常现象。重校验必须从路径投影向前检查，不能把机器人回接
  // 到旧起点并把这条短线误认为一次锐角转向。
  constexpr double kPoseDriftX = 0.015;
  constexpr double kPoseDriftY = 0.010;
  setRobotTransform(context, kPoseDriftX, kPoseDriftY);
  const auto drifted_pose = makePose(kPoseDriftX, kPoseDriftY);
  for (int version = 0; version < 3; ++version) {
    context.manager->updateCostmap(
        std::make_shared<const nav_msgs::msg::OccupancyGrid>(costmap));
    std::this_thread::sleep_for(80ms);
    output = context.manager->update(drifted_pose, observation, true, false,
                                     false, 0U);
  }

  EXPECT_EQ(output.mode, AvoidanceTrackingMode::LOCAL_PATH);
  ASSERT_TRUE(output.temporary_goal_index);
  EXPECT_EQ(*output.temporary_goal_index, locked_index);
  EXPECT_EQ(output.task_generation, locked_generation);
  EXPECT_TRUE(output.local_path.has_value());
}

TEST_F(LocalAvoidanceManagerTest,
       SettingGlobalPathCancelsLockedTemporaryGoalWithoutCallerReset) {
  auto context = makeManager("manager_global_path_replacement_test");
  auto costmap = makeCostmap();
  addObstacleRectangle(costmap, 1.8, 2.2, -0.30, 0.30);
  context.manager->updateCostmap(
      std::make_shared<const nav_msgs::msg::OccupancyGrid>(costmap));
  const auto robot_pose = makePose(0.0);
  auto output = waitForOutput(
      *context.manager, robot_pose, makeBlockedObservation(),
      [](const LocalAvoidanceOutput &candidate) {
        return candidate.mode == AvoidanceTrackingMode::LOCAL_PATH &&
               candidate.temporary_goal_index.has_value();
      });
  ASSERT_TRUE(output.tracking_local);
  ASSERT_TRUE(output.temporary_goal_index);
  const std::uint64_t old_generation = output.task_generation;

  auto replacement_path = makeStraightPath();
  for (auto &pose : replacement_path.poses) {
    pose.pose.position.y = 1.0;
  }
  context.manager->setGlobalPath(replacement_path);

  AvoidanceObservation clear_observation;
  output = context.manager->update(robot_pose, clear_observation, true, false,
                                   false, 0U);
  EXPECT_FALSE(output.tracking_local);
  EXPECT_EQ(output.mode, AvoidanceTrackingMode::GLOBAL_PATH);
  EXPECT_EQ(output.stage, AvoidanceStage::IDLE);
  EXPECT_FALSE(output.temporary_goal_index);
  EXPECT_FALSE(output.temporary_goal);
  EXPECT_FALSE(output.local_path);
  EXPECT_GT(output.task_generation, old_generation);
}

TEST_F(LocalAvoidanceManagerTest,
       DynamicTaskResumesLastGoalAfterReachingTemporaryGoal) {
  auto context = makeManager("manager_dynamic_path_resume_test");
  context.manager->setDynamicPathMode(true);

  auto costmap = makeCostmap();
  addObstacleRectangle(costmap, 1.8, 2.2, -0.30, 0.30);
  context.manager->updateCostmap(
      std::make_shared<const nav_msgs::msg::OccupancyGrid>(costmap));

  const auto observation = makeBlockedObservation();
  auto output = waitForOutput(
      *context.manager, makePose(0.0), observation,
      [](const LocalAvoidanceOutput &candidate) {
        return candidate.mode == AvoidanceTrackingMode::LOCAL_PATH &&
               candidate.temporary_goal_index.has_value() &&
               candidate.temporary_goal.has_value();
      });
  ASSERT_TRUE(output.temporary_goal_index);
  ASSERT_TRUE(output.temporary_goal);
  const std::size_t locked_index = *output.temporary_goal_index;
  const auto locked_goal = *output.temporary_goal;
  const std::uint64_t avoidance_generation = output.task_generation;

  // 动态任务到达障碍后临时点时，必须从对应原路径索引继续。否则人员
  // 目标未超过发布阈值或UWB丢失继续旧目标时，没有新路径可等，会永久
  // 停在临时点。显著变化的新路径仍可通过setGlobalPath取消当前任务。
  setRobotTransform(context, locked_goal.pose.position.x,
                    locked_goal.pose.position.y);
  const auto reached_pose = makePose(locked_goal.pose.position.x,
                                     locked_goal.pose.position.y);
  output = waitForOutput(
      *context.manager, reached_pose, observation,
      [](const LocalAvoidanceOutput &candidate) {
        return candidate.mode == AvoidanceTrackingMode::GLOBAL_PATH &&
               candidate.exited_local_tracking &&
               candidate.resume_global_path_index.has_value();
      });
  EXPECT_FALSE(output.tracking_local);
  EXPECT_FALSE(output.local_path);
  ASSERT_TRUE(output.resume_global_path_index);
  EXPECT_EQ(*output.resume_global_path_index, locked_index);
  EXPECT_EQ(output.stage, AvoidanceStage::IDLE);
  EXPECT_FALSE(output.temporary_goal_index);
  EXPECT_GT(output.task_generation, avoidance_generation);

  // 新动态路径仍必须原子替换旧任务；调用方不需要额外reset。
  auto replacement_path = makeStraightPath();
  for (auto &pose : replacement_path.poses) {
    pose.pose.position.y = 1.0;
  }
  const std::uint64_t resumed_generation = output.task_generation;
  context.manager->setGlobalPath(replacement_path);

  AvoidanceObservation clear_observation;
  output = context.manager->update(reached_pose, clear_observation, true,
                                   false, false, locked_index);
  EXPECT_EQ(output.mode, AvoidanceTrackingMode::GLOBAL_PATH);
  EXPECT_EQ(output.stage, AvoidanceStage::IDLE);
  EXPECT_FALSE(output.tracking_local);
  EXPECT_FALSE(output.temporary_goal_index);
  EXPECT_FALSE(output.temporary_goal);
  EXPECT_FALSE(output.local_path);
  EXPECT_GT(output.task_generation, resumed_generation);
}

TEST_F(LocalAvoidanceManagerTest,
       DynamicShortPathUsesConfirmedClearSideAsTemporaryGoal) {
  auto context = makeManager("manager_dynamic_side_fallback_test");
  context.manager->setDynamicPathMode(true);
  context.manager->setGlobalPath(makeShortDynamicPath());

  auto costmap = makeCostmap();
  addObstacleRectangle(costmap, 1.8, 2.2, -0.30, 0.30);
  context.manager->updateCostmap(
      std::make_shared<const nav_msgs::msg::OccupancyGrid>(costmap));

  auto observation = makeBlockedObservation();
  observation.left_clear = false;
  observation.right_clear = true;
  auto output = waitForOutput(
      *context.manager, makePose(0.0), observation,
      [](const LocalAvoidanceOutput &candidate) {
        return candidate.mode == AvoidanceTrackingMode::LOCAL_PATH &&
               candidate.temporary_goal.has_value() &&
               candidate.local_path.has_value();
      });

  ASSERT_EQ(output.mode, AvoidanceTrackingMode::LOCAL_PATH);
  ASSERT_TRUE(output.temporary_goal);
  ASSERT_TRUE(output.local_path);
  // 侧向兜底点不是原动态路径索引，不能让控制器错误推进全局进度。
  EXPECT_FALSE(output.temporary_goal_index);
  EXPECT_LT(output.temporary_goal->pose.position.y, -0.3);
  EXPECT_DOUBLE_EQ(output.local_path->poses.back().pose.position.x,
                   output.temporary_goal->pose.position.x);
  EXPECT_DOUBLE_EQ(output.local_path->poses.back().pose.position.y,
                   output.temporary_goal->pose.position.y);
}

}  // namespace
}  // namespace pure_pursuit_local_planner
