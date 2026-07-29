#include "robot_runtime/navigation_task_executor.hpp"

#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <stdexcept>
#include <unordered_set>
#include <utility>
#include <vector>

#include "robot_interfaces/msg/navigation_node.hpp"
#include "robot_interfaces/msg/runtime_state.hpp"

namespace robot_runtime {

namespace {
using RuntimeState = robot_interfaces::msg::RuntimeState;
}

NavigationTaskExecutor::NavigationTaskExecutor(
    rclcpp::Node *node,
    rclcpp::CallbackGroup::SharedPtr worker_callback_group,
    RobotStateMachine *state_machine, TopologyFileWriter *topology_writer,
    Config config, Callbacks callbacks)
    : node_(node),
      state_machine_(state_machine),
      topology_writer_(topology_writer),
      controller_(node, std::move(worker_callback_group)),
      config_(config),
      callbacks_(std::move(callbacks)) {
  if (!node_ || !state_machine_ || !topology_writer_ ||
      config_.control_timeout_sec <= 0.0 ||
      config_.worker_timeout_sec <= 0.0 ||
      config_.goal_timeout_sec <= 0.0 || !callbacks_.publish_mode ||
      !callbacks_.publish_runtime_state || !callbacks_.enter_error) {
    throw std::invalid_argument("invalid navigation task executor config");
  }
  target_pose_pub_ = node_->create_publisher<geometry_msgs::msg::Pose>(
      "/target_points", rclcpp::QoS(10).reliable());
}

NavigationTaskExecutor::~NavigationTaskExecutor() {
  {
    std::lock_guard<std::mutex> lock(result_mutex_);
    shutting_down_ = true;
    cancel_requested_ = true;
  }
  result_cv_.notify_all();
  if (execution_thread_.joinable()) execution_thread_.join();
}

bool NavigationTaskExecutor::validatePose(
    const geometry_msgs::msg::Pose &pose, std::string &message) const {
  const auto &position = pose.position;
  const auto &orientation = pose.orientation;
  if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
      !std::isfinite(position.z) || !std::isfinite(orientation.x) ||
      !std::isfinite(orientation.y) || !std::isfinite(orientation.z) ||
      !std::isfinite(orientation.w)) {
    message = "pose contains non-finite values";
    return false;
  }
  const double quaternion_norm =
      orientation.x * orientation.x + orientation.y * orientation.y +
      orientation.z * orientation.z + orientation.w * orientation.w;
  if (quaternion_norm < 1e-12) {
    message = "pose has a zero quaternion";
    return false;
  }
  return true;
}

bool NavigationTaskExecutor::validateGoal(
    const StartNavigation::Goal &goal, std::string &message) const {
  // APP节点ID用于反馈，必须唯一；Pose必须能安全交给规划和控制算法。
  if (goal.nodes.empty()) {
    message = "nodes are required";
    return false;
  }
  if (goal.nodes.size() >
      static_cast<std::size_t>(std::numeric_limits<int>::max() - 2)) {
    message = "too many navigation nodes";
    return false;
  }

  std::unordered_set<uint32_t> node_ids;
  for (const auto &node : goal.nodes) {
    if (!node_ids.insert(node.node_id).second) {
      message = "duplicate node_id=" + std::to_string(node.node_id);
      return false;
    }
    if (!validatePose(node.pose, message)) {
      message = "node_id=" + std::to_string(node.node_id) + ": " + message;
      return false;
    }
  }
  return true;
}

bool NavigationTaskExecutor::reserveOperation(std::string &message) {
  // Action巡检和Service单点导航共用同一执行资源，禁止并发改写拓扑文件。
  std::lock_guard<std::mutex> lock(operation_mutex_);
  if (operation_active_) {
    message = "another navigation task is active";
    return false;
  }
  operation_active_ = true;
  {
    std::lock_guard<std::mutex> result_lock(result_mutex_);
    cancel_requested_ = false;
    goal_armed_ = false;
    goal_reached_ = false;
  }
  return true;
}

void NavigationTaskExecutor::configureMap(
    const std::filesystem::path &task_directory,
    const std::filesystem::path &map_directory) {
  task_directory_ = task_directory;
  topology_file_.clear();
  map_directory_ = map_directory;
}

void NavigationTaskExecutor::startAction(
    const std::shared_ptr<GoalHandle> &goal_handle, uint64_t session_id) {
  if (execution_thread_.joinable()) execution_thread_.join();
  execution_thread_ = std::thread(
      &NavigationTaskExecutor::executeAction, this, goal_handle, session_id);
}

void NavigationTaskExecutor::cancel() {
  {
    std::lock_guard<std::mutex> lock(result_mutex_);
    cancel_requested_ = true;
  }
  result_cv_.notify_all();
}

void NavigationTaskExecutor::executeAction(
    const std::shared_ptr<GoalHandle> &goal_handle, uint64_t session_id) {
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<StartNavigation::Result>();
  uint32_t completed_count = 0U;

  if (task_directory_.empty()) {
    result->success = false;
    result->completed_count = 0U;
    result->message = "navigation map is not configured; call load_map first";
    goal_handle->abort(result);
    releaseOperation();
    return;
  }

  // uint32 task_id使用patrol_<id>.txt命名；相同ID再次下发时原子更新。
  topology_file_ =
      task_directory_ / ("patrol_" + std::to_string(goal->task_id) + ".txt");
  topology_writer_->setOutputFile(topology_file_);

  // APP的nodes替代旧的打点文件，先按数组顺序生成节点和相邻双向边。
  std::string topology_message;
  if (!topology_writer_->write(goal->nodes, topology_message)) {
    result->success = false;
    result->completed_count = 0U;
    result->message = topology_message;
    goal_handle->abort(result);
    releaseOperation();
    return;
  }
  RCLCPP_INFO(node_->get_logger(), "%s", topology_message.c_str());

  if (!state_machine_->beginNavigation()) {
    result->success = false;
    result->completed_count = 0U;
    result->message = "navigation state changed before execution";
    goal_handle->abort(result);
    releaseOperation();
    return;
  }
  callbacks_.publish_mode();

  // 拓扑文件发布成功后再启动规划、感知和控制Worker，确保规划器读取新文件。
  std::string worker_message;
  if (!startWorkers(session_id, worker_message)) {
    result->success = false;
    result->completed_count = 0U;
    result->message = worker_message;
    callbacks_.enter_error(RuntimeState::NAVIGATION_ERROR, worker_message);
    goal_handle->abort(result);
    releaseOperation();
    return;
  }
  state_machine_->navigationStarted();
  callbacks_.publish_mode();

  bool cancelled = false;
  bool failed = false;
  std::string failure_message;
  do {
    for (const auto &node : goal->nodes) {
      {
        std::lock_guard<std::mutex> lock(result_mutex_);
        goal_armed_ = false;
        goal_reached_ = false;
        cancelled = cancel_requested_ || shutting_down_;
      }
      if (cancelled || goal_handle->is_canceling()) {
        cancelled = true;
        break;
      }

      // 每次只下发一个目标；到达后反馈该APP node_id，再继续下一个节点。
      target_pose_pub_->publish(node.pose);
      if (!waitForGoal(goal_handle, config_.goal_timeout_sec)) {
        std::lock_guard<std::mutex> lock(result_mutex_);
        cancelled = cancel_requested_ || shutting_down_ ||
                    goal_handle->is_canceling();
        if (!cancelled) {
          failed = true;
          failure_message =
              "navigation timeout at node_id=" +
              std::to_string(node.node_id);
        }
        break;
      }

      if (completed_count < std::numeric_limits<uint32_t>::max()) {
        ++completed_count;
      }
      auto feedback = std::make_shared<StartNavigation::Feedback>();
      feedback->current_node_id = node.node_id;
      feedback->message =
          "node completed: " + std::to_string(node.node_id);
      goal_handle->publish_feedback(feedback);
    }
  } while (goal->loop && !cancelled && !failed);

  if (state_machine_->beginStopNavigation()) callbacks_.publish_mode();
  std::string stop_message;
  const bool stopped = stopWorkers(session_id, stop_message);
  if (stopped) {
    state_machine_->navigationStopped();
  } else {
    failed = true;
    failure_message = stop_message;
  }

  result->completed_count = completed_count;
  if (cancelled) {
    result->success = false;
    result->message = stopped ? "navigation cancelled"
                              : "navigation cancelled; " + stop_message;
    if (!stopped) {
      callbacks_.enter_error(RuntimeState::NAVIGATION_ERROR, result->message);
    } else {
      callbacks_.publish_mode();
    }
    goal_handle->canceled(result);
  } else if (failed) {
    result->success = false;
    result->message = failure_message;
    callbacks_.enter_error(RuntimeState::NAVIGATION_ERROR, failure_message);
    goal_handle->abort(result);
  } else {
    result->success = true;
    result->message = "navigation succeeded";
    callbacks_.publish_runtime_state(RuntimeState::NAVIGATION_SUCCEEDED);
    callbacks_.publish_mode();
    goal_handle->succeed(result);
  }
  releaseOperation();
}

bool NavigationTaskExecutor::navigateToPose(
    const geometry_msgs::msg::PoseStamped &current_pose,
    const geometry_msgs::msg::Pose &target_pose, uint64_t session_id,
    std::string &message) {
  if (task_directory_.empty()) {
    message = "navigation map is not configured; call load_map first";
    releaseOperation();
    return false;
  }
  if (current_pose.header.frame_id != "map") {
    message = "current pose frame is '" + current_pose.header.frame_id +
              "', expected 'map'";
    releaseOperation();
    return false;
  }
  if (!validatePose(current_pose.pose, message)) {
    message = "invalid current pose: " + message;
    releaseOperation();
    return false;
  }
  if (!validatePose(target_pose, message)) {
    message = "invalid target pose: " + message;
    releaseOperation();
    return false;
  }

  // 单点协议没有现成拓扑：用实时定位位姿作为起点、APP Pose作为终点。
  std::vector<robot_interfaces::msg::NavigationNode> topology_nodes(2U);
  topology_nodes[0].node_id = 0U;
  topology_nodes[0].pose = current_pose.pose;
  topology_nodes[1].node_id = 1U;
  topology_nodes[1].pose = target_pose;
  // 单点导航使用固定专用文件，不覆盖APP巡检任务文件。
  topology_file_ = task_directory_ / "single_point_navigation.txt";
  topology_writer_->setOutputFile(topology_file_);
  if (!topology_writer_->write(topology_nodes, message)) {
    releaseOperation();
    return false;
  }
  RCLCPP_INFO(node_->get_logger(),
              "Single-point topology generated from current pose to target: %s",
              message.c_str());

  if (!state_machine_->beginNavigation()) {
    message = "single-point navigation rejected, localization is not ready, "
              "current state=" +
              std::string(toString(state_machine_->mode()));
    releaseOperation();
    return false;
  }
  resetGoalState();
  callbacks_.publish_mode();

  if (!startWorkers(session_id, message)) {
    callbacks_.enter_error(RuntimeState::NAVIGATION_ERROR, message);
    releaseOperation();
    return false;
  }
  state_machine_->navigationStarted();
  callbacks_.publish_mode();
  target_pose_pub_->publish(target_pose);

  bool reached = false;
  {
    std::unique_lock<std::mutex> lock(result_mutex_);
    reached = result_cv_.wait_for(
                  lock, std::chrono::duration<double>(config_.goal_timeout_sec),
                  [this]() { return goal_reached_ || shutting_down_; }) &&
              goal_reached_;
  }

  if (state_machine_->beginStopNavigation()) callbacks_.publish_mode();
  std::string stop_message;
  const bool stopped = stopWorkers(session_id, stop_message);
  if (stopped) state_machine_->navigationStopped();

  if (!reached || !stopped) {
    message = !reached ? "single-point navigation timed out"
                       : "single-point navigation completed but " +
                             stop_message;
    callbacks_.enter_error(RuntimeState::NAVIGATION_ERROR, message);
    releaseOperation();
    return false;
  }

  message = "single-point navigation succeeded";
  callbacks_.publish_runtime_state(RuntimeState::NAVIGATION_SUCCEEDED);
  callbacks_.publish_mode();
  releaseOperation();
  return true;
}

void NavigationTaskExecutor::onNavigationState(bool reached) {
  // 必须观察到当前目标的false后才接受true，避免误用上一目标遗留的true。
  {
    std::lock_guard<std::mutex> lock(result_mutex_);
    if (!reached) {
      goal_armed_ = true;
      goal_reached_ = false;
    } else if (goal_armed_) {
      goal_reached_ = true;
    }
  }
  result_cv_.notify_all();
}

bool NavigationTaskExecutor::setPaused(bool paused, std::string &message) {
  return controller_.setPaused(paused, config_.control_timeout_sec, message);
}

bool NavigationTaskExecutor::startWorkers(uint64_t session_id,
                                          std::string &message) {
  using Result = std::pair<bool, std::string>;
  auto completion = std::make_shared<std::promise<Result>>();
  auto future = completion->get_future();
  controller_.start(
      session_id, topology_file_.string(), map_directory_.string(),
      [completion](bool success, const std::string &detail) {
        completion->set_value({success, detail});
      });
  if (future.wait_for(std::chrono::duration<double>(
          config_.worker_timeout_sec)) != std::future_status::ready) {
    message = "navigation workers start timed out";
    return false;
  }
  const auto result = future.get();
  message = result.second;
  return result.first;
}

bool NavigationTaskExecutor::stopWorkers(uint64_t session_id,
                                         std::string &message) {
  using Result = std::pair<bool, std::string>;
  auto completion = std::make_shared<std::promise<Result>>();
  auto future = completion->get_future();
  controller_.stop(
      session_id, [completion](bool success, const std::string &detail) {
        completion->set_value({success, detail});
      });
  if (future.wait_for(std::chrono::duration<double>(
          config_.worker_timeout_sec)) != std::future_status::ready) {
    message = "navigation workers stop timed out";
    return false;
  }
  const auto result = future.get();
  message = result.second;
  return result.first;
}

bool NavigationTaskExecutor::waitForGoal(
    const std::shared_ptr<GoalHandle> &goal_handle, double timeout_sec) {
  std::unique_lock<std::mutex> lock(result_mutex_);
  return result_cv_.wait_for(
             lock, std::chrono::duration<double>(timeout_sec),
             [this, &goal_handle]() {
               return goal_reached_ || cancel_requested_ || shutting_down_ ||
                      goal_handle->is_canceling();
             }) &&
         goal_reached_;
}

void NavigationTaskExecutor::resetGoalState() {
  std::lock_guard<std::mutex> lock(result_mutex_);
  cancel_requested_ = false;
  goal_armed_ = false;
  goal_reached_ = false;
}

void NavigationTaskExecutor::releaseOperation() {
  resetGoalState();
  std::lock_guard<std::mutex> lock(operation_mutex_);
  operation_active_ = false;
}

}  // namespace robot_runtime
