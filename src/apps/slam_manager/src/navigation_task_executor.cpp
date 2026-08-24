#include "slam_manager/navigation_task_executor.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "slam_manager/pose_utils.hpp"
#include "slam_manager_interfaces/msg/navigation_node.hpp"
#include "slam_manager_interfaces/msg/runtime_state.hpp"

namespace slam_manager {

namespace {
using RuntimeState = slam_manager_interfaces::msg::RuntimeState;
bool validTaskId(const std::string &task_id, std::string &message) {
  constexpr std::size_t kMaximumIdLength = 64U;
  if (task_id.empty() || task_id.size() > kMaximumIdLength) {
    message = "task_id must contain 1 to " + std::to_string(kMaximumIdLength) +
              " characters";
    return false;
  }
  for (const char character : task_id) {
    const bool valid = (character >= 'a' && character <= 'z') ||
                       (character >= 'A' && character <= 'Z') ||
                       (character >= '0' && character <= '9') ||
                       character == '_' || character == '-';
    if (!valid) {
      message = "task_id may contain only letters, digits, '_' and '-'";
      return false;
    }
  }
  return true;
}

}  // namespace

NavigationTaskExecutor::NavigationTaskExecutor(
    rclcpp::Node *node, rclcpp::CallbackGroup::SharedPtr worker_callback_group,
    SlamStateMachine *state_machine, TopologyFileWriter *topology_writer,
    Config config, Callbacks callbacks)
    : node_(node),
      state_machine_(state_machine),
      topology_writer_(topology_writer),
      controller_(node, std::move(worker_callback_group)),
      config_(config),
      callbacks_(std::move(callbacks)) {
  if (!node_ || !state_machine_ || !topology_writer_ ||
      config_.control_timeout_sec <= 0.0 || config_.worker_timeout_sec <= 0.0 ||
      config_.goal_timeout_sec <= 0.0 ||
      !std::isfinite(config_.skip_first_target_threshold) ||
      config_.skip_first_target_threshold < 0.0 || !callbacks_.publish_mode ||
      !callbacks_.publish_runtime_state || !callbacks_.enter_error ||
      !callbacks_.get_current_pose || !callbacks_.relocalize) {
    throw std::invalid_argument("invalid navigation task executor config");
  }
  target_pose_pub_ =
      node_->create_publisher<geometry_msgs::msg::Pose>("/target_points", 10);
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

void NavigationTaskExecutor::fillCurrentPoseFeedback(
    StartNavigation::Feedback &feedback) const {
  geometry_msgs::msg::PoseStamped current_pose;
  if (!callbacks_.get_current_pose(current_pose)) {
    return;
  }
  feedback.x = current_pose.pose.position.x;
  feedback.y = current_pose.pose.position.y;
  feedback.z = current_pose.pose.position.z;
  feedback.theta = pose_utils::yaw(current_pose.pose.orientation);
}

bool NavigationTaskExecutor::validatePose(const geometry_msgs::msg::Pose &pose,
                                          std::string &message) const {
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

bool NavigationTaskExecutor::validatePoseFields(double x, double y, double z,
                                                double theta,
                                                std::string &message) const {
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
      !std::isfinite(theta)) {
    message = "x/y/z/theta contain non-finite values";
    return false;
  }
  return true;
}

slam_manager_interfaces::msg::NavigationNode
NavigationTaskExecutor::nodeFromPose(uint32_t node_id,
                                     const geometry_msgs::msg::Pose &pose) {
  slam_manager_interfaces::msg::NavigationNode node;
  node.node_id = node_id;
  node.x = pose.position.x;
  node.y = pose.position.y;
  node.z = pose.position.z;
  node.theta = pose_utils::yaw(pose.orientation);
  return node;
}

bool NavigationTaskExecutor::validateGoal(const StartNavigation::Goal &goal,
                                          std::string &message) const {
  // APP节点ID用于反馈，必须唯一；四自由度位姿必须都是有限值。
  if (!validTaskId(goal.task_id, message)) {
    return false;
  }
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
    if (node.node_id >
        static_cast<uint32_t>(std::numeric_limits<int>::max() - 2)) {
      message =
          "node_id exceeds planner range: " + std::to_string(node.node_id);
      return false;
    }
    if (!node_ids.insert(node.node_id).second) {
      message = "duplicate node_id=" + std::to_string(node.node_id);
      return false;
    }
    if (!validatePoseFields(node.x, node.y, node.z, node.theta, message)) {
      message = "node_id=" + std::to_string(node.node_id) + ": " + message;
      return false;
    }
    if (!std::isfinite(node.stop_duration_sec) ||
        node.stop_duration_sec < 0.0) {
      message = "node_id=" + std::to_string(node.node_id) +
                ": stop_duration_sec must be finite and non-negative";
      return false;
    }
  }
  return true;
}

bool NavigationTaskExecutor::validateGoal(const NavigateToPose::Goal &goal,
                                          std::string &message) const {
  return validatePoseFields(goal.x, goal.y, goal.z, goal.theta, message);
}

bool NavigationTaskExecutor::reserveOperation(std::string &message) {
  // 两种导航Action共用同一套Worker和拓扑文件，协议要求禁止并发执行。
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

void NavigationTaskExecutor::startPatrolAction(
    const std::shared_ptr<StartNavigationGoalHandle> &goal_handle,
    uint64_t session_id) {
  if (execution_thread_.joinable()) execution_thread_.join();
  execution_thread_ = std::thread(&NavigationTaskExecutor::executePatrolAction,
                                  this, goal_handle, session_id);
}

void NavigationTaskExecutor::startNavigateToPoseAction(
    const std::shared_ptr<NavigateToPoseGoalHandle> &goal_handle,
    const geometry_msgs::msg::PoseStamped &current_pose, uint64_t session_id) {
  if (execution_thread_.joinable()) execution_thread_.join();
  execution_thread_ =
      std::thread(&NavigationTaskExecutor::executeNavigateToPoseAction, this,
                  goal_handle, current_pose, session_id);
}

void NavigationTaskExecutor::cancel() {
  {
    std::lock_guard<std::mutex> lock(result_mutex_);
    cancel_requested_ = true;
  }
  result_cv_.notify_all();
}

void NavigationTaskExecutor::executePatrolAction(
    const std::shared_ptr<StartNavigationGoalHandle> &goal_handle,
    uint64_t session_id) {
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

  // 按Action Goal中的task_id读取当前地图专属的巡检任务JSON。
  const auto task_json_file = task_directory_ / (goal->task_id + ".json");
  topology_file_ = task_directory_ / "topology_map.json";
  topology_writer_->setOutputFile(topology_file_);

  std::vector<slam_manager_interfaces::msg::NavigationNode> patrol_nodes;
  std::vector<bool> requires_relocalization(goal->nodes.size(), false);
  std::string topology_message;
  std::error_code filesystem_error;
  const bool task_json_exists =
      std::filesystem::exists(task_json_file, filesystem_error);
  if (filesystem_error) {
    result->success = false;
    result->completed_count = 0U;
    result->message =
        "failed to inspect patrol JSON: " + filesystem_error.message();
    goal_handle->abort(result);
    releaseOperation();
    return;
  }

  bool topology_written = false;
  if (task_json_exists) {
    TaskTopologyGraph graph;
    DirectedEdgeIndex directed_edge_indexes;
    if (!parseTaskJson(task_json_file, graph, directed_edge_indexes,
                       topology_message)) {
      result->success = false;
      result->completed_count = 0U;
      result->message = topology_message;
      goal_handle->abort(result);
      releaseOperation();
      return;
    }

    std::unordered_map<uint32_t, std::size_t> graph_node_indexes;
    std::unordered_set<uint32_t> fallback_node_ids;
    graph_node_indexes.reserve(graph.nodes.size() + goal->nodes.size());
    for (std::size_t index = 0; index < graph.nodes.size(); ++index) {
      graph_node_indexes.emplace(graph.nodes[index].node_id, index);
    }

    patrol_nodes.reserve(goal->nodes.size());
    for (const auto &action_node : goal->nodes) {
      const auto json_node_it = graph_node_indexes.find(action_node.node_id);
      if (json_node_it != graph_node_indexes.end()) {
        patrol_nodes.push_back(graph.nodes[json_node_it->second]);
        RCLCPP_INFO(node_->get_logger(),
                    "Patrol node_id=%u resolved from local JSON",
                    action_node.node_id);
      } else {
        const auto fallback_node = action_node;
        graph_node_indexes.emplace(action_node.node_id, graph.nodes.size());
        graph.nodes.push_back(fallback_node);
        patrol_nodes.push_back(fallback_node);
        fallback_node_ids.insert(action_node.node_id);
        RCLCPP_WARN(node_->get_logger(),
                    "Patrol node_id=%u is absent from JSON; using Action pose "
                    "and default policies",
                    action_node.node_id);
      }
    }

    uint64_t next_edge_id = 0U;
    for (const auto &edge : graph.edges) {
      next_edge_id =
          std::max(next_edge_id, static_cast<uint64_t>(edge.id) + 1U);
    }
    for (std::size_t index = 0; index + 1U < patrol_nodes.size(); ++index) {
      const uint32_t from_id = patrol_nodes[index].node_id;
      const uint32_t to_id = patrol_nodes[index + 1U].node_id;
      const TaskTopologyEdge *direct_edge =
          findDirectEdge(graph, directed_edge_indexes, from_id, to_id);
      if (direct_edge != nullptr) {
        if (direct_edge->edge_type == "map_connection") {
          requires_relocalization[index + 1U] = true;
          RCLCPP_INFO(node_->get_logger(),
                      "Patrol transition %u -> %u is map_connection; node %u "
                      "will be used as a relocalization pose",
                      from_id, to_id, to_id);
        }
        continue;
      }

      const bool uses_action_fallback =
          fallback_node_ids.find(from_id) != fallback_node_ids.end() ||
          fallback_node_ids.find(to_id) != fallback_node_ids.end();
      if (!uses_action_fallback) {
        result->success = false;
        result->completed_count = 0U;
        result->message =
            "patrol consecutive nodes require a direct JSON edge: " +
            std::to_string(from_id) + " -> " + std::to_string(to_id);
        goal_handle->abort(result);
        releaseOperation();
        return;
      }
      if (next_edge_id >
          static_cast<uint64_t>(std::numeric_limits<int>::max())) {
        result->success = false;
        result->completed_count = 0U;
        result->message = "cannot allocate a default edge id";
        goal_handle->abort(result);
        releaseOperation();
        return;
      }
      TaskTopologyEdge default_edge;
      default_edge.id = static_cast<uint32_t>(next_edge_id++);
      default_edge.from_id = from_id;
      default_edge.to_id = to_id;
      graph.edges.push_back(default_edge);
      directed_edge_indexes.emplace(makeDirectedEdgeKey(from_id, to_id),
                                    graph.edges.size() - 1U);
      RCLCPP_WARN(node_->get_logger(),
                  "Patrol transition %u -> %u uses an Action fallback node; "
                  "adding a default directed straight edge",
                  from_id, to_id);
    }
    topology_written = topology_writer_->write(graph, topology_message);
  } else {
    result->success = false;
    result->completed_count = 0U;
    result->message = "task JSON does not exist for task_id=" + goal->task_id +
                      ": " + task_json_file.string();
    goal_handle->abort(result);
    releaseOperation();
    return;
  }

  if (!topology_written) {
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

  std::size_t first_patrol_index = 0U;
  if (config_.enable_skip_first_target && patrol_nodes.size() >= 2U) {
    geometry_msgs::msg::PoseStamped current_pose;
    if (callbacks_.get_current_pose(current_pose)) {
      const auto &first_node = patrol_nodes.front();
      const double first_node_distance = std::hypot(
          current_pose.pose.position.x - first_node.x,
          current_pose.pose.position.y - first_node.y);
      if (first_node_distance <= config_.skip_first_target_threshold) {
        first_patrol_index = 1U;
        RCLCPP_WARN(
            node_->get_logger(),
            "[SKIP_FIRST_TARGET] Current pose is near first node_id=%u: "
            "distance=%.3f <= %.3f m; starting from second node_id=%u",
            first_node.node_id, first_node_distance,
            config_.skip_first_target_threshold,
            patrol_nodes[first_patrol_index].node_id);

        auto feedback = std::make_shared<StartNavigation::Feedback>();
        feedback->current_node_id = first_node.node_id;
        fillCurrentPoseFeedback(*feedback);
        feedback->message =
            "first node skipped: distance=" +
            std::to_string(first_node_distance) + " m, threshold=" +
            std::to_string(config_.skip_first_target_threshold) + " m";
        goal_handle->publish_feedback(feedback);
      } else {
        RCLCPP_INFO(node_->get_logger(),
                    "[START_FROM_FIRST_TARGET] Current pose is %.3f m from "
                    "first node_id=%u, above threshold %.3f m",
                    first_node_distance, first_node.node_id,
                    config_.skip_first_target_threshold);
      }
    } else {
      RCLCPP_WARN(node_->get_logger(),
                  "[SKIP_FIRST_NO_POSE] Current pose is unavailable; starting "
                  "from the first patrol node");
    }
  }

  bool cancelled = false;
  bool failed = false;
  int32_t failure_error_code = RuntimeState::NAVIGATION_ERROR;
  std::string failure_message;
  bool first_patrol_round = true;
  do {
    const std::size_t round_start_index =
        first_patrol_round ? first_patrol_index : 0U;
    first_patrol_round = false;
    for (std::size_t patrol_index = round_start_index;
         patrol_index < patrol_nodes.size();
         ++patrol_index) {
      const auto &node = patrol_nodes[patrol_index];
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

      if (requires_relocalization[patrol_index]) {
        auto feedback = std::make_shared<StartNavigation::Feedback>();
        feedback->current_node_id = node.node_id;
        fillCurrentPoseFeedback(*feedback);
        feedback->message =
            "map_connection: stopping navigation before "
            "relocalizing at node " +
            std::to_string(node.node_id);
        goal_handle->publish_feedback(feedback);

        if (!state_machine_->beginStopNavigation()) {
          failed = true;
          failure_message =
              "cannot stop navigation for map_connection at node_id=" +
              std::to_string(node.node_id);
          break;
        }
        callbacks_.publish_mode();

        std::string transition_message;
        if (!stopWorkers(session_id, transition_message)) {
          failed = true;
          failure_message = "failed to stop navigation for map_connection: " +
                            transition_message;
          break;
        }
        if (!state_machine_->navigationStopped()) {
          failed = true;
          failure_message =
              "navigation state transition failed before relocalization";
          break;
        }
        callbacks_.publish_mode();

        {
          std::lock_guard<std::mutex> lock(result_mutex_);
          cancelled = cancel_requested_ || shutting_down_ ||
                      goal_handle->is_canceling();
        }
        if (cancelled) break;

        feedback = std::make_shared<StartNavigation::Feedback>();
        feedback->current_node_id = node.node_id;
        fillCurrentPoseFeedback(*feedback);
        feedback->message = "map_connection: relocalizing at node " +
                            std::to_string(node.node_id);
        goal_handle->publish_feedback(feedback);

        if (!callbacks_.relocalize(
                node,
                [this, goal_handle]() {
                  std::lock_guard<std::mutex> lock(result_mutex_);
                  return cancel_requested_ || shutting_down_ ||
                         goal_handle->is_canceling();
                },
                transition_message)) {
          {
            std::lock_guard<std::mutex> lock(result_mutex_);
            cancelled = cancel_requested_ || shutting_down_ ||
                        goal_handle->is_canceling();
          }
          if (!cancelled) {
            failed = true;
            failure_error_code = RuntimeState::RELOCALIZATION_ERROR;
            failure_message =
                "map_connection relocalization failed at node_id=" +
                std::to_string(node.node_id) + ": " + transition_message;
          }
          break;
        }

        {
          std::lock_guard<std::mutex> lock(result_mutex_);
          cancelled = cancel_requested_ || shutting_down_ ||
                      goal_handle->is_canceling();
        }
        if (cancelled) break;

        if (!state_machine_->beginNavigation()) {
          failed = true;
          failure_message =
              "cannot restart navigation after map_connection at node_id=" +
              std::to_string(node.node_id);
          break;
        }
        callbacks_.publish_mode();
        if (!startWorkers(session_id, transition_message)) {
          failed = true;
          failure_message =
              "failed to restart navigation after map_connection: " +
              transition_message;
          break;
        }
        if (!state_machine_->navigationStarted()) {
          failed = true;
          failure_message =
              "navigation state transition failed after relocalization";
          break;
        }
        callbacks_.publish_mode();

        if (node.stop_duration_sec > 0.0 &&
            !waitForStopDuration(node.stop_duration_sec, [goal_handle]() {
              return goal_handle->is_canceling();
            })) {
          std::lock_guard<std::mutex> lock(result_mutex_);
          cancelled = cancel_requested_ || shutting_down_ ||
                      goal_handle->is_canceling();
          if (!cancelled) {
            failed = true;
            failure_message =
                "node wait interrupted at map_connection node_id=" +
                std::to_string(node.node_id);
          }
          break;
        }

        if (completed_count < std::numeric_limits<uint32_t>::max()) {
          ++completed_count;
        }
        feedback = std::make_shared<StartNavigation::Feedback>();
        feedback->current_node_id = node.node_id;
        fillCurrentPoseFeedback(*feedback);
        feedback->message = "map_connection relocalization completed at node " +
                            std::to_string(node.node_id);
        goal_handle->publish_feedback(feedback);
        continue;
      }

      // 每次只下发一个目标；到达后反馈该APP node_id，再继续下一个节点。
      target_pose_pub_->publish(
          pose_utils::fromXyzTheta(node.x, node.y, node.z, node.theta));
      if (!waitForGoal(config_.goal_timeout_sec, [goal_handle]() {
            return goal_handle->is_canceling();
          })) {
        std::lock_guard<std::mutex> lock(result_mutex_);
        cancelled =
            cancel_requested_ || shutting_down_ || goal_handle->is_canceling();
        if (!cancelled) {
          failed = true;
          failure_message =
              "navigation timeout at node_id=" + std::to_string(node.node_id);
        }
        break;
      }

      if (node.stop_duration_sec > 0.0) {
        auto waiting_feedback = std::make_shared<StartNavigation::Feedback>();
        waiting_feedback->current_node_id = node.node_id;
        fillCurrentPoseFeedback(*waiting_feedback);
        waiting_feedback->message =
            "node reached; waiting " +
            std::to_string(node.stop_duration_sec) + " seconds";
        goal_handle->publish_feedback(waiting_feedback);
        if (!waitForStopDuration(node.stop_duration_sec, [goal_handle]() {
              return goal_handle->is_canceling();
            })) {
          std::lock_guard<std::mutex> lock(result_mutex_);
          cancelled = cancel_requested_ || shutting_down_ ||
                      goal_handle->is_canceling();
          if (!cancelled) {
            failed = true;
            failure_message = "node wait interrupted at node_id=" +
                              std::to_string(node.node_id);
          }
          break;
        }
      }

      if (completed_count < std::numeric_limits<uint32_t>::max()) {
        ++completed_count;
      }
      auto feedback = std::make_shared<StartNavigation::Feedback>();
      feedback->current_node_id = node.node_id;
      fillCurrentPoseFeedback(*feedback);
      feedback->message = "node completed: " + std::to_string(node.node_id);
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
    callbacks_.enter_error(failure_error_code, failure_message);
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

void NavigationTaskExecutor::executeNavigateToPoseAction(
    const std::shared_ptr<NavigateToPoseGoalHandle> &goal_handle,
    geometry_msgs::msg::PoseStamped current_pose, uint64_t session_id) {
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<NavigateToPose::Result>();
  std::string message;

  if (task_directory_.empty()) {
    result->success = false;
    result->message = "navigation map is not configured; call load_map first";
    goal_handle->abort(result);
    releaseOperation();
    return;
  }
  if (current_pose.header.frame_id != "map") {
    result->success = false;
    result->message = "current pose frame is '" + current_pose.header.frame_id +
                      "', expected 'map'";
    goal_handle->abort(result);
    releaseOperation();
    return;
  }
  if (!validatePose(current_pose.pose, message)) {
    result->success = false;
    result->message = "invalid current pose: " + message;
    goal_handle->abort(result);
    releaseOperation();
    return;
  }
  if (!validateGoal(*goal, message)) {
    result->success = false;
    result->message = "invalid target pose: " + message;
    goal_handle->abort(result);
    releaseOperation();
    return;
  }
  const auto target_pose =
      pose_utils::fromXyzTheta(goal->x, goal->y, goal->z, goal->theta);

  // 单点协议没有现成拓扑：用实时map位姿作为起点、APP目标作为终点。
  std::vector<slam_manager_interfaces::msg::NavigationNode> topology_nodes(2U);
  topology_nodes[0] = nodeFromPose(0U, current_pose.pose);
  topology_nodes[1].node_id = 1U;
  topology_nodes[1].x = goal->x;
  topology_nodes[1].y = goal->y;
  topology_nodes[1].z = goal->z;
  topology_nodes[1].theta = goal->theta;

  // 单点导航同样更新统一规划器输入，不覆盖APP的task.json。
  topology_file_ = task_directory_ / "topology_map.json";
  topology_writer_->setOutputFile(topology_file_);
  if (!topology_writer_->write(topology_nodes, message)) {
    result->success = false;
    result->message = message;
    goal_handle->abort(result);
    releaseOperation();
    return;
  }
  RCLCPP_INFO(node_->get_logger(),
              "Single-point topology generated from current pose to target: %s",
              message.c_str());
  auto feedback = std::make_shared<NavigateToPose::Feedback>();
  feedback->message = "navigation topology generated";
  goal_handle->publish_feedback(feedback);

  bool cancelled_before_start = false;
  {
    std::lock_guard<std::mutex> lock(result_mutex_);
    cancelled_before_start =
        cancel_requested_ || shutting_down_ || goal_handle->is_canceling();
  }
  if (cancelled_before_start) {
    result->success = false;
    result->message = "navigation cancelled";
    goal_handle->canceled(result);
    releaseOperation();
    return;
  }

  if (!state_machine_->beginNavigation()) {
    result->success = false;
    result->message =
        "single-point navigation rejected, localization is not ready, "
        "current state=" +
        std::string(toString(state_machine_->mode()));
    goal_handle->abort(result);
    releaseOperation();
    return;
  }
  callbacks_.publish_mode();

  if (!startWorkers(session_id, message)) {
    callbacks_.enter_error(RuntimeState::NAVIGATION_ERROR, message);
    result->success = false;
    result->message = message;
    goal_handle->abort(result);
    releaseOperation();
    return;
  }
  state_machine_->navigationStarted();
  callbacks_.publish_mode();
  feedback = std::make_shared<NavigateToPose::Feedback>();
  feedback->message = "navigating to target";
  goal_handle->publish_feedback(feedback);
  target_pose_pub_->publish(target_pose);

  const bool reached = waitForGoal(config_.goal_timeout_sec, [goal_handle]() {
    return goal_handle->is_canceling();
  });
  bool cancelled = false;
  {
    std::lock_guard<std::mutex> lock(result_mutex_);
    cancelled =
        cancel_requested_ || shutting_down_ || goal_handle->is_canceling();
  }

  if (state_machine_->beginStopNavigation()) callbacks_.publish_mode();
  std::string stop_message;
  const bool stopped = stopWorkers(session_id, stop_message);
  if (stopped) state_machine_->navigationStopped();

  if (cancelled) {
    result->success = false;
    result->message = stopped ? "navigation cancelled"
                              : "navigation cancelled; " + stop_message;
    if (stopped) {
      callbacks_.publish_mode();
    } else {
      callbacks_.enter_error(RuntimeState::NAVIGATION_ERROR, result->message);
    }
    goal_handle->canceled(result);
  } else if (!reached || !stopped) {
    result->success = false;
    result->message = !reached ? "navigation timed out"
                               : "navigation completed but " + stop_message;
    callbacks_.enter_error(RuntimeState::NAVIGATION_ERROR, result->message);
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
  controller_.start(session_id, topology_file_.string(),
                    map_directory_.string(),
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
  controller_.stop(session_id,
                   [completion](bool success, const std::string &detail) {
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
    double timeout_sec, const std::function<bool()> &is_canceling) {
  std::unique_lock<std::mutex> lock(result_mutex_);
  return result_cv_.wait_for(lock, std::chrono::duration<double>(timeout_sec),
                             [this, &is_canceling]() {
                               return goal_reached_ || cancel_requested_ ||
                                      shutting_down_ || is_canceling();
                             }) &&
         goal_reached_;
}

bool NavigationTaskExecutor::waitForStopDuration(
    double duration_sec, const std::function<bool()> &is_canceling) {
  if (duration_sec <= 0.0) return true;
  std::unique_lock<std::mutex> lock(result_mutex_);
  const bool interrupted = result_cv_.wait_for(
      lock, std::chrono::duration<double>(duration_sec),
      [this, &is_canceling]() {
        return cancel_requested_ || shutting_down_ || is_canceling();
      });
  return !interrupted;
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

}  // namespace slam_manager
