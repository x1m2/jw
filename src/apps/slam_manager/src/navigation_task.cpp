#include "slam_manager/navigation_task.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>
#include <tf2/utils.h>

#include "slam_manager/pose_utils.hpp"
#include "slam_manager_interfaces/msg/runtime_state.hpp"

namespace slam_manager {
namespace {

bool finiteNonNegative(double value) {
  return std::isfinite(value) && value >= 0.0;
}

double distance3d(double first_x, double first_y, double first_z,
                  double second_x, double second_y, double second_z) {
  const double dx = second_x - first_x;
  const double dy = second_y - first_y;
  const double dz = second_z - first_z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double edgeLength(const TaskTopologyEdge &edge,
                  const slam_manager_interfaces::msg::NavigationNode &from,
                  const slam_manager_interfaces::msg::NavigationNode &to) {
  if (edge.edge_type != "bezier" || !edge.has_bezier_control_points) {
    return distance3d(from.x, from.y, from.z, to.x, to.y, to.z);
  }
  constexpr int kSamples = 100;
  double length = 0.0;
  double previous_x = from.x;
  double previous_y = from.y;
  double previous_z = from.z;
  for (int index = 1; index <= kSamples; ++index) {
    const double t = static_cast<double>(index) / kSamples;
    const double one_minus_t = 1.0 - t;
    const double start_weight = one_minus_t * one_minus_t * one_minus_t;
    const double first_weight = 3.0 * one_minus_t * one_minus_t * t;
    const double second_weight = 3.0 * one_minus_t * t * t;
    const double end_weight = t * t * t;
    const double x = start_weight * from.x +
                     first_weight * edge.bezier_control_points[0][0] +
                     second_weight * edge.bezier_control_points[1][0] +
                     end_weight * to.x;
    const double y = start_weight * from.y +
                     first_weight * edge.bezier_control_points[0][1] +
                     second_weight * edge.bezier_control_points[1][1] +
                     end_weight * to.y;
    const double z = start_weight * from.z +
                     first_weight * edge.bezier_control_points[0][2] +
                     second_weight * edge.bezier_control_points[1][2] +
                     end_weight * to.z;
    length += distance3d(previous_x, previous_y, previous_z, x, y, z);
    previous_x = x;
    previous_y = y;
    previous_z = z;
  }
  return length;
}

bool publishTemporaryFile(const std::filesystem::path &temporary_file,
                          const std::filesystem::path &output_file,
                          std::string &message) {
  std::error_code filesystem_error;
  std::filesystem::rename(temporary_file, output_file, filesystem_error);
  if (!filesystem_error)
    return true;
  const std::string rename_error = filesystem_error.message();
  std::error_code cleanup_error;
  std::filesystem::remove(temporary_file, cleanup_error);
  message = "failed to publish runtime topology JSON " + output_file.string() +
            ": " + rename_error;
  return false;
}

}  // namespace

uint64_t makeDirectedEdgeKey(uint32_t from_id, uint32_t to_id) {
  return (static_cast<uint64_t>(from_id) << 32U) | static_cast<uint64_t>(to_id);
}

bool parseTaskJson(const std::filesystem::path &path, TaskTopologyGraph &graph,
                   DirectedEdgeIndex &directed_edge_indexes,
                   std::string &message) {
  std::ifstream input(path);
  if (!input.is_open()) {
    message = "failed to open patrol JSON: " + path.string();
    return false;
  }

  try {
    const nlohmann::json document = nlohmann::json::parse(input);
    if (!document.is_object() || !document.contains("nodes") ||
        !document["nodes"].is_array() || !document.contains("edges") ||
        !document["edges"].is_array()) {
      message = "patrol JSON must contain nodes[] and edges[]";
      return false;
    }

    std::unordered_set<uint32_t> node_ids;
    for (const auto &json_node : document["nodes"]) {
      if (!json_node.is_object()) {
        message = "patrol JSON nodes must be objects";
        return false;
      }
      const uint64_t raw_id = json_node.at("id").get<uint64_t>();
      if (raw_id > static_cast<uint64_t>(std::numeric_limits<int>::max() - 2)) {
        message =
            "JSON node id exceeds planner range: " + std::to_string(raw_id);
        return false;
      }
      slam_manager_interfaces::msg::NavigationNode node;
      node.node_id = static_cast<uint32_t>(raw_id);
      node.x = json_node.at("x").get<double>();
      node.y = json_node.at("y").get<double>();
      node.z = json_node.at("z").get<double>();
      node.theta = json_node.at("theta").get<double>();
      node.stop_duration_sec =
          json_node.value("stop_duration_sec", 0.0);
      if (!node_ids.insert(node.node_id).second) {
        message = "duplicate JSON node id=" + std::to_string(raw_id);
        return false;
      }
      if (!std::isfinite(node.x) || !std::isfinite(node.y) ||
          !std::isfinite(node.z) || !std::isfinite(node.theta) ||
          !finiteNonNegative(node.stop_duration_sec)) {
        message = "JSON node contains an invalid pose or stop duration: id=" +
                  std::to_string(raw_id);
        return false;
      }
      graph.nodes.push_back(node);
    }
    if (graph.nodes.empty()) {
      message = "patrol JSON nodes[] must not be empty";
      return false;
    }

    std::unordered_set<uint32_t> edge_ids;
    directed_edge_indexes.reserve(document["edges"].size());
    for (const auto &json_edge : document["edges"]) {
      if (!json_edge.is_object()) {
        message = "patrol JSON edges must be objects";
        return false;
      }
      const uint64_t raw_id = json_edge.at("id").get<uint64_t>();
      const uint64_t raw_from_id = json_edge.at("from_id").get<uint64_t>();
      const uint64_t raw_to_id = json_edge.at("to_id").get<uint64_t>();
      if (raw_id > static_cast<uint64_t>(std::numeric_limits<int>::max()) ||
          raw_from_id >
              static_cast<uint64_t>(std::numeric_limits<int>::max()) ||
          raw_to_id > static_cast<uint64_t>(std::numeric_limits<int>::max())) {
        message = "JSON edge or referenced node id is out of range";
        return false;
      }

      TaskTopologyEdge edge;
      edge.id = static_cast<uint32_t>(raw_id);
      edge.from_id = static_cast<uint32_t>(raw_from_id);
      edge.to_id = static_cast<uint32_t>(raw_to_id);
      edge.edge_type = json_edge.value("edge_type", "straight");
      edge.policy.linear_velocity = json_edge.value("linear_velocity", 0.5);
      edge.policy.angular_velocity = json_edge.value("angular_velocity", 0.5);
      edge.policy.motion_direction = json_edge.value("motion_direction", 1);
      edge.policy.obstacle_stop_enabled =
          json_edge.value("obstacle_stop_enabled", true);
      edge.policy.obstacle_avoidance_enabled =
          json_edge.value("obstacle_avoidance_enabled", false);
      edge.policy.obstacle_avoidance_distance_m =
          json_edge.value("obstacle_avoidance_distance_m", 0.5);

      if (!edge_ids.insert(edge.id).second) {
        message = "duplicate JSON edge id=" + std::to_string(edge.id);
        return false;
      }
      const uint64_t directed_edge_key =
          makeDirectedEdgeKey(edge.from_id, edge.to_id);
      if (!directed_edge_indexes.emplace(directed_edge_key, graph.edges.size())
               .second) {
        message = "multiple directed JSON edges connect node " +
                  std::to_string(edge.from_id) + " -> " +
                  std::to_string(edge.to_id);
        return false;
      }
      if (node_ids.find(edge.from_id) == node_ids.end() ||
          node_ids.find(edge.to_id) == node_ids.end()) {
        message = "JSON edge references an unknown node: edge_id=" +
                  std::to_string(edge.id);
        return false;
      }
      if (edge.edge_type != "straight" && edge.edge_type != "map_connection" &&
          edge.edge_type != "bezier") {
        message =
            "unsupported edge_type for edge_id=" + std::to_string(edge.id);
        return false;
      }
      if (!finiteNonNegative(edge.policy.linear_velocity) ||
          !finiteNonNegative(edge.policy.angular_velocity) ||
          !finiteNonNegative(edge.policy.obstacle_avoidance_distance_m) ||
          (edge.policy.motion_direction != 1 &&
           edge.policy.motion_direction != -1)) {
        message = "invalid edge policy for edge_id=" + std::to_string(edge.id);
        return false;
      }

      if (edge.edge_type == "bezier") {
        const auto &control_points = json_edge.at("bezier_control_points");
        if (!control_points.is_array() || control_points.size() != 2U) {
          message =
              "bezier edge requires exactly two control points: edge_id=" +
              std::to_string(edge.id);
          return false;
        }
        for (std::size_t index = 0; index < 2U; ++index) {
          const auto &point = control_points[index];
          edge.bezier_control_points[index] = {point.at("x").get<double>(),
                                               point.at("y").get<double>(),
                                               point.at("z").get<double>()};
          for (double coordinate : edge.bezier_control_points[index]) {
            if (!std::isfinite(coordinate)) {
              message = "bezier control point is not finite: edge_id=" +
                        std::to_string(edge.id);
              return false;
            }
          }
        }
        edge.has_bezier_control_points = true;
      } else if (json_edge.contains("bezier_control_points")) {
        message = "only bezier edges may define control points: edge_id=" +
                  std::to_string(edge.id);
        return false;
      }
      graph.edges.push_back(edge);
    }
  } catch (const nlohmann::json::exception &error) {
    message = "invalid patrol JSON " + path.string() + ": " + error.what();
    return false;
  }
  return true;
}

const TaskTopologyEdge *findDirectEdge(
    const TaskTopologyGraph &graph,
    const DirectedEdgeIndex &directed_edge_indexes, uint32_t from_id,
    uint32_t to_id) {
  const auto edge =
      directed_edge_indexes.find(makeDirectedEdgeKey(from_id, to_id));
  if (edge == directed_edge_indexes.end() ||
      edge->second >= graph.edges.size()) {
    return nullptr;
  }
  return &graph.edges[edge->second];
}

void TopologyFileWriter::setOutputFile(std::filesystem::path output_file) {
  if (output_file.empty() || !output_file.is_absolute()) {
    throw std::invalid_argument(
        "runtime topology JSON path must be an absolute path");
  }
  output_file_ = std::move(output_file);
}

const std::filesystem::path &TopologyFileWriter::outputFile() const {
  return output_file_;
}

bool TopologyFileWriter::write(
    const std::vector<slam_manager_interfaces::msg::NavigationNode> &nodes,
    std::string &message) const {
  if (nodes.empty()) {
    message = "cannot generate runtime topology from an empty node list";
    return false;
  }
  TaskTopologyGraph graph;
  graph.nodes = nodes;
  graph.edges.reserve(nodes.size() > 1U ? (nodes.size() - 1U) * 2U : 0U);
  uint32_t edge_id = 0U;
  for (std::size_t index = 0; index + 1U < nodes.size(); ++index) {
    TaskTopologyEdge forward;
    forward.id = edge_id++;
    forward.from_id = nodes[index].node_id;
    forward.to_id = nodes[index + 1U].node_id;
    graph.edges.push_back(forward);
    TaskTopologyEdge reverse = forward;
    reverse.id = edge_id++;
    reverse.from_id = forward.to_id;
    reverse.to_id = forward.from_id;
    graph.edges.push_back(reverse);
  }
  return write(graph, message);
}

bool TopologyFileWriter::write(const TaskTopologyGraph &graph,
                               std::string &message) const {
  if (output_file_.empty()) {
    message = "navigation map is not configured; call load_map first";
    return false;
  }
  if (graph.nodes.empty()) {
    message = "cannot generate runtime topology from an empty graph";
    return false;
  }

  std::error_code filesystem_error;
  const auto parent = output_file_.parent_path();
  if (!std::filesystem::is_directory(parent, filesystem_error) ||
      filesystem_error) {
    message = filesystem_error
                  ? "failed to inspect topology directory " + parent.string() +
                        ": " + filesystem_error.message()
                  : "topology directory does not exist: " + parent.string();
    return false;
  }

  std::unordered_map<uint32_t,
                     const slam_manager_interfaces::msg::NavigationNode *>
      nodes_by_id;
  nodes_by_id.reserve(graph.nodes.size());
  for (const auto &node : graph.nodes) {
    if (!nodes_by_id.emplace(node.node_id, &node).second) {
      message = "duplicate runtime node id=" +
                std::to_string(node.node_id);
      return false;
    }
  }

  nlohmann::json document;
  document["schema_version"] = 1;
  document["format"] = "navigation_topology";
  document["source"] = "slam_manager_runtime";
  document["frame_id"] = "map";
  document["nodes"] = nlohmann::json::array();
  document["edges"] = nlohmann::json::array();
  for (const auto &navigation_node : graph.nodes) {
    document["nodes"].push_back(
        {{"id", navigation_node.node_id},
         {"x", navigation_node.x},
         {"y", navigation_node.y},
         {"z", navigation_node.z},
         {"theta", navigation_node.theta},
         {"stop_duration_sec", navigation_node.stop_duration_sec}});
  }
  for (const auto &edge : graph.edges) {
    const auto from_it = nodes_by_id.find(edge.from_id);
    const auto to_it = nodes_by_id.find(edge.to_id);
    if (from_it == nodes_by_id.end() || to_it == nodes_by_id.end()) {
      message =
          "edge_id=" + std::to_string(edge.id) + " references an unknown node";
      return false;
    }
    nlohmann::json json_edge = {
        {"id", edge.id},
        {"from_id", edge.from_id},
        {"to_id", edge.to_id},
        {"weight", edgeLength(edge, *from_it->second, *to_it->second)},
        {"edge_type", edge.edge_type},
        {"linear_velocity", edge.policy.linear_velocity},
        {"angular_velocity", edge.policy.angular_velocity},
        {"motion_direction", edge.policy.motion_direction},
        {"obstacle_stop_enabled", edge.policy.obstacle_stop_enabled},
        {"obstacle_avoidance_enabled", edge.policy.obstacle_avoidance_enabled},
        {"obstacle_avoidance_distance_m",
         edge.policy.obstacle_avoidance_distance_m}};
    if (edge.edge_type == "bezier" && edge.has_bezier_control_points) {
      json_edge["bezier_control_points"] = nlohmann::json::array();
      for (const auto &control_point : edge.bezier_control_points) {
        json_edge["bezier_control_points"].push_back({{"x", control_point[0]},
                                                      {"y", control_point[1]},
                                                      {"z", control_point[2]}});
      }
    }
    document["edges"].push_back(std::move(json_edge));
  }

  auto temporary_file = output_file_;
  temporary_file += ".tmp";
  {
    std::ofstream output(temporary_file, std::ios::trunc);
    if (!output.is_open()) {
      message = "failed to open temporary runtime topology JSON: " +
                temporary_file.string();
      return false;
    }
    output << document.dump(2) << '\n';
    output.flush();
    if (!output) {
      output.close();
      std::filesystem::remove(temporary_file, filesystem_error);
      message = "failed while writing runtime topology JSON: " +
                temporary_file.string();
      return false;
    }
  }

  if (!publishTemporaryFile(temporary_file, output_file_, message))
    return false;
  message = "runtime topology JSON generated: " + output_file_.string() +
            ", nodes=" + std::to_string(graph.nodes.size()) +
            ", edges=" + std::to_string(graph.edges.size());
  return true;
}

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
      !callbacks_.relocalization_succeeded || !callbacks_.get_current_pose ||
      !callbacks_.relocalize) {
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
  feedback.theta = tf2::getYaw(current_pose.pose.orientation);
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

bool NavigationTaskExecutor::beginNavigationIfLocalized() {
  return callbacks_.relocalization_succeeded() &&
         state_machine_->beginNavigation();
}

slam_manager_interfaces::msg::NavigationNode
NavigationTaskExecutor::nodeFromPose(uint32_t node_id,
                                     const geometry_msgs::msg::Pose &pose) {
  slam_manager_interfaces::msg::NavigationNode node;
  node.node_id = node_id;
  node.x = pose.position.x;
  node.y = pose.position.y;
  node.z = pose.position.z;
  node.theta = tf2::getYaw(pose.orientation);
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
  std::lock_guard<std::mutex> execution_lock(execution_mutex_);
  if (execution_thread_.joinable()) execution_thread_.join();
  execution_thread_ = std::thread(&NavigationTaskExecutor::executePatrolAction,
                                  this, goal_handle, session_id);
}

void NavigationTaskExecutor::startNavigateToPoseAction(
    const std::shared_ptr<NavigateToPoseGoalHandle> &goal_handle,
    const geometry_msgs::msg::PoseStamped &current_pose, uint64_t session_id) {
  std::lock_guard<std::mutex> execution_lock(execution_mutex_);
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

bool NavigationTaskExecutor::stopAndWait(uint64_t session_id,
                                         std::string &message) {
  std::lock_guard<std::mutex> execution_lock(execution_mutex_);
  cancel();
  if (execution_thread_.joinable()) execution_thread_.join();
  // 再发送一次幂等STOP，确保启动早期退出或异常路径没有遗留Worker。
  return stopWorkers(session_id, message);
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
    // 没有任务JSON时保留基础巡检能力：直接使用Action数组，并为相邻节点
    // 生成默认双向直线边。此路径没有自定义边策略和区域配置。
    patrol_nodes.assign(goal->nodes.begin(), goal->nodes.end());
    topology_written = topology_writer_->write(patrol_nodes, topology_message);
    if (topology_written) {
      RCLCPP_WARN(node_->get_logger(),
                  "[APP][NAVIGATION_FALLBACK] task JSON does not exist for "
                  "task_id='%s': %s; using %zu Action nodes with default "
                  "straight edges and no region settings",
                  goal->task_id.c_str(), task_json_file.string().c_str(),
                  patrol_nodes.size());
    }
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

  if (!beginNavigationIfLocalized()) {
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

        if (!beginNavigationIfLocalized()) {
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
    const bool client_cancel = goal_handle->is_canceling();
    result->success = false;
    const std::string interruption_message =
        client_cancel ? "navigation cancelled by client"
                      : "navigation preempted by lifecycle command";
    result->message = stopped ? interruption_message
                              : interruption_message + "; " + stop_message;
    if (!stopped) {
      callbacks_.enter_error(RuntimeState::NAVIGATION_ERROR, result->message);
    } else {
      callbacks_.publish_mode();
    }
    // CANCELED只允许从CANCELING状态进入。load_map等生命周期命令只是
    // 内部请求导航线程退出，此时Action仍为EXECUTING，必须使用ABORTED。
    if (client_cancel) {
      goal_handle->canceled(result);
    } else {
      goal_handle->abort(result);
    }
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
    const bool client_cancel = goal_handle->is_canceling();
    result->success = false;
    result->message = client_cancel
                          ? "navigation cancelled by client"
                          : "navigation preempted by lifecycle command";
    if (client_cancel) {
      goal_handle->canceled(result);
    } else {
      goal_handle->abort(result);
    }
    releaseOperation();
    return;
  }

  if (!beginNavigationIfLocalized()) {
    result->success = false;
    result->message =
        "single-point navigation rejected, relocalization has not succeeded, "
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
    const bool client_cancel = goal_handle->is_canceling();
    result->success = false;
    const std::string interruption_message =
        client_cancel ? "navigation cancelled by client"
                      : "navigation preempted by lifecycle command";
    result->message = stopped ? interruption_message
                              : interruption_message + "; " + stop_message;
    if (stopped) {
      callbacks_.publish_mode();
    } else {
      callbacks_.enter_error(RuntimeState::NAVIGATION_ERROR, result->message);
    }
    if (client_cancel) {
      goal_handle->canceled(result);
    } else {
      goal_handle->abort(result);
    }
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
  auto operation_token = std::make_shared<WorkerOperationToken>();
  const auto result = awaitWorkerCommand(
      "navigation workers start", config_.worker_timeout_sec, operation_token,
      [this, session_id, operation_token](WorkerCompletion completion) {
        controller_.start(session_id, topology_file_.string(),
                          map_directory_.string(), operation_token,
                          std::move(completion));
      },
      [this]() {
        std::lock_guard<std::mutex> lock(result_mutex_);
        return !cancel_requested_ && !shutting_down_;
      },
      [this, session_id](WorkerCompletion completion) {
        controller_.stop(session_id, std::move(completion));
      },
      config_.worker_timeout_sec);
  message = result.message;
  return result.success();
}

bool NavigationTaskExecutor::stopWorkers(uint64_t session_id,
                                         std::string &message) {
  const auto result = awaitWorkerCommand(
      "navigation workers stop", config_.worker_timeout_sec,
      std::make_shared<WorkerOperationToken>(),
      [this, session_id](WorkerCompletion completion) {
        controller_.stop(session_id, std::move(completion));
      });
  message = result.message;
  return result.success();
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
