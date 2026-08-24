#include "slam_manager/task_topology.hpp"

#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <nlohmann/json.hpp>

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

}  // namespace slam_manager
