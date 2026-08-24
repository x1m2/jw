#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

#include "slam_manager_interfaces/msg/navigation_node.hpp"

namespace slam_manager {

struct TaskEdgePolicy {
  double linear_velocity{0.5};
  double angular_velocity{0.5};
  int8_t motion_direction{1};
  bool obstacle_stop_enabled{true};
  bool obstacle_avoidance_enabled{false};
  double obstacle_avoidance_distance_m{0.5};
};

struct TaskTopologyEdge {
  uint32_t id{0U};
  uint32_t from_id{0U};
  uint32_t to_id{0U};
  std::string edge_type{"straight"};
  TaskEdgePolicy policy;
  bool has_bezier_control_points{false};
  std::array<std::array<double, 3>, 2> bezier_control_points{};
};

struct TaskTopologyGraph {
  std::vector<slam_manager_interfaces::msg::NavigationNode> nodes;
  std::vector<TaskTopologyEdge> edges;
};

using DirectedEdgeIndex = std::unordered_map<uint64_t, std::size_t>;

uint64_t makeDirectedEdgeKey(uint32_t from_id, uint32_t to_id);
bool parseTaskJson(const std::filesystem::path &path, TaskTopologyGraph &graph,
                   DirectedEdgeIndex &directed_edge_indexes,
                   std::string &message);
const TaskTopologyEdge *findDirectEdge(
    const TaskTopologyGraph &graph,
    const DirectedEdgeIndex &directed_edge_indexes, uint32_t from_id,
    uint32_t to_id);

// 将合并后的任务拓扑原子保存为全局规划器使用的runtime JSON。
class TopologyFileWriter {
 public:
  TopologyFileWriter() = default;

  void setOutputFile(std::filesystem::path output_file);
  const std::filesystem::path &outputFile() const;
  bool write(const std::vector<slam_manager_interfaces::msg::NavigationNode> &nodes,
             std::string &message) const;
  bool write(const TaskTopologyGraph &graph, std::string &message) const;

 private:
  std::filesystem::path output_file_;
};

}  // namespace slam_manager
