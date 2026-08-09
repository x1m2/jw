#include "robot_runtime/topology_file_writer.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <utility>

namespace robot_runtime {

void TopologyFileWriter::setOutputFile(std::filesystem::path output_file) {
  if (output_file.empty() || !output_file.is_absolute()) {
    throw std::invalid_argument(
        "navigation topology output file must be an absolute path");
  }
  output_file_ = std::move(output_file);
}

const std::filesystem::path &TopologyFileWriter::outputFile() const {
  return output_file_;
}

bool TopologyFileWriter::write(
    const std::vector<robot_interfaces::msg::NavigationNode> &nodes,
    std::string &message) const {
  if (output_file_.empty()) {
    message = "navigation map is not configured; call load_map first";
    return false;
  }
  if (nodes.empty()) {
    message = "cannot generate navigation topology from an empty node list";
    return false;
  }

  std::error_code filesystem_error;
  const auto parent = output_file_.parent_path();
  std::filesystem::create_directories(parent, filesystem_error);
  if (filesystem_error) {
    message = "failed to create topology directory " + parent.string() +
              ": " + filesystem_error.message();
    return false;
  }

  auto temporary_file = output_file_;
  temporary_file += ".tmp";
  {
    std::ofstream output(temporary_file, std::ios::trunc);
    if (!output.is_open()) {
      message = "failed to open temporary topology file: " +
                temporary_file.string();
      return false;
    }

    output << "# Topology map generated from APP navigation nodes\n";
    output << "# Format: internal_id X Y Z label\n";
    output << "# Topology Nodes\n";
    output << std::fixed << std::setprecision(6);
    // 文件使用连续内部ID；APP node_id保留在标签中，用于识别原始节点。
    for (std::size_t index = 0; index < nodes.size(); ++index) {
      const auto &node = nodes[index];
      output << index << " " << node.x << " " << node.y << " " << node.z
             << " app_node_" << node.node_id << "\n";
    }

    output << "# Format: edge_id from_id to_id weight type bidirectional "
              "obstacle_stop_enabled obstacle_avoidance_enabled\n";
    output << "# Topology Edges\n";
    // APP数组顺序就是巡检顺序，相邻节点按三维距离建立双向边。
    for (std::size_t index = 0; index + 1U < nodes.size(); ++index) {
      const auto &from = nodes[index];
      const auto &to = nodes[index + 1U];
      const double dx = to.x - from.x;
      const double dy = to.y - from.y;
      const double dz = to.z - from.z;
      const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
      output << index << " " << index << " " << (index + 1U) << " "
             << distance << " auto 1 1 0\n";
    }
    output.flush();
    if (!output) {
      output.close();
      std::filesystem::remove(temporary_file, filesystem_error);
      message = "failed while writing topology file: " +
                temporary_file.string();
      return false;
    }
  }

  // 原子替换正式文件，避免全局规划器启动时读到未写完的拓扑。
  filesystem_error.clear();
  std::filesystem::rename(temporary_file, output_file_, filesystem_error);
  if (filesystem_error) {
    const std::string rename_error = filesystem_error.message();
    std::error_code cleanup_error;
    std::filesystem::remove(temporary_file, cleanup_error);
    message = "failed to publish topology file " + output_file_.string() +
              ": " + rename_error;
    return false;
  }

  message = "navigation topology generated: " + output_file_.string() +
            ", nodes=" + std::to_string(nodes.size()) + ", edges=" +
            std::to_string(nodes.size() - 1U);
  return true;
}

}  // namespace robot_runtime
