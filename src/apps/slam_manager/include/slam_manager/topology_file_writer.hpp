#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include "slam_manager_interfaces/msg/navigation_node.hpp"

namespace slam_manager {

// 将APP有序节点转换为全局规划器使用的顺序双向拓扑文件。
class TopologyFileWriter {
 public:
  TopologyFileWriter() = default;

  void setOutputFile(std::filesystem::path output_file);
  const std::filesystem::path &outputFile() const;
  bool write(const std::vector<slam_manager_interfaces::msg::NavigationNode> &nodes,
             std::string &message) const;

 private:
  std::filesystem::path output_file_;
};

}  // namespace slam_manager
