#pragma once

#include <memory>

#include <rclcpp/node.hpp>

namespace rviz_topology_editor {

// 节点实现保留在cpp中，通过工厂函数隔离PCL等内部依赖。
rclcpp::Node::SharedPtr makeRvizTopologyEditorNode();

} // namespace rviz_topology_editor
