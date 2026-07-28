#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "rviz_topology_editor/rviz_topology_editor_node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(rviz_topology_editor::makeRvizTopologyEditorNode());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("rviz_topology_editor"), "%s",
                 error.what());
  }
  rclcpp::shutdown();
  return 0;
}
