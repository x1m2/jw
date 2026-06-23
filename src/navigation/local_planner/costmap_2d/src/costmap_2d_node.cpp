#include <exception>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "local_planner/costmap_2d.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("costmap_2d_node");

  RCLCPP_INFO(node->get_logger(), "Starting CostMap2D node...");

  try {
    // 创建代价地图对象
    local_planner::CostMap2D costmap(node);

    // 初始化
    if (!costmap.initialize()) {
      RCLCPP_ERROR(node->get_logger(), "Failed to initialize costmap");
      rclcpp::shutdown();
      return -1;
    }

    RCLCPP_INFO(node->get_logger(), "CostMap2D node started successfully");

    // 运行ROS2循环
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "CostMap2D node failed: %s", e.what());
    rclcpp::shutdown();
    return -1;
  }

  RCLCPP_INFO(node->get_logger(), "CostMap2D node shutting down");

  rclcpp::shutdown();
  return 0;
}