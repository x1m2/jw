#include "path_smoother/path_smoother.hpp"

#include <rclcpp/rclcpp.hpp>

#include <clocale>
#include <exception>
#include <memory>

int main(int argc, char ** argv)
{
  std::setlocale(LC_ALL, "");

  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("path_smoother_node");

  RCLCPP_INFO(node->get_logger(), "启动路径平滑节点...");

  try {
    path_smoother::PathSmoother smoother(node);

    if (!smoother.initialize()) {
      RCLCPP_ERROR(node->get_logger(), "路径平滑器初始化失败");
      rclcpp::shutdown();
      return -1;
    }

    RCLCPP_INFO(node->get_logger(), "路径平滑节点启动成功");
    smoother.spin();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "路径平滑节点运行异常: %s", e.what());
    rclcpp::shutdown();
    return -1;
  }

  RCLCPP_INFO(node->get_logger(), "路径平滑节点已退出");
  rclcpp::shutdown();
  return 0;
}
