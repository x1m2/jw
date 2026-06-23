#include <rclcpp/rclcpp.hpp>

#include "localization_sc.hpp"

int main(int argc, char **argv) {
  // 初始化 ROS 2
  rclcpp::init(argc, argv);

  // 创建节点
  auto node = std::make_shared<FastLioLocalizationScQn>("localization_sc_node");

  // 创建多线程 executor（默认线程数 = CPU 核心数）
  rclcpp::executors::MultiThreadedExecutor executor;

  // 添加节点
  executor.add_node(node);

  // 开始执行（会阻塞直到 rclcpp::shutdown()）
  executor.spin();

  // 关闭 ROS 2
  rclcpp::shutdown();
  return 0;
}