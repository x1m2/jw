#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "slam_manager/slam_manager_node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // 多线程执行器用于同时处理管理服务和异步 Worker 响应。
  auto node = std::make_shared<slam_manager::SlamManagerNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                    4U);
  executor.add_node(node);
  executor.spin();

  executor.remove_node(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
