#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "pgo_sc.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PgoSc>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                    4U);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}