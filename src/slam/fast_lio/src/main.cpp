#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "laser_mapping.hpp"
#include "robot_runtime/managed_worker.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions algorithm_options;

  auto worker = std::make_shared<robot_runtime::ManagedWorker>(
      "fast_lio_worker", [algorithm_options]() {
        robot_runtime::ManagedWorker::AlgorithmInstance instance;
        instance.node = std::make_shared<LaserMappingNode>(algorithm_options);
        return instance;
      });

  rclcpp::spin(worker);
  worker.reset();
  rclcpp::shutdown();
  return 0;
}
