#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "laser_mapping.hpp"
#include "slam_manager/managed_worker.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions algorithm_options;

  auto worker = std::make_shared<slam_manager::ManagedWorker>(
      "fast_lio_worker", [algorithm_options]() {
        slam_manager::ManagedWorker::AlgorithmInstance instance;
        instance.node = std::make_shared<LaserMappingNode>(algorithm_options);
        return instance;
      });

  rclcpp::spin(worker);
  worker.reset();
  rclcpp::shutdown();
  return 0;
}
