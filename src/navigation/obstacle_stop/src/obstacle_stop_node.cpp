#include <memory>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>

#include "obstacle_stop/obstacle_stop.hpp"
#include "slam_manager/managed_worker.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto worker = std::make_shared<slam_manager::ManagedWorker>(
        "obstacle_stop_worker", []() {
          auto node = std::make_shared<rclcpp::Node>("obstacle_stop");
          auto obstacle_stop = std::make_shared<obstacle_stop::ObstacleStop>(node);
          if (!obstacle_stop->initialize()) {
            throw std::runtime_error("failed to initialize obstacle stop");
          }
          slam_manager::ManagedWorker::AlgorithmInstance instance;
          instance.node = node;
          instance.owner = obstacle_stop;
          instance.before_stop = [obstacle_stop]() { obstacle_stop->stop(); };
          return instance;
        });
    rclcpp::spin(worker);
    worker.reset();
  } catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("obstacle_stop_worker"), "%s",
                 error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
