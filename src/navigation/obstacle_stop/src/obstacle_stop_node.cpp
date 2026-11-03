#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>

#include "obstacle_stop/obstacle_stop.hpp"
#include "slam_manager/managed_worker.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto worker = std::make_shared<slam_manager::ManagedWorker>(
        "obstacle_stop_worker",
        [](const slam_manager::ManagedWorker::StartContext &context) {
          rclcpp::NodeOptions options;
          if (!context.resource_id.empty()) {
            options.append_parameter_override("topology_map_file",
                                              context.resource_id);
          }
          if (context.algorithm_profile == "uwb_follow") {
            options.append_parameter_override("global_frame", "odom");
          }
          auto node = std::make_shared<rclcpp::Node>("obstacle_stop", options);
          auto obstacle_stop =
              std::make_shared<obstacle_stop::ObstacleStop>(node);
          std::string initialization_message;
          if (!obstacle_stop->initialize(initialization_message)) {
            throw std::runtime_error(initialization_message);
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
