#include <exception>
#include <memory>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>

#include "local_planner/costmap_2d.hpp"
#include "slam_manager/managed_worker.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  try {
    auto worker = std::make_shared<slam_manager::ManagedWorker>(
        "costmap_worker",
        [](const slam_manager::ManagedWorker::StartContext &context) {
          rclcpp::NodeOptions options;
          if (context.algorithm_profile == "uwb_follow") {
            options.append_parameter_override("global_frame", "odom");
            options.append_parameter_override("pointcloud2_in_base_frame", true);
            options.append_parameter_override("pointcloud2_apply_height_filter",
                                              false);
            options.append_parameter_override("clear_unobserved_rays", false);
          }
          auto node = std::make_shared<rclcpp::Node>("costmap_2d", options);
          auto costmap = std::make_shared<local_planner::CostMap2D>(node);
          if (!costmap->initialize()) {
            throw std::runtime_error("Failed to initialize costmap");
          }
          slam_manager::ManagedWorker::AlgorithmInstance instance;
          instance.node = node;
          instance.owner = costmap;
          return instance;
        });
    rclcpp::spin(worker);
    worker.reset();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("costmap_worker"),
                 "CostMap2D node failed: %s", e.what());
    rclcpp::shutdown();
    return -1;
  }

  rclcpp::shutdown();
  return 0;
}
