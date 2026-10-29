#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "slam_manager/managed_worker.hpp"
#include "uwb_follow_planner/uwb_follow_planner.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto worker = std::make_shared<slam_manager::ManagedWorker>(
      "uwb_follow_planner_worker",
      [](const slam_manager::ManagedWorker::StartContext &context) {
        rclcpp::NodeOptions options;
        if (context.algorithm_profile == "managed_follow") {
          // Manager owns the external BLE command. The algorithm only receives
          // commands after the complete follow stack is ready.
          options.append_parameter_override(
              "follow_command_topic", "/slam_manager/uwb_follow_cmd");
          options.append_parameter_override("manage_pure_pursuit_pause", true);
        }
        slam_manager::ManagedWorker::AlgorithmInstance instance;
        instance.node =
            std::make_shared<uwb_follow_planner::UwbFollowPlanner>(options);
        return instance;
      });
  rclcpp::spin(worker);
  worker.reset();
  rclcpp::shutdown();
  return 0;
}
