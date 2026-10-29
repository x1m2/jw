#include <stdexcept>

#include "pure_pursuit/pure_pursuit_local_planner.hpp"
#include "slam_manager/managed_worker.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto worker = std::make_shared<
      slam_manager::ManagedWorker>(
      "pure_pursuit_worker",
      [](const slam_manager::ManagedWorker::StartContext &context) {
    rclcpp::NodeOptions options;
    if (context.algorithm_profile == "uwb_follow") {
      options.append_parameter_override("global_frame", "odom");
    }
    auto node = std::make_shared<rclcpp::Node>("pure_pursuit", options);
    auto planner =
        std::make_shared<pure_pursuit_local_planner::PurePursuitLocalPlanner>();
    if (!planner->initialize(node)) {
      throw std::runtime_error(
          "Failed to initialize Pure Pursuit Local Planner");
    }
    if (context.algorithm_profile == "uwb_follow") {
      // Never expose a stale path or a nonzero velocity during follow startup.
      planner->setPaused(true);
    }
    slam_manager::ManagedWorker::AlgorithmInstance instance;
    instance.node = node;
    instance.owner = planner;
    instance.before_stop = [planner]() { planner->stop(); };
    return instance;
  });
  rclcpp::spin(worker);
  worker.reset();
  rclcpp::shutdown();
  return 0;
}
