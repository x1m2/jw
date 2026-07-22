#include <stdexcept>

#include "pure_pursuit_local_planner/pure_pursuit_local_planner.hpp"
#include "robot_runtime/managed_worker.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto worker = std::make_shared<
      robot_runtime::ManagedWorker>("pure_pursuit_worker", []() {
    auto node = std::make_shared<rclcpp::Node>("pure_pursuit_local_planner");
    auto planner =
        std::make_shared<pure_pursuit_local_planner::PurePursuitLocalPlanner>();
    if (!planner->initialize(node)) {
      throw std::runtime_error(
          "Failed to initialize Pure Pursuit Local Planner");
    }
    robot_runtime::ManagedWorker::AlgorithmInstance instance;
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
