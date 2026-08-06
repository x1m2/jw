#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "robot_runtime/managed_worker.hpp"
#include "ros2_main.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  const rclcpp::NodeOptions algorithm_options;
  auto worker = std::make_shared<robot_runtime::ManagedWorker>(
      "fastlio_localization_worker",
      [algorithm_options](
          const robot_runtime::ManagedWorker::StartContext &context) {
        robot_runtime::ManagedWorker::AlgorithmInstance instance;
        auto options = algorithm_options;
        if (!context.resource_path.empty()) {
          options.append_parameter_override("map_path", context.resource_path);
        }
        if (context.use_initial_pose) {
          options.append_parameter_override("algorithm_type",
                                            "manually_set");
        }
        auto localization =
            std::make_shared<FastlioLocalizationNode>(options);
        localization->initialize();
        instance.node = localization;
        instance.before_stop = [localization]() {
          localization->requestStop();
        };
        return instance;
      });

  rclcpp::spin(worker);
  worker.reset();
  rclcpp::shutdown();
  return 0;
}
