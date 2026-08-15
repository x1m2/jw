#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "pgo_sc.hpp"
#include "slam_manager/managed_worker.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto worker = std::make_shared<slam_manager::ManagedWorker>(
      "pgo_worker",
      [](const slam_manager::ManagedWorker::StartContext &context) {
        slam_manager::ManagedWorker::AlgorithmInstance instance;
        rclcpp::NodeOptions options;
        if (!context.resource_path.empty()) {
          options.append_parameter_override("basic.package_path",
                                            context.resource_path);
        }
        auto pgo = std::make_shared<PgoSc>(options);
        instance.node = pgo;
        if (!context.resource_path.empty()) {
          instance.before_stop = [pgo]() { pgo->saveMap(); };
        }
        return instance;
      });

  rclcpp::spin(worker);
  worker.reset();
  rclcpp::shutdown();
  return 0;
}
