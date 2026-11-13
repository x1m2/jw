#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "pgo_sc.hpp"
#include "slam_manager/managed_worker.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto worker = std::make_shared<slam_manager::ManagedWorker>(
      "pgo_worker",
      [](const slam_manager::ManagedWorker::StartContext &) {
        slam_manager::ManagedWorker::AlgorithmInstance instance;
        rclcpp::NodeOptions options;
        auto pgo = std::make_shared<PgoSc>(options);
        instance.node = pgo;
        instance.save = [pgo](const std::string &map_path) {
          return pgo->saveMap(map_path);
        };
        return instance;
      });

  rclcpp::spin(worker);
  worker.reset();
  rclcpp::shutdown();
  return 0;
}
