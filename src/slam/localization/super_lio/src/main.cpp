#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "slam_manager/managed_worker.hpp"
#include "lio/super_lio_reloc.h"

struct LocalizationInstance {
  std::shared_ptr<LI2Sup::SuperLIOReLoc> lio;
  rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto worker = std::make_shared<slam_manager::ManagedWorker>(
      "super_lio_localization_worker", [](const slam_manager::ManagedWorker::StartContext &context) {
        rclcpp::NodeOptions options;
        if (!context.resource_path.empty()) options.append_parameter_override("map_path", context.resource_path);
        if (context.use_initial_pose) options.append_parameter_override("relocalization.algorithm_type", "manually_set");
        if (!context.algorithm_profile.empty()) options.append_parameter_override("relocalization_profile", context.algorithm_profile);
        auto node = std::make_shared<LI2Sup::ROSWrapper>(options);
        auto owner = std::make_shared<LocalizationInstance>();
        owner->lio = std::make_shared<LI2Sup::SuperLIOReLoc>();
        owner->lio->setROSWrapper(node);
        owner->lio->init();
        std::weak_ptr<LI2Sup::SuperLIOReLoc> weak_lio = owner->lio;
        owner->timer = node->create_wall_timer(std::chrono::milliseconds(2), [weak_lio]() {
          if (auto lio = weak_lio.lock()) lio->process();
        }, node->getSensorCallbackGroup());
        slam_manager::ManagedWorker::AlgorithmInstance instance;
        instance.node = node;
        instance.owner = owner;
        instance.before_stop = [owner]() { owner->lio->requestStop(); owner->timer->cancel(); };
        return instance;
      });
  rclcpp::spin(worker);
  worker.reset();
  rclcpp::shutdown();
  return 0;
}
