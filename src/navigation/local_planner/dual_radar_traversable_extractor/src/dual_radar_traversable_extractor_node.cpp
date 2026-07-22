#include "dual_radar_traversable_extractor/dual_radar_traversable_extractor.hpp"

#include <clocale>
#include <exception>

#include "robot_runtime/managed_worker.hpp"

int main(int argc, char ** argv)
{
  setlocale(LC_ALL, "");
  rclcpp::init(argc, argv);

  try {
    auto worker = std::make_shared<robot_runtime::ManagedWorker>(
        "dual_radar_worker", []() {
          auto node = std::make_shared<rclcpp::Node>(
              "dual_radar_traversable_extractor");
          auto extractor =
              std::make_shared<DualRadarTraversableExtractor>(node);
          robot_runtime::ManagedWorker::AlgorithmInstance instance;
          instance.node = node;
          instance.owner = extractor;
          return instance;
        });
    rclcpp::spin(worker);
    worker.reset();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("dual_radar_worker"),
                 "节点运行异常: %s", e.what());
    rclcpp::shutdown();
    return -1;
  }

  rclcpp::shutdown();
  return 0;
}
