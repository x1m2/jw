#include "terrain_filter/terrain_filter.hpp"

#include <clocale>
#include <exception>

#include "slam_manager/managed_worker.hpp"

int main(int argc, char ** argv)
{
  setlocale(LC_ALL, "");
  rclcpp::init(argc, argv);

  try {
    auto worker = std::make_shared<slam_manager::ManagedWorker>(
        "terrain_filter_worker", []() {
          auto node = std::make_shared<rclcpp::Node>(
              "terrain_filter_node");
          auto extractor =
              std::make_shared<TerrainFilter>(node);
          slam_manager::ManagedWorker::AlgorithmInstance instance;
          instance.node = node;
          instance.owner = extractor;
          return instance;
        });
    rclcpp::spin(worker);
    worker.reset();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("terrain_filter_worker"),
                 "节点运行异常: %s", e.what());
    rclcpp::shutdown();
    return -1;
  }

  rclcpp::shutdown();
  return 0;
}
