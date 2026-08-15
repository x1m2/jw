#include "path_smoother/path_smoother.hpp"

#include <rclcpp/rclcpp.hpp>

#include <clocale>
#include <exception>
#include <memory>
#include <stdexcept>

#include "slam_manager/managed_worker.hpp"

int main(int argc, char ** argv)
{
  std::setlocale(LC_ALL, "");

  rclcpp::init(argc, argv);

  try {
    auto worker = std::make_shared<slam_manager::ManagedWorker>(
        "path_smoother_worker", []() {
          auto node = std::make_shared<rclcpp::Node>("path_smoother");
          auto smoother = std::make_shared<path_smoother::PathSmoother>(node);
          if (!smoother->initialize()) {
            throw std::runtime_error("路径平滑器初始化失败");
          }
          slam_manager::ManagedWorker::AlgorithmInstance instance;
          instance.node = node;
          instance.owner = smoother;
          return instance;
        });
    rclcpp::spin(worker);
    worker.reset();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("path_smoother_worker"),
                 "路径平滑节点运行异常: %s", e.what());
    rclcpp::shutdown();
    return -1;
  }

  rclcpp::shutdown();
  return 0;
}
