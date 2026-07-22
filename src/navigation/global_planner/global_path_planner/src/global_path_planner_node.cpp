#include "global_path_planner/global_path_planner.h"

#include <clocale>
#include <memory>
#include <stdexcept>

#include "robot_runtime/managed_worker.hpp"

int main(int argc, char ** argv)
{
    setlocale(LC_ALL, "");

    rclcpp::init(argc, argv);

    auto worker = std::make_shared<robot_runtime::ManagedWorker>(
        "global_path_planner_worker", []() {
            auto planner =
                std::make_shared<global_path_planner::GlobalPathPlanner>();
            if (!planner->initialize()) {
                throw std::runtime_error("全局路径规划器初始化失败");
            }
            robot_runtime::ManagedWorker::AlgorithmInstance instance;
            instance.node = planner;
            return instance;
        });

    rclcpp::spin(worker);
    worker.reset();

    rclcpp::shutdown();
    return 0;
}
