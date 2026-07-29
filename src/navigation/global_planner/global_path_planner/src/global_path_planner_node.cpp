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
        "global_path_planner_worker",
        [](const robot_runtime::ManagedWorker::StartContext &context) {
            rclcpp::NodeOptions options;
            if (!context.resource_id.empty()) {
                options.append_parameter_override(
                    "topology_map_file", context.resource_id);
            }
            if (!context.resource_path.empty()) {
                options.append_parameter_override(
                    "free_space_pcd_file",
                    context.resource_path + "/result_free_space.pcd");
            }
            auto planner =
                std::make_shared<global_path_planner::GlobalPathPlanner>(
                    options);
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
