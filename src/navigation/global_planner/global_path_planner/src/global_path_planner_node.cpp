#include "global_path_planner/global_path_planner.h"

#include <clocale>
#include <memory>

int main(int argc, char ** argv)
{
    setlocale(LC_ALL, "");

    rclcpp::init(argc, argv);

    auto planner = std::make_shared<global_path_planner::GlobalPathPlanner>();

    if (!planner->initialize())
    {
        RCLCPP_ERROR(planner->get_logger(), "全局路径规划器初始化失败");
        rclcpp::shutdown();
        return -1;
    }

    RCLCPP_INFO(planner->get_logger(), "全局路径规划器节点启动");
    rclcpp::spin(planner);

    rclcpp::shutdown();
    return 0;
}
