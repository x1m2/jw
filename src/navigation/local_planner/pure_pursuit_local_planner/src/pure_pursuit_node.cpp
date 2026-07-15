#include "pure_pursuit_local_planner/pure_pursuit_local_planner.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("pure_pursuit_local_planner_node");

    pure_pursuit_local_planner::PurePursuitLocalPlanner planner;
    if (!planner.initialize(node))
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to initialize Pure Pursuit Local Planner");
        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "Pure Pursuit Local Planner initialized successfully");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
