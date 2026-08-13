#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "uwb_follow_planner/uwb_follow_planner.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<uwb_follow_planner::UwbFollowPlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
