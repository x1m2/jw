#ifndef COSTMAP_TYPES_HPP
#define COSTMAP_TYPES_HPP

#include <nav_msgs/msg/occupancy_grid.hpp>

namespace pure_pursuit_local_planner {

// Costmap在ROS回调后只读，所有模块共享同一消息对象，避免复制data数组。
using CostmapConstPtr = nav_msgs::msg::OccupancyGrid::ConstSharedPtr;

}  // namespace pure_pursuit_local_planner

#endif  // COSTMAP_TYPES_HPP
