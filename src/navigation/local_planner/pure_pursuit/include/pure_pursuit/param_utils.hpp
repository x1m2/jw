#ifndef PURE_PURSUIT_LOCAL_PLANNER_PARAM_UTILS_HPP
#define PURE_PURSUIT_LOCAL_PLANNER_PARAM_UTILS_HPP

#include <rclcpp/rclcpp.hpp>
#include <string>

namespace pure_pursuit_local_planner
{

template <typename T>
T declareOrGetParameter(const rclcpp::Node::SharedPtr & node,
                        const std::string & name,
                        const T & default_value)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter<T>(name, default_value);
  }
  return node->get_parameter(name).get_value<T>();
}

}  // namespace pure_pursuit_local_planner

#endif  // PURE_PURSUIT_LOCAL_PLANNER_PARAM_UTILS_HPP
