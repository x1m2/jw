#pragma once

#include <stdexcept>
#include <string>
#include <rclcpp/rclcpp.hpp>

namespace scan_planner {

// Multiple components share one node; declare each topic parameter only once.
inline std::string topicName(rclcpp::Node &node, const std::string &key,
                             const std::string &fallback) {
  const auto parameter = "topics." + key;
  if (!node.has_parameter(parameter))
    node.declare_parameter<std::string>(parameter, fallback);
  const auto topic = node.get_parameter(parameter).as_string();
  if (topic.empty()) throw std::invalid_argument(parameter + " must not be empty");
  return topic;
}

}  // namespace scan_planner
