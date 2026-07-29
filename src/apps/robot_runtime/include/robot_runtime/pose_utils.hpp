#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace robot_runtime::pose_utils {

inline geometry_msgs::msg::Pose fromXyzTheta(double x, double y, double z,
                                              double theta) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;

  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, theta);
  pose.orientation = tf2::toMsg(orientation);
  return pose;
}

inline double yaw(
    const geometry_msgs::msg::Quaternion &orientation) {
  return tf2::getYaw(orientation);
}

}  // namespace robot_runtime::pose_utils
