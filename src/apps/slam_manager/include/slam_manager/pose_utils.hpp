#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace slam_manager::pose_utils {

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

inline void rpy(const geometry_msgs::msg::Quaternion &orientation,
                double &roll, double &pitch, double &yaw_angle) {
  tf2::Quaternion quaternion;
  tf2::fromMsg(orientation, quaternion);
  tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw_angle);
}

}  // namespace slam_manager::pose_utils
