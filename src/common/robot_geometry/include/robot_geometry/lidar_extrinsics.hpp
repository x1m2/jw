#pragma once

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace robot_geometry::lidar_extrinsics {

// Correct tilt with the smallest rotation in base axes. Heading is referenced
// to the configured mount; gravity alone does not measure yaw.
inline Eigen::Matrix3d levelRotation(const Eigen::Matrix3d &base_from_lidar,
                                     const Eigen::Vector3d &mean_acc_lidar) {
  if (!base_from_lidar.allFinite() || !mean_acc_lidar.allFinite() ||
      mean_acc_lidar.norm() <= 1e-3) {
    throw std::invalid_argument("Invalid mount rotation or mean acceleration");
  }
  // Normalize the input to remove small decimal rounding errors in YAML.
  const Eigen::Quaterniond mount = Eigen::Quaterniond(base_from_lidar).normalized();
  const Eigen::Vector3d gravity_base = mount * (-mean_acc_lidar.normalized());
  const Eigen::Quaterniond correction = Eigen::Quaterniond::FromTwoVectors(
      gravity_base, -Eigen::Vector3d::UnitZ()).normalized();
  return (correction * mount).normalized().toRotationMatrix();
}

// p_base = R * p_lidar + t. Matrix values are row-major; RPY is Rz * Ry * Rx.
inline Eigen::Matrix4d decode(const std::vector<double> &translation,
                              const std::string &type,
                              const std::vector<double> &rotation,
                              const std::string &name = "lidar_to_base") {
  const auto fail = [&name](const std::string &reason) {
    throw std::invalid_argument(name + ": " + reason);
  };
  if (translation.size() != 3) fail("translation must contain 3 values in meters");
  for (const auto *values : {&translation, &rotation}) {
    for (double value : *values) {
      if (!std::isfinite(value)) fail("all values must be finite");
    }
  }

  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  Eigen::Matrix3d R;
  if (type == "rpy_rad") {
    if (rotation.size() != 3) fail("rpy_rad requires [roll, pitch, yaw]");
    R = (Eigen::AngleAxisd(rotation[2], Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(rotation[1], Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(rotation[0], Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
  } else if (type == "matrix") {
    if (rotation.size() != 9) fail("matrix requires 9 row-major values");
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) R(row, col) = rotation[row * 3 + col];
    }
  } else {
    fail("rotation.type must be rpy_rad or matrix");
  }
  // Allow decimal rounding, but reject scaling, reflections and invalid frames.
  if (!R.allFinite() ||
      (R.transpose() * R - Eigen::Matrix3d::Identity()).norm() > 1e-5 ||
      std::abs(R.determinant() - 1.0) > 1e-5) {
    fail("rotation must be orthogonal with determinant +1");
  }
  transform.topLeftCorner<3, 3>() = R;
  transform.topRightCorner<3, 1>() =
      Eigen::Vector3d(translation[0], translation[1], translation[2]);
  return transform;
}

inline Eigen::Matrix4d load(rclcpp::Node &node,
                            const std::string &name = "lidar_to_base",
                            const Eigen::Matrix4d &fallback = Eigen::Matrix4d::Identity()) {
  // Fail explicitly for old robot files instead of silently using identity.
  const auto old_t = node.declare_parameter<std::vector<double>>(name + "_T", std::vector<double>{});
  const auto old_r = node.declare_parameter<std::vector<double>>(name + "_R", std::vector<double>{});
  if (!old_t.empty() || !old_r.empty()) {
    throw std::invalid_argument(name + "_T/R is obsolete; use " + name +
                                ".translation and .rotation.{type,value}");
  }
  const auto translation = node.declare_parameter<std::vector<double>>(
      name + ".translation", std::vector<double>{});
  const auto type = node.declare_parameter<std::string>(name + ".rotation.type", "");
  const auto rotation = node.declare_parameter<std::vector<double>>(
      name + ".rotation.value", std::vector<double>{});
  if (translation.empty() && type.empty() && rotation.empty()) {
    RCLCPP_WARN(node.get_logger(), "%s not configured; using default transform", name.c_str());
    return fallback;
  }
  return decode(translation, type, rotation, name);
}

}  // namespace robot_geometry::lidar_extrinsics
