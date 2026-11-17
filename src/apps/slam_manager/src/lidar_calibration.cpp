#include "slam_manager/slam_manager_node.hpp"
#include "robot_geometry/lidar_extrinsics.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <map>
#include <random>
#include <sstream>
#include <stdexcept>
#include <tuple>

namespace slam_manager {

namespace {
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
using Points = std::vector<Eigen::Vector3d>;

// Callers protect updates and snapshots with their sampling-window mutex.
struct ImuStatistics {
  Eigen::Vector3d acceleration_sum{Eigen::Vector3d::Zero()};
  double acceleration_squared_norm_sum{0.0};
  double gyro_squared_norm_sum{0.0};
  std::size_t count{0U};

  bool add(const sensor_msgs::msg::Imu &message) {
    const Eigen::Vector3d acceleration(message.linear_acceleration.x,
                                       message.linear_acceleration.y,
                                       message.linear_acceleration.z);
    const Eigen::Vector3d gyro(message.angular_velocity.x,
                               message.angular_velocity.y,
                               message.angular_velocity.z);
    if (!acceleration.allFinite() || !gyro.allFinite()) return false;
    acceleration_sum += acceleration;
    acceleration_squared_norm_sum += acceleration.squaredNorm();
    gyro_squared_norm_sum += gyro.squaredNorm();
    ++count;
    return true;
  }

  struct Summary {
    Eigen::Vector3d mean_acceleration{Eigen::Vector3d::Zero()};
    double acceleration_variance{0.0};
    double accel_relative_std{0.0};
    double gyro_rms{0.0};
    std::size_t count{0U};

    bool stationary(double max_gyro_rms, double max_accel_relative_std) const {
      return count >= 2 && mean_acceleration.norm() > 1e-3 &&
          std::isfinite(acceleration_variance) &&
          std::isfinite(accel_relative_std) && std::isfinite(gyro_rms) &&
          accel_relative_std <= max_accel_relative_std && gyro_rms <= max_gyro_rms;
    }
  };

  Summary summarize() const {
    Summary result;
    result.count = count;
    if (count == 0) return result;
    result.mean_acceleration = acceleration_sum / static_cast<double>(count);
    result.acceleration_variance = acceleration_squared_norm_sum / count -
                                   result.mean_acceleration.squaredNorm();
    result.accel_relative_std =
        std::sqrt(std::max(0.0, result.acceleration_variance)) /
        result.mean_acceleration.norm();
    result.gyro_rms = std::sqrt(gyro_squared_norm_sum / count);
    return result;
  }
};

struct PlaneResult {
  Eigen::Vector3d normal{Eigen::Vector3d::UnitZ()};
  double offset{0.0};
  double rmse{0.0};
  double ratio{0.0};
};

double tiltDegrees(const Eigen::Vector3d &normal) {
  return std::atan2(normal.head<2>().norm(), normal.z()) * kRadToDeg;
}

double normalDifference(const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
  return std::atan2(a.cross(b).norm(), a.dot(b)) * kRadToDeg;
}

// Each occupied voxel contributes one point, independent of scan density.
// Fit only near-horizontal planes and require spatially distributed support.
template<typename Config>
PlaneResult fitGround(const Points &points, const Config &cfg) {
  if (points.size() < static_cast<std::size_t>(cfg.min_points))
    throw std::runtime_error("Not enough ground ROI voxels; inspect ground_roi, base_height_m and height_tolerance_m");
  std::mt19937 generator(5489U);
  std::uniform_int_distribution<std::size_t> pick(0, points.size() - 1);
  std::vector<std::size_t> best;
  for (int iteration = 0; iteration < cfg.ransac_iterations; ++iteration) {
    const auto a = pick(generator), b = pick(generator), c = pick(generator);
    if (a == b || a == c || b == c) continue;
    Eigen::Vector3d n = (points[b] - points[a]).cross(points[c] - points[a]);
    if (n.norm() < 1e-8) continue;
    n.normalize();
    if (n.z() < 0.0) n = -n;
    if (tiltDegrees(n) > cfg.max_correction_deg) continue;
    const double d = -n.dot(points[a]);
    std::vector<std::size_t> inliers;
    for (std::size_t i = 0; i < points.size(); ++i)
      if (std::abs(n.dot(points[i]) + d) <= cfg.distance_threshold) inliers.push_back(i);
    if (inliers.size() > best.size()) best = std::move(inliers);
  }
  if (best.size() < static_cast<std::size_t>(cfg.min_points))
    throw std::runtime_error("No supported near-horizontal ground plane in ROI");
  PlaneResult result;
  // Refit by total least squares, then update inliers and refit once more.
  for (int pass = 0; pass < 2; ++pass) {
    if (best.size() < static_cast<std::size_t>(cfg.min_points))
      throw std::runtime_error("Too few ground inliers after refinement");
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    for (auto i : best) center += points[i];
    center /= static_cast<double>(best.size());
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (auto i : best) {
      const Eigen::Vector3d delta = points[i] - center;
      covariance.noalias() += delta * delta.transpose();
    }
    covariance /= static_cast<double>(best.size());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success || solver.eigenvalues()[1] < 0.0025)
      throw std::runtime_error("Ground support is degenerate or too narrow");
    result.normal = solver.eigenvectors().col(0);
    if (result.normal.z() < 0.0) result.normal = -result.normal;
    result.offset = -result.normal.dot(center);
    if (pass == 0) {
      best.clear();
      for (std::size_t i = 0; i < points.size(); ++i)
        if (std::abs(result.normal.dot(points[i]) + result.offset) <= cfg.distance_threshold)
          best.push_back(i);
    }
  }
  if (tiltDegrees(result.normal) > cfg.max_correction_deg)
    throw std::runtime_error("Ground correction exceeds max_correction_deg");
  Eigen::Vector3d low = points[best.front()], high = low;
  double squared_error = 0.0;
  for (auto i : best) {
    low = low.cwiseMin(points[i]); high = high.cwiseMax(points[i]);
    const double error = result.normal.dot(points[i]) + result.offset;
    squared_error += error * error;
  }
  bool covered[3][3] = {};
  for (auto i : best) {
    const int x = std::min(2, static_cast<int>(3.0 * (points[i].x() - low.x()) /
                                              std::max(1e-9, high.x() - low.x())));
    const int y = std::min(2, static_cast<int>(3.0 * (points[i].y() - low.y()) /
                                              std::max(1e-9, high.y() - low.y())));
    covered[x][y] = true;
  }
  int cells = 0;
  for (const auto &row : covered) for (bool value : row) cells += value ? 1 : 0;
  result.ratio = static_cast<double>(best.size()) / points.size();
  result.rmse = std::sqrt(squared_error / best.size());
  if (high.x() - low.x() < cfg.min_x_span || high.y() - low.y() < cfg.min_y_span || cells < 6)
    throw std::runtime_error("Ground inliers do not cover enough longitudinal/lateral area");
  if (result.ratio < cfg.min_inlier_ratio || result.rmse > cfg.max_rmse)
    throw std::runtime_error("Ground plane inlier ratio or RMS did not pass acceptance");
  return result;
}

sensor_msgs::msg::PointCloud2 cloudMessage(const Points &points, const rclcpp::Time &stamp) {
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.frame_id = "base_link";
  msg.header.stamp = stamp;
  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());
  sensor_msgs::PointCloud2Iterator<float> x(msg, "x"), y(msg, "y"), z(msg, "z");
  for (const auto &point : points) {
    *x = point.x(); *y = point.y(); *z = point.z();
    ++x; ++y; ++z;
  }
  return msg;
}
}  // namespace

void SlamManagerNode::configureLidarCalibration() {
  imu_calibration_topic_ =
      declare_parameter<std::string>("imu_calibration.topic", imu_calibration_topic_);
  const auto configured_imu_calibration_sample_count =
      declare_parameter<int64_t>("imu_calibration.sample_count",
                                 static_cast<int64_t>(imu_calibration_sample_count_));
  imu_calibration_timeout_sec_ =
      declare_parameter<double>("imu_calibration.timeout_sec", imu_calibration_timeout_sec_);
  imu_calibration_max_gyro_rms_ =
      declare_parameter<double>("imu_calibration.max_gyro_rms_rad_s", imu_calibration_max_gyro_rms_);
  imu_calibration_max_accel_relative_std_ =
      declare_parameter<double>("imu_calibration.max_accel_relative_std", imu_calibration_max_accel_relative_std_);
  bool mount_valid = false;
  try {
    const Eigen::Matrix4d mount = robot_geometry::lidar_extrinsics::load(*this);
    calibration_base_from_lidar_ = mount.topLeftCorner<3, 3>();
    calibration_translation_ = mount.topRightCorner<3, 1>();
    calibration_rotation_type_ =
        get_parameter("lidar_to_base.rotation.type").as_string();
    if (calibration_rotation_type_ == "rpy_rad") {
      calibration_yaw_rad_ =
          get_parameter("lidar_to_base.rotation.value").as_double_array()[2];
    }
    mount_valid = !calibration_rotation_type_.empty();
  } catch (const std::exception &error) {
    RCLCPP_ERROR(get_logger(), "Invalid calibration mount: %s", error.what());
  }
  const auto imu_calibration_extrinsic_rotation =
      declare_parameter<std::vector<double>>(
          "imu_calibration.extrinsic_R",
          std::vector<double>{});
  if (mount_valid && !imu_calibration_topic_.empty() &&
      configured_imu_calibration_sample_count >= 2 &&
      std::isfinite(imu_calibration_timeout_sec_) &&
      imu_calibration_timeout_sec_ > 0.0 &&
      std::isfinite(imu_calibration_max_gyro_rms_) &&
      imu_calibration_max_gyro_rms_ > 0.0 &&
      std::isfinite(imu_calibration_max_accel_relative_std_) &&
      imu_calibration_max_accel_relative_std_ > 0.0 &&
      imu_calibration_extrinsic_rotation.size() == 9U) {
    imu_from_lidar_rotation_ =
        Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
            imu_calibration_extrinsic_rotation.data());
    imu_calibration_config_valid_ =
        imu_from_lidar_rotation_.allFinite() &&
        (imu_from_lidar_rotation_.transpose() * imu_from_lidar_rotation_)
            .isApprox(Eigen::Matrix3d::Identity(), 1e-3) &&
        std::abs(imu_from_lidar_rotation_.determinant() - 1.0) < 1e-3;
  }
  if (imu_calibration_config_valid_) {
    imu_calibration_sample_count_ =
        static_cast<std::size_t>(configured_imu_calibration_sample_count);
    RCLCPP_INFO(get_logger(),
                "LiDAR tilt calibration configured: topic=%s, samples=%zu, "
                "timeout=%.3fs, extrinsic_R=[%.8f, %.8f, %.8f; "
                "%.8f, %.8f, %.8f; %.8f, %.8f, %.8f]",
                imu_calibration_topic_.c_str(), imu_calibration_sample_count_,
                imu_calibration_timeout_sec_,
                imu_from_lidar_rotation_(0, 0), imu_from_lidar_rotation_(0, 1),
                imu_from_lidar_rotation_(0, 2), imu_from_lidar_rotation_(1, 0),
                imu_from_lidar_rotation_(1, 1), imu_from_lidar_rotation_(1, 2),
                imu_from_lidar_rotation_(2, 0), imu_from_lidar_rotation_(2, 1),
                imu_from_lidar_rotation_(2, 2));
  } else {
    RCLCPP_ERROR(get_logger(),
                 "LiDAR tilt calibration disabled: load a robot YAML with a "
                 "nonempty imu_calibration.topic and a valid 3x3 imu_calibration.extrinsic_R; "
                 "a valid lidar_to_base mount is required; sample_count must be >= 2, "
                 "timeout and motion thresholds must be positive and finite");
  }
  auto &g = ground_calibration_;
  const std::string prefix = "imu_calibration.ground_fitting.";
  g.enabled = declare_parameter<bool>(prefix + "enabled", g.enabled);
  g.topic = declare_parameter<std::string>("lidars.front.topic", g.topic);
  g.frame = declare_parameter<std::string>("lidars.front.frame", g.frame);
  g.cloud_type = declare_parameter<std::string>("imu_calibration.pointcloud_type", g.cloud_type);
  g.collection_sec = declare_parameter<double>(prefix + "collection_sec", g.collection_sec);
  g.validation_sec = declare_parameter<double>(prefix + "validation_sec", g.validation_sec);
  g.x_min = declare_parameter<double>(prefix + "x_min", g.x_min);
  g.x_max = declare_parameter<double>(prefix + "x_max", g.x_max);
  g.y_min = declare_parameter<double>(prefix + "y_min", g.y_min);
  g.y_max = declare_parameter<double>(prefix + "y_max", g.y_max);
  g.base_height_m = declare_parameter<double>(prefix + "base_height_m", g.base_height_m);
  g.height_tolerance_m = declare_parameter<double>(prefix + "height_tolerance_m", g.height_tolerance_m);
  g.voxel_size = declare_parameter<double>(prefix + "voxel_size", g.voxel_size);
  g.distance_threshold = declare_parameter<double>(prefix + "distance_threshold", g.distance_threshold);
  g.min_inlier_ratio = declare_parameter<double>(prefix + "min_inlier_ratio", g.min_inlier_ratio);
  g.min_x_span = declare_parameter<double>(prefix + "min_x_span", g.min_x_span);
  g.min_y_span = declare_parameter<double>(prefix + "min_y_span", g.min_y_span);
  g.max_correction_deg = declare_parameter<double>(prefix + "max_correction_deg", g.max_correction_deg);
  g.max_residual_deg = declare_parameter<double>(prefix + "max_residual_deg", g.max_residual_deg);
  g.max_rmse = declare_parameter<double>(prefix + "max_rmse", g.max_rmse);
  g.max_repeat_deg = declare_parameter<double>(prefix + "max_repeat_deg", g.max_repeat_deg);
  g.min_points = declare_parameter<int>(prefix + "min_points", g.min_points);
  g.min_frames = declare_parameter<int>(prefix + "min_frames", g.min_frames);
  g.ransac_iterations = declare_parameter<int>(prefix + "ransac_iterations", g.ransac_iterations);
  ground_calibration_config_valid_ = !g.topic.empty() && !g.frame.empty() &&
      (g.cloud_type == "pointcloud2" || g.cloud_type == "livox") &&
      g.x_min < g.x_max && g.y_min < g.y_max && g.min_points >= 3 &&
      g.min_frames >= 2 && g.ransac_iterations > 0 &&
      g.min_inlier_ratio > 0.0 && g.min_inlier_ratio <= 1.0 &&
      g.min_x_span <= g.x_max - g.x_min && g.min_y_span <= g.y_max - g.y_min &&
      g.max_correction_deg < 45.0 && g.base_height_m >= 0.0;
  for (double value : {g.collection_sec, g.validation_sec, g.height_tolerance_m, g.voxel_size,
       g.distance_threshold, g.min_x_span, g.min_y_span, g.max_correction_deg,
       g.max_residual_deg, g.max_rmse, g.max_repeat_deg})
    ground_calibration_config_valid_ &= std::isfinite(value) && value > 0.0;
  for (double value : {g.x_min, g.x_max, g.y_min, g.y_max, g.base_height_m, g.min_inlier_ratio})
    ground_calibration_config_valid_ &= std::isfinite(value);
  if (g.enabled) {
    if (!ground_calibration_config_valid_)
      RCLCPP_ERROR(get_logger(), "Ground fitting enabled with invalid configuration; calibration will fail");
    const auto qos = rclcpp::QoS(1).reliable().durability_volatile();
    calibration_roi_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/robot/calibration/ground_roi", qos);
    calibration_corrected_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/robot/calibration/ground_corrected", qos);
  }
  RCLCPP_INFO(get_logger(), "Ground fitting: enabled=%s, topic=%s, type=%s, "
              "ROI x=[%.2f, %.2f], y=[%.2f, %.2f], base_height=%.2f m, "
              "height_tolerance=+/- %.2f m",
              g.enabled ? "true" : "false", g.topic.c_str(), g.cloud_type.c_str(),
              g.x_min, g.x_max, g.y_min, g.y_max, g.base_height_m, g.height_tolerance_m);
}

bool SlamManagerNode::refineLidarTiltWithGround(
    Eigen::Matrix3d &rotation, std::string &message,
    std::vector<Eigen::Vector3d> &validation_points_lidar) {
  const auto cfg = ground_calibration_;
  const Eigen::Matrix3d imu_rotation = rotation;
  const Eigen::Vector3d translation = calibration_translation_;
  struct MotionData {
    std::mutex mutex;
    ImuStatistics statistics;
  };
  const auto motion = std::make_shared<MotionData>();
  rclcpp::SubscriptionOptions options;
  options.callback_group = imu_calibration_callback_group_;
  auto imu_sub = create_subscription<sensor_msgs::msg::Imu>(
      imu_calibration_topic_, rclcpp::SensorDataQoS(),
      [motion](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(motion->mutex);
        motion->statistics.add(*msg);
      }, options);
  const auto motion_ok = [&]() {
    std::lock_guard<std::mutex> lock(motion->mutex);
    return motion->statistics.summarize().stationary(
        imu_calibration_max_gyro_rms_, imu_calibration_max_accel_relative_std_);
  };
  const auto reset_motion = [&]() {
    std::lock_guard<std::mutex> lock(motion->mutex);
    motion->statistics = ImuStatistics{};
  };
  const auto collect = [&](double seconds, const char *stage) -> Points {
    struct Window {
      std::mutex mutex;
      std::condition_variable cv;
      std::map<std::tuple<int, int, int>, Eigen::Vector3d> voxels;
      std::string error;
      std::size_t frames{0};
      int64_t last_stamp{-1};
      bool active{true};
    };
    const auto window = std::make_shared<Window>();
    reset_motion();
    const auto accept = [window, cfg, imu_rotation, translation](
        const std_msgs::msg::Header &header, const auto &read_points) {
      std::lock_guard<std::mutex> lock(window->mutex);
      if (!window->active || !window->error.empty()) return;
      if (header.frame_id != cfg.frame) {
        window->error = "Raw point cloud frame mismatch: expected " + cfg.frame +
                        ", got " + header.frame_id;
        window->cv.notify_one(); return;
      }
      const int64_t stamp = static_cast<int64_t>(header.stamp.sec) * 1000000000LL +
                            header.stamp.nanosec;
      if (stamp <= 0 || stamp <= window->last_stamp) return;
      window->last_stamp = stamp;
      try {
        read_points([&](double x, double y, double z) {
          const Eigen::Vector3d raw(x, y, z);
          if (!raw.allFinite()) return;
          const Eigen::Vector3d p = imu_rotation * raw + translation;
          if (p.x() < cfg.x_min || p.x() > cfg.x_max ||
              p.y() < cfg.y_min || p.y() > cfg.y_max ||
              std::abs(p.z() + cfg.base_height_m) > cfg.height_tolerance_m) return;
          if (window->voxels.size() >= 50000) return;
          const auto key = std::make_tuple(static_cast<int>(std::floor(p.x() / cfg.voxel_size)),
              static_cast<int>(std::floor(p.y() / cfg.voxel_size)),
              static_cast<int>(std::floor(p.z() / cfg.voxel_size)));
          window->voxels.emplace(key, p);
        });
        ++window->frames;
      } catch (const std::exception &error) {
        window->error = error.what(); window->cv.notify_one();
      }
    };
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc2_sub;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_sub;
    if (cfg.cloud_type == "livox") {
      livox_sub = create_subscription<livox_ros_driver2::msg::CustomMsg>(
          cfg.topic, rclcpp::SensorDataQoS(),
          [accept](livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg) {
            accept(msg->header, [&](const auto &add) {
              for (const auto &p : msg->points) add(p.x, p.y, p.z);
            });
          }, options);
    } else {
      pc2_sub = create_subscription<sensor_msgs::msg::PointCloud2>(
          cfg.topic, rclcpp::SensorDataQoS(),
          [accept](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
            accept(msg->header, [&](const auto &add) {
              if (msg->is_bigendian || msg->point_step == 0 ||
                  msg->row_step < static_cast<uint64_t>(msg->width) * msg->point_step ||
                  msg->data.size() < static_cast<uint64_t>(msg->height) * msg->row_step)
                throw std::runtime_error("Unsupported or malformed PointCloud2 layout");
              uint32_t offsets[3] = {};
              for (int axis = 0; axis < 3; ++axis) {
                const std::string name = axis == 0 ? "x" : axis == 1 ? "y" : "z";
                const auto field = std::find_if(msg->fields.begin(), msg->fields.end(),
                    [&](const auto &f) { return f.name == name; });
                if (field == msg->fields.end() || field->datatype !=
                    sensor_msgs::msg::PointField::FLOAT32 || field->count != 1 ||
                    static_cast<uint64_t>(field->offset) + sizeof(float) > msg->point_step)
                  throw std::runtime_error("PointCloud2 needs float32 x/y/z fields");
                offsets[axis] = field->offset;
              }
              for (uint32_t row = 0; row < msg->height; ++row) {
                for (uint32_t col = 0; col < msg->width; ++col) {
                  const auto *p = msg->data.data() + static_cast<std::size_t>(row) * msg->row_step +
                                  static_cast<std::size_t>(col) * msg->point_step;
                  float xyz[3];
                  for (int axis = 0; axis < 3; ++axis)
                    std::memcpy(&xyz[axis], p + offsets[axis], sizeof(float));
                  add(xyz[0], xyz[1], xyz[2]);
                }
              }
            });
          }, options);
    }
    RCLCPP_INFO(get_logger(), "Ground %s: collecting %.1fs from %s", stage, seconds, cfg.topic.c_str());
    std::unique_lock<std::mutex> lock(window->mutex);
    window->cv.wait_for(lock, std::chrono::duration<double>(seconds),
                        [&] { return !window->error.empty(); });
    window->active = false;
    const auto error = window->error;
    const auto frames = window->frames;
    Points points;
    for (const auto &entry : window->voxels) points.push_back(entry.second);
    lock.unlock();
    pc2_sub.reset(); livox_sub.reset();
    if (!error.empty()) throw std::runtime_error(error);
    if (!rclcpp::ok()) throw std::runtime_error("Calibration interrupted by ROS shutdown");
    if (frames < static_cast<std::size_t>(cfg.min_frames))
      throw std::runtime_error("Too few fresh cloud frames; check topic/type/frame and timestamps");
    if (!motion_ok()) throw std::runtime_error("IMU motion/noise or missing IMU during ground collection");
    RCLCPP_INFO(get_logger(), "Ground %s: %zu frames, %zu ROI voxels", stage, frames, points.size());
    return points;
  };
  try {
    const Points training = collect(cfg.collection_sec, "fit");
    if (calibration_roi_pub_->get_subscription_count() > 0U ||
        calibration_roi_pub_->get_intra_process_subscription_count() > 0U) {
      calibration_roi_pub_->publish(cloudMessage(training, now()));
    }
    const PlaneResult fitted = fitGround(training, cfg);
    const Eigen::Matrix3d correction = Eigen::Quaterniond::FromTwoVectors(
        fitted.normal, Eigen::Vector3d::UnitZ()).normalized().toRotationMatrix();
    const Points validation_a = collect(cfg.validation_sec / 2.0, "validation A");
    const Points validation_b = collect(cfg.validation_sec / 2.0, "validation B");
    const PlaneResult a = fitGround(validation_a, cfg), b = fitGround(validation_b, cfg);
    const double residual = std::max(tiltDegrees(correction * a.normal),
                                     tiltDegrees(correction * b.normal));
    const double repeat = normalDifference(a.normal, b.normal);
    if (residual > cfg.max_residual_deg || repeat > cfg.max_repeat_deg)
      throw std::runtime_error("Independent ground validation failed: residual=" +
          std::to_string(residual) + " deg, repeat=" + std::to_string(repeat) + " deg");
    // Return points in LiDAR axes; the caller applies the final output rotation,
    // including the configured Euler yaw for rpy_rad mounts, before publishing.
    validation_points_lidar.clear();
    validation_points_lidar.reserve(validation_b.size());
    for (const auto &p : validation_b)
      validation_points_lidar.push_back(imu_rotation.transpose() * (p - translation));
    rotation = correction * imu_rotation;
    std::ostringstream summary;
    summary << "IMU + ground calibrated: correction=" << tiltDegrees(fitted.normal)
            << " deg, validation_residual=" << residual << " deg, repeat=" << repeat
            << " deg, RMS=" << std::max({fitted.rmse, a.rmse, b.rmse}) << " m";
    message = summary.str();
    RCLCPP_INFO(get_logger(), "%s; normal=[%.6f, %.6f, %.6f], inlier_ratio=%.3f",
                message.c_str(), fitted.normal.x(), fitted.normal.y(), fitted.normal.z(), fitted.ratio);
    return true;
  } catch (const std::exception &error) {
    message = error.what();
    RCLCPP_WARN(get_logger(), "Ground calibration rejected: %s", message.c_str());
    return false;
  }
}

void SlamManagerNode::calibrateLidarTiltCallback(
    const CalibrateLidarTilt::Request::SharedPtr,
    CalibrateLidarTilt::Response::SharedPtr response) {
  if (calibration_roi_pub_ &&
      (calibration_roi_pub_->get_subscription_count() > 0U ||
       calibration_roi_pub_->get_intra_process_subscription_count() > 0U)) {
    calibration_roi_pub_->publish(cloudMessage({}, now()));
  }
  if (calibration_corrected_pub_ &&
      (calibration_corrected_pub_->get_subscription_count() > 0U ||
       calibration_corrected_pub_->get_intra_process_subscription_count() > 0U)) {
    calibration_corrected_pub_->publish(cloudMessage({}, now()));
  }
  if (!imu_calibration_config_valid_ ||
      (ground_calibration_.enabled && !ground_calibration_config_valid_)) {
    response->success = false;
    response->message = "Invalid IMU calibration or lidar_to_base configuration";
    RCLCPP_ERROR(get_logger(),
                 "LiDAR tilt calibration rejected: invalid configuration");
    return;
  }
  struct CalibrationData {
    std::mutex mutex;
    std::condition_variable cv;
    ImuStatistics statistics;
  };
  const auto data = std::make_shared<CalibrationData>();

  rclcpp::SubscriptionOptions options;
  options.callback_group = imu_calibration_callback_group_;
  auto imu_subscription = create_subscription<sensor_msgs::msg::Imu>(
      imu_calibration_topic_, rclcpp::SensorDataQoS(),
      [data, target_samples = imu_calibration_sample_count_](
          const sensor_msgs::msg::Imu::SharedPtr message) {
        std::lock_guard<std::mutex> lock(data->mutex);
        if (data->statistics.count >= target_samples) return;
        if (!data->statistics.add(*message)) return;
        if (data->statistics.count >= target_samples) data->cv.notify_one();
      },
      options);

  RCLCPP_INFO(get_logger(),
              "LiDAR tilt calibration started: topic=%s, samples=%zu",
              imu_calibration_topic_.c_str(), imu_calibration_sample_count_);

  std::unique_lock<std::mutex> lock(data->mutex);
  const bool completed = data->cv.wait_for(
      lock, std::chrono::duration<double>(imu_calibration_timeout_sec_),
      [data, target_samples = imu_calibration_sample_count_]() {
        return data->statistics.count >= target_samples;
      });
  const auto summary = data->statistics.summarize();
  const std::size_t collected_samples = summary.count;
  lock.unlock();
  imu_subscription.reset();

  if (!completed || collected_samples < imu_calibration_sample_count_) {
    response->success = false;
    response->message = "IMU sampling timed out; check topic and IMU data";
    RCLCPP_WARN(get_logger(),
                "LiDAR tilt calibration timed out after %zu samples",
                collected_samples);
    return;
  }

  const Eigen::Vector3d mean_acc_lidar =
      imu_from_lidar_rotation_.transpose() * summary.mean_acceleration;
  if (!mean_acc_lidar.allFinite() || mean_acc_lidar.norm() <= 1e-3) {
    response->success = false;
    response->message = "Invalid mean IMU acceleration";
    RCLCPP_ERROR(get_logger(),
                 "LiDAR tilt calibration failed: invalid mean acceleration");
    return;
  }

  if (!summary.stationary(imu_calibration_max_gyro_rms_,
                          imu_calibration_max_accel_relative_std_)) {
    response->success = false;
    response->message = "IMU motion/noise too high; keep the leveled robot stationary";
    RCLCPP_WARN(get_logger(), "%s: accel_relative_std=%.6f, gyro_rms=%.6f rad/s",
                response->message.c_str(), summary.accel_relative_std, summary.gyro_rms);
    return;
  }

  Eigen::Matrix3d rotation = robot_geometry::lidar_extrinsics::levelRotation(
      calibration_base_from_lidar_, mean_acc_lidar);
  std::string ground_summary;
  Points validation_points_lidar;
  if (ground_calibration_.enabled &&
      !refineLidarTiltWithGround(rotation, ground_summary, validation_points_lidar)) {
    response->success = false;
    response->message = ground_summary;
    return;
  }
  response->roll_rad = std::atan2(rotation(2, 1), rotation(2, 2));
  response->pitch_rad =
      std::atan2(-rotation(2, 0), std::hypot(rotation(0, 0), rotation(1, 0)));
  if (calibration_rotation_type_ == "rpy_rad") {
    // Match the MID360 YAML output exactly: gravity fixes tilt, not Euler yaw.
    rotation = robot_geometry::lidar_extrinsics::decode(
        {0.0, 0.0, 0.0}, "rpy_rad",
        {response->roll_rad, response->pitch_rad, calibration_yaw_rad_})
                   .topLeftCorner<3, 3>();
  }
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      response->rotation_matrix[row * 3 + col] = rotation(row, col);
    }
  }
  if (ground_calibration_.enabled &&
      (calibration_corrected_pub_->get_subscription_count() > 0U ||
       calibration_corrected_pub_->get_intra_process_subscription_count() > 0U)) {
    Points corrected;
    corrected.reserve(validation_points_lidar.size());
    for (const auto &p : validation_points_lidar)
      corrected.push_back(rotation * p + calibration_translation_);
    calibration_corrected_pub_->publish(cloudMessage(corrected, now()));
  }
  response->success = true;
  response->message =
      "Tilt calibrated using the configured heading reference; yaw and translation "
      "are not calibrated. Copy the result to the robot YAML and restart workers. "
      "For matrix mounts use rotation_matrix; for rpy_rad use roll_rad/pitch_rad "
      "and retain the configured yaw. The robot base must have been level.";
  if (!ground_summary.empty()) response->message = ground_summary + ". " + response->message;
  RCLCPP_INFO(get_logger(),
              "LiDAR tilt calibration succeeded: samples=%zu, roll=%.6f, "
              "pitch=%.6f",
              collected_samples, response->roll_rad, response->pitch_rad);
  if (calibration_rotation_type_ == "rpy_rad") {
    RCLCPP_INFO(get_logger(),
                "MID360 lidar_to_base.rotation:\n"
                "  type: rpy_rad\n  value: [%.6f, %.6f, %.6f]",
                response->roll_rad, response->pitch_rad, calibration_yaw_rad_);
  } else {
    RCLCPP_INFO(get_logger(),
                "lidar_to_base.rotation (row-major, copy to robot YAML):\n"
                "  type: matrix\n"
                "  value: [%.6f, %.6f, %.6f,\n"
                "          %.6f, %.6f, %.6f,\n"
                "          %.6f, %.6f, %.6f]",
                rotation(0, 0), rotation(0, 1), rotation(0, 2),
                rotation(1, 0), rotation(1, 1), rotation(1, 2),
                rotation(2, 0), rotation(2, 1), rotation(2, 2));
  }
}


}  // namespace slam_manager
