#include "ros2_main.hpp"

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <csignal>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>

#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "system/Header.h"
#include "system/ParametersRos2.h"
#include "system/System.hpp"

#define MEASURES_BUFFER
#define WORK
#ifdef WORK
// #include "localization_msg/vehicle_pose.h"
// #include "robot_msgs/Level.h"
// #include "robot_msgs/ModuleStatus.h"
#endif

FILE *location_log = nullptr;

void FastlioLocalizationNode::publish_cloud(
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubCloud,
    PointCloudType::Ptr cloud, const double &lidar_end_time,
    const std::string &frame_id) {
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud, cloud_msg);
  cloud_msg.header.stamp = rclcpp::Time(lidar_end_time * 1e9);
  cloud_msg.header.frame_id = frame_id;
  pubCloud->publish(cloud_msg);
}

void FastlioLocalizationNode::publish_cloud_world(
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        &pubLaserCloudFull,
    PointCloudType::Ptr laserCloud, const state_ikfom &state,
    const double &lidar_end_time) {
  PointCloudType::Ptr laserCloudWorld(
      new PointCloudType(laserCloud->size(), 1));
  pointcloudLidarToWorld(laserCloud, laserCloudWorld, state);
  publish_cloud(pubLaserCloudFull, laserCloudWorld, lidar_end_time, map_frame);
}

void FastlioLocalizationNode::publish_cloud_body(
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &publisher,
    PointCloudType::Ptr cloud, const double &lidar_end_time) {
  PointCloudType::Ptr cloud_body(new PointCloudType());
  pcl::transformPointCloud(*cloud, *cloud_body, T_base_lidar);
  publish_cloud(publisher, cloud_body, lidar_end_time, baselink_frame);
}

template <typename T>
void FastlioLocalizationNode::set_posestamp(T &out, const QD &rot,
                                            const V3D &pos) {
  out.pose.position.x = pos.x();
  out.pose.position.y = pos.y();
  out.pose.position.z = pos.z();
  out.pose.orientation.x = rot.x();
  out.pose.orientation.y = rot.y();
  out.pose.orientation.z = rot.z();
  out.pose.orientation.w = rot.w();
}

void FastlioLocalizationNode::publish_tf(const QD &rot, const V3D &pos,
                                         const double &lidar_end_time) {
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = rclcpp::Time(lidar_end_time * 1e9);
  transform_stamped.header.frame_id = map_frame;
  transform_stamped.child_frame_id = baselink_frame;
  transform_stamped.transform.translation.x = pos.x();
  transform_stamped.transform.translation.y = pos.y();
  transform_stamped.transform.translation.z = pos.z();
  transform_stamped.transform.rotation.x = rot.x();
  transform_stamped.transform.rotation.y = rot.y();
  transform_stamped.transform.rotation.z = rot.z();
  transform_stamped.transform.rotation.w = rot.w();
  tf_broadcaster->sendTransform(transform_stamped);
}

void FastlioLocalizationNode::compute_baselink_pose(const state_ikfom &state,
                                                    QD &baselink_rot,
                                                    V3D &baselink_pos) const {
  // FAST-LIO state: IMU pose in map, ^M T_I.
  Eigen::Matrix4d T_map_imu = Eigen::Matrix4d::Identity();
  T_map_imu.topLeftCorner(3, 3) = state.rot.toRotationMatrix();
  T_map_imu.topRightCorner(3, 1) = state.pos;

  // FAST-LIO extrinsic: LiDAR pose in IMU, ^I T_L.
  Eigen::Matrix4d T_imu_lidar = Eigen::Matrix4d::Identity();
  T_imu_lidar.topLeftCorner(3, 3) = state.offset_R_L_I.toRotationMatrix();
  T_imu_lidar.topRightCorner(3, 1) = state.offset_T_L_I;

  // ^M T_B = ^M T_I * ^I T_L * inverse(^B T_L).
  const Eigen::Matrix4d T_map_base =
      T_map_imu * T_imu_lidar * T_base_lidar.inverse();
  baselink_rot = QD(M3D(T_map_base.topLeftCorner(3, 3)));
  baselink_pos = T_map_base.topRightCorner(3, 1);
}

void FastlioLocalizationNode::publish_geometry_msgs(
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
        &pub_robot_pose,
    const state_ikfom &state, const double &lidar_end_time, QD &baselink_rot,
    V3D &baselink_pos) {
  compute_baselink_pose(state, baselink_rot, baselink_pos);

  if (std::abs(baselink_pos.x()) > 1e7 || std::abs(baselink_pos.y()) > 1e7 ||
      std::abs(baselink_pos.z()) > 1e7) {
    LOG_WARN(
        "localization state maybe abnormal! (baselink frame) pos(%f, %f, %f)",
        baselink_pos.x(), baselink_pos.y(), baselink_pos.z());
  }

  geometry_msgs::msg::PoseStamped robotPose;
  robotPose.header.frame_id = map_frame;
  robotPose.header.stamp = rclcpp::Time(lidar_end_time * 1e9);
  set_posestamp(robotPose, baselink_rot, baselink_pos);
  const auto rpy = EigenMath::Quaternion2RPY(baselink_rot);
  LOG_INFO(
      "/pose_stamped_localization (xyz, rpy): (%.5f, %.5f, %.5f, %.5f, "
      "%.5f, %.5f)",
      baselink_pos.x(), baselink_pos.y(), baselink_pos.z(), RAD2DEG(rpy.x()),
      RAD2DEG(rpy.y()), RAD2DEG(rpy.z()));
  pub_robot_pose->publish(robotPose);

  slam_manager_interfaces::msg::LocalizationPose app_pose;
  app_pose.pose = robotPose;
  app_pose.match_score = static_cast<float>(slam.frontend->get_match_score());
  pubAppLocalizationPose->publish(app_pose);

  if (lidar_tf_broadcast)
    publish_tf(baselink_rot, baselink_pos, lidar_end_time);
}

void FastlioLocalizationNode::publish_odometry(
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr &pubLidarOdom,
    const state_ikfom &state, const double &lidar_end_time, QD &baselink_rot,
    V3D &baselink_pos) {
  compute_baselink_pose(state, baselink_rot, baselink_pos);

  if (std::abs(baselink_pos.x()) > 1e7 || std::abs(baselink_pos.y()) > 1e7 ||
      std::abs(baselink_pos.z()) > 1e7) {
    LOG_WARN(
        "localization state maybe abnormal! (baselink frame) pos(%f, %f, %f)",
        baselink_pos.x(), baselink_pos.y(), baselink_pos.z());
  }

  nav_msgs::msg::Odometry odomAftMapped;
  odomAftMapped.header.frame_id = map_frame;
  odomAftMapped.child_frame_id = baselink_frame;
  odomAftMapped.header.stamp = rclcpp::Time(lidar_end_time * 1e9);
  set_posestamp(odomAftMapped.pose, baselink_rot, baselink_pos);
  pubLidarOdom->publish(odomAftMapped);
  if (lidar_tf_broadcast)
    publish_tf(baselink_rot, baselink_pos, lidar_end_time);
}

void FastlioLocalizationNode::publish_module_status(bool level) {
  // robot_msgs::ModuleStatus status;
  // status.header.stamp = ros::Time().fromSec(time);
  // status.header.frame_id = "LOCATION";
  // status.level = level;
  // robot_msgs::ModuleStatusItem item;
  // item.error_code = 4000101;
  // item.level = level;
  // status.items.emplace_back(item);
  // pubModulesStatus.publish(status);
  std_msgs::msg::Bool msg;
  msg.data = level;
  reloc_status_pub->publish(msg);
}

void FastlioLocalizationNode::publish_imu_odometry(
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr &pubImuOdom,
    const state_ikfom &state, const double &imu_time) {
  QD baselink_rot;
  V3D baselink_pos;
  compute_baselink_pose(state, baselink_rot, baselink_pos);

  if (std::abs(baselink_pos.x()) > 1e7 || std::abs(baselink_pos.y()) > 1e7 ||
      std::abs(baselink_pos.z()) > 1e7) {
    LOG_WARN(
        "localization state maybe abnormal! (baselink frame) pos(%f, %f, %f)",
        baselink_pos.x(), baselink_pos.y(), baselink_pos.z());
  }

  nav_msgs::msg::Odometry imuOdom;
  imuOdom.header.frame_id = map_frame;
  imuOdom.child_frame_id = baselink_frame;
  imuOdom.header.stamp = rclcpp::Time(imu_time * 1e9);
  set_posestamp(imuOdom.pose, baselink_rot, baselink_pos);
  pubImuOdom->publish(imuOdom);
  if (imu_tf_broadcast) publish_tf(baselink_rot, baselink_pos, imu_time);
}

void FastlioLocalizationNode::publish_imu_path(
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &pubPath, const QD &rot,
    const V3D &pos, const double &lidar_end_time) {
  static geometry_msgs::msg::PoseStamped msg_body_pose;
  set_posestamp(msg_body_pose, rot, pos);
  msg_body_pose.header.stamp = rclcpp::Time(lidar_end_time * 1e9);
  msg_body_pose.header.frame_id = map_frame;

  static nav_msgs::msg::Path path;
  path.header.stamp = rclcpp::Clock().now();
  path.header.frame_id = map_frame;

  path.poses.push_back(msg_body_pose);
  pubPath->publish(path);
  /*** if path is too large, the rvis will crash ***/
  if (path.poses.size() >= 300) {
    path.poses.erase(path.poses.begin());
  }
}

#ifdef gnss_with_direction
void FastlioLocalizationNode::gnss_cbk(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  slam.relocalization->gnss_pose =
      GnssPose(msg->header.stamp.sec + msg->header.stamp.nanosec * 1.0e-9,
               V3D(msg->pose.pose.position.x, msg->pose.pose.position.y,
                   msg->pose.pose.position.z),
               QD(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                  msg->pose.pose.orientation.y, msg->pose.pose.orientation.z));
}
#else
void FastlioLocalizationNode::gnss_cbk(
    const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
  slam.relocalization->gnss_pose =
      GnssPose(msg->header.stamp.sec + msg->header.stamp.nanosec * 1.0e-9,
               V3D(msg->latitude, msg->longitude, msg->altitude));
}
#endif

bool FastlioLocalizationNode::load_last_pose(const PointCloudType::Ptr &scan) {
  static bool use_last_pose = true;
  if (!use_last_pose) return false;
  use_last_pose = false;
  LOG_WARN("relocate try to use last pose!");

  fseek(last_pose_record, 0, SEEK_SET);
  Eigen::Quaterniond quat;
  Eigen::Vector3d pos;
  auto res = fscanf(last_pose_record,
                    "last_pose(xyz,rpy):( %lf %lf %lf %lf %lf %lf %lf )end!\n",
                    &pos.x(), &pos.y(), &pos.z(), &quat.w(), &quat.x(),
                    &quat.y(), &quat.z());
  Eigen::Matrix4d imu_pose = Eigen::Matrix4d::Identity();
  imu_pose.topLeftCorner(3, 3) = quat.normalized().toRotationMatrix();
  imu_pose.topRightCorner(3, 1) = pos;
  slam.frontend->reset_state(imu_pose);

  if (res == -1) {
    LOG_ERROR("can't find last pose! parse error!");
    return false;
  }
  double bnb_score = 0, ndt_score = 0;
  Timer timer;
  slam.relocalization->get_pose_score(imu_pose, scan, bnb_score, ndt_score);
  if (bnb_score < 0.4) {
    LOG_ERROR("bnb_score too small = %.3f, should be greater than = %.3f!",
              bnb_score, 0.8);
    return false;
  } else if (ndt_score > 0.2) {
    LOG_ERROR("ndt_score too high = %.3f, should be less than = %.3f!",
              ndt_score, 0.1);
    return false;
  } else {
    LOG_WARN(
        "relocate use last pose successfully! bnb_score = %.3f, ndt_score = "
        "%.3f. cost time = %.2lf ms.",
        bnb_score, ndt_score, timer.elapsedLast());
    slam.system_state_vaild = true;
  }

  return slam.system_state_vaild;
}

void FastlioLocalizationNode::sensor_data_process() {
  if (stopping.load()) return;

  if (!slam.frontend->sync_sensor_data()) return;

  LOG_DEBUG("sync_sensor_data");
#ifdef MEASURES_BUFFER
  static std::deque<shared_ptr<MeasureCollection>> measures_cache;
#endif
  if (!slam.system_state_vaild) {
    publish_module_status(false);
    if (last_pose_record && load_last_pose(slam.frontend->measures->lidar)) {
      // only for restart!
    } else if (!slam.run_relocalization_thread) {
      if (slam.relocalization_thread.joinable()) {
        slam.relocalization_thread.join();
        measures_cache.clear();
        LOG_WARN("measures cache clear!");
      }

      PointCloudType::Ptr cur_scan(new PointCloudType);
      *cur_scan = *slam.frontend->measures->lidar;
      const V3D mean_acc =
          System::mean_acceleration(*slam.frontend->measures);
      const double lidar_beg_time = slam.frontend->measures->lidar_beg_time;
      slam.relocalization_thread = std::thread(
          [this, cur_scan, lidar_beg_time, mean_acc]() {
            if (slam.run_relocalization(cur_scan, lidar_beg_time, mean_acc)) {
              // Report completion even when offline playback is paused.
              publish_module_status(true);
            }
          });
    }
    // publish_module_status(slam.frontend->measures->lidar_end_time,
    // robot_msgs::Level::WARN);
#ifdef MEASURES_BUFFER
    measures_cache.emplace_back(std::make_shared<MeasureCollection>());
    *measures_cache.back() = *slam.frontend->measures;
#endif
    return;
  }

#ifdef MEASURES_BUFFER
  if (!measures_cache.empty()) {
    measures_cache.emplace_back(std::make_shared<MeasureCollection>());
    *measures_cache.back() = *slam.frontend->measures;

    while (!measures_cache.empty()) {
      slam.frontend->measures = measures_cache.front();

      QD baselink_rot;
      V3D baselink_pos;
      if (slam.run()) {
        const auto &state = slam.frontend->get_state();
        LOG_INFO(
            "location valid. feats_down = %d, match_score = %.3f, "
            "cost time = %.1fms.",
            slam.frontend->loger.feats_down_size,
            slam.frontend->get_match_score(), slam.frontend->loger.total_time);
        // slam.frontend->loger.print_pose(state, "cur_imu_pose");

        /******* Publish odometry *******/
#ifndef WORK
        publish_odometry(pubLidarOdom, state,
                         slam.frontend->measures->lidar_end_time, baselink_rot,
                         baselink_pos);
#else
        // publish_odometry2(pubOdomDev, state,
        // slam.frontend->measures->lidar_end_time, slam.system_state_vaild,
        // baselink_rot, baselink_pos);
        // publish_module_status(slam.frontend->measures->lidar_end_time,
        // robot_msgs::Level::OK);
        publish_geometry_msgs(pub_robot_pose, state,
                              slam.frontend->measures->lidar_end_time,
                              baselink_rot, baselink_pos);
        publish_module_status(true);
#endif

        if (path_en)
          publish_imu_path(pubImuPath, baselink_rot, baselink_pos,
                           slam.frontend->measures->lidar_end_time);

        /******* Publish points *******/
        if (scan_pub_en) {
          publish_cloud_body(pubLaserCloudBody, slam.feats_undistort,
                             slam.frontend->measures->lidar_end_time);
          if (dense_pub_en)
            publish_cloud_world(pubLaserCloudFull, slam.feats_undistort, state,
                                slam.frontend->measures->lidar_end_time);
          else
            publish_cloud(pubLaserCloudFull, slam.frontend->feats_down_world,
                          slam.frontend->measures->lidar_end_time, map_frame);
        }
      } else {
        LOG_ERROR("location invalid!");
#ifdef WORK
        // publish_odometry2(pubOdomDev, slam.frontend->get_state(),
        // slam.frontend->measures->lidar_end_time, slam.system_state_vaild,
        // baselink_rot, baselink_pos);
        // publish_module_status(slam.frontend->measures->lidar_end_time,
        // robot_msgs::Level::WARN);
        publish_module_status(false);
#endif
#ifdef DEDUB_MODE
        publish_cloud_world(pubrelocalizationDebug,
                            slam.frontend->measures->lidar,
                            slam.frontend->get_state(),
                            slam.frontend->measures->lidar_end_time);
#endif
      }

      measures_cache.pop_front();
    }
    LOG_WARN("measures cache have all been processed!");
    return;
  }
#endif

  LOG_DEBUG("run fastlio");
  QD baselink_rot;
  V3D baselink_pos;
  if (slam.run()) {
    const auto &state = slam.frontend->get_state();
    LOG_INFO(
        "location valid. feats_down = %d, match_score = %.3f, "
        "cost time = %.1fms.",
        slam.frontend->loger.feats_down_size, slam.frontend->get_match_score(),
        slam.frontend->loger.total_time);
    // slam.frontend->loger.print_pose(state, "cur_imu_pose");

    if (last_pose_record) {
      // record last pose
      fseek(last_pose_record, 0, SEEK_SET);
      fprintf(last_pose_record,
              "last_pose(xyz,rpy):( %.5lf %.5lf %.5lf %.5lf %.5lf %.5lf %.5lf "
              ")end!\n",
              state.pos.x(), state.pos.y(), state.pos.z(), state.rot.w(),
              state.rot.x(), state.rot.y(), state.rot.z());
    }

    /******* Publish odometry *******/
#ifndef WORK
    publish_odometry(pubLidarOdom, state, slam.frontend->lidar_end_time,
                     baselink_rot, baselink_pos);
#else
    // publish_odometry2(pubOdomDev, state,
    // slam.frontend->measures->lidar_end_time, slam.system_state_vaild,
    // baselink_rot, baselink_pos);
    // publish_module_status(slam.frontend->measures->lidar_end_time,
    // robot_msgs::Level::OK);
    publish_geometry_msgs(pub_robot_pose, state,
                          slam.frontend->measures->lidar_end_time, baselink_rot,
                          baselink_pos);
    publish_module_status(true);
#endif

    if (path_en)
      publish_imu_path(pubImuPath, baselink_rot, baselink_pos,
                       slam.frontend->lidar_end_time);

    /******* Publish points *******/
    if (scan_pub_en) {
      publish_cloud_body(pubLaserCloudBody, slam.feats_undistort,
                         slam.frontend->lidar_end_time);
      if (dense_pub_en)
        publish_cloud_world(pubLaserCloudFull, slam.feats_undistort, state,
                            slam.frontend->lidar_end_time);
      else
        publish_cloud(pubLaserCloudFull, slam.frontend->feats_down_world,
                      slam.frontend->lidar_end_time, map_frame);
    }
  } else {
    LOG_ERROR("location invalid!");
#ifdef WORK
    // publish_odometry2(pubOdomDev, slam.frontend->get_state(),
    // slam.frontend->measures->lidar_end_time, slam.system_state_vaild,
    // baselink_rot, baselink_pos);
    // publish_module_status(slam.frontend->measures->lidar_end_time,
    // robot_msgs::Level::WARN);
    publish_module_status(false);
#endif
#ifdef DEDUB_MODE
    publish_cloud_world(pubrelocalizationDebug, slam.frontend->measures->lidar,
                        slam.frontend->get_state(),
                        slam.frontend->lidar_end_time);
#endif
  }
}

void FastlioLocalizationNode::standard_pcl_cbk(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  static double last_time = 0;
  check_time_interval(
      last_time, msg->header.stamp.sec + msg->header.stamp.nanosec * 1.0e-9,
      1.0 / slam.frontend->lidar->scan_rate, "lidar_time_interval");
  LOG_DEBUG("lidar_handler");

  Timer timer;
  pcl::PointCloud<ouster_ros::Point> pl_orig_oust;
  pcl::PointCloud<velodyne_ros::Point> pl_orig_velo;
  PointCloudType::Ptr scan(new PointCloudType());

  switch (lidar_type) {
    case OUST64:
      pcl::fromROSMsg(*msg, pl_orig_oust);
      slam.frontend->lidar->oust64_handler(pl_orig_oust, scan);
      break;

    case VELO16:
    case RSAIRY:
      if (lidar_type == RSAIRY) {
        const auto has_field = [msg](const char *name, uint8_t datatype) {
          return std::any_of(msg->fields.begin(), msg->fields.end(),
                             [name, datatype](const auto &field) {
                               return field.name == name &&
                                      field.datatype == datatype;
                             });
        };
        using PointField = sensor_msgs::msg::PointField;
        if (!has_field("x", PointField::FLOAT32) ||
            !has_field("y", PointField::FLOAT32) ||
            !has_field("z", PointField::FLOAT32) ||
            !has_field("intensity", PointField::FLOAT32) ||
            !has_field("ring", PointField::UINT16) ||
            !has_field("timestamp", PointField::FLOAT64)) {
          RCLCPP_ERROR_ONCE(
              get_logger(),
              "Airy requires x/y/z/intensity(float32), ring(uint16), "
              "timestamp(float64) PointCloud2 fields");
          return;
        }
      }
      pcl::fromROSMsg(*msg, pl_orig_velo);
      if (lidar_type == RSAIRY &&
          (pl_orig_velo.empty() ||
           !std::isfinite(pl_orig_velo.points.front().timestamp) ||
           !std::isfinite(pl_orig_velo.points.back().timestamp) ||
           pl_orig_velo.points.back().timestamp <= 0.0)) {
        RCLCPP_ERROR_ONCE(get_logger(), "Airy point timestamps are invalid");
        return;
      }
      slam.frontend->lidar->velodyne_handler(pl_orig_velo, scan);
      break;

    default:
      printf("Error LiDAR Type");
      break;
  }

  LOG_DEBUG("lidar_cache");
  slam.frontend->cache_pointcloud_data(
      msg->header.stamp.sec + msg->header.stamp.nanosec * 1.0e-9, scan);
  LOG_DEBUG("lidar_process");
  sensor_data_process();
}

void FastlioLocalizationNode::livox_pcl_cbk(
    const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
  static double last_time = 0;
  check_time_interval(
      last_time, msg->header.stamp.sec + msg->header.stamp.nanosec * 1.0e-9,
      1.0 / slam.frontend->lidar->scan_rate, "lidar_time_interval");

  Timer timer;
  auto plsize = msg->point_num;
  PointCloudType::Ptr scan(new PointCloudType());
  PointCloudType::Ptr pl_orig(new PointCloudType());
  pl_orig->reserve(plsize);
  PointType point;
  for (uint i = 1; i < plsize; i++) {
    if (((msg->points[i].tag & 0x30) == 0x10 ||
         (msg->points[i].tag & 0x30) == 0x00)) {
      point.x = msg->points[i].x;
      point.y = msg->points[i].y;
      point.z = msg->points[i].z;
      point.intensity = msg->points[i].reflectivity;
      point.curvature = msg->points[i].offset_time /
                        float(1000000);  // use curvature as time of each laser
                                         // points, curvature unit: ms

      pl_orig->points.push_back(point);
    }
  }
  slam.frontend->lidar->avia_handler(pl_orig, scan);
  slam.frontend->cache_pointcloud_data(
      msg->header.stamp.sec + msg->header.stamp.nanosec * 1.0e-9, scan);
  sensor_data_process();
}

void FastlioLocalizationNode::imu_cbk(
    const sensor_msgs::msg::Imu::SharedPtr msg) {
  const auto &angular_velocity =
      V3D(msg->angular_velocity.x, msg->angular_velocity.y,
          msg->angular_velocity.z);
  const auto &linear_acceleration =
      V3D(msg->linear_acceleration.x, msg->linear_acceleration.y,
          msg->linear_acceleration.z);
  if (!check_imu_meas(angular_velocity, linear_acceleration,
                      slam.frontend->imu->imu_meas_check))
    return;

  static double last_time = 0;
  check_time_interval(
      last_time, msg->header.stamp.sec + msg->header.stamp.nanosec * 1.0e-9,
      1.0 / slam.frontend->imu->imu_rate, "imu_time_interval");

  LOG_DEBUG("imu_cache");
  slam.frontend->cache_imu_data(
      msg->header.stamp.sec + msg->header.stamp.nanosec * 1.0e-9,
      angular_velocity, linear_acceleration);
  LOG_DEBUG("imu_process");
  sensor_data_process();
  if (slam.system_state_vaild) {
    auto imu_state =
        slam.frontend->integrate_imu_odom(*slam.frontend->imu_buffer.back());
    publish_imu_odometry(
        pubImuOdom, imu_state,
        msg->header.stamp.sec + msg->header.stamp.nanosec * 1.0e-9);
  }
}

void FastlioLocalizationNode::publish_global_map() {
  if (pubGlobalMap->get_subscription_count() != 0) {
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*slam.global_map, cloud_msg);
    cloud_msg.header.stamp = rclcpp::Clock().now();
    cloud_msg.header.frame_id = map_frame;
    pubGlobalMap->publish(cloud_msg);
  }

  if (pubOccGrid->get_subscription_count() != 0 && slam.p2p &&
      !slam.p2p->map_data.empty()) {
    nav_msgs::msg::OccupancyGrid msg;
    msg.header.stamp = rclcpp::Clock().now();
    msg.header.frame_id = map_frame;

    msg.info.map_load_time = rclcpp::Clock().now();
    msg.info.resolution = slam.p2p->resolution_;

    msg.info.origin.position.x = slam.p2p->map_info_origin_position_x;
    msg.info.origin.position.y = slam.p2p->map_info_origin_position_y;
    msg.info.origin.orientation.w = 1.0;

    msg.info.width = slam.p2p->map_info_width;
    msg.info.height = slam.p2p->map_info_height;
    msg.data = slam.p2p->map_data;
    pubOccGrid->publish(msg);
  }
}

void FastlioLocalizationNode::initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  const geometry_msgs::msg::Pose &pose = msg->pose.pose;
  const auto &ori = msg->pose.pose.orientation;
  const Eigen::Quaterniond quat(ori.w, ori.x, ori.y, ori.z);
  if (!quat.coeffs().allFinite() || quat.norm() <= 1e-6) {
    LOG_ERROR("ignore invalid /initialpose orientation");
    return;
  }

  // T_map_imu = T_map_base * T_base_lidar * inverse(T_imu_lidar).
  Eigen::Matrix4d imu_pose_matrix = Eigen::Matrix4d::Identity();
  imu_pose_matrix.topLeftCorner<3, 3>() =
      quat.normalized().toRotationMatrix();
  imu_pose_matrix.topRightCorner<3, 1>() =
      Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  imu_pose_matrix *= T_base_lidar;
  imu_pose_matrix *=
      slam.relocalization->lidar_extrinsic.toMatrix4d().inverse();

  Pose init_pose;
  EigenMath::DecomposeAffineMatrix(imu_pose_matrix, init_pose.x, init_pose.y,
                                   init_pose.z, init_pose.roll, init_pose.pitch,
                                   init_pose.yaw);
  slam.relocalization->set_init_pose(init_pose);
}

FastlioLocalizationNode::FastlioLocalizationNode(
    const rclcpp::NodeOptions &options)
    : rclcpp::Node("fastlio_localization", options),
      slam_owner(std::make_unique<System>()),
      slam(*slam_owner) {}

void FastlioLocalizationNode::initialize() {
  auto node = std::static_pointer_cast<rclcpp::Node>(shared_from_this());
  tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*node);

  string lidar_topic, imu_topic, gnss_topic;
  bool relocate_use_last_pose = true;
  bool location_log_enable = true;
  std::string last_pose_record_path;
  std::string location_log_save_path;

  node->declare_parameter<std::vector<double>>("lidar_to_base_T",
                                               std::vector<double>(3, 0.0));
  node->declare_parameter<std::vector<double>>("lidar_to_base_R",
                                               std::vector<double>(3, 0.0));
  node->get_parameter("lidar_to_base_T", lidar_to_base_T);
  node->get_parameter("lidar_to_base_R", lidar_to_base_R);

  // Fixed LiDAR pose in base_link, ^B T_L, from robot_param.yaml.
  T_base_lidar.setIdentity();
  T_base_lidar.topLeftCorner<3, 3>() =
      (Eigen::AngleAxisd(lidar_to_base_R[2], Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(lidar_to_base_R[1], Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(lidar_to_base_R[0], Eigen::Vector3d::UnitX()))
          .toRotationMatrix();
  T_base_lidar.topRightCorner(3, 1) = Eigen::Vector3d(
      lidar_to_base_T[0], lidar_to_base_T[1], lidar_to_base_T[2]);

  load_log_parameters(node, relocate_use_last_pose, last_pose_record_path,
                      location_log_enable, location_log_save_path);

  // 1.record last pose
  if (relocate_use_last_pose) {
    if (last_pose_record_path.compare("") != 0) {
      FileOperation::createFileWhenNotExist(last_pose_record_path);
      last_pose_record = fopen(last_pose_record_path.c_str(), "r+");
    } else {
      FileOperation::createFileWhenNotExist(
          DEBUG_FILE_DIR("last_pose_record.txt"));
      last_pose_record =
          fopen(DEBUG_FILE_DIR("last_pose_record.txt").c_str(), "r+");
    }
  }
  // 2.record log
  if (location_log_enable) {
    if (location_log_save_path.compare("") != 0)
      location_log = fopen(location_log_save_path.c_str(), "a");
    else
      location_log = fopen(DEBUG_FILE_DIR("location.log").c_str(), "a");
    if (location_log)
      LOG_WARN("open file %s successfully!", location_log_save_path.c_str());
    else
      LOG_ERROR("open file %s failed!", location_log_save_path.c_str());
  }
  load_ros_parameters(node, path_en, scan_pub_en, dense_pub_en,
                      lidar_tf_broadcast, imu_tf_broadcast, use_gnss,
                      lidar_topic, imu_topic, gnss_topic, map_frame,
                      lidar_frame, baselink_frame);
  slam.relocalization->use_gnss = use_gnss;
  load_parameters(node, slam, lidar_type);

  /*** ROS subscribe initialization ***/
  if (lidar_type == AVIA)
    sub_pcl1 = node->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        lidar_topic, 10000,
        [this](livox_ros_driver2::msg::CustomMsg::SharedPtr message) {
          livox_pcl_cbk(std::move(message));
        });
  else
    sub_pcl2 = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_topic, 10000,
        [this](sensor_msgs::msg::PointCloud2::SharedPtr message) {
          standard_pcl_cbk(std::move(message));
        });
  sub_imu = node->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, 200000, [this](sensor_msgs::msg::Imu::SharedPtr message) {
        imu_cbk(std::move(message));
      });
  if (use_gnss) {
    sub_gnss = node->create_subscription<sensor_msgs::msg::NavSatFix>(
        gnss_topic, 1000,
        [this](sensor_msgs::msg::NavSatFix::SharedPtr message) {
          gnss_cbk(std::move(message));
        });
  }
  pubLaserCloudFull = node->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/cloud_registered", 1000);
  pubLaserCloudBody = node->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/cloud_registered_body", 20);
  pubLidarOdom = node->create_publisher<nav_msgs::msg::Odometry>(
      "/lidar_localization", 1000);
  pubImuOdom = node->create_publisher<nav_msgs::msg::Odometry>(
      "/imu_localization", 1000);
  pubImuPath = node->create_publisher<nav_msgs::msg::Path>("/imu_path", 1000);
  pubGlobalMap =
      node->create_publisher<sensor_msgs::msg::PointCloud2>("/global_map", 1);
  pubOccGrid =
      node->create_publisher<nav_msgs::msg::OccupancyGrid>("/grid_map", 1);

  timer = node->create_wall_timer(2000ms, [this]() { publish_global_map(); });
  sub_initpose =
      node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/initialpose", 1,
          [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr
                     message) { initialPoseCallback(std::move(message)); });
  pubrelocalizationDebug =
      node->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/relocalization_debug", 1);

#ifdef WORK
  // pubOdomDev =
  // node->create_publisher<localization_msg::msg::vehicle_pose>("/robot/pose_dynamic_data",
  // 1); pubModulesStatus =
  // node->create_publisher<robot_msgs::msg::ModuleStatus>("/robot/module_status",
  // 1);
  reloc_status_pub =
      node->create_publisher<std_msgs::msg::Bool>("/reloc_success", 1);
  pub_robot_pose = node->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/pose_stamped_localization", 1000);
  pubAppLocalizationPose =
      node->create_publisher<slam_manager_interfaces::msg::LocalizationPose>(
          "/app/localization_pose", 20);
#endif
}

FastlioLocalizationNode::~FastlioLocalizationNode() {
  requestStop();
  if (slam.relocalization_thread.joinable()) slam.relocalization_thread.join();
  if (last_pose_record) {
    fclose(last_pose_record);
    last_pose_record = nullptr;
  }
  if (location_log) {
    fclose(location_log);
    location_log = nullptr;
  }
}

void FastlioLocalizationNode::requestStop() {
  if (stopping.exchange(true)) return;
  if (timer) timer->cancel();
}
