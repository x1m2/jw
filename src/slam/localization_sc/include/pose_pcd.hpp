#ifndef LOCALIZATION_SC_POSE_PCD_HPP
#define LOCALIZATION_SC_POSE_PCD_HPP

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "utilities.hpp"

struct PosePcd {
  pcl::PointCloud<PointType> pcd_;
  Eigen::Matrix4d pose_eig_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d pose_corrected_eig_ = Eigen::Matrix4d::Identity();
  int idx_;
  bool processed_ = false;

  PosePcd() {}

  PosePcd(const nav_msgs::msg::Odometry &odom_in,
          const sensor_msgs::msg::PointCloud2 &pcd_in, const int &idx_in);
};

struct PosePcdReduced {
  pcl::PointCloud<PointType> pcd_;
  Eigen::Matrix4d pose_eig_ = Eigen::Matrix4d::Identity();
  int idx_;

  PosePcdReduced() {}

  PosePcdReduced(const geometry_msgs::msg::PoseStamped &pose_in,
                 const sensor_msgs::msg::PointCloud2 &pcd_in,
                 const int &idx_in);
};

// -----------------------------
// PosePcd implementation
// -----------------------------
inline PosePcd::PosePcd(const nav_msgs::msg::Odometry &odom_in,
                        const sensor_msgs::msg::PointCloud2 &pcd_in,
                        const int &idx_in) {
  // ROS 2: tf2::Quaternion -> Eigen::Matrix3d
  tf2::Quaternion q(
      odom_in.pose.pose.orientation.x, odom_in.pose.pose.orientation.y,
      odom_in.pose.pose.orientation.z, odom_in.pose.pose.orientation.w);
  tf2::Matrix3x3 rot_mat_tf(q);

  Eigen::Matrix3d rot_mat_eig;
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) rot_mat_eig(i, j) = rot_mat_tf[i][j];

  pose_eig_.block<3, 3>(0, 0) = rot_mat_eig;
  pose_eig_(0, 3) = odom_in.pose.pose.position.x;
  pose_eig_(1, 3) = odom_in.pose.pose.position.y;
  pose_eig_(2, 3) = odom_in.pose.pose.position.z;
  pose_corrected_eig_ = pose_eig_;

  pcl::PointCloud<PointType> tmp_pcd;
  pcl::fromROSMsg(pcd_in, tmp_pcd);
  tmp_pcd.is_dense = false;
  //过滤无效点
  std::vector<int> valid_indices;
  pcl::removeNaNFromPointCloud(tmp_pcd, tmp_pcd, valid_indices);
  tmp_pcd.is_dense = true;
  pcd_ = transformPcd(tmp_pcd, pose_eig_.inverse());

  idx_ = idx_in;
}

// -----------------------------
// PosePcdReduced implementation
// -----------------------------
inline PosePcdReduced::PosePcdReduced(
    const geometry_msgs::msg::PoseStamped &pose_in,
    const sensor_msgs::msg::PointCloud2 &pcd_in, const int &idx_in) {
  tf2::Quaternion q(pose_in.pose.orientation.x, pose_in.pose.orientation.y,
                    pose_in.pose.orientation.z, pose_in.pose.orientation.w);
  tf2::Matrix3x3 rot_mat_tf(q);

  Eigen::Matrix3d rot_mat_eig;
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) rot_mat_eig(i, j) = rot_mat_tf[i][j];

  pose_eig_.block<3, 3>(0, 0) = rot_mat_eig;
  pose_eig_(0, 3) = pose_in.pose.position.x;
  pose_eig_(1, 3) = pose_in.pose.position.y;
  pose_eig_(2, 3) = pose_in.pose.position.z;

  pcl::fromROSMsg(pcd_in, pcd_);
  pcd_.is_dense = false;
  //过滤无效点
  std::vector<int> valid_indices;
  pcl::removeNaNFromPointCloud(pcd_, pcd_, valid_indices);
  pcd_.is_dense = true;
  idx_ = idx_in;
}

#endif