#ifndef PGO_SC_POSE_PCD_HPP_
#define PGO_SC_POSE_PCD_HPP_

#include <rclcpp/rclcpp.hpp>

#include "utilities.hpp"

struct PosePcd {
  pcl::PointCloud<PointType> pcd_;  // base_link 局部坐标系点云
  Eigen::Matrix4d pose_eig_ =
      Eigen::Matrix4d::Identity();  // odom 坐标系下的位姿
  Eigen::Matrix4d pose_corrected_eig_ =
      Eigen::Matrix4d::Identity();  // map 坐标系下的位姿
  double timestamp_;
  int idx_;
  bool processed_ = false;
  PosePcd() = default;
  PosePcd(const nav_msgs::msg::Odometry& odom_in,
          const sensor_msgs::msg::PointCloud2& pcd_in, const int& idx_in);
};

inline PosePcd::PosePcd(const nav_msgs::msg::Odometry& odom_in,
                        const sensor_msgs::msg::PointCloud2& pcd_in,
                        const int& idx_in) {
  const Eigen::Quaterniond q(
      odom_in.pose.pose.orientation.w, odom_in.pose.pose.orientation.x,
      odom_in.pose.pose.orientation.y, odom_in.pose.pose.orientation.z);
  const Eigen::Matrix3d rot_mat_eig = q.normalized().toRotationMatrix();
  pose_eig_.block<3, 3>(0, 0) = rot_mat_eig;
  pose_eig_(0, 3) = odom_in.pose.pose.position.x;
  pose_eig_(1, 3) = odom_in.pose.pose.position.y;
  pose_eig_(2, 3) = odom_in.pose.pose.position.z;
  pose_corrected_eig_ = pose_eig_;
  pcl::PointCloud<PointType> tmp_pcd;
  pcl::fromROSMsg(pcd_in, tmp_pcd);
  // pcd_in: /cloud_registered，odom 坐标系点云
  // pose_eig_: odom -> base_link
  // pcd_: base_link 局部坐标系点云
  pcd_ = transformPcd(tmp_pcd, pose_eig_.inverse());
  timestamp_ = rclcpp::Time(odom_in.header.stamp).seconds();
  idx_ = idx_in;
}

#endif  // PGO_SC__POSE_PCD_HPP_
