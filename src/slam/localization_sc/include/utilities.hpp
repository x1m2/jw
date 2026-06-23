#ifndef SC_PGO_UTILITIES_HPP
#define SC_PGO_UTILITIES_HPP

///// common headers
#include <memory>
#include <string>
#include <vector>

///// ROS2
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

///// PCL
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

///// Eigen
#include <Eigen/Eigen>

using PointType = pcl::PointXYZI;

//////////////////////////////////////////////////////////////////////
///// voxelization
inline pcl::PointCloud<PointType>::Ptr voxelizePcd(
    const pcl::PointCloud<PointType> &pcd_in, const float voxel_res) {
  static pcl::VoxelGrid<PointType> voxelgrid;
  voxelgrid.setLeafSize(voxel_res, voxel_res, voxel_res);
  pcl::PointCloud<PointType>::Ptr pcd_in_ptr(
      new pcl::PointCloud<PointType>(pcd_in));
  pcl::PointCloud<PointType>::Ptr pcd_out(new pcl::PointCloud<PointType>);
  voxelgrid.setInputCloud(pcd_in_ptr);
  voxelgrid.filter(*pcd_out);
  return pcd_out;
}

inline pcl::PointCloud<PointType>::Ptr voxelizePcd(
    const pcl::PointCloud<PointType>::Ptr &pcd_in, const float voxel_res) {
  static pcl::VoxelGrid<PointType> voxelgrid;
  voxelgrid.setLeafSize(voxel_res, voxel_res, voxel_res);
  pcl::PointCloud<PointType>::Ptr pcd_out(new pcl::PointCloud<PointType>);
  voxelgrid.setInputCloud(pcd_in);
  voxelgrid.filter(*pcd_out);
  return pcd_out;
}

inline geometry_msgs::msg::PoseStamped poseEigToPoseStamped(
    const Eigen::Matrix4d &pose_eig_in, std::string frame_id = "map") {
  Eigen::Quaterniond q(pose_eig_in.block<3, 3>(0, 0));
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame_id;
  pose.pose.position.x = pose_eig_in(0, 3);
  pose.pose.position.y = pose_eig_in(1, 3);
  pose.pose.position.z = pose_eig_in(2, 3);
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.orientation.w = q.w();
  return pose;
}

inline geometry_msgs::msg::Transform poseEigToROSTf(
    const Eigen::Matrix4d &pose) {
  geometry_msgs::msg::Transform tf_msg;
  tf_msg.translation.x = pose(0, 3);
  tf_msg.translation.y = pose(1, 3);
  tf_msg.translation.z = pose(2, 3);

  Eigen::Quaterniond q(pose.block<3, 3>(0, 0));
  tf_msg.rotation.x = q.x();
  tf_msg.rotation.y = q.y();
  tf_msg.rotation.z = q.z();
  tf_msg.rotation.w = q.w();
  return tf_msg;
}

inline geometry_msgs::msg::TransformStamped poseEigToROSTfStamped(
    const Eigen::Matrix4d &pose, const std::string &frame_id = "map",
    const std::string &child_frame_id = "base_link") {
  geometry_msgs::msg::TransformStamped tf_stamped;
  tf_stamped.header.stamp = rclcpp::Clock().now();
  tf_stamped.header.frame_id = frame_id;
  tf_stamped.child_frame_id = child_frame_id;

  tf_stamped.transform = poseEigToROSTf(pose);
  return tf_stamped;
}

inline geometry_msgs::msg::Transform poseStampedToROSTf(
    const geometry_msgs::msg::PoseStamped &pose) {
  geometry_msgs::msg::Transform tf_msg;
  tf_msg.translation.x = pose.pose.position.x;
  tf_msg.translation.y = pose.pose.position.y;
  tf_msg.translation.z = pose.pose.position.z;
  tf_msg.rotation = pose.pose.orientation;
  return tf_msg;
}

//////////////////////////////////////////////////////////////////////
///// PCL -> ROS2 sensor_msgs
template <typename T>
inline sensor_msgs::msg::PointCloud2 pclToPclRos(
    const pcl::PointCloud<T> &cloud, const std::string &frame_id = "map") {
  sensor_msgs::msg::PointCloud2 cloud_ROS;
  pcl::toROSMsg(cloud, cloud_ROS);
  cloud_ROS.header.frame_id = frame_id;
  return cloud_ROS;
}

//////////////////////////////////////////////////////////////////////
///// transformation
template <typename T>
inline pcl::PointCloud<T> transformPcd(const pcl::PointCloud<T> &cloud_in,
                                       const Eigen::Matrix4d &pose_tf) {
  if (cloud_in.empty()) return cloud_in;

  pcl::PointCloud<T> pcl_out;
  pcl::transformPointCloud(cloud_in, pcl_out, pose_tf);
  return pcl_out;
}

#endif  // SC_PGO_UTILITIES_HPP