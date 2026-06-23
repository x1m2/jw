#include "dual_radar_traversable_extractor/dual_radar_traversable_extractor.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

DualRadarTraversableExtractor::DualRadarTraversableExtractor(
    const rclcpp::Node::SharedPtr& node)
    : node_(node),
      lidar_transform_ready_(false),
      radar_transform_ready_(false),
      tf_buffer_(nullptr),
      tf_listener_(nullptr) {
  loadParameters();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  lidar_to_base_matrix_ = Eigen::Matrix4d::Identity();
  radar_to_base_matrix_ = Eigen::Matrix4d::Identity();

  lidar_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_topic_, rclcpp::QoS(10),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        this->lidarCallback(msg);
      });

  if (use_dual_radar_) {
    radar_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        radar_topic_, rclcpp::QoS(10),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
          this->radarCallback(msg);
        });

    RCLCPP_INFO(node_->get_logger(), "双雷达模式启用，订阅毫米波雷达: %s",
                radar_topic_.c_str());
  } else {
    RCLCPP_INFO(node_->get_logger(), "单雷达模式，仅使用激光雷达数据");
  }

  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(10),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        this->odomCallback(msg);
      });

  ground_cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "ground_cloud", rclcpp::QoS(10));

  obstacle_cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "obstacle_cloud", rclcpp::QoS(10));

  combined_cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "combined_cloud", rclcpp::QoS(10));

  voxel_filter_.setLeafSize(static_cast<float>(voxel_leaf_size_),
                            static_cast<float>(voxel_leaf_size_),
                            static_cast<float>(voxel_leaf_size_));

  if (use_statistical_filter_) {
    stat_filter_.setMeanK(statistical_filter_k_);
    stat_filter_.setStddevMulThresh(statistical_filter_std_);
  }

  if (use_patchworkpp_) {
    patchworkpp_ = std::make_shared<PatchWorkpp<PointType>>(node_);
    RCLCPP_INFO(node_->get_logger(), "PatchWorkpp 地面分割器初始化完成");
  }

  RCLCPP_INFO(node_->get_logger(), "DualRadarTraversableExtractor 初始化完成");
  RCLCPP_INFO(node_->get_logger(), "订阅激光雷达: %s", lidar_topic_.c_str());

  if (use_dual_radar_) {
    RCLCPP_INFO(node_->get_logger(), "订阅毫米波雷达: %s",
                radar_topic_.c_str());
  }

  RCLCPP_INFO(node_->get_logger(), "订阅里程计: %s", odom_topic_.c_str());
  RCLCPP_INFO(node_->get_logger(), "基坐标系: %s", base_frame_.c_str());
}

DualRadarTraversableExtractor::~DualRadarTraversableExtractor() = default;

void DualRadarTraversableExtractor::loadParameters() {
  lidar_topic_ = node_->declare_parameter<std::string>(
      "lidar_topic", "/cloud_registered_body");
  radar_topic_ =
      node_->declare_parameter<std::string>("radar_topic", "/radar_points");
  odom_topic_ = node_->declare_parameter<std::string>("odom_topic", "/odom");

  use_dual_radar_ = node_->declare_parameter<bool>("use_dual_radar", false);
  use_patchworkpp_ = node_->declare_parameter<bool>("use_patchworkpp", true);
  refined_height_threshold_ =
      node_->declare_parameter<double>("refined_height_threshold", 0.1);

  base_frame_ =
      node_->declare_parameter<std::string>("base_frame", "base_link");
  lidar_frame_ = node_->declare_parameter<std::string>("lidar_frame", "lidar");
  radar_frame_ = node_->declare_parameter<std::string>("radar_frame", "radar");

  time_sync_threshold_ =
      node_->declare_parameter<double>("time_sync_threshold", 0.05);
  voxel_leaf_size_ = node_->declare_parameter<double>("voxel_leaf_size", 0.05);
  height_threshold_ = node_->declare_parameter<double>("height_threshold", 0.1);
  robot_radius_ = node_->declare_parameter<double>("robot_radius", 0.3);
  max_range_ = node_->declare_parameter<double>("max_range", 10.0);

  use_statistical_filter_ =
      node_->declare_parameter<bool>("use_statistical_filter", true);
  statistical_filter_k_ =
      node_->declare_parameter<int>("statistical_filter_k", 50);
  statistical_filter_std_ =
      node_->declare_parameter<double>("statistical_filter_std", 1.0);

  use_ground_expansion_ =
      node_->declare_parameter<bool>("use_ground_expansion", true);
  expansion_radius_ = node_->declare_parameter<double>("expansion_radius", 0.5);
  expansion_height_ = node_->declare_parameter<double>("expansion_height", 0.3);

  use_obstacle_clustering_ =
      node_->declare_parameter<bool>("use_obstacle_clustering", true);
  cluster_tolerance_ =
      node_->declare_parameter<double>("cluster_tolerance", 0.1);
  min_cluster_size_ = node_->declare_parameter<int>("min_cluster_size", 10);
  max_cluster_size_ = node_->declare_parameter<int>("max_cluster_size", 25000);

  RCLCPP_INFO(node_->get_logger(), "参数加载完成:");
  RCLCPP_INFO(node_->get_logger(), "  激光雷达话题: %s", lidar_topic_.c_str());

  if (use_dual_radar_) {
    RCLCPP_INFO(node_->get_logger(), "  毫米波雷达话题: %s",
                radar_topic_.c_str());
    RCLCPP_INFO(node_->get_logger(), "  双雷达模式: 启用");
  } else {
    RCLCPP_INFO(node_->get_logger(), "  双雷达模式: 禁用（仅使用激光雷达）");
  }

  RCLCPP_INFO(node_->get_logger(), "  里程计话题: %s", odom_topic_.c_str());
  RCLCPP_INFO(node_->get_logger(), "  PatchWorkpp 地面分割: %s",
              use_patchworkpp_ ? "启用" : "禁用");
  RCLCPP_INFO(node_->get_logger(), "  体素滤波器叶子大小: %.3f",
              voxel_leaf_size_);
  RCLCPP_INFO(node_->get_logger(), "  地面高度阈值: %.3f", height_threshold_);
  RCLCPP_INFO(node_->get_logger(), "  精细高度过滤阈值: %.3f",
              refined_height_threshold_);
  RCLCPP_INFO(node_->get_logger(), "  机器人半径: %.3f", robot_radius_);
  RCLCPP_INFO(node_->get_logger(), "  最大处理距离: %.1f", max_range_);
  RCLCPP_INFO(node_->get_logger(), "  地面点云膨胀: %s",
              use_ground_expansion_ ? "启用" : "禁用");

  if (use_ground_expansion_) {
    RCLCPP_INFO(node_->get_logger(), "  膨胀半径: %.3f 米", expansion_radius_);
    RCLCPP_INFO(node_->get_logger(), "  膨胀高度范围: %.3f 米",
                expansion_height_);
  }

  RCLCPP_INFO(node_->get_logger(), "  障碍物点云聚类滤波: %s",
              use_obstacle_clustering_ ? "启用" : "禁用");

  if (use_obstacle_clustering_) {
    RCLCPP_INFO(node_->get_logger(), "  聚类容忍距离: %.3f 米",
                cluster_tolerance_);
    RCLCPP_INFO(node_->get_logger(), "  最小聚类大小: %d 点",
                min_cluster_size_);
    RCLCPP_INFO(node_->get_logger(), "  最大聚类大小: %d 点",
                max_cluster_size_);
  }
}

void DualRadarTraversableExtractor::lidarCallback(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  {
    std::lock_guard<std::mutex> lock(lidar_mutex_);

    lidar_queue_.push_back(msg);
    lidar_time_queue_.push_back(rclcpp::Time(msg->header.stamp).seconds());

    while (lidar_queue_.size() > 10) {
      lidar_queue_.pop_front();
      lidar_time_queue_.pop_front();
    }
  }

  if (!lidar_transform_ready_) {
    lidar_transform_ready_ =
        getTransform(lidar_frame_, base_frame_, lidar_to_base_matrix_);
  }

  processData();
}

void DualRadarTraversableExtractor::radarCallback(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  if (!use_dual_radar_) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(radar_mutex_);

    radar_queue_.push_back(msg);
    radar_time_queue_.push_back(rclcpp::Time(msg->header.stamp).seconds());

    while (radar_queue_.size() > 10) {
      radar_queue_.pop_front();
      radar_time_queue_.pop_front();
    }
  }

  if (!radar_transform_ready_) {
    radar_transform_ready_ =
        getTransform(radar_frame_, base_frame_, radar_to_base_matrix_);
  }
}

void DualRadarTraversableExtractor::odomCallback(
    nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  (void)msg;
}

void DualRadarTraversableExtractor::processData() {
  sensor_msgs::msg::PointCloud2::ConstSharedPtr lidar_msg;
  double lidar_time = 0.0;

  {
    std::lock_guard<std::mutex> lock(lidar_mutex_);

    if (lidar_queue_.empty() || lidar_time_queue_.empty()) {
      return;
    }

    lidar_msg = lidar_queue_.back();
    lidar_time = lidar_time_queue_.back();
  }

  pcl::PointCloud<PointType>::Ptr lidar_cloud(new pcl::PointCloud<PointType>);
  pcl::fromROSMsg(*lidar_msg, *lidar_cloud);

  if (lidar_cloud->empty()) {
    RCLCPP_WARN(node_->get_logger(), "接收到空的激光雷达点云");
    return;
  }

  if (lidar_transform_ready_) {
    lidar_cloud = transformPointCloud(lidar_cloud, lidar_to_base_matrix_);
    RCLCPP_DEBUG(node_->get_logger(), "激光雷达点云已变换到 base_link 坐标系");
  } else {
    RCLCPP_WARN(node_->get_logger(), "激光雷达变换矩阵未就绪，跳过变换");
  }

  pcl::PointCloud<PointType>::Ptr radar_cloud(new pcl::PointCloud<PointType>);
  bool radar_data_found = false;

  if (use_dual_radar_ && radar_transform_ready_) {
    std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> radar_queue_copy;
    std::deque<double> radar_time_queue_copy;

    {
      std::lock_guard<std::mutex> lock(radar_mutex_);
      radar_queue_copy = radar_queue_;
      radar_time_queue_copy = radar_time_queue_;
    }

    if (!radar_queue_copy.empty() &&
        radar_queue_copy.size() == radar_time_queue_copy.size()) {
      double min_time_diff = std::numeric_limits<double>::max();
      std::size_t best_idx = 0;

      for (std::size_t i = 0; i < radar_time_queue_copy.size(); ++i) {
        const double time_diff =
            std::abs(radar_time_queue_copy[i] - lidar_time);

        if (time_diff < min_time_diff) {
          min_time_diff = time_diff;
          best_idx = i;
        }
      }

      if (min_time_diff < time_sync_threshold_) {
        pcl::fromROSMsg(*radar_queue_copy[best_idx], *radar_cloud);

        if (!radar_cloud->empty()) {
          radar_cloud = transformPointCloud(radar_cloud, radar_to_base_matrix_);
          radar_data_found = true;

          RCLCPP_DEBUG(node_->get_logger(),
                       "找到同步的雷达数据，时间差: %.3f 秒", min_time_diff);
        }
      }
    }
  }

  pcl::PointCloud<PointType>::Ptr combined_cloud;

  if (use_dual_radar_ && radar_data_found) {
    combined_cloud = combinePointClouds(lidar_cloud, radar_cloud);

    RCLCPP_DEBUG(node_->get_logger(),
                 "合并点云: 激光雷达 %zu 点, 毫米波雷达 %zu 点",
                 lidar_cloud->points.size(), radar_cloud->points.size());
  } else {
    combined_cloud = lidar_cloud;

    if (use_dual_radar_) {
      RCLCPP_DEBUG(node_->get_logger(),
                   "未找到同步的雷达数据，仅使用激光雷达数据: %zu 点",
                   lidar_cloud->points.size());
    } else {
      RCLCPP_DEBUG(node_->get_logger(), "单雷达模式，使用激光雷达数据: %zu 点",
                   lidar_cloud->points.size());
    }
  }

  pcl::PointCloud<PointType>::Ptr preprocessed_cloud =
      preprocessPointCloud(combined_cloud);

  pcl::PointCloud<PointType>::Ptr ground_cloud(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr obstacle_cloud(
      new pcl::PointCloud<PointType>);

  if (!segmentGround(preprocessed_cloud, ground_cloud, obstacle_cloud)) {
    RCLCPP_WARN(node_->get_logger(), "地面分割失败");
    return;
  }

  if (use_ground_expansion_) {
    expandGroundCloud(preprocessed_cloud, ground_cloud, obstacle_cloud);
  }

  if (use_obstacle_clustering_) {
    obstacle_cloud = filterObstacleCloud(obstacle_cloud);
  }

  publishResults(combined_cloud, ground_cloud, obstacle_cloud,
                 lidar_msg->header);
}

bool DualRadarTraversableExtractor::getTransform(
    const std::string& from_frame, const std::string& to_frame,
    Eigen::Matrix4d& transform_matrix) {
  try {
    geometry_msgs::msg::TransformStamped transform =
        tf_buffer_->lookupTransform(to_frame, from_frame, tf2::TimePointZero,
                                    tf2::durationFromSec(1.0));

    Eigen::Vector3d translation(transform.transform.translation.x,
                                transform.transform.translation.y,
                                transform.transform.translation.z);

    Eigen::Quaterniond rotation(
        transform.transform.rotation.w, transform.transform.rotation.x,
        transform.transform.rotation.y, transform.transform.rotation.z);

    rotation.normalize();

    Eigen::Isometry3d tf_matrix = Eigen::Isometry3d::Identity();
    tf_matrix.linear() = rotation.toRotationMatrix();
    tf_matrix.translation() = translation;

    transform_matrix = tf_matrix.matrix();

    RCLCPP_INFO(node_->get_logger(), "获取变换矩阵: %s -> %s",
                from_frame.c_str(), to_frame.c_str());

    return true;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(node_->get_logger(), "无法获取变换 %s -> %s: %s",
                from_frame.c_str(), to_frame.c_str(), ex.what());

    return false;
  }
}

pcl::PointCloud<PointType>::Ptr
DualRadarTraversableExtractor::transformPointCloud(
    const pcl::PointCloud<PointType>::Ptr& cloud,
    const Eigen::Matrix4d& transform) {
  pcl::PointCloud<PointType>::Ptr transformed_cloud(
      new pcl::PointCloud<PointType>);
  transformed_cloud->points.reserve(cloud->points.size());

  for (const auto& point : cloud->points) {
    Eigen::Vector4d p_src(point.x, point.y, point.z, 1.0);
    Eigen::Vector4d p_dst = transform * p_src;

    PointType transformed_point;
    transformed_point.x = static_cast<float>(p_dst.x());
    transformed_point.y = static_cast<float>(p_dst.y());
    transformed_point.z = static_cast<float>(p_dst.z());
    transformed_point.intensity = point.intensity;

    transformed_cloud->points.push_back(transformed_point);
  }

  transformed_cloud->width =
      static_cast<std::uint32_t>(transformed_cloud->points.size());
  transformed_cloud->height = 1;
  transformed_cloud->is_dense = cloud->is_dense;

  return transformed_cloud;
}

pcl::PointCloud<PointType>::Ptr
DualRadarTraversableExtractor::combinePointClouds(
    const pcl::PointCloud<PointType>::Ptr& lidar_cloud,
    const pcl::PointCloud<PointType>::Ptr& radar_cloud) {
  pcl::PointCloud<PointType>::Ptr combined_cloud(
      new pcl::PointCloud<PointType>);
  combined_cloud->points.reserve(lidar_cloud->points.size() +
                                 radar_cloud->points.size());

  combined_cloud->points.insert(combined_cloud->points.end(),
                                lidar_cloud->points.begin(),
                                lidar_cloud->points.end());

  combined_cloud->points.insert(combined_cloud->points.end(),
                                radar_cloud->points.begin(),
                                radar_cloud->points.end());

  combined_cloud->width =
      static_cast<std::uint32_t>(combined_cloud->points.size());
  combined_cloud->height = 1;
  combined_cloud->is_dense = lidar_cloud->is_dense && radar_cloud->is_dense;

  return combined_cloud;
}

pcl::PointCloud<PointType>::Ptr
DualRadarTraversableExtractor::preprocessPointCloud(
    const pcl::PointCloud<PointType>::Ptr& input_cloud) {
  pcl::PointCloud<PointType>::Ptr filtered_cloud(
      new pcl::PointCloud<PointType>);
  filtered_cloud->points.reserve(input_cloud->points.size());

  for (const auto& point : input_cloud->points) {
    const double distance = std::sqrt(static_cast<double>(point.x) * point.x +
                                      static_cast<double>(point.y) * point.y +
                                      static_cast<double>(point.z) * point.z);

    if (distance <= max_range_) {
      filtered_cloud->points.push_back(point);
    }
  }

  filtered_cloud->width =
      static_cast<std::uint32_t>(filtered_cloud->points.size());
  filtered_cloud->height = 1;
  filtered_cloud->is_dense = true;

  if (filtered_cloud->empty()) {
    return filtered_cloud;
  }

  voxel_filter_.setInputCloud(filtered_cloud);
  pcl::PointCloud<PointType>::Ptr downsampled_cloud(
      new pcl::PointCloud<PointType>);
  voxel_filter_.filter(*downsampled_cloud);

  if (use_statistical_filter_ &&
      downsampled_cloud->points.size() >
          static_cast<std::size_t>(statistical_filter_k_)) {
    stat_filter_.setInputCloud(downsampled_cloud);
    pcl::PointCloud<PointType>::Ptr stat_filtered_cloud(
        new pcl::PointCloud<PointType>);
    stat_filter_.filter(*stat_filtered_cloud);
    return stat_filtered_cloud;
  }

  return downsampled_cloud;
}

bool DualRadarTraversableExtractor::segmentGround(
    const pcl::PointCloud<PointType>::Ptr& input_cloud,
    pcl::PointCloud<PointType>::Ptr& ground_cloud,
    pcl::PointCloud<PointType>::Ptr& obstacle_cloud) {
  if (input_cloud->empty()) {
    return false;
  }

  if (use_patchworkpp_ && patchworkpp_) {
    try {
      double time_taken = 0.0;
      patchworkpp_->estimate_ground(*input_cloud, *ground_cloud,
                                    *obstacle_cloud, time_taken);

      ground_cloud->width =
          static_cast<std::uint32_t>(ground_cloud->points.size());
      ground_cloud->height = 1;
      ground_cloud->is_dense = true;

      obstacle_cloud->width =
          static_cast<std::uint32_t>(obstacle_cloud->points.size());
      obstacle_cloud->height = 1;
      obstacle_cloud->is_dense = true;

      RCLCPP_DEBUG(
          node_->get_logger(),
          "PatchWorkpp 地面分割完成: 地面点 %zu, 障碍物点 %zu, 耗时 %.3f ms",
          ground_cloud->points.size(), obstacle_cloud->points.size(),
          time_taken * 1000.0);

      if (!ground_cloud->empty() || !obstacle_cloud->empty()) {
        return true;
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN(node_->get_logger(),
                  "PatchWorkpp 分割失败，回退到简单高度阈值方法: %s", e.what());
    }
  }

  double min_z = std::numeric_limits<double>::max();
  double max_z = std::numeric_limits<double>::lowest();

  for (const auto& point : input_cloud->points) {
    min_z = std::min(min_z, static_cast<double>(point.z));
    max_z = std::max(max_z, static_cast<double>(point.z));
  }

  if (max_z - min_z < 0.1) {
    *ground_cloud = *input_cloud;
    obstacle_cloud->clear();

    ground_cloud->width =
        static_cast<std::uint32_t>(ground_cloud->points.size());
    ground_cloud->height = 1;
    ground_cloud->is_dense = true;

    obstacle_cloud->width = 0;
    obstacle_cloud->height = 1;
    obstacle_cloud->is_dense = true;

    RCLCPP_DEBUG(node_->get_logger(), "点云高度差很小 (%.3f)，全部视为地面点",
                 max_z - min_z);

    return true;
  }

  const double ground_threshold = min_z + height_threshold_;

  ground_cloud->clear();
  obstacle_cloud->clear();

  for (const auto& point : input_cloud->points) {
    if (point.z <= ground_threshold) {
      ground_cloud->points.push_back(point);
    } else {
      obstacle_cloud->points.push_back(point);
    }
  }

  ground_cloud->width = static_cast<std::uint32_t>(ground_cloud->points.size());
  ground_cloud->height = 1;
  ground_cloud->is_dense = true;

  obstacle_cloud->width =
      static_cast<std::uint32_t>(obstacle_cloud->points.size());
  obstacle_cloud->height = 1;
  obstacle_cloud->is_dense = true;

  if (!ground_cloud->empty()) {
    pcl::PointCloud<PointType> pc_ground_filtered;
    pcl::PointCloud<PointType> elevated_points;

    pc_ground_filtered.points.reserve(ground_cloud->points.size());
    elevated_points.points.reserve(ground_cloud->points.size());

    double avg_height = 0.0;

    for (const auto& pt : ground_cloud->points) {
      avg_height += pt.z;
    }

    avg_height /= static_cast<double>(ground_cloud->points.size());

    const double filter_threshold = avg_height + refined_height_threshold_;

    for (const auto& pt : ground_cloud->points) {
      if (pt.z > filter_threshold) {
        elevated_points.push_back(pt);
      } else {
        pc_ground_filtered.push_back(pt);
      }
    }

    ground_cloud->clear();
    ground_cloud->points = pc_ground_filtered.points;
    ground_cloud->width =
        static_cast<std::uint32_t>(ground_cloud->points.size());
    ground_cloud->height = 1;
    ground_cloud->is_dense = true;

    for (const auto& pt : elevated_points.points) {
      obstacle_cloud->points.push_back(pt);
    }

    obstacle_cloud->width =
        static_cast<std::uint32_t>(obstacle_cloud->points.size());
    obstacle_cloud->height = 1;
    obstacle_cloud->is_dense = true;

    RCLCPP_DEBUG(node_->get_logger(),
                 "高度过滤完成: 过滤掉 %zu 个高点, 平均高度 %.3f",
                 elevated_points.points.size(), avg_height);
  }

  RCLCPP_DEBUG(node_->get_logger(),
               "地面分割完成: 地面点 %zu, 障碍物点 %zu, 地面高度阈值 %.3f",
               ground_cloud->points.size(), obstacle_cloud->points.size(),
               ground_threshold);

  return !ground_cloud->empty() || !obstacle_cloud->empty();
}

void DualRadarTraversableExtractor::publishResults(
    const pcl::PointCloud<PointType>::Ptr& combined_cloud,
    const pcl::PointCloud<PointType>::Ptr& ground_cloud,
    const pcl::PointCloud<PointType>::Ptr& obstacle_cloud,
    const std_msgs::msg::Header& header) {
  std_msgs::msg::Header base_link_header = header;
  base_link_header.frame_id = base_frame_;

  if (!combined_cloud->empty() &&
      combined_cloud_pub_->get_subscription_count() > 0) {
    sensor_msgs::msg::PointCloud2 combined_msg;
    pcl::toROSMsg(*combined_cloud, combined_msg);
    combined_msg.header = base_link_header;
    combined_cloud_pub_->publish(combined_msg);
  }

  if (!ground_cloud->empty() &&
      ground_cloud_pub_->get_subscription_count() > 0) {
    sensor_msgs::msg::PointCloud2 ground_msg;
    pcl::toROSMsg(*ground_cloud, ground_msg);
    ground_msg.header = base_link_header;
    ground_cloud_pub_->publish(ground_msg);
  }

  if (!obstacle_cloud->empty() &&
      obstacle_cloud_pub_->get_subscription_count() > 0) {
    sensor_msgs::msg::PointCloud2 obstacle_msg;
    pcl::toROSMsg(*obstacle_cloud, obstacle_msg);
    obstacle_msg.header = base_link_header;
    obstacle_cloud_pub_->publish(obstacle_msg);
  }

  RCLCPP_DEBUG(node_->get_logger(),
               "发布完成: 合并点云 %zu 点, 地面点云 %zu 点, 障碍物点云 %zu 点",
               combined_cloud->points.size(), ground_cloud->points.size(),
               obstacle_cloud->points.size());
}

void DualRadarTraversableExtractor::expandGroundCloud(
    const pcl::PointCloud<PointType>::Ptr& original_cloud,
    pcl::PointCloud<PointType>::Ptr& ground_cloud,
    pcl::PointCloud<PointType>::Ptr& obstacle_cloud) {
  (void)original_cloud;

  if (ground_cloud->empty() || obstacle_cloud->empty()) {
    RCLCPP_DEBUG(node_->get_logger(), "地面点云或障碍物点云为空，跳过膨胀");
    return;
  }

  const std::size_t original_ground_size = ground_cloud->points.size();
  const std::size_t original_obstacle_size = obstacle_cloud->points.size();

  pcl::PointCloud<PointType>::Ptr expanded_ground_cloud(
      new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr filtered_obstacle_cloud(
      new pcl::PointCloud<PointType>);

  *expanded_ground_cloud = *ground_cloud;
  filtered_obstacle_cloud->points.reserve(obstacle_cloud->points.size());

  for (const auto& obstacle_point : obstacle_cloud->points) {
    bool is_near_ground = false;

    for (const auto& ground_point : ground_cloud->points) {
      const double dx = static_cast<double>(obstacle_point.x) - ground_point.x;
      const double dy = static_cast<double>(obstacle_point.y) - ground_point.y;
      const double dz = static_cast<double>(obstacle_point.z) - ground_point.z;

      const double horizontal_distance = std::sqrt(dx * dx + dy * dy);
      const double vertical_distance = std::fabs(dz);

      if (horizontal_distance <= expansion_radius_ &&
          vertical_distance <= expansion_height_) {
        is_near_ground = true;
        break;
      }
    }

    if (is_near_ground) {
      expanded_ground_cloud->points.push_back(obstacle_point);
    } else {
      filtered_obstacle_cloud->points.push_back(obstacle_point);
    }
  }

  expanded_ground_cloud->width =
      static_cast<std::uint32_t>(expanded_ground_cloud->points.size());
  expanded_ground_cloud->height = 1;
  expanded_ground_cloud->is_dense = true;

  filtered_obstacle_cloud->width =
      static_cast<std::uint32_t>(filtered_obstacle_cloud->points.size());
  filtered_obstacle_cloud->height = 1;
  filtered_obstacle_cloud->is_dense = true;

  *ground_cloud = *expanded_ground_cloud;
  *obstacle_cloud = *filtered_obstacle_cloud;

  const std::size_t expanded_points =
      ground_cloud->points.size() - original_ground_size;
  const std::size_t removed_points =
      original_obstacle_size > obstacle_cloud->points.size()
          ? original_obstacle_size - obstacle_cloud->points.size()
          : 0U;

  RCLCPP_DEBUG(node_->get_logger(), "地面点云膨胀完成:");
  RCLCPP_DEBUG(node_->get_logger(), "  原始地面点: %zu -> %zu (新增 %zu 点)",
               original_ground_size, ground_cloud->points.size(),
               expanded_points);

  RCLCPP_DEBUG(node_->get_logger(), "  原始障碍物点: %zu -> %zu (移除 %zu 点)",
               original_obstacle_size, obstacle_cloud->points.size(),
               removed_points);

  RCLCPP_DEBUG(node_->get_logger(),
               "  膨胀参数: 半径 %.3f 米, 高度范围 %.3f 米", expansion_radius_,
               expansion_height_);
}

pcl::PointCloud<PointType>::Ptr
DualRadarTraversableExtractor::filterObstacleCloud(
    const pcl::PointCloud<PointType>::Ptr& obstacle_cloud) {
  if (obstacle_cloud->empty()) {
    RCLCPP_DEBUG(node_->get_logger(), "障碍物点云为空，跳过聚类滤波");
    return obstacle_cloud;
  }

  const std::size_t original_size = obstacle_cloud->points.size();

  pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
  tree->setInputCloud(obstacle_cloud);

  std::vector<pcl::PointIndices> cluster_indices;

  pcl::EuclideanClusterExtraction<PointType> ec;
  ec.setClusterTolerance(cluster_tolerance_);
  ec.setMinClusterSize(min_cluster_size_);
  ec.setMaxClusterSize(max_cluster_size_);
  ec.setSearchMethod(tree);
  ec.setInputCloud(obstacle_cloud);
  ec.extract(cluster_indices);

  pcl::PointCloud<PointType>::Ptr filtered_cloud(
      new pcl::PointCloud<PointType>);

  for (const auto& cluster : cluster_indices) {
    for (const auto& idx : cluster.indices) {
      filtered_cloud->points.push_back(obstacle_cloud->points[idx]);
    }
  }

  filtered_cloud->width =
      static_cast<std::uint32_t>(filtered_cloud->points.size());
  filtered_cloud->height = 1;
  filtered_cloud->is_dense = true;

  const std::size_t filtered_size = filtered_cloud->points.size();
  const std::size_t removed_points =
      original_size > filtered_size ? original_size - filtered_size : 0U;

  RCLCPP_DEBUG(node_->get_logger(), "障碍物点云聚类滤波完成:");
  RCLCPP_DEBUG(node_->get_logger(), "  原始点云: %zu 点", original_size);
  RCLCPP_DEBUG(node_->get_logger(), "  滤波后点云: %zu 点", filtered_size);
  RCLCPP_DEBUG(node_->get_logger(), "  移除孤立点: %zu 点", removed_points);
  RCLCPP_DEBUG(node_->get_logger(), "  有效聚类数: %zu 个",
               cluster_indices.size());
  RCLCPP_DEBUG(node_->get_logger(),
               "  聚类参数: 容忍距离 %.3f 米, 最小大小 %d 点, 最大大小 %d 点",
               cluster_tolerance_, min_cluster_size_, max_cluster_size_);

  return filtered_cloud;
}