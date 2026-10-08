#include "terrain_filter/terrain_filter.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

TerrainFilter::TerrainFilter(const rclcpp::Node::SharedPtr& node)
    : node_(node),
      tf_buffer_(nullptr),
      tf_listener_(nullptr),
      radar_transform_ready_(false) {
  // 参数在构造时读取；随后创建 TF、订阅和滤波器。处理由激光点云回调触发，
  // 毫米波雷达只缓存数据，等待激光帧按时间差选择可融合的观测。
  loadParameters();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

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

  ground_cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "ground_cloud", rclcpp::QoS(10));

  obstacle_cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "obstacle_cloud", rclcpp::QoS(10));

  stair_cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "stair_cloud", rclcpp::QoS(10));

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
    patchworkpp_ = std::make_shared<patchwork::PclGroundSegmenter>(node_);
    RCLCPP_INFO(node_->get_logger(), "公共 PatchWork++ 地面分割器初始化完成");
  }

  RCLCPP_INFO(node_->get_logger(), "TerrainFilter 初始化完成");
  RCLCPP_INFO(node_->get_logger(), "订阅激光雷达: %s", lidar_topic_.c_str());

  if (use_dual_radar_) {
    RCLCPP_INFO(node_->get_logger(), "订阅毫米波雷达: %s",
                radar_topic_.c_str());
  }

  RCLCPP_INFO(node_->get_logger(), "基坐标系: %s", base_frame_.c_str());
}

TerrainFilter::~TerrainFilter() = default;

void TerrainFilter::loadParameters() {
  lidar_topic_ =
      node_->declare_parameter<std::string>("lidar_topic", "/cloud_registered");
  radar_topic_ =
      node_->declare_parameter<std::string>("radar_topic", "/radar_points");

  use_dual_radar_ = node_->declare_parameter<bool>("use_dual_radar", false);
  use_patchworkpp_ = node_->declare_parameter<bool>("use_patchworkpp", true);
  refined_height_threshold_ =
      node_->declare_parameter<double>("refined_height_threshold", 0.1);

  base_frame_ =
      node_->declare_parameter<std::string>("base_frame", "base_link");
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

  use_stair_region_growing_ =
      node_->declare_parameter<bool>("use_stair_region_growing", true);
  use_stair_riser_detection_ =
      node_->declare_parameter<bool>("use_stair_riser_detection", true);
  stair_grid_resolution_ =
      node_->declare_parameter<double>("stair_grid_resolution", 0.1);
  stair_max_step_height_ =
      node_->declare_parameter<double>("stair_max_step_height", 0.22);
  stair_max_cell_height_span_ =
      node_->declare_parameter<double>("stair_max_cell_height_span", 0.25);
  stair_max_gap_cells_ =
      node_->declare_parameter<int>("stair_max_gap_cells", 1);
  stair_hysteresis_frames_ =
      node_->declare_parameter<int>("stair_hysteresis_frames", 5);
  stair_hysteresis_neighbor_cells_ =
      node_->declare_parameter<int>("stair_hysteresis_neighbor_cells", 1);
  stair_surface_tolerance_ =
      node_->declare_parameter<double>("stair_surface_tolerance", 0.025);
  stair_min_step_height_ =
      node_->declare_parameter<double>("stair_min_step_height", 0.07);
  stair_min_step_count_ =
      node_->declare_parameter<int>("stair_min_step_count", 3);
  stair_min_total_rise_ =
      node_->declare_parameter<double>("stair_min_total_rise", 0.20);
  stair_min_tread_depth_ =
      node_->declare_parameter<double>("stair_min_tread_depth", 0.15);
  stair_max_tread_depth_ =
      node_->declare_parameter<double>("stair_max_tread_depth", 0.45);
  stair_step_height_tolerance_ =
      node_->declare_parameter<double>("stair_step_height_tolerance", 0.03);
  stair_model_tread_height_tolerance_ = node_->declare_parameter<double>(
      "stair_model_tread_height_tolerance", 0.05);
  stair_model_riser_position_tolerance_ = node_->declare_parameter<double>(
      "stair_model_riser_position_tolerance", 0.08);
  stair_model_max_extrapolated_steps_ = node_->declare_parameter<int>(
      "stair_model_max_extrapolated_steps", 6);
  stair_surface_history_duration_ = node_->declare_parameter<double>(
      "stair_surface_history_duration", 3.0);
  stair_surface_history_distance_ = node_->declare_parameter<double>(
      "stair_surface_history_distance", 0.04);
  stair_direction_tolerance_deg_ =
      node_->declare_parameter<double>("stair_direction_tolerance_deg", 15.0);
  stair_min_width_ =
      node_->declare_parameter<double>("stair_min_width", 0.50);
  stair_ground_seed_min_points_ =
      node_->declare_parameter<int>("stair_ground_seed_min_points", 3);
  stair_ground_seed_max_span_ =
      node_->declare_parameter<double>("stair_ground_seed_max_span", 0.03);
  stair_ground_reference_max_abs_height_ = node_->declare_parameter<double>(
      "stair_ground_reference_max_abs_height", 0.15);

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

  RCLCPP_INFO(node_->get_logger(), "  PatchWorkpp 地面分割: %s",
              use_patchworkpp_ ? "启用" : "禁用");
  RCLCPP_INFO(node_->get_logger(), "  体素滤波器叶子大小: %.3f",
              voxel_leaf_size_);
  RCLCPP_INFO(node_->get_logger(), "  地面高度阈值: %.3f", height_threshold_);
  RCLCPP_INFO(node_->get_logger(), "  精细高度过滤阈值: %.3f",
              refined_height_threshold_);
  RCLCPP_INFO(node_->get_logger(), "  机器人半径: %.3f", robot_radius_);
  RCLCPP_INFO(node_->get_logger(), "  最大处理距离: %.1f", max_range_);
  RCLCPP_INFO(node_->get_logger(), "  楼梯结构识别: %s",
              use_stair_region_growing_ ? "启用" : "禁用");

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

void TerrainFilter::lidarCallback(
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

  processData();
}

void TerrainFilter::radarCallback(
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

// 单帧处理主链路：读取最新点云，统一到 base_link，完成预处理、地面分割、
// 楼梯识别和障碍物聚类，最后发布各类点云。TerrainLabel 在整个流程中与
// 预处理点云保持相同索引，避免重复拷贝点云时丢失分类关系。
void TerrainFilter::processData() {
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
  // 处理时间来自传感器消息而非系统时钟，bag 回绕或重复时间戳时清理旧状态，
  // 避免上一段数据的楼梯模型被应用到另一段观测。
  if (lidar_time <= current_cloud_stamp_) {
    stair_model_valid_ = false;
    stair_model_hold_frames_remaining_ = 0;
    stair_cell_hold_frames_.clear();
    stair_cell_surface_heights_.clear();
    stair_cell_height_spans_.clear();
    stair_surface_history_.clear();
  }
  current_cloud_stamp_ = lidar_time;

  pcl::PointCloud<PointType>::Ptr lidar_cloud(new pcl::PointCloud<PointType>);
  pcl::fromROSMsg(*lidar_msg, *lidar_cloud);

  if (lidar_cloud->empty()) {
    RCLCPP_WARN(node_->get_logger(), "接收到空的激光雷达点云");
    return;
  }

  // 配准点云通常位于 map/odom。这里使用点云采集时刻的 TF 转到 base_link，
  // 避免机器人运动后使用最新 TF 造成楼梯和障碍物位置漂移。
  const auto& source_frame = lidar_msg->header.frame_id;
  if (source_frame.empty()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "输入点云缺少frame_id，跳过该帧");
    return;
  }
  if (source_frame != base_frame_) {
    const rclcpp::Time stamp(lidar_msg->header.stamp);
    if (stamp.nanoseconds() == 0) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "输入点云缺少时间戳，跳过动态坐标变换");
      return;
    }
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    if (!getTransform(source_frame, base_frame_, transform, stamp)) return;
    current_cloud_to_base_ = transform;
    current_cloud_to_base_valid_ = true;
    current_cloud_frame_ = source_frame;
    lidar_cloud = transformPointCloud(lidar_cloud, transform);
  } else {
    current_cloud_to_base_ = Eigen::Matrix4d::Identity();
    current_cloud_to_base_valid_ = false;
    current_cloud_frame_.clear();
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
  pcl::PointCloud<PointType>::Ptr stair_cloud(new pcl::PointCloud<PointType>);
  std::vector<TerrainLabel> labels;

  if (!segmentGround(preprocessed_cloud, ground_cloud, obstacle_cloud,
                     &labels)) {
    RCLCPP_WARN(node_->get_logger(), "地面分割失败");
    return;
  }

  // 普通地面邻近膨胀会把门槛、砖块和矮箱一并吞掉，因此不再参与主流程。
  // 楼梯识别独立运行，只有完整的多级结构通过验证后才从障碍中移除。
  if (use_stair_region_growing_) {
    stair_cloud = detectTraversableStairs(preprocessed_cloud, labels);
  }

  buildClassifiedClouds(preprocessed_cloud, labels, ground_cloud,
                        obstacle_cloud, stair_cloud);

  if (use_obstacle_clustering_) {
    obstacle_cloud = filterObstacleCloud(obstacle_cloud);
  }

  publishResults(combined_cloud, ground_cloud, obstacle_cloud, stair_cloud,
                 lidar_msg->header);
}

bool TerrainFilter::getTransform(const std::string& from_frame,
                                 const std::string& to_frame,
                                 Eigen::Matrix4d& transform_matrix,
                                 const rclcpp::Time& stamp) {
  try {
    geometry_msgs::msg::TransformStamped transform =
        tf_buffer_->lookupTransform(to_frame, from_frame, stamp,
                                    rclcpp::Duration::from_seconds(
                                        stamp.nanoseconds() == 0 ? 1.0 : 0.1));

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

    RCLCPP_DEBUG(node_->get_logger(), "获取变换矩阵: %s -> %s",
                 from_frame.c_str(), to_frame.c_str());

    return true;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "无法获取变换 %s -> %s: %s", from_frame.c_str(),
                         to_frame.c_str(), ex.what());

    return false;
  }
}

pcl::PointCloud<PointType>::Ptr TerrainFilter::transformPointCloud(
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

pcl::PointCloud<PointType>::Ptr TerrainFilter::combinePointClouds(
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

// 预处理只删除量程外点并降低点云密度。统计滤波是可选项，因为滤波过强
// 会优先删掉稀疏的低矮障碍和楼梯立面点，影响后续结构判断。
pcl::PointCloud<PointType>::Ptr TerrainFilter::preprocessPointCloud(
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

// 地面分割同时返回点云和逐点标签。后续以 labels 为准重新构造输出点云，
// 这样楼梯检测可直接把原先的 Ground/Obstacle 改为 TraversableStair，且不依赖
// 浮点坐标相等来反查原始点。
bool TerrainFilter::segmentGround(
    const pcl::PointCloud<PointType>::Ptr& input_cloud,
    pcl::PointCloud<PointType>::Ptr& ground_cloud,
    pcl::PointCloud<PointType>::Ptr& obstacle_cloud,
    std::vector<TerrainLabel>* labels) {
  if (input_cloud->empty()) {
    return false;
  }

  if (labels != nullptr) {
    labels->assign(input_cloud->size(), TerrainLabel::Unknown);
  }

  if (use_patchworkpp_ && patchworkpp_) {
    try {
      double time_taken = 0.0;
      std::vector<std::uint8_t> raw_labels;
      patchworkpp_->segment(*input_cloud, *ground_cloud, *obstacle_cloud,
                            &time_taken, labels != nullptr ? &raw_labels : nullptr);
      if (labels != nullptr && raw_labels.size() == labels->size()) {
        for (std::size_t index = 0; index < raw_labels.size(); ++index) {
          (*labels)[index] = raw_labels[index] == 1U
                                 ? TerrainLabel::Ground
                                 : TerrainLabel::Obstacle;
        }
      }

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

    if (labels != nullptr) {
      std::fill(labels->begin(), labels->end(), TerrainLabel::Ground);
    }

    RCLCPP_DEBUG(node_->get_logger(), "点云高度差很小 (%.3f)，全部视为地面点",
                 max_z - min_z);

    return true;
  }

  const double ground_threshold = min_z + height_threshold_;

  ground_cloud->clear();
  obstacle_cloud->clear();

  for (std::size_t index = 0; index < input_cloud->points.size(); ++index) {
    const auto& point = input_cloud->points[index];
    if (point.z <= ground_threshold) {
      ground_cloud->points.push_back(point);
      if (labels != nullptr) (*labels)[index] = TerrainLabel::Ground;
    } else {
      obstacle_cloud->points.push_back(point);
      if (labels != nullptr) (*labels)[index] = TerrainLabel::Obstacle;
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
      if (pt.z > filter_threshold) elevated_points.push_back(pt);
      else pc_ground_filtered.push_back(pt);
    }

    if (labels != nullptr) {
      for (std::size_t index = 0; index < input_cloud->points.size(); ++index) {
        if ((*labels)[index] == TerrainLabel::Ground &&
            input_cloud->points[index].z > filter_threshold) {
          (*labels)[index] = TerrainLabel::Obstacle;
        }
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

// 楼梯检测入口：input_cloud 已位于 base_frame_，labels 与点索引一一对应，
// 初始标签来自地面分割。本函数只把确认的点改成 TraversableStair；调用方
// 随后按最终标签重建三个互斥输出，楼梯点由此从 obstacle_cloud 中移除。
//
// 阅读顺序：
// 1. XY 栅格化并统计高度分位数，建立地面基准与候选单元。
// 2. 全局提取立面段，搜索多级序列（当前配置最少三级），不要求序列接地。
// 3. 根据序列拟合级高、进深、方向和宽度，有限外推并逐点补全踏面/立面。
// 4. 对候选连通组件执行局部立面重试、踏面高度序列兜底；该路径要求接地。
// 5. 应用栅格短期保持，再用固定坐标系中的历史表面点补回运动时的漏点。
//
// 三种历史依据不同：几何模型按级高/进深预测，栅格历史按位置和高度匹配，
// 表面记忆按完整 TF 变换后的三维近邻匹配。它们都不是整个区域无条件放行。
pcl::PointCloud<PointType>::Ptr TerrainFilter::detectTraversableStairs(
    const pcl::PointCloud<PointType>::Ptr& input_cloud,
    std::vector<TerrainLabel>& labels) {
  pcl::PointCloud<PointType>::Ptr stair_cloud(new pcl::PointCloud<PointType>);
  if (input_cloud->empty() || labels.size() != input_cloud->size()) {
    stair_surface_history_.clear();
    stair_cell_hold_frames_.clear();
    stair_cell_surface_heights_.clear();
    stair_cell_height_spans_.clear();
    stair_model_valid_ = false;
    stair_model_hold_frames_remaining_ = 0;
    return stair_cloud;
  }

  if (stair_grid_resolution_ <= 0.0 || stair_surface_tolerance_ <= 0.0 ||
      stair_min_step_height_ <= 0.0 ||
      stair_max_step_height_ < stair_min_step_height_ ||
      stair_min_step_count_ < 2 || stair_min_total_rise_ <= 0.0 ||
      stair_max_gap_cells_ < 0 || stair_hysteresis_frames_ < 0 ||
      stair_hysteresis_neighbor_cells_ < 0 ||
      stair_min_tread_depth_ <= 0.0 ||
      stair_max_tread_depth_ < stair_min_tread_depth_ ||
      stair_model_tread_height_tolerance_ <= 0.0 ||
      stair_model_riser_position_tolerance_ <= 0.0 ||
      stair_model_max_extrapolated_steps_ < 0 ||
      stair_direction_tolerance_deg_ <= 0.0 ||
      stair_direction_tolerance_deg_ >= 90.0 || stair_min_width_ <= 0.0 ||
      stair_ground_seed_min_points_ < 1 ||
      stair_ground_reference_max_abs_height_ <= 0.0) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "楼梯结构识别参数无效，当前帧按障碍处理");
    stair_surface_history_.clear();
    stair_cell_hold_frames_.clear();
    stair_cell_surface_heights_.clear();
    stair_cell_height_spans_.clear();
    return stair_cloud;
  }

  // StairCell 只描述当前帧的局部栅格，不属于节点的持久状态，因此定义在函数内。
  // surface_z 表示主要水平表面，low_z/high_z 和 height_span 用于识别竖直立面。
  struct StairCell {
    int x{0};
    int y{0};
    std::vector<std::size_t> point_indices;  // 回写原点云 labels 的索引，不是局部索引
    std::vector<double> heights;            // 本格所有点高度，排序后用于分位数统计
    std::size_t ground_count{0U};            // 初始地面标签点数
    std::size_t obstacle_count{0U};          // 初始非地面点数，包含 Unknown
    double surface_z{0.0};                  // 75% 分位高度，作为本格代表表面
    double low_z{0.0};                      // 10% 分位高度，不是绝对最低点
    double high_z{0.0};                     // 90% 分位高度，不是绝对最高点
    double height_span{0.0};                // high_z - low_z，区分平坦面与立面
    bool ground_seed{false};                // 严格地面种子：用于排除主地面候选
    bool ground_anchor{false};              // 较宽松几何地面锚点：用于接地判断
    bool candidate{false};                  // 仅具备候选资格，尚不能判为楼梯
  };

  // 将两个有符号栅格坐标的位模式打包成唯一键；floor 保证负坐标也正确分格。
  const auto make_key = [](int x, int y) -> std::uint64_t {
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x))
            << 32U) |
           static_cast<std::uint32_t>(y);
  };
  const auto grid_coordinate = [this](float value) -> int {
    return static_cast<int>(
        std::floor(static_cast<double>(value) / stair_grid_resolution_));
  };
  // 使用分位数而不是单个最低点或最高点，降低飞点对地面基准、踏面高度和
  // 立面高度跨度的影响。调用方需先对高度数组排序。
  const auto quantile = [](const std::vector<double>& sorted,
                           double ratio) -> double {
    if (sorted.empty()) return 0.0;
    const auto index = static_cast<std::size_t>(std::round(
        ratio * static_cast<double>(sorted.size() - 1U)));
    return sorted[std::min(index, sorted.size() - 1U)];
  };

  // 阶段 1：在水平面建格，保留每个点的索引和高度。地面标签也参与统计，
  // 因为地面分割可能已把一部分踏面标成 Ground，不能只从障碍点中找楼梯。
  std::unordered_map<std::uint64_t, StairCell> cells;
  cells.reserve(input_cloud->size());
  std::vector<double> reference_ground_heights;
  reference_ground_heights.reserve(input_cloud->size());

  for (std::size_t index = 0; index < input_cloud->size(); ++index) {
    const auto& point = input_cloud->points[index];
    const int cell_x = grid_coordinate(point.x);
    const int cell_y = grid_coordinate(point.y);
    auto& cell = cells[make_key(cell_x, cell_y)];
    cell.x = cell_x;
    cell.y = cell_y;
    cell.point_indices.push_back(index);
    cell.heights.push_back(point.z);
    if (labels[index] == TerrainLabel::Ground) {
      ++cell.ground_count;
      // base_link 原点位于地面，只用零高度附近的高置信度地面估计基准。
      // 这可防止被 PatchWork++ 误分为地面的高台和楼梯踏面抬高基准面。
      if (std::fabs(point.z) <= stair_ground_reference_max_abs_height_) {
        reference_ground_heights.push_back(point.z);
      }
    } else {
      ++cell.obstacle_count;
    }
  }

  // 地面退出视野不代表楼梯消失。立面序列与已有模型仍可独立工作；
  // 无近地面样本时沿用 base_link 位于地面的约定，基准暂取零。
  std::sort(reference_ground_heights.begin(), reference_ground_heights.end());
  const double reference_ground_z =
      reference_ground_heights.empty()
          ? 0.0
          : quantile(reference_ground_heights, 0.5);

  for (auto& entry : cells) {
    auto& cell = entry.second;
    std::sort(cell.heights.begin(), cell.heights.end());
    cell.low_z = quantile(cell.heights, 0.10);
    cell.high_z = quantile(cell.heights, 0.90);
    cell.surface_z = quantile(cell.heights, 0.75);
    cell.height_span = cell.high_z - cell.low_z;
    // 种子要求纯地面、点数足够、跨度小且接近基准；锚点只依据几何位置，
    // 即使初始地面分类不完整，踏面兜底路径仍有机会找到地面连接。
    cell.ground_seed =
        cell.ground_count >=
            static_cast<std::size_t>(stair_ground_seed_min_points_) &&
        cell.obstacle_count == 0U &&
        cell.height_span <= stair_ground_seed_max_span_ &&
        std::fabs(cell.surface_z - reference_ground_z) <
            stair_min_step_height_ * 0.5;
    cell.ground_anchor =
        !cell.heights.empty() &&
        cell.height_span <= std::max(0.08, stair_ground_seed_max_span_) &&
        std::fabs(cell.surface_z - reference_ground_z) <= 0.06;

    // 主地面不进入候选。高于或低于主地面的平台，以及仍被分割为障碍的
    // 低跨度单元，保留给完整楼梯结构验证。
    const bool separated_from_ground =
        std::fabs(cell.surface_z - reference_ground_z) >=
        stair_min_step_height_ * 0.5;
    cell.candidate =
        !cell.ground_seed &&
        cell.height_span <= stair_max_cell_height_span_ &&
        (separated_from_ground || cell.obstacle_count > 0U);
  }

  std::unordered_map<std::uint64_t, bool> visited;
  visited.reserve(cells.size());
  std::size_t candidate_components = 0U;
  std::size_t accepted_components = 0U;
  std::size_t accepted_riser_components = 0U;
  std::size_t rejected_no_ground = 0U;
  std::size_t rejected_few_levels = 0U;
  std::size_t rejected_total_rise = 0U;
  std::size_t rejected_step_height = 0U;
  std::size_t rejected_geometry = 0U;
  std::size_t point_model_fill = 0U;
  const int connection_radius = 1 + std::min(stair_max_gap_cells_, 2);

  // 阶段 2：立面检测主路径。这里的“立面”是同格高度有跨度、空间连续的
  // 点集，不是通过三维平面拟合得到的严格平面。搜索的是近似平行、级间
  // 高差一致、进深在允许范围内的序列；成功后也会更新成员中的楼梯模型。
  const auto detect_riser_sequence =
      [&](const std::vector<std::uint64_t>& component) {
        std::unordered_set<std::uint64_t> stair_keys;
        if (!use_stair_riser_detection_) return stair_keys;

        // 一段独立立面：top_z 为各格上沿的平均值，tangent 是 XY 内的横向。
        struct RiserLevel {
          double top_z{0.0};
          std::vector<std::uint64_t> keys;
          Eigen::Vector2d centroid{Eigen::Vector2d::Zero()};
          Eigen::Vector2d tangent{Eigen::Vector2d::UnitY()};
          bool has_tangent{false};
        };

        std::vector<std::uint64_t> vertical_keys;
        vertical_keys.reserve(component.size());
        // 单帧可能只扫到立面的一部分，因此跨度下限取最小级高的 70%。
        // 每格至少两个点、每段至少两个格，避免单点噪声直接形成一级立面。
        const double min_vertical_span = stair_min_step_height_ * 0.70;
        for (const auto key : component) {
          const auto& cell = cells.at(key);
          if (cell.point_indices.size() >= 2U &&
              cell.height_span >= min_vertical_span &&
              cell.height_span <= stair_max_cell_height_span_) {
            vertical_keys.push_back(key);
          }
        }
        if (vertical_keys.size() <
            static_cast<std::size_t>(stair_min_step_count_ * 2)) {
          return stair_keys;
        }

        std::unordered_set<std::uint64_t> vertical_set(vertical_keys.begin(),
                                                       vertical_keys.end());
        std::unordered_set<std::uint64_t> segmented;
        std::vector<RiserLevel> risers;
        // 先将同一高度、空间相邻的单元组成独立立面段。不同位置但高度
        // 相似的墙面或货架不能仅凭高度被合并成一个超宽立面。
        for (const auto start_key : vertical_keys) {
          if (segmented.count(start_key) > 0U) continue;
          RiserLevel riser;
          std::queue<std::uint64_t> open;
          open.push(start_key);
          segmented.insert(start_key);
          while (!open.empty()) {
            const auto key = open.front();
            open.pop();
            riser.keys.push_back(key);
            const auto& current = cells.at(key);
            for (int dx = -connection_radius; dx <= connection_radius; ++dx) {
              for (int dy = -connection_radius; dy <= connection_radius;
                   ++dy) {
                if (dx == 0 && dy == 0) continue;
                if (std::hypot(static_cast<double>(dx),
                               static_cast<double>(dy)) >
                    static_cast<double>(connection_radius)) {
                  continue;
                }
                const auto neighbor_key =
                    make_key(current.x + dx, current.y + dy);
                if (vertical_set.count(neighbor_key) == 0U ||
                    segmented.count(neighbor_key) > 0U) {
                  continue;
                }
                if (std::fabs(cells.at(neighbor_key).high_z - current.high_z) >
                    stair_step_height_tolerance_) {
                  continue;
                }
                segmented.insert(neighbor_key);
                open.push(neighbor_key);
              }
            }
          }
          // 单栅格无法可靠估计立面切向，容易把远处墙角和杆状物串成
          // 假楼梯。它不参与初始三级建模，但可在模型建立后被周期匹配。
          if (riser.keys.size() < 2U) continue;
          for (const auto key : riser.keys) {
            const auto& cell = cells.at(key);
            riser.top_z += cell.high_z;
            riser.centroid += Eigen::Vector2d(
                (static_cast<double>(cell.x) + 0.5) * stair_grid_resolution_,
                (static_cast<double>(cell.y) + 0.5) * stair_grid_resolution_);
          }
          riser.top_z /= static_cast<double>(riser.keys.size());
          riser.centroid /= static_cast<double>(riser.keys.size());
          // 对立面格中心做二维 PCA：最大特征值对应立面横向切线，
          // 其垂直方向作为楼梯进深方向，减少局部遮挡导致的质心偏移影响。
          Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
          for (const auto key : riser.keys) {
            const auto& cell = cells.at(key);
            const Eigen::Vector2d position(
                (static_cast<double>(cell.x) + 0.5) * stair_grid_resolution_,
                (static_cast<double>(cell.y) + 0.5) * stair_grid_resolution_);
            const Eigen::Vector2d offset = position - riser.centroid;
            covariance += offset * offset.transpose();
          }
          Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance);
          if (riser.keys.size() >= 2U && solver.info() == Eigen::Success &&
              solver.eigenvalues().maxCoeff() > 1e-8) {
            riser.tangent = solver.eigenvectors().col(1).normalized();
            riser.has_tangent = true;
          }
          risers.push_back(std::move(riser));
        }
        std::sort(risers.begin(), risers.end(),
                  [](const RiserLevel& lhs, const RiserLevel& rhs) {
                    return lhs.top_z < rhs.top_z;
                  });
        if (risers.size() < static_cast<std::size_t>(stair_min_step_count_)) {
          return stair_keys;
        }

        // 枚举起始两段，确定参考级高和进深方向，再贪心追加误差最小的段。
        // 优先保留级数多的序列；级数相同时取总高差大的序列。它不是全组合
        // 最优搜索，输入中仍可能存在未被选中的其他有效楼梯。
        std::vector<std::size_t> best_sequence;
        double best_rise = 0.0;
        for (std::size_t first = 0; first + 1U < risers.size(); ++first) {
          for (std::size_t second = first + 1U; second < risers.size();
               ++second) {
            const double expected_step =
                risers[second].top_z - risers[first].top_z;
            if (expected_step < stair_min_step_height_ ||
                expected_step > stair_max_step_height_) {
              continue;
            }
            const Eigen::Vector2d initial_delta =
                risers[second].centroid - risers[first].centroid;
            const double min_parallel_cos = std::cos(
                stair_direction_tolerance_deg_ *
                3.14159265358979323846 / 180.0);
            if (risers[first].has_tangent && risers[second].has_tangent &&
                std::fabs(risers[first].tangent.dot(
                              risers[second].tangent)) < min_parallel_cos) {
              continue;
            }
            // PCA 切线正负不固定，平行性用点积绝对值判断；进深方向则统一
            // 指向从低立面到高立面的方向，保证后面的 depth 为正向距离。
            Eigen::Vector2d expected_direction;
            if (risers[first].has_tangent) {
              expected_direction = Eigen::Vector2d(
                  -risers[first].tangent.y(), risers[first].tangent.x());
            } else if (risers[second].has_tangent) {
              expected_direction = Eigen::Vector2d(
                  -risers[second].tangent.y(), risers[second].tangent.x());
            } else {
              expected_direction = initial_delta.normalized();
            }
            if (initial_delta.dot(expected_direction) < 0.0) {
              expected_direction = -expected_direction;
            }
            const double expected_depth =
                initial_delta.dot(expected_direction);
            if (expected_depth < stair_min_tread_depth_ ||
                expected_depth > stair_max_tread_depth_) {
              continue;
            }
            std::vector<std::size_t> sequence{first, second};
            std::size_t last = second;
            while (last + 1U < risers.size()) {
              std::size_t best_next = risers.size();
              double best_error = std::numeric_limits<double>::max();
              for (std::size_t next = last + 1U; next < risers.size();
                   ++next) {
                const double step = risers[next].top_z - risers[last].top_z;
                const double error = std::fabs(step - expected_step);
                const Eigen::Vector2d delta =
                    risers[next].centroid - risers[last].centroid;
                const double depth = delta.dot(expected_direction);
                // 级高需落在范围内并接近参考级高，进深需落在允许区间；
                // 进深偏差参与误差评分，而非单独的等间距硬阈值。
                if (step >= stair_min_step_height_ &&
                    step <= stair_max_step_height_ &&
                    depth >= stair_min_tread_depth_ &&
                    depth <= stair_max_tread_depth_ &&
                    (!risers[next].has_tangent ||
                     !risers[first].has_tangent ||
                     std::fabs(risers[next].tangent.dot(
                                   risers[first].tangent)) >=
                         min_parallel_cos) &&
                    error <= stair_step_height_tolerance_ &&
                    error + 0.25 * std::fabs(depth - expected_depth) <
                        best_error) {
                  best_error =
                      error + 0.25 * std::fabs(depth - expected_depth);
                  best_next = next;
                }
              }
              if (best_next == risers.size()) break;
              sequence.push_back(best_next);
              last = best_next;
            }
            const double rise =
                risers[sequence.back()].top_z - risers[sequence.front()].top_z;
            if (sequence.size() > best_sequence.size() ||
                (sequence.size() == best_sequence.size() && rise > best_rise)) {
              best_sequence = std::move(sequence);
              best_rise = rise;
            }
          }
        }
        if (best_sequence.size() <
                static_cast<std::size_t>(stair_min_step_count_) ||
            best_rise < stair_min_total_rise_) {
          return stair_keys;
        }

        std::vector<RiserLevel> selected;
        selected.reserve(best_sequence.size());
        for (const auto index : best_sequence) selected.push_back(risers[index]);
        // 序列已满足最少级数、级高、进深、可用切线的平行度和总高约束；
        // 当前立面路径没有最小宽度硬门槛，宽度只用于限定后续模型覆盖范围。
        // 机器人靠近楼梯后，地面和最低一级经常退出视野；这里不再要求
        // 立面序列端点接地，避免清晰的上部三级楼梯被提前拒绝。
        Eigen::Vector2d stair_direction =
            (selected.back().centroid - selected.front().centroid).normalized();
        const auto tangent_source = std::find_if(
            selected.begin(), selected.end(),
            [](const RiserLevel& riser) { return riser.has_tangent; });
        if (tangent_source != selected.end()) {
          stair_direction = Eigen::Vector2d(-tangent_source->tangent.y(),
                                            tangent_source->tangent.x());
        }
        if ((selected.back().centroid - selected.front().centroid)
                .dot(stair_direction) < 0.0) {
          stair_direction = -stair_direction;
        }
        const Eigen::Vector2d width_direction(-stair_direction.y(),
                                              stair_direction.x());
        double min_width = std::numeric_limits<double>::max();
        double max_width = std::numeric_limits<double>::lowest();
        for (std::size_t index = 0; index < selected.size(); ++index) {
          auto& riser = selected[index];
          for (const auto key : riser.keys) {
            const auto& cell = cells.at(key);
            const Eigen::Vector2d position(
                (static_cast<double>(cell.x) + 0.5) * stair_grid_resolution_,
                (static_cast<double>(cell.y) + 0.5) * stair_grid_resolution_);
            const double lateral = position.dot(width_direction);
            min_width = std::min(min_width, lateral);
            max_width = std::max(max_width, lateral);
          }
        }

        // 三级立面在前面的序列搜索中已经通过级高、间距、方向和平行度
        // 检查。这里不再要求多数立面达到指定宽度，也不重复做几何复核，
        // 让单帧只扫到窄立面片段时仍可建立楼梯模型。

        for (const auto& riser : selected) {
          stair_keys.insert(riser.keys.begin(), riser.keys.end());
        }
        const double surface_span =
            std::max(stair_ground_seed_max_span_,
                     2.0 * stair_surface_tolerance_);
        const double model_margin = stair_grid_resolution_;
        // 阶段 3：用首尾观测拟合等间距模型。第 k 级上沿高度为 z0+k*h，
        // 立面纵向位置为 s0+k*d，踏面纵向区间为 [s0+k*d, s0+(k+1)*d]。
        // h 是平均级高，d 是沿上楼方向投影后的平均进深，k=0 对应最低
        // 被选立面，未必是实际楼梯第一级；外推只沿更高台阶方向进行。
        const double model_step_height =
            (selected.back().top_z - selected.front().top_z) /
            static_cast<double>(selected.size() - 1U);
        const double model_tread_depth =
            (selected.back().centroid - selected.front().centroid)
                .dot(stair_direction) /
            static_cast<double>(selected.size() - 1U);
        const double model_start_forward =
            selected.front().centroid.dot(stair_direction);
        const double model_start_height = selected.front().top_z;
        const int model_last_level =
            static_cast<int>(selected.size()) - 1 +
            stair_model_max_extrapolated_steps_;
        const double model_end_forward =
            model_start_forward +
            static_cast<double>(model_last_level + 1) * model_tread_depth;
        // 新序列直接替换旧模型；这里只保留一个模型，不做多楼梯跟踪或
        // 参数平滑。机器人靠近后最低可见级变化，模型起点也可能随之变化。
        stair_model_valid_ = true;
        stair_model_hold_frames_remaining_ = stair_hysteresis_frames_;
        stair_model_direction_ = stair_direction;
        stair_model_width_direction_ = width_direction;
        stair_model_start_forward_ = model_start_forward;
        stair_model_start_height_ = model_start_height;
        stair_model_step_height_ = model_step_height;
        stair_model_tread_depth_ = model_tread_depth;
        stair_model_min_width_ = min_width;
        stair_model_max_width_ = max_width;
        stair_model_last_level_ = model_last_level;
        if (current_cloud_to_base_valid_) {
          stair_model_cloud_to_base_ = current_cloud_to_base_;
          stair_model_cloud_frame_ = current_cloud_frame_;
        } else {
          stair_model_cloud_frame_.clear();
        }
        // 检测完成后重新检查整帧栅格，而不是只检查参与候选连接的单元。
        // 以已验证序列拟合平均级高和进深，并向上延伸有限级数。这样即使
        // 高处立面较稀疏、未进入初始三级序列，也能按周期模型补回。
        for (const auto& cell_entry : cells) {
          const auto key = cell_entry.first;
          const auto& cell = cell_entry.second;
          const Eigen::Vector2d position(
              (static_cast<double>(cell.x) + 0.5) * stair_grid_resolution_,
              (static_cast<double>(cell.y) + 0.5) * stair_grid_resolution_);
          const double forward = position.dot(stair_direction);
          const double lateral = position.dot(width_direction);
          if (forward < model_start_forward -
                            stair_model_riser_position_tolerance_ ||
              forward > model_end_forward +
                            stair_model_riser_position_tolerance_ ||
              lateral < min_width - model_margin ||
              lateral > max_width + model_margin) {
            continue;
          }
          if (cell.height_span > surface_span) {
            const int level = static_cast<int>(std::llround(
                (cell.high_z - model_start_height) / model_step_height));
            const double expected_height =
                model_start_height + level * model_step_height;
            const double expected_forward =
                model_start_forward + level * model_tread_depth;
            if (level >= 0 && level <= model_last_level &&
                std::fabs(forward - expected_forward) <=
                    stair_model_riser_position_tolerance_ &&
                std::fabs(cell.high_z - expected_height) <=
                    stair_model_tread_height_tolerance_ &&
                cell.low_z >= expected_height - stair_max_step_height_ -
                                  stair_model_tread_height_tolerance_) {
              stair_keys.insert(key);
            }
            continue;
          }
          const int level = static_cast<int>(std::llround(
              (cell.surface_z - model_start_height) / model_step_height));
          const double expected_height =
              model_start_height + level * model_step_height;
          const double tread_start =
              model_start_forward + level * model_tread_depth;
          const double tread_end = tread_start + model_tread_depth;
          if (level >= 0 && level <= model_last_level &&
              forward >= tread_start -
                             stair_model_riser_position_tolerance_ &&
              forward <= tread_end +
                             stair_model_riser_position_tolerance_ &&
              std::fabs(cell.surface_z - expected_height) <=
                  stair_model_tread_height_tolerance_) {
            stair_keys.insert(key);
          }
        }
        return stair_keys;
      };

  // 立面之间的真实进深通常大于普通栅格连接半径，因此必须跨越原来的
  // 候选组件搜索。完整序列本身负责级高、进深和方向验证；连续三级即可
  // 建立模型，模型会沿楼梯方向外推，用于覆盖更高但观测较稀疏的台阶。
  std::vector<std::uint64_t> all_candidate_keys;
  all_candidate_keys.reserve(cells.size());
  for (const auto& entry : cells) {
    if (entry.second.candidate) all_candidate_keys.push_back(entry.first);
  }
  auto global_riser_stair_keys =
      detect_riser_sequence(all_candidate_keys);
  if (global_riser_stair_keys.empty() && stair_model_valid_ &&
      stair_model_hold_frames_remaining_ > 0 &&
      stair_model_step_height_ > 0.0 && stair_model_tread_depth_ > 0.0) {
    // 阶段 3a：当前帧未能建立全局序列时，短期沿用旧模型。
    // 模型参数保存在建立时刻的 base_link 中。输入为 map/odom 配准点云时，
    // 用两帧 source->base 变换把模型更新到当前 base_link，补偿机器人靠近
    // 楼梯造成的整体平移和偏航，避免模型边缘重新落回障碍点云。
    // 此处是二维方向加高度偏移的近似补偿，未完整表达俯仰/横滚后的台阶面；
    // 后面的 retainStairSurfaces 使用独立的三维表面点记忆补充运动匹配。
    if (current_cloud_to_base_valid_ &&
        stair_model_cloud_frame_ == current_cloud_frame_) {
      const Eigen::Matrix4d relative =
          current_cloud_to_base_ * stair_model_cloud_to_base_.inverse();
      const Eigen::Matrix2d rotation = relative.block<2, 2>(0, 0);
      const Eigen::Vector2d translation(relative(0, 3), relative(1, 3));
      Eigen::Vector2d new_direction = rotation * stair_model_direction_;
      Eigen::Vector2d new_width_direction =
          rotation * stair_model_width_direction_;
      if (new_direction.norm() > 1e-6 &&
          new_width_direction.norm() > 1e-6) {
        new_direction.normalize();
        new_width_direction.normalize();
        stair_model_start_forward_ += translation.dot(new_direction);
        const double width_shift = translation.dot(new_width_direction);
        stair_model_min_width_ += width_shift;
        stair_model_max_width_ += width_shift;
        stair_model_start_height_ += relative(2, 3);
        stair_model_direction_ = new_direction;
        stair_model_width_direction_ = new_width_direction;
        stair_model_cloud_to_base_ = current_cloud_to_base_;
      }
    }
    const double surface_span =
        std::max(stair_ground_seed_max_span_, 2.0 * stair_surface_tolerance_);
    const double motion_margin =
        stair_model_riser_position_tolerance_ +
        stair_hysteresis_neighbor_cells_ * stair_grid_resolution_;
    const double height_margin =
        stair_model_tread_height_tolerance_ + stair_step_height_tolerance_;
    const double end_forward =
        stair_model_start_forward_ +
        static_cast<double>(stair_model_last_level_ + 1) *
            stair_model_tread_depth_;
    for (const auto& cell_entry : cells) {
      const auto key = cell_entry.first;
      const auto& cell = cell_entry.second;
      const Eigen::Vector2d position(
          (static_cast<double>(cell.x) + 0.5) * stair_grid_resolution_,
          (static_cast<double>(cell.y) + 0.5) * stair_grid_resolution_);
      const double forward = position.dot(stair_model_direction_);
      const double lateral = position.dot(stair_model_width_direction_);
      if (forward < stair_model_start_forward_ - motion_margin ||
          forward > end_forward + motion_margin ||
          lateral < stair_model_min_width_ - motion_margin ||
          lateral > stair_model_max_width_ + motion_margin) {
        continue;
      }
      const double cell_height =
          cell.height_span > surface_span ? cell.high_z : cell.surface_z;
      const int level = static_cast<int>(std::llround(
          (cell_height - stair_model_start_height_) /
          stair_model_step_height_));
      if (level < 0 || level > stair_model_last_level_) continue;
      const double expected_height =
          stair_model_start_height_ + level * stair_model_step_height_;
      const double expected_forward =
          stair_model_start_forward_ + level * stair_model_tread_depth_;
      if (std::fabs(cell_height - expected_height) > height_margin) continue;
      if (cell.height_span > surface_span) {
        if (std::fabs(forward - expected_forward) <= motion_margin &&
            cell.low_z >= expected_height - stair_max_step_height_ -
                              height_margin) {
          global_riser_stair_keys.insert(key);
        }
      } else if (forward >= expected_forward - motion_margin &&
                 forward <= expected_forward + stair_model_tread_depth_ +
                                motion_margin) {
        global_riser_stair_keys.insert(key);
      }
    }
    // 该模型寿命按处理帧递减，重新检测成功才重置。与表面记忆的“秒”不同，
    // 输入帧率改变会影响这里对应的实际保持时间。
    --stair_model_hold_frames_remaining_;
    if (stair_model_hold_frames_remaining_ <= 0) {
      stair_model_valid_ = false;
      stair_cell_hold_frames_.clear();
      stair_cell_surface_heights_.clear();
      stair_cell_height_spans_.clear();
    }
  }
  // 阶段 3b：在有限的模型范围内直接判断每个点，不依赖栅格连通性或整格高度统计。
  // 用拟合级高限制立面下沿，避免把相邻层之外的点也当成立面。
  const auto matches_stair_model = [&](const PointType& point) {
    if (!stair_model_valid_ || stair_model_step_height_ <= 0.0 ||
        stair_model_tread_depth_ <= 0.0 || !std::isfinite(point.x) ||
        !std::isfinite(point.y) || !std::isfinite(point.z)) return false;
    const Eigen::Vector2d position(point.x, point.y);
    const double forward = position.dot(stair_model_direction_);
    const double lateral = position.dot(stair_model_width_direction_);
    if (lateral < stair_model_min_width_ - stair_grid_resolution_ ||
        lateral > stair_model_max_width_ + stair_grid_resolution_) {
      return false;
    }

    // 踏面：先按高度找最近一级，再检查该级踏面的纵向区间。只按高度
    // 匹配会把同高度但位于楼梯前后其他位置的物体一起选入。
    const int tread_level = static_cast<int>(std::llround(
        (point.z - stair_model_start_height_) /
        stair_model_step_height_));
    bool matches_model = false;
    if (tread_level >= 0 && tread_level <= stair_model_last_level_) {
      const double tread_height =
          stair_model_start_height_ +
          tread_level * stair_model_step_height_;
      const double tread_start =
          stair_model_start_forward_ +
          tread_level * stair_model_tread_depth_;
      matches_model =
          std::fabs(point.z - tread_height) <=
              stair_model_tread_height_tolerance_ &&
          forward >= tread_start -
                         stair_model_riser_position_tolerance_ &&
          forward <= tread_start + stair_model_tread_depth_ +
                         stair_model_riser_position_tolerance_;
    }
    // 立面：先按纵向位置找最近一级，再检查从本级上沿向下一档的高度区间。
    // 横向宽度已在前面限制；超过模型级数、宽度或表面容差的点不在此处补全。
    if (!matches_model) {
      const int riser_level = static_cast<int>(std::llround(
          (forward - stair_model_start_forward_) /
          stair_model_tread_depth_));
      if (riser_level >= 0 &&
          riser_level <= stair_model_last_level_) {
        const double riser_forward =
            stair_model_start_forward_ +
            riser_level * stair_model_tread_depth_;
        const double riser_top =
            stair_model_start_height_ +
            riser_level * stair_model_step_height_;
        matches_model =
            std::fabs(forward - riser_forward) <=
                stair_model_riser_position_tolerance_ &&
            point.z >= riser_top - stair_model_step_height_ -
                           stair_model_tread_height_tolerance_ &&
            point.z <= riser_top +
                           stair_model_tread_height_tolerance_;
      }
    }
    return matches_model;
  };
  // 已验证栅格提供局部测量支持；理想等间距模型只负责补全，不能否决
  // 所有局部观测。逐点限制到该格的稳健高度范围，尝试保留明显高出的点。
  // 注意：这个范围来自当前格分位数；混合物体占比高时仍会污染统计值，
  // 因而该检查不等价于确认格内所有物体都可通行。
  const auto matches_stair_cell = [&](std::uint64_t key, std::size_t index) {
    const auto& cell = cells.at(key);
    const double z = input_cloud->points[index].z;
    return std::isfinite(z) &&
           z >= cell.low_z - stair_surface_tolerance_ &&
           z <= cell.high_z + stair_surface_tolerance_;
  };
  if (!global_riser_stair_keys.empty()) {
    for (const auto key : global_riser_stair_keys) {
      for (const auto index : cells.at(key).point_indices) {
        if (matches_stair_cell(key, index)) {
          labels[index] = TerrainLabel::TraversableStair;
        }
      }
    }
    ++accepted_components;
    ++accepted_riser_components;
  }

  if (stair_model_valid_) {
    for (std::size_t index = 0; index < input_cloud->size(); ++index) {
      if (labels[index] == TerrainLabel::TraversableStair ||
          !matches_stair_model(input_cloud->points[index])) continue;
      labels[index] = TerrainLabel::TraversableStair;
      ++point_model_fill;
    }
  }

  // 阶段 4：连通组件兜底。全局立面路径已写入的标签不会在这里被清除；
  // 其他组件仍尝试识别。组件通过接地检查后，必要时重试局部立面序列，
  // 再尝试由多个平坦高度层识别踏面，因此“未接地”日志主要来自这条路径。
  for (const auto& entry : cells) {
    if (!entry.second.candidate || visited[entry.first]) continue;

    std::queue<std::uint64_t> open;
    std::vector<std::uint64_t> component;
    bool touches_ground = false;
    open.push(entry.first);
    visited[entry.first] = true;

    while (!open.empty()) {
      const auto key = open.front();
      open.pop();
      component.push_back(key);
      const auto& current = cells.at(key);

      for (int dx = -connection_radius; dx <= connection_radius; ++dx) {
        for (int dy = -connection_radius; dy <= connection_radius; ++dy) {
          if (dx == 0 && dy == 0) continue;
          const int grid_distance = std::max(std::abs(dx), std::abs(dy));
          if (grid_distance > 1) {
            bool intermediate_occupied = false;
            for (int step = 1; step < grid_distance; ++step) {
              const int intermediate_x = current.x + static_cast<int>(
                  std::round(static_cast<double>(dx * step) / grid_distance));
              const int intermediate_y = current.y + static_cast<int>(
                  std::round(static_cast<double>(dy * step) / grid_distance));
              if (cells.find(make_key(intermediate_x, intermediate_y)) !=
                  cells.end()) {
                intermediate_occupied = true;
                break;
              }
            }
            // 只允许跨越完全没有回波的空格。中间存在墙、人或其他物体时
            // 必须沿实际相邻单元传播，不能从其上跨过去。
            if (intermediate_occupied) continue;
          }
          const auto neighbor_key = make_key(current.x + dx, current.y + dy);
          const auto neighbor_it = cells.find(neighbor_key);
          if (neighbor_it == cells.end()) continue;
          const auto& neighbor = neighbor_it->second;
          if (neighbor.ground_anchor) touches_ground = true;
          if (!neighbor.candidate || visited[neighbor_key]) continue;
          if (std::fabs(neighbor.surface_z - current.surface_z) >
              stair_max_step_height_ + stair_step_height_tolerance_) {
            continue;
          }
          visited[neighbor_key] = true;
          open.push(neighbor_key);
        }
      }
    }

    ++candidate_components;

    // 第一级踏面中心和地面边缘通常相隔一个踏面进深。稀疏点云中不能
    // 要求两者落在相邻5 cm栅格，只需在最大踏面进深内存在零高度锚点。
    if (!touches_ground) {
      const int ground_search_radius = static_cast<int>(std::ceil(
          stair_max_tread_depth_ / stair_grid_resolution_));
      for (const auto key : component) {
        const auto& candidate_cell = cells.at(key);
        for (int dx = -ground_search_radius;
             dx <= ground_search_radius && !touches_ground; ++dx) {
          for (int dy = -ground_search_radius; dy <= ground_search_radius;
               ++dy) {
            if (std::hypot(static_cast<double>(dx),
                           static_cast<double>(dy)) *
                    stair_grid_resolution_ >
                stair_max_tread_depth_) {
              continue;
            }
            const auto ground_it = cells.find(
                make_key(candidate_cell.x + dx, candidate_cell.y + dy));
            if (ground_it == cells.end() ||
                !ground_it->second.ground_anchor) {
              continue;
            }
            if (std::fabs(candidate_cell.surface_z -
                          ground_it->second.surface_z) <=
                stair_max_step_height_ + stair_step_height_tolerance_) {
              touches_ground = true;
              break;
            }
          }
        }
        if (touches_ground) break;
      }
    }

    if (!touches_ground || component.empty()) {
      ++rejected_no_ground;
      continue;
    }

    const auto riser_stair_keys =
        global_riser_stair_keys.empty()
            ? detect_riser_sequence(component)
            : std::unordered_set<std::uint64_t>{};
    if (!riser_stair_keys.empty()) {
      for (const auto key : riser_stair_keys) {
        for (const auto point_index : cells.at(key).point_indices) {
          if (matches_stair_cell(key, point_index)) {
            labels[point_index] = TerrainLabel::TraversableStair;
          }
        }
      }
      ++accepted_components;
      ++accepted_riser_components;
      continue;
    }

    // 踏面层与 RiserLevel 不同：这里只聚合跨度较小、代表高度接近的格。
    // z 是这一层各格 surface_z 的均值，centroid/width 用于后续几何验证。
    struct Level {
      double z{0.0};
      std::vector<std::uint64_t> keys;
      Eigen::Vector2d centroid{Eigen::Vector2d::Zero()};
      double width{0.0};
    };

    std::sort(component.begin(), component.end(), [&](std::uint64_t lhs,
                                                       std::uint64_t rhs) {
      return cells.at(lhs).surface_z < cells.at(rhs).surface_z;
    });
    std::vector<Level> levels;
    for (const auto key : component) {
      const auto& cell = cells.at(key);
      // 垂直立面用于连接上下踏面，但不能用其中间高度生成虚假的台阶层。
      const double max_surface_span =
          std::max(stair_ground_seed_max_span_,
                   2.0 * stair_surface_tolerance_);
      if (cell.height_span > max_surface_span) continue;
      if (levels.empty() ||
          std::fabs(cell.surface_z - levels.back().z) >
              stair_surface_tolerance_) {
        Level level;
        level.z = cell.surface_z;
        level.keys.push_back(key);
        levels.push_back(std::move(level));
      } else {
        auto& level = levels.back();
        const double count = static_cast<double>(level.keys.size());
        level.z = (level.z * count + cell.surface_z) / (count + 1.0);
        level.keys.push_back(key);
      }
    }

    // 很小的高度簇通常是立面散点或噪声，不作为独立踏面。
    levels.erase(std::remove_if(levels.begin(), levels.end(),
                                [](const Level& level) {
                                  return level.keys.size() < 2U;
                                }),
                 levels.end());
    if (levels.size() < static_cast<std::size_t>(stair_min_step_count_)) {
      ++rejected_few_levels;
      continue;
    }

    // 杂物、立面残点可能产生额外高度层。不要要求所有高度层都组成楼梯，
    // 而是在其中搜索级高最一致的最长子序列。
    std::vector<std::size_t> best_sequence;
    double best_sequence_rise = 0.0;
    for (std::size_t first = 0; first + 1U < levels.size(); ++first) {
      for (std::size_t second = first + 1U; second < levels.size(); ++second) {
        const double expected_step = levels[second].z - levels[first].z;
        if (expected_step < stair_min_step_height_ ||
            expected_step > stair_max_step_height_) {
          continue;
        }

        std::vector<std::size_t> sequence{first, second};
        std::size_t last = second;
        while (last + 1U < levels.size()) {
          std::size_t best_next = levels.size();
          double best_error = std::numeric_limits<double>::max();
          for (std::size_t next = last + 1U; next < levels.size(); ++next) {
            const double step = levels[next].z - levels[last].z;
            if (step < stair_min_step_height_ ||
                step > stair_max_step_height_) {
              continue;
            }
            const double error = std::fabs(step - expected_step);
            if (error <= stair_step_height_tolerance_ && error < best_error) {
              best_error = error;
              best_next = next;
            }
          }
          if (best_next == levels.size()) break;
          sequence.push_back(best_next);
          last = best_next;
        }

        const double rise = levels[sequence.back()].z - levels[sequence.front()].z;
        if (sequence.size() > best_sequence.size() ||
            (sequence.size() == best_sequence.size() &&
             rise > best_sequence_rise)) {
          best_sequence = std::move(sequence);
          best_sequence_rise = rise;
        }
      }
    }

    if (best_sequence.size() <
        static_cast<std::size_t>(stair_min_step_count_)) {
      ++rejected_step_height;
      continue;
    }
    if (best_sequence_rise < stair_min_total_rise_) {
      ++rejected_total_rise;
      continue;
    }

    std::vector<Level> selected_levels;
    selected_levels.reserve(best_sequence.size());
    for (const auto index : best_sequence) {
      selected_levels.push_back(std::move(levels[index]));
    }
    levels = std::move(selected_levels);

    // 仅踏面兜底路径要求高度序列有一端接近当前地面。这个约束能排除墙面、货架等
    // 在较高位置形成的规则水平条纹，同时兼容上楼和下楼两种方向。
    const double ground_end_tolerance =
        stair_max_step_height_ + stair_step_height_tolerance_;
    if (std::min(std::fabs(levels.front().z - reference_ground_z),
                 std::fabs(levels.back().z - reference_ground_z)) >
        ground_end_tolerance) {
      ++rejected_geometry;
      continue;
    }

    for (auto& level : levels) {
      for (const auto key : level.keys) {
        const auto& cell = cells.at(key);
        level.centroid += Eigen::Vector2d(
            (static_cast<double>(cell.x) + 0.5) * stair_grid_resolution_,
            (static_cast<double>(cell.y) + 0.5) * stair_grid_resolution_);
      }
      level.centroid /= static_cast<double>(level.keys.size());
    }

    Eigen::Vector2d stair_direction =
        levels.back().centroid - levels.front().centroid;
    if (stair_direction.norm() < stair_min_tread_depth_) {
      ++rejected_geometry;
      continue;
    }
    stair_direction.normalize();
    const Eigen::Vector2d width_direction(-stair_direction.y(),
                                          stair_direction.x());

    std::size_t sufficiently_wide_levels = 0U;
    std::size_t consistent_transitions = 0U;
    Eigen::Vector2d previous_delta = Eigen::Vector2d::Zero();
    const double min_direction_cos = std::cos(
        stair_direction_tolerance_deg_ * 3.14159265358979323846 / 180.0);
    for (std::size_t index = 0; index < levels.size(); ++index) {
      auto& level = levels[index];
      double min_width = std::numeric_limits<double>::max();
      double max_width = std::numeric_limits<double>::lowest();
      for (const auto key : level.keys) {
        const auto& cell = cells.at(key);
        const Eigen::Vector2d position(
            (static_cast<double>(cell.x) + 0.5) * stair_grid_resolution_,
            (static_cast<double>(cell.y) + 0.5) * stair_grid_resolution_);
        const double projection = position.dot(width_direction);
        min_width = std::min(min_width, projection);
        max_width = std::max(max_width, projection);
      }
      level.width = max_width - min_width + stair_grid_resolution_;
      if (level.width >= stair_min_width_) ++sufficiently_wide_levels;

      if (index == 0U) continue;
      const Eigen::Vector2d delta =
          level.centroid - levels[index - 1U].centroid;
      const double tread_depth = std::fabs(delta.dot(stair_direction));
      if (tread_depth >= stair_min_tread_depth_ &&
          tread_depth <= stair_max_tread_depth_ && delta.norm() > 0.0 &&
          delta.normalized().dot(stair_direction) >= min_direction_cos &&
          (previous_delta.norm() <= 0.0 ||
           delta.dot(previous_delta) > 0.0)) {
        ++consistent_transitions;
      }
      previous_delta = delta;
    }
    // 单帧稀疏点云经常只能扫到某一级踏面的一小段，不能要求每一级的
    // 宽度和中心都完美。至少 60% 的层和级间过渡满足几何条件即可。
    const auto required_majority = [](std::size_t count) {
      return (count * 3U + 4U) / 5U;
    };
    const bool valid_geometry =
        sufficiently_wide_levels >= required_majority(levels.size()) &&
        consistent_transitions >= required_majority(levels.size() - 1U);
    if (!valid_geometry) {
      ++rejected_geometry;
      continue;
    }

    // 只移动被已验证高度序列支持的踏面。与这些踏面直接相邻的低跨度
    // 立面一并移动，组件中相连的人、墙和其他高度簇继续保留为障碍。
    const double max_surface_span =
        std::max(stair_ground_seed_max_span_,
                 2.0 * stair_surface_tolerance_);
    std::unordered_set<std::uint64_t> tread_keys;
    for (const auto key : component) {
      const auto& cell = cells.at(key);
      if (cell.height_span > max_surface_span) continue;
      if (std::any_of(levels.begin(), levels.end(), [&](const Level& level) {
            return std::fabs(cell.surface_z - level.z) <=
                   stair_surface_tolerance_;
          })) {
        tread_keys.insert(key);
      }
    }
    auto stair_keys = tread_keys;
    for (const auto key : component) {
      const auto& cell = cells.at(key);
      if (cell.height_span <= max_surface_span ||
          cell.height_span > stair_max_cell_height_span_) {
        continue;
      }
      bool adjacent_to_tread = false;
      for (int dx = -1; dx <= 1 && !adjacent_to_tread; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          if (tread_keys.count(make_key(cell.x + dx, cell.y + dy)) > 0U) {
            adjacent_to_tread = true;
            break;
          }
        }
      }
      if (adjacent_to_tread) stair_keys.insert(key);
    }

    for (const auto key : stair_keys) {
      for (const auto point_index : cells.at(key).point_indices) {
        if (!matches_stair_cell(key, point_index)) continue;
        labels[point_index] = TerrainLabel::TraversableStair;
      }
    }
    ++accepted_components;
  }

  // 阶段 5：冻结当前结构/模型观测标签，随后进入两种历史保持。
  // 仅当前结构/模型确认的点可以更新表面记忆；历史补回的点不能续期。
  const auto observed_labels = labels;

  // 完整结构只需间歇性通过。这里保存的是栅格级短期历史，用来抵抗激光扫描
  // 疏密变化；它与上面的几何模型保持相互独立。后续帧必须同时满足位置邻近、
  // 高度接近并保持踏面/立面类型一致，才沿用历史标签，计数逐帧递减。
  // 栅格键仍处于各帧的 base_link 中，这里只通过邻格搜索容忍小位移，
  // 不执行完整 TF 变换；较大运动造成的漏匹配由后面的固定坐标系表面记忆补充。
  std::unordered_map<std::uint64_t, int> next_hold_frames;
  std::unordered_map<std::uint64_t, double> next_surface_heights;
  std::unordered_map<std::uint64_t, double> next_height_spans;
  next_hold_frames.reserve(stair_cell_hold_frames_.size());
  next_surface_heights.reserve(stair_cell_surface_heights_.size());
  next_height_spans.reserve(stair_cell_height_spans_.size());
  std::size_t hysteresis_cells = 0U;
  std::size_t hysteresis_points = 0U;
  const double history_max_surface_span =
      std::max(stair_ground_seed_max_span_, 2.0 * stair_surface_tolerance_);
  const double history_height_tolerance =
      std::max(2.0 * stair_surface_tolerance_,
               stair_step_height_tolerance_);

  for (const auto& entry : cells) {
    const auto key = entry.first;
    const auto& cell = entry.second;
    // 当前格只要有一个点已经得到楼梯标签，就刷新该格的历史统计；这里
    // 不会立即把同格其他点都改成楼梯。未直接命中的格才进入历史邻域搜索。
    const bool directly_detected = std::any_of(
        cell.point_indices.begin(), cell.point_indices.end(),
        [&](std::size_t point_index) {
          return labels[point_index] == TerrainLabel::TraversableStair;
        });

    if (directly_detected) {
      if (stair_hysteresis_frames_ > 0) {
        next_hold_frames[key] = stair_hysteresis_frames_;
        next_surface_heights[key] = cell.surface_z;
        next_height_spans[key] = cell.height_span;
      }
      continue;
    }
    if (stair_hysteresis_frames_ == 0 || !cell.candidate) {
      continue;
    }

    int best_hold = 0;
    for (int dx = -stair_hysteresis_neighbor_cells_;
         dx <= stair_hysteresis_neighbor_cells_; ++dx) {
      for (int dy = -stair_hysteresis_neighbor_cells_;
           dy <= stair_hysteresis_neighbor_cells_; ++dy) {
        const auto previous_key = make_key(cell.x + dx, cell.y + dy);
        const auto hold_it = stair_cell_hold_frames_.find(previous_key);
        const auto height_it = stair_cell_surface_heights_.find(previous_key);
        const auto span_it = stair_cell_height_spans_.find(previous_key);
        const bool current_is_surface =
            cell.height_span <= history_max_surface_span;
        if (hold_it == stair_cell_hold_frames_.end() ||
            height_it == stair_cell_surface_heights_.end() ||
            span_it == stair_cell_height_spans_.end() ||
            hold_it->second <= best_hold ||
            current_is_surface !=
                (span_it->second <= history_max_surface_span) ||
            std::fabs(cell.surface_z - height_it->second) >
                history_height_tolerance) {
          continue;
        }
        best_hold = hold_it->second;
      }
    }
    if (best_hold <= 0) continue;

    for (const auto point_index : cell.point_indices) {
      if (!matches_stair_cell(key, point_index)) continue;
      labels[point_index] = TerrainLabel::TraversableStair;
      ++hysteresis_points;
    }
    ++hysteresis_cells;
    if (best_hold > 1) {
      next_hold_frames[key] = best_hold - 1;
      next_surface_heights[key] = cell.surface_z;
      next_height_spans[key] = cell.height_span;
    }
  }
  stair_cell_hold_frames_.swap(next_hold_frames);
  stair_cell_surface_heights_.swap(next_surface_heights);
  stair_cell_height_spans_.swap(next_height_spans);

  // 放在栅格历史之后，防止一个记忆匹配点触发整格扩张。
  const std::size_t surface_history_points =
      retainStairSurfaces(input_cloud, observed_labels, labels);

  for (std::size_t index = 0; index < input_cloud->size(); ++index) {
    if (labels[index] == TerrainLabel::TraversableStair) {
      stair_cloud->push_back(input_cloud->points[index]);
    }
  }
  stair_cloud->width = static_cast<std::uint32_t>(stair_cloud->size());
  stair_cloud->height = 1;
  stair_cloud->is_dense = true;

  // 诊断计数并非统一的分类漏斗：候选/拒绝按连通组件统计，全局立面模型
  // 也会单独增加“通过”。“逐点补全”“表面记忆”分别表示对应路径新增点数，
  // “点”是最终楼梯总数；不能据此把“通过/候选”直接当作识别准确率。
  RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "楼梯结构识别: 地面基准=%.3f, 候选=%zu, 通过=%zu(立面=%zu), 点=%zu, 逐点补全=%zu, 表面记忆=%zu, 保持[格=%zu, 点=%zu], 拒绝[未接地=%zu, 层数=%zu, 总高=%zu, 级高=%zu, 几何=%zu]",
      reference_ground_z, candidate_components, accepted_components,
      accepted_riser_components, stair_cloud->size(), point_model_fill,
      surface_history_points, hysteresis_cells, hysteresis_points,
      rejected_no_ground, rejected_few_levels, rejected_total_rise,
      rejected_step_height, rejected_geometry);
  return stair_cloud;
}

// 表面记忆入口：observed_labels 是进入历史保持前的标签快照，labels 是当前
// 最终标签。前者决定哪些点能写入/刷新缓存，后者决定哪些点尚需补回。
// 历史保存真实观测到的表面点，而非整个楼梯包围盒。匹配使用完整三维 TF，
// 能处理平移、偏航、俯仰和横滚。只有结构检测/有效模型可以续期，历史命中不能。
std::size_t TerrainFilter::retainStairSurfaces(
    const pcl::PointCloud<PointType>::Ptr& cloud,
    const std::vector<TerrainLabel>& observed_labels,
    std::vector<TerrainLabel>& labels) {
  if (!current_cloud_to_base_valid_ || current_cloud_frame_.empty() ||
      current_cloud_stamp_ <= 0.0 ||
      !std::isfinite(stair_surface_history_duration_) ||
      !std::isfinite(stair_surface_history_distance_) ||
      stair_surface_history_duration_ <= 0.0 ||
      stair_surface_history_distance_ <= 0.0) {
    stair_surface_history_.clear();
    return 0U;
  }
  // bag 重新播放、时间倒退或输入坐标系改变时，不得沿用上一段历史。
  if (stair_surface_history_frame_ != current_cloud_frame_ ||
      current_cloud_stamp_ <= stair_surface_history_stamp_) {
    stair_surface_history_.clear();
  }
  stair_surface_history_frame_ = current_cloud_frame_;
  stair_surface_history_stamp_ = current_cloud_stamp_;
  // 寿命按点云时间戳计算，因此 bag 加速/减速播放不会改变数据时间上的
  // 保持长度。先删除过期样本，再建立查询树，过期点不能参与本帧匹配。
  for (auto it = stair_surface_history_.begin();
       it != stair_surface_history_.end();) {
    if (it->second.expires_at <= current_cloud_stamp_) {
      it = stair_surface_history_.erase(it);
    } else {
      ++it;
    }
  }

  // 调用约定：输入配准坐标系应为 map/odom 等固定参考系；代码无法仅凭
  // frame_id 判断任意坐标系是否固定。base_link 输入无运动参照，会被前面的
  // current_cloud_to_base_valid_ 检查禁用。source->base 取逆，将当前点还原到
  // 历史点所在的 source 中；保留完整三维旋转和平移，不使用二维栅格近似。
  const Eigen::Matrix4d base_to_source = current_cloud_to_base_.inverse();
  const auto fixed_cloud = transformPointCloud(cloud, base_to_source);
  pcl::PointCloud<PointType>::Ptr history(new pcl::PointCloud<PointType>);
  history->reserve(stair_surface_history_.size());
  for (const auto& entry : stair_surface_history_) {
    history->push_back(entry.second.point);
  }
  // 树只包含进入本帧前的历史样本。本帧新确认点写入缓存后，下帧才进入树，
  // 防止在同一帧沿近邻关系连续传播，把整片其他表面吸入楼梯。
  pcl::search::KdTree<PointType> tree;
  if (!history->empty()) tree.setInputCloud(history);
  std::vector<int> nearest(1);
  std::vector<float> distances(1);
  const double max_distance_sq = stair_surface_history_distance_ *
                                 stair_surface_history_distance_;
  std::size_t retained = 0U;
  for (std::size_t index = 0; index < fixed_cloud->size(); ++index) {
    const auto& point = fixed_cloud->points[index];
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) continue;
    // PCL 返回的是距离平方，须与阈值平方比较。只补回非地面、非楼梯点，
    // 不改动已确认地面。默认 4 cm 是到离散历史点的距离，不是到拟合平面的距离；
    // 扫描间隙较大时仍可能漏匹配，阈值内的微小物体也可能与表面噪声混淆。
    if (labels[index] != TerrainLabel::TraversableStair &&
        labels[index] != TerrainLabel::Ground && !history->empty() &&
        tree.nearestKSearch(point, 1, nearest, distances) > 0 &&
        distances[0] <= max_distance_sq) {
      labels[index] = TerrainLabel::TraversableStair;
      ++retained;
    }
    // 使用快照而非刚修改过的 labels，确保历史命中不能自我续期。
    // 有效几何模型本身确认的点仍可续期，因此表面记忆与模型保持时间可叠加。
    if (observed_labels[index] != TerrainLabel::TraversableStair) continue;
    // 3 cm 体素每格只保存一个样本，限制重复观测造成的缓存增长。
    constexpr double resolution = 0.03;
    const auto key = std::make_tuple(
        static_cast<int>(std::floor(point.x / resolution)),
        static_cast<int>(std::floor(point.y / resolution)),
        static_cast<int>(std::floor(point.z / resolution)));
    stair_surface_history_[key] = {
        point, current_cloud_stamp_ + stair_surface_history_duration_};
  }
  return retained;
}

// 根据最终标签一次性重建三个互斥输出。楼梯点不会再次进入 obstacle_cloud；
// 未分类点按障碍保留，防止感知异常时把真实障碍静默删除。
void TerrainFilter::buildClassifiedClouds(
    const pcl::PointCloud<PointType>::Ptr& input_cloud,
    const std::vector<TerrainLabel>& labels,
    pcl::PointCloud<PointType>::Ptr& ground_cloud,
    pcl::PointCloud<PointType>::Ptr& obstacle_cloud,
    pcl::PointCloud<PointType>::Ptr& stair_cloud) const {
  ground_cloud->clear();
  obstacle_cloud->clear();
  stair_cloud->clear();
  ground_cloud->reserve(input_cloud->size());
  obstacle_cloud->reserve(input_cloud->size());
  stair_cloud->reserve(input_cloud->size());

  for (std::size_t index = 0; index < input_cloud->size(); ++index) {
    const TerrainLabel label =
        index < labels.size() ? labels[index] : TerrainLabel::Unknown;
    if (label == TerrainLabel::Ground) {
      ground_cloud->push_back(input_cloud->points[index]);
    } else if (label == TerrainLabel::TraversableStair) {
      stair_cloud->push_back(input_cloud->points[index]);
    } else {
      // Unknown 也按障碍处理，避免分类遗漏导致真实物体消失。
      obstacle_cloud->push_back(input_cloud->points[index]);
    }
  }

  const auto finalize = [](pcl::PointCloud<PointType>::Ptr& cloud) {
    cloud->width = static_cast<std::uint32_t>(cloud->size());
    cloud->height = 1;
    cloud->is_dense = true;
  };
  finalize(ground_cloud);
  finalize(obstacle_cloud);
  finalize(stair_cloud);
}

void TerrainFilter::publishResults(
    const pcl::PointCloud<PointType>::Ptr& combined_cloud,
    const pcl::PointCloud<PointType>::Ptr& ground_cloud,
    const pcl::PointCloud<PointType>::Ptr& obstacle_cloud,
    const pcl::PointCloud<PointType>::Ptr& stair_cloud,
    const std_msgs::msg::Header& header) {
  // 所有输入点已在处理开始时转换到 base_link，因此输出必须使用同一坐标系；
  // 时间戳仍沿用传感器采集时间，供下游进行时序匹配。
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

  // 始终发布楼梯点云（包括空点云），便于调试端清除上一帧结果。
  if (stair_cloud_pub_->get_subscription_count() > 0) {
    sensor_msgs::msg::PointCloud2 stair_msg;
    pcl::toROSMsg(*stair_cloud, stair_msg);
    stair_msg.header = base_link_header;
    stair_cloud_pub_->publish(stair_msg);
  }

  // 分割成功但当前没有障碍时也要发布空点云，使代价地图和停障节点能够
  // 清除上一帧障碍，而不是继续沿用陈旧结果。
  if (obstacle_cloud_pub_->get_subscription_count() > 0) {
    sensor_msgs::msg::PointCloud2 obstacle_msg;
    pcl::toROSMsg(*obstacle_cloud, obstacle_msg);
    obstacle_msg.header = base_link_header;
    obstacle_cloud_pub_->publish(obstacle_msg);
  }

  RCLCPP_DEBUG(node_->get_logger(),
               "发布完成: 合并点云 %zu 点, 地面点云 %zu 点, 楼梯点云 %zu 点, 障碍物点云 %zu 点",
               combined_cloud->points.size(), ground_cloud->points.size(),
               stair_cloud->points.size(), obstacle_cloud->points.size());
}

// 欧式聚类仅用于删除规模小于 min_cluster_size_ 的孤立噪点。楼梯剔除已由
// 标签完成，此处不再依据高度或形状分类，避免误删低矮障碍和大型连通障碍。
pcl::PointCloud<PointType>::Ptr TerrainFilter::filterObstacleCloud(
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
  // 先提取所有连通簇，再由下面的逻辑只过滤小簇。PCL 会直接丢弃超过
  // MaxClusterSize 的整簇；楼梯与站在楼梯上的人一旦连成大簇，就会导致
  // 两者同时从 obstacle_cloud 消失，这是不可接受的安全行为。
  ec.setMinClusterSize(1);
  ec.setMaxClusterSize(std::numeric_limits<int>::max());
  ec.setSearchMethod(tree);
  ec.setInputCloud(obstacle_cloud);
  ec.extract(cluster_indices);

  pcl::PointCloud<PointType>::Ptr filtered_cloud(
      new pcl::PointCloud<PointType>);
  filtered_cloud->points.reserve(obstacle_cloud->points.size());

  std::size_t small_cluster_points = 0U;
  std::size_t oversized_cluster_count = 0U;

  for (const auto& cluster : cluster_indices) {
    const auto cluster_size = cluster.indices.size();
    if (cluster_size < static_cast<std::size_t>(min_cluster_size_)) {
      small_cluster_points += cluster_size;
      continue;
    }

    // max_cluster_size 仅作为异常大簇的诊断阈值，绝不能作为删除真实障碍物
    // 的条件。大墙、楼梯以及与楼梯接触的人都可能形成很大的有效点簇。
    if (max_cluster_size_ > 0 &&
        cluster_size > static_cast<std::size_t>(max_cluster_size_)) {
      ++oversized_cluster_count;
    }

    for (const auto& idx : cluster.indices) {
      filtered_cloud->points.push_back(obstacle_cloud->points[idx]);
    }
  }

  filtered_cloud->width =
      static_cast<std::uint32_t>(filtered_cloud->points.size());
  filtered_cloud->height = 1;
  filtered_cloud->is_dense = true;

  const std::size_t filtered_size = filtered_cloud->points.size();

  RCLCPP_DEBUG(node_->get_logger(), "障碍物点云聚类滤波完成:");
  RCLCPP_DEBUG(node_->get_logger(), "  原始点云: %zu 点", original_size);
  RCLCPP_DEBUG(node_->get_logger(), "  滤波后点云: %zu 点", filtered_size);
  RCLCPP_DEBUG(node_->get_logger(), "  移除小簇孤立点: %zu 点",
               small_cluster_points);
  RCLCPP_DEBUG(node_->get_logger(), "  有效聚类数: %zu 个",
               cluster_indices.size());
  RCLCPP_DEBUG(
      node_->get_logger(),
      "  聚类参数: 容忍距离 %.3f 米, 最小大小 %d 点, 大簇诊断阈值 %d 点",
      cluster_tolerance_, min_cluster_size_, max_cluster_size_);

  if (oversized_cluster_count > 0U) {
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "检测到 %zu 个超过 %d 点的大障碍簇，已为安全起见完整保留",
        oversized_cluster_count, max_cluster_size_);
  }

  return filtered_cloud;
}
