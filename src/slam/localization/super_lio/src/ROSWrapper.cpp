
#include "ros/ROSWrapper.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include <chrono>
#include <algorithm>
#include <stdexcept>


using namespace BASIC;

namespace LI2Sup{

void LoadParamFromRos(rclcpp::Node& node)
{
  node.declare_parameter<bool>("lio.eva.timer", false);
  node.get_parameter("lio.eva.timer", g_time_eva);

  node.declare_parameter<std::string>("lio.ros.lidar_topic", "/lidar");
  node.get_parameter("lio.ros.lidar_topic", g_lidar_topic);

  node.declare_parameter<std::string>("lio.ros.imu_topic", "/imu");
  node.get_parameter("lio.ros.imu_topic", g_imu_topic);

  node.declare_parameter<int>("lio.sensor.lidar_type", 0);
  node.get_parameter("lio.sensor.lidar_type", g_lidar_type);

  double temp_range_dis;
  node.declare_parameter<double>("lio.sensor.blind", 0.0);
  node.get_parameter("lio.sensor.blind", temp_range_dis);
  g_blind2 = temp_range_dis * temp_range_dis;

  node.declare_parameter<double>("lio.sensor.maxrange", 100.0);
  node.get_parameter("lio.sensor.maxrange", temp_range_dis);
  g_maxrange2 = temp_range_dis * temp_range_dis;

  node.declare_parameter<int>("lio.sensor.filter_rate", 1);
  node.get_parameter("lio.sensor.filter_rate", g_filter_rate);

  node.declare_parameter<bool>("lio.sensor.enable_downsample", false);
  node.get_parameter("lio.sensor.enable_downsample", g_enable_downsample);

  node.declare_parameter<double>("lio.sensor.voxel_fliter_size", 0.2);
  node.get_parameter("lio.sensor.voxel_fliter_size", g_voxel_fliter_size);

  node.declare_parameter<double>("lio.sensor.gravity_norm", 9.81);
  node.get_parameter("lio.sensor.gravity_norm", g_gravity_norm);

  node.declare_parameter<double>("lio.sensor.imu_na", 0.0);
  node.get_parameter("lio.sensor.imu_na", g_imu_na);

  node.declare_parameter<double>("lio.sensor.imu_ng", 0.0);
  node.get_parameter("lio.sensor.imu_ng", g_imu_ng);

  node.declare_parameter<double>("lio.sensor.imu_nba", 0.0);
  node.get_parameter("lio.sensor.imu_nba", g_imu_nba);

  node.declare_parameter<double>("lio.sensor.imu_nbg", 0.0);
  node.get_parameter("lio.sensor.imu_nbg", g_imu_nbg);

  // ================= extrinsic =================
  std::vector<double> extrinsic_lidar_imu;
  node.declare_parameter<std::vector<double>>(
      "lio.extrinsic.lidar_imu", std::vector<double>(12, 0.0));
  node.get_parameter("lio.extrinsic.lidar_imu", extrinsic_lidar_imu);

  V3 __t(extrinsic_lidar_imu[0],
         extrinsic_lidar_imu[1],
         extrinsic_lidar_imu[2]);
  std::vector<scalar> r_data(9);
  for (int i = 0; i < 9; ++i) {
    r_data[i] = static_cast<scalar>(extrinsic_lidar_imu[3 + i]);
  }
  M3 __R(r_data.data());
  g_lidar_imu = SE3(__R, __t);

  std::vector<double> extrinsic_odom_robo;
  node.declare_parameter<std::vector<double>>(
      "lio.extrinsic.odom_robo", std::vector<double>(6, 0.0));
  node.get_parameter("lio.extrinsic.odom_robo", extrinsic_odom_robo);

  __t = V3(extrinsic_odom_robo[0],
           extrinsic_odom_robo[1],
           extrinsic_odom_robo[2]);

  auto temp_R =
      Eigen::AngleAxisd(extrinsic_odom_robo[5] * M_PI / 180.0,
                          Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(extrinsic_odom_robo[4] * M_PI / 180.0,
                          Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(extrinsic_odom_robo[3] * M_PI / 180.0,
                          Eigen::Vector3d::UnitX());

  g_odom_robo.R_ = temp_R.cast<scalar>();
  g_odom_robo.R_ = g_odom_robo.R_.transpose().eval();
  g_odom_robo = SE3(g_odom_robo.R_, __t);

  auto temp_R_yaw =
      Eigen::AngleAxisd(extrinsic_odom_robo[5] * M_PI / 180.0,
                        Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  g_lidar_robo_yaw = temp_R_yaw.cast<scalar>();

  // ================= hash map =================
  node.declare_parameter<int>("lio.hash_map.hash_capacity", 1000000);
  node.get_parameter("lio.hash_map.hash_capacity", g_ivox_capacity);

  node.declare_parameter<double>("lio.hash_map.vox_resolution", 0.5);
  node.get_parameter("lio.hash_map.vox_resolution", g_ivox_resolution);

  // kf
  node.declare_parameter<int>("lio.kf.kf_max_iterations", 0);
  node.get_parameter("lio.kf.kf_max_iterations", g_kf_max_iterations);

  node.declare_parameter<bool>("lio.kf.kf_align_gravity", false);
  node.get_parameter("lio.kf.kf_align_gravity", g_kf_align_gravity);

  node.declare_parameter<double>("lio.kf.kf_quit_eps", 0.0);
  node.get_parameter("lio.kf.kf_quit_eps", g_kf_quit_eps);

  // visual
  node.declare_parameter<bool>("lio.output.robot", false);
  node.get_parameter("lio.output.robot", g_2_robot);

  node.declare_parameter<bool>("lio.output.map", false);
  node.get_parameter("lio.output.map", g_visual_map);

  node.declare_parameter<bool>("lio.output.dense", false);
  node.get_parameter("lio.output.dense", g_visual_dense);

  node.declare_parameter<int>("lio.output.pub_step", 0);
  node.get_parameter("lio.output.pub_step", g_pub_step);

  LOG(INFO) << GREEN << " ---> [Params]: Load from ROS2 parameter server."
            << RESET;
}


void livox2pcl(const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg, CloudPtr& point_cloud){
  point_cloud->clear();
  CloudPtr cloud_full(new PointCloudType());
  int plsize = msg->point_num;
  cloud_full->resize(plsize);
  point_cloud->reserve(plsize);
  std::vector<bool> is_valid_pt(plsize, false);
  std::vector<std::size_t> index(plsize - 1);
  std::iota(std::begin(index), std::end(index), 1);

  std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const uint &i) {
    if((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00)
    {
      // if (i % g_filter_rate == 0) 
      {
        cloud_full->at(i).x = msg->points[i].x;
        cloud_full->at(i).y = msg->points[i].y;
        cloud_full->at(i).z = msg->points[i].z;
        cloud_full->at(i).intensity = msg->points[i].reflectivity;

        if ((abs(cloud_full->at(i).x - cloud_full->at(i - 1).x) > 1e-7) ||
            (abs(cloud_full->at(i).y - cloud_full->at(i - 1).y) > 1e-7) ||
            (abs(cloud_full->at(i).z - cloud_full->at(i - 1).z) > 1e-7))
        {
          double normal_dis = cloud_full->at(i).x * cloud_full->at(i).x + 
                              cloud_full->at(i).y * cloud_full->at(i).y +
                              cloud_full->at(i).z * cloud_full->at(i).z;
          if(normal_dis > g_blind2 and normal_dis < g_maxrange2){
            is_valid_pt[i] = true;
          }
        }
      }
    }
  });

  for (int i = 1; i < plsize; i++) {
    if (is_valid_pt[i]) {
      point_cloud->points.push_back(cloud_full->at(i));
    }
  }
}


std::string lidarTypeToString(int type) {
  if (type <= 0 || type >= static_cast<int>(LID_TYPE_NAMES.size())) return "UNKNOWN";
  return LID_TYPE_NAMES[type];
}


inline bool validPoint(double x, double y, double z)
{
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
    return false;

  double d2 = x * x + y * y + z * z;
  return (d2 > g_blind2 && d2 < g_maxrange2);
}


inline double stampToSec(const builtin_interfaces::msg::Time& t)
{
  return static_cast<double>(t.sec) +
         static_cast<double>(t.nanosec) * 1e-9;
}


inline builtin_interfaces::msg::Time toRosTime(double t_sec)
{
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<int32_t>(std::floor(t_sec));
  t.nanosec = static_cast<uint32_t>((t_sec - t.sec) * 1e9);
  return t;
}


ROSWrapper::ROSWrapper(const rclcpp::NodeOptions& options)
: rclcpp::Node("super_lio_localization", options)
{
  LoadParamFromRos(*this);
  LOG(INFO) << GREEN << " ---> Using Lidar type: "
            << lidarTypeToString(g_lidar_type) << RESET;

  setup_localization_interface();
  msg2uav_.header.frame_id = map_frame_;
  path_.header.frame_id = map_frame_;

  setupIO();
}


void ROSWrapper::setupIO(){
  //// input ======================================
  cb_sensor_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions sub_opt;
  sub_opt.callback_group = cb_sensor_;

  auto imu_qos = rclcpp::QoS(rclcpp::KeepLast(500))
                 .best_effort()
                 .durability_volatile();

  auto lidar_qos = rclcpp::QoS(rclcpp::KeepLast(20))
                   .best_effort()
                   .durability_volatile();

  // Serialize the watchdog with sensor/processing callbacks. Use wall elapsed
  // time so paused bag playback or /clock cannot keep a stale success alive.
  status_timer_ = create_wall_timer(std::chrono::milliseconds(100),
      [this]() { publish_status(localization_valid_); }, cb_sensor_);

  sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      g_imu_topic,
      imu_qos,
      std::bind(&ROSWrapper::imuHandler, this, std::placeholders::_1),
      sub_opt);

  if (g_lidar_type == LID_TYPE::LIVOX) {
    sub_lidar_ =
        this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            g_lidar_topic,
            lidar_qos,
            std::bind(&ROSWrapper::livoxHandler, this, std::placeholders::_1),
            sub_opt);
  } else {
    sub_lidar_std_ =
        this->create_subscription<sensor_msgs::msg::PointCloud2>(
            g_lidar_topic,
            lidar_qos,
            std::bind(&ROSWrapper::stdMsgHandler, this, std::placeholders::_1),
            sub_opt);
  }

  /// output ======================================
  pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/lio/odom", 100);

  /// Same as fastlio_localization's /lidar_localization: base_link pose in world.
  pub_lidar_localization_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/lidar_localization", 100);

  pub_imu_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/lio/imu/odom", 10);

  pub_robo_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/lio/robo/odom", 10);

  pub_path_ = this->create_publisher<nav_msgs::msg::Path>(
      "/imu_path", 1000);

  pub_cloud_world_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/cloud_registered", 1000);

  tf_broadcaster_ =
      std::make_shared<tf2_ros::TransformBroadcaster>(this);

}


void ROSWrapper::imuHandler(const sensor_msgs::msg::Imu::SharedPtr msg){
  last_imu_received_ = std::chrono::steady_clock::now();
  IMUData data;
  data.secs = stampToSec(msg->header.stamp);
  data.acc  = V3(msg->linear_acceleration.x,
                 msg->linear_acceleration.y,
                 msg->linear_acceleration.z);
  data.gyr  = V3(msg->angular_velocity.x,
                 msg->angular_velocity.y,
                 msg->angular_velocity.z);

  if (data.secs < last_timestamp_imu_) {
    LOG(WARNING) << "imu loop back, clear buffer";
    imu_buffer_.clear();
    imu_buffer_.push_back(data);
    last_timestamp_imu_ = data.secs;
    // eskf_->Reset();   // todo:
    return;
  }

  imu_buffer_.push_back(data);
  last_timestamp_imu_ = data.secs;

  DynamicState imu_state, robo_state;
  if(eskf_->Predict(data, imu_state, robo_state)){
    nav_msgs::msg::Odometry odom_imu, odom_robo;

    {
      odom_imu.pose.pose.position.x = imu_state.p(0);
      odom_imu.pose.pose.position.y = imu_state.p(1);
      odom_imu.pose.pose.position.z = imu_state.p(2);

      Quat q(imu_state.R);
      q.normalize();

      odom_imu.pose.pose.orientation.x = q.x();
      odom_imu.pose.pose.orientation.y = q.y();
      odom_imu.pose.pose.orientation.z = q.z();
      odom_imu.pose.pose.orientation.w = q.w();

      odom_imu.twist.twist.linear.x = imu_state.v(0);
      odom_imu.twist.twist.linear.y = imu_state.v(1);
      odom_imu.twist.twist.linear.z = imu_state.v(2);

      odom_imu.twist.twist.angular.x = imu_state.w(0);
      odom_imu.twist.twist.angular.y = imu_state.w(1);
      odom_imu.twist.twist.angular.z = imu_state.w(2);
    }

    {
      odom_robo.pose.pose.position.x = robo_state.p(0);
      odom_robo.pose.pose.position.y = robo_state.p(1);
      odom_robo.pose.pose.position.z = robo_state.p(2);

      Quat q(robo_state.R);
      q.normalize();

      odom_robo.pose.pose.orientation.x = q.x();
      odom_robo.pose.pose.orientation.y = q.y();
      odom_robo.pose.pose.orientation.z = q.z();
      odom_robo.pose.pose.orientation.w = q.w();
    }

    odom_imu.header.stamp = msg->header.stamp;
    odom_robo.header.stamp = msg->header.stamp;
    odom_imu.header.frame_id = map_frame_;
    odom_robo.header.frame_id = map_frame_;
    odom_imu.child_frame_id = "imu";
    odom_robo.child_frame_id = "base_link";
    pub_imu_odom_->publish(odom_imu);
    pub_robo_odom_->publish(odom_robo);
    publish_imu_localization(imu_state);
  }
}


void ROSWrapper::livoxHandler(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg){
  if(msg->point_num < 10) return;
  LidarData lidar_data;
  std::size_t ptsize = msg->point_num;
  lidar_data.pc.reset(new pcl::PointCloud<LI2Sup::PointXTZIT>());
  lidar_data.pc->reserve(ptsize / g_filter_rate + 1);

  double offset_time = 0.0;
  for(std::size_t _i = 0; _i < ptsize; _i += g_filter_rate){
    auto& pt = msg->points[_i];
    auto tag = pt.tag & 0x30;
    if (tag == 0x10 || tag == 0x00){
      auto dis = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
      if(dis > g_blind2 && dis < g_maxrange2){
        offset_time = pt.offset_time * 1e-9;
        lidar_data.pc->emplace_back(pt.x, pt.y, pt.z, pt.reflectivity, offset_time);
      }
    }
  }
  lidar_data.start_time = stampToSec(msg->header.stamp);
  lidar_data.end_time   = lidar_data.start_time + offset_time;
  if (lidar_data.pc->empty()) return;
  last_lidar_received_ = std::chrono::steady_clock::now();
  lidar_buffer_.push_back(lidar_data);
}


void ROSWrapper::stdMsgHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
  if(msg->data.size() < 10) return;
  
  LidarData lidar_data;
  lidar_data.pc.reset(new pcl::PointCloud<LI2Sup::PointXTZIT>());

  double offset_time = 0.0;
  double dis = 0.0;

  switch (g_lidar_type) {

  case LID_TYPE::HESAI16:
  {
    pcl::PointCloud<hesai_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    lidar_data.pc->reserve(pl_orig.size() / g_filter_rate + 1);
    const double time_begin = pl_orig.points[0].timestamp;
    lidar_data.start_time = time_begin;
    for(std::size_t i = 0; i < pl_orig.size(); i += g_filter_rate)
    {
      auto& pt = pl_orig.points[i];
      if (!validPoint(pt.x, pt.y, pt.z)) continue;
      offset_time = pt.timestamp - time_begin;
      lidar_data.pc->emplace_back(
          pt.x, pt.y, pt.z, pt.intensity, offset_time);
    }
    lidar_data.end_time = time_begin + offset_time;
    break;
  }
  case LID_TYPE::VEL_NCLT:
  {
    pcl::PointCloud<NCLT::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    lidar_data.pc->reserve(pl_orig.size() / g_filter_rate + 1);
    lidar_data.start_time = stampToSec(msg->header.stamp);
    
    for(std::size_t i = 0; i < pl_orig.size(); i += g_filter_rate){
      auto& pt = pl_orig.points[i];
      if (!validPoint(pt.x, pt.y, pt.z)) continue;
      offset_time = pt.time * 1e-6;
      lidar_data.pc->emplace_back(
          pt.x, pt.y, pt.z, 1.0, offset_time);
    }
    lidar_data.end_time = lidar_data.start_time + offset_time;
    break;
  }
  case LID_TYPE::VELO16:
  case LID_TYPE::VELO32:
  {
    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    lidar_data.pc->reserve(pl_orig.size() / g_filter_rate + 1);
    lidar_data.start_time = stampToSec(msg->header.stamp);

    for(std::size_t i = 0; i < pl_orig.size(); i += g_filter_rate){
      auto& pt = pl_orig.points[i];
      if (!validPoint(pt.x, pt.y, pt.z)) continue;
      lidar_data.pc->emplace_back(
          pt.x, pt.y, pt.z, pt.intensity, pt.time);
    }
    lidar_data.end_time = lidar_data.start_time + lidar_data.pc->points.back().offset_time;
    break;
  }
  case OUSTER:
  {
    pcl::PointCloud<ouster_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    lidar_data.pc->reserve(pl_orig.size() / g_filter_rate + 1);
    lidar_data.start_time = stampToSec(msg->header.stamp);

    for(std::size_t i = 0; i < pl_orig.size(); i += g_filter_rate){
      auto& pt = pl_orig.points[i];
      if (!validPoint(pt.x, pt.y, pt.z)) continue;
      offset_time = pt.t * 1e-9;
      lidar_data.pc->emplace_back(
          pt.x, pt.y, pt.z, pt.intensity, offset_time);
    }
    lidar_data.end_time = lidar_data.start_time + offset_time;
    break;
  }
  default:
    return;
  }
  
  if (lidar_data.pc->empty()) return;
  last_lidar_received_ = std::chrono::steady_clock::now();
  lidar_buffer_.push_back(lidar_data);
}


bool ROSWrapper::sync_measure(MeasureGroup& meas){
  if (lidar_buffer_.empty() || imu_buffer_.empty()) {
    return false;
  }

  if (!lidar_pushed_) {
    meas.lidar = lidar_buffer_.front();
    lidar_pushed_ = true;
  }

  if(last_timestamp_lidar_ > meas.lidar.end_time){
    lidar_buffer_.pop_front();
    lidar_pushed_ = false;
    return false;
  }

  if (last_timestamp_imu_ < meas.lidar.end_time) {
    return false;
  }

  double imu_time = imu_buffer_.front().secs;
  meas.imu.clear();
  while ((!imu_buffer_.empty()) && (imu_time < meas.lidar.end_time)) {
    imu_time = imu_buffer_.front().secs;
    if (imu_time > meas.lidar.end_time) break;
    meas.imu.push_back(imu_buffer_.front());
    imu_buffer_.pop_front();
  }

  last_timestamp_lidar_ = meas.lidar.end_time;
  lidar_buffer_.pop_front();
  lidar_pushed_ = false;
  return true;
}


void ROSWrapper::pub_cloud_world(const CloudPtr& pc, double time){
  sensor_msgs::msg::PointCloud2 cloud;
  pcl::toROSMsg(*pc, cloud);
  cloud.header.frame_id = map_frame_;
  cloud.header.stamp = toRosTime(time);
  pub_cloud_world_->publish(cloud);
}

void ROSWrapper::pub_processing_time(double time, 
  double current_time, double mean_time, double std_time)
{
  if (!processing_time_pub_) processing_time_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/lio/processing_time", 10);
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = toRosTime(time);
  msg.pose.position.x = current_time;
  msg.pose.position.y = mean_time;
  msg.pose.position.z = std_time;
  processing_time_pub_->publish(msg);
}

void ROSWrapper::set_gnss_data()
{
  if (!use_gnss_) {
    LOG(INFO) << YELLOW << " ---> GNSS prior disabled (relocalization.use_gnss = false)." << RESET;
    return;
  }

  if (use_gnss_ && !sub_gnss_) {
    // The relocalization pipeline is compiled with `gnss_with_direction`, so the
    // prior must carry an orientation and be expressed in the map frame. That is
    // nav_msgs/Odometry, not a raw NavSatFix (lat/lon/alt would be fed as metres).
    sub_gnss_ = this->create_subscription<nav_msgs::msg::Odometry>(
        gnss_topic_, 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          const auto &pos = msg->pose.pose.position;
          const auto &ori = msg->pose.pose.orientation;
          const double stamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1.0e-9;
          std::lock_guard<std::mutex> lock(backend_.relocalization->gnss_mutex);
          backend_.relocalization->gnss_pose =
              GnssPose(stamp, V3D(pos.x, pos.y, pos.z), QD(ori.w, ori.x, ori.y, ori.z));
        });
    LOG(INFO) << YELLOW << " ---> GNSS prior enabled, topic: " << gnss_topic_
              << " (expects nav_msgs/Odometry in the map frame)" << RESET;
  }
}

void ROSWrapper::load_backend_param(Backend &backend)
{
  this->declare_parameter("map_path", "");
  this->declare_parameter("scan_context.lidar_height", 2.0);
  this->declare_parameter("scan_context.sc_dist_thres", 0.5);

  // ---------------------------------------------------------------- relocalization
  this->declare_parameter("relocalization.algorithm_type", std::string("bbs3d"));
  this->declare_parameter("relocalization.use_gnss", false);
  this->declare_parameter("relocalization.gnss_topic", std::string("/gnss/odom"));
  this->declare_parameter("relocalization.voxelmap_auto_build", true);

  // ---- BBS3D (global search, CPU/OpenMP) ----
  this->declare_parameter("bbs3d.min_level_res", 0.5);
  this->declare_parameter("bbs3d.max_level", 6);
  this->declare_parameter("bbs3d.gravity_align", true);
  this->declare_parameter("bbs3d.backend", std::string("cpu"));
  this->declare_parameter("bbs3d.opencl_batch_size", 10000);
  this->declare_parameter("bbs3d.opencl_fallback_cpu", false);
  this->declare_parameter("bbs3d.opencl_verify", false);
  this->declare_parameter("bbs3d.min_rpy", vector<double>{-0.1, -0.1, 0.0});
  this->declare_parameter("bbs3d.max_rpy", vector<double>{0.1, 0.1, 6.28});
  this->declare_parameter("bbs3d.score_threshold_percentage", 0.9);
  this->declare_parameter("bbs3d.num_threads", 6);
  this->declare_parameter("bbs3d.tar_leaf_size", 0.0);
  this->declare_parameter("bbs3d.src_leaf_size", 2.0);
  this->declare_parameter("bbs3d.min_scan_range", 0.0);
  this->declare_parameter("bbs3d.max_scan_range", 100.0);
  this->declare_parameter("bbs3d.timeout_msec", 0);
  this->declare_parameter("bbs3d.use_gicp", false);

  const auto profile = declare_parameter<std::string>("relocalization_profile", "initial");
  if (profile != "initial" && profile != "map_connection")
    throw std::invalid_argument("Unknown relocalization_profile: " + profile);
  const std::string bnb_prefix = "bnb3d." + profile + ".";
  // ---- BnB3D window refinement ----
  this->declare_parameter(bnb_prefix + "linear_xy_window_size", 8.0);
  this->declare_parameter(bnb_prefix + "linear_z_window_size", 0.5);
  this->declare_parameter(bnb_prefix + "angular_search_window", 30.0);  // degree
  this->declare_parameter(bnb_prefix + "pc_resolutions", vector<double>{0.3, 0.5, 0.8});
  this->declare_parameter(bnb_prefix + "bnb_depth", 3);
  this->declare_parameter(bnb_prefix + "min_score", 0.1);
  this->declare_parameter(bnb_prefix + "enough_score", 0.8);
  this->declare_parameter(bnb_prefix + "min_xy_resolution", 0.2);
  this->declare_parameter(bnb_prefix + "min_z_resolution", 0.1);
  this->declare_parameter(bnb_prefix + "min_angular_resolution", 5.0);  // degree
  this->declare_parameter(bnb_prefix + "filter_size_scan", 2.0);
  this->declare_parameter(bnb_prefix + "debug_mode", false);

  // ---- NDT / GICP fine alignment ----
  this->declare_parameter("ndt.step_size", 0.1);
  this->declare_parameter("ndt.resolution", 1.0);
  this->declare_parameter("gicp.use_gicp", true);
  this->declare_parameter("gicp.filter_range", 80.0);
  this->declare_parameter("gicp.downsample", 0.2);
  this->declare_parameter("gicp.search_radius", 0.5);
  this->declare_parameter("gicp.teps", 1e-3);
  this->declare_parameter("gicp.feps", 1e-3);
  this->declare_parameter("gicp.fitness_score", 0.3);

  this->get_parameter("map_path", backend.map_path);
  if (backend.map_path.compare("") != 0)
  {
    backend.globalmap_path = backend.map_path + "/globalmap.pcd";
    backend.trajectory_path = backend.map_path + "/trajectory.pcd";
    backend.scd_path = backend.map_path + "/scancontext/";
  }
  else
    backend.map_path = PCD_FILE_DIR("");

  this->get_parameter("scan_context.lidar_height", backend.relocalization->sc_manager->LIDAR_HEIGHT);
  this->get_parameter("scan_context.sc_dist_thres", backend.relocalization->sc_manager->SC_DIST_THRES);

  // ---------------------------------------------------------------- relocalization
  this->get_parameter("relocalization.algorithm_type", backend.relocalization->algorithm_type);
  this->get_parameter("relocalization.use_gnss", use_gnss_);
  this->get_parameter("relocalization.gnss_topic", gnss_topic_);
  this->get_parameter("relocalization.voxelmap_auto_build", backend.voxelmap_auto_build);
  backend.relocalization->gnss_enable = use_gnss_;

  // lidar extrinsic for relocalization: reuse the already-parsed g_lidar_imu (T_imu<-lidar)
  {
    const M3D R_lidar_d = g_lidar_imu.R_.matrix().cast<double>();
    const V3D rpy = EigenMath::RotationMatrix2RPY(R_lidar_d);
    Pose lidar_extrinsic;
    lidar_extrinsic.x = g_lidar_imu.t_.x();
    lidar_extrinsic.y = g_lidar_imu.t_.y();
    lidar_extrinsic.z = g_lidar_imu.t_.z();
    lidar_extrinsic.roll = RAD2DEG(rpy.x());
    lidar_extrinsic.pitch = RAD2DEG(rpy.y());
    lidar_extrinsic.yaw = RAD2DEG(rpy.z());

    BnbOptions match_option;
    this->get_parameter(bnb_prefix + "linear_xy_window_size", match_option.linear_xy_window_size);
    this->get_parameter(bnb_prefix + "linear_z_window_size", match_option.linear_z_window_size);
    this->get_parameter(bnb_prefix + "angular_search_window", match_option.angular_search_window);
    this->get_parameter(bnb_prefix + "pc_resolutions", match_option.pc_resolutions);
    this->get_parameter(bnb_prefix + "bnb_depth", match_option.bnb_depth);
    this->get_parameter(bnb_prefix + "min_score", match_option.min_score);
    this->get_parameter(bnb_prefix + "enough_score", match_option.enough_score);
    this->get_parameter(bnb_prefix + "min_xy_resolution", match_option.min_xy_resolution);
    this->get_parameter(bnb_prefix + "min_z_resolution", match_option.min_z_resolution);
    this->get_parameter(bnb_prefix + "min_angular_resolution", match_option.min_angular_resolution);
    this->get_parameter(bnb_prefix + "filter_size_scan", match_option.filter_size_scan);
    this->get_parameter(bnb_prefix + "debug_mode", match_option.debug_mode);
    // bnb3d.h asserts pc_resolutions.size() >= bnb_depth; clamp instead of aborting.
    if (match_option.pc_resolutions.size() < static_cast<size_t>(match_option.bnb_depth))
    {
      LOG(ERROR) << YELLOW << " ---> bnb3d.pc_resolutions.size() ("
                 << match_option.pc_resolutions.size() << ") < bnb3d.bnb_depth ("
                 << match_option.bnb_depth << "), clamping bnb_depth." << RESET;
      match_option.bnb_depth = static_cast<int>(match_option.pc_resolutions.size());
    }
    backend.relocalization->set_bnb3d_param(match_option, lidar_extrinsic);

    double ndt_step = 0.1, ndt_res = 1.0;
    this->get_parameter("ndt.step_size", ndt_step);
    this->get_parameter("ndt.resolution", ndt_res);
    backend.relocalization->set_ndt_param(ndt_step, ndt_res);

    bool use_gicp = true;
    double gicp_filter_range = 80.0, gicp_ds = 0.2, gicp_search = 0.5;
    double gicp_teps = 1e-3, gicp_feps = 1e-3, gicp_fitness = 0.3;
    this->get_parameter("gicp.use_gicp", use_gicp);
    this->get_parameter("gicp.filter_range", gicp_filter_range);
    this->get_parameter("gicp.downsample", gicp_ds);
    this->get_parameter("gicp.search_radius", gicp_search);
    this->get_parameter("gicp.teps", gicp_teps);
    this->get_parameter("gicp.feps", gicp_feps);
    this->get_parameter("gicp.fitness_score", gicp_fitness);
    backend.relocalization->set_gicp_param(use_gicp, gicp_filter_range, gicp_ds, gicp_search,
                                           gicp_teps, gicp_feps, gicp_fitness);
  }

  // ---------------------------------------------------------------- 3D-BBS options
  this->get_parameter("bbs3d.min_level_res", backend.bbs3d_option.min_level_res);
  this->get_parameter("bbs3d.max_level", backend.bbs3d_option.max_level);
  this->get_parameter("bbs3d.gravity_align", backend.bbs3d_option.gravity_align);
  this->get_parameter("bbs3d.backend", backend.bbs3d_option.backend);
  this->get_parameter("bbs3d.opencl_batch_size", backend.bbs3d_option.opencl_batch_size);
  this->get_parameter("bbs3d.opencl_fallback_cpu", backend.bbs3d_option.opencl_fallback_cpu);
  this->get_parameter("bbs3d.opencl_verify", backend.bbs3d_option.opencl_verify);
  this->get_parameter("bbs3d.min_rpy", backend.bbs3d_option.min_rpy);
  this->get_parameter("bbs3d.max_rpy", backend.bbs3d_option.max_rpy);
  this->get_parameter("bbs3d.score_threshold_percentage", backend.bbs3d_option.score_threshold_percentage);
  this->get_parameter("bbs3d.num_threads", backend.bbs3d_option.num_threads);
  this->get_parameter("bbs3d.tar_leaf_size", backend.bbs3d_option.tar_leaf_size);
  this->get_parameter("bbs3d.src_leaf_size", backend.bbs3d_option.src_leaf_size);
  this->get_parameter("bbs3d.min_scan_range", backend.bbs3d_option.min_scan_range);
  this->get_parameter("bbs3d.max_scan_range", backend.bbs3d_option.max_scan_range);
  this->get_parameter("bbs3d.timeout_msec", backend.bbs3d_option.timeout_msec);
  this->get_parameter("bbs3d.use_gicp", backend.bbs3d_option.use_gicp);

  LOG(WARNING) << YELLOW << " ---> Relocalization:"
            << " algorithm_type=" << backend.relocalization->algorithm_type
            << " map_path=" << backend.map_path
            << " use_gnss=" << (use_gnss_ ? 1 : 0)
            << " voxelmap_auto_build=" << (backend.voxelmap_auto_build ? 1 : 0)
            << " bbs3d(min_level_res=" << backend.bbs3d_option.min_level_res
            << ", max_level=" << backend.bbs3d_option.max_level
            << ", score_thres=" << backend.bbs3d_option.score_threshold_percentage
            << ", threads=" << backend.bbs3d_option.num_threads
            << ", scan_range=[" << backend.bbs3d_option.min_scan_range << ", "
            << backend.bbs3d_option.max_scan_range << "])"
            << " bnb3d(min_score=" << backend.relocalization->bnb_option.min_score
            << ", enough_score=" << backend.relocalization->bnb_option.enough_score << ")"
            << RESET;

  this->declare_parameter("publish.print_pose", false);  // echo world pose to terminal (debug)
  this->get_parameter("publish.print_pose", print_pose_);
}

} // namespace END.

namespace LI2Sup {
namespace {
builtin_interfaces::msg::Time stamp_from_seconds(double seconds) {
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(std::floor(seconds));
  stamp.nanosec = static_cast<uint32_t>((seconds - stamp.sec) * 1e9);
  return stamp;
}
void set_pose(geometry_msgs::msg::Pose &pose, const Eigen::Matrix4d &matrix) {
  const Eigen::Quaterniond q(matrix.topLeftCorner<3, 3>());
  pose.position.x = matrix(0, 3);
  pose.position.y = matrix(1, 3);
  pose.position.z = matrix(2, 3);
  pose.orientation.x = q.x(); pose.orientation.y = q.y();
  pose.orientation.z = q.z(); pose.orientation.w = q.w();
}
}

void ROSWrapper::setup_localization_interface() {
  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  lidar_frame_ = declare_parameter<std::string>("lidar_frame", "lidar");
  baselink_frame_ = declare_parameter<std::string>("baselink_frame", "base_link");
  path_en_ = declare_parameter<bool>("path_en", true);
  g_visual_map = declare_parameter<bool>("scan_publish_en", g_visual_map);
  g_visual_dense = declare_parameter<bool>("dense_publish_en", g_visual_dense);
  lidar_tf_broadcast_ = declare_parameter<bool>("lidar_tf_broadcast", true);
  imu_tf_broadcast_ = declare_parameter<bool>("imu_tf_broadcast", false);
  sensor_timeout_sec_ = declare_parameter<double>("sensor_timeout_sec", 1.0);
  if (!std::isfinite(sensor_timeout_sec_) || sensor_timeout_sec_ <= 0.0)
    throw std::invalid_argument("sensor_timeout_sec must be finite and positive");
  const auto t = declare_parameter<std::vector<double>>("lidar_to_base_T", {0., 0., 0.});
  const auto r = declare_parameter<std::vector<double>>("lidar_to_base_R", {0., 0., 0.});
  if (t.size() != 3 || r.size() != 3)
    throw std::invalid_argument("lidar_to_base_T/R must contain three values");
  for (const auto *values : {&t, &r})
    for (double value : *values)
      if (!std::isfinite(value)) throw std::invalid_argument("Nonfinite LiDAR/base extrinsic");
  T_base_lidar_.topLeftCorner<3, 3>() =
      (Eigen::AngleAxisd(r[2], Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(r[1], Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(r[0], Eigen::Vector3d::UnitX())).toRotationMatrix();
  T_base_lidar_.topRightCorner<3, 1>() = Eigen::Vector3d(t[0], t[1], t[2]);
  // Keep high-rate ESKF robot output consistent with the same rigid transform.
  const Eigen::Matrix4d imu_base = g_lidar_imu.matrix().cast<double>() * T_base_lidar_.inverse();
  const BASIC::M3 imu_base_rotation = imu_base.topLeftCorner<3, 3>().cast<BASIC::scalar>();
  const BASIC::V3 imu_base_translation = imu_base.topRightCorner<3, 1>().cast<BASIC::scalar>();
  g_odom_robo = BASIC::SE3(imu_base_rotation, imu_base_translation);

  reloc_status_pub_ = create_publisher<std_msgs::msg::Bool>("/reloc_success", 1);
  robot_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/pose_stamped_localization", 1000);
  app_pose_pub_ = create_publisher<slam_manager_interfaces::msg::LocalizationPose>("/app/localization_pose", 20);
  imu_localization_pub_ = create_publisher<nav_msgs::msg::Odometry>("/imu_localization", 1000);
  cloud_body_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_body", 20);
  global_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/global_map", 1);
  publish_status(false);
}

Eigen::Matrix4d ROSWrapper::base_pose(const Eigen::Matrix4d &map_imu) const {
  return localization_geometry::baseFromImuPose(
      map_imu, g_lidar_imu.matrix().cast<double>(), T_base_lidar_);
}

bool ROSWrapper::sensors_fresh() const {
  const auto now = std::chrono::steady_clock::now();
  const auto fresh = [&](const auto &received) {
    return received != std::chrono::steady_clock::time_point{} &&
        std::chrono::duration<double>(now - received).count() < sensor_timeout_sec_;
  };
  return fresh(last_imu_received_) && fresh(last_lidar_received_);
}

void ROSWrapper::publish_status(bool valid) {
  valid = valid && sensors_fresh();
  localization_valid_ = valid;
  std_msgs::msg::Bool status; status.data = valid;
  reloc_status_pub_->publish(status);
}

void ROSWrapper::publish_base_tf(const Eigen::Matrix4d &pose, double time) {
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = map_frame_; tf.child_frame_id = baselink_frame_;
  tf.header.stamp = stamp_from_seconds(time);
  tf.transform.translation.x = pose(0, 3);
  tf.transform.translation.y = pose(1, 3);
  tf.transform.translation.z = pose(2, 3);
  const Eigen::Quaterniond q(pose.topLeftCorner<3, 3>());
  tf.transform.rotation.x = q.x(); tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z(); tf.transform.rotation.w = q.w();
  tf_broadcaster_->sendTransform(tf);
}

void ROSWrapper::pub_odom(const NavState &state, double match_score) {
  const Eigen::Matrix4d map_imu = state.GetSE3().matrix().cast<double>();
  if (!map_imu.allFinite()) { publish_status(false); return; }
  nav_msgs::msg::Odometry imu;
  imu.header.frame_id = map_frame_; imu.child_frame_id = "imu";
  imu.header.stamp = stamp_from_seconds(state.timestamp);
  set_pose(imu.pose.pose, map_imu);
  pub_odom_->publish(imu);

  const Eigen::Matrix4d map_base = base_pose(map_imu);
  nav_msgs::msg::Odometry odom;
  odom.header = imu.header; odom.child_frame_id = baselink_frame_;
  set_pose(odom.pose.pose, map_base);
  pub_lidar_localization_->publish(odom);
  geometry_msgs::msg::PoseStamped robot;
  robot.header = odom.header; robot.pose = odom.pose.pose;
  robot_pose_pub_->publish(robot);
  slam_manager_interfaces::msg::LocalizationPose app;
  app.pose = robot;
  app.match_score = static_cast<float>(std::clamp(match_score, 0.0, 1.0));
  app_pose_pub_->publish(app);
  publish_status(true);
  if (lidar_tf_broadcast_) publish_base_tf(map_base, state.timestamp);
  if (path_en_) {
    path_.header = robot.header;
    path_.poses.push_back(robot);
    if (path_.poses.size() > 300) path_.poses.erase(path_.poses.begin());
    pub_path_->publish(path_);
  }
  if (print_pose_)
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "base position: %.3f %.3f %.3f", map_base(0,3), map_base(1,3), map_base(2,3));
}

void ROSWrapper::publish_imu_localization(const DynamicState &state) {
  if (!localization_valid_) return;
  Eigen::Matrix4d map_imu = Eigen::Matrix4d::Identity();
  map_imu.topLeftCorner<3, 3>() = state.R.cast<double>();
  map_imu.topRightCorner<3, 1>() = state.p.cast<double>();
  if (!map_imu.allFinite()) return;
  const Eigen::Matrix4d map_base = base_pose(map_imu);
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = map_frame_; odom.child_frame_id = baselink_frame_;
  odom.header.stamp = stamp_from_seconds(state.time);
  set_pose(odom.pose.pose, map_base);
  imu_localization_pub_->publish(odom);
  if (imu_tf_broadcast_) publish_base_tf(map_base, state.time);
}

void ROSWrapper::pub_cloud_body(const BASIC::CloudPtr &pc, double time) {
  if (!g_visual_map || cloud_body_pub_->get_subscription_count() == 0) return;
  BASIC::CloudPtr lidar_cloud(new BASIC::PointCloudType());
  pcl::transformPointCloud(*pc, *lidar_cloud, g_lidar_imu.inverse().matrix().cast<float>());
  sensor_msgs::msg::PointCloud2 cloud;
  pcl::toROSMsg(*lidar_cloud, cloud);
  cloud.header.frame_id = lidar_frame_; cloud.header.stamp = stamp_from_seconds(time);
  cloud_body_pub_->publish(cloud);
}

void ROSWrapper::set_global_map(const BASIC::CloudPtr &global_map) {
  pcl::toROSMsg(*global_map, global_map_msg_);
  global_map_msg_.header.frame_id = map_frame_;
  global_map_timer_ = create_wall_timer(std::chrono::seconds(2), [this]() {
    global_map_msg_.header.stamp = now();
    global_map_pub_->publish(global_map_msg_);
  });
}

void ROSWrapper::set_initial_data(bool &flg_get_init_guess, bool flg_finish_init) {
  if (flg_finish_init || init_pose_sub_) return;
  rclcpp::SubscriptionOptions options;
  options.callback_group = cb_sensor_;
  init_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 1, [this, &flg_get_init_guess](
          geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        const auto &q = msg->pose.pose.orientation;
        Eigen::Quaterniond rotation(q.w, q.x, q.y, q.z);
        const auto &p = msg->pose.pose.position;
        if (!rotation.coeffs().allFinite() || rotation.norm() <= 1e-6 ||
            !Eigen::Vector3d(p.x, p.y, p.z).allFinite()) {
          RCLCPP_ERROR(get_logger(), "Ignoring invalid /initialpose"); return;
        }
        if (!msg->header.frame_id.empty() && msg->header.frame_id != map_frame_) {
          RCLCPP_ERROR(get_logger(), "/initialpose must be expressed in %s", map_frame_.c_str()); return;
        }
        Eigen::Matrix4d map_base = Eigen::Matrix4d::Identity();
        map_base.topLeftCorner<3,3>() = rotation.normalized().toRotationMatrix();
        map_base.topRightCorner<3,1>() = Eigen::Vector3d(p.x,p.y,p.z);
        const Eigen::Matrix4d map_imu = localization_geometry::imuFromBasePose(
            map_base, g_lidar_imu.matrix().cast<double>(), T_base_lidar_);
        Pose pose;
        EigenMath::DecomposeAffineMatrix(map_imu, pose.x, pose.y, pose.z,
                                        pose.roll, pose.pitch, pose.yaw);
        backend_.relocalization->request_manual_pose(pose);
        flg_get_init_guess = true;
        manual_pose_requested_ = true;
      }, options);
}
} // namespace LI2Sup
