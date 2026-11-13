
#include "ros/ROSWrapper.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_msgs/msg/string.hpp"

#include <chrono>

bool showOptimizedPose = true;
double globalMapVisualizationSearchRadius = 1000;
double globalMapVisualizationPoseDensity = 10;
double globalMapVisualizationLeafSize = 1;
std::thread visualizeMapThread;
extern Backend backend;
bool save_globalmap_en = false;
bool save_pgm = false;
double pgm_resolution;
float min_z, max_z;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubGlobalmap;
rclcpp::Subscription<std_msgs::msg::String>::SharedPtr key_sub;

void publish_cloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubCloud,
                   PointCloudType::Ptr cloud, const std::string &frame_id)
{
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud, cloud_msg);
  cloud_msg.header.stamp = rclcpp::Clock().now();
  cloud_msg.header.frame_id = frame_id;
  pubCloud->publish(cloud_msg);
}

void visualize_globalmap_thread(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubGlobalmap)
{
  while (rclcpp::ok())
  {
    this_thread::sleep_for(std::chrono::seconds(1));
    auto submap_visual = backend.get_submap_visual(globalMapVisualizationSearchRadius,
                                                   globalMapVisualizationPoseDensity,
                                                   globalMapVisualizationLeafSize,
                                                   showOptimizedPose);
    if (submap_visual == nullptr)
      continue;
    publish_cloud(pubGlobalmap, submap_visual, "world");
  }
}

void keyCallback(const std_msgs::msg::String::ConstPtr &msg)
{
  if (msg->data.empty())
    return;

  char key = msg->data[0];

  switch (key)
  {
  case 'z':
    if (!backend.backend->z_axis_constraint_enable)
    {
      backend.backend->z_axis_constraint_enable = true;
      backend.backend->update_height = true;
      LOG_WARN("backend.backend->z_axis_constraint_enable = true");
    }
    else
    {
      backend.backend->z_axis_constraint_enable = false;
      backend.backend->update_height = true;
      LOG_WARN("backend.backend->z_axis_constraint_enable = false");
    }
    break;
  case 's':
    if (!backend.backend->update_height)
    {
      backend.backend->update_height = true;
      LOG_WARN("backend.backend->update_height = true");
    }
    else
    {
      backend.backend->update_height = false;
      LOG_WARN("backend.backend->update_height = false");
    }
    break;
  default:
    break;
  }
}

using namespace BASIC;

namespace LI2Sup{

void LoadParamFromRos(rclcpp::Node& node)
{
  node.declare_parameter<bool>("lio.map.save_map", false);
  node.get_parameter("lio.map.save_map", g_save_map);

  LOG(INFO) << GREEN << " ---> [Param] map/save_map: "
            << (g_save_map ? "true" : "false") << RESET;

  node.declare_parameter<bool>("lio.eva.timer", false);
  node.get_parameter("lio.eva.timer", g_time_eva);

  node.declare_parameter<bool>("lio.map.if_filter", false);
  node.get_parameter("lio.map.if_filter", g_if_filter);

  node.declare_parameter<std::string>("lio.map.save_map_dir", "");
  node.get_parameter("lio.map.save_map_dir", g_save_map_dir);
  g_save_map_dir = g_root_dir + g_save_map_dir;

  node.declare_parameter<std::string>("lio.map.map_name", "default");
  node.get_parameter("lio.map.map_name", g_map_name);

  node.declare_parameter<double>("lio.map.ds_size", 0.5);
  node.get_parameter("lio.map.ds_size", g_map_ds_size);

  node.declare_parameter<int>("lio.map.save_interval", 1);
  node.get_parameter("lio.map.save_interval", g_pcd_save_interval);

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

  // ================= localization =================
  node.declare_parameter<bool>("lio.localization.update_map", false);
  node.get_parameter("lio.localization.update_map", g_update_map);

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
: rclcpp::Node("super_lio", options)
{
  LoadParamFromRos(*this);
  LOG(INFO) << GREEN << " ---> Using Lidar type: "
            << lidarTypeToString(g_lidar_type) << RESET;

  msg2uav_.header.frame_id = "world";
  path_.header.frame_id = "world";

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
      "/lio/path", 10);

  pub_cloud_world_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/lio/cloud_world", 10);

  tf_broadcaster_ =
      std::make_shared<tf2_ros::TransformBroadcaster>(this);

  pub_path2_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
  pub_loopConstraint_edge_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/loop_constraint_edge", 10);
  pubGlobalmap = this->create_publisher<sensor_msgs::msg::PointCloud2>("/map_global", 1);
  visualizeMapThread = std::thread(&visualize_globalmap_thread, pubGlobalmap);
  key_sub = this->create_subscription<std_msgs::msg::String>("/key_input", 10, keyCallback);
}


void ROSWrapper::imuHandler(const sensor_msgs::msg::Imu::SharedPtr msg){
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
    odom_imu.header.frame_id = "world";
    odom_robo.header.frame_id = "world";
    odom_imu.child_frame_id = "imu";
    odom_robo.child_frame_id = "base_link";
    pub_imu_odom_->publish(odom_imu);
    pub_robo_odom_->publish(odom_robo);
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


void ROSWrapper::pub_odom(const NavState& state){
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "world";

  odom.header.stamp = toRosTime(state.timestamp);
  odom.pose.pose.position.x = state.p[0];
  odom.pose.pose.position.y = state.p[1];
  odom.pose.pose.position.z = state.p[2];

  V4 temp_q = state.R.coeffs();
  odom.pose.pose.orientation.x = temp_q[0];
  odom.pose.pose.orientation.y = temp_q[1];
  odom.pose.pose.orientation.z = temp_q[2];
  odom.pose.pose.orientation.w = temp_q[3];

  odom.twist.twist.linear.x = state.v[0];
  odom.twist.twist.linear.y = state.v[1];
  odom.twist.twist.linear.z = state.v[2];

  odom.child_frame_id = "imu";
  pub_odom_->publish(odom);    // imu frame -> lidar frequency

  V3 lidar_position = state.R.R_ * g_lidar_imu.t_ + state.p;
  M3 lidar_rotation = state.R.R_ * g_lidar_imu.R_;
  Quat lidar_quat(lidar_rotation);
  lidar_quat.normalize();

  V3 robo_position = state.R.R_ * g_odom_robo.t_ + state.p;
  M3 robo_rotation = state.R.R_ * g_odom_robo.R_;
  Quat robo_quat(robo_rotation);
  robo_quat.normalize();

  /// Current pose in the world frame, semantic-compatible with
  /// fastlio_localization's /lidar_localization (world -> lidar).
  {
    nav_msgs::msg::Odometry lidar_odom;
    lidar_odom.header.frame_id = "world";
    lidar_odom.child_frame_id = "lidar";
    lidar_odom.header.stamp = odom.header.stamp;
    lidar_odom.pose.pose.position.x = lidar_position[0];
    lidar_odom.pose.pose.position.y = lidar_position[1];
    lidar_odom.pose.pose.position.z = lidar_position[2];
    lidar_odom.pose.pose.orientation.w = lidar_quat.w();
    lidar_odom.pose.pose.orientation.x = lidar_quat.x();
    lidar_odom.pose.pose.orientation.y = lidar_quat.y();
    lidar_odom.pose.pose.orientation.z = lidar_quat.z();
    pub_lidar_localization_->publish(lidar_odom);
  }

  /// Debug: echo the current world pose to the terminal at ~1 Hz.
  if (print_pose_)
  {
    static auto last_print = std::chrono::steady_clock::now();
    const auto now = std::chrono::steady_clock::now();
    if (now - last_print >= std::chrono::seconds(1))
    {
      last_print = now;
      const M3D R_robo_d = robo_rotation.cast<double>();
      const V3D rpy = EigenMath::RotationMatrix2RPY(R_robo_d);
      LOG(INFO) << "[pose] world->base_link xyz=("
                << robo_position.x() << ", " << robo_position.y() << ", " << robo_position.z()
                << ") rpy(deg)=(" << RAD2DEG(rpy.x()) << ", " << RAD2DEG(rpy.y()) << ", "
                << RAD2DEG(rpy.z()) << ")" << RESET;
    }
  }

  if(g_2_robot){
    static auto pub_msg2uav_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/mavros/vision_pose/pose", 10);
    msg2uav_.header.stamp = odom.header.stamp;
    msg2uav_.pose.position.x = robo_position[0];
    msg2uav_.pose.position.y = robo_position[1];
    msg2uav_.pose.position.z = robo_position[2];
    msg2uav_.pose.orientation.w = robo_quat.w();
    msg2uav_.pose.orientation.x = robo_quat.x();
    msg2uav_.pose.orientation.y = robo_quat.y();
    msg2uav_.pose.orientation.z = robo_quat.z();
    pub_msg2uav_->publish(msg2uav_);
  }

  if((last_path_point_ - robo_position).norm() > 0.1)
  {
    path_.header.stamp = odom.header.stamp;
    geometry_msgs::msg::PoseStamped point;
    point.pose = odom.pose.pose;
    path_.poses.push_back(point);
    if (path_.poses.size() >= 300)
    {
        path_.poses.erase(path_.poses.begin());
    }
    pub_path_->publish(path_);
    last_path_point_ = robo_position;
  }

  geometry_msgs::msg::TransformStamped tf_msg;

  if (true)
  {
    tf_msg.header.stamp = odom.header.stamp;
    tf_msg.header.frame_id = "world";
    tf_msg.child_frame_id = "base_link";

    tf_msg.transform.translation.x = robo_position[0];
    tf_msg.transform.translation.y = robo_position[1];
    tf_msg.transform.translation.z = robo_position[2];

    tf_msg.transform.rotation.x = robo_quat.x();
    tf_msg.transform.rotation.y = robo_quat.y();
    tf_msg.transform.rotation.z = robo_quat.z();
    tf_msg.transform.rotation.w = robo_quat.w();
  }
  else
  {
    tf_msg.header.stamp = odom.header.stamp;
    tf_msg.header.frame_id = "world";
    tf_msg.child_frame_id = "imu";

    tf_msg.transform.translation.x = state.p[0];
    tf_msg.transform.translation.y = state.p[1];
    tf_msg.transform.translation.z = state.p[2];

    tf_msg.transform.rotation.x = temp_q.x();
    tf_msg.transform.rotation.y = temp_q.y();
    tf_msg.transform.rotation.z = temp_q.z();
    tf_msg.transform.rotation.w = temp_q.w();
  }

  tf_broadcaster_->sendTransform(tf_msg);

  // tf_msg.child_frame_id = "god";
  // tf_msg.transform.rotation.x = 0.0;
  // tf_msg.transform.rotation.y = 0.0;
  // tf_msg.transform.rotation.z = 0.0;
  // tf_msg.transform.rotation.w = 1.0;
  // tf_broadcaster_->sendTransform(tf_msg);

}


void ROSWrapper::pub_cloud_world(const CloudPtr& pc, double time){
  sensor_msgs::msg::PointCloud2 cloud;
  pcl::toROSMsg(*pc, cloud);
  cloud.header.frame_id = "world";
  cloud.header.stamp = toRosTime(time);
  pub_cloud_world_->publish(cloud);
}

void ROSWrapper::pub_processing_time(double time, 
  double current_time, double mean_time, double std_time)
{
  static auto pub_processing_time_ =
    this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/lio/processing_time", 10);
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = toRosTime(time);
  msg.pose.position.x = current_time;
  msg.pose.position.y = mean_time;
  msg.pose.position.z = std_time;
  pub_processing_time_->publish(msg);
}

void ROSWrapper::PublishKeyframeTrajectory(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &pubPath,
                                           const pcl::PointCloud<PointXYZIRPYT> &trajectory)
{
  nav_msgs::msg::Path path;
  path.header.stamp = rclcpp::Clock().now();
  path.header.frame_id = "world";

  geometry_msgs::msg::PoseStamped msg_lidar_pose;
  for (const auto &point : trajectory)
  {
    msg_lidar_pose.pose.position.x = point.x;
    msg_lidar_pose.pose.position.y = point.y;
    msg_lidar_pose.pose.position.z = point.z;
    auto quat = EigenMath::RPY2Quaternion(V3D(point.roll, point.pitch, point.yaw));
    msg_lidar_pose.pose.orientation.x = quat.x();
    msg_lidar_pose.pose.orientation.y = quat.y();
    msg_lidar_pose.pose.orientation.z = quat.z();
    msg_lidar_pose.pose.orientation.w = quat.w();

    msg_lidar_pose.header.stamp = rclcpp::Clock().now();
    msg_lidar_pose.header.frame_id = "world";

    path.poses.push_back(msg_lidar_pose);
  }

  pubPath->publish(path);
}

void ROSWrapper::VisualizeLoopClosureConstraints(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr &pubLoopConstraintEdge,
                                                 const unordered_map<int, int> &loop_constraint_records,
                                                 const pcl::PointCloud<PointXYZIRPYT>::Ptr keyframe_pose6d)
{
  if (loop_constraint_records.empty())
    return;

  visualization_msgs::msg::MarkerArray markerArray;
  // loop nodes
  visualization_msgs::msg::Marker markerNode;
  markerNode.header.frame_id = "world";
  markerNode.header.stamp = rclcpp::Clock().now();
  markerNode.action = visualization_msgs::msg::Marker::ADD;
  markerNode.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  markerNode.ns = "loop_nodes";
  markerNode.id = 0;
  markerNode.pose.orientation.w = 1;
  markerNode.scale.x = 0.3;
  markerNode.scale.y = 0.3;
  markerNode.scale.z = 0.3;
  markerNode.color.r = 0;
  markerNode.color.g = 0.8;
  markerNode.color.b = 1;
  markerNode.color.a = 1;
  // loop edges
  visualization_msgs::msg::Marker markerEdge;
  markerEdge.header.frame_id = "world";
  markerEdge.header.stamp = rclcpp::Clock().now();
  markerEdge.action = visualization_msgs::msg::Marker::ADD;
  markerEdge.type = visualization_msgs::msg::Marker::LINE_LIST;
  markerEdge.ns = "loop_edges";
  markerEdge.id = 1;
  markerEdge.pose.orientation.w = 1;
  markerEdge.scale.x = 0.1;
  markerEdge.color.r = 0.9;
  markerEdge.color.g = 0.9;
  markerEdge.color.b = 0;
  markerEdge.color.a = 1;

  for (auto it = loop_constraint_records.begin(); it != loop_constraint_records.end(); ++it)
  {
    int key_cur = it->first;
    int key_pre = it->second;
    geometry_msgs::msg::Point p;
    p.x = keyframe_pose6d->points[key_cur].x;
    p.y = keyframe_pose6d->points[key_cur].y;
    p.z = keyframe_pose6d->points[key_cur].z;
    markerNode.points.push_back(p);
    markerEdge.points.push_back(p);
    p.x = keyframe_pose6d->points[key_pre].x;
    p.y = keyframe_pose6d->points[key_pre].y;
    p.z = keyframe_pose6d->points[key_pre].z;
    markerNode.points.push_back(p);
    markerEdge.points.push_back(p);
  }

  markerArray.markers.push_back(markerNode);
  markerArray.markers.push_back(markerEdge);
  pubLoopConstraintEdge->publish(markerArray);
}

void ROSWrapper::saveMap() {
  backend.save_trajectory();
  // backend.save_factor_graph();

  if (save_globalmap_en)
    backend.save_globalmap();

  if (save_pgm)
    backend.save_pgm(pgm_resolution, min_z, max_z);
}

void ROSWrapper::set_global_map(const BASIC::CloudPtr& global_map){
  pcl::toROSMsg(*global_map, global_map_msg_);
  global_map_msg_.header.frame_id = "world";

  static auto global_map_pub =
    this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/lio/global_map", 10);

  static auto global_map_timer =
    this->create_wall_timer(
      std::chrono::seconds(1),
      [this]() {
        static int count = -1;
        static int publish_interval = 1;

        count++;
        if (count % publish_interval != 0) {
          return;
        }

        count = 0;
        publish_interval++;
        if (publish_interval > 10) {
          publish_interval = 10;
        }
        global_map_msg_.header.stamp = this->now();
        global_map_pub->publish(global_map_msg_);
      });
}


void ROSWrapper::set_initial_data(bool& flg_get_init_guess, bool flg_finish_init)
{
  static auto init_pose_sub =
    this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 1,
        [this, &flg_get_init_guess](
          const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) 
        {
          const auto &pos = msg->pose.pose.position;
          const auto &ori = msg->pose.pose.orientation;

          // Full 6-DoF: z is no longer hard-coded and roll/pitch are preserved.
          V3 init_translation(pos.x, pos.y, pos.z);
          Quat init_rotation(ori.w, ori.x, ori.y, ori.z);

          flg_get_init_guess = true;

          // Feed the backend relocalization pipeline as the highest-priority source.
          {
            // Relocalization::set_init_pose() and BnB operate in RADIANS
            // (lidar_extrinsic is also stored in radians), so keep rpy in radians here.
            const V3 rpy = EigenMath::RotationMatrix2RPY(init_rotation.toRotationMatrix());  // roll, pitch, yaw
            Pose manual_pose;  // imu pose, radians, consumed by Relocalization::set_init_pose()
            manual_pose.x = pos.x;
            manual_pose.y = pos.y;
            manual_pose.z = pos.z;
            manual_pose.roll = rpy(0);
            manual_pose.pitch = rpy(1);
            manual_pose.yaw = rpy(2);
            // Queue only: applying it runs on the relocalization owner thread so it
            // can never race an in-flight background search (and never blocks here).
            backend.relocalization->request_manual_pose(manual_pose);
          }

          LOG(INFO) << YELLOW
                  << " ---> GET Initial guess: "
                  << init_translation.transpose()
                  << " rpy(deg): "
                  << RAD2DEG(EigenMath::RotationMatrix2RPY(init_rotation.toRotationMatrix())).transpose()
                  << RESET;
        });

  if (flg_finish_init) {
    init_pose_sub.reset();
  }
}

void ROSWrapper::set_gnss_data()
{
  if (!use_gnss_) {
    LOG(INFO) << YELLOW << " ---> GNSS prior disabled (relocalization.use_gnss = false)." << RESET;
    return;
  }

  if (!sub_gnss_) {
    // The relocalization pipeline is compiled with `gnss_with_direction`, so the
    // prior must carry an orientation and be expressed in the map frame. That is
    // nav_msgs/Odometry, not a raw NavSatFix (lat/lon/alt would be fed as metres).
    sub_gnss_ = this->create_subscription<nav_msgs::msg::Odometry>(
        gnss_topic_, 10,
        [](const nav_msgs::msg::Odometry::SharedPtr msg) {
          const auto &pos = msg->pose.pose.position;
          const auto &ori = msg->pose.pose.orientation;
          const double stamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1.0e-9;
          std::lock_guard<std::mutex> lock(backend.relocalization->gnss_mutex);
          backend.relocalization->gnss_pose =
              GnssPose(stamp, V3D(pos.x, pos.y, pos.z), QD(ori.w, ori.x, ori.y, ori.z));
        });
    LOG(INFO) << YELLOW << " ---> GNSS prior enabled, topic: " << gnss_topic_
              << " (expects nav_msgs/Odometry in the map frame)" << RESET;
  }
}

void ROSWrapper::load_backend_param(Backend &backend)
{
  vector<double> extrinT;
  vector<double> extrinR;
  V3D extrinT_eigen;
  M3D extrinR_eigen;

  this->declare_parameter("backend.keyframe_add_dist_threshold", 1.f);
  this->declare_parameter("backend.keyframe_add_angle_threshold", 0.2f);
  this->declare_parameter("backend.numsv", 20);
  this->declare_parameter("backend.rtk_age", 30.f);
  this->declare_parameter("backend.gpsCovThreshold", vector<double>());
  this->declare_parameter("backend.pose_cov_threshold", 25.f);
  this->declare_parameter("backend.gnss_weight", vector<double>());
  this->declare_parameter("backend.gnssValidInterval", 0.2f);
  this->declare_parameter("backend.useGpsElevation", false);
  this->declare_parameter("backend.extrinsic_gnss_T", vector<double>());
  this->declare_parameter("backend.extrinsic_gnss_R", vector<double>());
  this->declare_parameter("backend.recontruct_kdtree", true);
  this->declare_parameter("backend.ikdtree_reconstruct_keyframe_num", 10);
  this->declare_parameter("backend.ikdtree_reconstruct_downsamp_size", 0.1f);
  this->declare_parameter("backend.loop_closure_enable_flag", false);
  this->declare_parameter("backend.loop_closure_interval", 1000);
  this->declare_parameter("backend.loop_keyframe_num_thld", 50);
  this->declare_parameter("backend.loop_closure_search_radius", 10.f);
  this->declare_parameter("backend.loop_closure_keyframe_interval", 30);
  this->declare_parameter("backend.keyframe_search_num", 20);
  this->declare_parameter("backend.loop_closure_fitness_use_adaptability", false);
  this->declare_parameter("backend.loop_closure_fitness_score_thld_min", 0.1);
  this->declare_parameter("backend.loop_closure_fitness_score_thld_max", 0.3);
  this->declare_parameter("backend.icp_downsamp_size", 0.1);
  this->declare_parameter("backend.manually_loop_vaild_period", vector<double>());
  this->declare_parameter("backend.odom_loop_vaild_period", vector<double>());
  this->declare_parameter("backend.scancontext_loop_vaild_period", vector<double>());
  this->declare_parameter("official.save_globalmap_en", true);
  this->declare_parameter("official.save_pgm", false);
  this->declare_parameter("official.pgm_resolution", 0.05);
  this->declare_parameter("official.min_z", -1.5f);
  this->declare_parameter("official.max_z", 0.1f);
  this->declare_parameter("official.save_keyframe_en", true);
  this->declare_parameter("official.save_keyframe_descriptor_en", true);
  this->declare_parameter("official.save_resolution", 0.1f);
  this->declare_parameter("official.map_path", "");
  this->declare_parameter("scan_context.lidar_height", 2.0);
  this->declare_parameter("scan_context.sc_dist_thres", 0.5);

  // ---------------------------------------------------------------- relocalization
  this->declare_parameter("official.system_mode", true);  // false => localization mode
  this->declare_parameter("relocalization.algorithm_type", std::string("bbs3d"));
  this->declare_parameter("relocalization.use_gnss", false);
  this->declare_parameter("relocalization.gnss_topic", std::string("/gnss/odom"));
  this->declare_parameter("relocalization.voxelmap_auto_build", true);

  // ---- BBS3D (global search, CPU/OpenMP) ----
  this->declare_parameter("bbs3d.min_level_res", 0.5);
  this->declare_parameter("bbs3d.max_level", 6);
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

  // ---- BnB3D window refinement ----
  this->declare_parameter("bnb3d.linear_xy_window_size", 8.0);
  this->declare_parameter("bnb3d.linear_z_window_size", 0.5);
  this->declare_parameter("bnb3d.angular_search_window", 30.0);  // degree
  this->declare_parameter("bnb3d.pc_resolutions", vector<double>{0.3, 0.5, 0.8});
  this->declare_parameter("bnb3d.bnb_depth", 3);
  this->declare_parameter("bnb3d.min_score", 0.1);
  this->declare_parameter("bnb3d.enough_score", 0.8);
  this->declare_parameter("bnb3d.min_xy_resolution", 0.2);
  this->declare_parameter("bnb3d.min_z_resolution", 0.1);
  this->declare_parameter("bnb3d.min_angular_resolution", 5.0);  // degree
  this->declare_parameter("bnb3d.filter_size_scan", 2.0);
  this->declare_parameter("bnb3d.debug_mode", false);

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

  this->get_parameter("backend.keyframe_add_dist_threshold", backend.backend->keyframe_add_dist_threshold);
  this->get_parameter("backend.keyframe_add_angle_threshold", backend.backend->keyframe_add_angle_threshold);
  this->get_parameter("backend.numsv", backend.gnss->numsv);
  this->get_parameter("backend.rtk_age", backend.gnss->rtk_age);
  this->get_parameter("backend.gpsCovThreshold", backend.gnss->gpsCovThreshold);
  this->get_parameter("backend.pose_cov_threshold", backend.backend->pose_cov_threshold);
  this->get_parameter("backend.gnss_weight", backend.backend->gnss_weight);
  this->get_parameter("backend.gnssValidInterval", backend.gnss->gnssValidInterval);
  this->get_parameter("backend.useGpsElevation", backend.gnss->useGpsElevation);

  this->get_parameter("backend.extrinsic_gnss_T", extrinT);
  this->get_parameter("backend.extrinsic_gnss_R", extrinR);
  extrinT_eigen << VEC_FROM_ARRAY(extrinT);
  extrinR_eigen << MAT_FROM_ARRAY(extrinR);
  backend.gnss->set_extrinsic(extrinT_eigen, extrinR_eigen);

  this->get_parameter("backend.recontruct_kdtree", backend.backend->recontruct_kdtree);
  this->get_parameter("backend.ikdtree_reconstruct_keyframe_num", backend.backend->ikdtree_reconstruct_keyframe_num);
  this->get_parameter("backend.ikdtree_reconstruct_downsamp_size", backend.backend->ikdtree_reconstruct_downsamp_size);

  this->get_parameter("backend.loop_closure_enable_flag", backend.loop_closure_enable_flag);
  this->get_parameter("backend.loop_closure_interval", backend.loop_closure_interval);
  this->get_parameter("backend.loop_keyframe_num_thld", backend.loopClosure->loop_keyframe_num_thld);
  this->get_parameter("backend.loop_closure_search_radius", backend.loopClosure->loop_closure_search_radius);
  this->get_parameter("backend.loop_closure_keyframe_interval", backend.loopClosure->loop_closure_keyframe_interval);
  this->get_parameter("backend.keyframe_search_num", backend.loopClosure->keyframe_search_num);
  this->get_parameter("backend.loop_closure_fitness_use_adaptability", backend.loopClosure->loop_closure_fitness_use_adaptability);
  this->get_parameter("backend.loop_closure_fitness_score_thld_min", backend.loopClosure->loop_closure_fitness_score_thld_min);
  this->get_parameter("backend.loop_closure_fitness_score_thld_max", backend.loopClosure->loop_closure_fitness_score_thld_max);
  this->get_parameter("backend.icp_downsamp_size", backend.loopClosure->icp_downsamp_size);
  this->get_parameter("backend.manually_loop_vaild_period", backend.loopClosure->loop_vaild_period["manually"]);
  this->get_parameter("backend.odom_loop_vaild_period", backend.loopClosure->loop_vaild_period["odom"]);
  this->get_parameter("backend.scancontext_loop_vaild_period", backend.loopClosure->loop_vaild_period["scancontext"]);

  this->declare_parameter("backend.gtsam_ground_constraint_enable", false);
  this->declare_parameter("backend.update_height", true);
  this->declare_parameter("backend.add_height_factor_threshold", 0.1);
  this->declare_parameter("backend.loop_closure_height_thld", 20.f);
  this->get_parameter("backend.gtsam_ground_constraint_enable", backend.backend->z_axis_constraint_enable);
  this->get_parameter("backend.update_height", backend.backend->update_height);
  this->get_parameter("backend.add_height_factor_threshold", backend.backend->add_height_factor_threshold);
  this->get_parameter("backend.loop_closure_height_thld", backend.loopClosure->loop_closure_height_thld);

  this->get_parameter("official.save_globalmap_en", save_globalmap_en);
  this->get_parameter("official.save_pgm", save_pgm);
  this->get_parameter("official.pgm_resolution", pgm_resolution);
  this->get_parameter("official.min_z", min_z);
  this->get_parameter("official.max_z", max_z);
  this->get_parameter("official.save_keyframe_en", backend.save_keyframe_en);
  this->get_parameter("official.save_keyframe_descriptor_en", backend.save_keyframe_descriptor_en);
  this->get_parameter("official.save_resolution", backend.save_resolution);
  this->get_parameter("official.map_path", backend.map_path);
  if (backend.map_path.compare("") != 0)
  {
    backend.globalmap_path = backend.map_path + "/globalmap.pcd";
    backend.trajectory_path = backend.map_path + "/trajectory.pcd";
    backend.keyframe_path = backend.map_path + "/keyframe/";
    backend.scd_path = backend.map_path + "/scancontext/";
  }
  else
    backend.map_path = PCD_FILE_DIR("");

  this->get_parameter("scan_context.lidar_height", backend.relocalization->sc_manager->LIDAR_HEIGHT);
  this->get_parameter("scan_context.sc_dist_thres", backend.relocalization->sc_manager->SC_DIST_THRES);

  // ---------------------------------------------------------------- relocalization
  bool system_mode = true;
  this->get_parameter("official.system_mode", system_mode);
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
    this->get_parameter("bnb3d.linear_xy_window_size", match_option.linear_xy_window_size);
    this->get_parameter("bnb3d.linear_z_window_size", match_option.linear_z_window_size);
    this->get_parameter("bnb3d.angular_search_window", match_option.angular_search_window);
    this->get_parameter("bnb3d.pc_resolutions", match_option.pc_resolutions);
    this->get_parameter("bnb3d.bnb_depth", match_option.bnb_depth);
    this->get_parameter("bnb3d.min_score", match_option.min_score);
    this->get_parameter("bnb3d.enough_score", match_option.enough_score);
    this->get_parameter("bnb3d.min_xy_resolution", match_option.min_xy_resolution);
    this->get_parameter("bnb3d.min_z_resolution", match_option.min_z_resolution);
    this->get_parameter("bnb3d.min_angular_resolution", match_option.min_angular_resolution);
    this->get_parameter("bnb3d.filter_size_scan", match_option.filter_size_scan);
    this->get_parameter("bnb3d.debug_mode", match_option.debug_mode);
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

  if (system_mode)
    backend.init_system_mode();
  else
    LOG(WARNING) << YELLOW << " ---> Localization mode: init_system_mode() skipped." << RESET;

  this->declare_parameter("publish.showOptimizedPose", true);
  this->declare_parameter("publish.print_pose", false);  // echo world pose to terminal (debug)
  this->declare_parameter("publish.globalMapVisualizationSearchRadius", 1000.);
  this->declare_parameter("publish.globalMapVisualizationPoseDensity", 10.);
  this->declare_parameter("publish.globalMapVisualizationLeafSize", 1.);
  this->get_parameter("publish.print_pose", print_pose_);
  this->get_parameter("publish.showOptimizedPose", showOptimizedPose);
  this->get_parameter("publish.globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius);
  this->get_parameter("publish.globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity);
  this->get_parameter("publish.globalMapVisualizationLeafSize", globalMapVisualizationLeafSize);
}

} // namespace END.