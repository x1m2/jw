#include "laser_mapping.hpp"

PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
LaserMappingNode::LaserMappingNode(const rclcpp::NodeOptions &options)
    : Node("laser_mapping_node", options),
      extrinT(3, 0.0),
      extrinR(9, 0.0),
      XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0),
      XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0),
      position_last(Zero3d),
      Lidar_T_wrt_IMU(Zero3d),
      Lidar_R_wrt_IMU(Eye3d) {
  p_pre.reset(new Preprocess());
  p_imu.reset(new ImuProcess());
  pcl_wait_pub.reset(new PointCloudXYZI());
  pcl_wait_save.reset(new PointCloudXYZI());
  featsFromMap.reset(new PointCloudXYZI());
  feats_undistort.reset(new PointCloudXYZI());
  feats_down_body.reset(new PointCloudXYZI());
  feats_down_world.reset(new PointCloudXYZI());
  normvec.reset(new PointCloudXYZI(100000, 1));
  readParameters();
  // 初始化path的header（包括时间戳和帧id），path用于保存odemetry的路径
  path.header.stamp = this->get_clock()->now();
  path.header.frame_id = "odom";
  initializeComponents();
  initializeFiles();
  initializeSubscribersAndPublishers();
  RCLCPP_INFO(this->get_logger(), "Node init finished.");
}
//--------------------init-------------------------//
LaserMappingNode::~LaserMappingNode() {
  fout_out.close();
  fout_pre.close();
  fclose(fp);
}

void LaserMappingNode::readParameters() {
  this->declare_parameter<bool>("publish.path_en", true);
  this->declare_parameter<bool>("publish.effect_map_en", false);
  this->declare_parameter<bool>("publish.map_en", false);
  this->declare_parameter<bool>("publish.scan_publish_en", true);
  this->declare_parameter<bool>("publish.dense_publish_en", true);
  this->declare_parameter<bool>("publish.scan_bodyframe_pub_en", true);
  this->declare_parameter<int>("max_iteration", 4);
  this->declare_parameter<string>("map_file_path", "");
  this->declare_parameter<string>("common.lid_topic", "/livox/lidar");
  this->declare_parameter<string>("common.imu_topic", "/livox/imu");
  this->declare_parameter<bool>("common.time_sync_en", false);
  this->declare_parameter<double>("common.time_offset_lidar_to_imu", 0.0);
  this->declare_parameter<double>("filter_size_corner", 0.5);
  this->declare_parameter<double>("filter_size_surf", 0.5);
  this->declare_parameter<double>("filter_size_map", 0.5);
  this->declare_parameter<double>("cube_side_length", 200.);
  this->declare_parameter<float>("mapping.det_range", 300.);
  this->declare_parameter<double>("mapping.fov_degree", 180.);
  this->declare_parameter<double>("mapping.gyr_cov", 0.1);
  this->declare_parameter<double>("mapping.acc_cov", 0.1);
  this->declare_parameter<double>("mapping.b_gyr_cov", 0.0001);
  this->declare_parameter<double>("mapping.b_acc_cov", 0.0001);
  this->declare_parameter<double>("preprocess.blind", 0.01);
  this->declare_parameter<int>("preprocess.lidar_type", AVIA);
  this->declare_parameter<int>("preprocess.scan_line", 16);
  this->declare_parameter<int>("preprocess.timestamp_unit", US);
  this->declare_parameter<int>("preprocess.scan_rate", 10);
  this->declare_parameter<int>("point_filter_num", 2);
  this->declare_parameter<bool>("feature_extract_enable", false);
  this->declare_parameter<bool>("runtime_pos_log_enable", false);
  this->declare_parameter<bool>("align_gravity", true);
  this->declare_parameter<bool>("mapping.extrinsic_est_en", true);
  this->declare_parameter<bool>("pcd_save.pcd_save_en", false);
  this->declare_parameter<int>("pcd_save.interval", -1);
  this->declare_parameter<vector<double>>("mapping.extrinsic_T",
                                          vector<double>());
  this->declare_parameter<vector<double>>("mapping.extrinsic_R",
                                          vector<double>());
  this->declare_parameter<vector<double>>("mapping.base_to_imu_T",
                                          vector<double>());
  this->declare_parameter<double>("mapping.base_to_imu_R", 0.0);
  this->declare_parameter<bool>("z_constraint.enable", true);
  // imu constraint
  this->declare_parameter<bool>("imu_constraint.enable_auto_constraint", false);

  this->declare_parameter<bool>("imu_constraint.enable_velocity_constraint",
                                false);

  this->declare_parameter<bool>("imu_constraint.enable_rotation_constraint",
                                false);

  this->declare_parameter<int>("imu_constraint.feature_threshold_strong", 100);

  this->declare_parameter<int>("imu_constraint.feature_threshold_medium", 200);

  this->declare_parameter<double>("imu_constraint.strong_constraint_weight",
                                  3.0);

  this->declare_parameter<double>("imu_constraint.medium_constraint_weight",
                                  2.0);

  this->declare_parameter<double>("imu_constraint.max_rotation_correction",
                                  0.1);

  this->get_parameter_or<bool>("publish.path_en", path_en, true);
  this->get_parameter_or<bool>("publish.effect_map_en", effect_pub_en, false);
  this->get_parameter_or<bool>("publish.map_en", map_pub_en, false);
  this->get_parameter_or<bool>("publish.scan_publish_en", scan_pub_en, true);
  this->get_parameter_or<bool>("publish.dense_publish_en", dense_pub_en, true);
  this->get_parameter_or<bool>("publish.scan_bodyframe_pub_en",
                               scan_body_pub_en, true);
  this->get_parameter_or<int>("max_iteration", NUM_MAX_ITERATIONS, 4);
  this->get_parameter_or<string>("map_file_path", map_file_path, "");
  this->get_parameter_or<string>("common.lid_topic", lid_topic, "/livox/lidar");
  this->get_parameter_or<string>("common.imu_topic", imu_topic, "/livox/imu");
  this->get_parameter_or<bool>("common.time_sync_en", time_sync_en, false);
  this->get_parameter_or<double>("common.time_offset_lidar_to_imu",
                                 time_diff_lidar_to_imu, 0.0);
  this->get_parameter_or<double>("filter_size_corner", filter_size_corner_min,
                                 0.5);
  this->get_parameter_or<double>("filter_size_surf", filter_size_surf_min, 0.5);
  this->get_parameter_or<double>("filter_size_map", filter_size_map_min, 0.5);
  this->get_parameter_or<double>("cube_side_length", cube_len, 200.f);
  this->get_parameter_or<float>("mapping.det_range", DET_RANGE, 300.f);
  this->get_parameter_or<double>("mapping.gyr_cov", gyr_cov, 0.1);
  this->get_parameter_or<double>("mapping.acc_cov", acc_cov, 0.1);
  this->get_parameter_or<double>("mapping.b_gyr_cov", b_gyr_cov, 0.0001);
  this->get_parameter_or<double>("mapping.b_acc_cov", b_acc_cov, 0.0001);
  this->get_parameter_or<double>("preprocess.blind", p_pre->blind, 0.01);
  this->get_parameter_or<int>("preprocess.lidar_type", p_pre->lidar_type, AVIA);
  this->get_parameter_or<int>("preprocess.scan_line", p_pre->N_SCANS, 16);
  this->get_parameter_or<int>("preprocess.timestamp_unit", p_pre->time_unit,
                              US);
  this->get_parameter_or<int>("preprocess.scan_rate", p_pre->SCAN_RATE, 10);
  this->get_parameter_or<int>("point_filter_num", p_pre->point_filter_num, 2);
  this->get_parameter_or<bool>("feature_extract_enable", p_pre->feature_enabled,
                               false);
  this->get_parameter_or<bool>("runtime_pos_log_enable", runtime_pos_log, 0);
  this->get_parameter_or<bool>("align_gravity", align_gravity, 1);

  this->get_parameter_or<bool>("mapping.extrinsic_est_en", extrinsic_est_en,
                               true);
  this->get_parameter_or<bool>("pcd_save.pcd_save_en", pcd_save_en, false);
  this->get_parameter_or<int>("pcd_save.interval", pcd_save_interval, -1);
  this->get_parameter_or<vector<double>>("mapping.extrinsic_T", extrinT,
                                         vector<double>());
  this->get_parameter_or<vector<double>>("mapping.extrinsic_R", extrinR,
                                         vector<double>());
  this->get_parameter_or<vector<double>>("mapping.base_to_imu_T", base_to_imu_T,
                                         vector<double>());
  this->get_parameter_or<double>("mapping.base_to_imu_R", base_to_imu_R, 0.0);
  this->get_parameter_or<bool>("z_constraint.enable", enable_z_constraint_,
                               true);
  // imu constraint
  this->get_parameter_or<bool>("imu_constraint.enable_auto_constraint",
                               enable_auto_constraint, false);

  this->get_parameter_or<bool>("imu_constraint.enable_velocity_constraint",
                               enable_velocity_constraint, false);

  this->get_parameter_or<bool>("imu_constraint.enable_rotation_constraint",
                               enable_rotation_constraint, false);

  this->get_parameter_or<int>("imu_constraint.feature_threshold_strong",
                              feature_threshold_strong, 100);

  this->get_parameter_or<int>("imu_constraint.feature_threshold_medium",
                              feature_threshold_medium, 200);

  this->get_parameter_or<double>("imu_constraint.strong_constraint_weight",
                                 strong_constraint_weight, 3.0);

  this->get_parameter_or<double>("imu_constraint.medium_constraint_weight",
                                 medium_constraint_weight, 2.0);

  this->get_parameter_or<double>("imu_constraint.max_rotation_correction",
                                 max_rotation_correction, 0.1);

  RCLCPP_INFO(this->get_logger(), "p_pre->lidar_type %d", p_pre->lidar_type);
}

void LaserMappingNode::initializeComponents() {
  // 将数组point_selected_surf内元素的值全部设为true，数组point_selected_surf用于选择平面点
  memset(point_selected_surf, true, sizeof(point_selected_surf));
  memset(res_last, -1000.0f, sizeof(res_last));
  downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min,
                                 filter_size_surf_min);
  downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min,
                                filter_size_map_min);
  // memset(point_selected_surf, true, sizeof(point_selected_surf));
  // memset(res_last, -1000.0f, sizeof(res_last));

  Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
  Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);
  // 设置IMU的参数，
  p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
  p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
  p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
  p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
  p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));
  p_imu->set_align_gravity(align_gravity);

  fill(epsi, epsi + 23, 0.001);  //从epsi填充到epsi+22 也就是全部数组置0.001
                                 //不安全 ，可用std::function + lambda
  static LaserMappingNode *p_node = nullptr;
  p_node = this;
  auto static_lambda = [](state_ikfom &s,
                          esekfom::dyn_share_datastruct<double> &ds) {
    p_node->h_share_model(s, ds);
  };
  // 将函数地址传入kf对象中，用于接收特定于系统的模型及其差异
  // 作为一个维数变化的特征矩阵进行测量。
  // 通过一个函数（h_dyn_share_in）同时计算测量（z）、估计测量（h）、偏微分矩阵（h_x，h_v）和噪声协方差（R）。
  kf.init_dyn_share(get_f, df_dx, df_dw, static_lambda, NUM_MAX_ITERATIONS,
                    epsi);
}

void LaserMappingNode::initializeFiles() {
  /*** debug record ***/
  // FILE *fp;
  string pos_log_dir = root_dir + "/Log/pos_log.txt";
  fp = fopen(pos_log_dir.c_str(), "w");

  // ofstream fout_pre, fout_out, fout_dbg;
  fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"), ios::out);
  fout_out.open(DEBUG_FILE_DIR("mat_out.txt"), ios::out);
  fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"), ios::out);
  if (fout_pre && fout_out)
    cout << "~~~~" << ROOT_DIR << " file opened" << endl;
  else
    cout << "~~~~" << ROOT_DIR << " doesn't exist" << endl;
}

void LaserMappingNode::initializeSubscribersAndPublishers() {
  /*** ROS subscribe initialization ***/
  // 雷达点云的订阅器sub_pcl，订阅点云的topic
  if (p_pre->lidar_type == AVIA) {
    //可以用lambda,但是不能传uniqueptr
    auto lidar_qos = rclcpp::QoS(rclcpp::KeepLast(100));
    lidar_qos.best_effort();
    lidar_qos.durability_volatile();

    sub_pcl_livox_ =
        this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            lid_topic, lidar_qos,
            std::bind(&LaserMappingNode::livox_pcl_cbk, this,
                      std::placeholders::_1));
  } else {
    sub_pcl_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        lid_topic, rclcpp::SensorDataQoS(),
        std::bind(&LaserMappingNode::standard_pcl_cbk, this,
                  std::placeholders::_1));
  }
  auto imu_qos = rclcpp::QoS(rclcpp::KeepLast(2000));
  imu_qos.best_effort();
  imu_qos.durability_volatile();

  sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, imu_qos,
      std::bind(&LaserMappingNode::imu_cbk, this, std::placeholders::_1));

  if (enable_z_constraint_) {
    imu_orientation_converter_ = std::make_shared<ImuOrientationConverter>(
        this, [this](bool is_stairs) { this->set_stairs_mode(is_stairs); });
    RCLCPP_INFO(this->get_logger(),
                "z constraint enabled. start as flat terrain. IMU orientation "
                "converter updates terrain state internally.");
  } else {
    RCLCPP_INFO(this->get_logger(), "z constraint disabled.");
  }

  // 发布当前正在扫描的点云，topic名字为/cloud_registered
  pubLaserCloudFull_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/cloud_registered", 20);
  // 发布经过运动畸变校正到IMU坐标系的点云，
  pubLaserCloudFull_body_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/cloud_registered_body", 20);
  pubLaserCloudEffect_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/cloud_effected", 20);
  pubLaserCloudMap_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", 20);
  pubOdomAftMapped_ =
      this->create_publisher<nav_msgs::msg::Odometry>("/Odometry", 20);
  pubPath_ = this->create_publisher<nav_msgs::msg::Path>("/path", 20);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  //------------------------------------------------------------------------------------------------------
  auto period_ms =
      std::chrono::milliseconds(static_cast<int64_t>(1000.0 / 100.0));
  timer_ =
      rclcpp::create_timer(this, this->get_clock(), period_ms,
                           std::bind(&LaserMappingNode::timer_callback, this));

  auto map_period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0));
  map_pub_timer_ = rclcpp::create_timer(
      this, this->get_clock(), map_period_ms,
      std::bind(&LaserMappingNode::map_publish_callback, this));

  map_save_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "map_save", std::bind(&LaserMappingNode::map_save_callback, this,
                            std::placeholders::_1, std::placeholders::_2));
}

//--------------------callbacks--------------------//
void LaserMappingNode::standard_pcl_cbk(
    const sensor_msgs::msg::PointCloud2::UniquePtr msg) {
  mtx_buffer.lock();
  scan_count++;
  double cur_time = get_time_sec(msg->header.stamp);
  double preprocess_start_time = omp_get_wtime();
  if (!is_first_lidar && cur_time < last_timestamp_lidar) {
    std::cerr << "lidar loop back, clear buffer" << std::endl;
    lidar_buffer.clear();
  }
  if (is_first_lidar) {
    is_first_lidar = false;
  }

  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);
  lidar_buffer.push_back(ptr);
  time_buffer.push_back(cur_time);
  last_timestamp_lidar = cur_time;
  s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
  mtx_buffer.unlock();
  // sig_buffer.notify_all();
}

void LaserMappingNode::livox_pcl_cbk(
    const livox_ros_driver2::msg::CustomMsg::UniquePtr msg) {
  double cb_start_time = omp_get_wtime();
  mtx_buffer.lock();
  double cur_time = get_time_sec(msg->header.stamp);
  double preprocess_start_time = omp_get_wtime();
  scan_count++;
  double lidar_dt = 0.0;
  if (!is_first_lidar) {
    lidar_dt = cur_time - last_timestamp_lidar;

    if (lidar_dt < 0.0) {
      RCLCPP_ERROR(this->get_logger(),
                   "[LIDAR_TIME_BACK] lidar time rollback, clear buffer. "
                   "cur=%.6f, last=%.6f, dt=%.6f",
                   cur_time, last_timestamp_lidar, lidar_dt);

      lidar_buffer.clear();
      time_buffer.clear();
      imu_buffer.clear();
      lidar_pushed = false;
      last_timestamp_lidar = cur_time;

      mtx_buffer.unlock();
      return;
    }

    if (lidar_dt >
        0.12) {  // 120ms
                 // 对于10Hz的雷达来说是比较大的间隔了，可能是数据丢失或者时间戳异常
      RCLCPP_WARN(this->get_logger(),
                  "[LIDAR_STAMP_GAP_WARN] lidar stamp gap. "
                  "dt=%.6f, cur=%.6f, last=%.6f",
                  lidar_dt, cur_time, last_timestamp_lidar);
    }
  }
  if (is_first_lidar) {
    is_first_lidar = false;
  }
  last_timestamp_lidar = cur_time;

  if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 &&
      !imu_buffer.empty() && !lidar_buffer.empty()) {
    RCLCPP_ERROR(this->get_logger(),
                 "[LIDAR_IMU_NOT_SYNC] IMU and LiDAR not synced. "
                 "imu_time=%.6f, lidar_time=%.6f, diff=%.6f",
                 last_timestamp_imu, last_timestamp_lidar,
                 last_timestamp_imu - last_timestamp_lidar);
  }

  if (time_sync_en && !timediff_set_flg &&
      abs(last_timestamp_lidar - last_timestamp_imu) > 1 &&
      !imu_buffer.empty()) {
    timediff_set_flg = true;
    timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
    RCLCPP_WARN(this->get_logger(),
                "[SELF_TIME_SYNC] Self sync IMU and LiDAR. "
                "time_diff=%.10f, lidar_time=%.6f, imu_time=%.6f",
                timediff_lidar_wrt_imu, last_timestamp_lidar,
                last_timestamp_imu);
  }

  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  // 对激光雷达数据进行预处理（特征提取或者降采样）
  double process_begin = omp_get_wtime();
  p_pre->process(msg, ptr);
  double process_ms = (omp_get_wtime() - process_begin) * 1000.0;

  if (process_ms > 20.0) {
    RCLCPP_ERROR(this->get_logger(),
                 "[LIDAR_PREPROCESS_SLOW] lidar preprocess too slow. "
                 "cost=%.3f ms, points=%zu, stamp=%.6f, lidar_dt=%.6f",
                 process_ms, ptr->points.size(), cur_time, lidar_dt);
  }
  if (ptr->empty()) {
    RCLCPP_ERROR(this->get_logger(),
                 "[LIDAR_EMPTY_CLOUD] lidar preprocess output empty. "
                 "stamp=%.6f, lidar_dt=%.6f",
                 cur_time, lidar_dt);
  }
  lidar_buffer.push_back(ptr);
  time_buffer.push_back(last_timestamp_lidar);
  if (lidar_buffer.size() != time_buffer.size()) {
    RCLCPP_ERROR(this->get_logger(),
                 "[LIDAR_TIME_BUFFER_SIZE_MISMATCH] "
                 "lidar_buffer=%zu, time_buffer=%zu",
                 lidar_buffer.size(), time_buffer.size());
  }
  s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
  mtx_buffer.unlock();
  double cb_total_ms = (omp_get_wtime() - cb_start_time) * 1000.0;

  if (cb_total_ms > 30.0) {
    RCLCPP_ERROR(this->get_logger(),
                 "[LIDAR_CALLBACK_SLOW] lidar callback too slow. "
                 "total=%.3f ms, preprocess=%.3f ms, points=%zu, "
                 "lidar_dt=%.6f, lidar_buffer=%zu, imu_buffer=%zu",
                 cb_total_ms, process_ms, ptr->points.size(), lidar_dt,
                 lidar_buffer.size(), imu_buffer.size());
  }
  // sig_buffer.notify_all();
}

void LaserMappingNode::set_stairs_mode(bool is_stairs) {
  const bool new_is_flat = !is_stairs;
  if (new_is_flat == is_flat_terrain_) {
    return;
  }

  is_flat_terrain_ = new_is_flat;
  z_constraint_initialized_ = false;
  output_z_constraint_initialized_ = false;

  if (is_flat_terrain_) {
    RCLCPP_INFO(this->get_logger(),
                "[Z_CONSTRAINT] terrain changed to flat. imu/output z locks "
                "will be initialized on next LIO update.");
  } else {
    RCLCPP_INFO(this->get_logger(),
                "[Z_CONSTRAINT] terrain changed to stairs. imu/output z locks "
                "released.");
  }
}

void LaserMappingNode::imu_cbk(const sensor_msgs::msg::Imu::UniquePtr msg_in) {
  publish_count++;
  sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));
  double raw_timestamp = get_time_sec(msg_in->header.stamp);
  msg->header.stamp =
      get_ros_time(get_time_sec(msg_in->header.stamp) - time_diff_lidar_to_imu);
  if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en) {
    msg->header.stamp = rclcpp::Time(timediff_lidar_wrt_imu +
                                     get_time_sec(msg_in->header.stamp));
  }

  double timestamp = get_time_sec(msg->header.stamp);

  // 1. 检查 IMU 原始 stamp / 修正后 stamp / wall time 是否异常
  {
    static double last_raw_stamp_check = -1.0;
    static double last_stamp_check = -1.0;
    static double last_wall_check = -1.0;

    double wall_now = this->now().seconds();

    if (last_raw_stamp_check > 0.0 && last_stamp_check > 0.0 &&
        last_wall_check > 0.0) {
      double raw_dt = raw_timestamp - last_raw_stamp_check;
      double stamp_dt = timestamp - last_stamp_check;
      double wall_dt = wall_now - last_wall_check;

      if (std::abs(raw_dt) > 0.02 || std::abs(stamp_dt) > 0.02 ||
          wall_dt > 0.08) {
        RCLCPP_ERROR(
            this->get_logger(),
            "[IMU_GAP_CHECK] imu gap abnormal. "
            "raw_dt=%.6f, stamp_dt=%.6f, wall_dt=%.6f, "
            "raw_stamp=%.6f, stamp=%.6f, wall=%.6f, "
            "time_diff_lidar_to_imu=%.6f, timediff_lidar_wrt_imu=%.6f, "
            "time_sync_en=%d",
            raw_dt, stamp_dt, wall_dt, raw_timestamp, timestamp, wall_now,
            time_diff_lidar_to_imu, timediff_lidar_wrt_imu, time_sync_en);
      }
    }

    last_raw_stamp_check = raw_timestamp;
    last_stamp_check = timestamp;
    last_wall_check = wall_now;
  }
  mtx_buffer.lock();

  if (timestamp < last_timestamp_imu) {
    RCLCPP_WARN(this->get_logger(),
                "imu loop back, clear imu buffer. cur_imu=%.6f, last_imu=%.6f",
                timestamp, last_timestamp_imu);
    imu_buffer.clear();
  }
  // 4. 检查 IMU stamp 间隔
  if (last_timestamp_imu > 0.0) {
    double imu_dt = timestamp - last_timestamp_imu;

    if (imu_dt > 0.02 || imu_dt < 0.0) {
      RCLCPP_ERROR(this->get_logger(),
                   "[IMU_STAMP_GAP] imu stamp gap too large. "
                   "imu_dt=%.6f s, cur_imu=%.6f, last_imu=%.6f",
                   imu_dt, timestamp, last_timestamp_imu);
    }
  }
  last_timestamp_imu = timestamp;

  imu_buffer.push_back(msg);
  mtx_buffer.unlock();
  // sig_buffer.notify_all();
}

void LaserMappingNode::timer_callback() {
  // 将激光雷达点云数据和IMU数据从缓存队列中取出，进行时间对齐，并保存到Measures中

  static double last_timer_begin = 0.0;
  double timer_begin = omp_get_wtime();

  if (last_timer_begin > 0) {
    double timer_interval_ms = (timer_begin - last_timer_begin) * 1000.0;

    // 正常 10ms 左右，只在明显卡顿时打印
    if (timer_interval_ms > 50.0) {
      RCLCPP_WARN(this->get_logger(), "[TIMER_INTERVAL_WARN] interval=%.3f ms",
                  timer_interval_ms);
    }
  }

  last_timer_begin = timer_begin;
  if (sync_packages(Measures)) {
    if (flg_first_scan) {
      first_lidar_time = Measures.lidar_beg_time;
      p_imu->first_lidar_time = first_lidar_time;
      flg_first_scan = false;
      return;
    }

    double t0, t1, t2, t3, t4, t5, match_start, solve_start, svd_time;

    match_time = 0;
    kdtree_search_time = 0.0;
    solve_time = 0;
    solve_const_H_time = 0;
    svd_time = 0;
    t0 = omp_get_wtime();
    // 对IMU数据进行预处理，其中包含了点云畸变处理 前向传播 反向传播
    p_imu->Process(Measures, kf, feats_undistort);
    // 获取kf预测的全局状态（imu）
    state_point = kf.get_x();
    pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

    if (feats_undistort->empty() || (feats_undistort == NULL)) {
      RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\n");
      return;
    }

    flg_EKF_inited =
        (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? false : true;
    /*** Segment the map in lidar FOV ***/
    // 动态调整局部地图,在拿到eskf前馈结果后
    lasermap_fov_segment();

    /*** downsample the feature points in a scan ***/
    downSizeFilterSurf.setInputCloud(feats_undistort);
    downSizeFilterSurf.filter(*feats_down_body);
    t1 = omp_get_wtime();
    feats_down_size = feats_down_body->points.size();
    /*** initialize the map kdtree ***/
    if (ikdtree.Root_Node == nullptr) {
      RCLCPP_INFO(this->get_logger(), "Initialize the map kdtree");
      if (feats_down_size > 5) {
        ikdtree.set_downsample_param(filter_size_map_min);
        feats_down_world->resize(feats_down_size);
        for (int i = 0; i < feats_down_size; i++) {
          pointBodyToWorld(&(feats_down_body->points[i]),
                           &(feats_down_world->points[i]));
        }
        ikdtree.Build(feats_down_world->points);  //构建ikd树
      }
      return;
    }

    int featsFromMapNum = ikdtree.validnum();
    kdtree_size_st = ikdtree.size();

    // cout<<"[ mapping ]: In num: "<<feats_undistort->points.size()<<"
    // downsamp "<<feats_down_size<<" Map num: "<<featsFromMapNum<<"effect
    // num:"<<effct_feat_num<<endl;

    /*** ICP and iterated Kalman filter update ***/
    if (feats_down_size < 5) {
      RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\n");
      return;
    }
    // ICP和迭代卡尔曼滤波更新
    normvec->resize(feats_down_size);
    feats_down_world->resize(feats_down_size);

    V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
    fout_pre << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
             << euler_cur.transpose() << " " << state_point.pos.transpose()
             << " " << ext_euler.transpose() << " "
             << state_point.offset_T_L_I.transpose() << " "
             << state_point.vel.transpose() << " " << state_point.bg.transpose()
             << " " << state_point.ba.transpose() << " " << state_point.grav
             << endl;

    if (0)  // If you need to see map point, change to "if(1)"
    {
      PointVector().swap(ikdtree.PCL_Storage);
      ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
      featsFromMap->clear();
      featsFromMap->points = ikdtree.PCL_Storage;
    }

    pointSearchInd_surf.resize(feats_down_size);
    Nearest_Points.resize(feats_down_size);
    int rematch_num = 0;
    bool nearest_search_en = true;  //

    t2 = omp_get_wtime();
    /*** 迭代状态估计 ***/
    double t_update_start = omp_get_wtime();
    double solve_H_time = 0;

    kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
    state_point = kf.get_x();

    if (enable_z_constraint_) {
      if (is_flat_terrain_) {
        if (!z_constraint_initialized_) {
          z_constraint_value_ = state_point.pos(2);
          z_constraint_initialized_ = true;
          RCLCPP_INFO(this->get_logger(),
                      "[Z_CONSTRAINT] flat mode: lock FAST-LIO imu z at %.3f",
                      z_constraint_value_);
        }

        state_point.pos(2) = z_constraint_value_;
        state_point.vel(2) = 0.0;
        kf.change_x(state_point);
      } else if (z_constraint_initialized_) {
        RCLCPP_INFO(this->get_logger(),
                    "[Z_CONSTRAINT] stairs mode: release FAST-LIO imu z");
        z_constraint_initialized_ = false;
      }
    }
    ////////////////////////////////////////
    euler_cur = SO3ToEuler(state_point.rot);
    pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
    geoQuat.x = state_point.rot.coeffs()[0];
    geoQuat.y = state_point.rot.coeffs()[1];
    geoQuat.z = state_point.rot.coeffs()[2];
    geoQuat.w = state_point.rot.coeffs()[3];

    double t_update_end = omp_get_wtime();

    /******* Publish odometry *******/
    publish_odometry(pubOdomAftMapped_, tf_broadcaster_);

    /*** add the feature points to map kdtree ***/
    t3 = omp_get_wtime();
    map_incremental();
    t5 = omp_get_wtime();

    /******* Publish points *******/
    if (path_en) publish_path(pubPath_);
    if (scan_pub_en) publish_frame_world(pubLaserCloudFull_);
    if (scan_pub_en && scan_body_pub_en)
      publish_frame_body(pubLaserCloudFull_body_);
    if (effect_pub_en) publish_effect_world(pubLaserCloudEffect_);
    // if (map_pub_en) publish_map(pubLaserCloudMap_);

    /*** Debug variables ***/
    if (runtime_pos_log) {
      frame_num++;
      kdtree_size_end = ikdtree.size();
      aver_time_consu =
          aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
      aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num +
                      (t_update_end - t_update_start) / frame_num;
      aver_time_match = aver_time_match * (frame_num - 1) / frame_num +
                        (match_time) / frame_num;
      aver_time_incre = aver_time_incre * (frame_num - 1) / frame_num +
                        (kdtree_incremental_time) / frame_num;
      aver_time_solve = aver_time_solve * (frame_num - 1) / frame_num +
                        (solve_time + solve_H_time) / frame_num;
      aver_time_const_H_time =
          aver_time_const_H_time * (frame_num - 1) / frame_num +
          solve_time / frame_num;
      T1[time_log_counter] = Measures.lidar_beg_time;
      s_plot[time_log_counter] = t5 - t0;
      s_plot2[time_log_counter] = feats_undistort->points.size();
      s_plot3[time_log_counter] = kdtree_incremental_time;
      s_plot4[time_log_counter] = kdtree_search_time;
      s_plot5[time_log_counter] = kdtree_delete_counter;
      s_plot6[time_log_counter] = kdtree_delete_time;
      s_plot7[time_log_counter] = kdtree_size_st;
      s_plot8[time_log_counter] = kdtree_size_end;
      s_plot9[time_log_counter] = aver_time_consu;
      s_plot10[time_log_counter] = add_point_size;
      time_log_counter++;
      printf(
          "[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: "
          "%0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave "
          "total: %0.6f icp: %0.6f construct H: %0.6f \n",
          t1 - t0, aver_time_match, aver_time_solve, t3 - t1, t5 - t3,
          aver_time_consu, aver_time_icp, aver_time_const_H_time);
      ext_euler = SO3ToEuler(state_point.offset_R_L_I);
      fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
               << euler_cur.transpose() << " " << state_point.pos.transpose()
               << " " << ext_euler.transpose() << " "
               << state_point.offset_T_L_I.transpose() << " "
               << state_point.vel.transpose() << " "
               << state_point.bg.transpose() << " "
               << state_point.ba.transpose() << " " << state_point.grav << " "
               << feats_undistort->points.size() << endl;
      dump_lio_state_to_log(fp);
    }
  }
  {
    double timer_end = omp_get_wtime();
    double process_time_ms = (timer_end - timer_begin) * 1000.0;

    if (process_time_ms > 50.0) {
      RCLCPP_WARN(this->get_logger(), "[PROCESS_TIME_WARN] total=%.3f ms",
                  process_time_ms);
    }
  }
}

void LaserMappingNode::applyImuConstraintFusion(
    const state_ikfom &state_before_update,
    const state_ikfom &state_after_update, double update_trans_delta,
    double update_yaw_delta, double valid_ratio, bool bad_pos, bool bad_yaw) {
  // 默认先使用激光 EKF 更新后的状态
  // 后面只对异常的部分进行融合修正
  state_point = state_after_update;

  // EKF 更新前状态，近似认为是 IMU 预测状态
  const V3D imu_predicted_pos = state_before_update.pos;
  const V3D imu_predicted_vel = state_before_update.vel;
  const SO3 imu_predicted_rot = state_before_update.rot;

  // EKF 更新后状态，认为是激光匹配修正后的状态
  const V3D laser_updated_pos = state_after_update.pos;
  const V3D laser_updated_vel = state_after_update.vel;
  const SO3 laser_updated_rot = state_after_update.rot;

  double dynamic_imu_constraint_weight = 1.0;

  if (effct_feat_num < feature_threshold_strong) {
    dynamic_imu_constraint_weight = strong_constraint_weight;
  } else if (effct_feat_num < feature_threshold_medium) {
    dynamic_imu_constraint_weight = medium_constraint_weight;
  }

  // 有效比例太低时，提升 IMU 权重
  if (valid_ratio < 0.12) {
    dynamic_imu_constraint_weight =
        std::max(dynamic_imu_constraint_weight, medium_constraint_weight);
  }

  // yaw 被 EKF 拉得太大时，提升 IMU 权重
  if (update_yaw_delta > 5.0 * M_PI / 180.0) {
    dynamic_imu_constraint_weight =
        std::max(dynamic_imu_constraint_weight, strong_constraint_weight);
  }

  const double laser_weight = 1.0 / (1.0 + dynamic_imu_constraint_weight);
  const double imu_weight =
      dynamic_imu_constraint_weight / (1.0 + dynamic_imu_constraint_weight);

  // 只有位置异常时，才融合位置
  if (bad_pos) {
    state_point.pos =
        laser_weight * laser_updated_pos + imu_weight * imu_predicted_pos;
  }

  // 速度先不建议开；即使开，也只在位置异常时融合速度
  if (enable_velocity_constraint && bad_pos) {
    state_point.vel =
        laser_weight * laser_updated_vel + imu_weight * imu_predicted_vel;
  }

  // 只有 yaw 异常时，才融合姿态
  if (enable_rotation_constraint && bad_yaw) {
    SO3 rot_diff = laser_updated_rot.inverse() * imu_predicted_rot;
    Eigen::Vector3d rot_diff_log = SO3::log(rot_diff);

    if (rot_diff_log.norm() > max_rotation_correction) {
      rot_diff_log = rot_diff_log.normalized() * max_rotation_correction;
    }

    state_point.rot = laser_updated_rot * SO3::exp(imu_weight * rot_diff_log);
  }

  V3D before_euler = SO3ToEuler(state_before_update.rot);
  V3D laser_euler = SO3ToEuler(laser_updated_rot);
  V3D fused_euler = SO3ToEuler(state_point.rot);

  RCLCPP_WARN(this->get_logger(),
              "[IMU_CONSTRAINT_FUSION] "
              "effct_feat_num=%d, valid_ratio=%.3f, mean_residual=%.4f, "
              "delta_pos=%.3f, delta_yaw=%.2f deg, "
              "bad_pos=%d, bad_yaw=%d, "
              "laser_weight=%.2f, imu_weight=%.2f, "
              "before_yaw=%.2f deg, laser_yaw=%.2f deg, fused_yaw=%.2f deg, "
              "laser_pos=[%.3f %.3f %.3f], "
              "imu_pos=[%.3f %.3f %.3f], "
              "fused_pos=[%.3f %.3f %.3f]",
              effct_feat_num, valid_ratio, res_mean_last, update_trans_delta,
              update_yaw_delta * 180.0 / M_PI, bad_pos, bad_yaw, laser_weight,
              imu_weight, before_euler(2) * 180.0 / M_PI,
              laser_euler(2) * 180.0 / M_PI, fused_euler(2) * 180.0 / M_PI,
              laser_updated_pos(0), laser_updated_pos(1), laser_updated_pos(2),
              imu_predicted_pos(0), imu_predicted_pos(1), imu_predicted_pos(2),
              state_point.pos(0), state_point.pos(1), state_point.pos(2));
}

void LaserMappingNode::map_publish_callback() {
  if (map_pub_en) publish_map(pubLaserCloudMap_);
}

void LaserMappingNode::map_save_callback(
    std_srvs::srv::Trigger::Request::ConstSharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res) {
  RCLCPP_INFO(this->get_logger(), "Saving map to %s...", map_file_path.c_str());
  if (pcd_save_en) {
    save_to_pcd();
    res->success = true;
    res->message = "Map saved.";
  } else {
    res->success = false;
    res->message = "Map save disabled.";
  }
}

//--------------------------------functions-----------------------------
//地图的增量更新，主要完成对ikd-tree的地图建立
void LaserMappingNode::map_incremental() {
  PointVector PointToAdd;
  PointVector PointNoNeedDownsample;
  PointToAdd.reserve(feats_down_size);
  PointNoNeedDownsample.reserve(feats_down_size);
  //根据点与所在包围盒中心点的距离，分类是否需要降采样
  for (int i = 0; i < feats_down_size; i++) {
    /* transform to world frame */
    pointBodyToWorld(&(feats_down_body->points[i]),
                     &(feats_down_world->points[i]));
    // 判断是否有关键点需要加到地图中
    if (!Nearest_Points[i].empty() && flg_EKF_inited) {
      const PointVector &points_near = Nearest_Points[i];
      bool need_add = true;
      BoxPointType Box_of_Point;
      PointType downsample_result, mid_point;
      mid_point.x = floor(feats_down_world->points[i].x / filter_size_map_min) *
                        filter_size_map_min +
                    0.5 * filter_size_map_min;
      mid_point.y = floor(feats_down_world->points[i].y / filter_size_map_min) *
                        filter_size_map_min +
                    0.5 * filter_size_map_min;
      mid_point.z = floor(feats_down_world->points[i].z / filter_size_map_min) *
                        filter_size_map_min +
                    0.5 * filter_size_map_min;
      // 当前点与box中心的距离
      float dist = calc_dist(feats_down_world->points[i], mid_point);
      //判断最近点在x、y、z三个方向上，与中心的距离，判断是否加入时需要降采样
      if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min &&
          fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min &&
          fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min) {
        //若三个方向距离都大于地图栅格半轴长，无需降采样
        PointNoNeedDownsample.push_back(feats_down_world->points[i]);
        continue;
      }
      //判断当前点的 NUM_MATCH_POINTS 个邻近点与包围盒中心的范围
      for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i++) {
        if (points_near.size() < NUM_MATCH_POINTS) break;
        // 如果存在邻近点到中心的距离小于当前点到中心的距离，则不需要添加当前点
        if (calc_dist(points_near[readd_i], mid_point) < dist) {
          need_add = false;
          break;
        }
      }
      if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
    } else {
      PointToAdd.push_back(feats_down_world->points[i]);
    }
  }

  double st_time = omp_get_wtime();
  add_point_size = ikdtree.Add_Points(PointToAdd, true);
  ikdtree.Add_Points(PointNoNeedDownsample, false);
  add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
  kdtree_incremental_time = omp_get_wtime() - st_time;
}
//得到被剔除的点
void LaserMappingNode::points_cache_collect() {
  PointVector points_history;
  ikdtree.acquire_removed_points(points_history);
  // for (int i = 0; i < points_history.size(); i++)
  // _featsArray->push_back(points_history[i]);
}

void LaserMappingNode::lasermap_fov_segment() {
  cub_needrm.clear();
  kdtree_delete_counter = 0;
  kdtree_delete_time = 0.0;
  pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
  V3D pos_LiD = pos_lid;
  //初始化局部地图包围盒角点
  if (!Localmap_Initialized) {
    for (int i = 0; i < 3; i++) {
      LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
      LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
    }
    Localmap_Initialized = true;
    return;
  }
  float dist_to_map_edge[3][2];  // lidar与立方体盒子六个面的距离
  bool need_move = false;
  // 当前雷达系中心到各个地图边缘的距离
  for (int i = 0; i < 3; i++) {
    dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
    dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
    if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE ||
        dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
      need_move = true;
  }
  if (!need_move) return;
  BoxPointType New_LocalMap_Points, tmp_boxpoints;
  // 新的局部地图盒子边界点
  New_LocalMap_Points = LocalMap_Points;
  float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9,
                       double(DET_RANGE * (MOV_THRESHOLD - 1)));
  for (int i = 0; i < 3; i++) {
    tmp_boxpoints = LocalMap_Points;
    //与包围盒最小值边界点距离
    if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE) {
      New_LocalMap_Points.vertex_max[i] -= mov_dist;
      New_LocalMap_Points.vertex_min[i] -= mov_dist;
      tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
      cub_needrm.push_back(tmp_boxpoints);
    } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) {
      New_LocalMap_Points.vertex_max[i] += mov_dist;
      New_LocalMap_Points.vertex_min[i] += mov_dist;
      tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
      cub_needrm.push_back(tmp_boxpoints);
    }
  }
  LocalMap_Points = New_LocalMap_Points;

  points_cache_collect();
  double delete_begin = omp_get_wtime();
  // 使用Boxs删除指定盒内的点
  if (cub_needrm.size() > 0)
    kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
  kdtree_delete_time = omp_get_wtime() - delete_begin;
}

bool LaserMappingNode::sync_packages(MeasureGroup &meas) {
  if (lidar_buffer.empty() || imu_buffer.empty()) {
    return false;
  }
  // 1. 队列堆积检查
  const size_t lidar_buffer_warn_size = 5;
  const size_t imu_buffer_warn_size = 500;

  if (lidar_buffer.size() > lidar_buffer_warn_size ||
      imu_buffer.size() > imu_buffer_warn_size) {
    RCLCPP_ERROR(this->get_logger(),
                 "[SYNC_BUFFER_LARGE] buffer too large. "
                 "lidar_buffer=%zu/%zu, time_buffer=%zu, imu_buffer=%zu/%zu, "
                 "last_lidar=%.6f, last_imu=%.6f, imu_minus_lidar=%.6f",
                 lidar_buffer.size(), lidar_buffer_warn_size,
                 time_buffer.size(), imu_buffer.size(), imu_buffer_warn_size,
                 last_timestamp_lidar, last_timestamp_imu,
                 last_timestamp_imu - last_timestamp_lidar);
  }
  // 2. lidar_buffer 和 time_buffer 一致性检查
  if (lidar_buffer.size() != time_buffer.size()) {
    RCLCPP_ERROR(
        this->get_logger(),
        "[SYNC_BUFFER_SIZE_MISMATCH] lidar_buffer=%zu, time_buffer=%zu",
        lidar_buffer.size(), time_buffer.size());
    return false;
  }
  /*** push a lidar scan ***/
  if (!lidar_pushed) {
    meas.lidar = lidar_buffer.front();
    meas.lidar_beg_time = time_buffer.front();
    if (meas.lidar->points.size() <= 1)  // time too little
    {
      lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;

      RCLCPP_ERROR(this->get_logger(),
                   "[SYNC_LIDAR_TOO_FEW_POINTS] points=%zu, "
                   "lidar_beg=%.6f, lidar_end=%.6f, mean_scan_time=%.6f",
                   meas.lidar->points.size(), meas.lidar_beg_time,
                   lidar_end_time, lidar_mean_scantime);

    } else if (meas.lidar->points.back().curvature / double(1000) <
               0.5 * lidar_mean_scantime) {
      lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
    } else {
      scan_num++;
      lidar_end_time = meas.lidar_beg_time +
                       meas.lidar->points.back().curvature / double(1000);
      lidar_mean_scantime +=
          (meas.lidar->points.back().curvature / double(1000) -
           lidar_mean_scantime) /
          scan_num;
    }

    meas.lidar_end_time = lidar_end_time;
    //检查当前这一帧 LiDAR 的扫描时间长度是否正常
    double scan_duration = meas.lidar_end_time - meas.lidar_beg_time;
    if (scan_duration < 0.07 || scan_duration > 0.13) {
      RCLCPP_ERROR(this->get_logger(),
                   "[SYNC_LIDAR_DURATION_ABNORMAL] duration=%.6f, "
                   "lidar_beg=%.6f, lidar_end=%.6f, points=%zu",
                   scan_duration, meas.lidar_beg_time, meas.lidar_end_time,
                   meas.lidar->points.size());
    }
    lidar_pushed = true;
  }

  if (last_timestamp_imu < lidar_end_time) {
    return false;
  }

  /*** push imu data, and pop from imu buffer ***/
  double imu_time = get_time_sec(imu_buffer.front()->header.stamp);
  meas.imu.clear();
  while ((!imu_buffer.empty()) && (imu_time < lidar_end_time)) {
    imu_time = get_time_sec(imu_buffer.front()->header.stamp);
    if (imu_time > lidar_end_time) break;
    meas.imu.push_back(imu_buffer.front());
    imu_buffer.pop_front();
  }
  //不能太少
  if (meas.imu.size() < 5) {
    RCLCPP_ERROR(this->get_logger(),
                 "[SYNC_IMU_SIZE_TOO_SMALL] imu size too small for lidar. "
                 "imu_size=%zu, lidar_beg=%.6f, lidar_end=%.6f, "
                 "last_imu=%.6f, lidar_buffer=%zu, imu_buffer=%zu",
                 meas.imu.size(), meas.lidar_beg_time, meas.lidar_end_time,
                 last_timestamp_imu, lidar_buffer.size(), imu_buffer.size());
  }
  lidar_buffer.pop_front();
  time_buffer.pop_front();
  lidar_pushed = false;
  return true;
}

void LaserMappingNode::save_to_pcd() {
  pcl::PCDWriter pcd_writer;
  pcd_writer.writeBinary(map_file_path, *pcl_wait_pub);
}
//计算残差信息
void LaserMappingNode::h_share_model(
    state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) {
  double match_start = omp_get_wtime();
  laserCloudOri->clear();
  corr_normvect->clear();
  total_residual = 0.0;

/** 最接近曲面搜索和残差计算  **/
#ifdef MP_EN
  omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
  //对降采样后的每个特征点进行残差计算
  for (int i = 0; i < feats_down_size; i++) {
    PointType &point_body = feats_down_body->points[i];
    PointType &point_world = feats_down_world->points[i];

    /* transform to world frame */
    V3D p_body(point_body.x, point_body.y, point_body.z);
    V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);
    point_world.x = p_global(0);
    point_world.y = p_global(1);
    point_world.z = p_global(2);
    point_world.intensity = point_body.intensity;

    vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

    auto &points_near = Nearest_Points[i];

    if (ekfom_data.converge) {  //如果收敛了
      //在已构造的地图上查找特征点的最近邻(NUM_MATCH_POINTS)
      ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near,
                             pointSearchSqDis);
      point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false
                               : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5
                                   ? false
                                   : true;
    }

    if (!point_selected_surf[i]) continue;

    VF(4) pabcd;
    point_selected_surf[i] = false;
    //拟合平面方程ax+by+cz+d=0并求解点到平面距离
    if (esti_plane(pabcd, points_near, 0.1f)) {
      float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y +
                  pabcd(2) * point_world.z + pabcd(3);  //计算点到平面的距离
      float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());  //计算残差

      if (s > 0.9) {
        point_selected_surf[i] = true;
        normvec->points[i].x = pabcd(0);  //将法向量存储至normvec
        normvec->points[i].y = pabcd(1);
        normvec->points[i].z = pabcd(2);
        normvec->points[i].intensity = pd2;
        res_last[i] = abs(pd2);
      }
    }
  }

  effct_feat_num = 0;  //有效特征点数

  for (int i = 0; i < feats_down_size; i++) {
    if (point_selected_surf[i]) {
      laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
      corr_normvect->points[effct_feat_num] = normvec->points[i];
      total_residual += res_last[i];  //计算总残差
      effct_feat_num++;
    }
  }

  if (effct_feat_num < 1) {
    ekfom_data.valid = false;
    RCLCPP_WARN(this->get_logger(), "No Effective Points!");
    return;
  }

  res_mean_last = total_residual / effct_feat_num;  //计算残差平均值
  {
    double valid_ratio = 0.0;
    if (feats_down_size > 0) {
      valid_ratio = static_cast<double>(effct_feat_num) /
                    static_cast<double>(feats_down_size);
    }

    static int fastlio_match_print_count = 0;
    fastlio_match_print_count++;

    {
      if (effct_feat_num < 100 || valid_ratio < 0.10 || res_mean_last > 0.15) {
        RCLCPP_WARN(
            rclcpp::get_logger("laser_mapping_node"),
            "[FASTLIO_BAD_MATCH] feats_down_size=%d, effct_feat_num=%d, "
            "valid_ratio=%.3f, mean_residual=%.4f, total_residual=%.4f",
            feats_down_size, effct_feat_num, valid_ratio, res_mean_last,
            total_residual);

      } else if (fastlio_match_print_count % 10 == 0) {
        RCLCPP_INFO(rclcpp::get_logger("laser_mapping_node"),
                    "[FASTLIO_MATCH] feats_down_size=%d, effct_feat_num=%d, "
                    "valid_ratio=%.3f, mean_residual=%.4f, total_residual=%.4f",
                    feats_down_size, effct_feat_num, valid_ratio, res_mean_last,
                    total_residual);
      }
    }
  }
  match_time += omp_get_wtime() - match_start;
  double solve_start_ = omp_get_wtime();

  // 测量雅可比矩阵H和测量向量的计算 H=J*P*J'
  ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12);  // 23
  ekfom_data.h.resize(effct_feat_num);
  //求观测值与误差的雅克比矩阵，
  for (int i = 0; i < effct_feat_num; i++) {
    const PointType &laser_p = laserCloudOri->points[i];
    V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
    M3D point_be_crossmat;
    point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
    V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
    M3D point_crossmat;
    point_crossmat << SKEW_SYM_MATRX(point_this);

    /*** get the normal vector of closest surface/corner ***/
    const PointType &norm_p = corr_normvect->points[i];
    V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

    /*** calculate the Measuremnt Jacobian matrix H ***/
    V3D C(s.rot.conjugate() * norm_vec);
    V3D A(point_crossmat * C);
    if (extrinsic_est_en) {
      V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() *
            C);  // s.rot.conjugate()*norm_vec);
      ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z,
          VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
    } else {
      ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z,
          VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    }

    // 测量:到最近表面/角落的距离
    ekfom_data.h(i) = -norm_p.intensity;
  }
  solve_time += omp_get_wtime() - solve_start_;
}

Eigen::Matrix4d LaserMappingNode::get_odom_to_base_transform() const {
  // FAST-LIO state: odom -> imu_link
  Eigen::Matrix4d T_odom_imu = Eigen::Matrix4d::Identity();
  T_odom_imu.block<3, 3>(0, 0) = state_point.rot.toRotationMatrix();
  T_odom_imu.block<3, 1>(0, 3) = state_point.pos;

  // Static extrinsic: base_link -> imu_link.
  Eigen::Matrix4d T_base_imu = Eigen::Matrix4d::Identity();
  T_base_imu.block<3, 3>(0, 0) =
      Eigen::AngleAxisd(base_to_imu_R, Eigen::Vector3d::UnitY())
          .toRotationMatrix();
  T_base_imu.block<3, 1>(0, 3) =
      Eigen::Vector3d(base_to_imu_T[0], base_to_imu_T[1], base_to_imu_T[2]);

  // Output pose: odom -> base_link.
  return T_odom_imu * T_base_imu.inverse();
}

Eigen::Matrix4d LaserMappingNode::get_output_base_transform() {
  const Eigen::Matrix4d T_odom_base = get_odom_to_base_transform();

  if (!output_origin_initialized_) {
    T_output_origin_inv_ = T_odom_base.inverse();
    output_origin_initialized_ = true;

    const Eigen::Matrix3d R_odom_base = T_odom_base.block<3, 3>(0, 0);
    const Eigen::Vector3d euler =
        R_odom_base.eulerAngles(2, 1, 0) * 180.0 / M_PI;
    RCLCPP_INFO(this->get_logger(),
                "[OUTPUT_ORIGIN] first base_link set as output origin. "
                "raw odom base: x=%.3f, y=%.3f, z=%.3f, "
                "roll=%.2f deg, pitch=%.2f deg, yaw=%.2f deg",
                T_odom_base(0, 3), T_odom_base(1, 3), T_odom_base(2, 3),
                euler(2), euler(1), euler(0));
  }

  return T_output_origin_inv_ * T_odom_base;
}

Eigen::Vector3d LaserMappingNode::transform_odom_point_to_output(
    const Eigen::Vector3d &p_odom) {
  if (!output_origin_initialized_) {
    (void)get_output_base_transform();
  }

  const Eigen::Vector4d p_homo(p_odom.x(), p_odom.y(), p_odom.z(), 1.0);
  return (T_output_origin_inv_ * p_homo).head<3>();
}

template <typename T>
void LaserMappingNode::set_posestamp(T &out) {
  Eigen::Matrix4d T_output_base = get_output_base_transform();
  const Eigen::Matrix3d R_output_base = T_output_base.block<3, 3>(0, 0);
  const Eigen::Quaterniond q_base(R_output_base);

  if (enable_z_constraint_) {
    if (is_flat_terrain_) {
      if (!output_z_constraint_initialized_) {
        output_z_constraint_value_ = T_output_base(2, 3);
        output_z_constraint_initialized_ = true;
        RCLCPP_INFO(
            this->get_logger(),
            "[Z_CONSTRAINT] flat mode: lock output base_link z at %.3f",
            output_z_constraint_value_);
      }
      T_output_base(2, 3) = output_z_constraint_value_;
    } else if (output_z_constraint_initialized_) {
      RCLCPP_INFO(this->get_logger(),
                  "[Z_CONSTRAINT] stairs mode: release output base_link z");
      output_z_constraint_initialized_ = false;
    }
  }

  out.pose.position.x = T_output_base(0, 3);
  out.pose.position.y = T_output_base(1, 3);
  out.pose.position.z = T_output_base(2, 3);
  out.pose.orientation.x = q_base.x();
  out.pose.orientation.y = q_base.y();
  out.pose.orientation.z = q_base.z();
  out.pose.orientation.w = q_base.w();
}

void LaserMappingNode::pointBodyToWorld_ikfom(PointType const *const pi,
                                              PointType *const po,
                                              state_ikfom &s) {
  V3D p_body(pi->x, pi->y, pi->z);
  V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);

  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
}

void LaserMappingNode::pointBodyToWorld(PointType const *const pi,
                                        PointType *const po) {
  V3D p_body(pi->x, pi->y, pi->z);
  V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body +
                                  state_point.offset_T_L_I) +
               state_point.pos);

  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
}

template <typename T>
void LaserMappingNode::pointBodyToWorld(const Matrix<T, 3, 1> &pi,
                                        Matrix<T, 3, 1> &po) {
  V3D p_body(pi[0], pi[1], pi[2]);
  V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body +
                                  state_point.offset_T_L_I) +
               state_point.pos);

  po[0] = p_global(0);
  po[1] = p_global(1);
  po[2] = p_global(2);
}

void LaserMappingNode::RGBpointBodyToWorld(PointType const *const pi,
                                           PointType *const po) {
  V3D p_body(pi->x, pi->y, pi->z);
  V3D p_odom(state_point.rot * (state_point.offset_R_L_I * p_body +
                                state_point.offset_T_L_I) +
             state_point.pos);
  const Eigen::Vector3d p_global = transform_odom_point_to_output(
      Eigen::Vector3d(p_odom(0), p_odom(1), p_odom(2)));

  po->x = p_global.x();
  po->y = p_global.y();
  po->z = p_global.z();
  po->intensity = pi->intensity;
}

void LaserMappingNode::RGBpointBodyLidarToIMU(PointType const *const pi,
                                              PointType *const po) {
  V3D p_body_lidar(pi->x, pi->y, pi->z);
  V3D p_body_imu(state_point.offset_R_L_I * p_body_lidar +
                 state_point.offset_T_L_I);

  po->x = p_body_imu(0);
  po->y = p_body_imu(1);
  po->z = p_body_imu(2);
  po->intensity = pi->intensity;
}

//------------------------------public functions------------------------------
void LaserMappingNode::publish_frame_world(
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        pubLaserCloudFull) {
  if (scan_pub_en) {
    PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort
                                                       : feats_down_body);
    int size = laserCloudFullRes->points.size();
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++) {
      RGBpointBodyToWorld(&laserCloudFullRes->points[i],
                          &laserCloudWorld->points[i]);
    }

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
    // laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "odom";
    pubLaserCloudFull->publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
  }

  /**************** save map ****************/
  /* 1. make sure you have enough memories
  /* 2. noted that pcd save will influence the real-time performences **/
  /*
  if (pcd_save_en)
  {
      int size = feats_undistort->points.size();
      PointCloudXYZI::Ptr laserCloudWorld( \
                      new PointCloudXYZI(size, 1));

      for (int i = 0; i < size; i++)
      {
          RGBpointBodyToWorld(&feats_undistort->points[i], \
                              &laserCloudWorld->points[i]);
      }
      *pcl_wait_save += *laserCloudWorld;

      static int scan_wait_num = 0;
      scan_wait_num ++;
      if (pcl_wait_save->size() > 0 && pcd_save_interval > 0  && scan_wait_num
  >= pcd_save_interval)
      {
          pcd_index ++;
          string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") +
  to_string(pcd_index) + string(".pcd")); pcl::PCDWriter pcd_writer; cout <<
  "current scan saved to /PCD/" << all_points_dir << endl;
          pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
          pcl_wait_save->clear();
          scan_wait_num = 0;
      }
  }
  */
}

void LaserMappingNode::publish_frame_body(
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        pubLaserCloudFull_body) {
  int size = feats_undistort->points.size();
  PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

  for (int i = 0; i < size; i++) {
    RGBpointBodyLidarToIMU(&feats_undistort->points[i],
                           &laserCloudIMUBody->points[i]);
  }

  sensor_msgs::msg::PointCloud2 laserCloudmsg;
  pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
  laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
  laserCloudmsg.header.frame_id = "base_link";
  pubLaserCloudFull_body->publish(laserCloudmsg);
  publish_count -= PUBFRAME_PERIOD;
}

void LaserMappingNode::publish_effect_world(
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        pubLaserCloudEffect) {
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(effct_feat_num, 1));
  for (int i = 0; i < effct_feat_num; i++) {
    RGBpointBodyToWorld(&laserCloudOri->points[i], &laserCloudWorld->points[i]);
  }
  sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
  pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
  laserCloudFullRes3.header.stamp = get_ros_time(lidar_end_time);
  laserCloudFullRes3.header.frame_id = "odom";
  pubLaserCloudEffect->publish(laserCloudFullRes3);
}

void LaserMappingNode::publish_map(
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        pubLaserCloudMap) {
  PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort
                                                     : feats_down_body);
  int size = laserCloudFullRes->points.size();
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

  for (int i = 0; i < size; i++) {
    RGBpointBodyToWorld(&laserCloudFullRes->points[i],
                        &laserCloudWorld->points[i]);
  }
  *pcl_wait_pub += *laserCloudWorld;

  sensor_msgs::msg::PointCloud2 laserCloudmsg;
  pcl::toROSMsg(*pcl_wait_pub, laserCloudmsg);
  // laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
  laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
  laserCloudmsg.header.frame_id = "odom";
  pubLaserCloudMap->publish(laserCloudmsg);

  // sensor_msgs::msg::PointCloud2 laserCloudMap;
  // pcl::toROSMsg(*featsFromMap, laserCloudMap);
  // laserCloudMap.header.stamp = get_ros_time(lidar_end_time);
  // laserCloudMap.header.frame_id = "odom";
  // pubLaserCloudMap->publish(laserCloudMap);
}

void LaserMappingNode::publish_odometry(
    const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr
        pubOdomAftMapped,
    std::unique_ptr<tf2_ros::TransformBroadcaster> &tf_br) {
  odomAftMapped.header.frame_id = "odom";
  odomAftMapped.child_frame_id = "base_link";
  odomAftMapped.header.stamp = get_ros_time(lidar_end_time);
  set_posestamp(odomAftMapped.pose);
  auto P = kf.get_P();
  for (int i = 0; i < 6; i++) {
    int k = i < 3 ? i + 3 : i - 3;
    odomAftMapped.pose.covariance[i * 6 + 0] = P(k, 3);
    odomAftMapped.pose.covariance[i * 6 + 1] = P(k, 4);
    odomAftMapped.pose.covariance[i * 6 + 2] = P(k, 5);
    odomAftMapped.pose.covariance[i * 6 + 3] = P(k, 0);
    odomAftMapped.pose.covariance[i * 6 + 4] = P(k, 1);
    odomAftMapped.pose.covariance[i * 6 + 5] = P(k, 2);
  }
  pubOdomAftMapped->publish(odomAftMapped);

  geometry_msgs::msg::TransformStamped trans;
  trans.header.frame_id = "odom";
  trans.header.stamp = odomAftMapped.header.stamp;
  trans.child_frame_id = "base_link";
  trans.transform.translation.x = odomAftMapped.pose.pose.position.x;
  trans.transform.translation.y = odomAftMapped.pose.pose.position.y;
  trans.transform.translation.z = odomAftMapped.pose.pose.position.z;
  trans.transform.rotation.w = odomAftMapped.pose.pose.orientation.w;
  trans.transform.rotation.x = odomAftMapped.pose.pose.orientation.x;
  trans.transform.rotation.y = odomAftMapped.pose.pose.orientation.y;
  trans.transform.rotation.z = odomAftMapped.pose.pose.orientation.z;
  tf_br->sendTransform(trans);
}

void LaserMappingNode::publish_path(
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath) {
  set_posestamp(msg_body_pose);
  msg_body_pose.header.stamp =
      get_ros_time(lidar_end_time);  // ros::Time().fromSec(lidar_end_time);
  msg_body_pose.header.frame_id = "odom";

  /*** if path is too large, the rvis will crash ***/
  static int jjj = 0;
  jjj++;
  if (jjj % 10 == 0) {
    path.poses.push_back(msg_body_pose);
    pubPath->publish(path);
  }
}
