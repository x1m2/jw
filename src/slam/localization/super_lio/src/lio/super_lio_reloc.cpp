
#include "lio/super_lio_reloc.h"

#include <chrono>
#include <sys/resource.h>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <tbb/concurrent_vector.h>
#include <tbb/enumerable_thread_specific.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/kdtree/kdtree_flann.h>


using namespace BASIC;

namespace LI2Sup{

inline bool calc_plane_coeff(const int N, const std::array<V3, 5>& points, std::array<double, 4>& abcd)
{
  Eigen::Vector3d normvec;
  if (N == 5) {
    Eigen::Matrix<double, 5, 3> A;
    Eigen::Matrix<double, 5, 1> b;
    for (int j = 0; j < 5; j++) {
      A.row(j) = points[j].cast<double>();
      b(j) = -1.0;
    }
    normvec = A.colPivHouseholderQr().solve(b);
  }
  else {
    Eigen::Matrix<double, 4, 3> A;
    Eigen::Matrix<double, 4, 1> b;

    for (int j = 0; j < N; j++) {
      A.row(j) = points[j].cast<double>();
      b(j) = -1.0;
    }
    normvec = A.colPivHouseholderQr().solve(b);
  }

  double n = normvec.norm();
  if (n < 1e-6f) return false;

  abcd[3] = 1.0 / n;
  normvec *= abcd[3];
  abcd[0] = normvec[0];
  abcd[1] = normvec[1];
  abcd[2] = normvec[2];
  
  for (int i = 0; i < N; ++i) {
    const V3& p = points[i];
    auto dist = abcd[0] * p(0) + abcd[1] * p(1) + abcd[2] * p(2) + abcd[3];
    if (std::abs(dist) > 0.1) return false;
  }
  return true;
}


inline bool compute_error(
  const std::array<double, 4>& abcd, const V3& point, 
  const float length, scalar& error)
{
  error = abcd[0] * point[0] + abcd[1] * point[1] + abcd[2] * point[2] + abcd[3];
  return length > 81 * error * error;
}


void SuperLIOReLoc::init(){
  ivox_.reset(new OctVoxMapType(OctVoxMapType::Options{g_ivox_resolution, g_ivox_capacity}));
  kf_.reset(new ESKF());
  data_wrapper_->setESKF(kf_);

  data_wrapper_->load_backend_param(data_wrapper_->backend());

  scan_undistort_full_.reset(new PointCloudType());
  ds_undistort_.reset(new PointCloudType());
  point_map_.reset(new PointCloudType());
  init_obs_data_.reset(new PointCloudType());
  
  points_world_v3_.reserve(21000);
  abcd_vec_.resize(20000);
  effect_knn_idxs_.resize(20000);
  voxel_grid_fliter_.setLeafSize(g_voxel_fliter_size);

  LOG(INFO) << GREEN << " ---> [SuperLIO]: initialized." << RESET;

  auto start_time = std::chrono::high_resolution_clock::now();

  if (!data_wrapper_->backend().init_localization_mode())
    LOG(WARNING) << YELLOW << " ---> [Relocalization] prior map not loaded; "
                 << "relocalization is disabled until a prior map is provided." << RESET;
  else if (data_wrapper_->backend().global_map && !data_wrapper_->backend().global_map->empty())
  {
    ivox_->clear();
    point_map_->clear();
    *point_map_ = *data_wrapper_->backend().global_map;
    VV3 map_v3;
    map_v3.reserve(point_map_->size());
    for (const auto &p : *point_map_)
      map_v3.emplace_back(p.x, p.y, p.z);
    ivox_->insert(map_v3, true);
    data_wrapper_->set_global_map(point_map_);
    LOG(INFO) << GREEN << " ---> ivox_ seeded from the prior global map: "
              << point_map_->size() << RESET;
    ivox_->printInfo();
  }

  data_wrapper_->set_initial_data(flg_get_init_guess_);
  data_wrapper_->set_gnss_data();

  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  LOG(INFO) << GREEN << " ---> [SuperLIO]: Map init success. Time: " << duration.count() << " ms." << RESET;

  state_fn_ = &SuperLIOReLoc::stateWaitKFInit;
}

bool SuperLIOReLoc::process(){
  if (stopping_) return false;
  if (data_wrapper_->initial_pose_requested()) {
    data_wrapper_->consume_initial_pose_request();
    if (kf_->init_) tracking_failed();
  }
  /// While the background search runs, only poll its result. Skipping sync_measure
  /// keeps the sensor callback fast; the buffered data is cleared once it succeeds.
  if (reloc_inflight_)
  {
    (this->*state_fn_)();
    if (reloc_inflight_)
      return false;
  }
  return SuperLIO::process();
}


void SuperLIOReLoc::tracking_failed() {
  data_wrapper_->publish_status(false);
  kf_ = std::make_shared<ESKF>();
  data_wrapper_->setESKF(kf_);
  init_obs_data_.reset(new PointCloudType());
  imu_cout = init_frame_count = 0;
  mean_gyro.setZero(); mean_acce.setZero();
  data_wrapper_->clear();
  state_fn_ = &SuperLIOReLoc::stateWaitKFInit;
}

bool SuperLIOReLoc::kf_init(){
  const int need_init_frames = 10;

  /// A previous relocalization is still running in the background. Poll it here;
  /// never block the sensor callback. The heavy BBS3D/NDT/GICP work lives in the worker.
  if (reloc_inflight_)
  {
    static auto last_heartbeat = std::chrono::steady_clock::now();
    const auto now = std::chrono::steady_clock::now();
    if (now - last_heartbeat > std::chrono::seconds(5))
    {
      last_heartbeat = now;
      LOG(INFO) << YELLOW << " ---> relocalization still running in background..." << RESET;
    }

    if (reloc_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
      return false;

    RelocTaskResult result = reloc_future_.get();
    reloc_inflight_ = false;

    /// /initialpose arrived while the search was running: prefer the manual pose.
    if (data_wrapper_->backend().relocalization->apply_pending_manual_pose())
    {
      LOG(WARNING) << YELLOW << " ---> Manual pose received during relocalization, "
                   << "discarding the background result." << RESET;
      imu_cout = 0;
      init_frame_count = 0;
      init_obs_data_->clear();
      mean_gyro = V3::Zero();
      mean_acce = V3::Zero();
      return false;
    }

    if (!result.ok)
    {
      LOG(WARNING) << RED << " ---> Relocalization failed, retrying with the next scan." << RESET;
      /// Restart the stationary-window estimation as well: the robot may be moving
      /// now, so keeping the running mean would corrupt the gravity prior.
      imu_cout = 0;
      init_frame_count = 0;
      init_obs_data_->clear();
      mean_gyro = V3::Zero();
      mean_acce = V3::Zero();
      return false;
    }

    LOG(INFO) << GREEN << " ---> Relocalization success, imu_pose:\n" << result.imu_pose << RESET;

    ESKF::Options options;
    options.gyro_var_ = g_imu_ng;
    options.acce_var_ = g_imu_na;
    options.bias_gyro_var_ = g_imu_nbg;
    options.bias_acce_var_ = g_imu_nba;
    options.num_iterations_ = g_kf_max_iterations;
    options.quit_eps_ = g_kf_quit_eps;

    V3 ref_gravity(0, 0, - g_gravity_norm);
    float imu_scale = g_gravity_norm / mean_acce.norm();
    kf_->SetInitialConditions(options, mean_gyro, V3::Zero(), imu_scale, ref_gravity);

    /// Inject the relocalization result as the filter's initial state.
    auto state = kf_->GetSysState();
    state.R = SO3(result.imu_pose.block<3, 3>(0, 0).cast<scalar>());
    state.p = result.imu_pose.block<3, 1>(0, 3).cast<scalar>();
    state.timestamp = -1.0;
    kf_->SetX(state);
    sys_init_pose_ = kf_->GetSE3();

    /// Drop data buffered while the background search was running.
    data_wrapper_->clear();
    data_wrapper_->publish_status(true);

    {
      /// ivox_ already holds the prior map: it must NOT be cleared here.
      point_map_->clear();
      point_map_.reset(new PointCloudType());
      init_obs_data_->clear();
      init_obs_data_ = nullptr;
      data_wrapper_->set_initial_data(flg_get_init_guess_, true);
    }

    return true;
  }

  /// Apply a manual pose queued by /initialpose before starting a new accumulation.
  data_wrapper_->backend().relocalization->apply_pending_manual_pose();

  /// get init guess from ROS topic.
  if(flg_get_init_guess_){
    imu_cout = 0;
    init_frame_count = 0;
    init_obs_data_->clear();
    mean_gyro = V3::Zero();
    mean_acce = V3::Zero();
    flg_get_init_guess_ = false;
    return false;
  }

  CloudPtr point_cloud_pcl = CloudPtr(new PointCloudType());
  for(std::size_t i = 0; i < measures_.lidar.pc->size(); i++){
    auto p = measures_.lidar.pc->points[i];
    PointType point;
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
    point.intensity = p.intensity;
    point_cloud_pcl->points.push_back(point);
  }

  if(init_frame_count < need_init_frames){
    *init_obs_data_ += *point_cloud_pcl;
  }
  init_frame_count++;

  for(auto& imu: measures_.imu){
    imu_cout ++;
    mean_gyro += (imu.gyr - mean_gyro) / imu_cout;
    mean_acce += (imu.acc - mean_acce) / imu_cout;
  }

  if(imu_cout < 20){
    return false;
  }

  if(init_frame_count < need_init_frames){
    return false;
  }

  /// Never touch the relocalization backends before the prior map is loaded:
  /// bnb3d / bbs3d are null otherwise and would crash.
  if (!data_wrapper_->backend().relocalization_ready)
  {
    static bool warned_not_ready = false;
    if (!warned_not_ready)
    {
      warned_not_ready = true;
      LOG(ERROR) << RED << " ---> Prior map not loaded, relocalization unavailable. "
                 << "Check map_path / globalmap.pcd." << RESET;
    }
    init_frame_count = 0;
    init_obs_data_->clear();
    return false;
  }

  /// Launch the heavy part (BBS3D -> BnB/NDT/GICP) on a worker thread. Copy the
  /// scan so the worker never touches measures_ / init_obs_data_ while the sensor
  /// callback keeps receiving data.
  CloudPtr scan_copy(new PointCloudType(*init_obs_data_));
  const double lidar_time = measures_.lidar.end_time;
  const V3D mean_acc = mean_acce.cast<double>();
  LOG(INFO) << YELLOW << " ---> RELOC start in background... obs_data size: " << scan_copy->size()
            << ", prior map size: " << point_map_->size()
            << ", algorithm_type: " << data_wrapper_->backend().relocalization->algorithm_type << RESET;
  try
  {
    reloc_future_ = std::async(std::launch::async, [this, scan_copy, lidar_time, mean_acc]() {
      RelocTaskResult r;
      try
      {
        r.ok = data_wrapper_->backend().run_relocalization(scan_copy, lidar_time, mean_acc, r.imu_pose);
      }
      catch (const std::exception &e)
      {
        LOG_ERROR("relocalization worker threw: %s", e.what());
        r.ok = false;
      }
      return r;
    });
  }
  catch (const std::exception &e)
  {
    LOG_ERROR("Failed to start relocalization worker: %s", e.what());
    return false;
  }
  reloc_inflight_ = true;

  /// Prepare a fresh accumulation window in case this attempt fails.
  init_frame_count = 0;
  init_obs_data_->clear();
  return false;
}


void SuperLIOReLoc::Output() {
  auto state = kf_->GetNavState();
  data_wrapper_->pub_odom(state, match_score_);
  data_wrapper_->pub_cloud_body(scan_undistort_full_, state.timestamp);

  Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
  transformation.block<3, 3>(0, 0) = state.R.R_.cast<float>();
  transformation.block<3, 1>(0, 3) = state.p.cast<float>();

  CloudPtr world_pc(new PointCloudType());
  
  if(g_visual_map){
    output_count_++;
    if(output_count_ % g_pub_step != 0){
      return;
    }
    output_count_ = 0;
    if(g_visual_dense){
      pcl::transformPointCloud(*scan_undistort_full_, *world_pc, transformation);
      data_wrapper_->pub_cloud_world(world_pc, state.timestamp);
    }else{
      pcl::transformPointCloud(*ds_undistort_, *world_pc, transformation);
      data_wrapper_->pub_cloud_world(world_pc, state.timestamp);
    }
  }

}



} // namespace END.