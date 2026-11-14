

#include "lio/params.h"

using namespace std;
using namespace BASIC;

namespace LI2Sup{

  const std::string g_root_dir = std::string(ROOT);
  std::atomic<bool> g_flag_run = true; 

  /// evaluation
  bool g_time_eva = false;

  
  string g_imu_topic;
  string g_lidar_topic;

  int    g_lidar_type;
  float  g_blind2;
  float  g_maxrange2;
  int    g_filter_rate;
  bool   g_enable_downsample;
  float  g_voxel_fliter_size;
  double g_timedelay_lidar2imu = 0.0;

  double g_gravity_norm = 9.7946;
  double g_imu_na;
  double g_imu_ng;
  double g_imu_nba;
  double g_imu_nbg;

  SE3 g_lidar_imu;
  SE3 g_odom_robo;

  /// hash_map
  std::size_t g_ivox_capacity = 100000;
  float       g_ivox_resolution = 0.5;

  /// kf
  int g_kf_max_iterations = 4;
  double g_kf_quit_eps;

  /// output
  bool g_visual_map = true;
  bool g_visual_dense = false;
  int  g_pub_step;

  ResidualType g_residual_type = PROB;

  /// for localization

}
