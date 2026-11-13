#pragma once
#include "backend/Header.h"
#include "Relocalization.h"

// Owns the fixed prior map and the matchers used to initialize localization.
class Backend
{
public:
    Backend();
    bool init_localization_mode();
    bool build_voxelmaps();
    bool run_relocalization(PointCloudType::Ptr scan, const double &lidar_beg_time, Eigen::Matrix4d &imu_pose);

    std::shared_ptr<Relocalization> relocalization;
    std::string map_path;
    std::string globalmap_path = PCD_FILE_DIR("globalmap.pcd");
    std::string trajectory_path = PCD_FILE_DIR("trajectory.pcd");
    std::string scd_path = PCD_FILE_DIR("scancontext/");
    BBS3DOptions bbs3d_option;
    bool voxelmap_auto_build = true;
    bool relocalization_ready = false;
    PointCloudType::Ptr global_map;
};
