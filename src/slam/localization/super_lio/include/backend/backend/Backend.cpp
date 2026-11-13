#include "Backend.h"

Backend::Backend() : relocalization(std::make_shared<Relocalization>()) {}

bool Backend::init_localization_mode()
{
    relocalization_ready = false;
    // Manual/SC initialization also consumes this option, without loading the
    // BBS search engine or requiring its voxel index.
    relocalization->bbs3d.gravity_align = bbs3d_option.gravity_align;

    if (access(globalmap_path.c_str(), F_OK) != 0)
    {
        LOG_ERROR("File not exist! Please check the \"globalmap_path\".");
        throw std::runtime_error("Cannot initialize localization prior map: " + globalmap_path);
    }

    Timer2 timer;
    global_map.reset(new PointCloudType());
    if (pcl::io::loadPCDFile(globalmap_path, *global_map) < 0 || global_map->empty())
        throw std::runtime_error("Invalid or empty prior map: " + globalmap_path);
    if (global_map->points.size() < 5000)
    {
        LOG_WARN("Too few point clouds! Please check the map file.");
        // std::exit(100);
    }
    LOG_WARN("Load pcd successfully! There are %lu points in map. Cost time %fms.", global_map->points.size(), timer.elapsedLast());

    // Only Scan Context consumes recorded poses and descriptors. BBS3D and
    // manually supplied poses need only the fixed point-cloud map.
    if (relocalization->algorithm_type == "scan_context")
    {
        const bool has_trajectory =
            pcl::io::loadPCDFile(trajectory_path, *relocalization->trajectory_poses) >= 0 &&
            relocalization->trajectory_poses->size() >= 10;
        if (!has_trajectory || !relocalization->load_keyframe_descriptor(scd_path))
        {
            relocalization->algorithm_type = "manually_set";
            LOG_ERROR("Scan Context prior is missing; falling back to manually_set.");
        }
    }

    if (!relocalization->load_prior_map(global_map))
    {
        throw std::runtime_error("Cannot initialize localization prior map: " + globalmap_path);
    }
    relocalization_ready = true;

    // 5) BBS3D hierarchical voxel map (auto-build when missing).
    //    Only bbs3d needs it; scan_context / manually_set must not be blocked by it.
    if (relocalization->algorithm_type == "bbs3d")
    {
        const std::string resolved_dir = map_path;
        bbs3d_option.tar_path = resolved_dir;
        const std::string voxelmap_path = resolved_dir + "/voxelmaps_coords";
        bool voxelmap_ok = true;
        if (!fs::exists(voxelmap_path))
        {
            if (voxelmap_auto_build)
            {
                LOG_WARN("voxelmaps_coords not found at %s, building it now...", voxelmap_path.c_str());
                voxelmap_ok = build_voxelmaps();
            }
            else
            {
                voxelmap_ok = false;
                LOG_ERROR("voxelmaps_coords not found at %s and voxelmap_auto_build is disabled.",
                          voxelmap_path.c_str());
            }
        }

        if (!voxelmap_ok || !relocalization->bbs3d.load_config(bbs3d_option))
        {
            // Do not return false: the prior map is loaded, so /initialpose can still work.
            LOG_ERROR("BBS3D init failed (voxelmaps_coords missing or corrupted at %s), "
                      "fallback to manually_set.",
                      voxelmap_path.c_str());
            if (bbs3d_option.backend == "opencl" && !bbs3d_option.opencl_fallback_cpu)
                throw std::runtime_error("Requested OpenCL BBS3D backend is unavailable");
            relocalization->algorithm_type = "manually_set";
        }
    }

    LOG_WARN("Relocalization ready. algorithm_type = %s", relocalization->algorithm_type.c_str());
    return relocalization_ready;
}

bool Backend::build_voxelmaps()
{
    PointCloudType::Ptr cloud(new PointCloudType());
    if (pcl::io::loadPCDFile(globalmap_path, *cloud) < 0 || cloud->empty())
    {
        LOG_ERROR("build_voxelmaps: cannot load %s", globalmap_path.c_str());
        return false;
    }

    std::vector<Eigen::Vector3d> tar_points;
    tar_points.reserve(cloud->size());
    for (const auto &p : cloud->points)
    {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
            continue;
        tar_points.emplace_back(static_cast<double>(p.x), static_cast<double>(p.y), static_cast<double>(p.z));
    }
    if (tar_points.empty())
    {
        LOG_ERROR("build_voxelmaps: no valid points in %s", globalmap_path.c_str());
        return false;
    }

    const double min_level_res = bbs3d_option.min_level_res;
    const int max_level = bbs3d_option.max_level;
    if (min_level_res <= 0.0 || max_level <= 0)
    {
        LOG_ERROR("build_voxelmaps: invalid bbs3d.min_level_res=%f / bbs3d.max_level=%d",
                  min_level_res, max_level);
        return false;
    }

    Timer2 timer;
    const std::string dir = map_path;
    cpu::BBS3D builder;
    builder.set_tar_points(tar_points, min_level_res, max_level);
    if (!builder.save_voxel_params(dir) || !builder.save_voxelmaps_pcd(dir))
    {
        LOG_ERROR("build_voxelmaps: save failed under %s", dir.c_str());
        return false;
    }

    LOG_WARN("build_voxelmaps done: %s/voxelmaps_coords (min_level_res=%.3f, max_level=%d, %lu points, %.1f ms).",
             dir.c_str(), min_level_res, max_level, tar_points.size(), timer.elapsedLast());
    return true;
}

bool Backend::run_relocalization(PointCloudType::Ptr scan, const double &lidar_beg_time, const V3D &mean_acc, Eigen::Matrix4d &imu_pose)
{
    return relocalization->run(scan, imu_pose, lidar_beg_time, mean_acc);
}
