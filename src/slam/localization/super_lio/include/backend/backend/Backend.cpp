#include "Backend.h"

Backend::Backend()
{
    save_resolution = 0.1;

    gnss = make_shared<GnssProcessor>();

    keyframe_pose6d_unoptimized.reset(new pcl::PointCloud<PointXYZIRPYT>());
    keyframe_pose6d_optimized.reset(new pcl::PointCloud<PointXYZIRPYT>());
    keyframe_scan.reset(new deque<PointCloudType::Ptr>());

    backend = std::make_shared<FactorGraphOptimization>(keyframe_pose6d_optimized, keyframe_scan, gnss);
    relocalization = make_shared<Relocalization>();
    isABlocked.store(false);
    loopClosure = make_shared<LoopClosure>(relocalization->sc_manager, cv, isABlocked);
}

Backend::~Backend()
{
    if (loopthread.joinable())
        loopthread.join();
}

void Backend::init_system_mode()
{
    loopthread = std::thread(&Backend::loopClosureThread, this);

    FileOperation::createDirectoryOrRecreate(keyframe_path);
    FileOperation::createDirectoryOrRecreate(scd_path);
}

bool Backend::init_localization_mode()
{
    relocalization_ready = false;

    if (access(globalmap_path.c_str(), F_OK) != 0)
    {
        LOG_ERROR("File not exist! Please check the \"globalmap_path\".");
        std::exit(100);
    }

    Timer2 timer;
    global_map.reset(new PointCloudType());
    pcl::io::loadPCDFile(globalmap_path, *global_map);
    if (global_map->points.size() < 5000)
    {
        LOG_WARN("Too few point clouds! Please check the map file.");
        // std::exit(100);
    }
    LOG_WARN("Load pcd successfully! There are %lu points in map. Cost time %fms.", global_map->points.size(), timer.elapsedLast());

    pcl::io::loadPCDFile(trajectory_path, *relocalization->trajectory_poses);
    if (relocalization->trajectory_poses->points.size() < 10)
    {
        LOG_ERROR("Too few point clouds! Please check the trajectory file.");
        std::exit(100);
    }
    LOG_WARN("Load trajectory poses successfully! There are %lu poses.", relocalization->trajectory_poses->points.size());

    if (relocalization->algorithm_type == "scan_context" && !relocalization->load_keyframe_descriptor(scd_path))
    {
        relocalization->algorithm_type = "manually_set";
        LOG_ERROR("Load keyframe descriptor failed, set algorithm_type to manually_set!");
    }
    else
        LOG_WARN("Load keyframe descriptor successfully! There are %lu descriptors.", relocalization->sc_manager->polarcontexts_.size());

    if (!relocalization->load_prior_map(global_map))
    {
        std::exit(100);
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

void Backend::run(PointXYZIRPYT &this_pose6d, PointCloudType::Ptr &feats_undistort, PointCloudType::Ptr &submap_fix)
{
    {
        std::unique_lock<std::mutex> lock(m);
        while (isABlocked.load())
        {
            cv.wait(lock);
        }
    } // Mutex is released here

    this_pose6d.intensity = keyframe_pose6d_unoptimized->size();
    if (backend->is_keyframe(this_pose6d))
    {
        // save keyframe info
        keyframe_pose6d_unoptimized->push_back(this_pose6d);

        PointCloudType::Ptr this_keyframe(new PointCloudType());
        octreeDownsampling(feats_undistort, this_keyframe, 0.1);
        keyframe_scan->push_back(this_keyframe);

        if (save_keyframe_descriptor_en)
            relocalization->add_keyframe_descriptor(this_keyframe, scd_path);
        else
            relocalization->add_keyframe_descriptor(this_keyframe, "");

        if (save_keyframe_en)
            save_keyframe(feats_undistort, keyframe_scan->size());

        /*** loop closure ***/
        if (loop_closure_enable_flag && test_mode)
        {
            backend->get_keyframe_pose6d(loopClosure->copy_keyframe_pose6d);
            loopClosure->run(*keyframe_scan);
        }

        loopClosure->get_loop_constraint(loop_constraint);
        backend->run(loop_constraint, this_pose6d, submap_fix);
    }
}

void Backend::save_globalmap()
{
    auto keyframe_num = keyframe_scan->size();
    PointCloudType::Ptr pcl_map_full(new PointCloudType());
    if (keyframe_pose6d_optimized->size() == keyframe_num)
        for (auto i = 0; i < keyframe_num; ++i)
            *pcl_map_full += *pointcloudKeyframeToWorld((*keyframe_scan)[i], (*keyframe_pose6d_optimized)[i]);
    else if (keyframe_pose6d_unoptimized->size() == keyframe_num)
        for (auto i = 0; i < keyframe_num; ++i)
            *pcl_map_full += *pointcloudKeyframeToWorld((*keyframe_scan)[i], (*keyframe_pose6d_unoptimized)[i]);
    else
        LOG_ERROR("no keyframe_num matched, when save global map!");

    octreeDownsampling(pcl_map_full, pcl_map_full, save_resolution);
    savePCDFile(globalmap_path, *pcl_map_full);
    LOG_WARN("Success save global map to %s.", globalmap_path.c_str());
}

void Backend::save_pgm(const double &pgm_resolution, const float &min_z, const float &max_z)
{
    pcl::PointCloud<PointXYZIRPYT>::Ptr keyframe_pose6d(new pcl::PointCloud<PointXYZIRPYT>());
    PointCloudType::Ptr global_map(new PointCloudType());
    pcl::io::loadPCDFile(trajectory_path, *keyframe_pose6d);
    for (auto i = 0; i < keyframe_pose6d->size(); ++i)
    {
        PointCloudType::Ptr keyframe_pc(new PointCloudType());
        load_keyframe(keyframe_path, keyframe_pc, i, 6, min_z, max_z);
        // octreeDownsampling(keyframe_pc, keyframe_pc, 0.1);
        *global_map += *pointcloudKeyframeToWorld(keyframe_pc, (*keyframe_pose6d)[i]);
    }
    Pcd2Pgm mg(pgm_resolution, map_path + "/map");
    mg.convert_from_pcd(global_map);
    mg.convert_to_pgm();
    LOG_WARN("Success save pgm to %s.", map_path.c_str());
}

void Backend::save_trajectory()
{
#ifndef NO_LOGER
    FILE *file_pose_unoptimized = fopen(DEBUG_FILE_DIR("keyframe_pose.txt").c_str(), "w");
    fprintf(file_pose_unoptimized, "# keyframe trajectory unoptimized\n# timestamp tx ty tz qx qy qz qw\n");
    int pose_num = keyframe_pose6d_unoptimized->points.size();
    for (auto i = 0; i < pose_num; ++i)
    {
        const auto &pose = keyframe_pose6d_unoptimized->points[i];
        const auto &lidar_rot = EigenMath::RPY2Quaternion(V3D(pose.roll, pose.pitch, pose.yaw));
        const auto &lidar_pos = V3D(pose.x, pose.y, pose.z);
        LogAnalysis::save_trajectory(file_pose_unoptimized, lidar_pos, lidar_rot, pose.time);
    }
    LOG_WARN("Success save global unoptimized lidar poses to file ...");
    fclose(file_pose_unoptimized);

    FILE *file_pose_optimized = fopen(DEBUG_FILE_DIR("keyframe_pose_optimized.txt").c_str(), "w");
    fprintf(file_pose_optimized, "# keyframe trajectory optimized\n# timestamp tx ty tz qx qy qz qw\n");
    pose_num = keyframe_pose6d_optimized->points.size();
    for (auto i = 0; i < pose_num; ++i)
    {
        const auto &pose = keyframe_pose6d_optimized->points[i];
        const auto &lidar_rot = EigenMath::RPY2Quaternion(V3D(pose.roll, pose.pitch, pose.yaw));
        const auto &lidar_pos = V3D(pose.x, pose.y, pose.z);
        LogAnalysis::save_trajectory(file_pose_optimized, lidar_pos, lidar_rot, pose.time);
    }
    LOG_WARN("Success save global optimized lidar poses to file ...");
    fclose(file_pose_optimized);
#endif

    pcl::PCDWriter pcd_writer;
    pcd_writer.writeBinary(trajectory_path, *keyframe_pose6d_optimized);
    LOG_WARN("Success save trajectory poses to %s.", trajectory_path.c_str());

#ifndef NO_LOGER
    if (map_path.compare("") != 0)
        fs::copy_file(DEBUG_FILE_DIR("keyframe_pose_optimized.txt"), map_path + "/keyframe_pose_optimized.txt", fs::copy_options::overwrite_existing);
#endif
}

void Backend::save_trajectory_to_other_frame(const QD &extR, const V3D &extP, const std::string& frame)
{
    FILE *file_pose_unoptimized_imu = fopen(DEBUG_FILE_DIR("keyframe_pose_" + frame + ".txt").c_str(), "w");
    fprintf(file_pose_unoptimized_imu, "# keyframe trajectory unoptimized in %s frame\n# timestamp tx ty tz qx qy qz qw\n", frame.c_str());
    int pose_num = keyframe_pose6d_unoptimized->points.size();
    for (auto i = 0; i < pose_num; ++i)
    {
        const auto &pose = keyframe_pose6d_unoptimized->points[i];
        const auto &lidar_rot = EigenMath::RPY2Quaternion(V3D(pose.roll, pose.pitch, pose.yaw));
        const auto &lidar_pos = V3D(pose.x, pose.y, pose.z);
        QD imu_rot;
        V3D imu_pos;
        poseTransformFrame2(lidar_rot, lidar_pos, extR, extP, imu_rot, imu_pos);
        LogAnalysis::save_trajectory(file_pose_unoptimized_imu, imu_pos, imu_rot, pose.time);
    }
    LOG_WARN("Success save global unoptimized %s poses to file ...", frame.c_str());
    fclose(file_pose_unoptimized_imu);

    FILE *file_pose_optimized_imu = fopen(DEBUG_FILE_DIR("keyframe_pose_optimized_" + frame + ".txt").c_str(), "w");
    fprintf(file_pose_optimized_imu, "# keyframe trajectory optimized in %s frame\n# timestamp tx ty tz qx qy qz qw\n", frame.c_str());
    pose_num = keyframe_pose6d_optimized->points.size();
    for (auto i = 0; i < pose_num; ++i)
    {
        const auto &pose = keyframe_pose6d_optimized->points[i];
        const auto &lidar_rot = EigenMath::RPY2Quaternion(V3D(pose.roll, pose.pitch, pose.yaw));
        const auto &lidar_pos = V3D(pose.x, pose.y, pose.z);
        QD imu_rot;
        V3D imu_pos;
        poseTransformFrame2(lidar_rot, lidar_pos, extR, extP, imu_rot, imu_pos);
        LogAnalysis::save_trajectory(file_pose_optimized_imu, imu_pos, imu_rot, pose.time);
    }
    LOG_WARN("Success save global optimized %s poses to file ...", frame.c_str());
    fclose(file_pose_optimized_imu);
}

PointCloudType::Ptr Backend::get_submap_visual(float globalMapVisualizationSearchRadius, float globalMapVisualizationPoseDensity, float globalMapVisualizationLeafSize, bool showOptimizedPose)
{
    pcl::PointCloud<PointXYZIRPYT>::Ptr keyframe_pose(new pcl::PointCloud<PointXYZIRPYT>());
    backend->pose_mtx.lock();
    if (showOptimizedPose)
        *keyframe_pose = *keyframe_pose6d_optimized;
    else
        *keyframe_pose = *keyframe_pose6d_unoptimized;
    backend->pose_mtx.unlock();

    if (keyframe_pose->points.empty())
        return PointCloudType::Ptr(nullptr);

    pcl::KdTreeFLANN<PointXYZIRPYT>::Ptr kdtreeGlobalMap(new pcl::KdTreeFLANN<PointXYZIRPYT>());
    pcl::PointCloud<PointXYZIRPYT>::Ptr globalMapKeyPoses(new pcl::PointCloud<PointXYZIRPYT>());
    pcl::PointCloud<PointXYZIRPYT>::Ptr globalMapKeyPosesDS(new pcl::PointCloud<PointXYZIRPYT>());
    PointCloudType::Ptr globalMapKeyFrames(new PointCloudType());
    PointCloudType::Ptr globalMapKeyFramesDS(new PointCloudType());

    // search near key frames to visualize
    std::vector<int> pointSearchIndGlobalMap;
    std::vector<float> pointSearchSqDisGlobalMap;
    kdtreeGlobalMap->setInputCloud(keyframe_pose);
    kdtreeGlobalMap->radiusSearch(keyframe_pose->back(), globalMapVisualizationSearchRadius, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);

    for (int i = 0; i < (int)pointSearchIndGlobalMap.size(); ++i)
        globalMapKeyPoses->push_back(keyframe_pose->points[pointSearchIndGlobalMap[i]]);
    // downsample near selected key frames pose
    pcl::VoxelGrid<PointXYZIRPYT> downSizeFilterGlobalMapKeyPoses;
    downSizeFilterGlobalMapKeyPoses.setLeafSize(globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity);
    downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
    downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);
    for (auto &pt : globalMapKeyPosesDS->points)
    {
        kdtreeGlobalMap->nearestKSearch(pt, 1, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap);
        pt.intensity = keyframe_pose->points[pointSearchIndGlobalMap[0]].intensity;
    }

    for (int i = 0; i < (int)globalMapKeyPosesDS->size(); ++i)
    {
        if (pointDistance(globalMapKeyPosesDS->points[i], keyframe_pose->back()) > globalMapVisualizationSearchRadius)
            continue;
        int thisKeyInd = (int)globalMapKeyPosesDS->points[i].intensity;
        *globalMapKeyFrames += *pointcloudKeyframeToWorld((*keyframe_scan)[thisKeyInd], keyframe_pose->points[thisKeyInd]);
    }
    // downsample key frames
    octreeDownsampling(globalMapKeyFrames, globalMapKeyFramesDS, globalMapVisualizationLeafSize);
    return globalMapKeyFramesDS;
}

bool Backend::run_relocalization(PointCloudType::Ptr scan, const double &lidar_beg_time, Eigen::Matrix4d &imu_pose)
{
    run_relocalization_thread = true;
    if (relocalization->run(scan, imu_pose, lidar_beg_time))
        return true;
    run_relocalization_thread = false;
    return false;
}

void Backend::load_keyframe(const std::string &keyframe_path, PointCloudType::Ptr keyframe_pc,
                   int keyframe_cnt, int num_digits,
                   const float &min_z, const float &max_z)
{
    std::ostringstream out;
    out << std::internal << std::setfill('0') << std::setw(num_digits) << keyframe_cnt;
    std::string keyframe_idx = out.str();
    string keyframe_file(keyframe_path + keyframe_idx + string(".pcd"));
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_pc(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::io::loadPCDFile(keyframe_file, *tmp_pc);
    for (auto i = 0; i < tmp_pc->points.size(); ++i)
    {
        if (tmp_pc->points[i].z < min_z || tmp_pc->points[i].z > max_z)
            continue;
        if (tmp_pc->points[i].x < 0)
            continue;
        PointType point;
        pcl::copyPoint(tmp_pc->points[i], point);
        keyframe_pc->points.emplace_back(point);
    }
}

void Backend::save_keyframe(PointCloudType::Ptr &cloud, int keyframe_cnt, int num_digits)
{
    std::ostringstream out;
    out << std::internal << std::setfill('0') << std::setw(num_digits) << keyframe_cnt - 1;
    std::string keyframe_idx = out.str();
    string keyframe_file(keyframe_path + keyframe_idx + string(".pcd"));
    savePCDFile(keyframe_file, *cloud);
}

void Backend::loopClosureThread()
{
    if (loop_closure_enable_flag == false)
        return;

    LOG_WARN("loop closure enabled!");
    while (test_mode == false)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(loop_closure_interval));
        backend->get_keyframe_pose6d(loopClosure->copy_keyframe_pose6d);
        loopClosure->run(*keyframe_scan);
    }
}
