#pragma once
#include <cmath>
#include <atomic>
#include <cstdint>
#include <iostream>
#include <exception>
#include <string>
#include <vector>
#include <Eigen/Core>
#ifdef USE_CUDA
#include <gpu_bbs3d/bbs3d.cuh>
#else
#include <cpu_bbs3d/bbs3d.hpp>
#endif
#ifdef BBS_ENABLE_OPENCL
#include <opencl_bbs3d/bbs3d.hpp>
#endif
#include <pointcloud_iof/pcl_eigen_converter.hpp>
// #include <pointcloud_iof/pcd_loader.hpp>

struct BBS3DOptions
{
    std::string tar_path;

    // 3D-BBS parameters
    double min_level_res;
    int max_level;

    // angular search range
    std::vector<double> min_rpy;
    std::vector<double> max_rpy;
    bool gravity_align = false;

    // score threshold percentage
    double score_threshold_percentage;

    int num_threads;

    // downsample
    double tar_leaf_size, src_leaf_size;
    double min_scan_range, max_scan_range;

    // timeout
    int timeout_msec;

    // align
    bool use_gicp;

    std::string backend = "opencl";
    int64_t opencl_batch_size = 10000;
    bool opencl_fallback_cpu = true;
    bool opencl_verify = false;
};

class BBS3D
{
public:
    // Sticky for this algorithm instance; a new START creates a fresh instance.
    void request_cancel() { cancelled_.store(true); }
    bool is_cancelled() const { return cancelled_.load(); }

    bool load_config(const BBS3DOptions &bbs3d_option)
    {
        backend_ = bbs3d_option.backend;
        fallback_cpu_ = bbs3d_option.opencl_fallback_cpu;
        fallback_reason_.clear();
#ifdef BBS_ENABLE_OPENCL
        // The engine holds a reference to bbs3d_ptr: release it before reloading a map.
        opencl_engine_.reset();
#endif
        tar_path = bbs3d_option.tar_path;

        std::cout << "[BBS3D] Loading 3D-BBS parameters..." << std::endl;
        min_level_res = bbs3d_option.min_level_res;
        max_level = bbs3d_option.max_level;

        if (min_level_res == 0.0 || max_level == 0)
        {
            std::cout << "[ERROR] Set min_level and num_layers except for 0" << std::endl;
            return false;
        }

        std::cout << "[BBS3D] Loading angular search range..." << std::endl;
        if (bbs3d_option.min_rpy.size() == 3 && bbs3d_option.max_rpy.size() == 3)
        {
            min_rpy = to_eigen(bbs3d_option.min_rpy);
            max_rpy = to_eigen(bbs3d_option.max_rpy);
        }
        else
        {
            std::cout << "[ERROR] Set min_rpy and max_rpy correctly" << std::endl;
            return false;
        }
        gravity_align = bbs3d_option.gravity_align;

        std::cout << "[BBS3D] Loading score threshold percentage..." << std::endl;
        score_threshold_percentage = bbs3d_option.score_threshold_percentage;

        std::cout << "[BBS3D] Loading downsample parameters..." << std::endl;
        tar_leaf_size = bbs3d_option.tar_leaf_size;
        src_leaf_size = bbs3d_option.src_leaf_size;
        min_scan_range = bbs3d_option.min_scan_range;
        max_scan_range = bbs3d_option.max_scan_range;

        timeout_msec = bbs3d_option.timeout_msec;
        use_gicp = bbs3d_option.use_gicp;

        // ====3D-BBS====
#ifdef USE_CUDA
        bbs3d_ptr = std::make_unique<gpu::BBS3D>();
        std::cout << "[Localize] USE CUDA!" << std::endl;
#else
        bbs3d_ptr = std::make_unique<cpu::BBS3D>();
        bbs3d_ptr->set_cancel_check([this]() { return is_cancelled(); });
        std::cout << "[Localize] NOT USE CUDA!" << std::endl;
#endif

        // Set target points
        std::cout << "[Voxel map] Creating hierarchical voxel map..." << std::endl;
        auto initi_t1 = std::chrono::high_resolution_clock::now();
        if (bbs3d_ptr->set_voxelmaps_coords(tar_path))
        {
            std::cout << "[Voxel map] Loaded voxelmaps coords directly" << std::endl;
        }
        else
        {
            // bbs3d_ptr->set_tar_points(tar_points, min_level_res, max_level);
            // bbs3d_ptr->set_trans_search_range(tar_points);
            std::cout << "\033[31m" << "[Voxel map] Loaded voxelmaps failed!" << "\033[0m" << std::endl;
            return false;
        }
        auto init_t2 = std::chrono::high_resolution_clock::now();
        double init_time = std::chrono::duration_cast<std::chrono::nanoseconds>(init_t2 - initi_t1).count() / 1e6;
        std::cout << "[Voxel map] Execution time: " << init_time << "[msec] " << std::endl;

#ifdef USE_CUDA
        bbs3d_ptr->set_angular_search_range(min_rpy.cast<float>(), max_rpy.cast<float>());
        bbs3d_ptr->set_score_threshold_percentage(static_cast<float>(score_threshold_percentage));
#else
        bbs3d_ptr->set_angular_search_range(min_rpy, max_rpy);
        bbs3d_ptr->set_score_threshold_percentage(score_threshold_percentage);
#endif
        if (timeout_msec > 0)
        {
            bbs3d_ptr->enable_timeout();
            bbs3d_ptr->set_timeout_duration_in_msec(timeout_msec);
        }

#ifndef USE_CUDA
        bbs3d_ptr->set_num_threads(bbs3d_option.num_threads);
#endif
        if (backend_ == "opencl")
        {
#ifdef BBS_ENABLE_OPENCL
            oclbbs::SearchOptions options;
            options.use_opencl = true;
            options.batch_size = static_cast<size_t>(bbs3d_option.opencl_batch_size);
            options.verify_gpu = bbs3d_option.opencl_verify;
            options.fallback_cpu = fallback_cpu_;
            opencl_engine_ = std::make_unique<oclbbs::CudaStyleSearch>(*bbs3d_ptr, options);
            if (!opencl_engine_->gpu_available())
            {
                fallback_reason_ = opencl_engine_->setup_error();
                opencl_engine_.reset();
            }
            else
            {
                std::cout << "[BBS_RUNTIME] requested=opencl active=opencl map_bytes="
                          << opencl_engine_->gpu_map_bytes() << " setup_ms="
                          << opencl_engine_->runtime_setup_ms << " batch=" << options.batch_size
                          << " fallback_cpu=" << fallback_cpu_ << std::endl;
            }
#else
            fallback_reason_ = "localization built without ENABLE_OPENCL_BBS";
#endif
            if (!fallback_reason_.empty())
            {
                std::cerr << "[BBS_BACKEND] requested=opencl active="
                          << (fallback_cpu_ ? "cpu" : "unavailable")
                          << " reason=" << fallback_reason_ << std::endl;
                if (!fallback_cpu_) return false;
            }
        }
        else
            std::cout << "[BBS_RUNTIME] requested=cpu active=cpu" << std::endl;
        return true;
    }

    bool run(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud, Eigen::Matrix4d &lidar_pose_mat)
    {
#ifdef USE_CUDA
        std::vector<Eigen::Vector3f> src_points;
#else
        std::vector<Eigen::Vector3d> src_points;
#endif
        if (is_cancelled()) return false;
        pciof::pcl_to_eigen(src_cloud, src_points);
        if (src_points.empty() || !bbs3d_ptr)
        {
            std::cerr << "[BBS_BACKEND_ERROR] Empty input or map not initialized" << std::endl;
            return false;
        }
        for (const auto &point : src_points)
            if (!point.allFinite())
            {
                std::cerr << "[BBS_BACKEND_ERROR] Nonfinite input point" << std::endl;
                return false;
            }
        std::cout << "[Localize] bbs3d start localize." << std::endl;
        try
        {
#ifdef BBS_ENABLE_OPENCL
            if (opencl_engine_)
            {
                opencl_engine_->prepare(src_points);
                if (is_cancelled()) return false;
                const auto result = opencl_engine_->localize();
                const bool gpu_used = result.gpu_candidates != 0;
                std::cout << "[BBS_SEARCH_STATS] requested=opencl active="
                          << (gpu_used ? (result.cpu_batches ? "opencl+cpu" : "opencl") : "cpu")
                          << " search_ms=" << result.search_ms
                          << " scan_setup_ms=" << result.scan_setup_ms
                          << " gpu_kernel_ms=" << result.gpu_kernel_ms
                          << " boundary_ms=" << result.boundary_ms
                          << " gpu_candidates=" << result.gpu_candidates
                          << " cpu_batches=" << result.cpu_batches
                          << " gpu_failures=" << result.gpu_failures
                          << " success=" << result.success
                          << " cancelled=" << result.cancelled
                          << " timed_out=" << result.timed_out << std::endl;
                if (result.cancelled || is_cancelled()) {
                    std::cout << "[BBS_CANCELLED] backend=opencl" << std::endl;
                    return false;
                }
                if (!gpu_used || result.gpu_failures || result.cpu_batches)
                    std::cerr << "[BBS_CPU_FALLBACK] reason="
                              << (!result.gpu_error.empty() ? result.gpu_error :
                                  (!result.gpu_eligible ? "input requires CPU precision" :
                                   "candidate range requires CPU precision, or no candidates scored"))
                              << std::endl;
                // A timeout or insufficient score is a normal failed attempt;
                // do not restart the search on CPU with a fresh timeout budget.
                if (!result.success) return false;
                lidar_pose_mat = result.pose;
                return true;
            }
#endif
            std::cout << "[BBS_SEARCH_STATS] requested=" << backend_ << " active=cpu"
                      << (fallback_reason_.empty() ? "" : " reason=" + fallback_reason_) << std::endl;
            bbs3d_ptr->set_src_points(src_points);
            bbs3d_ptr->localize();
        }
        catch (const std::exception &error)
        {
            std::cerr << "[BBS_BACKEND_ERROR] " << error.what() << std::endl;
            return false;
        }

        std::cout << "[Localize] Execution time: " << bbs3d_ptr->get_elapsed_time() << "[msec] " << std::endl;
        std::cout << "[Localize] Score: " << bbs3d_ptr->get_best_score() << std::endl;

        if (is_cancelled()) {
            std::cout << "[BBS_CANCELLED] backend=" << backend_ << std::endl;
            return false;
        }
        if (!bbs3d_ptr->has_localized())
        {
            if (bbs3d_ptr->has_timed_out())
                std::cout << "[Failed] Localization timed out." << std::endl;
            else
                std::cout << "[Failed] Score is below the threshold." << std::endl;
            return false;
        }

        lidar_pose_mat = bbs3d_ptr->get_global_pose().cast<double>();
        return true;
    }

private:
    Eigen::Vector3d to_eigen(const std::vector<double>& vec) {
        Eigen::Vector3d e_vec;
        for (int i = 0; i < 3; ++i) {
            if (vec[i] == 6.28) {
            e_vec(i) = 2 * M_PI;
            } else {
            e_vec(i) = vec[i];
            }
        }
        return e_vec;
    }

public:
#ifdef USE_CUDA
    std::unique_ptr<gpu::BBS3D> bbs3d_ptr;
#else
    std::unique_ptr<cpu::BBS3D> bbs3d_ptr;
#endif

    std::string tar_path;

    // 3D-BBS parameters
    double min_level_res;
    int max_level;

    // angular search range
    Eigen::Vector3d min_rpy;
    Eigen::Vector3d max_rpy;
    bool gravity_align = false;

    // score threshold percentage
    double score_threshold_percentage;

    // downsample
    float tar_leaf_size, src_leaf_size;
    double min_scan_range, max_scan_range;

    // timeout
    int timeout_msec;

    // align
    bool use_gicp;

private:
    std::atomic<bool> cancelled_{false};
    std::string backend_ = "cpu";
    bool fallback_cpu_ = true;
    std::string fallback_reason_;
#ifdef BBS_ENABLE_OPENCL
    std::unique_ptr<oclbbs::CudaStyleSearch> opencl_engine_;
#endif
};
