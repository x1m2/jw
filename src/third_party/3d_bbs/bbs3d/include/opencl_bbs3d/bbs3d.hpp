#pragma once
#include <cpu_bbs3d/bbs3d.hpp>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace oclbbs {
using Transform = DiscreteTransformation<double>;
struct Pose;
class Scorer;
struct SearchOptions {
 bool use_opencl=false;
 bool verify_gpu=false;
 bool fallback_cpu=true;
 size_t batch_size=10000;
 size_t gpu_min_batch=1;
 size_t max_queue=0; // 0 matches the original CUDA implementation: no fixed queue cap
 int budget_ms=-1;
 std::string kernel_path; // Empty selects the built-in kernel.
};
struct SearchResult {
 bool completed=false,success=false,timed_out=false,cancelled=false,resource_limit=false,gpu_available=false,gpu_eligible=false;
 int score=0,threshold=0;
 Eigen::Matrix4d pose=Eigen::Matrix4d::Identity();
 double search_ms=0,scan_setup_ms=0,cpu_score_ms=0,gpu_score_ms=0,gpu_kernel_ms=0,boundary_ms=0,verify_ms=0;
 uint64_t scored=0,generated=0,popped=0,pruned=0,leaves=0,cpu_batches=0,gpu_batches=0,gpu_candidates=0,boundary_points=0,verified_candidates=0,gpu_failures=0;
 size_t max_queue=0;
 std::string gpu_error;
};
class CudaStyleSearch {
 cpu::BBS3D& cpu_;
 SearchOptions options_;
 std::unique_ptr<Scorer> gpu_;
 std::vector<Eigen::Vector3d> points_;
 Eigen::Vector3d abs_point_max_=Eigen::Vector3d::Zero();
 bool eligible_=false;
 std::string setup_error_;
 double scan_ms_=0;
 void cpu_scores(std::vector<Transform>& candidates, size_t first, size_t count,
                 const std::vector<cpu::AngularInfo>& angles, SearchResult& result);
 bool pose_supported(const std::vector<Pose>& poses);
 void score_range(std::vector<Transform>& candidates, size_t first, size_t count,
                  const std::vector<cpu::AngularInfo>& angles, SearchResult& result);
public:
 double runtime_setup_ms=0;
 explicit CudaStyleSearch(cpu::BBS3D& cpu, SearchOptions options);
 ~CudaStyleSearch();
 bool gpu_available() const;
 std::string map_storage() const;
 uint64_t gpu_map_bytes() const;
 uint64_t cpu_hash_bytes() const;
 const std::string& setup_error() const;
 double prepare(const std::vector<Eigen::Vector3d>& points);
 SearchResult localize();
};
} // namespace oclbbs
