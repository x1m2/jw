#include <opencl_bbs3d/bbs3d.hpp>
#include "scoring_helpers.hpp"
#include <algorithm>
#include <queue>
#include <stdexcept>
#include <utility>

namespace oclbbs {
class ScoreMismatch:public std::runtime_error {public:using std::runtime_error::runtime_error;};
void CudaStyleSearch::cpu_scores(std::vector<Transform>& ts,size_t first,size_t count,const std::vector<cpu::AngularInfo>& angles,SearchResult& result){
  if(options_.use_opencl && !options_.fallback_cpu)
    throw std::runtime_error("OpenCL scoring unavailable and CPU fallback is disabled");
  double start=clock_ms();int threads=Access::threads(cpu_);
#pragma omp parallel for num_threads(threads)
  for(int i=0;i<int(count);++i){auto& t=ts[first+i];t.score=Access::score(cpu_,t,angles[t.level],points_);}
  result.cpu_score_ms+=clock_ms()-start;result.cpu_batches++;
 }

bool CudaStyleSearch::pose_supported(const std::vector<Pose>& poses) {
  auto& map=Access::map(cpu_);
  for(const auto& pose:poses){
   for(int row=0;row<3;++row){
    const auto& r=pose.rows[row];double magnitude=std::abs(double(r[3]));
    for(int k=0;k<3;++k)magnitude+=std::abs(double(r[k]))*abs_point_max_[k];
    if(!std::isfinite(magnitude)||magnitude/map.voxelmaps_res_[pose.meta[0]]>1e6)return false;
   }
  }return true;
 }

void CudaStyleSearch::score_range(std::vector<Transform>& ts,size_t first,size_t count,const std::vector<cpu::AngularInfo>& angles,SearchResult& result){
  result.scored+=count;
  if(!gpu_||!eligible_||count<options_.gpu_min_batch){cpu_scores(ts,first,count,angles,result);return;}
  double begin=clock_ms();
  try {
   std::vector<Transform> chunk(ts.begin()+first,ts.begin()+first+count);
   auto poses=make_poses(chunk,angles,Access::map(cpu_));
   if(!pose_supported(poses)){cpu_scores(ts,first,count,angles,result);return;}
   auto raw=gpu_->score(poses,false);
   double correction_start=clock_ms();auto scores=correct(cpu_,chunk,angles,points_,raw);
   result.boundary_ms+=clock_ms()-correction_start;
   result.gpu_kernel_ms+=raw.kernel_ms;
   for(const auto& value:raw.values)result.boundary_points+=value[2];
   if(options_.verify_gpu){
    double verify_start=clock_ms();size_t mismatches=0;int threads=Access::threads(cpu_);
#pragma omp parallel for num_threads(threads) reduction(+:mismatches)
    for(int i=0;i<int(count);++i)if(scores[i]!=Access::score(cpu_,chunk[i],angles[chunk[i].level],points_))mismatches++;
    result.verify_ms+=clock_ms()-verify_start;result.verified_candidates+=count;
    if(mismatches)throw ScoreMismatch("Corrected OpenCL score differs from original CPU: "+std::to_string(mismatches));
   }
   for(size_t i=0;i<count;++i)ts[first+i].score=scores[i];
   result.gpu_batches++;result.gpu_candidates+=count;result.gpu_score_ms+=clock_ms()-begin;
  }catch(const ScoreMismatch&){throw;}
  catch(const std::exception& error){
   result.gpu_failures++;result.gpu_error=error.what();gpu_.reset();
   cpu_scores(ts,first,count,angles,result);
  }
 }

CudaStyleSearch::CudaStyleSearch(cpu::BBS3D& cpu,SearchOptions options):cpu_(cpu),options_(std::move(options)){
  if(!options_.batch_size||options_.batch_size>65536||!options_.gpu_min_batch)throw std::runtime_error("Invalid search batch/queue limits");
  double start=clock_ms();
  if(options_.use_opencl){
   try{gpu_=std::make_unique<Scorer>(Access::map(cpu_),options_.kernel_path);}
   catch(const std::exception& error){setup_error_=error.what();}
  }
  runtime_setup_ms=clock_ms()-start;
 }

double CudaStyleSearch::prepare(const std::vector<Eigen::Vector3d>& points){
  double start=clock_ms();if(points.empty())throw std::runtime_error("Empty source scan");
  points_=points;abs_point_max_.setZero();eligible_=true;double max_norm=0;
  std::vector<Float4> floats;floats.reserve(points.size());
  for(const auto& p:points){
   if(!p.allFinite())throw std::runtime_error("Nonfinite source point");
   abs_point_max_=abs_point_max_.cwiseMax(p.cwiseAbs());max_norm=std::max(max_norm,p.norm());
   Float4 value={float(p.x()),float(p.y()),float(p.z()),0};
   for(int k=0;k<3;++k)if(double(value[k])!=p[k])eligible_=false;
   floats.push_back(value);
  }
  if(max_norm<1e-9)throw std::runtime_error("Degenerate zero-radius scan");
  for(double res:Access::map(cpu_).voxelmaps_res_)if(!std::isfinite(res)||res<=0||double(float(1.0/res))!=1.0/res)eligible_=false;
  cpu_.set_src_points(points);
  if(gpu_&&eligible_){
   try{gpu_->set_points(floats);}
   catch(const std::exception& error){setup_error_=error.what();gpu_.reset();}
  }
  scan_ms_=clock_ms()-start;return scan_ms_;
 }

SearchResult CudaStyleSearch::localize(){
  if(points_.empty())throw std::runtime_error("prepare must precede localize");
  SearchResult result;result.scan_setup_ms=scan_ms_;result.gpu_available=bool(gpu_);
  result.gpu_eligible=eligible_;result.gpu_error=setup_error_;
  const double start=clock_ms();
  int budget=options_.budget_ms>=0?options_.budget_ms:(Access::timeout_enabled(cpu_)?Access::timeout_ms(cpu_):0);
  auto expired=[&](){if(budget>0&&clock_ms()-start>=budget){result.timed_out=true;return true;}return false;};
  auto angles=Access::angles(cpu_);auto& map=Access::map(cpu_);
  const int rate=int(Access::rate(cpu_));
  if(rate<1||double(rate)!=Access::rate(cpu_))throw std::runtime_error("Noninteger voxel expansion rate");
  result.threshold=int(std::floor(points_.size()*Access::threshold(cpu_)));
  int best=result.threshold;Transform best_transform(best);bool found=false;
  auto initial=Access::initial(cpu_,angles.back());result.generated=initial.size();
  for(size_t first=0;first<initial.size();first+=options_.batch_size){
   if(expired())break;
   score_range(initial,first,std::min(options_.batch_size,initial.size()-first),angles,result);
  }
  std::priority_queue<Transform> queue;
  if(!result.timed_out)queue=std::priority_queue<Transform>(initial.begin(),initial.end());
  std::vector<Transform> stock;stock.reserve(options_.batch_size);
  auto queue_check=[&](){
   result.max_queue=std::max(result.max_queue,queue.size());
   if(options_.max_queue&&queue.size()>options_.max_queue){result.resource_limit=true;return false;}
   return true;
  };
  queue_check();
  // Same host responsibilities as original CUDA calc_scores/branch_stock.
  // Score all pending children before any further queue pop.
  auto flush=[&](){
   for(size_t first=0;first<stock.size();first+=options_.batch_size){
    if(expired())return false;
    size_t count=std::min(options_.batch_size,stock.size()-first);
    score_range(stock,first,count,angles,result);
    for(size_t i=first;i<first+count;++i){
     if(stock[i].score<best){result.pruned++;continue;}
     queue.push(stock[i]);
    }
    if(!queue_check())return false;
   }
   stock.clear();return true;
  };
  // The original CUDA loop tests only queue.empty(). Including stock here
  // also submits children created from the very last popped parent.
  while((!queue.empty()||!stock.empty())&&!result.timed_out&&!result.resource_limit){
   if(expired())break;
   if(queue.empty()){
    if(!flush())break;
    if(queue.empty())continue;
   }
   Transform trans=queue.top();queue.pop();result.popped++;
   // Preserve original CUDA order: flush older stock after the last pop,
   // then evaluate/branch the popped transform.
   if(queue.empty()&&!stock.empty()&&!flush())break;
   if(trans.score<best){result.pruned++;continue;}
   if(trans.is_leaf()){
    result.leaves++;best_transform=trans;best=trans.score;found=true;
   }else{
    size_t before=stock.size();int child_level=trans.level-1;

    trans.branch(stock,child_level,rate,angles[child_level].num_division);
    result.generated+=stock.size()-before;
   }
   if(stock.size()>=options_.batch_size&&!flush())break;
  }
  result.completed=!result.timed_out&&!result.resource_limit;
  result.success=result.completed&&found&&best>result.threshold;
  result.score=result.success?best:0;
  if(result.success)result.pose=best_transform.create_matrix(map.get_min_res(),angles[0].rpy_res,angles[0].min_rpy);
  result.search_ms=clock_ms()-start;
  Access::publish(cpu_,result.success,result.timed_out,result.score,result.search_ms,result.pose);
  return result;
 }

CudaStyleSearch::~CudaStyleSearch() = default;

bool CudaStyleSearch::gpu_available() const { return bool(gpu_); }
std::string CudaStyleSearch::map_storage() const { return gpu_ ? "dense" : "cpu"; }
uint64_t CudaStyleSearch::gpu_map_bytes() const { return gpu_ ? gpu_->gpu_map_bytes : 0; }
uint64_t CudaStyleSearch::cpu_hash_bytes() const { return gpu_ ? gpu_->cpu_hash_bucket_bytes : 0; }
const std::string& CudaStyleSearch::setup_error() const { return setup_error_; }
} // namespace oclbbs
