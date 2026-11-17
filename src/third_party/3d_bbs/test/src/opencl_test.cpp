#include <opencl_bbs3d/bbs3d.hpp>
#include <chrono>
#include <fstream>
#include <stdexcept>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>
using namespace oclbbs;
static double clock_ms() {
  return std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}
static std::string json_string(const std::string& s){
 std::ostringstream out;out<<'"';
 for(unsigned char c:s){if(c=='"'||c=='\\')out<<'\\'<<c;else if(c<32)out<<"\\u"<<std::hex<<std::setw(4)<<std::setfill('0')<<int(c)<<std::dec;else out<<c;}
 out<<'"';return out.str();
}
int main(int argc,char** argv){
 try{
  if(argc<6||argc>9)throw std::runtime_error("usage: opencl_test MAP SCAN original|cuda_style_cpu|opencl_dense REPEATS OUTPUT [BATCH] [BUDGET_MS] [VERIFY]");
  std::string mode=argv[3];if(mode!="original"&&mode!="cuda_style_cpu"&&mode!="opencl_dense"&&mode!="opencl_cuda_style")throw std::runtime_error("Invalid backend");
  int repeats=std::stoi(argv[4]);if(repeats<1)throw std::runtime_error("Invalid repeats");
  {std::ifstream exists(argv[5]);if(exists)throw std::runtime_error("Output already exists");}
  std::ifstream input(argv[2]);std::vector<Eigen::Vector3d> points;double x,y,z;
  while(input>>x>>y>>z)points.emplace_back(x,y,z);if(!input.eof()||points.empty())throw std::runtime_error("Invalid scan");
  SearchOptions options;options.use_opencl=(mode=="opencl_dense"||mode=="opencl_cuda_style");
  if(argc>=7)options.batch_size=std::stoul(argv[6]);
  options.budget_ms=argc>=8?std::stoi(argv[7]):0;
  options.verify_gpu=argc>=9&&std::stoi(argv[8])!=0;
  if(const char* kernel=std::getenv("BBS_OPENCL_KERNEL"))options.kernel_path=kernel;
  cpu::BBS3D cpu;double start=clock_ms();if(!cpu.set_voxelmaps_coords(argv[1]))throw std::runtime_error("Cannot load voxel maps");
  double map_ms=clock_ms()-start;
  cpu.set_num_threads(6);cpu.set_score_threshold_percentage(.9);
  cpu.set_angular_search_range(Eigen::Vector3d(-.03,-.03,0),Eigen::Vector3d(.03,.03,2*std::acos(-1.)));
  if(options.budget_ms>0){cpu.enable_timeout();cpu.set_timeout_duration_in_msec(options.budget_ms);}else cpu.disable_timeout();
  std::unique_ptr<CudaStyleSearch> engine;
  if(mode!="original")engine=std::make_unique<CudaStyleSearch>(cpu,options);
  if((mode=="opencl_dense"||mode=="opencl_cuda_style")&&!engine->gpu_available())throw std::runtime_error("OpenCL setup failed: "+engine->setup_error());
  double runtime_ms=engine?engine->runtime_setup_ms:0;
  std::ofstream out(argv[5]);out<<std::setprecision(17);if(!out)throw std::runtime_error("Cannot create result file");
  for(int repeat=0;repeat<repeats;++repeat){
   SearchResult result;
   if(engine){engine->prepare(points);result=engine->localize();}
   else {
    start=clock_ms();cpu.set_src_points(points);result.scan_setup_ms=clock_ms()-start;
    cpu.localize();result.search_ms=cpu.get_elapsed_time();result.success=cpu.has_localized();result.timed_out=cpu.has_timed_out();result.completed=!result.timed_out;result.score=cpu.get_best_score();
    result.threshold=int(std::floor(points.size()*.9));if(result.success)result.pose=cpu.get_global_pose();
   }
   out<<"{\"backend\":"<<json_string(mode)<<",\"repeat\":"<<repeat<<",\"points\":"<<points.size()<<",\"cpu_threads\":6,\"batch_size\":"<<options.batch_size<<",\"gpu_min_batch\":"<<options.gpu_min_batch<<",\"budget_ms\":"<<options.budget_ms<<",\"verify_gpu\":"<<(options.verify_gpu?"true":"false")<<",\"map_load_ms\":"<<map_ms<<",\"runtime_setup_ms\":"<<runtime_ms<<",\"scan_setup_ms\":"<<result.scan_setup_ms<<",\"search_ms\":"<<result.search_ms<<",\"first_request_component_ms\":"<<map_ms+runtime_ms+result.scan_setup_ms+result.search_ms<<",\"request_with_resident_map_ms\":"<<result.scan_setup_ms+result.search_ms;
   out<<",\"completed\":"<<(result.completed?"true":"false")<<",\"success\":"<<(result.success?"true":"false")<<",\"timed_out\":"<<(result.timed_out?"true":"false")<<",\"resource_limit\":"<<(result.resource_limit?"true":"false")<<",\"score\":"<<result.score<<",\"threshold\":"<<result.threshold;
   out<<",\"gpu_available\":"<<(result.gpu_available?"true":"false")<<",\"gpu_eligible\":"<<(result.gpu_eligible?"true":"false")<<",\"gpu_error\":"<<json_string(result.gpu_error)<<",\"statistics_available\":"<<(engine?"true":"false");
   out<<",\"scored_candidates\":"<<result.scored<<",\"generated_candidates\":"<<result.generated<<",\"popped_candidates\":"<<result.popped<<",\"pruned_candidates\":"<<result.pruned<<",\"leaf_candidates\":"<<result.leaves<<",\"max_queue\":"<<result.max_queue<<",\"cpu_batches\":"<<result.cpu_batches<<",\"gpu_batches\":"<<result.gpu_batches<<",\"gpu_candidates\":"<<result.gpu_candidates<<",\"boundary_points\":"<<result.boundary_points<<",\"verified_candidates\":"<<result.verified_candidates<<",\"gpu_failures\":"<<result.gpu_failures;
   out<<",\"cpu_score_ms\":"<<result.cpu_score_ms<<",\"gpu_score_ms\":"<<result.gpu_score_ms<<",\"gpu_kernel_ms\":"<<result.gpu_kernel_ms<<",\"boundary_correction_ms\":"<<result.boundary_ms<<",\"verification_ms\":"<<result.verify_ms<<",\"pose_map_gravity\":";
   if(result.success){out<<'[';for(int i=0;i<16;++i){if(i)out<<',';out<<result.pose(i/4,i%4);}out<<']';}else out<<"null";
   out<<"}\n";out.flush();if(!out)throw std::runtime_error("Result write failed");
   std::cerr<<"FULL_SEARCH "<<mode<<" repeat="<<repeat<<" completed="<<result.completed<<" success="<<result.success<<" ms="<<result.search_ms<<" score="<<result.score<<" scored="<<result.scored<<" gpu_candidates="<<result.gpu_candidates<<" max_queue="<<result.max_queue<<std::endl;
   if(!result.completed)break;
  }return 0;
 }catch(const std::exception& error){std::cerr<<error.what()<<std::endl;return 1;}
}

