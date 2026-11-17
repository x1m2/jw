#pragma once
#define CL_TARGET_OPENCL_VERSION 120
#include <CL/cl.h>
#include <array>
#include <chrono>
#include <cstring>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include "benchmark_access.hpp"
#include "score_source.hpp"
namespace oclbbs {
using Float4=std::array<float,4>;
using UInt4=std::array<cl_uint,4>;
using Int4=std::array<cl_int,4>;
struct alignas(16) Pose { Float4 rows[3];UInt4 meta; };
static_assert(sizeof(Pose)==64,"Pose layout");
inline double clock_ms(){return std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now().time_since_epoch()).count();}
inline void check(cl_int code,const char* text){if(code!=CL_SUCCESS)throw std::runtime_error(std::string(text)+" OpenCL="+std::to_string(code));}
struct ScoreResult {
 std::vector<Int4> values;
 std::vector<cl_uint> masks;
 size_t words=0;
 double host_ms=0,kernel_ms=0,upload_ms=0,read_ms=0;
};
class Scorer {
 cl_context context_=nullptr;
 cl_command_queue queue_=nullptr;
 cl_program program_=nullptr;
 cl_kernel serial_=nullptr,group_=nullptr;
 cl_mem buckets_=nullptr,info_=nullptr,inverses_=nullptr,points_=nullptr,poses_=nullptr,output_=nullptr,masks_=nullptr;
 size_t capacity_=0,words_=0;
 cl_uint num_points_=0;
 cl_ulong max_allocation_bytes_=0;
 void release() {
  for(auto b:{buckets_,info_,inverses_,points_,poses_,output_,masks_})if(b)clReleaseMemObject(b);
  if(serial_)clReleaseKernel(serial_);if(group_)clReleaseKernel(group_);
  if(program_)clReleaseProgram(program_);if(queue_)clReleaseCommandQueue(queue_);if(context_)clReleaseContext(context_);
 }
 cl_mem buffer(cl_mem_flags flags,size_t bytes,void* data=nullptr){
  if(!bytes || (max_allocation_bytes_ && bytes>max_allocation_bytes_))
   throw std::runtime_error("GPU buffer size exceeds device allocation limit: "+std::to_string(bytes));
  cl_int err;auto b=clCreateBuffer(context_,flags,bytes,data,&err);check(err,"clCreateBuffer");return b;
 }
 static double event_ms(cl_event e) {
  cl_ulong start,end;check(clGetEventProfilingInfo(e,CL_PROFILING_COMMAND_START,sizeof(start),&start,nullptr),"event start");
  check(clGetEventProfilingInfo(e,CL_PROFILING_COMMAND_END,sizeof(end),&end,nullptr),"event end");return double(end-start)/1e6;
 }
 void reserve(size_t count) {
  if(count<=capacity_)return;
  for(auto b:{poses_,output_,masks_})if(b)clReleaseMemObject(b);
  poses_=output_=masks_=nullptr;capacity_=0;
  poses_=buffer(CL_MEM_READ_ONLY,sizeof(Pose)*count);
  output_=buffer(CL_MEM_WRITE_ONLY,sizeof(Int4)*count);
  masks_=buffer(CL_MEM_READ_WRITE,sizeof(cl_uint)*count*words_);
  capacity_=count;
 }
public:
 std::string device_name;
 uint64_t gpu_map_bytes=0,cpu_hash_bucket_bytes=0;
 std::vector<uint64_t> map_level_bytes;
 double startup_ms=0,map_upload_ms=0,scan_upload_ms=0;
 explicit Scorer(cpu::VoxelMaps& map,const std::string& kernel_path) {
  double start=clock_ms();
  for(const auto& level:map.multi_buckets_)cpu_hash_bucket_bytes+=uint64_t(level.size())*sizeof(Int4);
  try{
   cl_uint n=0;check(clGetPlatformIDs(0,nullptr,&n),"platform count");
   std::vector<cl_platform_id> platforms(n);check(clGetPlatformIDs(n,platforms.data(),nullptr),"platform list");
   cl_device_id device=nullptr;
   for(auto p:platforms){if(clGetDeviceIDs(p,CL_DEVICE_TYPE_GPU,1,&device,nullptr)==CL_SUCCESS)break;device=nullptr;}
   if(!device)throw std::runtime_error("No GPU device");
   check(clGetDeviceInfo(device,CL_DEVICE_MAX_MEM_ALLOC_SIZE,sizeof(max_allocation_bytes_),&max_allocation_bytes_,nullptr),"allocation limit");

   char name[256]={};check(clGetDeviceInfo(device,CL_DEVICE_NAME,sizeof(name),name,nullptr),"device");device_name=name;
   cl_int err;context_=clCreateContext(nullptr,1,&device,nullptr,nullptr,&err);check(err,"context");
   queue_=clCreateCommandQueue(context_,device,CL_QUEUE_PROFILING_ENABLE,&err);check(err,"queue");
   std::string source = kDenseScoreSource;
   if (!kernel_path.empty()) {
    std::ifstream kernel(kernel_path);
    if (!kernel) throw std::runtime_error("Cannot open OpenCL kernel: " + kernel_path);
    source.assign(std::istreambuf_iterator<char>(kernel), {});
   }
   const char* ptr = source.c_str();
   program_=clCreateProgramWithSource(context_,1,&ptr,nullptr,&err);check(err,"program");
   err=clBuildProgram(program_,1,&device,"-cl-std=CL1.2",nullptr,nullptr);
   if(err!=CL_SUCCESS){size_t bytes=0;clGetProgramBuildInfo(program_,device,CL_PROGRAM_BUILD_LOG,0,nullptr,&bytes);std::vector<char> log(bytes);clGetProgramBuildInfo(program_,device,CL_PROGRAM_BUILD_LOG,bytes,log.data(),nullptr);std::cerr<<log.data()<<std::endl;check(err,"build");}
   serial_=clCreateKernel(program_,"score_serial",&err);check(err,"serial kernel");
   group_=clCreateKernel(program_,"score_group",&err);check(err,"group kernel");
   startup_ms=clock_ms()-start;start=clock_ms();
   std::vector<cl_uchar> buckets;std::vector<UInt4> info;std::vector<float> inverses;
   for(size_t level=0;level<map.multi_buckets_.size();++level){
    const auto& input=map.multi_buckets_[level];
    auto hash=[](int x,int y,int z){return (uint32_t(x)*73856093u)^(uint32_t(y)*19349669u)^(uint32_t(z)*83492791u);};
    auto original_contains=[&](const Int4& key){
     uint32_t h=hash(key[0],key[1],key[2]);
     for(int j=0;j<map.get_max_bucket_scan_count();++j){
      const auto& b=input[(h+uint32_t(j))%uint32_t(input.size())];
      if(b.w()==0)break;
      if(b.x()==key[0]&&b.y()==key[1]&&b.z()==key[2])return true;
     }return false;
    };
    std::vector<Int4> keys;
    for(const auto& b:input){
     if(!b.w() || (b.x()==0&&b.y()==0&&b.z()==0))continue;
     Int4 key={b.x(),b.y(),b.z(),1};
     if(!original_contains(key))throw std::runtime_error("Stored key is not reachable in original CPU table");
     keys.push_back(key);
    }
    // The existing loader assigns w=1 to serialized empty slots. Preserve its
    // effective origin membership rather than silently changing CPU semantics.
    if(original_contains(Int4{0,0,0,1}))keys.push_back({0,0,0,1});
    if(keys.empty())throw std::runtime_error("Empty occupancy set");
    Int4 low=keys[0],high=keys[0];
    for(const auto& key:keys)for(int axis=0;axis<3;++axis){low[axis]=std::min(low[axis],key[axis]);high[axis]=std::max(high[axis],key[axis]);}
    uint64_t dx=uint64_t(int64_t(high[0])-low[0]+1),dy=uint64_t(int64_t(high[1])-low[1]+1),dz=uint64_t(int64_t(high[2])-low[2]+1);
    const uint64_t budget=128ull*1024*1024;
    if(!dx||!dy||!dz||dx>budget||dy>budget||dz>budget||dx>budget/dy||dx*dy>budget/dz)throw std::runtime_error("Dense GPU map dimensions exceed budget");
    uint64_t cells=dx*dy*dz;
    if(cells>128ull*1024*1024 || buckets.size()+cells>128ull*1024*1024)throw std::runtime_error("Dense GPU map exceeds 128 MiB budget");
    cl_uint offset=cl_uint(buckets.size());
    map_level_bytes.push_back(cells);
    buckets.resize(buckets.size()+size_t(cells),0);
    for(const auto& key:keys){
     uint64_t index=((int64_t(key[2])-low[2])*dy+(int64_t(key[1])-low[1]))*dx+(int64_t(key[0])-low[0]);
     if(index>=cells)throw std::runtime_error("Dense index overflow");
     buckets[offset+index]=1;
    }
    info.push_back({offset,cl_uint(dx),cl_uint(dy),cl_uint(dz)});
    UInt4 origin;for(int j=0;j<3;++j)std::memcpy(&origin[j],&low[j],sizeof(cl_uint));origin[3]=0;
    info.push_back(origin);
    inverses.push_back(float(1.0/map.voxelmaps_res_[level]));
    std::cerr<<"GPU_DENSE_MAP level="<<level<<" original_slots="<<input.size()<<" occupied_keys="<<keys.size()<<" dense_bytes="<<cells<<std::endl;
   }
   gpu_map_bytes=buckets.size()*sizeof(cl_uchar);
   buckets_=buffer(CL_MEM_READ_ONLY|CL_MEM_COPY_HOST_PTR,gpu_map_bytes,buckets.data());
   info_=buffer(CL_MEM_READ_ONLY|CL_MEM_COPY_HOST_PTR,info.size()*sizeof(UInt4),info.data());
   inverses_=buffer(CL_MEM_READ_ONLY|CL_MEM_COPY_HOST_PTR,inverses.size()*sizeof(float),inverses.data());
   check(clFinish(queue_),"finish map upload");map_upload_ms=clock_ms()-start;
   std::cerr<<"GPU_MAP_STORAGE=dense bytes="<<gpu_map_bytes
            <<" device_max_single_allocation="<<max_allocation_bytes_<<std::endl;
  }catch(...){release();throw;}
 }
 Scorer(const Scorer&)=delete;Scorer& operator=(const Scorer&)=delete;
 ~Scorer(){release();}
 void set_points(const std::vector<Float4>& points){
  if(points.empty()||points.size()>std::numeric_limits<cl_uint>::max())throw std::runtime_error("Invalid point count");
  double start=clock_ms();
  if(points_)clReleaseMemObject(points_);points_=nullptr;
  for(auto b:{poses_,output_,masks_})if(b)clReleaseMemObject(b);
  poses_=output_=masks_=nullptr;capacity_=0;
  num_points_=cl_uint(points.size());words_=(points.size()+31)/32;
  points_=buffer(CL_MEM_READ_ONLY|CL_MEM_COPY_HOST_PTR,points.size()*sizeof(Float4),const_cast<Float4*>(points.data()));
  check(clFinish(queue_),"finish points upload");scan_upload_ms=clock_ms()-start;
 }
 ScoreResult score(const std::vector<Pose>& poses,bool group){
  if(poses.empty()||poses.size()>std::numeric_limits<cl_uint>::max()||!points_)throw std::runtime_error("Invalid scoring input");
  double start=clock_ms();reserve(poses.size());ScoreResult result;
  result.words=words_;result.values.resize(poses.size());result.masks.resize(poses.size()*words_);
  cl_event upload,fill,kernel,read_scores,read_masks;
  check(clEnqueueWriteBuffer(queue_,poses_,CL_FALSE,0,poses.size()*sizeof(Pose),poses.data(),0,nullptr,&upload),"upload poses");
  cl_uint zero=0;
  check(clEnqueueFillBuffer(queue_,masks_,&zero,sizeof(zero),0,result.masks.size()*sizeof(cl_uint),0,nullptr,&fill),"clear masks");
  cl_kernel k=group?group_:serial_;cl_uint nc=cl_uint(poses.size()),words=cl_uint(words_);
  check(clSetKernelArg(k,0,sizeof(poses_),&poses_),"arg0");check(clSetKernelArg(k,1,sizeof(nc),&nc),"arg1");
  check(clSetKernelArg(k,2,sizeof(points_),&points_),"arg2");check(clSetKernelArg(k,3,sizeof(num_points_),&num_points_),"arg3");
  check(clSetKernelArg(k,4,sizeof(buckets_),&buckets_),"arg4");check(clSetKernelArg(k,5,sizeof(info_),&info_),"arg5");
  check(clSetKernelArg(k,6,sizeof(inverses_),&inverses_),"arg6");check(clSetKernelArg(k,7,sizeof(output_),&output_),"arg7");
  check(clSetKernelArg(k,8,sizeof(masks_),&masks_),"arg8");check(clSetKernelArg(k,9,sizeof(words),&words),"arg9");
  size_t local=64,global=group?poses.size()*64:((poses.size()+63)/64)*64;
  check(clEnqueueNDRangeKernel(queue_,k,1,nullptr,&global,&local,0,nullptr,&kernel),"score kernel");
  check(clEnqueueReadBuffer(queue_,output_,CL_FALSE,0,result.values.size()*sizeof(Int4),result.values.data(),0,nullptr,&read_scores),"read scores");
  check(clEnqueueReadBuffer(queue_,masks_,CL_TRUE,0,result.masks.size()*sizeof(cl_uint),result.masks.data(),0,nullptr,&read_masks),"read masks");
  result.host_ms=clock_ms()-start;result.kernel_ms=event_ms(kernel);result.upload_ms=event_ms(upload);
  result.read_ms=event_ms(read_scores)+event_ms(read_masks);
  for(auto e:{upload,fill,kernel,read_scores,read_masks})clReleaseEvent(e);
  return result;
 }
};
}

