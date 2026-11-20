#pragma once
#include "opencl_scorer.hpp"
#include <omp.h>
namespace oclbbs {
using Transform=DiscreteTransformation<double>;
using Access=cpu::BbsBenchmarkAccess;
inline std::vector<Pose> make_poses(const std::vector<Transform>& ts,const std::vector<cpu::AngularInfo>& angles,cpu::VoxelMaps& map) {
 std::vector<Pose> poses(ts.size());
 for(size_t i=0;i<ts.size();++i){
  auto t=ts[i];auto& a=angles[t.level];auto m=t.create_matrix(map.voxelmaps_res_[t.level],a.rpy_res,a.min_rpy);
  Pose p{};for(int row=0;row<3;++row)for(int col=0;col<4;++col)p.rows[row][col]=float(m(row,col));
  p.meta[0]=cl_uint(t.level);poses[i]=p;
 }
 return poses;
}
inline std::vector<int> correct(cpu::BBS3D& b,const std::vector<Transform>& ts,const std::vector<cpu::AngularInfo>& a,const std::vector<Eigen::Vector3d>& points,const ScoreResult& gpu) {
 std::vector<int> corrected(ts.size());int bad_masks=0;
#pragma omp parallel for num_threads(6) reduction(+:bad_masks)
 for(int i=0;i<int(ts.size());++i){
  if(b.cancel_requested())continue;
  int score=gpu.values[i][0];
  if(gpu.values[i][2]){
   std::vector<Eigen::Vector3d> uncertain;uncertain.reserve(gpu.values[i][2]);
   for(size_t word=0;word<gpu.words;++word){
    if(b.cancel_requested())break;
    uint32_t bits=gpu.masks[i*gpu.words+word];
    while(bits){int bit=__builtin_ctz(bits);size_t p=word*32+bit;if(p<points.size())uncertain.push_back(points[p]);else bad_masks++;bits&=bits-1;}
   }
   if(b.cancel_requested())continue;
   if(uncertain.size()!=size_t(gpu.values[i][2]))bad_masks++;
   score=score-gpu.values[i][1]+Access::score(b,ts[i],a[ts[i].level],uncertain);
  }corrected[i]=score;
 }
 if(b.cancel_requested())return corrected;
 if(bad_masks)throw std::runtime_error("Invalid boundary mask");return corrected;
}

}
