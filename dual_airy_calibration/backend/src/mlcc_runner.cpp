#include <iostream>
#include <fstream>
#include <iomanip>
#include <set>
#include <chrono>
#include <filesystem>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#if MLCC_STAGE == 0
#include "pose_refine.hpp"
#elif MLCC_STAGE == 1
#include "extrinsic_refine.hpp"
#else
#include "global.hpp"
#endif
using namespace std;
using namespace Eigen;
using Json=nlohmann::json;
double voxel_size=1.,eigen_thr=20.;
#if MLCC_STAGE == 0
#include "cut_pose.hpp"
#elif MLCC_STAGE == 1
#include "cut_extrinsic.hpp"
#else
#include "cut_global.hpp"
#endif
using Poses=vector<mypcl::pose>;
Poses read_poses(const string &path){
 ifstream f(path); if(!f)throw runtime_error("cannot read "+path);
 Poses out; double x,y,z,w,qx,qy,qz;
 while(f>>x>>y>>z>>w>>qx>>qy>>qz){
  Eigen::Quaterniond q(w,qx,qy,qz);Eigen::Vector3d t(x,y,z);
  if(!q.coeffs().allFinite() || !t.allFinite() || q.norm()<.99 || q.norm()>1.01)throw runtime_error("invalid input pose");
  out.emplace_back(q.normalized(),t);
 }
 if(!f.eof() || out.empty())throw runtime_error("bad/empty pose file "+path);
 return out;
}
void write_poses(const Poses &v,const string &path){
 ofstream f(path);if(!f)throw runtime_error("cannot write "+path);f<<setprecision(17);
 for(const auto &p:v)f<<p.t.transpose()<<" "<<p.q.w()<<" "<<p.q.x()<<" "<<p.q.y()<<" "<<p.q.z()<<"\n";
}
int main(int argc,char**argv){
 try{
  if(argc<4)throw runtime_error("usage: RUNNER DATA_DIR OUTPUT_PREFIX ITERATIONS [VOXEL=1] [EIGEN_RATIO=20] [DOWNSAMPLE=.08]");
  string path=string(argv[1])+"/",prefix=argv[2];int iterations=stoi(argv[3]);
  if(argc>4)voxel_size=stod(argv[4]);if(argc>5)eigen_thr=stod(argv[5]);double down=argc>6?stod(argv[6]):.08;
  auto poses=read_poses(path+"pose.json");auto refs=read_poses(path+"ref.json");
  if(refs.size()!=1)throw runtime_error("this runner expects exactly one rear LiDAR");
  size_t n=poses.size(); if(n<3)throw runtime_error("at least three poses required");
  vector<pcl::PointCloud<PointType>::Ptr> base(n),rear(n);
  for(size_t i=0;i<n;++i){
   base[i].reset(new pcl::PointCloud<PointType>);rear[i].reset(new pcl::PointCloud<PointType>);
   if(pcl::io::loadPCDFile(path+"0/"+to_string(i)+".pcd",*base[i])<0)throw runtime_error("missing front PCD");
   if(pcl::io::loadPCDFile(path+"1/"+to_string(i)+".pcd",*rear[i])<0)throw runtime_error("missing rear PCD");
   if(down>0){downsample_voxel(*base[i],down);downsample_voxel(*rear[i],down);}
   if(base[i]->size()<100 || rear[i]->size()<100)throw runtime_error("insufficient input points");
  }
  Json history=Json::array();
  for(int step=0;step<iterations;++step){
   auto started=chrono::steady_clock::now();
   unordered_map<VOXEL_LOC,OCTO_TREE*> map;
#if MLCC_STAGE == 0
   LM_OPTIMIZER lm(n);
#elif MLCC_STAGE == 1
   EXTRIN_OPTIMIZER lm(n,1);
#else
   LM_OPTIMIZER lm(n,1);
#endif
   for(size_t i=0;i<n;++i){
#if MLCC_STAGE == 0
    cut_voxel(map,base[i],poses[i].q,poses[i].t,i,n,eigen_thr);
#elif MLCC_STAGE == 1
    cut_voxel(map,base[i],poses[i].q,poses[i].t,i,n,eigen_thr);
    cut_voxel(map,rear[i],poses[i].q*refs[0].q,poses[i].q*refs[0].t+poses[i].t,i,n,eigen_thr,false);
#else
    cut_voxel(map,base[i],poses[i].q,poses[i].t,i,n,1,eigen_thr);
    cut_voxel(map,rear[i],poses[i].q*refs[0].q,poses[i].q*refs[0].t+poses[i].t,i,n,1,eigen_thr,0,false);
#endif
   }
   for(auto &item:map)item.second->recut();
   for(size_t i=0;i<n;++i)assign_qt(lm.poses[i],lm.ts[i],poses[i].q,poses[i].t);
#if MLCC_STAGE != 0
   assign_qt(lm.refQs[0],lm.refTs[0],refs[0].q,refs[0].t);
#endif
   for(auto &item:map)item.second->feed_pt(lm);
#if MLCC_STAGE == 0
   size_t features=lm.origin_points.size();
#else
   size_t features=lm.baseOriginPts.size();
#endif
   // Upstream's <4-feature path is not suitable for calibration; reject before solving.
   if(features<20)throw runtime_error("fewer than 20 valid multi-observation plane features");
   double before=0,after=0;
#if MLCC_STAGE == 0
   lm.evaluate_only_residual(lm.poses,lm.ts,before);
#else
   lm.evaluate_only_residual(lm.poses,lm.ts,lm.refQs,lm.refTs,before);
#endif
   lm.optimize();
#if MLCC_STAGE == 0
   lm.evaluate_only_residual(lm.poses,lm.ts,after);
#else
   lm.evaluate_only_residual(lm.poses,lm.ts,lm.refQs,lm.refTs,after);
#endif
   if(!isfinite(before)||!isfinite(after)||after>before+1e-8)throw runtime_error("optimizer returned invalid/increased fixed-feature objective");
   for(size_t i=0;i<n;++i){
    if(!lm.poses[i].coeffs().allFinite() || !lm.ts[i].allFinite())throw runtime_error("nonfinite optimized pose");
    assign_qt(poses[i].q,poses[i].t,lm.poses[i],lm.ts[i]);poses[i].q.normalize();
   }
#if MLCC_STAGE != 0
   if(!lm.refQs[0].coeffs().allFinite()||!lm.refTs[0].allFinite())throw runtime_error("nonfinite extrinsic");
   assign_qt(refs[0].q,refs[0].t,lm.refQs[0],lm.refTs[0]);refs[0].q.normalize();
#endif
   Json record={{"iteration",step},{"features",features},{"fixed_feature_variance_sum_before_m2",before},{"fixed_feature_variance_sum_after_m2",after},
    {"seconds",chrono::duration<double>(chrono::steady_clock::now()-started).count()},
    {"translation_m",{refs[0].t.x(),refs[0].t.y(),refs[0].t.z()}},
    {"quaternion_wxyz",{refs[0].q.w(),refs[0].q.x(),refs[0].q.y(),refs[0].q.z()}}};
#if MLCC_STAGE == 1
   Eigen::MatrixXd H(6,6);Eigen::VectorXd g(6);double r;
   lm.divide_thread(lm.poses,lm.ts,lm.refQs,lm.refTs,H,g,r);
   Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(.5*(H+H.transpose()));
   vector<double> ev(eig.eigenvalues().data(),eig.eigenvalues().data()+6);record["extrinsic_hessian_eigenvalues_mixed_units"]=ev;
#endif
   for(auto &item:map)delete item.second;
   history.push_back(record);cout<<record.dump()<<endl;
   // Match upstream: keep the world gauge unchanged between outer voxel rebuilds.
   write_poses(poses,prefix+".poses.txt");write_poses(refs,prefix+".ref.txt");
   ofstream log(prefix+".history.json");log<<history.dump(2)<<'\n';
  }
  return 0;
 }catch(const exception&e){cerr<<"MLCC runner: "<<e.what()<<endl;return 2;}
}
