#pragma once
#include <cpu_bbs3d/bbs3d.hpp>
#include <cpu_bbs3d/voxelmaps.hpp>
namespace cpu {
struct BbsBenchmarkAccess {
 static VoxelMaps& map(BBS3D& b){return *b.voxelmaps_ptr_;}
 static double rate(const BBS3D& b){return b.v_rate_;}
 static int threads(const BBS3D& b){return b.num_threads_;}
 static double threshold(const BBS3D& b){return b.score_threshold_percentage_;}
 static bool timeout_enabled(const BBS3D& b){return b.use_timeout_;}
 static int timeout_ms(const BBS3D& b){return int(b.timeout_duration_.count());}
 static void publish(BBS3D& b,bool success,bool timeout,int score,double ms,const Eigen::Matrix4d& pose){
  b.has_localized_=success;b.has_timed_out_=timeout;b.best_score_=success?score:0;b.elapsed_time_=ms;
  if(success)b.global_pose_=pose;
 }

 static std::vector<AngularInfo> angles(BBS3D& b){std::vector<AngularInfo> a(b.voxelmaps_ptr_->get_max_level()+1);b.calc_angular_info(a);return a;}
 static std::vector<DiscreteTransformation<double>> initial(BBS3D& b,const AngularInfo& a){return b.create_init_transset(a);}
 static int score(BBS3D& b,DiscreteTransformation<double> t,const AngularInfo& a,const std::vector<Eigen::Vector3d>& points){
  auto& m=*b.voxelmaps_ptr_;t.score=0;
  b.calc_score(t,m.voxelmaps_res_[t.level],a.rpy_res,a.min_rpy,m.multi_buckets_[t.level],m.get_max_bucket_scan_count(),points);return t.score;
 }
};
}

