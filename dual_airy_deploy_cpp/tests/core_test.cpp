#include "dual_airy/core.hpp"
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <unistd.h>
using namespace dual_airy;
void check(bool value,const std::string &message) {if(!value) throw std::runtime_error(message);}
void near(double a,double b,double eps,const std::string &message) {check(std::abs(a-b)<=eps,message);}
template<class F> void rejects(F f,const std::string &message) {
  bool rejected=false; try {f();} catch(const std::exception &) {rejected=true;} check(rejected,message);
}
template<class T> void put(Point &p,size_t offset,T value) {std::memcpy(p.bytes.data()+offset,&value,sizeof(value));}
Point point(double t,float x=1,float y=2,float z=3,uint16_t ring=7) {
  Point p; p.time=t; put(p,0,x); put(p,4,y); put(p,8,z); put(p,12,4.5f); put(p,16,ring); put(p,18,t); return p;
}
void config_tests(const std::string &path) {
  auto c=load_calibration(path);
  // Packaged MLCC default: protect full-precision translation on YAML promotion.
  near(c.matrix[2][3],-.9277915511715523,1e-15,"translation exact");
  auto p=point(10,1,0,0); auto before=p;
  std::vector<Point> points={p}; transform(points,c);
  for(size_t i=0;i<3;++i) near(coordinate(points[0],i),c.matrix[i][0]+c.matrix[i][3],1e-6,"rear-to-front direction");
  near(points[0].time,10+c.delta,1e-14,"corrected time sign");
  check(std::memcmp(points[0].bytes.data()+12,before.bytes.data()+12,6)==0,"intensity and ring preserved");
  auto file=std::filesystem::temp_directory_path()/("dual_airy_matrix_test_"+std::to_string(getpid())+".yaml");
  for(int kind=0;kind<4;++kind) {
    auto bad=YAML::Clone(c.document);
    if(kind==0) bad["T_front_rear"][3][0]=1.;
    if(kind==1) bad["T_front_rear"][0][0]=2.;
    if(kind==2) bad["T_front_rear"][0][3]=std::numeric_limits<double>::quiet_NaN();
    if(kind==3) bad["transform"]="rear <- front";
    {std::ofstream out(file); out<<bad;}
    rejects([&]{load_calibration(file.string());},"bad calibration rejected");
  }
  std::filesystem::remove(file);
}
void layout_tests() {
  auto msg=cloud({point(10),point(10.02,4,5,6)},"rslidar_front");
  auto decoded=decode(*msg);
  check(decoded.size()==2 && msg->point_step==26 && msg->row_step==52,"packed layout");
  near(decoded[1].time,10.02,0,"unaligned timestamp");
  check(decoded[1].bytes==point(10.02,4,5,6).bytes,"unchanged records roundtrip");
  auto padded=*msg;
  padded.height=2; padded.width=1; padded.row_step=30;
  padded.data.resize(60,0xff);
  std::memcpy(padded.data.data(),decoded[0].bytes.data(),26);
  std::memcpy(padded.data.data()+30,decoded[1].bytes.data(),26);
  check(decode(padded)[1].bytes==decoded[1].bytes,"row padding respected");
  auto bad=*msg; bad.fields.back().offset=20;
  rejects([&]{decode(bad);},"bad field rejected");
  bad=*msg; bad.is_bigendian=true; rejects([&]{decode(bad);},"endianness rejected");
  bad=*msg; bad.data.pop_back(); rejects([&]{decode(bad);},"truncation rejected");
  auto nan=point(10); put(nan,0,std::numeric_limits<float>::quiet_NaN());
  auto filtered=cloud({nan,point(10.02)},"rslidar_front"); check(decode(*filtered).size()==1,"NaN filtered");
  auto zero=point(0); filtered=cloud({point(10),zero},"rslidar_front"); check(decode(*filtered).size()==1,"nonpositive time filtered");
}
Calibration identity() {
  Calibration c; for(size_t i=0;i<4;++i) c.matrix[i][i]=1; c.delta=-.01; return c;
}
void buffer_tests() {
  FusionBuffer b(identity());
  b.add_front(10,{point(10,1),point(10.05,2)});
  b.add_front(10.1,{point(10.1),point(10.15)});
  b.add_rear(10.00,{point(10.00,3),point(10.04,4),point(10.09,5)});
  check(!b.next(),"wait for complete rear window");
  b.add_rear(10.10,{point(10.10,6),point(10.12,7),point(10.19,8)});
  auto out=b.next(); check(out.has_value(),"frame generated");
  check(out->front_count==2 && out->rear_count==3,"rear frame split into front interval");
  for(size_t i=1;i<out->points.size();++i) check(out->points[i-1].time<=out->points[i].time,"stable time ordering");
  for(const auto &p:out->points) check(p.time>=10 && p.time<10.1,"no outside-window points");
  near(coordinate(out->points[1],0),4,0,"correct rear record");
  check(!b.next(true),"incomplete final rear window dropped");
  check(b.counts["unmatched_dropped"]==1,"tail drop counted");
  rejects([&]{b.add_front(10.1,{point(10.1)});},"timestamp rewind rejected");
  FusionBuffer startup(identity());
  startup.add_front(10,{point(10)}); startup.add_front(10.1,{point(10.1)});
  startup.add_rear(10.06,{point(10.06),point(10.16)});
  check(!startup.next(),"no incomplete startup output");
  check(startup.counts["startup_dropped"]==1,"startup drop counted");
}
Json timing_sequence(HostTimeAlignment &a,int mode) {
  Json report;
  for(int tick=0;tick<=2600;++tick) {
    double t=1000+tick*.005;
    for(size_t i=0;i<4;++i) {
      if(i>=2 && tick%20) continue;
      double offset=i%2 ? 49.99 : 50.;
      if(mode==2 && i==3) offset+=.03;
      double jitter=(mode==1 && i==0)? .02*std::sin(tick*.13):0.;
      a.observe(i,t-offset,t+jitter);
    }
    if(tick%200==0) report=a.report(t+.05);
  }
  return report;
}
void timing_tests() {
  HostTimeAlignment a;
  auto report=timing_sequence(a,0);
  check(report["state"]=="ready","stable timing ready");
  near(report["delta_s"].get<double>(),-.01,1e-10,"rear time sign matches receive offset");
  check(a.report(1016)["state"]=="stale","stale stream rejected");
  a.observe(0,900,1013.1); check(a.report(1013.1)["state"]=="fault","clock reversal latched");
  HostTimeAlignment noisy; report=timing_sequence(noisy,1);
  check(report["reason"]=="receive_jitter_too_large" && report["delta_s"].is_null(),"high jitter cannot lock");
  HostTimeAlignment disagree; report=timing_sequence(disagree,2);
  check(report["reason"]=="imu_points_alignment_disagree" && report["delta_s"].is_null(),"independent stream agreement required");
}
int main(int argc,char **argv) {
  try {
    check(argc==2,"config path required");
    config_tests(argv[1]); layout_tests(); buffer_tests(); timing_tests();
    std::cout<<"PASS: matrix direction/validation, packed XYZIRT, window split/order, clock readiness/failures\n";
    return 0;
  } catch(const std::exception &e) {std::cerr<<"FAIL: "<<e.what()<<'\n'; return 1;}
}
