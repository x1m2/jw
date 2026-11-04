#include "dual_airy/core.hpp"
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/static_transform_broadcaster.h>

namespace dual_airy {
// 常驻融合进程只接收两路点云，空间外参/固定时差来自启动脚本选定的 YAML。
// 不依赖 FAST-LIO 或定位话题；IMU 只在另一个启动时间检查进程中使用。
class FusionNode : public rclcpp::Node {
 public:
  FusionNode() : Node("dual_airy_fusion"),
    buffer_(load_calibration(declare_parameter<std::string>("extrinsics_file", "config/extrinsics.yaml"))) {
    auto front_topic=declare_parameter<std::string>("front_topic","/rslidar_front/points");
    auto rear_topic=declare_parameter<std::string>("rear_topic","/rslidar_rear/points");
    auto output_topic=declare_parameter<std::string>("output_topic","/dual_airy/points_merged");
    frame_=declare_parameter<std::string>("output_frame","rslidar_front");
    stats_path_=declare_parameter<std::string>("stats_file","");
    auto reliability=declare_parameter<std::string>("input_reliability","reliable");
    if (reliability!="reliable" && reliability!="best_effort") throw std::runtime_error("invalid input_reliability");
    // 输入保留少量历史，限制过载后的积压；正常配对等待由 FusionBuffer 显式管理。
    auto qos=rclcpp::QoS(rclcpp::KeepLast(2));
    if(reliability=="best_effort") qos.best_effort(); else qos.reliable();
    // Reliable 发布者允许 Best Effort 订阅者连接，但后者不会获得可靠重传保证。
    // 大消息流控位于 config/fastdds_fusion.xml，由 run.sh 在进程启动前加载。
    // 若修改 output_topic，须同步修改 XML 中按完整话题名称匹配的 writer profile。
    publisher_=create_publisher<sensor_msgs::msg::PointCloud2>(output_topic,rclcpp::QoS(5).reliable());
    status_=create_publisher<std_msgs::msg::String>("/dual_airy/status",5);
    front_=create_subscription<sensor_msgs::msg::PointCloud2>(front_topic,qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg){receive(*msg,false);});
    rear_=create_subscription<sensor_msgs::msg::PointCloud2>(rear_topic,qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg){receive(*msg,true);});
    // parent=前雷达、child=后雷达，后雷达坐标在前雷达中的位姿即同一个 T_front_rear。
    // 此处只发布传感器间 TF，不推算机身安装参数或改变雷达原生坐标轴。
    broadcaster_=std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp=now(); tf.header.frame_id=frame_; tf.child_frame_id="rslidar_rear";
    const auto &m=buffer_.calibration.matrix;
    tf.transform.translation.x=m[0][3]; tf.transform.translation.y=m[1][3]; tf.transform.translation.z=m[2][3];
    tf2::Matrix3x3 rotation(m[0][0],m[0][1],m[0][2],m[1][0],m[1][1],m[1][2],m[2][0],m[2][1],m[2][2]);
    tf2::Quaternion q; rotation.getRotation(q); q.normalize();
    tf.transform.rotation.x=q.x(); tf.transform.rotation.y=q.y(); tf.transform.rotation.z=q.z(); tf.transform.rotation.w=q.w();
    broadcaster_->sendTransform(tf);
    last_front_wall_=last_rear_wall_=last_log_=steady_seconds();
    // 200 ms 定时器只管状态和停流末帧；融合由点云回调驱动，并非 5 Hz 定时发布。
    timer_=create_wall_timer(std::chrono::milliseconds(200),[this]{idle();});
    RCLCPP_INFO(get_logger(),"C++ XYZIRT fusion: %s; delta=%+.9fs; no motion deskew",output_topic.c_str(),buffer_.calibration.delta);
    RCLCPP_WARN(get_logger(),"Calibration status=%s; valid_for=%s",buffer_.calibration.status.c_str(),buffer_.calibration.valid_for.c_str());
  }
  void save() {
    if(stats_path_.empty()) return;
    std::ofstream out(stats_path_);
    if(out) out<<latest_.dump(2)<<'\n';
  }
 private:
  void receive(const sensor_msgs::msg::PointCloud2 &msg,bool rear) {
    if(!fault_.empty()) return;
    const double begin=steady_seconds();
    try {
      const double stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9;
      auto points=decode(msg);
      stages_["decode"].add((steady_seconds()-begin)*1000.);
      double added=steady_seconds();
      if(rear) {
        buffer_.add_rear(stamp,std::move(points));
        stages_["rear_transform"].add((steady_seconds()-added)*1000.);
        last_rear_wall_=begin;
      } else { buffer_.add_front(stamp,std::move(points)); last_front_wall_=begin; }
      publish_ready(false);
      callback_.add((steady_seconds()-begin)*1000.);
    } catch(const std::exception &error) {
      // 时间回绕或字段错误后锁定 fault，避免静默混入旧缓存；修复原因后重启。
      fault_=error.what();
      RCLCPP_ERROR(get_logger(),"Fusion paused: %s",fault_.c_str());
    }
  }
  void publish_ready(bool final) {
    while(true) {
      double begin=steady_seconds();
      auto out=buffer_.next(final);
      if(!out) break;
      stages_["assemble_sort"].add((steady_seconds()-begin)*1000.);
      begin=steady_seconds();
      auto msg=cloud(out->points,frame_);
      double stamp=msg->header.stamp.sec+msg->header.stamp.nanosec*1e-9;
      double sending=steady_seconds();
      stages_["message_build"].add((sending-begin)*1000.);
      // unique_ptr 移交消息，避免一次应用层消息复制；DDS 序列化/传输仍可能复制。
      // XML 设为异步发布：返回只表示交给发布链路，不表示接收端已收到完整点云。
      publisher_->publish(std::move(msg));
      double returned=steady_seconds();
      stages_["publish_call"].add((returned-sending)*1000.);
      uint64_t n=buffer_.counts["publish_returned"].get<uint64_t>()+1;
      // published 为兼容旧日志保留，与 publish_returned 相同；交付频率须在订阅端测量。
      buffer_.counts["published"]=n; buffer_.counts["publish_returned"]=n;
      last_publish_=returned;
      publish_times_.push_back(returned);
      if(publish_times_.size()>101) publish_times_.pop_front();
      output_lag_=buffer_.last_front-stamp;
    }
  }
  Json performance() const {
    // 最近 100 次耗时及调用返回频率；output_lag 是采样时间差，不是端到端网络延迟。
    Json stages=Json::object();
    for(const auto &s:stages_) stages[s.first]=s.second.summary();
    Json hz=nullptr;
    if(last_publish_ && steady_seconds()-*last_publish_>1.) hz=0.;
    else if(publish_times_.size()>1 && publish_times_.back()>publish_times_.front())
      hz=(publish_times_.size()-1)/(publish_times_.back()-publish_times_.front());
    return {{"stage_ms",stages},{"callback_ms",callback_.summary()},{"publish_call_hz",hz},
      {"since_last_publish_s",last_publish_?Json(steady_seconds()-*last_publish_):Json(nullptr)},
      {"latest_front_minus_output_stamp_s",output_lag_?Json(*output_lag_):Json(nullptr)},
      {"queued_front",buffer_.front.size()},{"queued_rear",buffer_.rear.size()},
      {"matched_subscribers",publisher_->get_subscription_count()}};
  }
  void idle() {
    const double t=steady_seconds();
    // 前雷达停流 1.5 s 后尝试封口；后雷达覆盖不足仍由 next(true) 丢弃。
    if(fault_.empty() && !buffer_.front.empty() && t-last_front_wall_>1.5) {
      try {publish_ready(true);} catch(const std::exception &e) {fault_=e.what();}
    }
    if(t-last_status_<1.) return;
    last_status_=t;
    std::string state="ready";
    if(!fault_.empty()) state="fault";
    else if(buffer_.counts["front_received"]==0 || buffer_.counts["rear_received"]==0) state="waiting_for_points";
    else if(t-std::min(last_front_wall_,last_rear_wall_)>1.) state="stale";
    else if(!last_publish_) state="waiting_for_complete_window";
    else if(t-*last_publish_>1.) state="output_stalled";
    latest_={{"state",state},{"mode","fixed"},{"implementation","cpp"},{"version",5},
      {"delta_s",buffer_.calibration.delta}, {"counts",buffer_.counts},{"performance",performance()},
      {"spatial_calibration_status",buffer_.calibration.status},{"spatial_valid_for",buffer_.calibration.valid_for},
      {"limitations","Publish counters measure publish() returns, not delivery. Effective time offset includes differential delivery delay. No motion deskew."}};
    if(!fault_.empty()) latest_["reason"]=fault_;
    std_msgs::msg::String msg; msg.data=latest_.dump(); status_->publish(msg);
    save();
    if(t-last_log_>=5.) {last_log_=t; RCLCPP_INFO(get_logger(),"%s",latest_.dump().c_str());}
  }
  FusionBuffer buffer_;
  std::string frame_,stats_path_,fault_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr front_,rear_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::map<std::string,Samples> stages_;
  Samples callback_;
  std::deque<double> publish_times_;
  std::optional<double> last_publish_,output_lag_;
  double last_front_wall_=0,last_rear_wall_=0,last_log_=0,last_status_=0;
  Json latest_;
};
} // namespace dual_airy
int main(int argc,char **argv) {
  try {
    if(argc==3 && std::string(argv[1])=="--check") {
      auto c=dual_airy::load_calibration(argv[2]);
      std::cout<<"4x4 T_front_rear validated; status="<<c.status<<"; historical delta_s="<<c.delta<<'\n';
      return 0;
    }
    rclcpp::init(argc,argv);
    auto node=std::make_shared<dual_airy::FusionNode>();
    // 串行修改缓存/状态，避免两个点云回调同时变换和排序。DDS 仍有自己的传输线程。
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node); executor.spin(); node->save();
    rclcpp::shutdown();
    return 0;
  } catch(const std::exception &e) {
    std::cerr<<"dual_airy_fusion: "<<e.what()<<'\n';
    if(rclcpp::ok()) rclcpp::shutdown();
    return 2;
  }
}
