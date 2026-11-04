#include "dual_airy/core.hpp"
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <set>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>

namespace dual_airy {
// 启动阶段的独立短时进程；检查后退出，再由脚本启动融合，避免点云计算挤占 IMU 回调。
// 估计有效时间补偿，但不设置雷达或主机时钟。
class TimeCheck : public rclcpp::Node {
 public:
  TimeCheck(Calibration config,std::string output,double timeout)
    : Node("dual_airy_startup_time_check"),config_(std::move(config)),output_(std::move(output)),timeout_(timeout) {
    started_=steady_seconds();
    status_=create_publisher<std_msgs::msg::String>("/dual_airy/status",5);
    // IMU 与点云分别调度，点云反序列化不会占用唯一的执行线程。
    imu_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cloud_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    // 顺序固定为 front_imu、rear_imu、front_points、rear_points，与核心 keys 一致。
    // 回调只取 header 和接收时间，不做点云坐标变换/排序。
    for(size_t i=0;i<4;++i) {
      std::string topic=std::string("/rslidar_")+(i%2==0?"front/":"rear/")+(i<2?"imu":"points");
      topics_[i]=topic;
      auto qos=rclcpp::QoS(rclcpp::KeepLast(i<2?200:2)).reliable();
      rclcpp::SubscriptionOptions options;
      options.callback_group=i<2?imu_group_:cloud_group_;
      if(i<2) subscriptions_[i]=create_subscription<sensor_msgs::msg::Imu>(topic,qos,
        [this,i](sensor_msgs::msg::Imu::ConstSharedPtr m,const rclcpp::MessageInfo &info){observe(i,m->header.stamp,info);},options);
      else subscriptions_[i]=create_subscription<sensor_msgs::msg::PointCloud2>(topic,qos,
        [this,i](sensor_msgs::msg::PointCloud2::ConstSharedPtr m,const rclcpp::MessageInfo &info){observe(i,m->header.stamp,info);},options);
    }
    timer_=create_wall_timer(std::chrono::seconds(1),[this]{tick();});
    std::cout<<"阶段 1/2：v5 三线程检查，IMU/点云分组；IMU 抖动上限 20 ms、点云 40 ms。"<<std::endl;
  }
  bool done=false, success=false;
  std::function<void()> on_done;
 private:
  void observe(size_t i,const builtin_interfaces::msg::Time &stamp,const rclcpp::MessageInfo &info) {
    double wall=wall_seconds();
    // 优先用有效 DDS 接收时间，减少回调排队造成的偏差；无效时退回系统回调时间。
    // 60 s 检查用于排除明显不在系统时钟域的时间戳，来源会写入诊断状态。
    int64_t ns=info.get_rmw_message_info().received_timestamp;
    double middleware=static_cast<double>(ns)*1e-9;
    bool use_middleware=ns>0 && std::abs(middleware-wall)<60.;
    const double steady=steady_seconds();
    std::lock_guard<std::mutex> lock(mutex_);
    ++received_[i]; last_callback_[i]=steady;
    last_header_[i]=stamp.sec+stamp.nanosec*1e-9;
    middleware_age_[i]=ns>0?wall-middleware:std::numeric_limits<double>::quiet_NaN();
    sources_.insert(use_middleware?"middleware":"callback_wall");
    alignment_.observe(i,stamp.sec+stamp.nanosec*1e-9,use_middleware?middleware:wall);
  }
  void tick() {
    std::lock_guard<std::mutex> lock(mutex_);
    double elapsed=steady_seconds()-started_;
    Json report=alignment_.report(wall_seconds());
    report["phase"]="startup_check_only"; report["elapsed_s"]=elapsed;
    report["receive_time_sources"]=sources_; report["version"]=5; report["implementation"]="cpp";
    // 与窗口 counts 分开：累计回调数和 monotonic 新鲜度不依赖 DDS 接收时间戳。
    Json blocking=Json::array(); size_t total=0, publishers=0;
    for(size_t i=0;i<4;++i) {
      const auto endpoints=get_publishers_info_by_topic(topics_[i]);
      const size_t matched=subscriptions_[i]->get_publisher_count();
      Json writers=Json::array();
      for(const auto &ep:endpoints) {
        const auto q=ep.qos_profile().get_rmw_qos_profile();
        writers.push_back({{"node",ep.node_namespace()+"/"+ep.node_name()},
          {"type",ep.topic_type()}, {"reliability",q.reliability==RMW_QOS_POLICY_RELIABILITY_RELIABLE?"reliable":"best_effort_or_unknown"}});
      }
      Json age=received_[i]?Json(steady_seconds()-last_callback_[i]):Json(nullptr);
      report["streams"][HostTimeAlignment::keys[i]]={{"topic",topics_[i]}, {"callbacks_total",received_[i]},
        {"callback_age_s",age}, {"last_header_s",received_[i]?Json(last_header_[i]):Json(nullptr)},
        {"middleware_age_ms",received_[i] && std::isfinite(middleware_age_[i])?Json(middleware_age_[i]*1000.):Json(nullptr)},
        {"matched_publishers",matched}, {"discovered_publishers",endpoints.size()}, {"publishers",writers}};
      if(!received_[i] || steady_seconds()-last_callback_[i]>1.) blocking.push_back(HostTimeAlignment::keys[i]);
      total+=received_[i]; publishers+=endpoints.size();
    }
    report["blocking_streams"]=blocking;
    if(total==0) {
      report["state"]="waiting_for_inputs";
      report["reason"]=publishers==0?"no_publishers_discovered; check_driver_domain_localhost_and_startup":"publishers_discovered_but_no_callbacks; check_qos_and_transport";
    } else if(blocking.empty() && report["state"]=="stale") {
      report["reason"]="callbacks_fresh_but_receive_timestamps_stale; inspect_middleware_age_ms";
    }
    std::ofstream reportfile(output_+".timing.json");
    if(!reportfile) throw std::runtime_error("cannot write timing report");
    reportfile<<report.dump(2)<<'\n';
    std_msgs::msg::String msg; msg.data=report.dump(); status_->publish(msg);
    if(elapsed-last_log_>=5.) {last_log_=elapsed; std::cout<<msg.data<<std::endl;}
    // 至少观察 12 s 才交接；只改会话副本的 delta 和诊断元数据，原矩阵不变。
    // 17 位有效数字保留矩阵精度；输入 YAML 不被覆盖。
    if(report["state"]=="ready" && elapsed>=12.) {
      auto cfg=YAML::Clone(config_.document);
      cfg["delta_s"]=report["delta_s"].get<double>();
      cfg["time_source"]="independent_cpp_grouped_startup_check_v5";
      cfg["input_config_valid_for"]=config_.valid_for;
      cfg["valid_for"]="Same rigid mounting as input calibration; current startup-check clock session only";
      cfg["time_notes"]="Effective receive offset includes differential delivery delay; restart after sensor reset.";
      cfg["startup_time_check"]=YAML::Load(report.dump());
      YAML::Emitter out; out.SetDoublePrecision(17); out<<cfg;
      std::string temporary=output_+".tmp";
      {std::ofstream file(temporary); if(!file) throw std::runtime_error("cannot write session config"); file<<out.c_str()<<'\n'; if(!file) throw std::runtime_error("failed writing session config");}
      // 临时文件完整写入后再重命名，避免后续融合进程读取半份配置。
      std::filesystem::rename(temporary,output_);
      success=done=true;
      std::cout<<"本次检查通过：delta_s="<<report["delta_s"]<<"；会话矩阵配置："<<output_<<std::endl;
    } else if(elapsed>=timeout_ || report["state"]=="fault") {
      // 返回非零；run.sh 的 set -e 阻止启动融合，不会回退到历史时差。
      done=true;
      std::cerr<<"时间检查未通过，不沿用历史时差。报告："<<output_<<".timing.json\n";
    }
    if(done && on_done) on_done();
  }
  Calibration config_;
  std::string output_;
  double timeout_,started_=0,last_log_=-5.;
  HostTimeAlignment alignment_;
  std::set<std::string> sources_;
  std::mutex mutex_;
  std::array<std::string,4> topics_;
  std::array<rclcpp::SubscriptionBase::SharedPtr,4> subscriptions_;
  std::array<uint64_t,4> received_{};
  std::array<double,4> last_callback_{},last_header_{},middleware_age_{};
  rclcpp::CallbackGroup::SharedPtr imu_group_,cloud_group_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace dual_airy
int main(int argc,char **argv) {
  try {
    std::string input,output;
    double timeout=60.;
    for(int i=1;i<argc;++i) {
      std::string arg=argv[i];
      if(i+1>=argc) throw std::runtime_error("expected --extrinsics FILE --output FILE [--timeout SECONDS]");
      if(arg=="--extrinsics") input=argv[++i];
      else if(arg=="--output") output=argv[++i];
      else if(arg=="--timeout") timeout=std::stod(argv[++i]);
      else throw std::runtime_error("unknown argument: "+arg);
    }
    if(input.empty() || output.empty() || !std::isfinite(timeout) || timeout<12.) throw std::runtime_error("invalid time-check arguments (timeout >=12)");
    if(std::filesystem::exists(output)) throw std::runtime_error("refusing to overwrite existing session config");
    auto cfg=dual_airy::load_calibration(input);
    auto parent=std::filesystem::path(output).parent_path();
    if(!parent.empty()) std::filesystem::create_directories(parent);
    rclcpp::init(1,argv);
    auto node=std::make_shared<dual_airy::TimeCheck>(std::move(cfg),output,timeout);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),3); executor.add_node(node);
    node->on_done=[&executor]{executor.cancel();};
    executor.spin();
    bool success=node->success;
    rclcpp::shutdown(); return success?0:2;
  } catch(const std::exception &e) {
    std::cerr<<"dual_airy_time_check: "<<e.what()<<'\n';
    if(rclcpp::ok()) rclcpp::shutdown();
    return 2;
  }
}
