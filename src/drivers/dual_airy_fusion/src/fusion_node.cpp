#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <utility>

#include "dual_airy/core.hpp"
#include "dual_airy/startup_time_check.hpp"

namespace dual_airy {
// 单节点两阶段：启动检查通过后冻结时差，再创建融合订阅。
// 检查模块保留到节点退出；融合回调和状态定时器在默认互斥组内串行执行。
class FusionNode : public rclcpp::Node {
 public:
  FusionNode()
      : Node("dual_airy_fusion_node"),
        buffer_(load_calibration(declare_parameter<std::string>(
            "extrinsics_file", "config/dual_airy_fusion.yaml"))) {
    front_topic_ =
        declare_parameter<std::string>("front_topic", "/rslidar_front/points");
    rear_topic_ =
        declare_parameter<std::string>("rear_topic", "/rslidar_rear/points");
    auto output_topic = declare_parameter<std::string>(
        "output_topic", "/dual_airy/points_merged");
    frame_ = declare_parameter<std::string>("output_frame", "rslidar_front");
    stats_path_ = declare_parameter<std::string>("stats_file", "");
    reliability_ =
        declare_parameter<std::string>("input_reliability", "reliable");
    if (reliability_ != "reliable" && reliability_ != "best_effort")
      throw std::runtime_error("invalid input_reliability");
    auto_time_ = declare_parameter<bool>("auto_time", true);
    auto timeout = declare_parameter<double>("startup_timeout_s", 60.);
    auto session_file = declare_parameter<std::string>("session_file", "");
    auto front_imu =
        declare_parameter<std::string>("front_imu_topic", "/rslidar_front/imu");
    auto rear_imu =
        declare_parameter<std::string>("rear_imu_topic", "/rslidar_rear/imu");
    // Reliable 发布者允许 Best Effort 订阅者连接，但后者不会获得可靠重传保证。
    // DDS 发布模式和传输配置由运行环境决定。
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        output_topic, rclcpp::QoS(10).reliable());
    status_ = create_publisher<std_msgs::msg::String>("/dual_airy/status", 5);
    // parent=前雷达、child=后雷达，后雷达坐标在前雷达中的位姿即同一个
    // T_front_rear。 此处只发布传感器间
    // TF，不推算机身安装参数或改变雷达原生坐标轴。
    broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now();
    tf.header.frame_id = frame_;
    tf.child_frame_id = "rslidar_rear";
    const auto &m = buffer_.calibration.matrix;
    tf.transform.translation.x = m[0][3];
    tf.transform.translation.y = m[1][3];
    tf.transform.translation.z = m[2][3];
    tf2::Matrix3x3 rotation(m[0][0], m[0][1], m[0][2], m[1][0], m[1][1],
                            m[1][2], m[2][0], m[2][1], m[2][2]);
    tf2::Quaternion q;
    rotation.getRotation(q);
    q.normalize();
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();
    broadcaster_->sendTransform(tf);
    if (auto_time_) {
      startup_ = std::make_unique<StartupTimeCheck>(
          *this, buffer_.calibration, session_file, timeout,
          std::array<std::string, 4>{front_imu, rear_imu, front_topic_,
                                     rear_topic_},
          reliability_, status_);
    } else {
      start_fusion();
    }
    // 200 ms 定时器只管状态和停流末帧；融合由点云回调驱动，并非 5 Hz 定时发布。
    timer_ =
        create_wall_timer(std::chrono::milliseconds(200), [this] { idle(); });
    RCLCPP_DEBUG(get_logger(), "Calibration status=%s; valid_for=%s",
                buffer_.calibration.status.c_str(),
                buffer_.calibration.valid_for.c_str());
  }
  bool failed() const { return state_.phase == Phase::Fault; }
  void save() {
    if (stats_path_.empty()) return;
    std::ofstream out(stats_path_);
    if (out) out << latest_.dump(2) << '\n';
  }

 private:
  enum class Phase { StartupCheck, Fusing, Fault };
  struct RuntimeState {
    Phase phase = Phase::StartupCheck;
    std::string reason;
  };
  void set_fault(std::string reason) {
    state_ = {Phase::Fault, std::move(reason)};
    RCLCPP_ERROR(get_logger(), "Node fault: %s", state_.reason.c_str());
  }
  void start_fusion() {
    // 检查期间不向 FusionBuffer 加点；切换后从新点云开始，避免混用历史时差。
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    if (reliability_ == "best_effort")
      qos.best_effort();
    else
      qos.reliable();
    front_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        front_topic_, qos,
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg,
               const rclcpp::MessageInfo &info) {
          receive(*msg, false, info);
        });
    rear_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        rear_topic_, qos,
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg,
               const rclcpp::MessageInfo &info) {
          receive(*msg, true, info);
        });
    last_front_wall_ = last_rear_wall_ = steady_seconds();
    state_ = {Phase::Fusing, {}};
    RCLCPP_DEBUG(get_logger(),
                "Phase 2/2: XYZIRT fusion; delta=%+.9fs; no motion deskew",
                buffer_.calibration.delta);
  }
  void receive(const sensor_msgs::msg::PointCloud2 &msg, bool rear,
               const rclcpp::MessageInfo &info) {
    if (state_.phase != Phase::Fusing) return;
    const double begin = steady_seconds();
    const double wall = wall_seconds();
    const double stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
    const double previous_stamp = rear ? buffer_.last_rear : buffer_.last_front;
    const double header_ms = std::isfinite(previous_stamp) ? (stamp - previous_stamp) * 1000. : 0.;
    const double interval_ms = std::isfinite(previous_stamp)
        ? (begin - (rear ? last_rear_wall_ : last_front_wall_)) * 1000. : 0.;
    const auto received = info.get_rmw_message_info().received_timestamp;
    const double wait_ms = received > 0 ? (wall - received * 1e-9) * 1000. : -1.;
    try {
      auto points = decode(msg);
      stages_["decode"].add((steady_seconds() - begin) * 1000.);
      double added = steady_seconds();
      if (rear) {
        buffer_.add_rear(stamp, std::move(points));
        stages_["rear_transform"].add((steady_seconds() - added) * 1000.);
        last_rear_wall_ = begin;
      } else {
        buffer_.add_front(stamp, std::move(points));
        last_front_wall_ = begin;
      }
      publish_ready(false);
    } catch (const std::exception &error) {
      // 时间回绕或字段错误后锁定 fault，避免静默混入旧缓存；修复原因后重启。
      set_fault(error.what());
    }
    const double finished = steady_seconds();
    const double processing_ms = (finished - begin) * 1000.;
    callback_.add(processing_ms);
    auto &last_warning = last_input_warning_[rear ? 1 : 0];
    // 固定阈值用于当前 10 Hz 输入；无效 DDS 接收时间不参与等待时间判断。
    const bool slow_wait = wait_ms > 100. && wait_ms < 60000.;
    if (finished - last_warning >= 5. &&
        (header_ms > 150. || interval_ms > 150. || processing_ms > 50. || slow_wait)) {
      last_warning = finished;
      std::ostringstream text;
      const auto append = [&text](const char *name, double ms, double limit) {
        if (ms > limit) text << name << '=' << ms << "ms>" << limit << "ms ";
      };
      append("header_dt", header_ms, 150.);
      append("callback_dt", interval_ms, 150.);
      append("processing", processing_ms, 50.);
      if (slow_wait) append("receive_to_callback", wait_ms, 100.);
      RCLCPP_WARN(get_logger(), "[CLOUD_INPUT] %s %s",
                  rear ? "rear" : "front", text.str().c_str());
    }
  }
  void publish_ready(bool final) {
    while (true) {
      double begin = steady_seconds();
      auto out = buffer_.next(final);
      if (!out) break;
      stages_["assemble_sort"].add((steady_seconds() - begin) * 1000.);
      begin = steady_seconds();
      auto msg = cloud(out->points, frame_);
      double stamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
      double sending = steady_seconds();
      stages_["message_build"].add((sending - begin) * 1000.);
      // unique_ptr 移交消息，避免一次应用层消息复制；DDS
      // 序列化/传输仍可能复制。 publish 返回不表示接收端已收到完整点云。
      publisher_->publish(std::move(msg));
      double returned = steady_seconds();
      stages_["publish_call"].add((returned - sending) * 1000.);
      uint64_t n = buffer_.counts["publish_returned"].get<uint64_t>() + 1;
      buffer_.counts["publish_returned"] = n;
      last_publish_ = returned;
      publish_times_.push_back(returned);
      if (publish_times_.size() > 101) publish_times_.pop_front();
      output_lag_ = buffer_.last_front - stamp;
    }
  }
  Json performance() const {
    // 最近 100 次耗时及调用返回频率；output_lag
    // 是采样时间差，不是端到端网络延迟。
    Json stages = Json::object();
    for (const auto &s : stages_) stages[s.first] = s.second.summary();
    Json hz = nullptr;
    if (last_publish_ && steady_seconds() - *last_publish_ > 1.)
      hz = 0.;
    else if (publish_times_.size() > 1 &&
             publish_times_.back() > publish_times_.front())
      hz = (publish_times_.size() - 1) /
           (publish_times_.back() - publish_times_.front());
    return {{"stage_ms", stages},
            {"callback_ms", callback_.summary()},
            {"publish_call_hz", hz},
            {"since_last_publish_s",
             last_publish_ ? Json(steady_seconds() - *last_publish_)
                           : Json(nullptr)},
            {"latest_front_minus_output_stamp_s",
             output_lag_ ? Json(*output_lag_) : Json(nullptr)},
            {"queued_front", buffer_.front.size()},
            {"queued_rear", buffer_.rear.size()},
            {"matched_subscribers", publisher_->get_subscription_count()}};
  }
  void idle() {
    if (state_.phase == Phase::StartupCheck) {
      try {
        const auto result = startup_->poll();
        if (result == StartupTimeCheck::State::Ready) {
          buffer_ = FusionBuffer(startup_->calibration());
          start_fusion();
        } else if (result == StartupTimeCheck::State::Failed) {
          set_fault("startup_time_check_failed");
        }
      } catch (const std::exception &error) {
        set_fault(error.what());
      }
      if (state_.phase == Phase::Fault) {
        rclcpp::shutdown();
      }
      return;
    }
    const double t = steady_seconds();
    // 前雷达停流 1.5 s 后尝试封口；后雷达覆盖不足仍由 next(true) 丢弃。
    if (state_.phase == Phase::Fusing && !buffer_.front.empty() &&
        t - last_front_wall_ > 1.5) {
      try {
        publish_ready(true);
      } catch (const std::exception &e) {
        set_fault(e.what());
      }
    }
    if (t - last_status_ < 1.) return;
    last_status_ = t;
    std::string state = "ready";
    if (state_.phase == Phase::Fault)
      state = "fault";
    else if (buffer_.counts["front_received"] == 0 ||
             buffer_.counts["rear_received"] == 0)
      state = "waiting_for_points";
    else if (t - std::min(last_front_wall_, last_rear_wall_) > 1.)
      state = "stale";
    else if (!last_publish_)
      state = "waiting_for_complete_window";
    else if (t - *last_publish_ > 1.)
      state = "output_stalled";
    if (state_.phase == Phase::Fusing && t - last_stall_warning_ >= 5. &&
        (state == "stale" || state == "output_stalled" ||
         (state == "waiting_for_points" &&
          t - std::min(last_front_wall_, last_rear_wall_) > 1.))) {
      last_stall_warning_ = t;
      RCLCPP_WARN(get_logger(), "[CLOUD_STALL] state=%s front_age=%.1fs rear_age=%.1fs",
                  state.c_str(), t - last_front_wall_, t - last_rear_wall_);
    }
    latest_ = {{"state", state},
               {"phase", "fusion"},
               {"mode", auto_time_ ? "auto_time" : "fixed"},
               {"implementation", "cpp"},
               {"version", 5},
               {"delta_s", buffer_.calibration.delta},
               {"counts", buffer_.counts},
               {"performance", performance()},
               {"spatial_calibration_status", buffer_.calibration.status},
               {"spatial_valid_for", buffer_.calibration.valid_for},
               {"limitations",
                "Publish counters measure publish() returns, not delivery. "
                "Effective time offset includes differential delivery delay. "
                "No motion deskew."}};
    if (state_.phase == Phase::Fault) latest_["reason"] = state_.reason;
    std_msgs::msg::String msg;
    msg.data = latest_.dump();
    status_->publish(msg);
    save();
  }
  FusionBuffer buffer_;
  std::unique_ptr<StartupTimeCheck> startup_;
  RuntimeState state_;
  bool auto_time_ = true;
  std::array<double, 2> last_input_warning_{};
  std::string front_topic_, rear_topic_, reliability_;
  std::string frame_, stats_path_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr front_, rear_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::map<std::string, Samples> stages_;
  Samples callback_;
  std::deque<double> publish_times_;
  std::optional<double> last_publish_, output_lag_;
  double last_front_wall_ = 0, last_rear_wall_ = 0,
         last_status_ = 0, last_stall_warning_ = 0;
  Json latest_;
};
}  // namespace dual_airy
int main(int argc, char **argv) {
  try {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<dual_airy::FusionNode>();
    // 检查阶段分组并发；融合阶段只有默认互斥组工作，缓存仍串行修改。
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                      3);
    executor.add_node(node);
    executor.spin();
    node->save();
    if (rclcpp::ok()) rclcpp::shutdown();
    return node->failed() ? 2 : 0;
  } catch (const std::exception &e) {
    std::cerr << "dual_airy_fusion_node: " << e.what() << '\n';
    if (rclcpp::ok()) rclcpp::shutdown();
    return 2;
  }
}
