#pragma once
#include <array>
#include <cstdint>
#include <deque>
#include <limits>
#include <optional>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <yaml-cpp/yaml.h>

namespace dual_airy {
using Json = nlohmann::json;
// Airy 驱动的线上布局：XYZI 各 float32，ring uint16，timestamp float64，共 26 字节。
// 下面 Point 是程序内部缓存；它的 sizeof 不等于线上 point_step，不可整体 memcpy 发布。
constexpr size_t point_bytes = 26;
struct Calibration {
  YAML::Node document;
  // 后雷达 -> 前雷达，列向量 [p_F;1] = matrix * [p_R;1]；平移列单位为米。
  std::array<std::array<double, 4>, 4> matrix{};
  // 后雷达时间换算到前雷达：t_F = t_R + delta。与空间矩阵分开处理。
  double delta = 0;
  std::string status, valid_for;
};
Calibration load_calibration(const std::string &path);
// timestamp 位于第 18 字节，并非 8 字节对齐；用 memcpy 读取，兼容 ARM64。
struct Point {
  std::array<uint8_t, point_bytes> bytes{};
  double time = 0;
};
float coordinate(const Point &p, size_t axis);
std::vector<Point> decode(const sensor_msgs::msg::PointCloud2 &msg);
void transform(std::vector<Point> &points, const Calibration &calibration);
sensor_msgs::msg::PointCloud2::UniquePtr cloud(const std::vector<Point> &points,
                                             const std::string &frame);
struct Frame { double start; std::vector<Point> points; };
struct Output { std::vector<Point> points; size_t front_count = 0, rear_count = 0; };
// 按前雷达扫描时间划窗，允许一帧后雷达数据分配到相邻两个输出窗口。
// 这是原始扫描融合：不使用 SLAM 位姿，也不把所有点运动补偿到同一时刻。
class FusionBuffer {
 public:
  explicit FusionBuffer(Calibration calibration);
  void add_front(double start, std::vector<Point> points);
  void add_rear(double start, std::vector<Point> points);
  // final 仅在输入停止后尝试处理末帧；后雷达覆盖不足时仍丢弃，不补造数据。
  std::optional<Output> next(bool final = false);
  Calibration calibration;
  std::deque<Frame> front, rear;
  double last_front = -std::numeric_limits<double>::infinity();
  double last_rear = -std::numeric_limits<double>::infinity();
  double rear_end = -std::numeric_limits<double>::infinity();
  // generated：已组装；publish_returned/published：publish() 已返回；均不证明接收端收到。
  Json counts = {{"front_received", 0}, {"rear_received", 0}, {"generated", 0},
    {"published", 0}, {"publish_returned", 0}, {"startup_dropped", 0},
    {"unmatched_dropped", 0}, {"overflow_dropped", 0}, {"rear_overflow_dropped", 0},
    {"front_points", 0}, {"rear_points", 0}};
};
// 仅供启动检查进程使用。估计“时钟偏差 + 差分传输延迟”，不是硬件同步器。
// 同时检查 IMU 和点云；通过后冻结本次 delta，由脚本交给独立融合进程。
class HostTimeAlignment {
 public:
  static const std::array<std::string, 4> keys;
  void observe(size_t stream, double header, double received);
  Json report(double now);
 private:
  std::array<std::deque<std::array<double, 2>>, 4> data_;
  std::deque<double> previous_;
  double last_update_ = -std::numeric_limits<double>::infinity();
  std::optional<double> delta_;
  std::string fault_;
};
double percentile(std::vector<double> values, double fraction);
double wall_seconds();
double steady_seconds();
// 最近 100 次耗时的分位数，不是整个运行期间的累计最大耗时。
class Samples {
 public:
  void add(double value);
  Json summary() const;
 private:
  std::deque<double> values_;
};
}  // namespace dual_airy
