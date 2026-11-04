#include "dual_airy/core.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <utility>

namespace dual_airy {
namespace {
template <class T> T read(const uint8_t *src) {
  T value;
  std::memcpy(&value, src, sizeof(T));
  return value;
}
template <class T> void write(uint8_t *dst, T value) {
  std::memcpy(dst, &value, sizeof(T));
}
void increment(Json &j, const char *key, size_t value = 1) {
  j[key] = j.at(key).get<uint64_t>() + value;
}
void require(bool ok, const char *message) {
  if (!ok) throw std::runtime_error(message);
}
}  // namespace
Calibration load_calibration(const std::string &path) {
  // 先验证方向、矩阵和时间约定。配置格式通过不表示该外参已达到精度验收标准。
  Calibration c;
  c.document = YAML::LoadFile(path);
  require(c.document["transform"].as<std::string>() == "front <- rear", "transform must be front <- rear");
  require(c.document["time_convention"].as<std::string>() == "t_front = t_rear + delta_s", "unsupported time convention");
  require(!c.document["R"] && !c.document["t_m"], "use only the 4x4 T_front_rear matrix, not R/t_m");
  auto m = c.document["T_front_rear"];
  require(m.IsSequence() && m.size() == 4, "T_front_rear must have four rows");
  for (size_t i = 0; i < 4; ++i) {
    require(m[i].IsSequence() && m[i].size() == 4, "T_front_rear must be 4x4");
    for (size_t j = 0; j < 4; ++j) {
      c.matrix[i][j] = m[i][j].as<double>();
      require(std::isfinite(c.matrix[i][j]), "nonfinite matrix value");
    }
  }
  for (size_t j = 0; j < 4; ++j)
    require(std::abs(c.matrix[3][j] - (j == 3 ? 1. : 0.)) < 1e-12, "matrix last row must be [0,0,0,1]");
  double orthogonal_error = 0;
  for (size_t i = 0; i < 3; ++i) for (size_t j = 0; j < 3; ++j) {
    double dot = 0;
    for (size_t k = 0; k < 3; ++k) dot += c.matrix[k][i] * c.matrix[k][j];
    orthogonal_error += std::pow(dot - (i == j ? 1. : 0.), 2);
  }
  const auto &r = c.matrix;
  double det = r[0][0]*(r[1][1]*r[2][2]-r[1][2]*r[2][1])
             - r[0][1]*(r[1][0]*r[2][2]-r[1][2]*r[2][0])
             + r[0][2]*(r[1][0]*r[2][1]-r[1][1]*r[2][0]);
  require(std::sqrt(orthogonal_error) < 1e-5 && std::abs(det - 1) < 1e-5, "matrix rotation must be orthonormal with determinant +1");
  c.delta = c.document["delta_s"].as<double>();
  require(std::isfinite(c.delta), "delta_s must be finite");
  c.status = c.document["status"].as<std::string>("unspecified");
  c.valid_for = c.document["valid_for"].as<std::string>("unspecified");
  return c;
}
float coordinate(const Point &p, size_t axis) { return read<float>(p.bytes.data() + axis*4); }
std::vector<Point> decode(const sensor_msgs::msg::PointCloud2 &msg) {
  // PointField 类型编号：7=FLOAT32，4=UINT16，8=FLOAT64；字段偏移必须与驱动一致。
  const std::array<std::string, 6> names = {"x", "y", "z", "intensity", "ring", "timestamp"};
  const std::array<uint32_t, 6> offsets = {0,4,8,12,16,18};
  const std::array<uint8_t, 6> types = {7,7,7,7,4,8};
  require(!msg.is_bigendian && msg.point_step == point_bytes, "expected little-endian Airy XYZIRT (26 bytes/point)");
  for (size_t i = 0; i < names.size(); ++i) {
    size_t matches = 0;
    for (const auto &f : msg.fields) if (f.name == names[i]) {
      require(f.offset == offsets[i] && f.datatype == types[i] && f.count == 1, "unexpected XYZIRT field layout");
      ++matches;
    }
    require(matches == 1, "missing or duplicate XYZIRT field");
  }
  require(uint64_t(msg.row_step) >= uint64_t(msg.width)*point_bytes &&
          uint64_t(msg.data.size()) >= uint64_t(msg.row_step)*msg.height,
          "incomplete PointCloud2 data");
  std::vector<Point> out;
  out.reserve(uint64_t(msg.height)*msg.width);
  for (uint32_t row = 0; row < msg.height; ++row) for (uint32_t col = 0; col < msg.width; ++col) {
    // 按 row_step 逐行读取，兼容有行填充的 organized cloud；不假定整帧紧密相连。
    const auto *src = msg.data.data() + size_t(row)*msg.row_step + size_t(col)*point_bytes;
    double t = read<double>(src+18);
    if (!(std::isfinite(read<float>(src)) && std::isfinite(read<float>(src+4)) &&
          std::isfinite(read<float>(src+8)) && std::isfinite(t) && t > 0)) continue;
    Point p;
    std::memcpy(p.bytes.data(), src, point_bytes);
    p.time = t;
    out.push_back(p);
  }
  require(!out.empty(), "no finite points");
  return out;
}
void transform(std::vector<Point> &points, const Calibration &c) {
  for (auto &p : points) {
    // 先保存原始 XYZ，避免逐轴写回影响后续轴计算；仅后雷达调用此变换。
    double x = coordinate(p,0), y = coordinate(p,1), z = coordinate(p,2);
    for (size_t i = 0; i < 3; ++i) {
      const auto &r = c.matrix[i];
      float value = static_cast<float>(r[0]*x + r[1]*y + r[2]*z + r[3]);
      require(std::isfinite(value), "nonfinite transformed coordinate");
      write(p.bytes.data()+4*i, value);
    }
    // 逐点时间也加同一个时差。只改 header 而不改逐点 timestamp 会破坏融合时间窗。
    // 此处没有按运动轨迹修改 XYZ，因此不是运动去畸变。
    p.time += c.delta;
    require(std::isfinite(p.time) && p.time > 0, "invalid corrected point time");
    write(p.bytes.data()+18, p.time);
  }
}
sensor_msgs::msg::PointCloud2::UniquePtr cloud(const std::vector<Point> &points, const std::string &frame) {
  require(!points.empty(), "cannot publish empty cloud");
  auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  // 上游已按采样时间排序：header 用首点采样时间，不用主机 now() 掩盖缓存等待。
  const double t = points.front().time;
  require(t > 0 && t < double(std::numeric_limits<int32_t>::max()), "point timestamp outside ROS2 Time range");
  int64_t ns = std::llround(t*1e9);
  msg->header.stamp.sec = static_cast<int32_t>(ns/1000000000);
  msg->header.stamp.nanosec = static_cast<uint32_t>(ns%1000000000);
  msg->header.frame_id = frame;
  require(points.size() <= std::numeric_limits<uint32_t>::max()/point_bytes, "cloud too large");
  msg->height = 1;
  msg->width = static_cast<uint32_t>(points.size());
  const std::array<std::string, 6> names = {"x", "y", "z", "intensity", "ring", "timestamp"};
  const std::array<uint32_t, 6> offsets = {0,4,8,12,16,18};
  const std::array<uint8_t, 6> types = {7,7,7,7,4,8};
  for (size_t i=0; i<names.size(); ++i) {
    sensor_msgs::msg::PointField f;
    f.name=names[i]; f.offset=offsets[i]; f.datatype=types[i]; f.count=1;
    msg->fields.push_back(f);
  }
  msg->point_step = point_bytes;
  msg->row_step = msg->width*point_bytes;
  msg->is_bigendian = false;
  msg->is_dense = true;
  // 只复制每点的 26 字节线上记录；intensity/ring 保持原值，后雷达 ring 不重新编号。
  msg->data.resize(msg->row_step);
  for (size_t i=0; i<points.size(); ++i)
    std::memcpy(msg->data.data()+i*point_bytes, points[i].bytes.data(), point_bytes);
  return msg;
}
FusionBuffer::FusionBuffer(Calibration c) : calibration(std::move(c)) {}
void FusionBuffer::add_front(double start, std::vector<Point> points) {
  // 时间倒退可能是 bag 回绕或雷达重启，交给节点锁定 fault，避免混合两个时钟会话。
  require(std::isfinite(start) && start > last_front, "front timestamp reversed; restart node and downstream consumer");
  last_front = start;
  front.push_back({start, std::move(points)});
  increment(counts, "front_received");
  if (front.size() > 30) { front.pop_front(); increment(counts,"overflow_dropped"); }
}
void FusionBuffer::add_rear(double start, std::vector<Point> points) {
  require(std::isfinite(start) && start > last_rear, "rear timestamp reversed; restart node and downstream consumer");
  transform(points, calibration);
  last_rear = start;
  for (const auto &p : points) rear_end = std::max(rear_end, p.time);
  // 帧边界与帧内点同时换算到前雷达时钟；last_rear 仍检查原始后雷达时间单调性。
  rear.push_back({start+calibration.delta, std::move(points)});
  increment(counts,"rear_received");
  if (rear.size() > 40) { rear.pop_front(); increment(counts,"rear_overflow_dropped"); }
}
std::optional<Output> FusionBuffer::next(bool final) {
  // 输出窗口为 [本帧前雷达开始时间, 下一帧前雷达开始时间)。
  // 必须等下一帧才能确定边界，因此通常缓存约一帧；这不会把 10 Hz 变成 5 Hz。
  while (!front.empty() && (front.size()>1 || final)) {
    const auto &f = front.front();
    double end = front.size()>1 ? front[1].start : f.start+.1;
    // 遇到较大的前雷达缺口，按 nominal 0.1 s 封口，不把缺口拼成一个超长扫描。
    if (end-f.start > .15) end = f.start+.1;
    // 后雷达点时间尚未到达窗口末端时先等待；这是覆盖端点检查，不证明中间没有丢帧。
    if (rear_end < end-1e-5 && !final) return std::nullopt;
    if (rear.empty() || rear.front().start > f.start+.002) {
      front.pop_front(); increment(counts,"startup_dropped"); continue;
    }
    if (rear_end < end-1e-5) {
      front.pop_front(); increment(counts,"unmatched_dropped"); continue;
    }
    Output out;
    out.points.reserve(f.points.size()*2);
    for (const auto &p : f.points) if (p.time >= f.start && p.time < end) out.points.push_back(p);
    out.front_count = out.points.size();
    // 同一输出可使用相邻两帧的后雷达点，按校正后的逐点时间选取，而非最近帧整帧相加。
    for (const auto &r : rear) {
      if (r.start >= end) break;
      for (const auto &p : r.points) if (p.time >= f.start && p.time < end) out.points.push_back(p);
    }
    out.rear_count = out.points.size()-out.front_count;
    front.pop_front();
    if (!out.front_count || !out.rear_count) { increment(counts,"unmatched_dropped"); continue; }
    // 保证输出逐点时间非递减；相同时间保留原输入次序。无 voxel 降采样。
    std::stable_sort(out.points.begin(), out.points.end(), [](const Point &a,const Point &b) {return a.time<b.time;});
    increment(counts,"generated");
    increment(counts,"front_points",out.front_count);
    increment(counts,"rear_points",out.rear_count);
    while (rear.size()>1 && rear[1].start <= end) rear.pop_front();
    return out;
  }
  return std::nullopt;
}
const std::array<std::string,4> HostTimeAlignment::keys = {"front_imu","rear_imu","front_points","rear_points"};
void HostTimeAlignment::observe(size_t stream, double header, double received) {
  if (!std::isfinite(header) || !std::isfinite(received)) return;
  auto &d = data_.at(stream);
  if (!d.empty() && header <= d.back()[0]) fault_ = "sensor_timestamp_reversed; restart fusion and downstream consumer";
  d.push_back({header,received});
  if (d.size()>5000) d.pop_front();
}
double percentile(std::vector<double> a, double p) {
  if (a.empty()) throw std::runtime_error("empty percentile");
  std::sort(a.begin(),a.end());
  double pos = p*(a.size()-1);
  size_t lo = static_cast<size_t>(pos), hi = static_cast<size_t>(std::ceil(pos));
  return a[lo]+(a[hi]-a[lo])*(pos-lo);
}
Json HostTimeAlignment::report(double now) {
  Json base={{"state","warming_up"},{"method","same_host_receive_offset"},
    {"limitations","Includes differential transport delay; raw output is not deskewed."},
    {"delta_s",delta_ ? Json(*delta_) : Json(nullptr)}};
  if (!fault_.empty()) {base["state"]="fault"; base["reason"]=fault_; return base;}
  double hi=std::numeric_limits<double>::infinity();
  for (const auto &d : data_) {
    if (d.empty()) {base["reason"]="waiting_for_front_rear_points_and_imus"; return base;}
    hi=std::min(hi,d.back()[1]);
  }
  for (const auto &d : data_) if (now-d.back()[1]>1.) {
    base["state"]="stale"; base["reason"]="one_or_more_streams_missing"; return base;
  }
  // 取四路共同覆盖的最近 10 秒接收窗口；接收时间与传感器时间允许不同起点。
  std::array<std::vector<double>,4> offsets;
  bool enough = true;
  for (size_t i=0; i<4; ++i) {
    double first=0,last=0;
    for (const auto &row : data_[i]) if (row[1]>=hi-10. && row[1]<=hi) {
      if (offsets[i].empty()) first=row[1];
      last=row[1]; offsets[i].push_back(row[1]-row[0]);
    }
    base["counts"][keys[i]]=offsets[i].size();
    if (offsets[i].size()<(i<2?500u:30u) || last-first<4.8) enough=false;
  }
  if (!enough) {base["reason"]="collecting_startup_window"; return base;}
  std::array<double,4> median,jitter;
  bool excessive=false;
  for (size_t i=0;i<4;++i) {
    // offsets = 主机接收时间 - 传感器 header；以中位数抗偶发调度/传输抖动。
    median[i]=percentile(offsets[i],.5);
    jitter[i]=percentile(offsets[i],.95)-percentile(offsets[i],.05);
    base["receive_jitter_p95_p05_ms"][keys[i]]=jitter[i]*1000.;
    // 抖动采用 P95-P05，IMU 上限 20 ms、点云 40 ms；这不是校准精度的保证。
    if (jitter[i]>(i<2?.020:.040)) excessive=true;
  }
  // 有效时差的符号：delta = median(offset_rear) - median(offset_front)。
  // 后雷达点实际采用 t_R + delta；两路的固定传输延迟差也会进入该估计。
  double dimu=median[1]-median[0], dcloud=median[3]-median[2];
  base["imu_delta_s"]=dimu; base["points_delta_s"]=dcloud;
  base["disagreement_ms"]=std::abs(dimu-dcloud)*1000.;
  if (excessive) {base["state"]="unstable"; base["reason"]="receive_jitter_too_large"; return base;}
  // 冻结前 IMU 与点云估计须相差不超过 6 ms；冻结后用较宽门限观察传输变化。
  if (std::abs(dimu-dcloud)>(delta_?.020:.006)) {
    base["state"]="unstable"; base["reason"]="imu_points_alignment_disagree"; return base;
  }
  if (delta_ && std::abs(dimu-dcloud)>.006) base["warning"]="cloud_delivery_asymmetry; frozen IMU clock correction unchanged";
  if (now-last_update_>=.9) {
    previous_.push_back(dimu); last_update_=now;
    if (previous_.size()>3) previous_.pop_front();
  }
  // 至少三个约每秒一次的 IMU 估计，极差 <=2 ms 才冻结中位数。
  // 融合过程中不滚动修改 delta，避免逐点时间跳变；传感器重启后须重新启动脚本。
  if (!delta_) {
    if (previous_.size()<3 || *std::max_element(previous_.begin(),previous_.end())- *std::min_element(previous_.begin(),previous_.end())>.002) {
      base["reason"]="waiting_for_stable_estimates"; return base;
    }
    delta_=percentile(std::vector<double>(previous_.begin(),previous_.end()),.5);
  }
  base["delta_s"]=*delta_;
  if (std::abs(dimu-*delta_)>.006 || std::abs(dcloud-*delta_)>.020) {
    base["state"]="unstable"; base["reason"]="time_relation_changed; restart to recalibrate startup offset"; return base;
  }
  base["state"]="ready"; base["reason"]="startup_correction_frozen";
  return base;
}
double wall_seconds() { return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count(); }
double steady_seconds() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
void Samples::add(double v) { values_.push_back(v); if(values_.size()>100) values_.pop_front(); }
Json Samples::summary() const {
  if(values_.empty()) return nullptr;
  std::vector<double> v(values_.begin(),values_.end());
  return {{"p50",percentile(v,.5)},{"p95",percentile(v,.95)},{"max",*std::max_element(v.begin(),v.end())},{"n",v.size()}};
}
}  // namespace dual_airy
