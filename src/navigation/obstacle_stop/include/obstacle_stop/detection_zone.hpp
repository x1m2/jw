#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <optional>
#include "obstacle_stop/height_cell.hpp"

namespace obstacle_stop {

struct DetectionZoneConfig {
  bool enabled{true};
  unsigned sources{1U};  // front=1, rear=2
  double stop_distance{0.1};
  std::optional<double> slowdown_distance;  // front/rear only; side zones have no slowdown
  std::size_t obstacle_cell_threshold{1U};
};

struct ZoneRectangle {
  double x_min{}, x_max{}, y_min{}, y_max{};
  bool contains(double x, double y, double scale = 1.0) const {
    const double cx = (x_min + x_max) * 0.5;
    const double cy = (y_min + y_max) * 0.5;
    return x >= cx - (x_max-x_min)*0.5*scale &&
           x <= cx + (x_max-x_min)*0.5*scale &&
           y >= cy - (y_max-y_min)*0.5*scale &&
           y <= cy + (y_max-y_min)*0.5*scale;
  }
};

// Direction order: front(+X), rear(-X), left(+Y), right(-Y).
inline ZoneRectangle zoneRectangle(std::size_t direction, double ox, double oy,
                                   double start, double length, double span) {
  const double sign = (direction == 1U || direction == 3U) ? -1.0 : 1.0;
  const double a = sign * start;
  const double b = sign * (start + length);
  if (direction < 2U) {
    return {ox+std::min(a,b), ox+std::max(a,b), oy-span*0.5, oy+span*0.5};
  }
  return {ox-span*0.5, ox+span*0.5, oy+std::min(a,b), oy+std::max(a,b)};
}

// Merge occupancy only after each source independently meets its point threshold.
// A cell seen by two sensors remains one cell; point counts are never added.
inline void mergeOccupiedCell(HeightCell &target, const HeightCell &source,
                              std::size_t minimum_points) {
  if (!source.occupied(minimum_points)) return;
  if (target.point_count == 0U) { target = source; return; }
  target.point_count = std::max(target.point_count, source.point_count);
  target.z_min = std::min(target.z_min, source.z_min);
  target.z_max = std::max(target.z_max, source.z_max);
}

}  // namespace obstacle_stop
