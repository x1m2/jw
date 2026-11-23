#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <optional>
#include <vector>
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

// Keep side protection along the body and fill the corners beside enabled
// front/rear stop zones. Slowdown distances never extend the side zones.
inline ZoneRectangle sideStopRectangle(
    std::size_t direction, double robot_width, double robot_length,
    double thickness, const DetectionZoneConfig &front,
    const DetectionZoneConfig &rear, double front_origin_x,
    double rear_origin_x) {
  const double x_min = rear.enabled
      ? std::min(-robot_length * 0.5, rear_origin_x - rear.stop_distance)
      : -robot_length * 0.5;
  const double x_max = front.enabled
      ? std::max(robot_length * 0.5, front_origin_x + front.stop_distance)
      : robot_length * 0.5;
  const double half_width = robot_width * 0.5;
  return direction == 2U
      ? ZoneRectangle{x_min, x_max, half_width, half_width + thickness}
      : ZoneRectangle{x_min, x_max, -half_width - thickness, -half_width};
}

// Check only the space beyond the stop-zone outer boundary. Normal stop zones
// remain independent and continue protecting the inner strip.
inline ZoneRectangle clearanceCheckRectangle(std::size_t direction,
    const ZoneRectangle &stop_box, double robot_width, double robot_length,
    double distance) {
  if (direction == 1U) {
    const double rear = std::min(-robot_length * 0.5, stop_box.x_min);
    return {rear-distance, rear, -robot_width*0.5, robot_width*0.5};
  }
  const double side = direction == 2U ? stop_box.y_max : stop_box.y_min;
  return {stop_box.x_min, stop_box.x_max,
          direction == 2U ? side : side-distance,
          direction == 2U ? side+distance : side};
}

inline double gridClearance(std::size_t direction, const ZoneRectangle &box,
    double check_distance, int width, int height, double resolution,
    std::size_t minimum_points, std::size_t obstacle_cell_threshold,
    const std::vector<HeightCell> *front, const std::vector<HeightCell> *rear,
    std::size_t &count) {
  double clearance = check_distance;
  count = 0U;
  for (int my = 0; my < height; ++my) {
    const double y = (my-height*0.5+0.5)*resolution;
    if (y < box.y_min || y > box.y_max) continue;
    // A cell centered on the shared boundary belongs to the stop zone.
    if ((direction == 2U && y <= box.y_min) ||
        (direction == 3U && y >= box.y_max)) continue;
    for (int mx = 0; mx < width; ++mx) {
      const double x = (mx-width*0.5+0.5)*resolution;
      if (x < box.x_min || x > box.x_max) continue;
      if (direction == 1U && x >= box.x_max) continue;
      const auto index = static_cast<std::size_t>(my*width+mx);
      if ((front && (*front)[index].occupied(minimum_points)) ||
          (rear && (*rear)[index].occupied(minimum_points))) {
        ++count; // Sources must qualify independently; shared cells count once.
        const double distance = direction == 1U ? box.x_max-x :
            direction == 2U ? y-box.y_min : box.y_max-y;
        clearance = std::min(clearance, distance-resolution*0.5);
      }
    }
  }
  return count >= obstacle_cell_threshold ? std::max(0.0, clearance) : check_distance;
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
