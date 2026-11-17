#ifndef PATCHWORK_PCL_GROUND_SEGMENTER_HPP_
#define PATCHWORK_PCL_GROUND_SEGMENTER_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "patchwork/patchworkpp.h"

namespace patchwork {

class PclGroundSegmenter {
 public:
  explicit PclGroundSegmenter(const Params &params)
      : params_(params), patchwork_(params_) {}

  explicit PclGroundSegmenter(rclcpp::Node &node,
                              const std::string &prefix = "patchworkpp")
      : PclGroundSegmenter(loadParams(node, prefix)) {}

  explicit PclGroundSegmenter(const rclcpp::Node::SharedPtr &node,
                              const std::string &prefix = "patchworkpp")
      : PclGroundSegmenter(*node, prefix) {}

  bool segment(const pcl::PointCloud<pcl::PointXYZI> &input,
               pcl::PointCloud<pcl::PointXYZI> &ground,
               pcl::PointCloud<pcl::PointXYZI> &nonground,
               double *time_taken_sec = nullptr,
               std::vector<std::uint8_t> *labels = nullptr) {
    ground.clear();
    nonground.clear();

    if (input.empty()) {
      if (time_taken_sec != nullptr) {
        *time_taken_sec = 0.0;
      }
      return false;
    }

    if (labels != nullptr) {
      labels->assign(input.size(), 0U);
    }

    std::vector<int> row_to_input_index;
    row_to_input_index.reserve(input.size());

    Eigen::MatrixXf cloud_eig(input.size(), 4);
    int row = 0;
    for (std::size_t i = 0; i < input.size(); ++i) {
      const auto &pt = input.points[i];
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) ||
          !std::isfinite(pt.z)) {
        continue;
      }

      cloud_eig(row, 0) = pt.x;
      cloud_eig(row, 1) = pt.y;
      cloud_eig(row, 2) = pt.z;
      cloud_eig(row, 3) = pt.intensity;
      row_to_input_index.push_back(static_cast<int>(i));
      ++row;
    }

    if (row == 0) {
      if (time_taken_sec != nullptr) {
        *time_taken_sec = 0.0;
      }
      return false;
    }

    cloud_eig.conservativeResize(row, Eigen::NoChange);

    const auto t0 = std::chrono::steady_clock::now();
    patchwork_.estimateGround(cloud_eig);
    const auto t1 = std::chrono::steady_clock::now();

    if (time_taken_sec != nullptr) {
      *time_taken_sec =
          std::chrono::duration<double>(t1 - t0).count();
    }

    appendByIndices(input, row_to_input_index, patchwork_.getGroundIndices(),
                    ground);
    appendByIndices(input, row_to_input_index, patchwork_.getNongroundIndices(),
                    nonground);
    if (labels != nullptr) {
      labelByIndices(row_to_input_index, patchwork_.getGroundIndices(), 1U,
                     *labels);
      labelByIndices(row_to_input_index, patchwork_.getNongroundIndices(), 2U,
                     *labels);
    }

    finalizeCloudLike(input, ground);
    finalizeCloudLike(input, nonground);

    return !ground.empty() || !nonground.empty();
  }

  const Params &params() const { return params_; }

  static Params loadParams(rclcpp::Node &node,
                           const std::string &prefix = "patchworkpp") {
    Params params;

    params.verbose = declareOrGet(node, prefix + ".verbose", params.verbose);
    params.enable_RNR =
        declareOrGet(node, prefix + ".enable_RNR", params.enable_RNR);
    params.enable_RVPF =
        declareOrGet(node, prefix + ".enable_RVPF", params.enable_RVPF);
    params.enable_TGR =
        declareOrGet(node, prefix + ".enable_TGR", params.enable_TGR);

    params.num_iter =
        declareOrGet(node, prefix + ".num_iter", params.num_iter);
    params.num_lpr = declareOrGet(node, prefix + ".num_lpr", params.num_lpr);
    params.num_min_pts =
        declareOrGet(node, prefix + ".num_min_pts", params.num_min_pts);
    params.num_zones =
        declareOrGet(node, prefix + ".czm.num_zones", params.num_zones);
    params.num_rings_of_interest = declareOrGet(
        node, prefix + ".num_rings_of_interest",
        params.num_rings_of_interest);

    params.RNR_ver_angle_thr = declareOrGet(
        node, prefix + ".RNR_ver_angle_thr", params.RNR_ver_angle_thr);
    params.RNR_intensity_thr = declareOrGet(
        node, prefix + ".RNR_intensity_thr", params.RNR_intensity_thr);

    params.sensor_height =
        declareOrGet(node, prefix + ".sensor_height", params.sensor_height);
    params.th_seeds =
        declareOrGet(node, prefix + ".th_seeds", params.th_seeds);
    params.th_dist =
        declareOrGet(node, prefix + ".th_dist", params.th_dist);
    params.th_seeds_v =
        declareOrGet(node, prefix + ".th_seeds_v", params.th_seeds_v);
    params.th_dist_v =
        declareOrGet(node, prefix + ".th_dist_v", params.th_dist_v);
    params.max_range =
        declareOrGet(node, prefix + ".max_r", params.max_range);
    params.min_range =
        declareOrGet(node, prefix + ".min_r", params.min_range);
    params.uprightness_thr = declareOrGet(
        node, prefix + ".uprightness_thr", params.uprightness_thr);
    params.adaptive_seed_selection_margin =
        declareOrGet(node, prefix + ".adaptive_seed_selection_margin",
                     params.adaptive_seed_selection_margin);
    params.intensity_thr =
        declareOrGet(node, prefix + ".intensity_thr", params.intensity_thr);

    params.num_sectors_each_zone = declareOrGetIntVector(
        node, prefix + ".czm.num_sectors_each_zone",
        params.num_sectors_each_zone);
    params.num_rings_each_zone = declareOrGetIntVector(
        node, prefix + ".czm.num_rings_each_zone",
        params.num_rings_each_zone);

    params.max_flatness_storage = declareOrGet(
        node, prefix + ".max_flatness_storage", params.max_flatness_storage);
    params.max_elevation_storage = declareOrGet(
        node, prefix + ".max_elevation_storage",
        params.max_elevation_storage);

    params.elevation_thr = declareOrGet(
        node, prefix + ".czm.elevation_thresholds", params.elevation_thr);
    params.flatness_thr = declareOrGet(
        node, prefix + ".czm.flatness_thresholds", params.flatness_thr);

    return params;
  }

 private:
  template <typename T>
  static T declareOrGet(rclcpp::Node &node, const std::string &name,
                        const T &default_value) {
    if (node.has_parameter(name)) {
      T value;
      node.get_parameter(name, value);
      return value;
    }
    return node.declare_parameter<T>(name, default_value);
  }

  static std::vector<int> declareOrGetIntVector(
      rclcpp::Node &node, const std::string &name,
      const std::vector<int> &default_value) {
    std::vector<int64_t> default_i64(default_value.begin(),
                                     default_value.end());
    std::vector<int64_t> values;
    if (node.has_parameter(name)) {
      node.get_parameter(name, values);
    } else {
      values = node.declare_parameter<std::vector<int64_t>>(name, default_i64);
    }
    return std::vector<int>(values.begin(), values.end());
  }

  static void appendByIndices(
      const pcl::PointCloud<pcl::PointXYZI> &input,
      const std::vector<int> &row_to_input_index,
      const Eigen::VectorXi &indices,
      pcl::PointCloud<pcl::PointXYZI> &output) {
    output.reserve(output.size() + static_cast<std::size_t>(indices.size()));
    for (int i = 0; i < indices.size(); ++i) {
      const int row_index = indices(i);
      if (row_index < 0 ||
          row_index >= static_cast<int>(row_to_input_index.size())) {
        continue;
      }

      const int input_index = row_to_input_index[row_index];
      if (input_index < 0 || input_index >= static_cast<int>(input.size())) {
        continue;
      }

      output.push_back(input.points[static_cast<std::size_t>(input_index)]);
    }
  }

  static void labelByIndices(const std::vector<int> &row_to_input_index,
                             const Eigen::VectorXi &indices,
                             std::uint8_t label,
                             std::vector<std::uint8_t> &labels) {
    for (int i = 0; i < indices.size(); ++i) {
      const int row_index = indices(i);
      if (row_index < 0 ||
          row_index >= static_cast<int>(row_to_input_index.size())) {
        continue;
      }
      const int input_index = row_to_input_index[row_index];
      if (input_index >= 0 && input_index < static_cast<int>(labels.size())) {
        labels[static_cast<std::size_t>(input_index)] = label;
      }
    }
  }

  static void finalizeCloudLike(const pcl::PointCloud<pcl::PointXYZI> &input,
                                pcl::PointCloud<pcl::PointXYZI> &cloud) {
    cloud.header = input.header;
    cloud.width = static_cast<std::uint32_t>(cloud.size());
    cloud.height = 1;
    cloud.is_dense = true;
  }

  Params params_;
  PatchWorkpp patchwork_;
};

}  // namespace patchwork

#endif  // PATCHWORK_PCL_GROUND_SEGMENTER_HPP_
