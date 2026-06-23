#ifndef COMMON_HPP
#define COMMON_HPP

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

// ROS2 下不要包含这个，它会引入 ROS1 message_traits，导致 MD5Sum /
// sensor_msgs::PointCloud2 报错。 #include <pcl_ros/point_cloud.hpp>

// CLASSES
constexpr double SENSOR_HEIGHT = 1.73;

constexpr int UNLABELED = 0;
constexpr int OUTLIER = 1;
constexpr int NUM_ALL_CLASSES = 34;
constexpr int ROAD = 40;
constexpr int PARKING = 44;
constexpr int SIDEWALKR = 48;
constexpr int OTHER_GROUND = 49;
constexpr int BUILDING = 50;
constexpr int FENSE = 51;
constexpr int LANE_MARKING = 60;
constexpr int VEGETATION = 70;
constexpr int TERRAIN = 72;

constexpr int TRUEPOSITIVE = 3;
constexpr int TRUENEGATIVE = 2;
constexpr int FALSEPOSITIVE = 1;
constexpr int FALSENEGATIVE = 0;

inline constexpr int NUM_ZEROS = 5;
inline constexpr double VEGETATION_THR = -SENSOR_HEIGHT * 3.0 / 4.0;

/** Euclidean Velodyne coordinate, including intensity and ring number, and
 * label. */
struct PointXYZILID {
  PCL_ADD_POINT4D;
  float intensity;
  std::uint16_t label;
  std::uint16_t id;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZILID,
                                  (float, x, x)(float, y, y)(float, z, z)(
                                      float, intensity,
                                      intensity)(std::uint16_t, label,
                                                 label)(std::uint16_t, id, id))

inline const std::vector<int> outlier_classes = {UNLABELED, OUTLIER};

inline const std::vector<int> ground_classes = {
    ROAD, PARKING, SIDEWALKR, OTHER_GROUND, LANE_MARKING, VEGETATION, TERRAIN};

inline const std::vector<int> ground_classes_except_terrain = {
    ROAD, PARKING, SIDEWALKR, OTHER_GROUND, LANE_MARKING};

inline const std::vector<int> traversable_ground_classes = {
    ROAD, PARKING, LANE_MARKING, OTHER_GROUND};

inline void PointXYZILID2XYZI(pcl::PointCloud<PointXYZILID>& src,
                              pcl::PointCloud<pcl::PointXYZI>::Ptr dst) {
  dst->points.clear();
  dst->points.reserve(src.points.size());

  for (const auto& pt : src.points) {
    pcl::PointXYZI pt_xyzi;
    pt_xyzi.x = pt.x;
    pt_xyzi.y = pt.y;
    pt_xyzi.z = pt.z;
    pt_xyzi.intensity = pt.intensity;
    dst->points.push_back(pt_xyzi);
  }

  dst->width = static_cast<std::uint32_t>(dst->points.size());
  dst->height = 1;
  dst->is_dense = src.is_dense;
}

inline int count_num_ground(const pcl::PointCloud<PointXYZILID>& pc) {
  int num_ground = 0;

  for (const auto& pt : pc.points) {
    const auto iter =
        std::find(ground_classes.begin(), ground_classes.end(), pt.label);

    if (iter != ground_classes.end()) {
      if (pt.label == VEGETATION) {
        if (pt.z < VEGETATION_THR) {
          ++num_ground;
        }
      } else {
        ++num_ground;
      }
    }
  }

  return num_ground;
}

inline int count_num_ground_without_vegetation(
    const pcl::PointCloud<PointXYZILID>& pc) {
  int num_ground = 0;

  const std::vector<int> classes = {ROAD,         PARKING,      SIDEWALKR,
                                    OTHER_GROUND, LANE_MARKING, TERRAIN};

  for (const auto& pt : pc.points) {
    const auto iter = std::find(classes.begin(), classes.end(), pt.label);

    if (iter != classes.end()) {
      ++num_ground;
    }
  }

  return num_ground;
}

inline std::map<int, int> set_initial_gt_counts(
    const std::vector<int>& gt_classes) {
  std::map<int, int> gt_counts;

  for (const auto class_id : gt_classes) {
    gt_counts.insert(std::pair<int, int>(class_id, 0));
  }

  return gt_counts;
}

inline std::map<int, int> count_num_each_class(
    const pcl::PointCloud<PointXYZILID>& pc) {
  auto gt_counts = set_initial_gt_counts(ground_classes);

  for (const auto& pt : pc.points) {
    const auto iter =
        std::find(ground_classes.begin(), ground_classes.end(), pt.label);

    if (iter != ground_classes.end()) {
      if (pt.label == VEGETATION) {
        if (pt.z < VEGETATION_THR) {
          gt_counts[pt.label]++;
        }
      } else {
        gt_counts[pt.label]++;
      }
    }
  }

  return gt_counts;
}

inline int count_num_outliers(const pcl::PointCloud<PointXYZILID>& pc) {
  int num_outliers = 0;

  for (const auto& pt : pc.points) {
    const auto iter =
        std::find(outlier_classes.begin(), outlier_classes.end(), pt.label);

    if (iter != outlier_classes.end()) {
      ++num_outliers;
    }
  }

  return num_outliers;
}

inline void discern_ground(const pcl::PointCloud<PointXYZILID>& src,
                           pcl::PointCloud<PointXYZILID>& ground,
                           pcl::PointCloud<PointXYZILID>& non_ground) {
  ground.clear();
  non_ground.clear();

  for (const auto& pt : src.points) {
    if (pt.label == UNLABELED || pt.label == OUTLIER) {
      continue;
    }

    const auto iter =
        std::find(ground_classes.begin(), ground_classes.end(), pt.label);

    if (iter != ground_classes.end()) {
      if (pt.label == VEGETATION) {
        if (pt.z < VEGETATION_THR) {
          ground.push_back(pt);
        } else {
          non_ground.push_back(pt);
        }
      } else {
        ground.push_back(pt);
      }
    } else {
      non_ground.push_back(pt);
    }
  }
}

inline void discern_ground_without_vegetation(
    const pcl::PointCloud<PointXYZILID>& src,
    pcl::PointCloud<PointXYZILID>& ground,
    pcl::PointCloud<PointXYZILID>& non_ground) {
  ground.clear();
  non_ground.clear();

  for (const auto& pt : src.points) {
    if (pt.label == UNLABELED || pt.label == OUTLIER) {
      continue;
    }

    const auto iter =
        std::find(ground_classes.begin(), ground_classes.end(), pt.label);

    if (iter != ground_classes.end()) {
      if (pt.label != VEGETATION) {
        ground.push_back(pt);
      }
    } else {
      non_ground.push_back(pt);
    }
  }
}

inline void calculate_precision_recall(
    const pcl::PointCloud<PointXYZILID>& pc_curr,
    pcl::PointCloud<PointXYZILID>& ground_estimated, double& precision,
    double& recall, bool consider_outliers = true) {
  const int num_ground_est = static_cast<int>(ground_estimated.points.size());
  const int num_ground_gt = count_num_ground(pc_curr);
  const int num_TP = count_num_ground(ground_estimated);

  if (num_ground_gt <= 0 || num_ground_est <= 0) {
    precision = 0.0;
    recall = 0.0;
    return;
  }

  if (consider_outliers) {
    const int num_outliers_est = count_num_outliers(ground_estimated);
    const int denom = num_ground_est - num_outliers_est;

    precision = denom > 0 ? static_cast<double>(num_TP) / denom * 100.0 : 0.0;
    recall = static_cast<double>(num_TP) / num_ground_gt * 100.0;
  } else {
    precision = static_cast<double>(num_TP) / num_ground_est * 100.0;
    recall = static_cast<double>(num_TP) / num_ground_gt * 100.0;
  }
}

inline void calculate_precision_recall_without_vegetation(
    const pcl::PointCloud<PointXYZILID>& pc_curr,
    pcl::PointCloud<PointXYZILID>& ground_estimated, double& precision,
    double& recall, bool consider_outliers = true) {
  int num_veg = 0;

  for (const auto& pt : ground_estimated.points) {
    if (pt.label == VEGETATION) {
      ++num_veg;
    }
  }

  const int num_ground_est =
      static_cast<int>(ground_estimated.size()) - num_veg;
  const int num_ground_gt = count_num_ground_without_vegetation(pc_curr);
  const int num_TP = count_num_ground_without_vegetation(ground_estimated);

  if (num_ground_gt <= 0 || num_ground_est <= 0) {
    precision = 0.0;
    recall = 0.0;
    return;
  }

  if (consider_outliers) {
    const int num_outliers_est = count_num_outliers(ground_estimated);
    const int denom = num_ground_est - num_outliers_est;

    precision = denom > 0 ? static_cast<double>(num_TP) / denom * 100.0 : 0.0;
    recall = static_cast<double>(num_TP) / num_ground_gt * 100.0;
  } else {
    precision = static_cast<double>(num_TP) / num_ground_est * 100.0;
    recall = static_cast<double>(num_TP) / num_ground_gt * 100.0;
  }
}

inline void save_all_labels(const pcl::PointCloud<PointXYZILID>& pc,
                            const std::string& ABS_DIR, const std::string& seq,
                            int count) {
  const std::string count_str = std::to_string(count);
  const std::string count_str_padded =
      std::string(NUM_ZEROS - count_str.length(), '0') + count_str;

  const std::string output_filename =
      ABS_DIR + "/" + seq + "/" + count_str_padded + ".csv";

  std::ofstream sc_output(output_filename);

  std::vector<int> labels(NUM_ALL_CLASSES, 0);

  for (const auto& pt : pc.points) {
    if (pt.label == 0)
      labels[0]++;
    else if (pt.label == 1)
      labels[1]++;
    else if (pt.label == 10)
      labels[2]++;
    else if (pt.label == 11)
      labels[3]++;
    else if (pt.label == 13)
      labels[4]++;
    else if (pt.label == 15)
      labels[5]++;
    else if (pt.label == 16)
      labels[6]++;
    else if (pt.label == 18)
      labels[7]++;
    else if (pt.label == 20)
      labels[8]++;
    else if (pt.label == 30)
      labels[9]++;
    else if (pt.label == 31)
      labels[10]++;
    else if (pt.label == 32)
      labels[11]++;
    else if (pt.label == 40)
      labels[12]++;
    else if (pt.label == 44)
      labels[13]++;
    else if (pt.label == 48)
      labels[14]++;
    else if (pt.label == 49)
      labels[15]++;
    else if (pt.label == 50)
      labels[16]++;
    else if (pt.label == 51)
      labels[17]++;
    else if (pt.label == 52)
      labels[18]++;
    else if (pt.label == 60)
      labels[19]++;
    else if (pt.label == 70)
      labels[20]++;
    else if (pt.label == 71)
      labels[21]++;
    else if (pt.label == 72)
      labels[22]++;
    else if (pt.label == 80)
      labels[23]++;
    else if (pt.label == 81)
      labels[24]++;
    else if (pt.label == 99)
      labels[25]++;
    else if (pt.label == 252)
      labels[26]++;
    else if (pt.label == 253)
      labels[27]++;
    else if (pt.label == 254)
      labels[28]++;
    else if (pt.label == 255)
      labels[29]++;
    else if (pt.label == 256)
      labels[30]++;
    else if (pt.label == 257)
      labels[31]++;
    else if (pt.label == 258)
      labels[32]++;
    else if (pt.label == 259)
      labels[33]++;
  }

  for (std::size_t i = 0; i < labels.size(); ++i) {
    sc_output << labels[i];

    if (i + 1 < labels.size()) {
      sc_output << ",";
    } else {
      sc_output << std::endl;
    }
  }
}

inline void save_all_accuracy(const pcl::PointCloud<PointXYZILID>& pc_curr,
                              pcl::PointCloud<PointXYZILID>& ground_estimated,
                              const std::string& acc_filename, double& accuracy,
                              std::map<int, int>& pc_curr_gt_counts,
                              std::map<int, int>& g_est_gt_counts) {
  std::ofstream sc_output2(acc_filename, std::ios::app);

  const int num_True = count_num_ground(pc_curr);
  const int num_outliers_gt = count_num_outliers(pc_curr);
  const int num_outliers_est = count_num_outliers(ground_estimated);

  const int num_total_est =
      static_cast<int>(ground_estimated.points.size()) - num_outliers_est;
  const int num_total_gt =
      static_cast<int>(pc_curr.points.size()) - num_outliers_gt;

  if (num_total_gt <= 0) {
    accuracy = 0.0;
  } else {
    const int num_False = num_total_gt - num_True;
    const int num_TP = count_num_ground(ground_estimated);
    const int num_FP = num_total_est - num_TP;

    accuracy = static_cast<double>(num_TP + (num_False - num_FP)) /
               num_total_gt * 100.0;
  }

  pc_curr_gt_counts = count_num_each_class(pc_curr);
  g_est_gt_counts = count_num_each_class(ground_estimated);

  for (const auto class_id : ground_classes) {
    sc_output2 << g_est_gt_counts[class_id] << ","
               << pc_curr_gt_counts[class_id] << ",";
  }

  sc_output2 << accuracy << std::endl;
}

inline void pc2pcdfile(const pcl::PointCloud<PointXYZILID>& TP,
                       const pcl::PointCloud<PointXYZILID>& FP,
                       const pcl::PointCloud<PointXYZILID>& FN,
                       const pcl::PointCloud<PointXYZILID>& TN,
                       const std::string& pcd_filename) {
  pcl::PointCloud<pcl::PointXYZI> pc_out;

  pc_out.points.reserve(TP.points.size() + FP.points.size() + FN.points.size() +
                        TN.points.size());

  for (const auto& pt : TP.points) {
    pcl::PointXYZI pt_est;
    pt_est.x = pt.x;
    pt_est.y = pt.y;
    pt_est.z = pt.z;
    pt_est.intensity = TRUEPOSITIVE;
    pc_out.points.push_back(pt_est);
  }

  for (const auto& pt : FP.points) {
    pcl::PointXYZI pt_est;
    pt_est.x = pt.x;
    pt_est.y = pt.y;
    pt_est.z = pt.z;
    pt_est.intensity = FALSEPOSITIVE;
    pc_out.points.push_back(pt_est);
  }

  for (const auto& pt : FN.points) {
    pcl::PointXYZI pt_est;
    pt_est.x = pt.x;
    pt_est.y = pt.y;
    pt_est.z = pt.z;
    pt_est.intensity = FALSENEGATIVE;
    pc_out.points.push_back(pt_est);
  }

  for (const auto& pt : TN.points) {
    pcl::PointXYZI pt_est;
    pt_est.x = pt.x;
    pt_est.y = pt.y;
    pt_est.z = pt.z;
    pt_est.intensity = TRUENEGATIVE;
    pc_out.points.push_back(pt_est);
  }

  pc_out.width = static_cast<std::uint32_t>(pc_out.points.size());
  pc_out.height = 1;
  pc_out.is_dense = true;

  pcl::io::savePCDFileASCII(pcd_filename, pc_out);
}

#endif  // COMMON_HPP