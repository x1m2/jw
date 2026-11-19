#include <yaml-cpp/yaml.h>

#include <array>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kEpsilon = 1e-9;

struct Pose {
  std::array<double, 3> xyz{};
  std::array<double, 3> rpy{};
};

struct Lidar {
  std::string frame;
  Pose pose;
};

double number(const YAML::Node &node, const std::string &name) {
  if (!node || !node.IsScalar()) {
    throw std::runtime_error("missing or invalid " + name);
  }
  const double value = node.as<double>();
  if (!std::isfinite(value)) throw std::runtime_error(name + " is not finite");
  return value;
}

std::array<double, 3> vector3(const YAML::Node &node, const std::string &name) {
  if (!node || !node.IsSequence() || node.size() != 3) {
    throw std::runtime_error(name + " must contain exactly 3 values");
  }
  return {number(node[0], name + "[0]"), number(node[1], name + "[1]"),
          number(node[2], name + "[2]")};
}

std::array<double, 3> matrixToRpy(const YAML::Node &node) {
  if (!node || !node.IsSequence() || node.size() != 9) {
    throw std::runtime_error("rotation matrix must contain exactly 9 values");
  }
  std::array<double, 9> m{};
  for (std::size_t i = 0; i < m.size(); ++i) m[i] = number(node[i], "rotation matrix");

  // R = Rz(yaw) * Ry(pitch) * Rx(roll), matching robot_geometry.
  const double pitch = std::asin(std::clamp(-m[6], -1.0, 1.0));
  const double roll = std::atan2(m[7], m[8]);
  const double yaw = std::atan2(m[3], m[0]);
  return {roll, pitch, yaw};
}

Pose readPose(const YAML::Node &node, const std::string &name) {
  if (!node || !node.IsMap()) throw std::runtime_error("missing " + name);
  Pose pose;
  pose.xyz = vector3(node["translation"], name + ".translation");
  const YAML::Node rotation = node["rotation"];
  const std::string type = rotation["type"].as<std::string>();
  if (type == "rpy_rad") {
    pose.rpy = vector3(rotation["value"], name + ".rotation.value");
  } else if (type == "matrix") {
    pose.rpy = matrixToRpy(rotation["value"]);
  } else {
    throw std::runtime_error(name + ".rotation.type must be rpy_rad or matrix");
  }
  return pose;
}

std::string xml(const std::string &robot, const std::vector<Lidar> &lidars) {
  std::string out;
  out += "<?xml version=\"1.0\"?>\n<robot name=\"" + robot + "\">\n";
  out += "  <link name=\"base_link\"/>\n";
  for (const auto &lidar : lidars)
    out += "  <link name=\"" + lidar.frame + "\"/>\n";
  for (const auto &lidar : lidars) {
    out += "\n  <joint name=\"base_to_" + lidar.frame + "\" type=\"fixed\">\n";
    out += "    <parent link=\"base_link\"/>\n";
    out += "    <child link=\"" + lidar.frame + "\"/>\n";
    out += "    <origin xyz=\"" + std::to_string(lidar.pose.xyz[0]) + " " +
           std::to_string(lidar.pose.xyz[1]) + " " + std::to_string(lidar.pose.xyz[2]) +
           "\" rpy=\"" + std::to_string(lidar.pose.rpy[0]) + " " +
           std::to_string(lidar.pose.rpy[1]) + " " + std::to_string(lidar.pose.rpy[2]) +
           "\"/>\n  </joint>\n";
  }
  out += "</robot>\n";
  return out;
}

void usage(const char *program) {
  std::cerr << "Usage: " << program << " --yaml robot.yaml --output robot.urdf\n";
}

}  // namespace

int main(int argc, char **argv) {
  try {
    std::filesystem::path yaml_path;
    std::filesystem::path output_path;
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--yaml" && i + 1 < argc) yaml_path = argv[++i];
      else if (arg == "--output" && i + 1 < argc) output_path = argv[++i];
      else {
        usage(argv[0]);
        return 2;
      }
    }
    if (yaml_path.empty() || output_path.empty()) {
      usage(argv[0]);
      return 2;
    }

    const YAML::Node root = YAML::LoadFile(yaml_path.string());
    const YAML::Node parameters = root["/**"]["ros__parameters"];
    if (!parameters) throw std::runtime_error("YAML must contain /**.ros__parameters");
    const std::string robot = parameters["robot_model"].as<std::string>();
    const YAML::Node lidars = parameters["lidars"];
    if (!lidars || !lidars.IsMap()) throw std::runtime_error("missing lidars map");

    std::vector<Lidar> output;
    for (const std::string side : {"front", "rear"}) {
      const YAML::Node config = lidars[side];
      if (!config || !config["enabled"].as<bool>(false)) continue;
      const std::string frame = config["frame"].as<std::string>();
      const std::string pose_key = side == "front" ? "lidar_to_base" : "rear_lidar_to_base";
      output.push_back({frame, readPose(parameters[pose_key], pose_key)});
    }
    if (output.empty()) throw std::runtime_error("no enabled lidar found");

    const std::string content = xml(robot, output);
    const auto temp = output_path.string() + ".tmp";
    std::ofstream stream(temp, std::ios::trunc);
    if (!stream) throw std::runtime_error("cannot open output: " + temp);
    stream << content;
    stream.close();
    if (!stream) throw std::runtime_error("cannot write output: " + temp);
    std::filesystem::rename(temp, output_path);
    std::cout << "Generated " << output_path << " from " << yaml_path << "\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "urdf_generator_node: " << error.what() << "\n";
    return 1;
  }
}
