#pragma once

#include <cstdint>
#include <filesystem>
#include <string>

namespace slam_manager {

// 负责3D-BBS saver子进程的启动、超时终止和输出文件校验。
class BbsVoxelGenerator {
 public:
  struct Config {
    std::filesystem::path executable;
    double min_level_resolution{0.5};
    int64_t max_level{6};
    double timeout_sec{120.0};
  };

  explicit BbsVoxelGenerator(Config config);

  bool generate(const std::filesystem::path &map_directory,
                const std::string &map_id, std::string &message) const;

 private:
  Config config_;
};

}  // namespace slam_manager
