#include "slam_manager/map_check.hpp"

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <thread>
#include <utility>

namespace slam_manager {

MapStorage::MapStorage(std::filesystem::path root_directory,
                       std::size_t max_maps)
    : root_directory_(std::move(root_directory)), max_maps_(max_maps) {
  if (root_directory_.empty() || !root_directory_.is_absolute()) {
    throw std::invalid_argument(
        "map storage root directory must be an absolute path");
  }
  if (max_maps_ == 0U) {
    throw std::invalid_argument(
        "map storage max_maps must be greater than zero");
  }
}

const std::filesystem::path &MapStorage::rootDirectory() const {
  return root_directory_;
}

std::size_t MapStorage::maxMaps() const { return max_maps_; }

std::filesystem::path MapStorage::dataPath(const std::string &map_id) const {
  return root_directory_ / map_id;
}

std::filesystem::path MapStorage::mapPath(const std::string &map_id) const {
  return dataPath(map_id) / "map";
}

std::filesystem::path MapStorage::taskPath(const std::string &map_id) const {
  return dataPath(map_id) / "task";
}

bool MapStorage::countSavedMaps(std::size_t &count,
                                std::string &message) const {
  // 只有包含globalmap.pcd的非隐藏目录才计为一张完整地图。
  count = 0U;
  std::error_code filesystem_error;
  if (!std::filesystem::exists(root_directory_, filesystem_error)) {
    if (filesystem_error) {
      message = "failed to inspect map root: " + filesystem_error.message();
      return false;
    }
    return true;
  }

  std::filesystem::directory_iterator iterator(root_directory_,
                                               filesystem_error);
  const std::filesystem::directory_iterator end;
  if (filesystem_error) {
    message = "failed to list map root: " + filesystem_error.message();
    return false;
  }
  while (iterator != end) {
    const auto &entry = *iterator;
    const std::string name = entry.path().filename().string();
    if (!name.empty() && name.front() != '.' &&
        entry.is_directory(filesystem_error) &&
        std::filesystem::exists(entry.path() / "map" / "globalmap.pcd",
                                filesystem_error)) {
      ++count;
    }
    if (filesystem_error) {
      message = "failed to inspect map entry: " + filesystem_error.message();
      return false;
    }
    iterator.increment(filesystem_error);
    if (filesystem_error) {
      message = "failed to iterate map root: " + filesystem_error.message();
      return false;
    }
  }
  return true;
}

bool MapStorage::prepareForMapping(const std::string &map_id,
                                   std::string &message) const {
  const auto target = dataPath(map_id);
  std::error_code filesystem_error;
  const bool replacing = std::filesystem::exists(target, filesystem_error);
  if (filesystem_error) {
    message = "failed to inspect map directory: " + filesystem_error.message();
    return false;
  }

  if (replacing) {
    std::filesystem::remove_all(target, filesystem_error);
    if (filesystem_error) {
      message = "failed to clear existing map directory: " +
                filesystem_error.message();
      return false;
    }
  }

  std::filesystem::create_directories(mapPath(map_id), filesystem_error);
  if (filesystem_error) {
    message = "failed to create map directory: " + filesystem_error.message();
    return false;
  }
  std::filesystem::create_directories(taskPath(map_id), filesystem_error);
  if (filesystem_error) {
    message = "failed to create task directory: " + filesystem_error.message();
    return false;
  }

  message = std::string(replacing ? "map directory replaced: "
                                  : "map directory created: ") +
            target.string();
  return true;
}

bool MapStorage::deleteMap(const std::string &map_id,
                           std::string &message) const {
  // 删除map_id数据目录；运行日志位于root/log下，不由MapStorage管理。
  const auto target = dataPath(map_id);
  std::error_code filesystem_error;
  const bool exists = std::filesystem::exists(target, filesystem_error);
  if (filesystem_error) {
    message = "failed to inspect map directory: " + filesystem_error.message();
    return false;
  }
  if (!exists) {
    message = "map does not exist: " + map_id;
    return false;
  }
  if (!std::filesystem::is_directory(target, filesystem_error) ||
      filesystem_error) {
    message = "map target is not a valid directory: " + target.string();
    return false;
  }

  const auto removed_count =
      std::filesystem::remove_all(target, filesystem_error);
  if (filesystem_error) {
    message = "failed to delete map directory: " + filesystem_error.message();
    return false;
  }
  message = "map deleted: " + map_id +
            ", removed entries=" + std::to_string(removed_count);
  return true;
}

bool MapStorage::finalize(const std::filesystem::path &map_path,
                          const std::string &map_id,
                          std::string &message) const {
  // 目录由start_mapping预先创建，此处检查并写入保存结果。
  if (map_id.empty() || map_path.empty()) {
    message = "no active map task";
    return false;
  }

  const auto expected_map_path = mapPath(map_id);
  if (map_path != expected_map_path) {
    message = "unexpected map save path: " + map_path.string();
    return false;
  }

  std::error_code filesystem_error;
  if (!std::filesystem::exists(dataPath(map_id), filesystem_error) ||
      filesystem_error) {
    message = filesystem_error ? "failed to inspect map data directory: " +
                                     filesystem_error.message()
                               : "map data directory does not exist: " +
                                     dataPath(map_id).string();
    return false;
  }
  if (!std::filesystem::is_directory(map_path, filesystem_error) ||
      filesystem_error) {
    message =
        filesystem_error
            ? "failed to inspect map directory: " + filesystem_error.message()
            : "map directory does not exist: " + map_path.string();
    return false;
  }
  const auto task_path = taskPath(map_id);
  if (!std::filesystem::is_directory(task_path, filesystem_error) ||
      filesystem_error) {
    message =
        filesystem_error
            ? "failed to inspect task directory: " + filesystem_error.message()
            : "task directory does not exist: " + task_path.string();
    return false;
  }

  // globalmap.pcd是地图保存成功的最低完整性条件。
  const auto global_map = map_path / "globalmap.pcd";
  if (!std::filesystem::exists(global_map, filesystem_error) ||
      filesystem_error) {
    message = filesystem_error
                  ? "failed to inspect saved map: " + filesystem_error.message()
                  : "PGO did not produce globalmap.pcd";
    return false;
  }
  if (!std::filesystem::is_regular_file(global_map, filesystem_error) ||
      filesystem_error) {
    message = filesystem_error ? "failed to inspect globalmap.pcd: " +
                                     filesystem_error.message()
                               : "globalmap.pcd is not a regular file";
    return false;
  }

  // manifest属于地图保存结果文件，可以写入，但不创建任何目录。
  std::ofstream manifest(map_path / "manifest.yaml");
  if (!manifest.is_open()) {
    message = "failed to create manifest.yaml in existing map directory";
    return false;
  }
  manifest << "schema_version: 1\n"
           << "map_id: \"" << map_id << "\"\n"
           << "frame_id: \"map\"\n"
           << "status: \"ready\"\n"
           << "global_map: \"globalmap.pcd\"\n";
  manifest.close();
  if (!manifest) {
    message = "failed to write manifest.yaml";
    return false;
  }

  message = "map saved: map_id=" + map_id + ", map_path=" + global_map.string();
  return true;
}

BbsVoxelGenerator::BbsVoxelGenerator(Config config)
    : config_(std::move(config)) {
  if (config_.executable.empty() || !config_.executable.is_absolute() ||
      config_.min_level_resolution <= 0.0 || config_.max_level < 0 ||
      config_.timeout_sec <= 0.0) {
    throw std::invalid_argument(
        "3D-BBS config requires an absolute executable, positive "
        "resolution/timeout and non-negative max_level");
  }
}

bool BbsVoxelGenerator::generate(
    const std::filesystem::path &map_directory, const std::string &map_id,
    std::string &message, const std::function<bool()> &is_canceling) const {
  // load_map的前置步骤：定位Worker启动前重新生成3D-BBS层级体素地图。
  std::error_code filesystem_error;
  if (!std::filesystem::exists(config_.executable, filesystem_error) ||
      filesystem_error) {
    message = filesystem_error ? "failed to inspect saver executable: " +
                                     filesystem_error.message()
                               : "saver executable does not exist: " +
                                     config_.executable.string();
    return false;
  }
  if (!std::filesystem::is_regular_file(config_.executable, filesystem_error) ||
      filesystem_error || ::access(config_.executable.c_str(), X_OK) != 0) {
    message = "saver is not an executable regular file: " +
              config_.executable.string();
    return false;
  }
  if (!std::filesystem::is_directory(map_directory, filesystem_error) ||
      filesystem_error) {
    message =
        filesystem_error
            ? "failed to inspect map directory: " + filesystem_error.message()
            : "map directory does not exist: " + map_directory.string();
    return false;
  }

  // 无条件移除上一次生成结果，保证体素一定来自当前PCD。
  const auto voxel_directory = map_directory / "voxelmaps_coords";
  std::filesystem::remove_all(voxel_directory, filesystem_error);
  if (filesystem_error.default_error_condition() ==
      std::errc::no_such_file_or_directory) {
    filesystem_error.clear();
  }
  if (filesystem_error) {
    message = "failed to remove previous 3D-BBS voxel map: " +
              filesystem_error.message();
    return false;
  }

  const std::string map_path = map_directory.string();
  const std::string resolution = std::to_string(config_.min_level_resolution);
  const std::string max_level = std::to_string(config_.max_level);
  const pid_t child_pid = ::fork();
  if (child_pid < 0) {
    message =
        "failed to start 3D-BBS saver: " + std::string(std::strerror(errno));
    return false;
  }
  if (child_pid == 0) {
    ::execl(config_.executable.c_str(), config_.executable.c_str(),
            map_path.c_str(), resolution.c_str(), max_level.c_str(),
            static_cast<char *>(nullptr));
    ::_exit(127);
  }

  int process_status = 0;
  const auto terminate_child = [&]() {
    ::kill(child_pid, SIGTERM);
    const auto terminate_deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
    while (std::chrono::steady_clock::now() < terminate_deadline) {
      const pid_t wait_result = ::waitpid(child_pid, &process_status, WNOHANG);
      if (wait_result == child_pid) return;
      if (wait_result < 0 && errno != EINTR) return;
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    ::kill(child_pid, SIGKILL);
    while (::waitpid(child_pid, &process_status, 0) < 0 && errno == EINTR) {
    }
  };

  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration<double>(config_.timeout_sec);
  while (std::chrono::steady_clock::now() < deadline) {
    if (is_canceling && is_canceling()) {
      terminate_child();
      message = "3D-BBS generation cancelled by a newer lifecycle command";
      return false;
    }
    const pid_t wait_result = ::waitpid(child_pid, &process_status, WNOHANG);
    if (wait_result == child_pid) {
      if (WIFEXITED(process_status) && WEXITSTATUS(process_status) == 0) {
        // 子进程返回0后仍需逐项检查输出，避免把不完整体素地图交给定位。
        const auto parameters_file = voxel_directory / "voxel_params.txt";
        if (!std::filesystem::is_regular_file(parameters_file,
                                              filesystem_error) ||
            filesystem_error) {
          message =
              "3D-BBS saver exited successfully but voxel_params.txt "
              "was not generated";
          return false;
        }
        for (int64_t level = 0; level <= config_.max_level; ++level) {
          const auto level_file =
              voxel_directory / (std::to_string(level) + ".pcd");
          if (!std::filesystem::is_regular_file(level_file, filesystem_error) ||
              filesystem_error) {
            message = "3D-BBS saver did not generate voxel level " +
                      std::to_string(level);
            return false;
          }
        }
        message = "3D-BBS voxel map generated, map_id=" + map_id;
        return true;
      }
      if (WIFEXITED(process_status)) {
        message = "3D-BBS saver exited with code " +
                  std::to_string(WEXITSTATUS(process_status));
      } else if (WIFSIGNALED(process_status)) {
        message = "3D-BBS saver terminated by signal " +
                  std::to_string(WTERMSIG(process_status));
      } else {
        message = "3D-BBS saver ended unexpectedly";
      }
      return false;
    }
    if (wait_result < 0) {
      if (errno == EINTR) continue;
      const int wait_error = errno;
      ::kill(child_pid, SIGKILL);
      ::waitpid(child_pid, &process_status, 0);
      message = "failed to wait for 3D-BBS saver: " +
                std::string(std::strerror(wait_error));
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  terminate_child();
  message = "3D-BBS saver timed out after " +
            std::to_string(config_.timeout_sec) + " seconds";
  return false;
}

}  // namespace slam_manager
