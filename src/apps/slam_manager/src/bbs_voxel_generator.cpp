#include "slam_manager/bbs_voxel_generator.hpp"

#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstring>
#include <stdexcept>
#include <thread>
#include <utility>

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

namespace slam_manager {

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
    std::string &message) const {
  // load_map的前置步骤：定位Worker启动前重新生成3D-BBS层级体素地图。
  std::error_code filesystem_error;
  if (!std::filesystem::exists(config_.executable, filesystem_error) ||
      filesystem_error) {
    message = filesystem_error
                  ? "failed to inspect saver executable: " +
                        filesystem_error.message()
                  : "saver executable does not exist: " +
                        config_.executable.string();
    return false;
  }
  if (!std::filesystem::is_regular_file(config_.executable,
                                        filesystem_error) ||
      filesystem_error || ::access(config_.executable.c_str(), X_OK) != 0) {
    message = "saver is not an executable regular file: " +
              config_.executable.string();
    return false;
  }
  if (!std::filesystem::is_directory(map_directory, filesystem_error) ||
      filesystem_error) {
    message = filesystem_error
                  ? "failed to inspect map directory: " +
                        filesystem_error.message()
                  : "map directory does not exist: " + map_directory.string();
    return false;
  }

  // 无条件移除上一次生成结果，保证体素一定来自当前PCD。
  const auto voxel_directory = map_directory / "voxelmaps_coords";
  std::filesystem::remove_all(voxel_directory, filesystem_error);
  if (filesystem_error) {
    message = "failed to remove previous 3D-BBS voxel map: " +
              filesystem_error.message();
    return false;
  }

  const std::string map_path = map_directory.string();
  const std::string resolution =
      std::to_string(config_.min_level_resolution);
  const std::string max_level = std::to_string(config_.max_level);
  const pid_t child_pid = ::fork();
  if (child_pid < 0) {
    message = "failed to start 3D-BBS saver: " +
              std::string(std::strerror(errno));
    return false;
  }
  if (child_pid == 0) {
    ::execl(config_.executable.c_str(), config_.executable.c_str(),
            map_path.c_str(), resolution.c_str(), max_level.c_str(),
            static_cast<char *>(nullptr));
    ::_exit(127);
  }

  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration<double>(config_.timeout_sec);
  int process_status = 0;
  while (std::chrono::steady_clock::now() < deadline) {
    const pid_t wait_result = ::waitpid(child_pid, &process_status, WNOHANG);
    if (wait_result == child_pid) {
      if (WIFEXITED(process_status) && WEXITSTATUS(process_status) == 0) {
        // 子进程返回0后仍需逐项检查输出，避免把不完整体素地图交给定位。
        const auto parameters_file = voxel_directory / "voxel_params.txt";
        if (!std::filesystem::is_regular_file(parameters_file,
                                               filesystem_error) ||
            filesystem_error) {
          message = "3D-BBS saver exited successfully but voxel_params.txt "
                    "was not generated";
          return false;
        }
        for (int64_t level = 0; level <= config_.max_level; ++level) {
          const auto level_file =
              voxel_directory / (std::to_string(level) + ".pcd");
          if (!std::filesystem::is_regular_file(level_file,
                                                filesystem_error) ||
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

  ::kill(child_pid, SIGTERM);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  if (::waitpid(child_pid, &process_status, WNOHANG) == 0) {
    ::kill(child_pid, SIGKILL);
  }
  ::waitpid(child_pid, &process_status, 0);
  message = "3D-BBS saver timed out after " +
            std::to_string(config_.timeout_sec) + " seconds";
  return false;
}

}  // namespace slam_manager
