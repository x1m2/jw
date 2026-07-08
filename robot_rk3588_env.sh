#!/usr/bin/env bash
set -eo pipefail

# ============================================================
# setup_robot_rk3588_env.sh
#
# 由原 Dockerfile 改成的宿主机/容器内一键环境部署脚本。
#
# 目录约定：
#   当前脚本所在目录/
#   └── robot_rk3588_ws/
#       ├── src/
#       ├── gtsam-4.1.1.zip
#       ├── Livox-SDK2.zip           # 或 Livox-SDK2/
#       ├── TEASER-plusplus.zip      # 或 TEASER-plusplus/
#       ├── pmc-master.zip           # 或 pmc/
#       ├── tinyply*.zip             # 或 tinyply/
#       └── spectra*.zip             # 或 spectra/
#
# 第三方源码默认从工作空间根目录读取，也就是 src 的同级目录。
#
# 用法：
#   chmod +x setup_robot_rk3588_env.sh
#   ./setup_robot_rk3588_env.sh
#
# 如果希望脚本结束后当前终端也 source 工作空间：
#   source ./setup_robot_rk3588_env.sh
#
# 可选环境变量：
#   WS=/path/to/robot_rk3588_ws ./setup_robot_rk3588_env.sh
#   WS_ZIP=/path/to/robot_rk3588_ws.zip ./setup_robot_rk3588_env.sh
#   THIRD_PARTY_DIR=/path/to/third_party ./setup_robot_rk3588_env.sh
#   PARALLEL_WORKERS=2 ./setup_robot_rk3588_env.sh
#   FORCE_THIRD_PARTY=1 ./setup_robot_rk3588_env.sh   # 强制重装 GTSAM/Livox/TEASER++
# ============================================================

SCRIPT_SOURCED=0
if [[ "${BASH_SOURCE[0]}" != "$0" ]]; then
  SCRIPT_SOURCED=1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${WS:-${SCRIPT_DIR}/robot_rk3588_ws}"
WS="${WS%/}"
# 工作空间可直接是目录；目录不存在时，自动尝试解压同名 zip。
# 默认：${SCRIPT_DIR}/robot_rk3588_ws.zip
# 可覆盖：WS_ZIP=/path/to/robot_rk3588_ws.zip ./setup_robot_rk3588_env.sh
WS_ZIP="${WS_ZIP:-${WS}.zip}"
SRC_DIR="${WS}/src"
# 默认第三方源码
THIRD_PARTY_DIR="${THIRD_PARTY_DIR:-${WS}/env}"
PARALLEL_WORKERS="${PARALLEL_WORKERS:-2}"
FORCE_THIRD_PARTY="${FORCE_THIRD_PARTY:-0}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
TZ_VALUE="${TZ_VALUE:-Asia/Shanghai}"
# 第三方库临时构建目录：默认使用启动脚本时的当前目录。
BUILD_TMP_ROOT="${BUILD_TMP_ROOT:-${PWD}/.build_tmp}"

info() { echo -e "\033[1;32m[INFO]\033[0m $*"; }
warn() { echo -e "\033[1;33m[WARN]\033[0m $*"; }
err()  { echo -e "\033[1;31m[ERROR]\033[0m $*" >&2; }

need_cmd() {
  command -v "$1" >/dev/null 2>&1 || {
    err "缺少命令：$1"
    exit 1
  }
}

prepare_workspace_from_zip() {
  # 目录已经存在时不重复解压，也不会覆盖现有工作空间。
  if [[ -d "${WS}" ]]; then
    return 0
  fi

  # 没有工作空间目录且没有 zip，交给 check_workspace() 输出统一错误。
  if [[ ! -f "${WS_ZIP}" ]]; then
    return 0
  fi

  need_cmd unzip

  local ws_parent ws_name tmp_dir
  ws_parent="$(dirname "${WS}")"
  ws_name="$(basename "${WS}")"

  mkdir -p "${ws_parent}"
  tmp_dir="$(mktemp -d "${ws_parent}/.${ws_name}.extract.XXXXXX")"

  info "检测到工作空间压缩包：${WS_ZIP}"
  info "临时解压目录：${tmp_dir}"

  if ! unzip -q "${WS_ZIP}" -d "${tmp_dir}"; then
    rm -rf "${tmp_dir}"
    err "工作空间压缩包解压失败：${WS_ZIP}"
    exit 1
  fi

  # 支持以下三种常见压缩包结构：
  # 1) robot_rk3588_ws/src/...           -> 直接取同名顶层目录
  # 2) 其他目录名/src/...                 -> 将那个唯一顶层目录重命名为 WS
  # 3) src/...、env/... 直接在 zip 根目录 -> 创建 WS 后移动所有根内容进去
  if [[ -d "${tmp_dir}/${ws_name}/src" ]]; then
    mv "${tmp_dir}/${ws_name}" "${WS}"

  elif [[ -d "${tmp_dir}/src" ]]; then
    mkdir -p "${WS}"
    # find 能保留隐藏文件；比 "${tmp_dir}"/* 更可靠。
    find "${tmp_dir}" -mindepth 1 -maxdepth 1 -exec mv -t "${WS}" {} +

  else
    local -a top_dirs=()
    mapfile -t top_dirs < <(
      find "${tmp_dir}" -mindepth 1 -maxdepth 1 -type d \
        ! -name "__MACOSX" -print
    )

    if [[ "${#top_dirs[@]}" -eq 1 && -d "${top_dirs[0]}/src" ]]; then
      mv "${top_dirs[0]}" "${WS}"
    else
      rm -rf "${tmp_dir}"
      err "无法识别工作空间 zip 结构：${WS_ZIP}"
      err "压缩包内应满足以下任一种结构："
      err "  robot_rk3588_ws/src/..."
      err "  任意单一顶层目录/src/..."
      err "  src/... 直接位于压缩包根目录"
      exit 1
    fi
  fi

  rm -rf "${tmp_dir}"

  if [[ ! -d "${WS}/src" ]]; then
    err "解压后仍未找到 ${WS}/src，压缩包结构不正确"
    exit 1
  fi

  info "工作空间解压完成：${WS}"
}

check_workspace() {
  info "检查工作空间：${WS}"

  if [[ ! -d "${WS}" ]]; then
    err "工作空间不存在：${WS}"
    err "请确认脚本同级目录下存在 robot_rk3588_ws/"
    return 1
  fi

  if [[ ! -d "${SRC_DIR}" ]]; then
    err "src 目录不存在：${SRC_DIR}"
    err "请确认已经创建：robot_rk3588_ws/src"
    return 1
  fi

  if [[ -z "$(find "${SRC_DIR}" -mindepth 1 -maxdepth 1 -print -quit 2>/dev/null)" ]]; then
    err "src 目录为空：${SRC_DIR}"
    err "请先把 ROS2 包放到 robot_rk3588_ws/src 下"
    return 1
  fi

  info "工作空间检查通过"
}

setup_locale_and_timezone() {
  info "配置中文 UTF-8 和时区"
  sudo ln -snf "/usr/share/zoneinfo/${TZ_VALUE}" /etc/localtime
  echo "${TZ_VALUE}" | sudo tee /etc/timezone >/dev/null

  if ! locale -a | grep -qi '^zh_CN.utf8$'; then
    sudo locale-gen zh_CN.UTF-8 || true
  fi

  sudo update-locale LANG=zh_CN.UTF-8 || true
}

ensure_ros_repo_if_needed() {
  if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    info "检测到 ROS ${ROS_DISTRO} 已安装"
    return 0
  fi

  warn "未检测到 /opt/ros/${ROS_DISTRO}/setup.bash，尝试添加 ROS2 apt 源并安装 ros-${ROS_DISTRO}-ros-base"

  sudo apt-get update
  sudo apt-get install -y --no-install-recommends \
    software-properties-common \
    curl \
    gnupg2 \
    lsb-release \
    ca-certificates

  sudo add-apt-repository universe -y || true

  sudo mkdir -p /etc/apt/keyrings
  if [[ ! -f /etc/apt/keyrings/ros-archive-keyring.gpg ]]; then
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      | sudo gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg
  fi

  echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "${UBUNTU_CODENAME}") main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

  sudo apt-get update
  sudo apt-get install -y --no-install-recommends "ros-${ROS_DISTRO}-ros-base"

  if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    err "ROS ${ROS_DISTRO} 安装后仍未找到 setup.bash"
    exit 1
  fi
}

install_apt_dependencies() {
  info "安装 apt 基础依赖、ROS2 依赖、PCL/OpenCV/Eigen/Nav2 等"

  sudo apt-get update

  sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    vim \
    xterm \
    xfce4-terminal \
    nano \
    unzip \
    lsb-release \
    gnupg2 \
    ca-certificates \
    locales \
    tzdata \
    fontconfig \
    fonts-noto-cjk \
    fonts-wqy-zenhei \
    fonts-wqy-microhei \
    python3-pip \
    python3-tk \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    net-tools \
    iputils-ping \
    iproute2 \
    netcat-openbsd \
    dnsutils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    mesa-utils \
    x11-apps \
    libpcl-dev \
    libeigen3-dev \
    libopencv-dev \
    libboost-all-dev \
    libtbb-dev \
    libyaml-cpp-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libomp-dev

  sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    "ros-${ROS_DISTRO}-rclcpp" \
    "ros-${ROS_DISTRO}-rclcpp-components" \
    "ros-${ROS_DISTRO}-std-msgs" \
    "ros-${ROS_DISTRO}-sensor-msgs" \
    "ros-${ROS_DISTRO}-nav-msgs" \
    "ros-${ROS_DISTRO}-geometry-msgs" \
    "ros-${ROS_DISTRO}-visualization-msgs" \
    "ros-${ROS_DISTRO}-std-srvs" \
    "ros-${ROS_DISTRO}-tf2" \
    "ros-${ROS_DISTRO}-tf2-ros" \
    "ros-${ROS_DISTRO}-tf2-eigen" \
    "ros-${ROS_DISTRO}-tf2-geometry-msgs" \
    "ros-${ROS_DISTRO}-tf2-sensor-msgs" \
    "ros-${ROS_DISTRO}-angles" \
    "ros-${ROS_DISTRO}-ament-cmake-auto" \
    "ros-${ROS_DISTRO}-laser-geometry" \
    "ros-${ROS_DISTRO}-message-filters" \
    "ros-${ROS_DISTRO}-ament-cmake" \
    "ros-${ROS_DISTRO}-ament-index-cpp" \
    "ros-${ROS_DISTRO}-rosbag2-cpp" \
    "ros-${ROS_DISTRO}-rosbag2-storage" \
    "ros-${ROS_DISTRO}-rosbag2-storage-default-plugins" \
    "ros-${ROS_DISTRO}-rosidl-default-generators" \
    "ros-${ROS_DISTRO}-rosidl-default-runtime" \
    "ros-${ROS_DISTRO}-pcl-ros" \
    "ros-${ROS_DISTRO}-pcl-conversions" \
    "ros-${ROS_DISTRO}-pcl-msgs" \
    "ros-${ROS_DISTRO}-sophus" \
    "ros-${ROS_DISTRO}-rviz2" \
    "ros-${ROS_DISTRO}-teleop-twist-keyboard" \
    "ros-${ROS_DISTRO}-teleop-twist-joy" \
    "ros-${ROS_DISTRO}-key-teleop" \
    "ros-${ROS_DISTRO}-rclpy" \
    "ros-${ROS_DISTRO}-ament-index-python" \
    "ros-${ROS_DISTRO}-navigation2" \
    "ros-${ROS_DISTRO}-nav2-bringup" \
    "ros-${ROS_DISTRO}-nav2-map-server" \
    "ros-${ROS_DISTRO}-nav2-costmap-2d" \
    "ros-${ROS_DISTRO}-nav2-controller" \
    "ros-${ROS_DISTRO}-nav2-planner" \
    "ros-${ROS_DISTRO}-nav2-bt-navigator" \
    "ros-${ROS_DISTRO}-nav2-util" \
    "ros-${ROS_DISTRO}-nav2-lifecycle-manager" \
    "ros-${ROS_DISTRO}-dwb-core" \
    "ros-${ROS_DISTRO}-dwb-critics" \
    "ros-${ROS_DISTRO}-dwb-plugins"

  sudo fc-cache -fv >/dev/null || true
}

init_rosdep() {
  info "初始化/更新 rosdep"
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init || true
  fi
  rosdep update || true
}

has_library() {
  local pattern="$1"
  ldconfig -p 2>/dev/null | grep -qi "$pattern"
}

# 查找源码目录或 zip。默认从 ${THIRD_PARTY_DIR} 查找，即 robot_rk3588_ws 根目录。
# 用法：prepare_source_dir 逻辑名 解压目标目录 候选zip...
# 输出：源码目录路径。
prepare_source_dir() {
  local logical_name="$1"
  local extract_parent="$2"
  shift 2

  if [[ -d "${THIRD_PARTY_DIR}/${logical_name}" ]]; then
    echo "${THIRD_PARTY_DIR}/${logical_name}"
    return 0
  fi

  local zip_path=""
  for z in "$@"; do
    if [[ -f "${THIRD_PARTY_DIR}/${z}" ]]; then
      zip_path="${THIRD_PARTY_DIR}/${z}"
      break
    fi
  done

  if [[ -z "${zip_path}" ]]; then
    return 1
  fi

  mkdir -p "${extract_parent}"
  unzip -q "${zip_path}" -d "${extract_parent}"

  # 优先找同名目录；否则取 zip 解压出来的第一个一级目录。
  if [[ -d "${extract_parent}/${logical_name}" ]]; then
    echo "${extract_parent}/${logical_name}"
    return 0
  fi

  local first_dir
  first_dir="$(find "${extract_parent}" -mindepth 1 -maxdepth 1 -type d | head -n 1 || true)"
  if [[ -n "${first_dir}" ]]; then
    echo "${first_dir}"
    return 0
  fi

  return 1
}

build_gtsam_411() {
  if [[ "${FORCE_THIRD_PARTY}" != "1" ]] && has_library "libgtsam"; then
    info "检测到 GTSAM 已安装，跳过源码安装。若要强制重装：FORCE_THIRD_PARTY=1 ./robot_rk3588_env.sh"
    return 0
  fi

  local zip_path="${THIRD_PARTY_DIR}/gtsam-4.1.1.zip"
  if [[ ! -f "${zip_path}" ]]; then
    err "未检测到 GTSAM，且缺少源码包：${zip_path}"
    err "请把 gtsam-4.1.1.zip 放到工作空间根目录，也就是 src 同级目录"
    exit 1
  fi

  info "源码安装 GTSAM 4.1.1：${zip_path}"
  local build_root="${BUILD_TMP_ROOT}/gtsam_build_$$"
  rm -rf "${build_root}"
  mkdir -p "${build_root}"
  unzip -q "${zip_path}" -d "${build_root}"

  cd "${build_root}/gtsam-4.1.1"
  rm -rf build
  mkdir build
  cd build

  cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
    -DGTSAM_USE_SYSTEM_EIGEN=ON \
    -DGTSAM_BUILD_TESTS=OFF \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_BUILD_UNSTABLE=OFF

  make -j"${PARALLEL_WORKERS}"
  sudo make install
  sudo ldconfig

  rm -rf "${build_root}"
}

build_livox_sdk2() {
  if [[ "${FORCE_THIRD_PARTY}" != "1" ]] && has_library "livox"; then
    info "检测到 Livox SDK 相关库已安装，跳过 Livox-SDK2。若要强制重装：FORCE_THIRD_PARTY=1 ./robot_rk3588_env.sh"
    return 0
  fi

  info "源码安装 Livox-SDK2"
  local build_root="${BUILD_TMP_ROOT}/livox_sdk2_build_$$"
  rm -rf "${build_root}"
  mkdir -p "${build_root}"

  local sdk_dir=""
  if ! sdk_dir="$(prepare_source_dir "Livox-SDK2" "${build_root}" "Livox-SDK2.zip" "livox-sdk2.zip" "livox_sdk2.zip")"; then
    err "未检测到 Livox-SDK2 源码"
    err "请把 Livox-SDK2.zip 或 Livox-SDK2/ 放到：${THIRD_PARTY_DIR}"
    exit 1
  fi

  cd "${sdk_dir}"
  rm -rf build
  mkdir build
  cd build

  cmake .. -DCMAKE_BUILD_TYPE=Release
  make -j"${PARALLEL_WORKERS}"
  sudo make install
  sudo ldconfig

  rm -rf "${build_root}"
}

build_teaserpp() {
  if [[ "${FORCE_THIRD_PARTY}" != "1" ]] && has_library "teaser"; then
    info "检测到 TEASER++ 相关库已安装，跳过源码安装。若要强制重装：FORCE_THIRD_PARTY=1 ./robot_rk3588_env.sh"
    return 0
  fi

  info "源码安装 TEASER++"
  local build_root="${BUILD_TMP_ROOT}/teaserpp_build_$$"
  rm -rf "${build_root}"
  mkdir -p "${build_root}"

  local teaser_dir=""
  local pmc_dir=""
  local tinyply_dir=""
  local spectra_dir=""

  if ! teaser_dir="$(prepare_source_dir "TEASER-plusplus" "${build_root}/teaser" "TEASER-plusplus.zip" "teaser-plusplus.zip" "TEASER-plusplus-master.zip")"; then
    err "缺少 TEASER++ 源码：请把 TEASER-plusplus.zip 或 TEASER-plusplus/ 放到 ${THIRD_PARTY_DIR}"
    exit 1
  fi

  if ! pmc_dir="$(prepare_source_dir "pmc" "${build_root}/pmc" "pmc-master.zip" "pmc.zip")"; then
    err "缺少 pmc 源码：请把 pmc-master.zip 或 pmc/ 放到 ${THIRD_PARTY_DIR}"
    exit 1
  fi

  if ! tinyply_dir="$(prepare_source_dir "tinyply" "${build_root}/tinyply" "tinyply.zip" "tinyply-master.zip")"; then
    err "缺少 tinyply 源码：请把 tinyply.zip / tinyply-master.zip 或 tinyply/ 放到 ${THIRD_PARTY_DIR}"
    exit 1
  fi

  if ! spectra_dir="$(prepare_source_dir "spectra" "${build_root}/spectra" "spectra.zip" "spectra-master.zip")"; then
    err "缺少 spectra 源码：请把 spectra.zip / spectra-master.zip 或 spectra/ 放到 ${THIRD_PARTY_DIR}"
    exit 1
  fi

  info "复制 TEASER++ 及其依赖到 /opt，保持和 Dockerfile 一致的目录结构"
  sudo rm -rf /opt/pmc /opt/tinyply /opt/spectra /opt/TEASER-plusplus
  sudo cp -a "${pmc_dir}" /opt/pmc
  sudo cp -a "${tinyply_dir}" /opt/tinyply
  sudo cp -a "${spectra_dir}" /opt/spectra
  sudo cp -a "${teaser_dir}" /opt/TEASER-plusplus
  sudo chown -R root:root /opt/pmc /opt/tinyply /opt/spectra /opt/TEASER-plusplus

  cd /opt/TEASER-plusplus
  sudo rm -rf build
  sudo mkdir build
  cd build

  sudo cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DENABLE_DIAGNOSTIC_PRINT=OFF

  sudo make -j"${PARALLEL_WORKERS}"
  sudo make install
  sudo ldconfig

  rm -rf "${build_root}"
}

rosdep_install_workspace_deps() {
  info "根据工作空间 src 执行 rosdep install"
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  cd "${WS}"
  rosdep install --from-paths "${SRC_DIR}" --ignore-src -r -y --rosdistro "${ROS_DISTRO}" || {
    warn "rosdep install 有失败项，可能是本地包或手动安装依赖；继续尝试编译"
  }
}

colcon_package_exists() {
  local pkg="$1"
  cd "${WS}"
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  colcon list --names-only 2>/dev/null | grep -qx "${pkg}"
}

build_workspace() {
  info "检查工作空间包列表"
  cd "${WS}"
  source "/opt/ros/${ROS_DISTRO}/setup.bash"

  local pkg_count
  pkg_count="$(colcon list --names-only 2>/dev/null | wc -l || true)"
  if [[ "${pkg_count}" -le 0 ]]; then
    err "colcon 没有在 ${SRC_DIR} 下找到任何 ROS2 包"
    exit 1
  fi
  info "检测到 ${pkg_count} 个 ROS2 包"

  # quattro 需要额外的 QUATRO_TBB=ON 配置，因此先单独处理。
  if colcon_package_exists "quatro"; then
    info "再单独编译 quatro，开启 TBB"
    colcon build \
      --packages-select quatro \
      --cmake-args \
      -DCMAKE_BUILD_TYPE=Release \
      -DQUATRO_TBB=ON
  else
    warn "未找到 quatro 包，跳过单独编译"
  fi

  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  if [[ -f "${WS}/install/setup.bash" ]]; then
    source "${WS}/install/setup.bash"
  fi

  info "第一阶段：编译工作空间，跳过 quatro/fast_lio/pgo_sc/localization_sc"
  colcon build --symlink-install \
    --parallel-workers "${PARALLEL_WORKERS}" \
    --packages-skip quatro fast_lio pgo_sc localization_sc \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DROS_EDITION=ROS2 \
    -DHUMBLE_ROS=humble

  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  if [[ -f "${WS}/install/setup.bash" ]]; then
    source "${WS}/install/setup.bash"
  fi

  info "第二阶段：单线程编译 fast_lio/localization_sc/pgo_sc"
  colcon build --symlink-install \
    --parallel-workers 1 \
    --packages-select fast_lio localization_sc pgo_sc \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

  if [[ ! -f "${WS}/install/setup.bash" ]]; then
    err "编译结束后未找到 ${WS}/install/setup.bash"
    exit 1
  fi

  source "${WS}/install/setup.bash"
}

setup_bashrc_source() {
  info "写入 ~/.bashrc 自动 source"

  local ros_source="source /opt/ros/${ROS_DISTRO}/setup.bash"
  local ws_source="source ${WS}/install/setup.bash"
  local robot_env_source="source ${WS}/src/load_robot_env.sh"
  local display_export="export DISPLAY=192.168.50.50:0.0"

  if [[ ! -f "${WS}/src/load_robot_env.sh" ]]; then
    err "找不到机器人环境加载脚本：${WS}/src/load_robot_env.sh"
    exit 1
  fi

  grep -qxF "${ros_source}" "${HOME}/.bashrc" 2>/dev/null || echo "${ros_source}" >> "${HOME}/.bashrc"
  grep -qxF "${ws_source}" "${HOME}/.bashrc" 2>/dev/null || echo "${ws_source}" >> "${HOME}/.bashrc"
  grep -qxF "${robot_env_source}" "${HOME}/.bashrc" 2>/dev/null || echo "${robot_env_source}" >> "${HOME}/.bashrc"
  grep -qxF "${display_export}" "${HOME}/.bashrc" 2>/dev/null || echo "${display_export}" >> "${HOME}/.bashrc"

  cat > "${WS}/source_robot_env.sh" <<EOF
#!/usr/bin/env bash
source /opt/ros/${ROS_DISTRO}/setup.bash
source ${WS}/install/setup.bash
source ${WS}/src/load_robot_env.sh
export DISPLAY=192.168.50.50:0.0
EOF
  chmod +x "${WS}/source_robot_env.sh"

  info "已生成：${WS}/source_robot_env.sh"
}

main() {
  info "开始部署 robot_rk3588 环境"
  info "脚本目录：${SCRIPT_DIR}"
  info "工作空间：${WS}"
  info "工作空间压缩包：${WS_ZIP}"
  info "第三方目录：${THIRD_PARTY_DIR}"
  info "临时构建目录：${BUILD_TMP_ROOT}"
  info "并行编译数：${PARALLEL_WORKERS}"

  need_cmd sudo
  sudo -v

  prepare_workspace_from_zip
  check_workspace

  setup_locale_and_timezone
  ensure_ros_repo_if_needed
  install_apt_dependencies
  init_rosdep

  build_gtsam_411
  build_livox_sdk2
  build_teaserpp

  rosdep_install_workspace_deps
  build_workspace
  setup_bashrc_source

  if [[ "${SCRIPT_SOURCED}" -eq 1 ]]; then
    info "脚本是 source 执行的，正在 source ROS 和工作空间环境"
    # shellcheck disable=SC1090
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
    # shellcheck disable=SC1090
    source "${WS}/install/setup.bash"
    # shellcheck disable=SC1090
    source "${WS}/src/load_robot_env.sh"
    export DISPLAY=192.168.50.50:0.0
    info "当前终端已 source 完成，ROBOT_DATA_DIR=${ROBOT_DATA_DIR}，DISPLAY=${DISPLAY}"
  else
    warn "脚本正常执行完毕，但无法修改当前父终端环境"
    warn "当前终端请执行：source ${WS}/source_robot_env.sh"
    warn "或者重新打开终端，~/.bashrc 会自动 source"
  fi

  info "部署完成"
}

main "$@"
