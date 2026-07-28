#!/usr/bin/env bash
# 录制 Livox / IMU / TF ROS 2 bag。
#
# 固定保存根目录：
#   /robotfs/data/slam_data/bag
#
# 行为：
# - 启动前清空 /robotfs/data/slam_data/bag 目录下的所有旧文件和旧 bag 子目录；
#   只清空内容，不删除 /robotfs/data/slam_data/bag 目录本身。
# - 若 bag 目录不存在、不是目录、或当前用户无写权限，直接报错退出。
# - 不会自动创建 /robotfs/data/slam_data/bag。
# - Ctrl+C 会正常停止 ros2 bag record 并写入 metadata.yaml。

set -o pipefail

BAG_ROOT="/robotfs/data/slam_data/bag"
WORKSPACE_SETUP="/home/jw/robot_rk3588_ws/install/setup.bash"

TOPICS=(
  "/livox/lidar"
  "/livox/imu"
  "/tf"
  "/tf_static"
)

info()  { echo "[INFO] $*"; }
error() { echo "[ERROR] $*" >&2; }

usage() {
  cat <<'EOF'
用法：
  ./record_bag.sh
  ./record_bag.sh -n test_001
  ./record_bag.sh --name test_001

说明：
  每次启动前会清空 /robotfs/data/slam_data/bag 下全部历史录包数据。
  保存根目录固定为 /robotfs/data/slam_data/bag。
  未指定名称时，自动生成 livox_bag_YYYYMMDD_HHMMSS。
EOF
}

BAG_NAME=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    -n|--name)
      if [[ $# -lt 2 || -z "${2}" ]]; then
        error "$1 缺少 bag 名称"
        usage
        exit 1
      fi
      BAG_NAME="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      error "未知参数：$1"
      usage
      exit 1
      ;;
  esac
done

# 先加载环境；两个文件存在时都加载。
if [[ -f "${WORKSPACE_SETUP}" ]]; then
  # shellcheck source=/dev/null
  source "${WORKSPACE_SETUP}"
fi

if ! command -v ros2 >/dev/null 2>&1; then
  error "未找到 ros2 命令。请检查 ROS 2 环境。"
  exit 1
fi

# 防止生成不含 /livox/lidar 的无效 bag。
if ! ros2 pkg prefix livox_ros_driver2 >/dev/null 2>&1; then
  error "当前环境找不到 livox_ros_driver2。"
  error "ros2 bag 无法识别 /livox/lidar 的 CustomMsg，已停止录制。"
  exit 1
fi

# 只验证，不创建 bag 根目录。
if [[ ! -e "${BAG_ROOT}" ]]; then
  error "bag 目录不存在：${BAG_ROOT}"
  error "不会自动创建目录，请先确认磁盘挂载和目录路径。"
  exit 1
fi

if [[ ! -d "${BAG_ROOT}" ]]; then
  error "路径不是目录：${BAG_ROOT}"
  exit 1
fi

if [[ ! -w "${BAG_ROOT}" || ! -x "${BAG_ROOT}" ]]; then
  error "当前用户没有 bag 目录写权限：${BAG_ROOT}"
  exit 1
fi

# 明确锁死清空目标，避免变量被意外改成其他路径。
if [[ "${BAG_ROOT}" != "/robotfs/data/slam_data/bag" ]]; then
  error "安全检查失败：BAG_ROOT 不是预期路径，拒绝清空。"
  exit 1
fi

# 已有 recorder 仍在运行时，不清空，避免删除正在写入的 bag。
if pgrep -af "[r]os2 bag record" >/dev/null 2>&1; then
  error "检测到已有 ros2 bag record 正在运行。"
  error "请先在旧录包终端按 Ctrl+C 正常结束后，再启动本脚本。"
  exit 1
fi

info "清空历史 bag 数据：${BAG_ROOT}"

# 清空目录内的一切：包括隐藏文件、bag 子目录、db3 和 metadata.yaml。
# 不会删除 BAG_ROOT 本身。
if ! find "${BAG_ROOT}" -mindepth 1 -maxdepth 1 -exec rm -rf -- {} +; then
  error "清空旧 bag 数据失败。请检查目录内是否有 root 所属文件或权限不足。"
  exit 1
fi

# 二次确认目录已空。
if find "${BAG_ROOT}" -mindepth 1 -maxdepth 1 -print -quit | grep -q .; then
  error "bag 目录仍有残留文件，已停止录制。"
  exit 1
fi

if [[ -z "${BAG_NAME}" ]]; then
  BAG_NAME="livox_bag_$(date +%Y%m%d_%H%M%S)"
fi

if [[ "${BAG_NAME}" == *"/"* || "${BAG_NAME}" == "." || "${BAG_NAME}" == ".." ]]; then
  error "bag 名称只能是单个目录名，不能包含 '/'。"
  exit 1
fi

BAG_DIR="${BAG_ROOT}/${BAG_NAME}"

info "Livox 消息包：$(ros2 pkg prefix livox_ros_driver2)"
info "开始录制 ROS 2 bag"
info "保存目录：${BAG_DIR}"
info "录制话题："
for topic in "${TOPICS[@]}"; do
  echo "  ${topic}"
done
info "在当前终端按 Ctrl+C 停止并保存。"

exec ros2 bag record \
  -o "${BAG_DIR}" \
  "${TOPICS[@]}"