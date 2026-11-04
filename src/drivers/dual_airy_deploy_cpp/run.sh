#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
# 默认采用 packet_225732 的本次 MLCC 矩阵；--extrinsics 可显式选择其他配置。
EXTRINSICS="$ROOT/config/extrinsics.yaml"
# 默认先测本次时差；fixed-time 仅适用于已验证的当前时钟会话。
MODE=startup_check
CHECK=false
ROS_ENV_MODE=auto
while [[ $# -gt 0 ]]; do
  case "$1" in
    --fixed-time) MODE=fixed; shift ;;
    --auto-time) MODE=startup_check; shift ;;
    --extrinsics) [[ $# -ge 2 && "$2" != --* ]] || { echo '--extrinsics requires FILE' >&2; exit 2; }; EXTRINSICS="$2"; shift 2 ;;
    --check) CHECK=true; shift ;;
    --ros-env) [[ $# -ge 2 ]] || exit 2; ROS_ENV_MODE="$2"; shift 2 ;;
    -h|--help)
      echo '用法：bash run.sh [--auto-time|--fixed-time] [--extrinsics FILE] [--ros-env auto|inherit] [--check]'
      echo '默认先独立检查四路时间，再用两个点云输入发布 /dual_airy/points_merged。'
      echo '--fixed-time 仅用于已验证的当前时钟会话；--check 只检查配置，不启动 ROS 节点。'
      echo '--ros-env 默认 auto：对齐唯一的本机驱动 domain/RMW，要求驱动取消 localhost-only；inherit 用于回放或远端驱动。'
      exit 0 ;;
    *) echo "未知参数：$1" >&2; exit 2 ;;
  esac
done
set +u
source "${DUAL_AIRY_ROS_SETUP:-/opt/ros/humble/setup.bash}"
set -u
[[ -x "$ROOT/build/dual_airy_fusion" && -x "$ROOT/build/dual_airy_time_check" ]] || {
  echo '请先在当前机器执行 bash build.sh 编译 C++ 源码。' >&2; exit 1;
}
# 此命令只验证配置/矩阵，不创建 ROS 节点，也不代表外参精度已验收。
"$ROOT/build/dual_airy_fusion" --check "$EXTRINSICS"
[[ -f "$ROOT/config/fastdds_fusion.xml" ]] || { echo '缺少 DDS 配置' >&2; exit 1; }
[[ "$CHECK" == false ]] || exit 0
source "$ROOT/ros_env.sh"
dual_airy_configure_ros_env "$ROS_ENV_MODE"
# 防止新版脚本重复启动同一 ROS 环境的检查器或融合进程；锁随进程退出释放。
command -v flock >/dev/null || { echo '缺少 util-linux 的 flock' >&2; exit 2; }
exec 9>"/tmp/dual_airy_cpp_${UID}_${ROS_DOMAIN_ID}_${ROS_LOCALHOST_ONLY}.lock"
flock -n 9 || { echo '本环境已有一份融合脚本运行，请先在旧终端 Ctrl+C。' >&2; exit 2; }
echo "C++ 部署版 v5；ROS_DOMAIN_ID=$ROS_DOMAIN_ID；startup_mode=$MODE"
if [[ "$MODE" == startup_check ]]; then
  SESSION_DIR="$(mktemp -d /tmp/dual_airy_cpp_session.XXXXXX)"
  echo "本次时间检查及会话矩阵：$SESSION_DIR"
  printf 'ROS_DOMAIN_ID=%s\nROS_LOCALHOST_ONLY=%s\nRMW_IMPLEMENTATION=%s\nFASTRTPS_DEFAULT_PROFILES_FILE=%s\n' \
    "$ROS_DOMAIN_ID" "$ROS_LOCALHOST_ONLY" "$RMW_IMPLEMENTATION" "$FASTRTPS_DEFAULT_PROFILES_FILE" > "$SESSION_DIR/environment.txt"
  # 同步等待检查进程退出；失败即退出。成功后读会话副本，不改原配置。
  "$ROOT/build/dual_airy_time_check" --extrinsics "$EXTRINSICS" --output "$SESSION_DIR/extrinsics.yaml"
  EXTRINSICS="$SESSION_DIR/extrinsics.yaml"
fi
# XML 已在启动检查前统一设置；两个阶段使用相同的通信配置。
echo '阶段 2/2：C++ 节点只订阅两路点云，后雷达变换到前雷达，发布 XYZIRT 融合云。'
echo "发布端 DDS 分片流控：$FASTRTPS_DEFAULT_PROFILES_FILE"
exec "$ROOT/build/dual_airy_fusion" --ros-args -p "extrinsics_file:=$EXTRINSICS"
