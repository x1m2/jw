#!/bin/bash

set -euo pipefail

# Select the runtime profile: airy, mid360s_3d or mid360s_2d.
LIDAR_TYPE="${1:-}"
case "$LIDAR_TYPE" in
  airy|mid360s_3d|mid360s_2d) shift ;;
  *) echo "Usage: bash $0 {airy|mid360s_3d|mid360s_2d} [运行用户名] [OTA目录]" >&2; exit 1 ;;
esac
if [[ $# -gt 2 ]]; then
  echo "[ERROR] 参数过多" >&2
  exit 1
fi
SERVICE_NAME="nav-runtime.service"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TEMPLATE_FILE="$SCRIPT_DIR/nav-runtime-${LIDAR_TYPE}.service.in"
SERVICE_FILE="/etc/systemd/system/$SERVICE_NAME"

if [ "$(id -u)" -ne 0 ]; then
  echo "[ERROR] 请使用 sudo 执行：sudo bash $0 $LIDAR_TYPE [运行用户名] [OTA目录]"
  exit 1
fi

RUN_USER="${1:-${SUDO_USER:-jw}}"

if ! USER_ENTRY="$(getent passwd "$RUN_USER")"; then
  echo "[ERROR] 用户不存在：$RUN_USER"
  exit 1
fi

RUN_GROUP="$(id -gn "$RUN_USER")"
RUN_HOME="$(printf '%s' "$USER_ENTRY" | cut -d: -f6)"
DEPLOY_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
OTA_ROOT="${2:-${OTA_ROOT:-}}"
if [[ -z "$OTA_ROOT" ]]; then
  if [[ -d "$DEPLOY_ROOT/slam/pack/content" ]]; then
    OTA_ROOT="$DEPLOY_ROOT"
  else
    OTA_ROOT="$RUN_HOME/ota"
  fi
fi
if [[ ! -d "$OTA_ROOT" ]]; then
  echo "[ERROR] OTA 目录不存在：$OTA_ROOT" >&2
  exit 1
fi
OTA_ROOT="$(cd -- "$OTA_ROOT" && pwd)"
for module in nav slam slam_manager; do
  if [[ ! -d "$OTA_ROOT/$module/pack/content" ]]; then
    echo "[ERROR] OTA 模块不存在：$OTA_ROOT/$module/pack/content" >&2
    exit 1
  fi
done
START_SCRIPT="$DEPLOY_ROOT/start_node_${LIDAR_TYPE}.sh"

if [ ! -f "$TEMPLATE_FILE" ]; then
  echo "[ERROR] service 模板不存在：$TEMPLATE_FILE"
  exit 1
fi

if [ ! -f "$START_SCRIPT" ]; then
  echo "[ERROR] 启动脚本不存在：$START_SCRIPT"
  exit 1
fi

# 通过 bash 运行，不需要给启动脚本增加执行权限。
# 检查运行用户是否可以读取脚本、进入工作目录。
if ! runuser -u "$RUN_USER" -- test -r "$START_SCRIPT"; then
  echo "[ERROR] 用户 $RUN_USER 无法读取：$START_SCRIPT"
  echo "[INFO] 请检查文件读取权限和上级目录访问权限"
  exit 1
fi

if ! runuser -u "$RUN_USER" -- test -x "$OTA_ROOT"; then
  echo "[ERROR] 用户 $RUN_USER 无法进入：$OTA_ROOT"
  exit 1
fi

TEMP_SERVICE="$(mktemp)"
trap 'rm -f "$TEMP_SERVICE"' EXIT

escape_sed_replacement() {
  printf '%s' "$1" | sed 's/[&|\\]/\\&/g'
}

sed \
  -e "s|@OTA_ROOT@|$(escape_sed_replacement "$OTA_ROOT")|g" \
  -e "s|@RUN_USER@|$(escape_sed_replacement "$RUN_USER")|g" \
  -e "s|@RUN_GROUP@|$(escape_sed_replacement "$RUN_GROUP")|g" \
  -e "s|@RUN_HOME@|$(escape_sed_replacement "$RUN_HOME")|g" \
  -e "s|@START_SCRIPT@|$(escape_sed_replacement "$START_SCRIPT")|g" \
  "$TEMPLATE_FILE" > "$TEMP_SERVICE"

install -m 0644 "$TEMP_SERVICE" "$SERVICE_FILE"

systemctl daemon-reload
systemctl enable "$SERVICE_NAME"

echo "[DONE] 已安装并启用：$SERVICE_NAME"
echo "[INFO] 启动方案：$LIDAR_TYPE"
echo "[INFO] 运行用户：$RUN_USER"
echo "[INFO] 启动脚本：$START_SCRIPT"
echo "[INFO] 下次开机将自动启动"
echo "[INFO] 立即启动或应用更新：sudo systemctl restart $SERVICE_NAME"
echo "[INFO] 查看状态：sudo systemctl status $SERVICE_NAME"
echo "[INFO] 查看输出：sudo journalctl -u $SERVICE_NAME -f"
