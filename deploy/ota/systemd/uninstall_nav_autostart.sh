#!/bin/bash

set -euo pipefail

SERVICE_NAME="nav-runtime.service"
SERVICE_FILE="/etc/systemd/system/$SERVICE_NAME"

if [ "$(id -u)" -ne 0 ]; then
  echo "[ERROR] 请使用 sudo 执行：sudo bash $0"
  exit 1
fi

# Also handle units that remain loaded after their service file was removed.
LOAD_STATE="$(systemctl show --property=LoadState --value "$SERVICE_NAME")"
if [ "$LOAD_STATE" != "not-found" ] || [ -e "$SERVICE_FILE" ] || [ -L "$SERVICE_FILE" ]; then
  systemctl stop "$SERVICE_NAME"
  systemctl disable "$SERVICE_NAME"
fi

rm -f -- "$SERVICE_FILE"
systemctl daemon-reload

echo "[DONE] 已停止并卸载：$SERVICE_NAME"
echo "[INFO] 已取消开机启动；OTA 程序、配置和运行数据已保留"
