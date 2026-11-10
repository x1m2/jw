#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ "$EUID" != 0 ]]; then
  exec sudo /bin/bash "$script_dir/uninstall_uwb_service.sh"
fi

service_name="uwb-aoa.service"
unit_file="/etc/systemd/system/$service_name"

# Allow repeated uninstalls when the service is already absent.
if [[ -e "$unit_file" || -L "$unit_file" ]] ||
   systemctl is-active --quiet "$service_name" ||
   systemctl is-enabled --quiet "$service_name"; then
  systemctl disable --now "$service_name"
fi
rm -f -- "$unit_file"
systemctl daemon-reload
if systemctl is-failed --quiet "$service_name"; then
  systemctl reset-failed "$service_name"
fi

echo "已卸载 uwb-aoa.service，停止服务并取消开机自启动。"
