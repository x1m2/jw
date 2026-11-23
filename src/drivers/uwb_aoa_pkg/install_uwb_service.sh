#!/usr/bin/env bash
set -euo pipefail

# --print prints the unit for inspection without installing or starting it.
print_only=false
if [[ "${1:-}" == "--print" ]]; then
  print_only=true
  shift
fi
run_user="${1:-${SUDO_USER:-$(id -un)}}"
if [[ "$run_user" == root ]]; then
  echo "请指定运行节点的普通用户: sudo bash $0 <用户名>" >&2
  exit 1
fi
id "$run_user" >/dev/null
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

render_unit() {
  cat <<EOF
[Unit]
Description=UWB AOA robot serial node
After=network.target
StartLimitIntervalSec=0

[Service]
Type=simple
User=$run_user
SupplementaryGroups=dialout
WorkingDirectory=$script_dir
Environment="ROS_DISTRO=${ROS_DISTRO:-humble}"
# Prefer ttyUSB0, then ttyUSB1; retry startup if neither device exists.
ExecStartPre=+/bin/bash -c 'for device in /dev/ttyUSB0 /dev/ttyUSB1; do if [[ -c "\$\$device" ]]; then /usr/bin/chgrp dialout "\$\$device" && /usr/bin/chmod 666 "\$\$device"; exit \$\$?; fi; done; echo "No UWB serial device found: /dev/ttyUSB0 or /dev/ttyUSB1" >&2; exit 1'
ExecStart=/bin/bash "$script_dir/start_libAoa_robot.sh"
Restart=on-failure
RestartSec=5
KillMode=control-group
TimeoutStopSec=15

[Install]
WantedBy=multi-user.target
EOF
}

if "$print_only"; then
  render_unit
  exit 0
fi
if [[ "$EUID" != 0 ]]; then
  exec sudo /usr/bin/env "ROS_DISTRO=${ROS_DISTRO:-humble}" /bin/bash "$script_dir/install_uwb_service.sh" "$run_user"
fi

if [[ ! -f "$script_dir/../../install/setup.bash" ]]; then
  echo "未找到脚本上两级的 install/setup.bash，请先编译 uwb_aoa_pkg" >&2
  exit 1
fi
getent group dialout >/dev/null
unit_file="$(mktemp)"
trap 'rm -f "$unit_file"' EXIT
render_unit > "$unit_file"
install -m 644 "$unit_file" /etc/systemd/system/uwb-aoa.service
systemctl daemon-reload
systemctl enable --now uwb-aoa.service
echo "已安装并启用 uwb-aoa.service。查看日志: journalctl -u uwb-aoa.service -f"
