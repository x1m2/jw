#!/usr/bin/env bash
set -euo pipefail

# Run after extracting this source package to /home/jw/uwb_aoa_pkg.
print_only=false
if [[ "${1:-}" == --print ]]; then
  print_only=true
  shift
fi
if (( $# > 1 )); then
  echo "Usage: bash $0 [--print] [run_user]" >&2
  exit 1
fi
run_user="${1:-${SUDO_USER:-$(id -un)}}"
if [[ "$run_user" == root ]]; then
  run_user=jw
fi
id "$run_user" >/dev/null
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ros_distro="${ROS_DISTRO:-humble}"
run_home="$(getent passwd "$run_user" | cut -d: -f6)"
workspace="${UWB_WS:-$run_home/uwb_ws}"
[[ "$workspace" =~ ^/[a-zA-Z0-9_./-]+$ ]] || { echo "Invalid UWB_WS" >&2; exit 1; }
workspace="$(realpath -m -- "$workspace")"
package_dir="$workspace/src/uwb_aoa_pkg"
# Keep paths safe for systemd Environment/ExecStart and specifier expansion.
if [[ ! "$script_dir" =~ ^/[a-zA-Z0-9_./-]+$ || ! "$ros_distro" =~ ^[a-zA-Z0-9_-]+$ ||
      ! "$run_user" =~ ^[a-zA-Z_][a-zA-Z0-9_-]*$ ]]; then
  echo "Unsupported package path, ROS distribution or username" >&2
  exit 1
fi

render_unit() {
  local unit
  unit="$(cat "$script_dir/dds/uwb-aoa.service")"
  unit="${unit//\/home\/jw\/uwb_ws\/src\/uwb_aoa_pkg/$package_dir}"
  unit="${unit//User=jw/User=$run_user}"
  unit="${unit//ROS_DISTRO=humble/ROS_DISTRO=$ros_distro}"
  printf '%s\n' "$unit"
}

if "$print_only"; then
  render_unit
  exit 0
fi
for required in "/opt/ros/$ros_distro/setup.bash" "$script_dir/start_libAoa_robot.sh" "$script_dir/dds/fastdds.xml" "$script_dir/dds/uwb-aoa.service"; do
  [[ -r "$required" ]] || { echo "Missing file: $required" >&2; exit 1; }
done
getent group dialout >/dev/null

# Copy and build as the service user. Keep the extracted source package intact.
# Exclude old generated files so another machine's build cache is never copied.
echo "部署源码到 $package_dir，并在 $workspace 下编译"
build_command='set -eo pipefail
source "/opt/ros/$3/setup.bash"
command -v colcon >/dev/null || { echo "请先安装 python3-colcon-common-extensions" >&2; exit 1; }
mkdir -p "$2/src/uwb_aoa_pkg"
source_dir="$(cd "$1" && pwd -P)"
target_dir="$(cd "$2/src/uwb_aoa_pkg" && pwd -P)"
if [[ "$source_dir" != "$target_dir" ]]; then
  tar -C "$source_dir" --exclude=./build --exclude=./install --exclude=./log --exclude=./.git --exclude=__pycache__ -cf - . |
    tar -C "$target_dir" --no-same-owner -xf -
fi
cd "$2"
colcon --log-base "$2/log" build --base-paths "$target_dir" --packages-select uwb_aoa_pkg --build-base "$2/build" --install-base "$2/install"'
if [[ "$EUID" == 0 ]]; then
  runuser -u "$run_user" -- /bin/bash -c "$build_command" -- "$script_dir" "$workspace" "$ros_distro"
else
  [[ "$(id -un)" == "$run_user" ]] || { echo "请以 $run_user 用户执行安装" >&2; exit 1; }
  /bin/bash -c "$build_command" -- "$script_dir" "$workspace" "$ros_distro"
fi
[[ -r "$workspace/install/setup.bash" ]] || { echo "编译产物缺失" >&2; exit 1; }

as_root() {
  if [[ "$EUID" == 0 ]]; then
    "$@"
  else
    sudo "$@"
  fi
}
unit_file="$(mktemp)"
trap 'rm -f "$unit_file"' EXIT
render_unit > "$unit_file"
as_root install -m 644 "$unit_file" /etc/systemd/system/uwb-aoa.service
as_root systemctl daemon-reload
# The original unit has no WantedBy; create the boot dependency explicitly.
as_root systemctl add-wants multi-user.target uwb-aoa.service
# Restart also applies changed paths / DDS settings to an existing service.
as_root systemctl restart uwb-aoa.service
echo "已安装、重启并启用 uwb-aoa.service 开机自启。"
echo "DDS 配置：$package_dir/dds/fastdds.xml（请核对其中网卡 IP）"
echo "查看日志：journalctl -u uwb-aoa.service -f"
