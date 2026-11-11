#!/bin/bash
# Install only the exact command used by nav_server; run once on the robot board.
set -euo pipefail

CHECK_ONLY=false
if [[ "${1:-}" == --check ]]; then
  CHECK_ONLY=true
  shift
fi
if (( $# > 1 )); then
  echo "Usage: sudo bash $0 [运行用户名] | bash $0 --check [运行用户名]" >&2
  exit 1
fi
RUN_USER="${1:-${SUDO_USER:-$(id -un)}}"
if [[ ! "$RUN_USER" =~ ^[a-zA-Z_][a-zA-Z0-9_-]*$ ]] || ! getent passwd "$RUN_USER" >/dev/null; then
  echo "[ERROR] 无效或不存在的运行用户：$RUN_USER" >&2
  exit 1
fi
if ! "$CHECK_ONLY" && (( EUID != 0 )); then
  echo "[ERROR] 首次安装需要管理员权限：sudo bash $0 $RUN_USER" >&2
  exit 1
fi
# Root already has restart permission; remove any grant left for a former user.
if [[ "$(id -u "$RUN_USER")" == 0 ]]; then
  if ! "$CHECK_ONLY"; then
    rm -f -- /etc/sudoers.d/nav-runtime-restart
  fi
  echo "[INFO] 运行用户为 root，无需额外重启授权"
  exit 0
fi
VISUDO="$(command -v visudo || true)"
if [[ -z "$VISUDO" || ! -x /usr/bin/systemctl || ! -x /usr/bin/sudo ]]; then
  echo "[ERROR] 需要安装 sudo、visudo 和 /usr/bin/systemctl" >&2
  exit 1
fi

umask 077
TEMP_POLICY="$(mktemp)"
STAGED_POLICY=""
trap 'rm -f -- "$TEMP_POLICY"; if [[ -n "$STAGED_POLICY" ]]; then rm -f -- "$STAGED_POLICY"; fi' EXIT
printf '%s\n' \
  '# Managed by robot_ws/deploy/ota/systemd/install_runtime_restart_permission.sh' \
  "$RUN_USER ALL=(root) NOPASSWD: /usr/bin/systemctl --no-block restart nav-runtime.service" \
  > "$TEMP_POLICY"
"$VISUDO" -cf "$TEMP_POLICY"
if "$CHECK_ONLY"; then
  cat "$TEMP_POLICY"
  echo "[DONE] 权限配置语法检查通过，未修改系统配置"
  exit 0
fi

install -d -o root -g root -m 0755 /etc/sudoers.d
# A dot in this temporary filename keeps sudo from loading it before rename.
STAGED_POLICY="$(mktemp /etc/sudoers.d/.nav-runtime-restart.XXXXXX)"
install -o root -g root -m 0440 "$TEMP_POLICY" "$STAGED_POLICY"
mv -fT -- "$STAGED_POLICY" /etc/sudoers.d/nav-runtime-restart
STAGED_POLICY=""
if ! runuser -u "$RUN_USER" -- /usr/bin/sudo -n -l -- /usr/bin/systemctl --no-block restart nav-runtime.service >/dev/null; then
  echo "[ERROR] 配置已写入但权限检查失败，请检查 sudoers 是否加载 /etc/sudoers.d" >&2
  exit 1
fi
echo "[DONE] 用户 $RUN_USER 可免密重启 nav-runtime.service"
echo "[INFO] 仅授权该服务的固定重启命令；未实际重启服务"
echo "[INFO] 请以 $RUN_USER 运行以下命令检查权限："
echo 'sudo -n -l -- /usr/bin/systemctl --no-block restart nav-runtime.service'
