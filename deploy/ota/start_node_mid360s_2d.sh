#!/bin/bash
# MID360s + terrain_filter / costmap_2d / Pure Pursuit (OTA nav 2D).
set -euo pipefail

CHECK_ONLY=false
case "${1:-}" in
  --check) CHECK_ONLY=true; shift ;;
  --finish-session)
    # systemd calls this after shutdown. Keep the session across failures so
    # automatic retries append to the same logs; a clean stop/restart ends it.
    if [[ "${SERVICE_RESULT:-}" == success && -n "${RUNTIME_DIRECTORY:-}" ]]; then
      rm -f -- "$RUNTIME_DIRECTORY/log-session"
    fi
    exit 0
    ;;
  "") ;;
  *) echo "Usage: bash $0 [--check]" >&2; exit 1 ;;
esac
if (( $# != 0 )); then
  echo "Usage: bash $0 [--check]" >&2
  exit 1
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -z "${OTA_ROOT:-}" ]]; then
  if [[ -d "$SCRIPT_DIR/slam/pack/content" ]]; then
    OTA_ROOT="$SCRIPT_DIR"
  else
    OTA_ROOT="/home/jw/ota"
  fi
fi
if [[ ! -d "$OTA_ROOT" ]]; then
  echo "[ERROR] OTA directory not found: $OTA_ROOT" >&2
  exit 1
fi
OTA_ROOT="$(cd -- "$OTA_ROOT" && pwd)"
for module in nav slam slam_manager; do
  if [[ ! -d "$OTA_ROOT/$module/pack/content" ]]; then
    echo "[ERROR] OTA module not found: $OTA_ROOT/$module/pack/content" >&2
    exit 1
  fi
done
SLAM_DIR="$OTA_ROOT/slam/pack/content"
NAV_DIR="$OTA_ROOT/nav/pack/content"
MANAGER_DIR="$OTA_ROOT/slam_manager/pack/content"
# Keep the existing deployed map / task / log location.
DATA_ROOT="/home/jw/ota/data/slam_data"
# Prefer the explicitly selected P2.5 profile; 2D OTA branches may ship only
# robot_param.yaml. An explicit override must exist (never silently fall back).
if [[ -z "${ROBOT_PARAM_CONFIG:-}" ]]; then
  if [[ -f "$MANAGER_DIR/config/p2.5_mid360s.yaml" ]]; then
    ROBOT_PARAM_CONFIG="$MANAGER_DIR/config/p2.5_mid360s.yaml"
  else
    ROBOT_PARAM_CONFIG="$MANAGER_DIR/config/robot_param.yaml"
  fi
fi
NAV_SERVER_SOURCE_CONFIG="$MANAGER_DIR/config/server.yaml"
LIVOX_CONFIG="$SLAM_DIR/config/MID360s_driver.yaml"
LIVOX_JSON_CONFIG="$SLAM_DIR/config/MID360s_config.json"
MAPPING_CONFIG="$SLAM_DIR/config/mapping_mid360s.yaml"
LOCALIZATION_CONFIG="$SLAM_DIR/config/reloc_mid360s.yaml"
DDS_CONFIG="$SLAM_DIR/config/fastshm.xml"

# Every process is a driver or a ManagedWorker. Manager starts the algorithms
# through set_state services when the APP requests mapping/localization/navigation.
BINARIES=(
  "$SLAM_DIR/lidar/livox_ros_driver2_node"
  "$SLAM_DIR/mapping/fastlio_mapping_node"
  "$SLAM_DIR/pgo_sc/pgo_sc_node"
  "$SLAM_DIR/localization/fastlio_localization_node"
  "$NAV_DIR/terrain_filter/terrain_filter_node"
  "$NAV_DIR/obstacle_stop/obstacle_stop_node"
  "$NAV_DIR/costmap_2d/costmap_2d_node"
  "$NAV_DIR/global_path_planner/global_path_planner_node"
  "$NAV_DIR/path_smoother/path_smoother_node"
  "$NAV_DIR/pure_pursuit/pure_pursuit_node"
  "$MANAGER_DIR/slam_manager/slam_manager_node"
  "$MANAGER_DIR/nav_server/nav_server_node"
)
CONFIG_FILES=(
  "$LIVOX_CONFIG" "$LIVOX_JSON_CONFIG" "$DDS_CONFIG"
  "$MAPPING_CONFIG" "$LOCALIZATION_CONFIG" "$ROBOT_PARAM_CONFIG"
  "$SLAM_DIR/config/pgo_mid360s.yaml"
  "$NAV_DIR/config/terrain_filter.yaml"
  "$NAV_DIR/config/obstacle_stop_p2.5_mid360s.yaml"
  "$NAV_DIR/config/costmap_2d.yaml"
  "$NAV_DIR/config/global_path_planner_params.yaml"
  "$NAV_DIR/config/path_smoother.yaml"
  "$NAV_DIR/config/pure_pursuit_params.yaml"
  "$MANAGER_DIR/config/slam_manager.yaml"
  "$NAV_SERVER_SOURCE_CONFIG"
  "$MANAGER_DIR/config/parameter_rules.yaml"
)

check_error=0
for binary in "${BINARIES[@]}" "$SLAM_DIR/localization/voxelmaps_saver"; do
  if [[ ! -f "$binary" || ! -x "$binary" ]]; then
    echo "[ERROR] executable missing or not executable: $binary" >&2
    check_error=1
  fi
done
for config_file in "${CONFIG_FILES[@]}"; do
  if [[ ! -f "$config_file" || ! -r "$config_file" ]]; then
    echo "[ERROR] configuration missing or unreadable: $config_file" >&2
    check_error=1
  fi
done

# The 2D OTA server.yaml uses /robotfs paths. Prepare a private runtime copy
# with the actual deployment paths; keep the source YAML and its settings intact.
# Empty output path validates only, so --check does not write files.
prepare_server_config() {
  python3 - "$NAV_SERVER_SOURCE_CONFIG" "$OTA_ROOT" "$DATA_ROOT" "${1:-}" <<'PYCONFIG'
import os
from pathlib import Path
import sys
import yaml
source, ota, data, output = sys.argv[1:]
source = Path(source)
with source.open(encoding="utf-8") as stream:
    config = yaml.safe_load(stream)
if not isinstance(config, dict) or not isinstance(config.get("server"), dict):
    raise ValueError("server.yaml must contain a server mapping")
replacements = [("/robotfs/data/slam_data", data)]
for module in ("nav", "slam", "slam_manager"):
    replacements.append((f"/robotfs/app/module/{module}", f"{ota}/{module}/pack/content"))
    replacements.append((f"/home/jw/ota/{module}/pack/content", f"{ota}/{module}/pack/content"))
def relocate(value):
    if isinstance(value, dict):
        return {key: relocate(item) for key, item in value.items()}
    if isinstance(value, list):
        return [relocate(item) for item in value]
    if isinstance(value, str):
        for old, new in replacements:
            if value == old or value.startswith(old + "/"):
                return new + value[len(old):]
    return value
config = relocate(config)
# Moving server.yaml must not change where relative rules/scan paths resolve.
for definition in config.get("configs", {}).values():
    rules = definition.get("parameter_rules")
    if rules and not Path(rules).is_absolute():
        definition["parameter_rules"] = str(source.parent / rules)
    for scan in definition.get("scan_roots", []):
        path = scan.get("path")
        if path and not Path(path).is_absolute():
            scan["path"] = str(source.parent / path)
if output:
    fd = os.open(output, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    os.fchmod(fd, 0o600)
    with os.fdopen(fd, "w", encoding="utf-8") as stream:
        yaml.safe_dump(config, stream, allow_unicode=True, sort_keys=False)
PYCONFIG
}
if [[ -r "$NAV_SERVER_SOURCE_CONFIG" ]] && ! prepare_server_config; then
  echo "[ERROR] cannot prepare nav_server configuration (Python 3 and PyYAML required)" >&2
  check_error=1
fi

# Known SCAN-only Manager builds cannot activate terrain_filter/costmap or pause
# Pure Pursuit. Remapping a SCAN service name does not add those missing workers.
# Inspect literals without executing the deployed ARM binary (also works on x86).
manager_binary="$MANAGER_DIR/slam_manager/slam_manager_node"
if [[ -r "$manager_binary" ]] && LC_ALL=C grep -aFq '/scan_planner_worker/set_state' "$manager_binary"; then
  if ! LC_ALL=C grep -aFq '/terrain_filter_worker/set_state' "$manager_binary" ||
     ! LC_ALL=C grep -aFq '/costmap_worker/set_state' "$manager_binary"; then
    echo "[ERROR] deployed slam_manager uses the 3D SCAN navigation controller." >&2
    echo "[ERROR] Install a 2D Manager build supporting terrain_filter, costmap and Pure Pursuit, with matching interface libraries." >&2
    check_error=1
  fi
fi
if (( check_error != 0 )); then
  echo "[ERROR] MID360s 2D preflight failed; no nodes have been stopped or started." >&2
  exit 1
fi
if [[ ! -r /opt/ros/humble/setup.bash ]]; then
  echo "[ERROR] ROS 2 Humble setup not found" >&2
  exit 1
fi
set +u
source /opt/ros/humble/setup.bash
set -u
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# All modules share the SLAM profile, including 2D nav which has no XML of its own.
unset FASTDDS_DEFAULT_PROFILES_FILE
export FASTRTPS_DEFAULT_PROFILES_FILE="$DDS_CONFIG"
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
# Resolve each module's own libraries first; avoid loading 3D message libraries
# from SLAM ahead of the 2D navigation module's libraries.
ROS_LIB_PATH="/opt/ros/humble/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
SLAM_LIB_PATH="$SLAM_DIR/lib:$MANAGER_DIR/lib:$NAV_DIR/lib:$ROS_LIB_PATH"
NAV_LIB_PATH="$NAV_DIR/lib:$MANAGER_DIR/lib:$SLAM_DIR/lib:$ROS_LIB_PATH"
MANAGER_LIB_PATH="$MANAGER_DIR/lib:$NAV_DIR/lib:$SLAM_DIR/lib:$ROS_LIB_PATH"
echo "[CONFIG] robot parameters: $ROBOT_PARAM_CONFIG"
echo "[OK] MID360s 2D runtime files and known Manager compatibility checks passed"
if "$CHECK_ONLY"; then exit 0; fi

mkdir -p "$DATA_ROOT/log/runtime"
# Prevent a second 2D supervisor from stopping the first one's children.
exec 9>"$DATA_ROOT/nav-runtime-2d.lock"
if ! flock -n 9; then
  echo "[ERROR] another MID360s 2D supervisor is already running" >&2
  exit 1
fi
# RuntimeDirectoryPreserve=restart keeps this marker across automatic retries.
# Manual execution has no marker and always starts a fresh log session.
SESSION_NAME=""
if [[ -n "${RUNTIME_DIRECTORY:-}" && -r "$RUNTIME_DIRECTORY/log-session" ]]; then
  read -r SESSION_NAME < "$RUNTIME_DIRECTORY/log-session" || true
fi
if [[ ! "$SESSION_NAME" =~ ^[0-9]{8}_[0-9]{6}_[0-9]+$ ]]; then
  SESSION_NAME="$(date '+%Y%m%d_%H%M%S')_$$"
fi
LOG_DIR="$DATA_ROOT/log/runtime/$SESSION_NAME"
if [[ -n "${RUNTIME_DIRECTORY:-}" ]]; then
  printf '%s\n' "$SESSION_NAME" > "$RUNTIME_DIRECTORY/log-session"
fi
export ROS_LOG_DIR="$LOG_DIR/ros"
mkdir -p "$ROS_LOG_DIR"
# Keep the private server configuration outside the exported log directory.
mkdir -p "$DATA_ROOT/.runtime"
chmod 700 "$DATA_ROOT/.runtime"
NAV_SERVER_CONFIG="$DATA_ROOT/.runtime/server_mid360s_2d.yaml"
prepare_server_config "$NAV_SERVER_CONFIG"
printf '%s\n' "$LOG_DIR" > "$DATA_ROOT/current_runtime_log.txt"
chmod 664 "$DATA_ROOT/current_runtime_log.txt" 2>/dev/null || true

NODE_PIDS=()
NODE_NAMES=()
cleanup() {
  trap - INT TERM EXIT
  # Reverse startup order: stop Manager/control before perception and SLAM.
  for ((i=${#NODE_PIDS[@]}-1; i>=0; i--)); do
    kill -INT "${NODE_PIDS[i]}" 2>/dev/null || true
  done
  for signal in TERM KILL; do
    for _ in {1..10}; do
      alive=false
      for pid in "${NODE_PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then alive=true; break; fi
      done
      if ! "$alive"; then break; fi
      sleep 0.2
    done
    for pid in "${NODE_PIDS[@]}"; do kill -"$signal" "$pid" 2>/dev/null || true; done
  done
  for pid in "${NODE_PIDS[@]}"; do wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT
trap 'exit 0' INT TERM

# Clear processes left by the previous profile/script. Match executable command
# lines, not shell commands or GUI names containing a node name as an argument.
LEGACY_NAMES=(livox_ros_driver2_node rslidar_sdk_node dual_airy_fusion_node
  woodox_imu_bridge_node fastlio_mapping_node pgo_sc_node fastlio_localization_node
  terrain_filter_node obstacle_stop_node costmap_2d_node global_path_planner_node
  path_smoother_node pure_pursuit_node scan_planner_node slam_manager_node nav_server_node)
for signal in INT TERM KILL; do
  for name in "${LEGACY_NAMES[@]}"; do
    pkill -"$signal" -f "^/[^[:space:]]*/${name}([[:space:]]|$)" 2>/dev/null || true
  done
  if [[ "$signal" != KILL ]]; then sleep 1; fi
done

# Keep a single supervisor history for all attempts in this session. The tee
# process must not inherit the instance lock; node PIDs remain directly tracked.
exec > >(tee -a "$LOG_DIR/supervisor.log" 9>&-) 2>&1
echo "[ATTEMPT] $(date '+%Y-%m-%d %H:%M:%S') supervisor PID=$$"
echo "[LOG] $LOG_DIR"
echo "[DDS] $FASTRTPS_DEFAULT_PROFILES_FILE"
start_node() {
  local name="$1" lib_path="$2" binary="$3"
  shift 3
  echo "[START] $name"
  printf '\n[ATTEMPT] %s supervisor PID=%s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$$" >> "$LOG_DIR/$name.log"
  # Direct child PID is the executable, so supervision observes node failures
  # rather than the exit status of a tee pipeline. Close the inherited lock FD.
  env LD_LIBRARY_PATH="$lib_path" stdbuf -oL -eL "$binary" "$@" \
    >> "$LOG_DIR/$name.log" 2>&1 9>&- &
  NODE_PIDS+=("$!")
  NODE_NAMES+=("$name")
}
start_node livox_ros_driver2 "$SLAM_LIB_PATH" "$SLAM_DIR/lidar/livox_ros_driver2_node" \
  --ros-args --params-file "$LIVOX_CONFIG" -p user_config_path:="$LIVOX_JSON_CONFIG"
start_node fastlio_mapping "$SLAM_LIB_PATH" "$SLAM_DIR/mapping/fastlio_mapping_node" \
  --ros-args --params-file "$MAPPING_CONFIG" --params-file "$ROBOT_PARAM_CONFIG"
start_node pgo_sc "$SLAM_LIB_PATH" "$SLAM_DIR/pgo_sc/pgo_sc_node" \
  --ros-args --params-file "$SLAM_DIR/config/pgo_mid360s.yaml" -p basic.package_path:="$DATA_ROOT/map"
start_node localization "$SLAM_LIB_PATH" "$SLAM_DIR/localization/fastlio_localization_node" \
  --ros-args --params-file "$LOCALIZATION_CONFIG" --params-file "$ROBOT_PARAM_CONFIG" -p map_path:="$DATA_ROOT/map/"
# Match the 2D data flow; all geometry consumers receive the P2.5 dimensions.
start_node terrain_filter "$NAV_LIB_PATH" "$NAV_DIR/terrain_filter/terrain_filter_node" \
  --ros-args --params-file "$NAV_DIR/config/terrain_filter.yaml" --params-file "$ROBOT_PARAM_CONFIG"
start_node obstacle_stop "$NAV_LIB_PATH" "$NAV_DIR/obstacle_stop/obstacle_stop_node" \
  --ros-args --params-file "$NAV_DIR/config/obstacle_stop_p2.5_mid360s.yaml" --params-file "$ROBOT_PARAM_CONFIG"
start_node costmap_2d "$NAV_LIB_PATH" "$NAV_DIR/costmap_2d/costmap_2d_node" \
  --ros-args --params-file "$NAV_DIR/config/costmap_2d.yaml" --params-file "$ROBOT_PARAM_CONFIG"
start_node global_path_planner "$NAV_LIB_PATH" "$NAV_DIR/global_path_planner/global_path_planner_node" \
  --ros-args --params-file "$NAV_DIR/config/global_path_planner_params.yaml"
start_node path_smoother "$NAV_LIB_PATH" "$NAV_DIR/path_smoother/path_smoother_node" \
  --ros-args --params-file "$NAV_DIR/config/path_smoother.yaml"
start_node pure_pursuit "$NAV_LIB_PATH" "$NAV_DIR/pure_pursuit/pure_pursuit_node" \
  --ros-args --params-file "$NAV_DIR/config/pure_pursuit_params.yaml" --params-file "$ROBOT_PARAM_CONFIG"
start_node slam_manager "$MANAGER_LIB_PATH" "$MANAGER_DIR/slam_manager/slam_manager_node" \
  --ros-args --params-file "$MANAGER_DIR/config/slam_manager.yaml" \
  -p map_storage.root_directory:="$DATA_ROOT" -p localization.bbs.saver_executable:="$SLAM_DIR/localization/voxelmaps_saver"
start_node nav_server_console "$MANAGER_LIB_PATH" "$MANAGER_DIR/nav_server/nav_server_node" \
  "$NAV_SERVER_CONFIG"

# Give immediate loader/configuration errors time to surface, then report the
# actual child processes. Worker algorithm readiness is managed separately.
sleep 1
missing=0
for ((i=0; i<${#NODE_PIDS[@]}; i++)); do
  if kill -0 "${NODE_PIDS[i]}" 2>/dev/null; then
    echo "[RUNNING] ${NODE_NAMES[i]} (PID=${NODE_PIDS[i]})"
  else
    echo "[ERROR] required node exited: ${NODE_NAMES[i]}, see $LOG_DIR/${NODE_NAMES[i]}.log" >&2
    missing=$((missing + 1))
  fi
done
if (( missing != 0 )); then
  echo "[ERROR] $missing required node process(es) failed to start" >&2
  exit 1
fi
echo "[DONE] 全部 ${#NODE_PIDS[@]} 个节点进程已启动"
echo "[SERVICE] MID360s 2D supervisor active; algorithm activation is managed by slam_manager"
# Any required process exiting is a failed runtime. EXIT cleans up the rest;
# systemd Restart=on-failure then restarts the complete stack.
set +e
wait -n "${NODE_PIDS[@]}"
node_status=$?
set -e
for ((i=0; i<${#NODE_PIDS[@]}; i++)); do
  if ! kill -0 "${NODE_PIDS[i]}" 2>/dev/null; then
    echo "[ERROR] required node exited: ${NODE_NAMES[i]} (wait status=$node_status), see $LOG_DIR/${NODE_NAMES[i]}.log" >&2
  fi
done
exit 1
