#!/bin/bash
# MID360s 3D navigation: SCAN planner.

set -u
CHECK_ONLY=false
LOCALIZATION_ALGORITHM="${LOCALIZATION_ALGORITHM:-superlio}"
while (( $# )); do
  case "$1" in
    --check) CHECK_ONLY=true; shift ;;
    --localization)
      if [[ $# -lt 2 ]]; then
        echo "[ERROR] --localization requires fastlio or superlio" >&2
        exit 1
      fi
      LOCALIZATION_ALGORITHM="$2"
      shift 2
      ;;
    *) echo "Usage: bash $0 [--check] [--localization fastlio|superlio]" >&2; exit 1 ;;
  esac
done
case "$LOCALIZATION_ALGORITHM" in
  fastlio|superlio) ;;
  *) echo "[ERROR] Unsupported localization algorithm: $LOCALIZATION_ALGORITHM" >&2; exit 1 ;;
esac
LOCALIZATION_NODE="${LOCALIZATION_ALGORITHM}_localization_node"
# Configuration, executable and Worker names follow the selected algorithm.
LOCALIZATION_CONFIG_PREFIX="reloc_${LOCALIZATION_ALGORITHM//_/}"
LOCALIZATION_WORKER="/${LOCALIZATION_ALGORITHM}_localization_worker/set_state"

# Resolve payloads independently of the deployment scripts location.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Retain support for scripts copied into the OTA root.
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

LIVOX_CONFIG="$SLAM_DIR/config/MID360s_driver.yaml"
MAPPING_CONFIG="$SLAM_DIR/config/mapping_mid360s.yaml"
LIVOX_JSON_CONFIG="$SLAM_DIR/config/MID360s_config.json"
LOCALIZATION_CONFIG="$SLAM_DIR/config/${LOCALIZATION_CONFIG_PREFIX}_mid360s.yaml"
echo "[CONFIG] localization: $LOCALIZATION_ALGORITHM ($LOCALIZATION_WORKER)"
echo "[CONFIG] localization parameters: $LOCALIZATION_CONFIG"

ROBOT_PARAM_CONFIG="$MANAGER_DIR/config/p2.5_mid360s.yaml"
NAV_SERVER_DIR="$MANAGER_DIR/nav_server"
NAV_SERVER_CONFIG="$MANAGER_DIR/config/server.yaml"

# ==================================================
# ROS environment
# ==================================================
if [ ! -r /opt/ros/humble/setup.bash ]; then
  echo "[ERROR] ROS 2 Humble setup not found"
  exit 1
fi
set +u
if ! source /opt/ros/humble/setup.bash; then
  echo "[ERROR] failed to load ROS 2 Humble environment"
  exit 1
fi
set -u

# Resolve the MID360s runtime libraries from the deployed modules.
export LD_LIBRARY_PATH="$SLAM_DIR/lib:$NAV_DIR/lib:$MANAGER_DIR/lib:/opt/ros/humble/lib:${LD_LIBRARY_PATH:-}"

# MID360s uses Fast DDS defaults without an XML profile. Clear any Airy
# profile inherited from the environment before starting runtime processes.
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
unset RMW_FASTRTPS_USE_QOS_FROM_XML

# ==================================================
# Node binaries
# ==================================================
BINARIES=(
  "$SLAM_DIR/lidar/livox_ros_driver2_node"
  "$SLAM_DIR/mapping/fastlio_mapping_node"
  "$SLAM_DIR/pgo_sc/pgo_sc_node"
  "$SLAM_DIR/localization/$LOCALIZATION_NODE"

  "$NAV_DIR/global_path_planner/global_path_planner_node"
  "$NAV_DIR/path_smoother/path_smoother_node"
  "$NAV_DIR/scan_planner/scan_planner_node"
  "$NAV_DIR/obstacle_stop/obstacle_stop_node"

  "$MANAGER_DIR/slam_manager/slam_manager_node"
  "$NAV_SERVER_DIR/nav_server_node"
  "$SLAM_DIR/localization/voxelmaps_saver"
)

# ==================================================
# Required configuration files
# ==================================================
CONFIG_FILES=(
  "$LIVOX_CONFIG"
  "$MAPPING_CONFIG"
  "$LIVOX_JSON_CONFIG"
  "$LOCALIZATION_CONFIG"
  "$ROBOT_PARAM_CONFIG"
  "$NAV_SERVER_CONFIG"
  "$MANAGER_DIR/config/slam_manager.yaml"
  "$SLAM_DIR/config/pgo_mid360s.yaml"
  "$NAV_DIR/config/global_path_planner_params.yaml"
  "$NAV_DIR/config/path_smoother.yaml"
  "$NAV_DIR/config/scan_planner_params.yaml"
  "$NAV_DIR/config/obstacle_stop_p2.5_mid360s.yaml"
)

# ==================================================
# Check binaries and execution permission
# ==================================================
echo "========================================"
echo "[CHECK] checking executable files..."
echo "========================================"

check_error=0

for binary in "${BINARIES[@]}"; do
  if [ ! -f "$binary" ]; then
    echo "[ERROR] binary not found:"
    echo "        $binary"
    check_error=1
    continue
  fi

  if [ ! -x "$binary" ]; then
    echo "[ERROR] binary is not executable:"
    echo "        $binary"
    check_error=1
    continue
  fi

  echo "[OK] $binary"
done

# ==================================================
# Check configuration files
# ==================================================
echo "========================================"
echo "[CHECK] checking configuration files..."
echo "========================================"

for config_file in "${CONFIG_FILES[@]}"; do
  if [ ! -f "$config_file" ]; then
    echo "[ERROR] config not found:"
    echo "        $config_file"
    check_error=1
    continue
  fi

  if [ ! -r "$config_file" ]; then
    echo "[ERROR] config is not readable:"
    echo "        $config_file"
    check_error=1
    continue
  fi

  echo "[OK] $config_file"
done

if [ "$check_error" -ne 0 ]; then
  echo "========================================"
  echo "[ERROR] runtime file check failed"
  echo "========================================"
  exit 1
fi

echo "[DONE] all runtime files are ready"
if "$CHECK_ONLY"; then
  exit 0
fi

# ==================================================
# Node process names
# Do not stop slam_manager_test_gui.
# ==================================================
NODE_PATTERNS=(
  "livox_ros_driver2_node"
  "rslidar_sdk_node"
  "dual_airy_fusion_node"
  "woodox_imu_bridge_node"
  "fastlio_mapping_node"
  "pgo_sc_node"
  "fastlio_localization_node"
  "superlio_localization_node"
  "global_path_planner_node"
  "path_smoother_node"
  "scan_planner_node"
  "obstacle_stop_node"
  "slam_manager_node"
  "nav_server_node"
)

# ==================================================
# Stop all runtime nodes
# ==================================================
stop_nodes()
{
  echo "========================================"
  echo "[STOP] stopping runtime nodes..."
  echo "========================================"

  for node in "${NODE_PATTERNS[@]}" terrain_filter_node costmap_2d_node pure_pursuit_node; do
    if pgrep -f "$node" >/dev/null 2>&1; then
      echo "[STOP] SIGINT: $node"
      pkill -INT -f "$node" 2>/dev/null || true
    fi
  done

  sleep 3

  for node in "${NODE_PATTERNS[@]}" terrain_filter_node costmap_2d_node pure_pursuit_node; do
    if pgrep -f "$node" >/dev/null 2>&1; then
      echo "[STOP] SIGTERM: $node"
      pkill -TERM -f "$node" 2>/dev/null || true
    fi
  done

  sleep 2

  for node in "${NODE_PATTERNS[@]}" terrain_filter_node costmap_2d_node pure_pursuit_node; do
    if pgrep -f "$node" >/dev/null 2>&1; then
      echo "[STOP] SIGKILL: $node"
      pkill -KILL -f "$node" 2>/dev/null || true
    fi
  done

  echo "[STOP] runtime nodes stopped"
}

# ==================================================
# Clean up nodes when systemd stops the service
# ==================================================
cleanup()
{
  trap - INT TERM EXIT

  echo ""
  echo "[SIGNAL] stop signal received"

  stop_nodes
  exit 0
}

trap cleanup INT TERM

# ==================================================
# Stop previous runtime nodes
# ==================================================
echo "[CLEAN] stopping old runtime nodes..."
stop_nodes

for node in "${NODE_PATTERNS[@]}"; do
  if pgrep -f "$node" >/dev/null 2>&1; then
    echo "[WARN] still running: $node"
    pgrep -af "$node" || true
  fi
done

# ==================================================
# Log directory
# ==================================================
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

SESSION_TIME="$(date '+%Y%m%d_%H%M%S')"
DATA_ROOT="/home/jw/ota/data/slam_data"
LOG_ROOT="$DATA_ROOT/log"
LOG_DIR="$LOG_ROOT/runtime/$SESSION_TIME"
ROS_LOG_DIR="$LOG_DIR/ros"

RUNTIME_LOG_PATH_FILE="$DATA_ROOT/current_runtime_log.txt"

if ! mkdir -p "$ROS_LOG_DIR"; then
  echo "[ERROR] cannot create log directory:"
  echo "        $ROS_LOG_DIR"
  exit 1
fi

if [ ! -w "$LOG_DIR" ]; then
  echo "[ERROR] log directory is not writable:"
  echo "        $LOG_DIR"
  exit 1
fi

if [ ! -w "$ROS_LOG_DIR" ]; then
  echo "[ERROR] ROS log directory is not writable:"
  echo "        $ROS_LOG_DIR"
  exit 1
fi

# Keep five distinct sessions, oldest first; retries of the same session reuse its entry.
RUNTIME_LOG_HISTORY_SOURCE="$RUNTIME_LOG_PATH_FILE"
if [[ ! -f "$RUNTIME_LOG_HISTORY_SOURCE" ]]; then
  RUNTIME_LOG_HISTORY_SOURCE=/dev/null
fi
RUNTIME_LOG_HISTORY_TEMP="$(mktemp "${RUNTIME_LOG_PATH_FILE}.XXXXXX")" || exit 1
if ! awk -v latest="$LOG_DIR" '
  {
    path = $0
    sub(/^\[[^]]*\][[:space:]]*/, "", path)
    if (path != "" && path != latest) paths[++count] = path
  }
  END {
    paths[++count] = latest
    first = count > 5 ? count - 4 : 1
    for (i = first; i <= count; ++i) {
      label = i == count ? " 最新" : ""
      printf "[第%d次%s] %s\n", i - first + 1, label, paths[i]
    }
  }
' "$RUNTIME_LOG_HISTORY_SOURCE" > "$RUNTIME_LOG_HISTORY_TEMP" ||
   ! chmod 664 "$RUNTIME_LOG_HISTORY_TEMP" ||
   ! mv -f -- "$RUNTIME_LOG_HISTORY_TEMP" "$RUNTIME_LOG_PATH_FILE"; then
  rm -f -- "$RUNTIME_LOG_HISTORY_TEMP"
  echo "[ERROR] cannot update runtime log history: $RUNTIME_LOG_PATH_FILE" >&2
  exit 1
fi
unset RUNTIME_LOG_HISTORY_SOURCE RUNTIME_LOG_HISTORY_TEMP
export ROS_LOG_DIR

echo "========================================"
echo "[LOG] $LOG_DIR"
echo "[DDS] RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<unset>}"
echo "[DDS] FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-<unset>}"
echo "========================================"

# ==================================================
# MID360s (LiDAR and built-in IMU)
# Override the relative JSON path in the deployed driver YAML.
# ==================================================
echo "[START] mid360s livox_ros_driver2"

stdbuf -oL -eL \
  "$SLAM_DIR/lidar/livox_ros_driver2_node" \
  --ros-args \
  --params-file "$LIVOX_CONFIG" \
  -p user_config_path:="$LIVOX_JSON_CONFIG" \
  2>&1 | tee -a "$LOG_DIR/livox_ros_driver2.log" &

sleep 2

# ==================================================
# FAST-LIO (MID360s)
# ==================================================
echo "[START] fastlio"

stdbuf -oL -eL \
  "$SLAM_DIR/mapping/fastlio_mapping_node" \
  --ros-args \
  --params-file "$MAPPING_CONFIG" \
  --params-file "$ROBOT_PARAM_CONFIG" \
  2>&1 | tee -a "$LOG_DIR/fastlio_mapping.log" &

sleep 2

# ==================================================
# PGO
# ==================================================
echo "[START] pgo"

stdbuf -oL -eL \
  "$SLAM_DIR/pgo_sc/pgo_sc_node" \
  --ros-args \
  --params-file "$SLAM_DIR/config/pgo_mid360s.yaml" \
  -p basic.package_path:="/home/jw/ota/data/slam_data/map" \
  2>&1 | tee -a "$LOG_DIR/pgo_sc.log" &

sleep 2

# ==================================================
# Localization
# Use the MID360s localization profile.
# ==================================================
echo "[START] localization"

stdbuf -oL -eL \
  "$SLAM_DIR/localization/$LOCALIZATION_NODE" \
  --ros-args \
  -p runtime_profile:=platform \
  --params-file "$LOCALIZATION_CONFIG" \
  -p map_path:="/home/jw/ota/data/slam_data/map/" \
  --params-file "$ROBOT_PARAM_CONFIG" \
  2>&1 | tee -a "$LOG_DIR/localization.log" &

sleep 2

# ==================================================
# Global Path Planner
# ==================================================
echo "[START] global_path_planner"

stdbuf -oL -eL \
  "$NAV_DIR/global_path_planner/global_path_planner_node" \
  --ros-args \
  --params-file "$NAV_DIR/config/global_path_planner_params.yaml" \
  2>&1 | tee -a "$LOG_DIR/global_path_planner.log" &

sleep 1

# ==================================================
# Path Smoother
# ==================================================
echo "[START] path_smoother"

stdbuf -oL -eL \
  "$NAV_DIR/path_smoother/path_smoother_node" \
  --ros-args \
  --params-file "$NAV_DIR/config/path_smoother.yaml" \
  2>&1 | tee -a "$LOG_DIR/path_smoother.log" &

sleep 1

# ==================================================
# SCAN Planner
# ==================================================
echo "[START] scan_planner"

stdbuf -oL -eL \
  "$NAV_DIR/scan_planner/scan_planner_node" \
  --ros-args \
  --params-file "$NAV_DIR/config/scan_planner_params.yaml" \
  2>&1 | tee -a "$LOG_DIR/scan_planner.log" &

sleep 1

# ==================================================
# Obstacle Stop
# ==================================================
echo "[START] obstacle_stop"

stdbuf -oL -eL \
  "$NAV_DIR/obstacle_stop/obstacle_stop_node" \
  --ros-args \
  --params-file "$NAV_DIR/config/obstacle_stop_p2.5_mid360s.yaml" \
  --params-file "$ROBOT_PARAM_CONFIG" \
  2>&1 | tee -a "$LOG_DIR/obstacle_stop.log" &

sleep 1

# ==================================================
# SLAM Manager
# ==================================================
echo "[START] slam_manager"

stdbuf -oL -eL \
  "$MANAGER_DIR/slam_manager/slam_manager_node" \
  --ros-args \
  --params-file "$MANAGER_DIR/config/slam_manager.yaml" \
  --params-file "$ROBOT_PARAM_CONFIG" \
  -p localization.worker_service:="$LOCALIZATION_WORKER" \
  -p localization.bbs.saver_executable:="$SLAM_DIR/localization/voxelmaps_saver" \
  2>&1 | tee -a "$LOG_DIR/slam_manager.log" &

sleep 2

# ==================================================
# Navigation Server
# nav_server_node first argument must be server.yaml.
# ==================================================
echo "[START] nav_server"

stdbuf -oL -eL \
  "$NAV_SERVER_DIR/nav_server_node" \
  "$NAV_SERVER_CONFIG" \
  2>&1 | tee -a "$LOG_DIR/nav_server_console.log" &

sleep 2

# ==================================================
# Check node status
# ==================================================
missing=0

echo "========================================"
echo "[CHECK] runtime node status"
echo "========================================"

for node in "${NODE_PATTERNS[@]}"; do
  # Both localization algorithms are cleaned up, but only the selected one runs.
  [[ "$node" == *_localization_node && "$node" != "$LOCALIZATION_NODE" ]] && continue
  # Other sensor drivers remain in the cleanup list only.
  [[ "$node" == "dual_airy_fusion_node" || "$node" == "rslidar_sdk_node" || "$node" == "woodox_imu_bridge_node" ]] && continue
  if pgrep -f "$node" >/dev/null 2>&1; then
    echo "[RUNNING] $node"
  else
    echo "[WARN] NOT running: $node"
    missing=$((missing + 1))
  fi
done

if [ "$missing" -eq 0 ]; then
  echo "========================================"
  echo "[DONE] all runtime nodes are running"
  echo "========================================"
else
  echo "========================================"
  echo "[WARN] $missing runtime node(s) are not running"
  echo "========================================"
fi

# ==================================================
# Keep the service running and supervise its nodes
# ==================================================
echo "[SERVICE] runtime supervisor is active"

while true; do
  sleep 10

  running=0

  for node in "${NODE_PATTERNS[@]}"; do
    [[ "$node" == *_localization_node && "$node" != "$LOCALIZATION_NODE" ]] && continue
    # Other sensor drivers remain in the cleanup list only.
    [[ "$node" == "dual_airy_fusion_node" || "$node" == "rslidar_sdk_node" || "$node" == "woodox_imu_bridge_node" ]] && continue
    if pgrep -f "$node" >/dev/null 2>&1; then
      running=$((running + 1))
    fi
  done

  if [ "$running" -eq 0 ]; then
    echo "[ERROR] all runtime nodes have stopped"
    exit 1
  fi
done
