#!/bin/bash
# Replace payloads atomically so running nodes can be updated before a service restart.
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -z "${OTA_ROOT:-}" ]]; then
  if [[ -d "$SCRIPT_DIR/slam/pack/content" ]]; then
    OTA_ROOT="$SCRIPT_DIR"
  else
    OTA_ROOT="/home/jw/ota"
  fi
fi
INSTALL_DIR="/home/jw/robot_ws/install"
DRY_RUN=false
SOURCE_SET=false

usage() {
  echo "Usage: bash $0 [install_directory] [--dry-run]"
  echo "Default install directory: /home/jw/robot_ws/install"
  echo "OTA directory: $OTA_ROOT (override with OTA_ROOT=/path/to/ota)"
}

for arg in "$@"; do
  case "$arg" in
    --dry-run) DRY_RUN=true ;;
    -h|--help) usage; exit 0 ;;
    -*) usage >&2; exit 1 ;;
    *)
      if "$SOURCE_SET"; then usage >&2; exit 1; fi
      INSTALL_DIR="$arg"
      SOURCE_SET=true
      ;;
  esac
done

if [[ ! -d "$INSTALL_DIR" ]]; then
  echo "[ERROR] install directory not found: $INSTALL_DIR" >&2
  exit 1
fi
INSTALL_DIR="$(cd -- "$INSTALL_DIR" && pwd)"
if [[ ! -d "$OTA_ROOT" ]]; then
  echo "[ERROR] OTA directory not found: $OTA_ROOT" >&2
  exit 1
fi
OTA_ROOT="$(cd -- "$OTA_ROOT" && pwd)"
for module in nav slam slam_manager; do
  if [[ ! -d "$OTA_ROOT/$module/pack/content" ]]; then
    echo "[ERROR] OTA module not found: $module" >&2
    exit 1
  fi
done

sources=()
targets=()
missing=0
identical=0

# The bundled BBS build installs these beside the localization package. They
# may not exist in an older OTA payload, so replacement-only scanning misses them.
for filename in libcpu_bbs3d.so libopencl_bbs3d.so; do
  source="$INSTALL_DIR/fastlio_localization/lib/$filename"
  target="$OTA_ROOT/slam/pack/content/lib/$filename"
  if [[ -f "$source" && ! -e "$target" ]]; then
    if [[ -L "$target" ]]; then
      echo "[ERROR] Refusing to overwrite OTA symlink: $target" >&2
      exit 1
    fi
    sources+=("$source")
    targets+=("$target")
  fi
done

# Build the complete replacement list before copying. Multiple source matches
# must contain identical bytes, otherwise stop without changing any payload.
for module in nav slam slam_manager; do
  content="$OTA_ROOT/$module/pack/content"
  while IFS= read -r -d '' target; do
    relative="${target#"$content/"}"
    component="${relative%%/*}"
    filename="${target##*/}"
    candidates=()
    case "$component" in
      management) continue ;;
      config)
        # 暂停 YAML 配置更新，保留 OTA 中的现有配置；XML 仍正常更新。
        # case "$filename" in *.yaml|*.yml|*.xml) ;; *) continue ;; esac
        case "$filename" in *.xml) ;; *) continue ;; esac
        while IFS= read -r -d '' candidate; do
          candidates+=("$candidate")
        done < <(find -L "$INSTALL_DIR" -type f -path '*/share/*/config/*' -name "$filename" -print0)
        # install uses params.yaml; OTA uses the descriptive filename.
        # alias="$INSTALL_DIR/woodox_imu_bridge/share/woodox_imu_bridge/config/params.yaml"
        # if [[ ${#candidates[@]} -eq 0 && "$filename" == woodox_imu_bridge.yaml && -f "$alias" ]]; then
        #   candidates+=("$alias")
        # fi
        ;;
      lib)
        # Match the BBS pair linked into this localization build, even if a
        # separately installed bbs3d package contains another build of the libs.
        case "$filename" in
          libcpu_bbs3d.so|libopencl_bbs3d.so)
            candidate="$INSTALL_DIR/fastlio_localization/lib/$filename"
            if [[ -f "$candidate" ]]; then
              candidates+=("$candidate")
            fi
            ;;
        esac
        if [[ ${#candidates[@]} -eq 0 ]]; then
          while IFS= read -r -d '' candidate; do
            candidates+=("$candidate")
          done < <(find -L "$INSTALL_DIR" -type f -path '*/lib/*' -name "$filename" -print0)
        fi
        ;;
      *)
        case "$component" in
          mapping) package=fast_lio ;;
          localization) package=fastlio_localization ;;
          imu) package=woodox_imu_bridge ;;
          lidar)
            case "$filename" in
              dual_airy_fusion_node) package=dual_airy_fusion ;;
              livox_ros_driver2_node) package=livox_ros_driver2 ;;
              *) package=rslidar_sdk ;;
            esac
            ;;
          *) package="$component" ;;
        esac
        candidate="$INSTALL_DIR/$package/lib/$package/$filename"
        if [[ -f "$candidate" ]]; then candidates+=("$candidate"); fi
        ;;
    esac
    label="${target#"$OTA_ROOT/"}"
    if [[ ${#candidates[@]} -eq 0 ]]; then
      echo "[KEEP] $label (not in install)"
      missing=$((missing + 1))
      continue
    fi
    source="${candidates[0]}"
    for candidate in "${candidates[@]}"; do
      if ! cmp -s -- "$source" "$candidate"; then
        echo "[ERROR] Different install files match $label:" >&2
        printf '  %s\n' "${candidates[@]}" >&2
        exit 1
      fi
    done
    if cmp -s -- "$source" "$target"; then
      identical=$((identical + 1))
      continue
    fi
    if [[ -L "$target" ]]; then
      echo "[ERROR] Refusing to overwrite OTA symlink: $target" >&2
      exit 1
    fi
    sources+=("$source")
    targets+=("$target")
  done < <(find "$content" \( -type f -o -type l \) -print0)
done

copy_target=""
trap 'if [[ -n "$copy_target" ]]; then rm -f -- "$copy_target"; fi' EXIT

for ((i=0; i<${#targets[@]}; i++)); do
  echo "[UPDATE] ${targets[i]#"$OTA_ROOT/"} <- ${sources[i]}"
  if ! "$DRY_RUN"; then
    # Keep the old inode valid for running binaries and loaded shared libraries.
    # Same-directory rename installs the complete file without ETXTBSY/truncation.
    mkdir -p -- "$(dirname -- "${targets[i]}")"
    copy_target="$(mktemp "${targets[i]}.update.XXXXXX")"
    cp -- "${sources[i]}" "$copy_target"
    if [[ -e "${targets[i]}" ]]; then
      chmod --reference="${targets[i]}" "$copy_target"
    else
      chmod 644 "$copy_target"
    fi
    mv -fT -- "$copy_target" "${targets[i]}"
    copy_target=""
  fi
done

echo "[DONE] updates=${#targets[@]} identical=$identical missing/kept=$missing dry-run=$DRY_RUN"
