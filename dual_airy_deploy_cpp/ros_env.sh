#!/usr/bin/env bash
# 仅读取本机驱动启动环境中的白名单变量；不读取配置、不停止或重启驱动。
dual_airy_configure_ros_env() {
  local mode="${1:-auto}" proc_root="${DUAL_AIRY_PROC_ROOT:-/proc}"
  local entry exe kv domain localhost rmw
  local -a pids=() domains=() localhosts=() rmws=()
  [[ "$mode" == auto || "$mode" == inherit ]] || { echo 'ros-env 必须为 auto 或 inherit' >&2; return 2; }
  if [[ "$mode" == auto ]]; then
    for entry in "$proc_root"/[0-9]*; do
      exe=$(readlink "$entry/exe" 2>/dev/null) || continue
      [[ "$exe" == */rslidar_sdk_node ]] || continue
      [[ -r "$entry/environ" ]] || { echo "无法读取驱动 ${entry##*/} 的启动环境；请使用相同用户运行。" >&2; return 2; }
      domain=0; localhost=0; rmw=rmw_fastrtps_cpp
      while IFS= read -r -d '' kv; do
        case "$kv" in
          ROS_DOMAIN_ID=*) domain="${kv#*=}"; domain="${domain:-0}" ;;
          ROS_LOCALHOST_ONLY=*) localhost="${kv#*=}"; localhost="${localhost:-0}" ;;
          RMW_IMPLEMENTATION=*) rmw="${kv#*=}"; rmw="${rmw:-rmw_fastrtps_cpp}" ;;
        esac
      done < "$entry/environ"
      pids+=("${entry##*/}"); domains+=("$domain"); localhosts+=("$localhost"); rmws+=("$rmw")
    done
    if (( ${#pids[@]} > 1 )); then
      echo "发现多份本机雷达驱动 PID=${pids[*]}，请保留一份再启动融合。" >&2; return 2
    fi
    if (( ${#pids[@]} == 1 )); then
      if [[ "${localhosts[0]}" != 0 ]]; then
        echo "驱动 PID=${pids[0]} 仍以 ROS_LOCALHOST_ONLY=${localhosts[0]} 运行。请在驱动终端 unset ROS_LOCALHOST_ONLY 后重启驱动。" >&2; return 2
      fi
      echo "对齐本机驱动 PID=${pids[0]}：ROS_DOMAIN_ID=${domains[0]}，ROS_LOCALHOST_ONLY=0"
      export ROS_DOMAIN_ID="${domains[0]}" RMW_IMPLEMENTATION="${rmws[0]}"
    else
      echo '未找到本机 rslidar_sdk_node，沿用当前 ROS 环境；等待四路数据，不猜测其他 domain。'
    fi
  fi
  # 按用户要求取消本机隔离；显式设 0，避免继承终端中残留的 1。
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" ROS_LOCALHOST_ONLY=0
  export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
  [[ "$ROS_DOMAIN_ID" =~ ^[0-9]+$ && ${#ROS_DOMAIN_ID} -le 3 ]] || { echo 'ROS_DOMAIN_ID 无效' >&2; return 2; }
  (( 10#$ROS_DOMAIN_ID <= 232 )) || { echo 'ROS_DOMAIN_ID 应为 0..232' >&2; return 2; }
  export ROS_DOMAIN_ID="$((10#$ROS_DOMAIN_ID))"
  [[ "$ROS_LOCALHOST_ONLY" == 0 || "$ROS_LOCALHOST_ONLY" == 1 ]] || { echo 'ROS_LOCALHOST_ONLY 应为 0 或 1' >&2; return 2; }
  [[ "$RMW_IMPLEMENTATION" == rmw_fastrtps_cpp ]] || { echo "驱动使用 $RMW_IMPLEMENTATION；本包流控需要 rmw_fastrtps_cpp，请统一驱动启动方式。" >&2; return 2; }
  # 两个阶段都用本包的固定配置，不再让检查器继承终端中另一个 XML。
  unset FASTDDS_DEFAULT_PROFILES_FILE RMW_FASTRTPS_PUBLICATION_MODE
  export FASTRTPS_DEFAULT_PROFILES_FILE="$ROOT/config/fastdds_fusion.xml"
  export RMW_FASTRTPS_USE_QOS_FROM_XML=1
  echo "统一环境：domain=$ROS_DOMAIN_ID localhost_only=$ROS_LOCALHOST_ONLY RMW=$RMW_IMPLEMENTATION"
  echo "两个阶段 DDS XML：$FASTRTPS_DEFAULT_PROFILES_FILE"
}
