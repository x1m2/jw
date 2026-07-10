#!/bin/bash
# kill_robot_ros2_nodes.sh
# 用于停止 run_slam.sh / run_edit_map.sh / run_relocalization.sh / run_nav.sh 启动的 ROS2 节点和相关 Python 脚本

set +e

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "${SCRIPT_PATH}")"
WORKSPACE_DIR="$(dirname "${SCRIPT_DIR}")"

if ! source "${SCRIPT_DIR}/load_robot_env.sh"; then
    exit 1
fi

LOG_ROOT_BASE="${ROBOT_DATA_DIR}/log"

SELF_PID="$$"

# 是否杀 RViz。默认杀，因为 topology_tool.launch.py 通常会启动 RViz。
KILL_RVIZ=${KILL_RVIZ:-1}

# 是否杀导航控制 GUI。默认不杀，避免把当前主 GUI 也关掉。
KILL_NAV_GUI=${KILL_NAV_GUI:-0}

# 是否杀 ros2 bag record。默认杀。
KILL_BAG=${KILL_BAG:-1}

print_header() {
    echo "=================================================="
    echo "$1"
    echo "=================================================="
}

is_alive() {
    local pid="$1"
    [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null
}

kill_pid_gracefully() {
    local pid="$1"
    local name="$2"

    if [ -z "$pid" ]; then
        return
    fi

    if [ "$pid" = "$SELF_PID" ]; then
        return
    fi

    if ! is_alive "$pid"; then
        return
    fi

    echo "[INFO] stopping ${name}, PID=${pid}"

    # 优先杀进程组。如果该 pid 不是组长，kill -PID 会失败，再杀单进程。
    kill -SIGINT "-${pid}" 2>/dev/null || kill -SIGINT "${pid}" 2>/dev/null || true
    sleep 0.3

    if is_alive "$pid"; then
        kill -SIGTERM "-${pid}" 2>/dev/null || kill -SIGTERM "${pid}" 2>/dev/null || true
        sleep 0.3
    fi

    if is_alive "$pid"; then
        echo "[WARN] ${name} still alive, force killing PID=${pid}"
        kill -SIGKILL "-${pid}" 2>/dev/null || kill -SIGKILL "${pid}" 2>/dev/null || true
    fi
}

kill_by_pattern() {
    local pattern="$1"
    local name="$2"

    local pids
    pids=$(pgrep -f "$pattern" 2>/dev/null | sort -u)

    if [ -z "$pids" ]; then
        return
    fi

    echo ""
    echo "[INFO] matched ${name}: ${pattern}"

    for pid in $pids; do
        # 跳过自己、父进程，防止误杀当前清理脚本
        if [ "$pid" = "$SELF_PID" ] || [ "$pid" = "$PPID" ]; then
            continue
        fi
        kill_pid_gracefully "$pid" "$name"
    done
}

kill_from_pid_files() {
    print_header "[STEP 1] Kill processes recorded in pid files"

    if [ ! -d "$LOG_ROOT_BASE" ]; then
        echo "[INFO] log root not found: $LOG_ROOT_BASE"
        return
    fi

    local pid_files
    pid_files=$(find "$LOG_ROOT_BASE" -type f -name "*.pid" 2>/dev/null | sort -u)

    if [ -z "$pid_files" ]; then
        echo "[INFO] no pid files found"
        return
    fi

    for f in $pid_files; do
        local pid
        pid=$(cat "$f" 2>/dev/null | tr -cd '0-9')
        if [ -n "$pid" ]; then
            kill_pid_gracefully "$pid" "pid-file: $f"
        fi
        rm -f "$f" 2>/dev/null || true
    done
}

kill_known_launches() {
    print_header "[STEP 2] Kill known ros2 launch processes"
    # Livox ROS Driver2
    kill_by_pattern "ros2 launch livox_ros_driver2 msg_MID360s_launch.py" "livox_ros_driver2 launch"

    # FAST-LIO
    kill_by_pattern "ros2 launch fast_lio mapping.launch.py" "fast_lio launch"
    
    # run_slam.sh
    kill_by_pattern "ros2 launch pgo_sc run.launch.py" "pgo_sc launch"
    kill_by_pattern "ros2 launch topology_map_generator topology_map_generator.launch.py" "topology_map_generator launch"

    # run_edit_map.sh
    kill_by_pattern "ros2 launch topology_rviz_plugin topology_connect.launch.py" "topology_connect launch"
    kill_by_pattern "ros2 launch topology_rviz_plugin topology_tool.launch.py" "topology_tool launch"

    # run_relocalization.sh
    kill_by_pattern "ros2 launch pcd2pgm pcd2pgm_launch.py" "pcd2pgm launch"
    kill_by_pattern "ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan.launch.py" "pointcloud_to_laserscan launch"
    kill_by_pattern "ros2 launch localization_sc run.launch.py" "localization_sc launch"

    # run_nav.sh
    kill_by_pattern "ros2 launch costmap_2d costmap_2d.launch.py" "costmap_2d launch"
    kill_by_pattern "ros2 launch global_path_planner global_path_planner.launch.py" "global_path_planner launch"
    kill_by_pattern "ros2 launch path_smoother path_smoother.launch.py" "path_smoother launch"
    kill_by_pattern "ros2 launch pure_pursuit_local_planner pure_pursuit_local_planner.launch.py" "pure_pursuit launch"
    kill_by_pattern "ros2 launch dual_radar_traversable_extractor dual_radar_extractor.launch.py" "dual_radar_extractor launch"
    kill_by_pattern "ros2 launch topology_rviz_plugin target_points_visualizer.launch.py" "target_points_visualizer launch"
    kill_by_pattern "ros2 launch user_interface navigation_task_state_machine.launch.py" "navigation_state_machine launch"
}

kill_known_python_scripts() {
    print_header "[STEP 3] Kill known Python helper scripts"

    kill_by_pattern "keyboard_control.py" "keyboard_control.py"
    kill_by_pattern "livox_custom_to_pointcloud2.py" "livox_custom_to_pointcloud2.py"
    kill_by_pattern "wait_reloc_success_" "wait_reloc_success"

    if [ "$KILL_NAV_GUI" = "1" ]; then
        kill_by_pattern "navigation_control_gui.py" "navigation_control_gui.py"
    else
        echo "[INFO] skip navigation_control_gui.py; set KILL_NAV_GUI=1 to kill it"
    fi
}

kill_node_executables() {
    print_header "[STEP 4] Kill common ROS2 node executables"

    # 这里是兜底：有些 ros2 launch 的子节点命令行不一定包含 launch 文件名。
    # pattern 尽量写具体，避免误杀无关程序。
    kill_by_pattern "livox_ros_driver2_node" "livox_ros_driver2_node executable"
    kill_by_pattern "livox_lidar_publisher" "livox_lidar_publisher node"
    kill_by_pattern "imu_filter_madgwick" "imu_filter_madgwick executable"
    kill_by_pattern "imu_filter_madgwick_node" "imu_filter_madgwick_node executable"
    kill_by_pattern "pgo_sc" "pgo_sc executable"
    kill_by_pattern "topology_map_generator" "topology_map_generator executable"
    kill_by_pattern "topology_connect" "topology_connect executable"
    kill_by_pattern "pcd2pgm" "pcd2pgm executable"
    kill_by_pattern "localization_sc" "localization_sc executable"
    kill_by_pattern "pointcloud_to_laserscan" "pointcloud_to_laserscan executable"
    kill_by_pattern "costmap_2d" "costmap_2d executable"
    kill_by_pattern "global_path_planner" "global_path_planner executable"
    kill_by_pattern "path_smoother" "path_smoother executable"
    kill_by_pattern "pure_pursuit" "pure_pursuit executable"
    kill_by_pattern "dual_radar" "dual_radar executable"
    kill_by_pattern "target_points_visualizer" "target_points_visualizer executable"
    kill_by_pattern "navigation_task_state_machine" "navigation_task_state_machine executable"

    if [ "$KILL_RVIZ" = "1" ]; then
        kill_by_pattern "rviz2" "rviz2"
    else
        echo "[INFO] skip rviz2; set KILL_RVIZ=1 to kill it"
    fi

    if [ "$KILL_BAG" = "1" ]; then
        kill_by_pattern "ros2 bag record" "ros2 bag record"
    else
        echo "[INFO] skip ros2 bag record; set KILL_BAG=1 to kill it"
    fi
}

show_remaining_ros2_nodes() {
    print_header "[STEP 5] Remaining ROS2 nodes"

    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash 2>/dev/null || true
    fi

    if [ -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
        source "${WORKSPACE_DIR}/install/setup.bash" 2>/dev/null || true
    fi

    if command -v ros2 >/dev/null 2>&1; then
        timeout 5s ros2 node list 2>/dev/null || echo "[INFO] ros2 node list timeout or no daemon"
    else
        echo "[INFO] ros2 command not found"
    fi
}

main() {
    print_header "Kill robot ROS2 workflow nodes"
    echo "[INFO] KILL_RVIZ=${KILL_RVIZ}"
    echo "[INFO] KILL_NAV_GUI=${KILL_NAV_GUI}"
    echo "[INFO] KILL_BAG=${KILL_BAG}"

    kill_from_pid_files
    kill_known_launches
    kill_known_python_scripts
    kill_node_executables

    sleep 1
    show_remaining_ros2_nodes

    print_header "Done"
}

main "$@"
