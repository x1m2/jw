# uwb_follow_planner

Generates a dynamic person-following `PathWithPolicy` from LibAoa UWB
range/bearing measurements and FAST-LIO2 mapping odometry.

## Data flow

```text
/libAoa_robot_publisher + /Odometry (nav_msgs/msg/Odometry, frame=odom)
  + /ble_follow_cmd (std_msgs/msg/Int8: 0=start, 1=stop, 3=pause)
  -> uwb_follow_planner
  -> /global_path_with_policy
  -> path_smoother
  -> pure_pursuit_local_planner
```

Do not run `global_path_planner` concurrently in follow mode: both nodes use
`/global_path_with_policy` as their default output.

## UWB message

The planner subscribes to the UWB driver's existing
`uwb_aoa_pkg/msg/LibAoaRobotMsg` type and uses these fields:

```text
float64 r
float64 a
```

The `uwb_aoa_pkg` interface package must be installed or included in the
workspace when this planner is built. ROS 2 does not connect messages merely
because their fields match; the package and message type must also match.

`r` is the target distance (metres by default). `a` is always in degrees: 0 is
straight ahead, counter-clockwise is positive up to 180, and clockwise is
negative down to -180. In the robot frame the conversion is
`x = r * cos(a)`, `y = r * sin(a)`, so positive angles are on the robot's left.
Distance scale, mounting offsets, filtering, hysteresis, timeouts, and follow
distance are configurable in `config/uwb_follow_planner.yaml`.

`/ble_follow_cmd` is an event command rather than a heartbeat. The last command
is latched internally when the publisher is silent. The planner starts in the
stopped state and requires `0` before following; `1` stops and clears the
locked control target, while `3` pauses without clearing the target so a later
`0` can resume it. Other command values are ignored.

## Outputs

- `/global_path_with_policy`: dynamic path consumed by the existing smoother;
  obstacle stop is enabled and active local avoidance follows the configured
  policy.
- `/uwb_follow/person_pose`: filtered person position in `odom`.
- `/uwb_follow/person_trail`: filtered breadcrumb trail for RViz.
- `/uwb_follow/state`: `WAITING_FOR_POSE`, `WAITING_FOR_UWB`, `HOLD`,
  `FOLLOWING`, `TARGET_LOST`, `COMMAND_STOPPED`, or `COMMAND_PAUSED`.
- `/uwb_follow/target_valid`: freshness/validity of the current UWB target.

The planner requests `/pure_pursuit/set_paused` while holding or when UWB or
FAST-LIO odometry is stale, so the controller cannot continue an old path.

The default input is the mapping node's `/Odometry`, not
`/pose_stamped_localization`. In UWB follow mode, configure the path smoother,
Pure Pursuit, and local costmap to use `odom` as their global frame. `pgo_sc`,
`fastlio_localization`, and `global_path_planner` are not required.

During local avoidance, the planner keeps receiving and filtering UWB data but
stops publishing reference paths. After the robot reaches the second side goal,
`pure_pursuit_local_planner` publishes `WAITING_DYNAMIC_PATH`; the planner then
publishes a fresh path based on the latest person position. Delayed
dynamic paths generated before this state are rejected by the local planner.

During normal obstacle-free following, the planner locks a target on the line
from the robot to the latest filtered person position while preserving
`follow_distance`. It replaces that target when the robot-to-person direction
differs from the robot-to-target direction by more than
`target_angle_update_threshold_degrees`, or when the new candidate target has
moved farther than `target_position_update_threshold`. Person history remains
available on `/uwb_follow/person_trail` for visualization, but it no longer
determines the control target. While avoidance owns motion, the locked target is
discarded; the first path after avoidance therefore uses the latest person
position.

## Build and run

```bash
colcon build --packages-select navigation_interfaces uwb_follow_planner
source install/setup.bash
ros2 launch uwb_follow_planner uwb_follow_planner.launch.py
```

For the complete mapping-time follow workflow, run
`src/run_uwb_follow.sh` or click `开始跟随` in
`src/navigation_workflow_gui.py`. That workflow starts Livox, FAST-LIO2,
obstacle stop, the `odom` local costmap, path smoother, Pure Pursuit, and this
planner; it deliberately does not start `pgo_sc` or `global_path_planner`.
RViz starts by default with the UWB follow view. For a headless run, use
`UWB_USE_RVIZ=false src/run_uwb_follow.sh`, or pass `launch_rviz:=false` when
launching the planner directly.
