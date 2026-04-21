# SLAM Integration Design — WALL-E

**Date:** 2026-04-20
**Author:** Julian (with Claude Opus 4.7)
**Status:** Design approved; ready for implementation plan
**Branch:** `oldcodetest`
**Baseline commit:** `82e4ad4f`

## 1. Purpose & Scope

Add `slam_toolbox`-based 2D lidar SLAM to WALL-E. Primary use case is
indoor/bounded-area testing with a stable `map` frame that does not
drift as the robot moves. Secondary use case is outdoor operation
with GPS available for logging only (no GPS fusion into the SLAM map
in Phase 1).

This is a retry of the 2026-04-14 attempt, which was reverted
because installing `ros-jazzy-slam-toolbox` caused a libfastcdr /
libfastrtps ABI break that crashed every ROS node with
`undefined symbol: _ZN8eprosima7fastcdr3Cdr9serializeEPc`. See
`~/.claude/projects/-home-julian-WALL-E/memory/project_slam_fastcdr.md`.

### In scope (Phase 1)

- `ros-jazzy-slam-toolbox` installed cleanly in the Docker image.
- `async_slam_toolbox_node` launched in `mapping` mode.
- SLAM owns `map→odom` TF whenever on; existing static-TF and map-EKF
  publishers are gated off.
- Throwaway maps: no save/load, no localization mode.
- `launch_slam` exposed as a top-level arg, default `true` so
  `./start_docker.sh -s` includes SLAM out of the box.
- Stable `map` frame for Nav2 and RViz goal poses; `/map` occupancy
  grid published but NOT added to Nav2 costmap as a static layer
  (Phase 2 item).

### Explicitly out of scope (Phase 1)

- Saving or loading maps (no `map_saver`, no `.posegraph` files).
- Localization-only mode (`mode: localization`).
- GPS fusion into the SLAM pose graph. When `launch_slam=true` and
  `launch_gps=true`, the GPS driver publishes `/fix` but
  `navsat_transform` and `ekf_filter_node_map` are disabled, so the
  SLAM map has no absolute georeference.
- Static layer of the SLAM map in Nav2's global costmap (current
  `nav2_no_map_params.yaml` stays as-is).
- Changes to `twist_mux`, collision_monitor, RoboClaw, or any part
  of the cmd_vel chain.

## 2. Success Criteria

All three gates must pass sequentially on hardware. Gate 1 is the
ABI check that was the prior attempt's failure point.

**Gate 1 — Image builds with intact ABI:**
- `ldd /opt/ros/jazzy/lib/libstd_msgs__rosidl_typesupport_fastrtps_c.so | grep fastcdr`
  resolves to a versioned `libfastcdr.so.*` with no `not found`.
- `ros2 node list` returns without symbol lookup errors.
- `ros2 pkg list | grep slam_toolbox` shows the package.

**Gate 2 — SLAM comes up without breaking the baseline:**
- `./start_docker.sh -s` on the Jetson: no `process has died` or
  exit-127 errors for any node.
- `ros2 topic list` shows `/map` and `/slam_toolbox/graph_visualization`.
- `ros2 topic echo /tf --once` shows `map→odom` TF with exactly one
  publisher (slam_toolbox).
- RViz on the host (Fixed Frame: `map`) renders the TF tree and a
  live occupancy grid.

**Gate 3 — Indoor SLAM quality:**
- Teleop for 2-5 minutes: map frame does NOT rotate with the robot
  (the original complaint this work fixes).
- Returning to a previously-visited pose triggers a visible loop
  closure in `/slam_toolbox/graph_visualization`; map snaps to
  correct accumulated drift.
- Nav2 2D Goal to a previously-visited spot plans and drives
  without `TF lookup failed` errors.
- No spurious loop closures causing the map to jump on repetitive
  indoor geometry (identical hallways, doorways). If observed,
  tighten `loop_match_minimum_response_fine` from 0.45 to 0.55.

## 3. Architecture

### 3.1 TF ownership rule

`map→odom` must have exactly one publisher. The launch system
enforces mutual exclusion via conditions on each candidate publisher:

| `launch_slam` | `launch_gps` | static identity TF | map EKF | navsat_transform | slam_toolbox |
|---|---|---|---|---|---|
| false | false | ON | off | off | off |
| false | true  | off | ON  | ON  | off |
| true  | false | off | off | off | ON  |
| true  | true  | off | off | off | ON  |

Row 3 (SLAM, no GPS) is the primary indoor-testing path. Row 4
(SLAM + GPS) is supported but GPS is decoupled from the map frame.

### 3.2 Launch hierarchy

```
start_docker.sh -s
  └→ ros2 launch robot_bringup robot_launch.py
       ├─ launch_slam (new arg, default TRUE)
       ├─ launch_gps  (existing, default false)
       └─ IncludeLaunchDescription gps_waypoint_follower.launch.py
            ├─ use_slam passed through from launch_slam
            ├─ use_gps  passed through from launch_gps
            ├─ IncludeLaunchDescription dual_ekf_navsat.launch.py
            │    ├─ gates static TF, map EKF, navsat_transform per table §3.1
            │    └─ ekf_filter_node_odom always runs (owns odom→base_link)
            ├─ IncludeLaunchDescription slam_toolbox.launch.py  (NEW)
            │    └─ async_slam_toolbox_node with slam_toolbox_params.yaml
            │    └─ condition=IfCondition(use_slam)
            └─ nav2_bringup/navigation_launch.py (unchanged)
```

### 3.3 Data flow when SLAM is active

```
RPLiDAR S3  ─── /scan ─┬──→ rf2o_laser_odometry ── odom_rf2o ──┐
                       │                                       ├─→ ekf_filter_node_odom ──┬─→ /odometry/local ──→ Nav2
                       │                                       │                          └─→ TF: odom→base_link
BNO085 IMU ─── /imu/data ──────────────────────────────────────┘
                       │
                       └──→ slam_toolbox ─┬─→ /map (occupancy grid, every 5s)
                                          └─→ TF: map→odom (50 Hz)
```

RF2O and slam_toolbox both consume `/scan`; they are independent
subscribers. slam_toolbox consumes TF (`odom→base_link`) published
by the odom EKF and produces the `map→odom` correction.

## 4. Components

### 4.1 Docker image — `docker/Dockerfile`

**Change:** add `ros-$ROS_DISTRO-slam-toolbox` to the existing nav2
`RUN apt-get install` block (around lines 63-67):

```dockerfile
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-slam-toolbox
```

Delete the commented-out stub at lines 69-71 (dead code).

**Why this works:** apt resolves slam-toolbox's libfastcdr requirement
together with nav2's libfastrtps requirement in a single transaction,
picking a coherent set. The prior attempt (commit `858f33d3`,
reverted) upgraded only fastcdr/fastrtps after the fact, which left
typesupport `.so` files (in `ros-jazzy-std-msgs`,
`ros-jazzy-rmw-fastrtps-cpp`, etc.) cached from the base image and
linked against the old ABI.

**Fallback if Gate 1 fails:** append `&& apt-get dist-upgrade -y` to
the same RUN. This bumps every ROS package transitively. Less
reproducible but always resolves ABI skew.

### 4.2 New launch file — `robot_navigation/launch/slam_toolbox.launch.py`

Single-node launch file. Runs `async_slam_toolbox_node` from the
`slam_toolbox` package, loading `slam_toolbox_params.yaml` from
`robot_navigation/config/`. No conditions in this file — gating is
the caller's job.

### 4.3 New config file — `robot_navigation/config/slam_toolbox_params.yaml`

Parameters tuned for WALL-E's hardware. Values deliberately set
(everything else stays at slam_toolbox defaults):

**Frames & topics (must match existing system):**
- `odom_frame: odom`
- `map_frame: map`
- `base_frame: base_link`
- `scan_topic: /scan`
- `mode: mapping`

**Lidar-specific (RPLiDAR S3 at 0.21m height):**
- `max_laser_range: 25.0` — S3 is rated to 40m but outdoor returns
  degrade past 25m; indoor is wall-bound. 25m is the useful ceiling.
- `min_laser_range: 0.05` — matches hard-coded `sllidar_node.cpp:228`.
- `resolution: 0.05` — 5cm grid. 2.5cm considered and deferred;
  bump post-Phase-1 if higher detail needed.

**Scan matching / motion gating:**
- `minimum_travel_distance: 0.5` — 50cm keyframe spacing, avoids
  node explosion when stationary.
- `minimum_travel_heading: 0.5` — ~29°, rotation-in-place scale.
- `minimum_time_interval: 0.5` — cap keyframe rate at 2 Hz.
- `use_scan_matching: true`
- `use_scan_barycenter: true`

**TF timing (must match Nav2's existing tolerance):**
- `transform_publish_period: 0.02` — 50 Hz `map→odom` publish.
- `transform_timeout: 0.2` — matches Nav2 `transform_tolerance: 0.2`
  set in commit `f4e71dc1`.
- `map_update_interval: 5.0` — 5s `/map` republish; RViz/Nav2 can
  afford this.

**Loop closure (matters for building-scale indoor runs):**
- `do_loop_closing: true` — non-negotiable; without this SLAM is
  just pose-graph odometry.
- `loop_search_space_dimension: 20.0` — search radius to match
  building-scale operation (user intent: entire building).
- `loop_match_minimum_response_fine: 0.45` — default; tighten to
  0.55 if Gate 3 shows false closures on repetitive geometry.

**Solver:** all defaults (Ceres + SPARSE_NORMAL_CHOLESKY +
SCHUR_JACOBI + Levenberg-Marquardt). Changing solver params without
a specific symptom is how weeks are wasted.

### 4.4 Modified — `robot_navigation/launch/dual_ekf_navsat.launch.py`

Add `use_slam` arg (default `false`). Update conditions on three
existing nodes:

- `ekf_filter_node_map`: `IfCondition(use_gps AND NOT use_slam)`
- `navsat_transform`: `IfCondition(use_gps AND NOT use_slam)`
- `map_to_odom_static`: `UnlessCondition(use_gps OR use_slam)`

`ekf_filter_node_odom` is unchanged — always runs, owns
`odom→base_link`. Conditions expressed via `PythonExpression` to
avoid the substitution-algebra headaches of `AndSubstitution`.

### 4.5 Modified — `robot_navigation/launch/gps_waypoint_follower.launch.py`

- Declare `use_slam` arg (default `false`).
- Pass `use_slam` through to `dual_ekf_navsat.launch.py`.
- Include new `slam_toolbox.launch.py` with
  `condition=IfCondition(use_slam)`.

No changes to the `twist_mux`, nav2, RViz, or mapviz includes.

### 4.6 Modified — `robot_bringup/launch/robot_launch.py`

- Declare `launch_slam` arg, **default `true`**.
- Pass through to `gps_waypoint_follower.launch.py` as `use_slam`.

### 4.7 Modified — `CLAUDE.md`

Update the "Launch hierarchy" defaults table to add
`launch_slam: true`, and add a one-paragraph note under
"Localization (dual EKF)" describing the new SLAM-as-map-owner mode.
Add the invocation hint:
`./start_docker.sh -c "ros2 launch robot_bringup robot_launch.py launch_slam:=false"`
for GPS-only outdoor runs.

## 5. Error handling & edge cases

- **Both `launch_slam=true` and `launch_gps=true`:** supported. GPS
  driver runs, `/fix` is published, but navsat_transform and map
  EKF are off. No georeferenced map. Documented in CLAUDE.md.
- **Stationary robot:** `minimum_travel_distance: 0.5` prevents
  slam_toolbox from adding useless keyframes. Pose graph stays
  small while robot idles.
- **Repetitive indoor geometry (long hallways):** risk of false
  loop closures. Mitigation path documented: tighten
  `loop_match_minimum_response_fine`.
- **Jetson thermal throttle / CPU pressure:** `resolution: 0.05` is
  the conservative choice. If CPU becomes a problem, first dial is
  to raise `map_update_interval` to 10s.
- **TF lookup failures during SLAM startup:** `transform_timeout:
  0.2` tolerates brief lag. If Gate 2 shows sustained TF timeouts,
  check that `ekf_filter_node_odom` is publishing at ≥20 Hz before
  blaming SLAM.
- **ABI still broken after Approach 1:** Gate 1 catches this. Do
  not advance to Gate 2 with a broken ABI. Escalate to Approach 2
  (dist-upgrade).

## 6. Rollback

**Fast (runtime, no rebuild):**
```bash
./start_docker.sh -c "ros2 launch robot_bringup robot_launch.py launch_slam:=false"
```
Disables SLAM for the current run; reverts to pre-SLAM behavior
path (static identity `map→odom` or map EKF per GPS flag).

**Clean (commit-level):** `git revert` the implementation commit(s).
Changes are additive (new launch file, new yaml) plus conditional
gates in two existing launch files; no tangled dependencies.

**Nuclear:** If the image itself is unusable, reset to commit
`82e4ad4f` (current HEAD) and rebuild. All SLAM-related changes
are on top of that baseline.

## 7. Post-success memory updates

- `project_slam_fastcdr.md` → mark resolved; note which install
  strategy worked.
- `project_instructions.md` → Priority 3.9 done; add tuning
  observations from Gate 3 (loop-closure behavior, any param
  changes made during testing).
- `walle.md` → new rule only if a genuinely surprising lesson
  emerged (don't rot the skill file with trivia).

## 8. Files changed summary

**New:**
- `ros2_ws/src/robot_navigation/launch/slam_toolbox.launch.py`
- `ros2_ws/src/robot_navigation/config/slam_toolbox_params.yaml`

**Modified:**
- `docker/Dockerfile` (add slam-toolbox to nav2 RUN, delete commented stub)
- `ros2_ws/src/robot_navigation/launch/dual_ekf_navsat.launch.py`
- `ros2_ws/src/robot_navigation/launch/gps_waypoint_follower.launch.py`
- `ros2_ws/src/robot_bringup/launch/robot_launch.py`
- `CLAUDE.md`

`robot_navigation/setup.py` needs no edit — its existing `data_files`
globs catch `*launch.py` and `*.yaml`.
