# SLAM Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Integrate `slam_toolbox` into WALL-E's ROS 2 stack so the `map` frame stops drifting during indoor testing, without triggering the libfastcdr/libfastrtps ABI break that killed the 2026-04-14 attempt.

**Architecture:** Install `ros-jazzy-slam-toolbox` in the same `apt-get install` transaction as nav2 so the dep resolver picks a coherent fastcdr set. Launch `async_slam_toolbox_node` in mapping mode, gated by a new `launch_slam` arg (default `true`). When SLAM is on, it becomes the sole `map→odom` publisher — existing static-identity TF and map-EKF publishers are gated off via `PythonExpression` conditions.

**Tech Stack:** ROS 2 Jazzy, slam_toolbox (async online mapping), robot_localization, nav2, Docker, Python launch files.

**Spec:** `docs/superpowers/specs/2026-04-20-slam-design.md`

**Baseline:** `oldcodetest` branch, commit `82e4ad4f` (or later — check `git log` at start).

---

## File Structure

**New files (both under `ros2_ws/src/robot_navigation/`):**
- `config/slam_toolbox_params.yaml` — tuned parameters for WALL-E's RPLiDAR S3 + indoor operation.
- `launch/slam_toolbox.launch.py` — single-node launch file; no conditions (caller gates).

**Modified files:**
- `docker/Dockerfile` — add `ros-$ROS_DISTRO-slam-toolbox` to the nav2 RUN; delete commented-out stub.
- `ros2_ws/src/robot_navigation/launch/dual_ekf_navsat.launch.py` — add `use_slam` arg; gate map EKF, navsat_transform, and static TF on it.
- `ros2_ws/src/robot_navigation/launch/gps_waypoint_follower.launch.py` — declare `use_slam`, pass through to dual_ekf, conditionally include slam launch file.
- `ros2_ws/src/robot_bringup/launch/robot_launch.py` — declare `launch_slam` arg (default `true`), pass through.
- `CLAUDE.md` — update defaults table + add SLAM-as-map-owner note.

**Untouched (by design):** `start_docker.sh`, `nav2_no_map_params.yaml`, `twist_mux.yaml`, `roboclaw_launch.py`, any ROS package `setup.py`/`CMakeLists.txt`. The `robot_navigation/setup.py` glob for launch/config files auto-picks up new files.

**Workflow reminder (from CLAUDE.md):** edit on host → commit → push → SSH Jetson → pull + rebuild → test. Do NOT build on the host machine.

---

## Task 1: Dockerfile — add slam-toolbox to nav2 RUN

**Files:**
- Modify: `docker/Dockerfile:63-71`

- [ ] **Step 1: Read the current Dockerfile section**

Confirm exact line numbers match before editing:

```bash
sed -n '63,71p' docker/Dockerfile
```

Expected output (lines 63-71):
```
# Install ROS navigation packages
RUN apt-get install -y \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup

# Install ROS SLAM packages
#RUN apt-get install -y \
#    ros-$ROS_DISTRO-slam-toolbox
```

If line numbers have shifted, adjust the Edit below to match the actual content.

- [ ] **Step 2: Edit Dockerfile — add slam-toolbox, add `apt-get update`, delete commented stub**

Use the Edit tool:

```
old_string:
# Install ROS navigation packages
RUN apt-get install -y \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup

# Install ROS SLAM packages
#RUN apt-get install -y \
#    ros-$ROS_DISTRO-slam-toolbox

new_string:
# Install ROS navigation + SLAM packages (single transaction so apt picks
# a coherent libfastcdr/libfastrtps set — see project_slam_fastcdr.md)
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-slam-toolbox
```

Note the added `apt-get update &&` (CLAUDE.md Rule 1: same RUN). The comment block is replaced so dead code doesn't accumulate.

- [ ] **Step 3: Verify the edit**

```bash
sed -n '63,71p' docker/Dockerfile
```

Expected:
```
# Install ROS navigation + SLAM packages (single transaction so apt picks
# a coherent libfastcdr/libfastrtps set — see project_slam_fastcdr.md)
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-slam-toolbox
```

- [ ] **Step 4: Commit**

```bash
git add docker/Dockerfile
git commit -m "$(cat <<'EOF'
docker: install slam-toolbox in same RUN as nav2

Bundles ros-jazzy-slam-toolbox into the existing nav2 apt-get
install so the dependency resolver picks a coherent libfastcdr /
libfastrtps set in a single transaction. The 2026-04-14 attempt
failed because installing slam-toolbox later left typesupport .so
files linked against the old fastcdr ABI; see memory
project_slam_fastcdr.md for full symptom.

Adds apt-get update per CLAUDE.md Rule 1.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
EOF
)"
```

- [ ] **Step 5: Push**

```bash
git push origin oldcodetest
```

- [ ] **Step 6: Rebuild image on Jetson (runs ~2 minutes)**

```bash
ssh walle@jetson "cd ~/WALL-E && git pull && cd docker && ./start_docker.sh -b"
```

Expected: build succeeds, no `E: Unable to locate package` or apt errors. Image `walle/ros2:jazzy` is rebuilt.

- [ ] **Step 7: Gate 1 — verify fastcdr ABI is intact**

Run inside the freshly built container (on the Jetson):

```bash
ssh walle@jetson "cd ~/WALL-E/docker && ./start_docker.sh -c 'ldd /opt/ros/jazzy/lib/libstd_msgs__rosidl_typesupport_fastrtps_c.so'"
```

Expected: every line resolves to a real path. **Zero** occurrences of `not found`. The `libfastcdr.so.*` line must show a versioned filename (e.g. `libfastcdr.so.2.2`) pointing to `/lib/aarch64-linux-gnu/libfastcdr.so.2.2` or similar.

- [ ] **Step 8: Gate 1 — verify ros2 CLI works**

```bash
ssh walle@jetson "cd ~/WALL-E/docker && ./start_docker.sh -c 'ros2 node list'"
```

Expected: prints an empty list or a small list of nodes. **Must NOT print** `symbol lookup error` or the `eprosima::fastcdr::Cdr::serialize` error.

- [ ] **Step 9: Gate 1 — verify slam_toolbox installed**

```bash
ssh walle@jetson "cd ~/WALL-E/docker && ./start_docker.sh -c 'ros2 pkg list' | grep slam_toolbox"
```

Expected: prints `slam_toolbox`.

**If any of Steps 7, 8, 9 fail:** STOP. Do not proceed to Task 2. Escalate to fallback:

```
old_string:
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-slam-toolbox

new_string:
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-slam-toolbox \
 && apt-get dist-upgrade -y
```

Commit with message `docker: dist-upgrade after slam-toolbox install to resolve fastcdr ABI`, push, rebuild on Jetson, re-run Steps 7-9.

---

## Task 2: Create `slam_toolbox_params.yaml`

**Files:**
- Create: `ros2_ws/src/robot_navigation/config/slam_toolbox_params.yaml`

- [ ] **Step 1: Write the config file**

Use the Write tool to create `ros2_ws/src/robot_navigation/config/slam_toolbox_params.yaml` with this exact content:

```yaml
# slam_toolbox parameters tuned for WALL-E
# Phase 1: indoor mapping mode, stable map frame, throwaway maps.
# Only deliberately-tuned parameters are set here; everything else
# stays at slam_toolbox's built-in defaults.
# See docs/superpowers/specs/2026-04-20-slam-design.md §4.3 for rationale.
slam_toolbox:
  ros__parameters:
    # --- Frames & topics (must match rest of the stack) ---
    odom_frame: odom
    map_frame: map
    base_frame: base_link
    scan_topic: /scan
    mode: mapping

    # --- Lidar (RPLiDAR S3 at 0.21 m above base_link) ---
    max_laser_range: 25.0
    min_laser_range: 0.05
    resolution: 0.05

    # --- Scan matching / motion gating ---
    minimum_travel_distance: 0.5
    minimum_travel_heading: 0.5
    minimum_time_interval: 0.5
    use_scan_matching: true
    use_scan_barycenter: true

    # --- TF timing (matches Nav2's transform_tolerance = 0.2) ---
    transform_publish_period: 0.02
    transform_timeout: 0.2
    map_update_interval: 5.0

    # --- Loop closure (building-scale) ---
    do_loop_closing: true
    loop_search_space_dimension: 20.0
    loop_match_minimum_response_fine: 0.45
```

- [ ] **Step 2: Sanity-check YAML syntax**

```bash
python3 -c "import yaml; yaml.safe_load(open('ros2_ws/src/robot_navigation/config/slam_toolbox_params.yaml'))"
```

Expected: no output (silent success). If it raises `yaml.YAMLError`, re-check indentation.

- [ ] **Step 3: Commit**

```bash
git add ros2_ws/src/robot_navigation/config/slam_toolbox_params.yaml
git commit -m "$(cat <<'EOF'
nav: add slam_toolbox_params.yaml tuned for WALL-E

Indoor mapping mode, 5cm resolution, 25m laser range matching
RPLiDAR S3 useful ceiling, 20m loop-closure search for
building-scale runs. Transform timing matches Nav2's existing
transform_tolerance of 0.2s (commit f4e71dc1).

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
EOF
)"
```

---

## Task 3: Create `slam_toolbox.launch.py`

**Files:**
- Create: `ros2_ws/src/robot_navigation/launch/slam_toolbox.launch.py`

- [ ] **Step 1: Write the launch file**

Use the Write tool to create `ros2_ws/src/robot_navigation/launch/slam_toolbox.launch.py` with this exact content:

```python
# Launch slam_toolbox in async online mapping mode.
# Caller is responsible for gating this include (we never
# start slam_toolbox unconditionally).
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('robot_navigation'),
        'config',
        'slam_toolbox_params.yaml',
    )

    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[params_file],
        ),
    ])
```

- [ ] **Step 2: Syntax-check the Python**

```bash
python3 -m py_compile ros2_ws/src/robot_navigation/launch/slam_toolbox.launch.py
```

Expected: no output (silent success).

- [ ] **Step 3: Commit**

```bash
git add ros2_ws/src/robot_navigation/launch/slam_toolbox.launch.py
git commit -m "$(cat <<'EOF'
nav: add slam_toolbox.launch.py

Single-purpose launch file for async_slam_toolbox_node. No
internal gating — callers are responsible for the IfCondition
on use_slam.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
EOF
)"
```

---

## Task 4: Gate `dual_ekf_navsat.launch.py` on `use_slam`

**Files:**
- Modify: `ros2_ws/src/robot_navigation/launch/dual_ekf_navsat.launch.py`

**Background:** Currently three candidate `map→odom` publishers exist (static identity, map EKF, navsat_transform). We add `use_slam`; when true, all three are off (slam_toolbox owns the TF).

- [ ] **Step 1: Read the current file**

```bash
sed -n '14,30p' ros2_ws/src/robot_navigation/launch/dual_ekf_navsat.launch.py
```

Confirm these imports:
```
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
```

- [ ] **Step 2: Update imports — add `PythonExpression`**

Use the Edit tool:

```
old_string:
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

new_string:
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
```

Note: `UnlessCondition` is dropped — we'll express every gate as `IfCondition(PythonExpression(...))` for consistency. This makes the 4-row truth table in the spec directly readable from the launch file.

- [ ] **Step 3: Add `use_slam` arg declaration**

```
old_string:
    use_gps = LaunchConfiguration('use_gps')

    return LaunchDescription(
        [
            DeclareLaunchArgument('use_gps', default_value='false',
                                 description='Use GPS and map-frame EKF'),

new_string:
    use_gps = LaunchConfiguration('use_gps')
    use_slam = LaunchConfiguration('use_slam')

    return LaunchDescription(
        [
            DeclareLaunchArgument('use_gps', default_value='false',
                                 description='Use GPS and map-frame EKF'),
            DeclareLaunchArgument('use_slam', default_value='false',
                                 description='slam_toolbox owns map->odom when true'),
```

- [ ] **Step 4: Gate map EKF on `use_gps AND NOT use_slam`**

```
old_string:
            # With GPS: map EKF publishes map->odom TF using GPS corrections
            launch_ros.actions.Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_map",
                output="screen",
                parameters=[rl_params_file, {'publish_tf': True}],
                remappings=[("odometry/filtered", "odometry/global")],
                condition=IfCondition(use_gps),
            ),

new_string:
            # With GPS and NOT SLAM: map EKF publishes map->odom TF.
            # SLAM always wins — when use_slam=true, slam_toolbox owns map->odom.
            launch_ros.actions.Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_map",
                output="screen",
                parameters=[rl_params_file, {'publish_tf': True}],
                remappings=[("odometry/filtered", "odometry/global")],
                condition=IfCondition(PythonExpression([
                    "'", use_gps, "' == 'true' and '", use_slam, "' != 'true'"
                ])),
            ),
```

- [ ] **Step 5: Gate navsat_transform on `use_gps AND NOT use_slam`**

```
old_string:
            # navsat_transform only needed with GPS
            launch_ros.actions.Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform",
                output="screen",
                parameters=[rl_params_file],
                remappings=[
                    ("imu/data", "imu/data"),
                    ("gps/fix", "fix"),
                    ("gps/filtered", "gps/filtered"),
                    ("odometry/gps", "odometry/gps"),
                    ("odometry/filtered", "odometry/global"),
                ],
                condition=IfCondition(use_gps),
            ),

new_string:
            # navsat_transform only needed with GPS AND not SLAM.
            # In SLAM+GPS mode, no georeference into the map frame (Phase 1 scope).
            launch_ros.actions.Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform",
                output="screen",
                parameters=[rl_params_file],
                remappings=[
                    ("imu/data", "imu/data"),
                    ("gps/fix", "fix"),
                    ("gps/filtered", "gps/filtered"),
                    ("odometry/gps", "odometry/gps"),
                    ("odometry/filtered", "odometry/global"),
                ],
                condition=IfCondition(PythonExpression([
                    "'", use_gps, "' == 'true' and '", use_slam, "' != 'true'"
                ])),
            ),
```

- [ ] **Step 6: Gate static identity TF on `NOT use_gps AND NOT use_slam`**

```
old_string:
            # Without GPS: static identity map->odom so the map frame is stable
            launch_ros.actions.Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_odom_static",
                output="screen",
                arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
                condition=UnlessCondition(use_gps),
            ),

new_string:
            # No GPS and no SLAM: static identity map->odom so the map frame is stable
            # (dead-reckoning mode). SLAM and GPS map-EKF each take precedence if on.
            launch_ros.actions.Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_odom_static",
                output="screen",
                arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
                condition=IfCondition(PythonExpression([
                    "'", use_gps, "' != 'true' and '", use_slam, "' != 'true'"
                ])),
            ),
```

- [ ] **Step 7: Syntax-check**

```bash
python3 -m py_compile ros2_ws/src/robot_navigation/launch/dual_ekf_navsat.launch.py
```

Expected: no output.

- [ ] **Step 8: Commit**

```bash
git add ros2_ws/src/robot_navigation/launch/dual_ekf_navsat.launch.py
git commit -m "$(cat <<'EOF'
nav: gate dual_ekf_navsat on use_slam

Adds use_slam launch arg. When true, slam_toolbox is expected
to own map->odom, so this launch file disables its three
candidate publishers (map EKF, navsat_transform, static
identity TF). Conditions expressed as PythonExpression so the
4-row truth table in the spec maps 1:1 to the launch file.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
EOF
)"
```

---

## Task 5: Plumb `use_slam` through `gps_waypoint_follower.launch.py`

**Files:**
- Modify: `ros2_ws/src/robot_navigation/launch/gps_waypoint_follower.launch.py`

- [ ] **Step 1: Declare `use_slam` and pass into dual_ekf**

```
old_string:
    use_rviz = LaunchConfiguration('use_rviz')
    use_mapviz = LaunchConfiguration('use_mapviz')
    launch_waypoint_follower = LaunchConfiguration('launch_waypoint_follower')
    use_gps = LaunchConfiguration('use_gps')

new_string:
    use_rviz = LaunchConfiguration('use_rviz')
    use_mapviz = LaunchConfiguration('use_mapviz')
    launch_waypoint_follower = LaunchConfiguration('launch_waypoint_follower')
    use_gps = LaunchConfiguration('use_gps')
    use_slam = LaunchConfiguration('use_slam')
```

- [ ] **Step 2: Add `DeclareLaunchArgument` for `use_slam`**

```
old_string:
    declare_use_gps_cmd = DeclareLaunchArgument(
        'use_gps',
        default_value='false',
        description='Use GPS and map-frame EKF')

new_string:
    declare_use_gps_cmd = DeclareLaunchArgument(
        'use_gps',
        default_value='false',
        description='Use GPS and map-frame EKF')

    declare_use_slam_cmd = DeclareLaunchArgument(
        'use_slam',
        default_value='false',
        description='Enable slam_toolbox (becomes map->odom owner)')
```

- [ ] **Step 3: Forward `use_slam` to dual_ekf_navsat include**

```
old_string:
    robot_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'dual_ekf_navsat.launch.py')),
        launch_arguments={'use_gps': use_gps}.items()
    )

new_string:
    robot_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'dual_ekf_navsat.launch.py')),
        launch_arguments={'use_gps': use_gps, 'use_slam': use_slam}.items()
    )
```

- [ ] **Step 4: Add slam_toolbox include with `IfCondition(use_slam)`**

Find the block with `mapviz_cmd`, then add the new `slam_toolbox_cmd` right before the `# Create the launch description and populate` comment.

```
old_string:
    mapviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'mapviz.launch.py')),
        condition=IfCondition(use_mapviz)
    )

    # Create the launch description and populate
    ld = LaunchDescription()

new_string:
    mapviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'mapviz.launch.py')),
        condition=IfCondition(use_mapviz)
    )

    slam_toolbox_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'slam_toolbox.launch.py')),
        condition=IfCondition(use_slam)
    )

    # Create the launch description and populate
    ld = LaunchDescription()
```

- [ ] **Step 5: Register the new `declare_use_slam_cmd` and `slam_toolbox_cmd` actions**

```
old_string:
    ld.add_action(declare_launch_waypoint_follower_cmd)
    ld.add_action(declare_use_gps_cmd)

    return ld

new_string:
    ld.add_action(declare_launch_waypoint_follower_cmd)
    ld.add_action(declare_use_gps_cmd)
    ld.add_action(declare_use_slam_cmd)
    ld.add_action(slam_toolbox_cmd)

    return ld
```

- [ ] **Step 6: Syntax-check**

```bash
python3 -m py_compile ros2_ws/src/robot_navigation/launch/gps_waypoint_follower.launch.py
```

Expected: no output.

- [ ] **Step 7: Commit**

```bash
git add ros2_ws/src/robot_navigation/launch/gps_waypoint_follower.launch.py
git commit -m "$(cat <<'EOF'
nav: plumb use_slam through gps_waypoint_follower

Declares use_slam arg, forwards it to dual_ekf_navsat, and
conditionally includes the new slam_toolbox.launch.py.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
EOF
)"
```

---

## Task 6: Expose `launch_slam` at the top level (`robot_launch.py`)

**Files:**
- Modify: `ros2_ws/src/robot_bringup/launch/robot_launch.py`

- [ ] **Step 1: Add `launch_slam` LaunchConfiguration binding**

```
old_string:
    launch_rplidar = LaunchConfiguration('launch_rplidar')
    launch_bno085 = LaunchConfiguration('launch_bno085')
    launch_gps = LaunchConfiguration('launch_gps')
    launch_urdf = LaunchConfiguration('launch_urdf')
    launch_nav = LaunchConfiguration('launch_nav')
    launch_d2oc = LaunchConfiguration('launch_d2oc')
    use_rviz = LaunchConfiguration('use_rviz')
    launch_waypoint_follower = LaunchConfiguration('launch_waypoint_follower')

new_string:
    launch_rplidar = LaunchConfiguration('launch_rplidar')
    launch_bno085 = LaunchConfiguration('launch_bno085')
    launch_gps = LaunchConfiguration('launch_gps')
    launch_urdf = LaunchConfiguration('launch_urdf')
    launch_nav = LaunchConfiguration('launch_nav')
    launch_slam = LaunchConfiguration('launch_slam')
    launch_d2oc = LaunchConfiguration('launch_d2oc')
    use_rviz = LaunchConfiguration('use_rviz')
    launch_waypoint_follower = LaunchConfiguration('launch_waypoint_follower')
```

- [ ] **Step 2: Add `DeclareLaunchArgument('launch_slam', default_value='true')`**

```
old_string:
        DeclareLaunchArgument('launch_rplidar', default_value='true'),
        DeclareLaunchArgument('launch_bno085', default_value='true'),
        DeclareLaunchArgument('launch_gps', default_value='false'),
        DeclareLaunchArgument('launch_urdf', default_value='true'),
        DeclareLaunchArgument('launch_nav', default_value='true'),
        DeclareLaunchArgument('launch_d2oc', default_value='false'),

new_string:
        DeclareLaunchArgument('launch_rplidar', default_value='true'),
        DeclareLaunchArgument('launch_bno085', default_value='true'),
        DeclareLaunchArgument('launch_gps', default_value='false'),
        DeclareLaunchArgument('launch_urdf', default_value='true'),
        DeclareLaunchArgument('launch_nav', default_value='true'),
        DeclareLaunchArgument('launch_slam', default_value='true'),
        DeclareLaunchArgument('launch_d2oc', default_value='false'),
```

- [ ] **Step 3: Forward `launch_slam` into the nav include as `use_slam`**

```
old_string:
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('robot_navigation'), '/launch/gps_waypoint_follower.launch.py'
            ]),
            condition=IfCondition(launch_nav),
            launch_arguments={
                'use_rviz': use_rviz,
                'launch_waypoint_follower': launch_waypoint_follower,
                'use_gps': launch_gps,
            }.items()
        ),

new_string:
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('robot_navigation'), '/launch/gps_waypoint_follower.launch.py'
            ]),
            condition=IfCondition(launch_nav),
            launch_arguments={
                'use_rviz': use_rviz,
                'launch_waypoint_follower': launch_waypoint_follower,
                'use_gps': launch_gps,
                'use_slam': launch_slam,
            }.items()
        ),
```

- [ ] **Step 4: Syntax-check**

```bash
python3 -m py_compile ros2_ws/src/robot_bringup/launch/robot_launch.py
```

Expected: no output.

- [ ] **Step 5: Commit**

```bash
git add ros2_ws/src/robot_bringup/launch/robot_launch.py
git commit -m "$(cat <<'EOF'
nav: expose launch_slam in robot_launch.py (default true)

New top-level arg launch_slam forwarded as use_slam to
gps_waypoint_follower.launch.py. Default true so that
./start_docker.sh -s launches with SLAM on.

To run without SLAM (e.g. outdoor GPS test):
  ./start_docker.sh -c "ros2 launch robot_bringup robot_launch.py launch_slam:=false launch_gps:=true"

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
EOF
)"
```

---

## Task 7: Update `CLAUDE.md` defaults table + SLAM note

**Files:**
- Modify: `CLAUDE.md`

- [ ] **Step 1: Add `launch_slam` row to the launch-hierarchy table**

```
old_string:
| `launch_nav`     | true | `gps_waypoint_follower.launch.py` → dual EKF, twist_mux, Nav2, RF2O |
| `launch_d2oc`    | false | D2OC exploration |

new_string:
| `launch_nav`     | true | `gps_waypoint_follower.launch.py` → dual EKF, twist_mux, Nav2, RF2O |
| `launch_slam`    | true | `slam_toolbox.launch.py` (async online mapping); owns `map→odom` when on |
| `launch_d2oc`    | false | D2OC exploration |
```

- [ ] **Step 2: Update the `map → odom` TF selection section**

```
old_string:
### `map → odom` TF selection

Only one node may publish `map → odom`. Selection is automatic:

| `launch_gps` | Publisher |
|---|---|
| false (default) | Static identity TF (dead-reckoning — map nailed to odom) |
| true            | `ekf_filter_node_map` (fuses GPS) |

SLAM is **not currently integrated**. A prior attempt to add
`slam_toolbox` caused a fastcdr/fastrtps ABI break that crashed
every ROS node. See memory: `project_slam_fastcdr.md` for the
symptom and three fix strategies to try next time.

new_string:
### `map → odom` TF selection

Only one node may publish `map → odom`. Selection is automatic:

| `launch_slam` | `launch_gps` | Publisher |
|---|---|---|
| true | any | `slam_toolbox` (async online mapping) |
| false | false | Static identity TF (dead-reckoning — map nailed to odom) |
| false | true | `ekf_filter_node_map` (fuses GPS) |

**SLAM is enabled by default** in `robot_launch.py` (Phase 1: indoor
mapping, stable `map` frame, throwaway maps). When `launch_slam=true`
and `launch_gps=true`, the GPS driver still publishes `/fix` but
`navsat_transform` and the map EKF are gated off — no GPS fusion into
the SLAM pose graph in Phase 1. To run without SLAM (e.g. outdoor
GPS test):

    ./start_docker.sh -c "ros2 launch robot_bringup robot_launch.py launch_slam:=false launch_gps:=true"

slam_toolbox parameters live in
`ros2_ws/src/robot_navigation/config/slam_toolbox_params.yaml`
(tuned for RPLiDAR S3 + indoor/building-scale).
```

- [ ] **Step 3: Verify the diff looks right**

```bash
git diff CLAUDE.md
```

Expected: two hunks, both within the Architecture section. No unrelated changes.

- [ ] **Step 4: Commit**

```bash
git add CLAUDE.md
git commit -m "$(cat <<'EOF'
docs: update CLAUDE.md for SLAM integration

Adds launch_slam row to defaults table. Rewrites map->odom
selection table to include SLAM as the priority publisher.
Documents the Phase 1 limitation (no GPS fusion into SLAM
map) and the override syntax for GPS-only outdoor runs.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
EOF
)"
```

---

## Task 8: Gate 2 — SLAM comes up on hardware

**Files:** no edits; hardware integration test.

- [ ] **Step 1: Push all Task 2-7 commits**

```bash
git push origin oldcodetest
```

- [ ] **Step 2: Sync the Jetson and rebuild**

Workspace is baked into the image at build time via `COPY ros2_ws`, so a full image rebuild is needed to pick up the new yaml + launch files:

```bash
ssh walle@jetson "cd ~/WALL-E && git pull && cd docker && ./start_docker.sh -b"
```

Expected: build succeeds.

- [ ] **Step 3: Show resolved args**

```bash
ssh walle@jetson "cd ~/WALL-E/docker && ./start_docker.sh -c 'ros2 launch robot_bringup robot_launch.py --show-args'"
```

Expected: the output lists `launch_slam` with default `true`. No launch-file import errors.

- [ ] **Step 4: Bring up the full system with SLAM**

Run the main launch (user drives this; watch the terminal output):

```bash
ssh walle@jetson "cd ~/WALL-E/docker && ./start_docker.sh -s"
```

Expected: no `process has died` or exit-127 lines. `slam_toolbox` node shows up in the log, reports "Registering sensor ... /scan" or similar, and enters online mapping.

- [ ] **Step 5: Verify SLAM topics exist**

In a second SSH session:

```bash
ssh walle@jetson "docker exec \$(docker ps -q -f ancestor=walle/ros2:jazzy) /entrypoint.sh ros2 topic list | grep -E '(^/map|slam_toolbox)'"
```

Expected: at minimum `/map`, `/slam_toolbox/graph_visualization`, `/slam_toolbox/scan_visualization`.

- [ ] **Step 6: Verify exactly ONE publisher of map→odom TF**

```bash
ssh walle@jetson "docker exec \$(docker ps -q -f ancestor=walle/ros2:jazzy) /entrypoint.sh ros2 run tf2_ros tf2_echo map odom --timeout 3"
```

Expected: TF prints every 1s with a translation/rotation. **If no TF or multiple conflicting TFs:** check that `map_to_odom_static` is NOT running (it should be gated off by `use_slam=true`):

```bash
ssh walle@jetson "docker exec \$(docker ps -q -f ancestor=walle/ros2:jazzy) /entrypoint.sh ros2 node list | grep map_to_odom_static"
```

Expected: **empty output** (node does not exist when SLAM is on).

- [ ] **Step 7: Open RViz on the host and verify map renders**

On Julian's host machine (NOT over SSH):

```bash
cd ~/WALL-E/docker && ./start_docker.sh -d -c "rviz2 -d /home/walle/ros2_ws/install/robot_navigation/share/robot_navigation/config/walle_default.rviz"
```

Expected:
- Fixed Frame `map` resolves (no red "No transform from ..." warning in RViz globals).
- The robot footprint renders.
- `/map` display shows an occupancy grid that grows as the robot is teleoped.

**If Gate 2 fails:** STOP. Diagnose with `ros2 node list`, `ros2 topic info /tf`, `ros2 launch ... --show-args`, and Jetson terminal log. Do NOT proceed to Gate 3 with SLAM broken.

---

## Task 9: Gate 3 — indoor SLAM quality

**Files:** no edits; live hardware validation.

- [ ] **Step 1: Teleop the robot around the test space for 2-5 minutes**

Use the joystick (hold LB to enable teleop). Drive straight lines, make turns, revisit the starting pose. Observe RViz on the host.

- [ ] **Step 2: Verify map frame stability (the original bug this fixes)**

In RViz, with Fixed Frame `map`, the map should be stable while the robot moves through it. If the map visibly rotates with the robot, SLAM is not publishing TF correctly — return to Gate 2 diagnosis.

- [ ] **Step 3: Verify a loop closure fires**

Drive the robot in a loop back to the starting pose. Watch `/slam_toolbox/graph_visualization` in RViz (add a MarkerArray display subscribed to it). Expected: pose-graph nodes appear as the robot moves; loop-closure edges (different color) appear when the robot revisits a previously-mapped area. The map snaps to fix accumulated drift.

If loop closures never fire even on obvious revisits, try raising `loop_match_minimum_response_fine` diagnostic range and rebuilding. (Phase 1 default is 0.45; if needed: lower to 0.35 and rebuild, BUT beware false positives.)

- [ ] **Step 4: Verify Nav2 2D Goal works on the built map**

In RViz, click "2D Nav Goal" and set a goal in a previously-mapped area. Expected: Nav2 plans a path, MPPI drives the robot, no `TF lookup failed` or `transform tolerance exceeded` errors in the Jetson terminal.

- [ ] **Step 5: Watch for false loop closures on repetitive geometry**

If the map visibly jumps or hallways misalign when the robot traverses a long, featureless corridor, tighten `loop_match_minimum_response_fine` from 0.45 to 0.55 in `slam_toolbox_params.yaml`, commit, push, rebuild, and re-test.

Exact edit (only if needed):

```
old_string:    loop_match_minimum_response_fine: 0.45
new_string:    loop_match_minimum_response_fine: 0.55
```

Commit message: `nav: tighten slam_toolbox loop closure threshold to 0.55 (observed false closures on repetitive geometry)`.

- [ ] **Step 6: Watch for CPU / thermal issues on the Jetson**

Over the course of Gate 3 testing, check `tegrastats` in a separate Jetson terminal:

```bash
ssh walle@jetson tegrastats --interval 2000
```

Expected: CPU usage steady < 80%, no thermal throttle (`CPU@` temperature < 80°C sustained). If CPU is pegged:
- First dial: raise `map_update_interval` from 5.0 to 10.0 in `slam_toolbox_params.yaml`.
- Second dial: leave `resolution: 0.05` (already conservative).

- [ ] **Step 7: Record observations for memory update (Task 10)**

Note any tuning changes made, unexpected behavior, or loop closure quirks. Rough notes are fine — Task 10 formalizes them.

**If Gate 3 passes:** SLAM integration is complete. Proceed to Task 10 (memory updates).

**If Gate 3 fails after exhausting diagnostic dials:** fall back to runtime rollback:

```bash
./start_docker.sh -c "ros2 launch robot_bringup robot_launch.py launch_slam:=false"
```

Then file observations in memory, do not revert commits — the infrastructure remains for future tuning.

---

## Task 10: Memory updates

**Files:**
- Modify: `/home/julian/.claude/projects/-home-julian-WALL-E/memory/project_slam_fastcdr.md`
- Modify: `/home/julian/.claude/projects/-home-julian-WALL-E/memory/project_instructions.md`
- Modify: `/home/julian/.claude/projects/-home-julian-WALL-E/memory/MEMORY.md` (only if filenames change)
- Consider: `.claude/skills/walle.md` (only if a genuinely new rule emerged)

**Note:** Memory files live outside the git repo. These are NOT committed to the repo.

- [ ] **Step 1: Mark `project_slam_fastcdr.md` as resolved**

Open the file and update its description and body to record which install strategy worked. At the bottom of the file, add a `## Resolution (2026-04-20)` section noting: which approach (single-transaction vs dist-upgrade) was used, the commit hash of the successful Dockerfile change, and which Jazzy apt versions of libfastcdr / libfastrtps the image ended up with (capture from `apt list --installed | grep fastcdr` inside the working container).

- [ ] **Step 2: Update `project_instructions.md`**

- Flip Priority 3.9 to `[x]`; reference the design spec and plan paths.
- Under "Solved Issues", add a row:

  | slam_toolbox integration | fastcdr ABI split install | Single-transaction nav2+slam-toolbox apt install | <commit hash> |

- Update the "Current Status" date and latest commit.
- Add a note under "Active Problems" if Gate 3 surfaced any new issues (false loop closures, CPU headroom, etc.).

- [ ] **Step 3: Only if a genuinely new rule emerged — add to `walle.md`**

Examples of rules worth adding:
- If we discovered a new slam_toolbox param that had to be tuned for our hardware beyond the spec.
- If a new failure mode appeared during Gate 2/3 that future-us should know about.

Examples of things NOT worth adding (would rot the skill file):
- "slam_toolbox needs /scan" (obvious from the code).
- "the yaml uses ros__parameters" (standard ROS).

If nothing qualifies, skip this step.

- [ ] **Step 4: Update `MEMORY.md` only if the name or purpose of an existing memory file changed**

The index is one line per file. If `project_slam_fastcdr.md` changed status from "active incident" to "resolved reference", update its one-line hook accordingly.

---

## Self-Review Notes

- **Spec coverage:**
  - §4.1 Dockerfile change → Task 1
  - §4.2 new launch file → Task 3
  - §4.3 new params yaml → Task 2
  - §4.4 dual_ekf_navsat gates → Task 4
  - §4.5 gps_waypoint_follower plumb → Task 5
  - §4.6 robot_launch top-level arg → Task 6
  - §4.7 CLAUDE.md update → Task 7
  - §2 Gate 1 → Task 1 steps 7-9 (with fallback)
  - §2 Gate 2 → Task 8
  - §2 Gate 3 → Task 9
  - §6 Rollback → embedded in Task 1 fallback + Task 9 failure path
  - §7 Memory updates → Task 10
  - All spec sections covered.

- **No placeholders** detected: every step has exact commands, file paths, or code content.

- **Type / name consistency:** `use_slam` (launch arg inside `gps_waypoint_follower` and `dual_ekf_navsat`) maps 1:1 to `launch_slam` (top-level in `robot_launch.py`). Condition expressions use the same `'$(var)' == 'true'` pattern throughout. Yaml keys match the spec exactly.
