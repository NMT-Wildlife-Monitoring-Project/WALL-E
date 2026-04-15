# CLAUDE.md

Persistent, project-specific instructions for Claude Code. Loaded into
every session in this repo. This is the single source of truth —
there is no AGENT.md. Keep it accurate. If something here contradicts
what you read in the code, trust the code and update this file.

---

## How to work on WALL-E (read this first, every session)

**Do not make generic ROS 2 assumptions.** This robot has a specific
hardware stack, a specific launch tree, and a history of subtle bugs
that "standard" fixes have already broken. Always:

1. Read the actual launch/config files before proposing a change.
2. Prefer editing existing files over adding new ones.
3. For any hardware-facing change (motors, IMU, lidar, GPS, TF tree,
   EKF, Nav2, twist_mux), quote the file + line number you're
   changing from and explain the blast radius in one sentence.
4. Never push directly to `main`. Working branch is `oldcodetest`.
5. Never add `apt install` lines to the Dockerfile without `apt-get
   update` in the same RUN (stale indexes have bitten us twice — see
   memory: `project_slam_fastcdr.md`).
6. When unsure, check `~/.claude/projects/-home-julian-WALL-E/memory/`
   for prior incidents before theorising. Start with
   `project_instructions.md`.

---

## Project overview

WALL-E (Wildlife Activity Life Explorer) is an autonomous outdoor
wildlife-monitoring robot built on ROS 2 **Jazzy**. Runs in Docker
on a Jetson Orin Nano (primary) or Raspberry Pi 5. Differential drive
via RoboClaw, RPLiDAR S3, BNO085 IMU, u-blox GPS.

---

## Connecting to the Jetson

The Jetson is the robot's onboard computer. All robot code runs
there, inside Docker.

**Connection workflow (on-demand only — don't auto-connect):**

```bash
ping jetson            # verify Jetson is reachable over mDNS (.local)
ssh walle@jetson       # passwordless SSH (key-based). No password prompt.
```

Once SSH'd in, you are in the `walle` user's home on the Jetson.
From there, `cd ~/WALL-E/docker` and use `./start_docker.sh` flags
described below.

mDNS (`avahi-daemon`) is what resolves the `jetson` hostname — do not
suggest editing `/etc/hosts` or hard-coding IPs.

---

## Dev workflow: edit on host, build on Jetson

The canonical workflow for every code change:

1. **Host terminal (where Claude runs):** edit files, `git add`,
   `git commit`, `git push`. Never build on the host — the host is
   not where the robot runs.
2. **Jetson terminal (SSH):** `git pull` and rebuild the Docker
   image (or just colcon-build the workspace inside the running
   container for quick Python changes). Claude does this via
   `ssh walle@jetson "cd ~/WALL-E && git pull && cd docker && ./start_docker.sh -b"`
   — one-shot SSH commands, not a persistent session.
3. **User tests** — starts the robot on the Jetson, reports behavior.

Claude is responsible for **both** sides: pushing from host AND
pulling + building on the Jetson. Don't leave the Jetson out of
sync. If you're unsure whether the Jetson pulled your latest push,
`ssh walle@jetson "cd ~/WALL-E && git log -1 --oneline"` to check.

For small Python-only tweaks, a faster loop is
`ssh walle@jetson "docker exec <container> bash -c 'cd /home/walle/ros2_ws && colcon build --symlink-install --packages-select <pkg>'"`
— avoids a full image rebuild.

---

## Testing workflow (when you want to actually run the robot)

1. User says they're testing → SSH to Jetson (`ssh walle@jetson`).
2. On the Jetson: `cd ~/WALL-E/docker && ./start_docker.sh -s` to
   launch the full system. `-s` sets `launch_nav=true` and
   `launch_rplidar=true` by default.
3. **Open RViz yourself, in a host terminal (not in the SSH session
   to the Jetson)**, so the user doesn't have to. RViz runs on the
   host's display and subscribes to the Jetson's ROS topics over
   the LAN (both use `ROS_DOMAIN_ID=62`). Command to run locally:

   ```bash
   cd ~/WALL-E/docker && ./start_docker.sh -d -c "rviz2 -d /home/walle/ros2_ws/install/robot_navigation/share/robot_navigation/config/walle_default.rviz"
   ```

   `-d` enables X11 forwarding; `-c` runs an arbitrary command.
   Only launch RViz during active testing — don't pre-open it.

4. Monitor the Jetson's terminal via the SSH session — watch for
   node crashes (`process has died`, exit 127, symbol lookup errors)
   and TF warnings.

---

## Build & run commands

Everything runs inside Docker. Workspace is `ros2_ws/`.

```bash
cd docker

# Build
./start_docker.sh -b           # rebuild image (CACHE_BUST forces fresh workspace copy)

# Run modes (pick one action flag):
./start_docker.sh -s           # Full system: robot_launch.py (lidar, imu, nav, roboclaw)
./start_docker.sh -t           # Teleop only (joystick)
./start_docker.sh -m           # Motors only (roboclaw_driver)
./start_docker.sh -c "cmd"     # Arbitrary command, e.g. rviz2, ros2 topic list
./start_docker.sh              # Interactive bash shell

# Modifiers (combine with action flags):
./start_docker.sh -d -s        # -d enables X11 display forwarding
./start_docker.sh -i 42 -s     # -i sets ROS_DOMAIN_ID (default 62)
./start_docker.sh -x           # stop the running container
./start_docker.sh -R           # restart the running container

# Manual build inside the container:
cd /home/walle/ros2_ws && colcon build --symlink-install && source install/setup.bash
```

No linters, formatters, or test suites are configured. If you think
you're "done", you aren't sure — rebuild the image (`-b`) and bring
the system up to verify no node crashes.

---

## Architecture (verified against code)

### Velocity command flow (critical path)

```
Joystick → joy_linux → /joy → teleop_twist_joy → /cmd_vel_teleop ─┐
                                                                   ├→ twist_mux → /cmd_vel_out → roboclaw_node → motors
Nav2 MPPI controller ──────────────────────────→ /cmd_vel_nav ────┘
```

**Non-obvious detail:** `twist_mux` publishes to `/cmd_vel_out` at
runtime regardless of the `output_topic` setting in
`twist_mux.yaml`. `roboclaw_node` subscribes via an explicit remap
in `roboclaw_launch.py:87` (`('cmd_vel', '/cmd_vel_out')`). Do not
"fix" the yaml — the current remapping is load-bearing.

**Collision monitor:** nav2_bringup's upstream `navigation_launch.py`
(Jazzy) **already launches** `collision_monitor` and includes it in
the `lifecycle_manager_navigation` node list by default. Our
`nav2_no_map_params.yaml` provides its config (scan source,
FootprintApproach polygon). To put it inline in the command path,
the only required change is flipping the RoboClaw remap in
`roboclaw_launch.py:87` from `/cmd_vel_out` → `/cmd_vel_safe`.
Do NOT launch a second `collision_monitor` node or a sidecar
lifecycle_manager — that causes a name collision and dual-manager
thrash that killed nav last time (reverted commit `6b613f5d`).

Twist_mux priorities: teleop 20, nav 10 (teleop wins).

### Localization (dual EKF)

Two `robot_localization` EKF nodes run in parallel
(`dual_ekf_navsat.launch.py`):

- **ekf_filter_node_odom** — fuses RF2O laser odometry + BNO085 IMU →
  publishes `/odometry/local` and `odom → base_link` TF. Always on.
- **ekf_filter_node_map** — fuses the above + GPS → publishes
  `/odometry/global` and `map → odom` TF. **Only when `launch_gps=true`.**
- **navsat_transform** — converts GPS fixes into the `map` frame.

Wheel odometry from the RoboClaw is **not** currently fused into
either EKF (was tried and reverted — see `d52ba681`).

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

### Launch hierarchy

`robot_launch.py` is the master file. Boolean args + defaults:

| Arg | Default | Effect |
|---|---|---|
| `launch_rplidar` | true | sllidar_ros2 S3 driver |
| `launch_bno085`  | true | BNO085 IMU driver |
| `launch_gps`     | false | NMEA GPS driver + map EKF path |
| `launch_urdf`    | true | robot_state_publisher from xacro |
| `launch_nav`     | true | `gps_waypoint_follower.launch.py` → dual EKF, twist_mux, Nav2, RF2O |
| `launch_d2oc`    | false | D2OC exploration |
| `use_rviz`       | false | RViz (we usually open it on host instead) |
| `launch_waypoint_follower` | false | Auto-start GPS waypoint following |

RoboClaw always launches — there is no toggle.

### ROS 2 packages (`ros2_ws/src/`)

| Package | Type | Role |
|---|---|---|
| `roboclaw_driver` | ament_python | Motor control, wheel odometry, encoder mgmt. Node: `roboclaw_node`. |
| `robot_bringup` | ament_cmake | Master launch file + URDF/xacro. |
| `robot_navigation` | ament_python | Nav2 config, twist_mux, dual EKF, collision monitor config. |
| `robot_teleop` | ament_python | Logitech F710 joystick. Holds LB to enable. |
| `d2oc_algorithm` | ament_python | Entropy-based frontier exploration. |
| `bno085_driver` | ament_python | BNO085 IMU over I2C bus 7, address 0x4B. |
| `sllidar_ros2` | ament_cmake (C++) | RPLiDAR S3 driver. Publishes `/scan` in `lidar_link` frame. |
| `nmea_navsat_driver` | ament_python | GPS/GNSS via serial. |
| `robot_messages` | ament_cmake | `RoboclawStatus.msg` + other custom msgs. |
| `waypoint_server` | ament_python | GPS waypoint load/follow. Uses `/fromLL`. |
| `scan_matcher` | ament_cmake (C++) | ICP scan matching — present but **not in default runtime**. |
| `robot_web_interface` | — | Web UI. |

### RoboClaw parameters

All motor parameters live in
`roboclaw_driver/config/roboclaw_params.yaml`. Derived from physical
measurement + BasicMicro auto-tune:

- Wheel separation: 0.39 m, wheel diameter: 0.095 m
- Encoder: 6533 QPPR
- Max hardware speed: 25410 QPPS (M2 is the ceiling)
- Working max: 9000 QPPS (~0.4 m/s)
- Both motors: `m1_reverse: false`, `m2_reverse: false`
  (M2 encoder A/B wires were physically swapped — see
  `project_m2_encoder.md` in memory)
- Serial: `/dev/roboclaw` at 115200 baud, address 128

**Rule:** YAML is authoritative for defaults; launch-arg defaults in
`roboclaw_launch.py` and in-node defaults should stay in sync with
the YAML. If you change one, change them all.

### Hardware frames and geometry

From `robot.urdf.xacro`:

- `imu_link`: `base_link` offset `(-0.07, 0, 0.05)`
- `gps_link`: `base_link` offset `(0, 0.05, 0.05)`
- `lidar_link`: `base_link` offset `(0, 0, 0.21)` — 21 cm above center
- Robot footprint (from `nav2_no_map_params.yaml`):
  `[[0.199, 0.185], [0.199, -0.185], [-0.199, -0.185], [-0.199, 0.185]]`
  Corner radius from base_link origin: **0.272 m**.
- RPLiDAR `range_min` = 0.05 m (hard-coded in
  `sllidar_node.cpp:228`). Acceptable because nothing on the chassis
  rises above 0.21 m within the 0.27 m corner radius, so the scan
  plane clears the body.

### Docker details

- Image: `walle/ros2:jazzy`
- Runs `--privileged --net=host`, full `/dev` mount for hardware
- `ROS_DOMAIN_ID` default: 62
- `BLINKA_FORCEBOARD=RaspberryPi5` for Adafruit libs
- User inside container: `walle`, passwordless sudo
- Groups: dialout, tty, audio, video, gpio (GID 993), i2c (GID 116), input

### udev rules (`scripts/99-walle-devices.rules`)

- RoboClaw → `/dev/roboclaw` (03eb:2404)
- u-blox GPS → `/dev/gps` (1546:01a7)
- RPLiDAR S3 → `/dev/rplidar` (10c4:ea60)

Install on host with:
`sudo cp scripts/99-walle-devices.rules /etc/udev/rules.d/ && sudo udevadm control --reload && sudo udevadm trigger`

---

## Coding standards

- **ROS 2 version:** Jazzy only. Reject suggestions from Humble/Foxy
  tutorials without verifying they still apply to Jazzy.
- **Python:** 3.12 (Jazzy default). Don't use deprecated `rclpy`
  APIs. No type-checker / linter configured — don't add one without
  asking.
- **C++:** C++17 (Jazzy default). CMake via `ament_cmake`.
- **Launch files:** Python launch only — no XML launch unless copying
  from an upstream template.
- **Parameter defaults:** YAML > launch arg > node default. Keep all
  three in sync for every parameter you touch.
- **Commits:** Imperative mood, scoped prefix (`nav:`, `docker:`,
  `ekf:`, `roboclaw:`). Co-author tag as usual. Never
  `git push --force`. Never squash-rebase already-pushed commits.
- **Reverts:** Use `git revert` (not `reset --hard`) once a commit
  is on the remote. Preserves history for future diagnosis.
- **No new Markdown docs** unless Julian asks. Put findings into
  memory files or this CLAUDE.md.

---

## Known issues

- Collision monitor bypassed (see velocity-flow section). Re-enabling
  takes more than a one-line change and previously broke nav.
- Map appears to rotate with robot in RViz — may be just RViz Fixed
  Frame setting, or may need scan matching. Unresolved.
- GPS datum hardcoded to New Mexico.
- `gps_waypoint_handler_node` uses blocking
  `spin_until_future_complete` in constructor.
- Phantom obstacles on costmap.

---

## When you're stuck

Check in this order:

1. This file.
2. Memory: `~/.claude/projects/-home-julian-WALL-E/memory/MEMORY.md`
   and the individual memory files it indexes.
3. The actual source. Read it — don't assume.
4. Git log on the relevant file (`git log -p <path>`). Past commits
   often explain why something looks "wrong".
5. Only then propose a change, and quote file:line.

## Project skill file

`.claude/skills/walle.md` is a living skill file updated over time
with performance-improving lessons for this project. Read it at the
start of any non-trivial task.
