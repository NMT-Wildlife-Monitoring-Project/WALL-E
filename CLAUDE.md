# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

WALL-E (Wildlife Activity Life Explorer) is an autonomous wildlife monitoring robot built on ROS 2 Jazzy. It runs in Docker on Jetson Orin Nano or Raspberry Pi 5. The robot uses differential drive (RoboClaw motor controller), RPLiDAR, BNO085 IMU, and GPS for autonomous navigation in outdoor environments.

## Build & Run

Everything runs inside Docker. The workspace is at `ros2_ws/`.

```bash
# Build Docker image (from repo root or docker/)
cd docker && ./start_docker.sh -b

# Run modes (pick one action flag):
./start_docker.sh -s          # Full system (robot_launch.py)
./start_docker.sh -t          # Teleop only (joystick)
./start_docker.sh -m          # Motors only
./start_docker.sh -c "cmd"    # Custom command
./start_docker.sh              # Interactive shell (default)
./start_docker.sh -d          # Enable X11 display forwarding (combine with other flags)

# Inside container, manual build:
cd /home/walle/ros2_ws && colcon build --symlink-install && source install/setup.bash
```

There are no linters, formatters, or test suites configured.

## Architecture

### Velocity Command Flow (most critical path)

```
Joystick → joy_linux → /joy → teleop_twist_joy → /cmd_vel_teleop ─┐
                                                                    ├→ twist_mux → /cmd_vel_out → roboclaw_node → motors
Nav2 MPPI controller ──────────────────────────→ /cmd_vel_nav ─────┘
```

**Key detail:** twist_mux publishes to `/cmd_vel_out` by default (regardless of its yaml `cmd_vel_out` setting). RoboClaw subscribes via remapping: `('cmd_vel', '/cmd_vel_out')` in `roboclaw_launch.py`.

Teleop has priority 20, Nav2 has priority 10. Teleop wins when both are active.

### Localization (Dual EKF)

Two `robot_localization` EKF nodes run in parallel (`dual_ekf_navsat.launch.py`):
- **ekf_filter_node_odom** → fuses wheel odometry + RF2O laser odometry + IMU → publishes `/odometry/local` (odom frame)
- **ekf_filter_node_map** → fuses the above + GPS → publishes `/odometry/global` (map frame)
- **navsat_transform** → converts GPS fixes into the map frame

### Launch Hierarchy

`robot_launch.py` (robot_bringup) is the master launch file with boolean toggles:
- `launch_rplidar` (default: true) → sllidar_ros2
- `launch_bno085` (default: true) → BNO085 IMU driver
- `launch_gps` (default: false) → NMEA GPS driver
- `launch_nav` (default: true) → `gps_waypoint_follower.launch.py` which starts: dual EKF, twist_mux, Nav2, and optionally RViz/mapviz/waypoint follower
- `launch_d2oc` (default: false) → D2OC exploration algorithm
- RoboClaw driver always launches (no toggle)
- RF2O laser odometry launches when nav is enabled

### ROS 2 Packages (ros2_ws/src/)

| Package | Type | Role |
|---------|------|------|
| `roboclaw_driver` | Python | Motor control, wheel odometry, encoder management |
| `robot_bringup` | CMake | Master launch file, URDF/xacro robot description |
| `robot_navigation` | Python | Nav2 config, twist_mux config, dual EKF launch, costmap params |
| `robot_teleop` | Python | Joystick teleoperation (Logitech F710) |
| `d2oc_algorithm` | Python | Entropy-based frontier exploration |
| `bno085_driver` | Python | BNO085 IMU driver (I2C) |
| `sllidar_ros2` | CMake | RPLiDAR driver |
| `nmea_navsat_driver` | Python | GPS/GNSS driver |
| `robot_messages` | CMake | Custom msgs (RoboclawStatus) |
| `waypoint_server` | Python | GPS waypoint loading and following |
| `scan_matcher` | CMake (C++) | ICP scan matching |
| `robot_web_interface` | - | Web interface |

### RoboClaw Parameters

All motor parameters are in `roboclaw_driver/config/roboclaw_params.yaml`. Key values derived from physical measurement and BasicMicro auto-tune:
- Wheel separation: 0.39m, wheel diameter: 0.095m
- Encoder: 6533 QPPR (quadrature pulses per revolution)
- Max hardware speed: 25410 QPPS (from BasicMicro, uses lower of M1/M2)
- M1 is reversed (`m1_reverse: true`), M2 is not
- Serial: `/dev/roboclaw` at 9600 baud, address 128

Parameters load from YAML first, then launch arguments can override. When changing motor parameters, update the YAML config file -- launch arg defaults and node defaults should stay in sync.

### Docker Details

- Image: `walle/ros2:jazzy`
- Runs `--privileged` with `--net=host` and full `/dev` mount for hardware access
- ROS_DOMAIN_ID defaults to 62
- `BLINKA_FORCEBOARD=RaspberryPi5` is set for Adafruit libraries
- udev rules in `scripts/99-walle-devices.rules` create `/dev/roboclaw` symlink

## Known Issues

- Motor speed oscillation during Nav2 navigation (likely encoder noise or RoboClaw PID tuning)
- GPS datum hardcoded to New Mexico
- `gps_waypoint_handler_node` uses blocking `spin_until_future_complete` in constructor

## Working Branch

Development happens on `oldcodetest`. Don't push directly to `main`.
