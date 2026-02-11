# WALL-E Robot: Complete Operating Manual

**Wildlife Activity Life Explorer** - Full Operational Guide

Last Updated: February 2026  
ROS 2 Distribution: Jazzy  
Hardware Target: Jetson Orin Nano with dual motors, LiDAR, IMU, GPS, camera

---

## Table of Contents

1. [System Architecture Overview](#system-architecture-overview)
2. [Pre-Requisites and Installation](#pre-requisites-and-installation)
3. [Docker Container Management](#docker-container-management)
4. [Core Robot Operations](#core-robot-operations)
5. [Sensor Operation](#sensor-operation)
6. [Navigation and Autonomy](#navigation-and-autonomy)
7. [Teleoperation Control](#teleoperation-control)
8. [Web Interface Control](#web-interface-control)
9. [Debugging and Monitoring](#debugging-and-monitoring)
10. [Troubleshooting](#troubleshooting)

---

## System Architecture Overview

### Hardware Components

| Component | Type | Interface | Purpose |
|-----------|------|-----------|---------|
| **Jetson Orin Nano** | ARM CPU (6-core) | System | Main compute/control |
| **RoboClaw Motor Driver** | Motor Controller | Serial `/dev/roboclaw` | Dual motor drive system |
| **Logitech F710** | Gamepad | `/dev/input/js*` | Teleoperation input |
| **S-Lidar S3** | 2D LiDAR Scanner | Serial | 360° environment sensing (5-10 Hz) |
| **BNO085** | IMU | I2C/Serial | Inertial measurement (9-DOF) at 100 Hz |
| **u-blox NEO-M9N** | GPS/GNSS | Serial `/dev/ttyUSB*` | Global positioning (1 Hz) |
| **USB Camera** | Webcam | `/dev/video*` | Live video feed |

### Software Stack

```
ROS 2 Jazzy (Robot Operating System)
  ├── Drivers Layer
  │   ├── roboclaw_driver - Motor control
  │   ├── sllidar_ros2 - LiDAR interface
  │   ├── bno085_driver - IMU sensor
  │   ├── nmea_navsat_driver - GPS receiver
  │   └── joy_linux - Joystick input
  ├── Localization Layer
  │   ├── robot_localization (EKF) - Sensor fusion
  │   ├── navsat_transform - GPS→local frame conversion
  │   └── scan_matcher - ICP-based odometry refinement
  ├── Navigation Layer
  │   ├── Nav2 Stack - Autonomous navigation
  │   ├── MPPI Controller - Trajectory generation
  │   ├── Costmap Layer - Obstacle representation
  │   └── Planner - Path computation
  ├── Control Layer
  │   ├── waypoint_server - GPS waypoint management
  │   ├── twist_mux - Command arbitration
  │   ├── velocity_smoother - Motion smoothing
  │   └── teleop_twist_joy - Manual control input
  └── Interface Layer
      ├── web_app - FastAPI backend + React frontend
      ├── rosbridge - WebSocket bridge
      └── RViz2 - Visualization
```

### ROS 2 Data Flow Diagram

```
SENSORS:
  [IMU: 100 Hz] ─┐
  [GPS: 1 Hz]   ├─→ EKF Localization ─→ [Odometry] ─→ Transform Tree (TF2)
  [Lidar: 5 Hz] │                           ↓
  [Motors: 20]  └─→ [Scan Matcher]─────→ [Pose]
                                            ↓
INPUTS:
  [Joystick]  ─→ teleop_twist_joy ─→ twist_mux ←─ Nav2 Controller
  [Waypoints] ─→ waypoint_server    │           ↓
                                    └→ velocity_smoother
                                            ↓
MOTOR CONTROL:
                                    RoboClaw Motor Driver ─→ [Motors]
```

---

## Pre-Requisites and Installation

### 1. System Requirements

**Host Machine:**
- Linux OS (Ubuntu 22.04 LTS or Jetson OS for target robot)
- Docker installed and running
- Current user added to docker group
- 20+ GB disk space (for ROS 2 image)
- SSH key configured with GitHub (optional, for cloning via SSH)

**Target Robot (Jetson Orin Nano):**
- 8 GB RAM minimum
- Jetpack 5.x or later
- udev rules installed for hardware access

### 2. Install Docker

```bash
# On host machine (Ubuntu/Debian)
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# Add current user to docker group
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker

# Verify Docker works without sudo
docker run hello-world
```

### 3. Clone and Setup Repository

```bash
# Clone the repository
git clone https://github.com/NMT-Wildlife-Monitoring-Project/WALL-E.git
cd WALL-E

# Install udev rules for hardware device access
cd scripts
sudo cp 99-walle-devices.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

# Return to repo root
cd ..
```

### 4. Build Docker Image

The Docker image contains:
- ROS 2 Jazzy base system
- All ROS 2 packages (Nav2, robot_localization, etc.)
- Custom WALL-E packages
- Development tools and dependencies
- Web app (Node.js + React frontend + FastAPI backend)

```bash
cd docker
./start_docker.sh -b
```

**Expected Output:**
```
Building walle/ros2:jazzy...
[Build progress...]
Successfully built walle/ros2:jazzy
```

**Build Time:** 10-20 minutes (first time), depends on internet speed

**Storage:** ~8-10 GB

---

## Docker Container Management

### Overview of `start_docker.sh`

The main interface for running the robot. Manages Docker container lifecycle, command execution, device mapping, and environment setup.

### Quick Reference: `start_docker.sh` Flags

#### Core Robot Operations

```bash
./start_docker.sh -r              # Start robot bringup (all sensors + motors)
./start_docker.sh -n              # Start full navigation stack (GPS waypoint following)
./start_docker.sh -t              # Start joystick teleoperation control
```

#### Individual Sensors

```bash
./start_docker.sh -l              # LiDAR scanner only
./start_docker.sh -c              # USB camera only
./start_docker.sh --imu           # IMU sensor only
./start_docker.sh --gps           # GPS receiver only
./start_docker.sh --imu-cal       # Calibrate IMU magnetometer (20 second rotation)
```

#### Visualization & Debugging

```bash
./start_docker.sh -R -d           # RViz visualization (requires -d for display)
./start_docker.sh -v -d           # LiDAR view in RViz
./start_docker.sh -B              # ROSBridge WebSocket server
./start_docker.sh --scan-matcher  # ICP scan matching node
./start_docker.sh --waypoint-server  # Waypoint management server
./start_docker.sh --ekf           # Dual EKF localization (map + odometry frames)
./start_docker.sh -m              # Motor control (RoboClaw driver)
```

#### Web Interface

```bash
./start_docker.sh -w              # Start web control panel (backend on :8000)
./start_docker.sh -B -w           # ROSBridge + web app (full web control)
```

#### Container Management

```bash
./start_docker.sh                 # Interactive bash shell in container
./start_docker.sh --command "ros2 topic list"  # Run single command
./start_docker.sh -x              # Stop running container
./start_docker.sh -R --restart    # Restart container
./start_docker.sh -k              # Clean Docker system (prune images/volumes)
```

#### Advanced Options

```bash
./start_docker.sh -d              # Enable X11 display forwarding (for RViz on host)
./start_docker.sh -q              # Quiet mode (run in background)
./start_docker.sh -i 100          # Set ROS_DOMAIN_ID to 100 (for network isolation)
```

### Common Usage Patterns

#### Pattern 1: Quick Robot Test (Local Machine)

```bash
cd docker
./start_docker.sh -r              # Start robot bringup
# In another terminal:
./start_docker.sh --command "ros2 topic list"  # Check topics
./start_docker.sh -x              # Stop when done
```

#### Pattern 2: Full Navigation with Visualization

```bash
cd docker
# Terminal 1 - Start navigation and RViz
./start_docker.sh -n -R -d

# Terminal 2 - Monitor topics
./start_docker.sh --command "ros2 topic echo /odom"
```

#### Pattern 3: Teleoperation with Motor Feedback

```bash
cd docker
# Terminal 1 - Start robot with motors and joystick
./start_docker.sh -r -t

# Terminal 2 - Monitor motor status
./start_docker.sh --command "ros2 topic echo /roboclaw_status"
```

#### Pattern 4: Web-based Control

```bash
cd docker
./start_docker.sh -B -w -q        # Start ROSBridge + web app in background
# Then open http://localhost:8000 in browser
```

---

## Core Robot Operations

### 1. Robot Bringup (Basic Hardware Startup)

**Purpose:** Start all core systems (sensors + motors) without navigation

```bash
./start_docker.sh -r
```

**What Gets Started:**
- **Motors:** RoboClaw driver (wheel odometry at 20 Hz)
- **Sensors:**
  - LiDAR scanner (S-Lidar S3, ~5-10 Hz)
  - IMU (BNO085, 100 Hz updates)
  - GPS receiver (1 Hz fix updates)
- **TF2 Tree:** robot_state_publisher (URDF-based transforms)
- **Logs:** All nodes output to screen

**Expected Output:**
```
[bno085_node] IMU initialized
[sllidar_node] Connected to LiDAR on /dev/ttyUSB0
[roboclaw_node] Motor driver connected at /dev/roboclaw
[robot_state_publisher] Publishing robot description
```

**Verify Functionality:**
```bash
# In separate terminal
./start_docker.sh --command "ros2 topic list"
```

**Expected Topics:**
- `/scan` - LiDAR data
- `/odom` - Wheel encoder odometry
- `/imu/data` - Raw IMU measurements
- `/fix` - GPS fix information
- `/tf` - Transform frames
- `/tf_static` - Static transforms (URDF)

**Typical Issues & Fixes:**

| Error | Cause | Fix |
|-------|-------|-----|
| Cannot open `/dev/roboclaw` | Motor driver not connected | Check serial cable, verify udev rules installed |
| Cannot open `/dev/ttyUSB*` | GPS/LiDAR not connected | Check USB devices: `lsusb` |
| BNO085 initialization failed | I2C bus error or address mismatch | Verify I2C connection, check i2cdetect output |
| High CPU usage (>80%) | ICP scan matcher running | Normal at startup; stabilizes after 30s |

### 2. Full Navigation Stack

**Purpose:** Enable GPS-based waypoint navigation with localization and obstacle avoidance

```bash
./start_docker.sh -n
```

**What Gets Started (in addition to robot bringup):**
- **Localization:**
  - Dual EKF filters (odometry frame + map frame)
  - NavSat transform (GPS→local conversion)
  - ICP scan matcher (LiDAR odometry refinement)
- **Navigation:**
  - Nav2 planner (path computation)
  - MPPI controller (trajectory generation)
  - Costmap layers (local/global obstacle representation)
  - Behavior tree executor
- **Utilities:**
  - Twist mux (command arbitration)
  - Velocity smoother (motion smoothing)
  - Waypoint server (GPS waypoint management)

**Expected Output:**
```
[localization_filter_map] Starting EKF filter
[navsat_transform] Datum set to [34.0658, -106.908, 0.0]
[navigation] Nav2 BringUp started
[planner_server] MPPI planner initialized
[controller_server] MPPI controller ready
```

**Key Topics (in addition to bringup):**
- `/map` - Global costmap
- `/local_costmap/costmap` - Local costmap
- `/global_plan` - Planned path
- `/plan` - Full trajectory
- `/goal_pose` - Current waypoint target
- `/nav2_feedback` - Navigation status

**Verify Navigation Stack:**
```bash
./start_docker.sh --command "ros2 topic echo /nav2_feedback"
```

**Typical Startup Issues:**

| Issue | Cause | Fix |
|-------|-------|-----|
| "Transform timeout" errors | EKF not publishing transforms fast enough | Wait 5-10s for filters to converge |
| "Costmap cannot get robot pose" | GPS not converging | Outdoor area required, wait for GPS fix |
| "Robot cannot find path" | Antenna visible to LiDAR as obstacle | Adjust footprint/inflation or mount antenna differently |
| Very high initial CPU (>90%) | ICP scan matcher initialization | Normal; CPU should drop after 30-60s |

### 3. Teleoperation Control

**Purpose:** Drive the robot manually using Logitech F710 controller

```bash
./start_docker.sh -t
```

**What Gets Started:**
- Robot bringup (sensors + motors)
- Joy Linux node (F710 controller input at 10 Hz)
- teleop_twist_joy converter (joystick→velocity commands)

**Controller Button Mapping:**

| Control | Function |
|---------|----------|
| **Left Stick Vertical** | Forward/backward movement |
| **Left Stick Horizontal** | Left/right turning |
| **LB (Left Bumper)** | **ENABLE BUTTON** - Must hold to move |
| **RB (Right Bumper)** | Turbo mode (currently disabled) |

**Motion Parameters:**
- **Max Forward Speed:** 1.0 m/s
- **Max Turn Speed:** 3.0 rad/s
- **Deadzone:** 0.05 (prevents stick drift)
- **Safety:** LB button must be held continuously; releasing stops robot

**Drive Instructions:**

1. **Prepare:**
   - Connect Logitech F710 to robot (USB or wireless dongle)
   - Verify `/dev/input/js0` exists: `ls /dev/input/js0`
   - Start teleoperation: `./start_docker.sh -t`

2. **Manual Control:**
   - **Hold LB button** (critical for safety)
   - Move **left stick forward** → robot moves forward
   - Move **left stick backward** → robot moves backward
   - Move **left stick left** → robot turns left
   - Move **left stick right** → robot turns right
   - **Release LB button** → robot stops immediately

3. **Verify Joystick is Working:**
   ```bash
   ./start_docker.sh --command "ros2 topic echo /joy"
   ```
   You should see axis/button values changing as you move the stick.

**Common Issues:**

| Issue | Cause | Fix |
|-------|-------|-----|
| Joystick not detected | `/dev/input/js*` missing | Reconnect controller, check `lsusb` |
| Robot doesn't respond to input | LB button not held | Hold LB while moving stick |
| Joystick moves but robot doesn't | RoboClaw not connected | Verify motor driver serial connection |
| Stick feels "dead" in center | Deadzone setting too high | Reduce `deadzone` in `joy_linux_f710.yaml` |

---

## Sensor Operation

### Individual Sensor Startup

Each sensor can be started independently for testing and calibration.

### 1. LiDAR Scanner (S-Lidar S3)

```bash
./start_docker.sh -l              # Start LiDAR only
./start_docker.sh -v -d           # Visualize LiDAR in RViz
```

**Specifications:**
- Model: S-Lidar S3 (LDROBOT)
- Range: 0.2 - 30 m
- Update Rate: ~5-10 Hz
- Resolution: ~360°

**Published Topic:**
```
/scan (sensor_msgs/LaserScan)
  - header.frame_id: "laser"
  - angle_min: -3.14159
  - angle_max: 3.14159
  - range_min: 0.2
  - range_max: 30.0
  - ranges[]: distance measurements (float array)
```

**Monitor LiDAR:**
```bash
./start_docker.sh --command "ros2 topic echo /scan --once"
```

**Visual Inspection:**
```bash
./start_docker.sh -l -R -d          # RViz visualization
```

**Common Issues:**

| Issue | Fix |
|-------|-----|
| No `/scan` topic | Check serial connection: `lsusb` |
| High ROS warnings "Input buffer overflow" | LiDAR publishing faster than processing; normal on Jetson |
| Inverted/rotated scan | Check URDF mounting angle |

---

### 2. IMU Sensor (BNO085)

```bash
./start_docker.sh --imu           # Start IMU
./start_docker.sh --imu-cal       # Run magnetometer calibration
```

**Specifications:**
- Model: BNO085 (Bosch Sensortec)
- Update Rate: 100 Hz
- Axes: 9-DOF (3-axis accelerometer, gyroscope, magnetometer)
- Connection: I2C (0x4A default) or Serial

**Published Topics:**
```
/imu/data (sensor_msgs/Imu)
  - linear_acceleration (m/s²)
  - angular_velocity (rad/s)
  - orientation (quaternion)

/imu/mag (sensor_msgs/MagneticField)
  - magnetic_field (Tesla)
```

**Verify IMU:**
```bash
./start_docker.sh --command "ros2 topic echo /imu/data --once"
```

**Expected Values:**
- Linear acceleration: ~9.81 m/s² (gravity) on Z axis when still
- Angular velocity: ~0 when still
- Magnetometer: varies by location (~25-60 µT)

**Magnetometer Calibration:**

Required for accurate heading estimates. Do this in your deployment location (magnetic field varies geographically).

```bash
./start_docker.sh --imu-cal
```

**Calibration Procedure:**
1. Place robot in open area (away from metal)
2. Slowly rotate robot 360° while tilting all axes (30 seconds)
3. Monitor calibration status in logs
4. Calibration saves automatically to IMU firmware

**Common Issues:**

| Issue | Fix |
|-------|-----|
| "I2C communication error" | Check i2cdetect output: `i2cdetect -y 1` |
| Orientation drifting | Run magnetometer calibration |
| High noise in gyro/accel | IMU mounting loose; check mechanical stability |

---

### 3. GPS/GNSS Receiver (u-blox NEO-M9N)

```bash
./start_docker.sh --gps           # Start GPS receiver
```

**Specifications:**
- Model: u-blox NEO-M9N
- Fix Rate: 1 Hz
- Accuracy: ±2.5 m (typical, depends on environment)
- Connection: Serial (baud: 38400)

**Published Topics:**
```
/fix (sensor_msgs/NavSatFix)
  - latitude, longitude, altitude
  - position_covariance (uncertainty estimate)
  - status.status: -1 (no fix), 0 (fix), 1 (SBAS), 2 (DGPS), etc.

/fix_velocity (geometry_msgs/TwistWithCovariance)
  - velocity in ECEF frame (ENU converted locally)

/diagnostics
  - GPS satellite count, DOP values, etc.
```

**Verify GPS:**
```bash
./start_docker.sh --command "ros2 topic echo /fix"
```

**Expected Output (when outdoors with clear sky):**
```yaml
latitude: 34.0658
longitude: -106.908
altitude: 1850.0
status: 0  # 0 = fixed
position_covariance:
  - 6.0  # ~2.5m std dev squared
  - 0.0
  - 6.0
```

**GPS Datum Configuration:**

The navigation system requires a reference point (datum) to convert GPS coordinates to local XY coordinates. Default datum is New Mexico Tech's location; change it for other sites:

**Set Datum via Launch Parameter:**
```bash
./start_docker.sh --command "ros2 launch robot_navigation gps_waypoint_follower.launch.py datum_lat:=40.1234 datum_lon:=-104.5678 datum_alt:=1500"
```

**Or Edit Default in Config:**
File: `ros2_ws/src/robot_navigation/config/dual_ekf_navsat_params.yaml`
```yaml
navsat_transform:
  ros__parameters:
    datum: [40.1234, -104.5678, 1500]  # lat, lon, alt (meters)
```

**Common GPS Issues:**

| Issue | Cause | Fix |
|-------|-------|-----|
| No `/fix` topic | GPS not connected or powered | Check serial port: `ls /dev/ttyUSB*` |
| Status = -1 (no fix) | Indoors or no sky view | Move to outdoor area with clear view |
| Lat/lon jumping erratically | Poor satellite geometry or multipath | Wait longer, move to open area |
| "Datum mismatch" warnings | Using default datum at wrong location | Set correct datum for deployment site |

---

### 4. USB Camera

```bash
./start_docker.sh -c              # Start USB camera
```

**Published Topic:**
```
/image_raw (sensor_msgs/Image)
  - format: BGR8 or MJPEG
  - resolution: varies by camera (typically 640×480 or 1280×720)
```

**Verify Camera:**
```bash
./start_docker.sh --command "ros2 topic echo /image_raw --once"
```

**View Camera Feed (web app):**
```bash
./start_docker.sh -B -w           # Start web interface
# Then open http://localhost:8000 in browser
# Camera feed displayed in real-time on web dashboard
```

---

## Navigation and Autonomy

### GPS Waypoint Navigation

**Purpose:** Autonomous navigation between GPS coordinates with obstacle avoidance

```bash
./start_docker.sh -n              # Full navigation stack
```

### Navigation System Components

#### 1. Localization (EKF Sensor Fusion)

```
GPS + Odometry + IMU + LiDAR → [EKF Filters] → Robot Pose
```

The system uses **dual EKF filters:**

- **Odometry Filter (`odom` frame):**
  - Inputs: wheel odometry (20 Hz), IMU (100 Hz)
  - Output: drift-free short-term pose
  - Used for: local planning, real-time control

- **Map Filter (`map` frame):**
  - Inputs: GPS (1 Hz), odometry, IMU
  - Output: global pose synchronized with GPS
  - Used for: long-term navigation, waypoint targeting

**Verify EKF is Running:**
```bash
./start_docker.sh --command "ros2 topic echo /odometry/filtered"
```

**Expected Output:**
```yaml
header:
  frame_id: "odom"
pose:
  pose:
    position:
      x: 0.0
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.7071  # varies as robot moves/turns
      w: 0.7071
```

#### 2. Scan Matching (ICP Odometry Refinement)

Optional component that improves pose estimates using laser scan matching.

```bash
./start_docker.sh --scan-matcher
```

**How It Works:**
- Compares consecutive LiDAR scans to estimate rotation/translation
- Outputs refined odometry (`/odom_matched`)
- Merged into EKF for better pose accuracy

**Configuration:** `ros2_ws/src/scan_matcher/config/scan_matcher.yaml`
```yaml
scan_matcher:
  ros__parameters:
    max_iterations: 20           # ICP convergence iterations
    max_correspondence_distance: 0.5  # Match search radius
    min_scan_range: 0.1          # Filter short-range noise
    max_scan_range: 10.0         # Ignore far points
```

**Note:** CPU-intensive; may reduce control update rate on Jetson Orin Nano.

#### 3. Nav2 Planning and Control

**Nav2 Planner:**
- Algorithm: MPPI (Model Predictive Path Integral)
- Function: Generates smooth, collision-free trajectories
- Update Rate: 1 Hz
- Trajectory Horizon: 5 seconds (adjustable)

**Nav2 Controller:**
- Algorithm: MPPI trajectory tracking
- Function: Converts plan into motor commands
- Update Rate: 20 Hz
- Samples: 2000 trajectory candidates per decision

**Costmap (Obstacle Representation):**

Two layers:
- **Local Costmap:** 10×10 m, 0.05 m resolution, updates at 5 Hz
  - Centered on robot
  - Used for immediate obstacle avoidance
  
- **Global Costmap:** 100×100 m, 0.2 m resolution, updates at 1 Hz
  - Rolling window centered on robot
  - Used for long-term path planning

### Setting Waypoints

#### Method 1: Via Web Interface

```bash
./start_docker.sh -B -w           # Start web interface
# Open http://localhost:8000
# Use Google Maps to click waypoints
# Click "Send Waypoints" to start navigation
```

#### Method 2: Via Command Line

**Waypoint YAML Format:**
File: `ros2_ws/src/waypoint_server/config/waypoints.yaml`
```yaml
waypoints:
  - name: "North Field"
    latitude: 34.0668
    longitude: -106.908
    altitude: 1850
    heading: 0.0  # degrees (optional)
    
  - name: "Forest Boundary"
    latitude: 34.0650
    longitude: -106.905
    altitude: 1845
    heading: 90.0
```

**Start Navigation with Waypoints:**
```bash
./start_docker.sh -n              # Full navigation stack (waypoints auto-loaded)
```

**Check Current Waypoint:**
```bash
./start_docker.sh --command "ros2 topic echo /goal_pose"
```

#### Method 3: Programmatically (ROS 2 Action)

```bash
# Send single goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{
    pose: {
      header: {frame_id: "map"},
      pose: {
        position: {x: 100.0, y: 50.0, z: 0.0},
        orientation: {x: 0, y: 0, z: 0.7071, w: 0.7071}
      }
    }
  }'
```

### Monitoring Navigation

**Navigation Status:**
```bash
./start_docker.sh --command "ros2 topic echo /nav2_feedback"
```

**Expected Output:**
```yaml
current_pose:
  pose:
    position: {x: 5.3, y: 2.1, z: 0.0}
    orientation: {x: 0, y: 0, z: 0.6, w: 0.8}
number_of_recoveries: 0
distance_remaining: 45.2
estimated_time_remaining: 120.0  # seconds
navigation_time: 30.0
cycles_since_last_recovery: 5
```

**Plan Visualization:**
```bash
./start_docker.sh -R -d           # RViz with default config
# Shows:
# - Global and local costmaps
# - Current robot position (colored circle)
# - Planned path (colored line)
# - Local trajectories (faint lines)
```

### Navigation Parameters

**File:** `ros2_ws/src/robot_navigation/config/nav2_no_map_params.yaml`

**Key Tuning Parameters:**

```yaml
# Robot footprint (collision detection)
footprint: [[0.16, 0.15], [-0.16, 0.15], [-0.16, -0.15], [0.16, -0.15]]
footprint_padding: 0.01

# Inflation layer (buffer around obstacles)
inflation_radius: 0.24          # 24 cm buffer
cost_scaling_factor: 3.0        # Cost increases near obstacles

# Local planning
local_costmap:
  width: 200                     # 10m at 0.05m resolution
  height: 200
  resolution: 0.05
  update_frequency: 5.0          # Hz
  publish_frequency: 3.0

# Global planning  
global_costmap:
  width: 500                     # 100m at 0.2m resolution
  height: 500
  resolution: 0.2
  update_frequency: 1.0
  publish_frequency: 0.5

# MPPI Controller
controller_frequency: 20.0       # Hz command rate
batch_size: 100                  # Reduce if CPU-limited (default 2000)
time_steps: 25                   # Planning horizon (5s at 20Hz)
```

**Common Navigation Tuning Issues:**

| Issue | Parameter | Fix |
|-------|-----------|-----|
| Robot gets stuck on antenna | `inflation_radius` too large | Reduce to 0.15 m |
| Path goes through obstacles | `inflation_radius` too small | Increase to 0.30 m |
| Robot stops randomly | `controller_frequency` too high | Reduce batch_size or increase time_steps |
| Very slow path planning | `batch_size` too large | Reduce to 100-200 |
| Robot overshoots turns | `time_steps` too small | Increase to 40-50 |

---

## Teleoperation Control

### Detailed Logitech F710 Controller Setup

**Prerequisites:**
- Logitech F710 connected (USB or wireless dongle)
- Robot bringup running: `./start_docker.sh -r`
- Teleop node running: `./start_docker.sh -t`

### Controller Button Mapping Reference

**Physical Buttons:**
```
         Y (Yellow)
     X       B (Red)
       A (Green)
    
    LB              RB
    LT              RT
    
    (L3)       (R3)
    ⬇              ⬇
    [←→ Left Stick] [←→ Right Stick]
    [↑↓]          [↑↓]
```

**F710 Button → ROS Mapping:**

| Physical Button | ROS Index | Function |
|-----------------|-----------|----------|
| A (Green) | Button 0 | (unused) |
| B (Red) | Button 1 | (unused) |
| X (Blue) | Button 2 | (unused) |
| Y (Yellow) | Button 3 | (unused) |
| LB | Button 4 | **ENABLE BUTTON** |
| RB | Button 5 | Turbo (disabled) |
| Back | Button 6 | (unused) |
| Start | Button 7 | (unused) |
| L3 | Button 9 | (unused) |
| R3 | Button 10 | (unused) |

**Axis Mapping:**

| Control | ROS Axis | Range | Purpose |
|---------|----------|-------|---------|
| Left Stick Horizontal | 0 | -1.0 to 1.0 | Turning |
| Left Stick Vertical | 1 | -1.0 to 1.0 | Forward/back |
| LT Trigger | 2 | 0.0 to 1.0 | (unused) |
| Right Stick Horizontal | 3 | -1.0 to 1.0 | (unused) |
| Right Stick Vertical | 4 | -1.0 to 1.0 | (unused) |
| RT Trigger | 5 | 0.0 to 1.0 | (unused) |
| D-Pad Horizontal | 6 | -1.0 to 1.0 | (unused) |
| D-Pad Vertical | 7 | -1.0 to 1.0 | (unused) |

### Configuration File

**Location:** `ros2_ws/src/robot_teleop/config/joy_linux_f710.yaml`

```yaml
# ============================================================================
# Logitech F710 Controller Configuration
# ============================================================================

# Linux joystick settings
joy_linux_node:
  ros__parameters:
    device_name: "/dev/input/js0"  # Default joystick device
    deadzone: 0.05                  # Prevents drift from stick center
    autorepeat_rate: 10.0          # Hz
    coalesce_interval_ms: 20       # Milliseconds

# Velocity conversion settings
teleop_twist_joy:
  ros__parameters:
    # Axis configuration
    axis_linear:
      x: 1                          # Left stick vertical (forward/back)
    axis_angular:
      yaw: 0                        # Left stick horizontal (turn)
    
    # Speed scaling
    scale_linear:
      x: 1.0                        # Max forward speed: 1.0 m/s
    scale_angular:
      yaw: 3.0                      # Max turn speed: 3.0 rad/s
    
    # Safety
    enable_button: 4                # LB button must be held
    require_enable_button: true     # Mandatory safety feature
    
    # Tuning (disabled)
    publish_stamped_twist: false    # Use TwistStamped messages
```

### How to Drive the Robot

**Step-by-Step:**

1. **Power on robot** and start teleoperation:
   ```bash
   ./start_docker.sh -t
   ```

2. **Verify joystick is detected:**
   ```bash
   ./start_docker.sh --command "ls -la /dev/input/js0"
   ```

3. **Hold LB button** (Left Bumper) - This is critical!

4. **Move left stick:**
   - **Forward:** Push stick up → robot moves forward
   - **Backward:** Pull stick down → robot moves backward
   - **Left turn:** Push stick left → robot turns left
   - **Right turn:** Push stick right → robot turns right
   - **Stop:** Release LB button → robot stops

5. **Fine-tuning:**
   - Small stick deflection = slower speed
   - Full stick deflection = maximum speed
   - Combination movements: diagonal stick = forward + turn

### Modifying Joystick Parameters

**To increase/decrease speed:**
Edit `joy_linux_f710.yaml`:
```yaml
scale_linear:
  x: 1.5                # Increase to 1.5 m/s
scale_angular:
  yaw: 6.0              # Increase to 6.0 rad/s
```

Then reload configuration:
```bash
./start_docker.sh -x           # Stop current
./start_docker.sh -t           # Restart with new config
```

**To disable LB enable requirement (NOT RECOMMENDED):**
```yaml
require_enable_button: false
```

**To reduce deadzone (stick is too sensitive):**
```yaml
deadzone: 0.01                # Smaller = more sensitive
```

### Safety Features

1. **LB Enable Button:** Prevents accidental movement
   - Must be held continuously to move
   - Releasing immediately stops robot
   - Enforced in `require_enable_button: true`

2. **Timeout Protection:** If joystick signal lost for >1 second:
   - Motor commands stop
   - RoboClaw enters safe state
   - Manual restart required

3. **Speed Limits:**
   - Forward: capped at 1.0 m/s
   - Turning: capped at 3.0 rad/s
   - Prevents unintended high-speed movement

4. **Antenna Detection:** Nav2 costmap inflates obstacles around antenna:
   - If antenna visible to LiDAR, it's treated as obstacle
   - Robot cannot command through antenna
   - Prevents self-collision

### Troubleshooting Joystick Issues

**Joystick not detected:**
```bash
# Check device file exists
ls /dev/input/js*

# If no device, reconnect controller:
# - Unplug USB/wireless dongle
# - Wait 2 seconds
# - Reconnect
```

**Joystick detected but robot doesn't move:**
```bash
# Verify you're holding LB button
# Check if motor driver is connected:
./start_docker.sh --command "ros2 topic echo /roboclaw_status"

# Check joy/cmd_vel messages:
./start_docker.sh --command "ros2 topic echo /cmd_vel"
```

**Stick feels "dead" (requires large deflection):**
- Deadzone too high
- Reduce `deadzone: 0.02` (from 0.05)
- Or recalibrate joystick in your OS

**Robot moves erratically with stick:**
- Joystick drifting (calibration needed)
- Try OS-level calibration: `jstest /dev/input/js0`
- Or reduce sensitivity by lowering `scale_linear/scale_angular`

---

## Web Interface Control

### Web Dashboard Overview

The WALL-E web interface provides:
- **Live Camera Feed** - Real-time MJPEG video stream
- **GPS Map Integration** - Google Maps with satellite view
- **Costmap Visualization** - Local costmap overlay on Leaflet map
- **Robot Statistics** - Temperature, battery, solar power, motor speeds
- **Manual Joystick Control** - On-screen virtual joystick
- **Waypoint Management** - Create/edit GPS waypoints via map
- **ROS2 Integration** - Full WebSocket connectivity via ROSBridge

### Starting the Web Interface

**Terminal 1 - Start ROSBridge and Backend:**
```bash
./start_docker.sh -B -w
```

This launches:
- ROSBridge server (WebSocket port 9090)
- FastAPI backend (HTTP port 8000)

**Terminal 2 - (Optional) Start Navigation:**
```bash
./start_docker.sh -n              # Full navigation with GPS waypoints
```

**Open Web Browser:**
```
http://localhost:8000
```

### Web Interface Layout

```
╔════════════════════════════════════════════════════════════════╗
║                    WALL-E Control Panel                         ║
╠════════════════════════════════════════════════════════════════╣
║                                                                  ║
║  ┌──────────────────────┬──────────────────────────────────┐   ║
║  │   Camera Feed        │     Google Maps Panel            │   ║
║  │   (MJPEG Stream)     │  (Satellite + Waypoint Editor)   │   ║
║  │                      │                                  │   ║
║  └──────────────────────┴──────────────────────────────────┘   ║
║                                                                  ║
║  ┌──────────────────────┬──────────────────────────────────┐   ║
║  │  Costmap View        │     Robot Stats                  │   ║
║  │  (Leaflet Map)       │  - Temperature                   │   ║
║  │  - Global costmap    │  - Battery voltage               │   ║
║  │  - Robot position    │  - Solar power                   │   ║
║  │  - Planned path      │  - Motor speeds                  │   ║
║  └──────────────────────┴──────────────────────────────────┘   ║
║                                                                  ║
║  ┌──────────────────────┐                                       ║
║  │  Joystick Control    │                                       ║
║  │  (Touch/Mouse)       │                                       ║
║  └──────────────────────┘                                       ║
║                                                                  ║
╚════════════════════════════════════════════════════════════════╝
```

### Web Interface Components

#### 1. Camera Feed

- **Format:** MJPEG stream from `/dev/video*`
- **Resolution:** Depends on connected camera (typically 640×480 or 1280×720)
- **Update Rate:** 30 FPS (HTTP streaming)

**Verify Camera Backend:**
```bash
# Check if camera endpoint is responding
curl http://localhost:8000/camera/frame
```

#### 2. Google Maps Panel

**Features:**
- Satellite imagery of deployment area
- Manual waypoint creation (click on map)
- Waypoint editing (drag markers)
- Heading visualization

**Setup:**
- Requires Google Maps API key
- Set via environment variable: `VITE_GOOGLE_MAPS_API_KEY`

**Create Waypoint:**
1. Click on map location
2. Drag to fine-tune position
3. Edit altitude if needed
4. Click "Save Waypoint"

#### 3. Costmap Viewer

- **Local Costmap:** 10×10 m local obstacle map
- **Global Costmap:** 100×100 m rolling window
- **Color Coding:**
  - Blue: Free space
  - Yellow: Unknown/edge
  - Red: Obstacles
  - Green: Robot position
  - Cyan: Planned path

**Interpretation:**
- Red areas: LiDAR detected obstacles
- Yellow areas: Outside LiDAR field of view
- Blue areas: Confirmed free space

#### 4. Robot Statistics Dashboard

Real-time monitoring of:
- **Temperature:** CPU/motor controller temp (°C)
- **Battery Voltage:** Main power rail (V)
- **Solar Panel Power:** Charging current (W)
- **Motor Speeds:** Left/right wheel speed (m/s)
- **Current Position:** X/Y in local frame, GPS coordinates

#### 5. On-Screen Joystick

Virtual joystick for manual control (mouse/touch):
- **Center:** No movement
- **Drag outward:** Movement command
- **Direction:** Forward/backward/left/right/diagonal

**Usage:**
- On desktop: Click and drag
- On touchscreen: Touch and drag

### Backend API Endpoints

**Note:** The web app uses these endpoints internally. Useful for custom development.

```
# Camera
GET /camera/frame              → MJPEG stream
GET /camera/latest_frame       → Latest frame as JPEG

# Robot Status
GET /robot/status              → Current robot stats
GET /robot/pose                → Robot position/orientation
GET /robot/cmd_vel             → Current velocity command

# Waypoints
GET /waypoints/list            → All stored waypoints
POST /waypoints/create         → Create new waypoint
PUT /waypoints/{id}            → Update waypoint
DELETE /waypoints/{id}         → Delete waypoint

# Navigation
POST /navigation/start         → Start navigation
POST /navigation/stop          → Stop navigation
POST /navigation/send_waypoints → Send waypoint list to robot

# Manual Control
POST /robot/cmd_vel            → Send velocity command
GET /robot/cmd_vel_status      → Current cmd_vel topic
```

### Troubleshooting Web Interface

**Camera feed not showing:**
```bash
# Check if camera is accessible
./start_docker.sh --command "ls -la /dev/video*"

# Check camera backend logs
./start_docker.sh --command "ros2 topic echo /image_raw"
```

**Maps not loading:**
- Verify Google Maps API key is set
- Check browser console for errors (F12)
- Verify internet connectivity

**Joystick not responding:**
- Verify ROSBridge is running: `./start_docker.sh -B`
- Check browser console for WebSocket errors
- Try refreshing page

**Costmap blank/not updating:**
- Verify navigation stack is running: `./start_docker.sh -n`
- Check costmap topics: `./start_docker.sh --command "ros2 topic list | grep costmap"`

---

## Debugging and Monitoring

### Topic Monitoring

**List all active topics:**
```bash
./start_docker.sh --command "ros2 topic list"
```

**Echo a specific topic (real-time values):**
```bash
./start_docker.sh --command "ros2 topic echo /odom"
```

**Get topic details (message type, publisher info):**
```bash
./start_docker.sh --command "ros2 topic info /scan"
```

**Important Topics Reference:**

| Topic | Type | Frequency | Contains |
|-------|------|-----------|----------|
| `/scan` | LaserScan | 5-10 Hz | LiDAR distance measurements |
| `/odom` | Odometry | 20 Hz | Wheel-based position estimate |
| `/imu/data` | Imu | 100 Hz | Accelerometer/gyro/magnetometer |
| `/fix` | NavSatFix | 1 Hz | GPS latitude/longitude/altitude |
| `/tf` | TF2 | Variable | Frame transformations (dynamic) |
| `/tf_static` | TF2 | Once | Frame transformations (static) |
| `/cmd_vel` | Twist | Variable | Motor velocity commands |
| `/roboclaw_status` | Custom | 5 Hz | Motor speeds, encoder counts, battery |
| `/nav2_feedback` | NavigationFeedback | 1 Hz | Navigation progress, distance remaining |
| `/joy` | Joy | 10 Hz | Joystick button/axis values |

### RViz Visualization

**Start RViz with default configuration:**
```bash
./start_docker.sh -R -d           # Requires X11 display forwarding
```

**Default visualization includes:**
- Global and local costmaps (color-coded obstacles)
- Robot position and orientation (arrow/circle)
- Planned path (colored line)
- LiDAR scan points (small dots)
- TF tree (coordinate frames with arrows)
- GPS position (if available)

**Common RViz Displays:**

| Display | Purpose | How to Add |
|---------|---------|-----------|
| Map (costmap) | Obstacle visualization | Add → By topic → /global_costmap → ... → Map |
| Path | Planned trajectory | Add → By topic → /plan → Path |
| TF | Frame relationships | Add → By display type → TF |
| Scan | Raw LiDAR points | Add → By topic → /scan → LaserScan |
| Odometry | Position trajectory | Add → By topic → /odom → Odometry |
| PoseWithCovariance | Pose uncertainty ellipse | Add → By topic → pose with covariance topic |

### Logging and Diagnostics

**Enable ROS logging at different levels:**

```bash
# Log everything (verbose)
./start_docker.sh --command "export ROS_LOG_DIR=/tmp/ros_logs && ros2 launch robot_bringup robot_launch.py"

# View current log level
./start_docker.sh --command "ros2 param list | grep log"

# Change node log level at runtime
./start_docker.sh --command "ros2 param set /roboclaw_node rcl_logging_level DEBUG"
```

**Check node status:**
```bash
./start_docker.sh --command "ros2 node list"       # List nodes
./start_docker.sh --command "ros2 node info /roboclaw_node"  # Node details
```

**Monitor system resources in container:**
```bash
./start_docker.sh --command "top -b -n 1"          # CPU/memory usage
./start_docker.sh --command "free -h"              # Memory statistics
./start_docker.sh --command "df -h"                # Disk usage
```

### TF2 Frame Debugging

**Visualize transform tree:**
```bash
./start_docker.sh --command "ros2 run tf2_tools view_frames.py"
# Generates: frames_TIMESTAMP.pdf (transform hierarchy)
```

**Expected Frame Hierarchy:**
```
map
  └── odom
       └── base_link
            ├── laser
            ├── imu
            ├── camera
            ├── antenna
            └── gps_link
```

**Check specific transform:**
```bash
./start_docker.sh --command "ros2 run tf2_ros tf2_echo map base_link"
```

**Output:**
```
At time 1234.567:
- Translation: [5.3, 2.1, 0.0]
- Rotation: in Quaternion [0, 0, 0.7071, 0.7071]
```

---

## Troubleshooting

### Common Startup Issues

#### Issue 1: "Cannot open `/dev/roboclaw`"

**Cause:** Motor driver not detected or udev rules not applied

**Diagnosis:**
```bash
lsusb                           # Check if motor driver appears
cat /dev/bus/usb/*/..../...     # Verify it's a serial device
```

**Solution:**
```bash
# Reinstall udev rules
cd scripts
sudo cp 99-walle-devices.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

# Reconnect motor driver USB cable
# Verify device now appears:
ls -la /dev/roboclaw
```

#### Issue 2: "I2C communication error" (IMU)

**Cause:** I2C bus not responding (loose connection, address conflict)

**Diagnosis:**
```bash
# Check if I2C bus is available
i2cdetect -y 1                  # Lists I2C devices
```

**Expected output:**
```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         -- -- -- -- -- -- -- -- 
10: -- -- -- -- -- -- -- -- -- -- 4a -- -- -- -- --
```

Address `4a` = IMU (BNO085 default)

**Solution:**
1. Check I2C cable connection (usually near Jetson)
2. Verify device address not conflicting: change in launch config
3. Reload i2c module: `sudo rmmod i2c_i801 && sudo modprobe i2c_i801`

#### Issue 3: "Transform timeout errors"

**Cause:** EKF not publishing transforms fast enough

**Typical Error:**
```
[ERROR] Lookup would require extrapolation into the future. 
        Requested time 1234567.890 but the latest data is 1234567.850.
```

**Solution:**
1. Wait 10-15 seconds for EKF to converge
2. Check EKF is running: `./start_docker.sh --command "ros2 topic echo /odometry/filtered"`
3. If still failing, reduce costmap update frequency:
   ```yaml
   # In nav2_no_map_params.yaml
   local_costmap:
     update_frequency: 3.0        # Reduce from 5.0
   ```

#### Issue 4: "High CPU usage" (>80%)

**Common Causes:**
- Scan matcher (ICP) running
- Multiple processes competing
- RViz visualization running

**Check CPU usage:**
```bash
./start_docker.sh --command "top -b -n 1 | head -20"
```

**Solution:**
- Reduce ICP iterations: `max_iterations: 10` (from 20)
- Reduce planner batch size: `batch_size: 100` (from 2000)
- Disable RViz if not needed
- Disable scan matcher: don't use `--scan-matcher` flag

#### Issue 5: "GPS not converging" (status = -1)

**Cause:** Indoors, poor sky view, or GPS not connected

**Solution:**
1. Move to outdoor area with clear sky view
2. Wait 30-60 seconds for fix
3. Verify GPS connected: `ls -la /dev/ttyUSB*`
4. Check GPS topic: `./start_docker.sh --command "ros2 topic echo /fix"`
5. Verify antenna connected (external antenna helps)

#### Issue 6: "Robot appears to not move" during navigation

**Cause:** One of several possibilities:

**Diagnosis Steps:**
```bash
# 1. Check if motor commands are being sent
./start_docker.sh --command "ros2 topic echo /cmd_vel_out"

# 2. Check motor driver receives commands
./start_docker.sh --command "ros2 topic echo /roboclaw_status"

# 3. Check if planner found a path
./start_docker.sh --command "ros2 topic echo /plan"

# 4. Check navigation feedback
./start_docker.sh --command "ros2 topic echo /nav2_feedback"
```

**If no `/cmd_vel` output:**
- Navigation stack not running
- Or planner couldn't find path (robot blocked)

**If `/cmd_vel` present but motors don't move:**
- Motor driver not responding to commands
- Check battery voltage: should be >12V
- Motor timeout: see Issue 7 below

#### Issue 7: "Motors suddenly stop during operation"

**Cause:** cmd_vel timeout (no new velocity commands received)

**Expected behavior:**
- RoboClaw stops motors if no cmd_vel for >1 second
- Prevents runaway if control connection lost

**Solution:**
```bash
# Check if nav controller is publishing at 20 Hz
./start_docker.sh --command "ros2 topic hz /cmd_vel"

# If rate is low or intermittent:
# - Increase update frequency (see nav2_no_map_params.yaml)
# - Reduce batch_size or time_steps (MPPI controller parameters)
```

### Advanced Debugging

#### Viewing ROS Domain Network

```bash
# Check current ROS_DOMAIN_ID
./start_docker.sh --command "printenv ROS_DOMAIN_ID"

# Run with different domain ID (network isolation)
./start_docker.sh -i 100 -r    # Domain ID 100

# Check network connectivity between nodes
./start_docker.sh --command "ros2 run ros2 daemon status"
```

#### Memory Leaks / OOM Errors

**Monitor memory over time:**
```bash
watch -n 1 'docker stats --no-stream'
```

**If memory continuously increasing:**
1. Identify which node: `ros2 node list`
2. Check node logs: `ros2 node info /node_name`
3. Possible causes:
   - Scan matcher buffer growing (ICP)
   - Costmap memory leak
   - Web interface streaming

**Quick fix:**
```bash
./start_docker.sh -x            # Stop container
./start_docker.sh -r            # Restart (clears memory)
```

#### Joystick Advanced Debugging

**Test joystick directly:**
```bash
# Check raw input
sudo jstest /dev/input/js0

# Check that ROS is reading events
./start_docker.sh --command "ros2 topic echo /joy -n 1"
```

**Calibrate joystick:**
```bash
# In Linux host
jstest-gtk /dev/input/js0

# Verify after calibration
ros2 topic echo /joy
```

---

## System Specifications and Defaults

### Hardware Configuration

**Motor System (RoboClaw):**
- Serial Port: `/dev/roboclaw` (via udev rules)
- Baud Rate: 9600
- Wheel Separation: 0.39 m
- Wheel Diameter: 0.095 m
- Max Speed: 1.0 m/s (linear), 3.0 rad/s (angular)
- Acceleration: 1.5 m/s²
- QPPR (quadrature pulses per revolution): 7000

**IMU (BNO085):**
- I2C Address: 0x4A (default)
- I2C Bus: 1
- Update Rate: 100 Hz
- Calibration: Magnetometer should be calibrated at deployment location

**GPS (u-blox NEO-M9N):**
- Serial Port: `/dev/ttyUSB0` (typically)
- Baud Rate: 38400
- Update Rate: 1 Hz
- Datum: [34.0658, -106.908, 0.0] (New Mexico Tech default)

**LiDAR (S-Lidar S3):**
- Serial Port: `/dev/ttyUSB1` (typically)
- Update Rate: 5-10 Hz
- Range: 0.2-30 m
- Frame ID: `laser`

**Camera:**
- Device: `/dev/video0` (typically)
- Resolution: 640×480 (default) or 1280×720
- Frame rate: 30 FPS

### Software Configuration Defaults

**ROS 2 Domain ID:** 62 (used for network isolation)

**Nav2 Parameters:**
- Local Costmap: 10×10 m, 0.05 m/cell, updates 5 Hz
- Global Costmap: 100×100 m, 0.2 m/cell, updates 1 Hz
- Inflation Radius: 0.24 m
- Planner: MPPI with 2000 trajectory samples
- Controller: MPPI tracking controller, 20 Hz update rate

**Teleop Parameters:**
- Enable Button: LB (Button 4)
- Max Linear Speed: 1.0 m/s
- Max Angular Speed: 3.0 rad/s
- Deadzone: 0.05
- Timeout: 1.0 second

---

## Appendix: Quick Command Reference

### Most Common Commands

```bash
# Robot Bringup
./start_docker.sh -r

# Full Navigation
./start_docker.sh -n

# Teleoperation
./start_docker.sh -t

# Web Interface
./start_docker.sh -B -w

# RViz Visualization
./start_docker.sh -R -d

# Monitor Topics
./start_docker.sh --command "ros2 topic list"

# Stop Everything
./start_docker.sh -x

# Interactive Shell
./start_docker.sh

# Rebuild Docker Image
./start_docker.sh -b
```

### Typical Multi-Component Scenarios

**Scenario 1: Field Deployment (Autonomous Navigation)**
```bash
# Terminal 1 - Start Navigation
./start_docker.sh -n -q

# Terminal 2 - Monitor (on another machine or inside container)
./start_docker.sh --command "ros2 topic echo /nav2_feedback"

# Terminal 3 - (Optional) Web Interface for remote monitoring
./start_docker.sh -B -w
```

**Scenario 2: Testing with Visualization**
```bash
# Terminal 1 - Start Robot + RViz
./start_docker.sh -r -R -d

# Terminal 2 - Start Teleop (same session or new)
./start_docker.sh -t
```

**Scenario 3: Calibration/Setup**
```bash
# Calibrate IMU magnetometer
./start_docker.sh --imu-cal

# Wait for calibration (20 seconds, watch logs)

# Test GPS fix
./start_docker.sh --gps
./start_docker.sh --command "ros2 topic echo /fix"
```

### Shutdown Procedures

**Normal Shutdown:**
```bash
./start_docker.sh -x              # Stops container gracefully
```

**Emergency Shutdown:**
```bash
docker stop $(docker ps -q)       # Force stops all containers
```

**Full Cleanup:**
```bash
./start_docker.sh -k              # Prune images and volumes
```

---

## Support and Additional Resources

**For issues and bug reports:**
- Repository: https://github.com/NMT-Wildlife-Monitoring-Project/WALL-E
- Open an issue with relevant logs

**Related Documentation in Repository:**
- `README.md` - Quick start guide
- `TECHNICAL_ANALYSIS.md` - Deep technical details and known issues
- `CODE_ISSUES.md` - Known bugs and workarounds
- `FIX_GUIDE.md` - Detailed fix procedures for known issues

**External Resources:**
- ROS 2 Jazzy Documentation: https://docs.ros.org/en/jazzy/
- Nav2 Documentation: https://navigation.ros.org/
- Jetson Orin Nano Docs: https://docs.nvidia.com/jetson/jetson-orin-nano-devkit/

---

**End of WALL-E Complete Operating Manual**
*Last Updated: February 2026*  
*Maintained by: NMT Wildlife Monitoring Project*  
*Document Version: 1.0*
