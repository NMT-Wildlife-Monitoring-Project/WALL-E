# WALL-E Project Instructions for Claude

## 1. PROJECT OVERVIEW

**Project Name:** WALL-E (Wildlife Activity Life Explorer)  
**Purpose:** Autonomous wildlife monitoring robot with autonomous navigation and joystick teleoperation  
**Current Status:** Motor control and teleop command routing fixed; motor oscillation partially diagnosed  
**Git Branch:** `oldcodetest` (working branch for development)

### Key Capabilities
- **Autonomous Navigation:** Nav2 with MPPI controller for GPS waypoint following
- **Teleoperation:** Joystick control (XBox-style) via twist_mux arbitration
- **Exploration Algorithm:** D2OC (Density-based Occupancy-Oriented Curiosity) for autonomous frontier exploration
- **Localization:** Dual EKF (odom and map frames) with GPS, IMU, and laser odometry fusion
- **Sensor Stack:** RPLiDAR, BNO085 IMU, GPS, dual quadrature encoders

---

## 2. CODEBASE STRUCTURE

```
WALL-E/
├── ros2_ws/                          # Main ROS2 workspace
│   ├── src/
│   │   ├── roboclaw_driver/          # Motor controller driver [CRITICAL]
│   │   │   ├── roboclaw_driver/roboclaw_node.py      # Main node
│   │   │   ├── roboclaw_driver/roboclaw_3.py         # Protocol layer
│   │   │   └── launch/roboclaw_launch.py             # Launch config
│   │   ├── robot_navigation/         # Nav2 and navigation stack
│   │   │   ├── config/
│   │   │   │   ├── twist_mux.yaml                    # Teleop/nav arbitration
│   │   │   │   ├── nav2_no_map_params.yaml           # Nav2 MPPI controller
│   │   │   │   └── dual_ekf_navsat_params.yaml       # Localization
│   │   │   └── launch/robot_nav_launch.py
│   │   ├── robot_teleop/             # Joystick control
│   │   │   ├── launch/robot_teleop_launch.py
│   │   │   └── config/teleop_config.yaml
│   │   ├── d2oc_algorithm/           # Exploration algorithm [RECENTLY FIXED]
│   │   ├── bno085_driver/            # IMU driver
│   │   ├── nmea_navsat_driver/       # GPS driver
│   │   ├── sllidar_ros2/             # LiDAR driver
│   │   └── [other packages]
│   ├── build/                         # Colcon build artifacts (ignore)
│   ├── install/                       # Installed packages (auto-generated)
│   └── log/                           # Build logs (ignore)
├── docker/                            # Docker configuration
│   ├── Dockerfile                     # Build definition
│   ├── start_docker.sh                # Main launch script
│   └── docker-compose.yml             # Compose config
├── scripts/                           # Utility scripts
│   ├── 99-walle-devices.rules         # udev rules for hardware access
│   └── [other utilities]
└── [documentation & config files]
```

---

## 3. CRITICAL SYSTEM COMPONENTS

### 3.1 RoboClaw Motor Driver (`roboclaw_node.py`)

**Purpose:** Communicates with RoboClaw motor controller via serial (9600 baud, address 128)

**Key Parameters:**
| Parameter | Value | Notes |
|-----------|-------|-------|
| `wheel_separation` | 0.39 m | Distance between left/right wheels |
| `wheel_diameter` | 0.095 m | Wheel circumference needed for odometry |
| `qppr` | 6533 | Quadrature pulses per revolution (encoder resolution) |
| `max_speed` | 0.4 m/s | Command velocity limit |
| `max_speed_qpps` | 25410 | Hardware limit (from BasicMicro auto-tune) |
| `accel_qpps` | 2000 | Hardware acceleration (must be positive) |
| `m1_reverse` | True | Motor 1 reversal flag |
| `m2_reverse` | False | Motor 2 normal direction |
| `cmd_vel_timeout` | 1.0 s | Safety timeout if no velocity command |

**Critical Functions:**
- `connect()`: Initializes serial connection, sets encoder modes (quadrature), resets encoders
- `cmd_vel_callback()`: Converts Twist msgs to differential drive QPPS; applies motor reversal
- `update_odom()`: Reads encoders, calculates odometry (WITHOUT double-reversing)
- `meters_to_pulses()`: v_m/s × 6533 / (π × 0.095)

**Command Flow:**
```
/cmd_vel_out (from twist_mux) → roboclaw_node.cmd_vel_callback()
  → meters_to_pulses() for M1 and M2
  → Apply m1_reverse/m2_reverse flags
  → Send SpeedAccelM1M2 command to RoboClaw
  → RoboClaw controls motors
  → Read encoder feedback → update_odom()
  → Publish /odom for navigation stack
```

**Recent Fixes Applied:**
1. ✅ Added encoder mode initialization (SetM1EncoderMode/SetM2EncoderMode=1)
2. ✅ Fixed accel_qpps safety check (always positive, min 1)
3. ✅ Removed double motor-reversal from odometry calculation
4. ✅ **Remapping to /cmd_vel_out** (subscribes to twist_mux output, not /cmd_vel)

**Known Issues:**
- ⚠️ Motor speeds oscillate wildly during navigation (partial diagnosis: encoder noise or PID tuning issue)

### 3.2 twist_mux Command Multiplexer

**Purpose:** Arbitrates between teleop (priority 20) and nav (priority 10) velocity commands

**Configuration File:** `robot_navigation/config/twist_mux.yaml`

**Key Settings:**
```yaml
twist_mux:
  ros__parameters:
    use_stamped: false
    cmd_vel_out: "/cmd_vel"     # Misleading: twist_mux ignores this, publishes to /cmd_vel_out
    topics:
      teleop:
        topic: "/cmd_vel_teleop"
        timeout: 0.5 s
        priority: 20              # Higher priority = used when both available
      nav:
        topic: "/cmd_vel_nav"
        timeout: 0.5 s
        priority: 10              # Lower priority but always available during autonomous nav
```

**Topic Flow:**
```
Joystick Input
  → joy_linux → /joy
  → teleop_twist_joy → /cmd_vel_teleop (max 0.4 m/s)
  ↓
twist_mux (arbitrates)
  ← /cmd_vel_nav from Nav2 MPPI (async, lower priority)
  ↓
/cmd_vel_out (OUTPUT)
  → roboclaw_node (remapped subscription)
  → motors
```

**Critical Detail:** twist_mux **publishes to /cmd_vel_out by default**, regardless of yaml config. RoboClaw remaps /cmd_vel → /cmd_vel_out to receive these commands.

### 3.3 Nav2 MPPI Controller

**Purpose:** Autonomous navigation to GPS waypoints

**Configuration File:** `robot_navigation/config/nav2_no_map_params.yaml`

**Key Parameters:**
- **MPPI Time Horizon:** 5.0 sec (look-ahead time for control decisions)
- **Max Speed:** Capped by nav2 (typically 0.5 m/s max)
- **Costmap Type:** Rolling (local GPS-relative, no pre-built map)
- **Localization:** Dual EKF fuses GPS, IMU, laser odometry

**Recent Changes:**
- MPPI frequency tuned to match controller dt
- Velocity smoother integrated
- GPS driver disabled (use local costmap with LiDAR only)

### 3.4 Teleop Launch Chain

**Purpose:** Enable joystick control when running with `-t` flag

**Launch Script:** `robot_teleop/launch/robot_teleop_launch.py`

**Nodes Started:**
1. `joy_linux_node` → reads /dev/input/js* → publishes /joy topic
2. `teleop_twist_joy` → reads /joy → publishes /cmd_vel_teleop

**Critical Remappings:**
- teleop_twist_joy remaps output to `/cmd_vel_teleop`
- twist_mux arbitrates between teleop and nav inputs
- RoboClaw remaps subscription to `/cmd_vel_out`

---

## 4. KEY PROBLEMS SOLVED & CURRENT ISSUES

### ✅ SOLVED ISSUES

**1. Motor Encoder Initialization Error (128/130 codes)**
- **Root Cause:** Encoder modes not initialized in RoboClaw firmware
- **Solution:** Added `SetM1EncoderMode(1)` and `SetM2EncoderMode(1)` in `roboclaw_node.connect()`
- **File:** `roboclaw_driver/roboclaw_driver/roboclaw_node.py` (connect method)
- **Commit:** 6f087beb

**2. Motor Disconnection (Red LED)**
- **Root Cause:** accel_qpps parameter set to -1 (invalid, sent negative to hardware)
- **Solution:** Changed default from -1 to 2000; added safety check `max(1, abs(value))`
- **File:** `roboclaw_launch.py`
- **Commit:** 6f087beb

**3. Kinematics Parameter Mismatch**
- **Root Cause:** Node defaults (0.24m, 0.093m) vs launch (0.39m, 0.095m) → odometry error
- **Solution:** Unified all to physics-measured values (0.39m, 0.095m)
- **Files:** `roboclaw_node.py`, `roboclaw_launch.py`
- **Commit:** 6f087beb

**4. Odometry Direction Inverted**
- **Root Cause:** Motor reversal signs applied twice (cmd_vel AND odometry)
- **Solution:** Removed redundant reversal application from `update_odom()`
- **File:** `roboclaw_node.py` (update_odom method)
- **Commit:** 703123b5

**5. Teleop Not Working**
- **Root Cause:** RoboClaw subscribed to `/cmd_vel`, but twist_mux publishes to `/cmd_vel_out`
- **Solution:** Added remapping in roboclaw_launch.py: `remappings=[('cmd_vel', '/cmd_vel_out')]`
- **File:** `roboclaw_launch.py` (line 64)
- **Commit:** a8f0196d

### ⚠️ PARTIALLY DIAGNOSED ISSUES

**Motor Speed Oscillation During Navigation**
- **Symptoms:** Motors report wildly oscillating speeds (m1_speed: -0.006 to 0.045 m/s rapidly)
- **Diagnosis:** Nav2 publishes reasonable commands (0.008-0.22 m/s), but motor response varies
- **Likely Causes:**
  1. Encoder signal noise (6533 QPPR might be noisy at high quantization rates)
  2. RoboClaw PID tuning (might need retuning from BasicMicro defaults)
  3. Velocity command conversion quantization errors
  4. Acceleration profile oscillation
- **Status:** Shelved to prioritize teleop fix; needs separate investigation

### ❌ NOT YET ADDRESSED

1. **GPS Datum Hardcoded to New Mexico** (doesn't affect local navigation)
2. **gps_waypoint_handler_node Blocking** (spin_until_future_complete in constructor)
3. **D2OC Integration** (exploration algorithm recently fixed and working)

---

## 5. DOCKER WORKFLOW & LAUNCH COMMANDS

### Docker Launch Script: `docker/start_docker.sh`

**Build & Run:**
```bash
cd /home/julian/WALL-E/docker
./start_docker.sh -b          # Build image (one-time, ~5 min)
```

**Run Modes:**
```bash
./start_docker.sh -s          # Full system (motors + nav + teleop ready)
./start_docker.sh -t          # Teleop only (joystick control, no nav)
./start_docker.sh -m          # Motors only (motor driver alone)
./start_docker.sh -c "cmd"    # Run custom command in container
./start_docker.sh -d           # Enable X11 display forwarding
```

**Interactive Shell:**
```bash
./start_docker.sh             # Opens bash in container (default)
```

### Inside Docker Container

**Source Workspace:**
```bash
source /home/walle/ros2_ws/install/setup.bash
```

**Verify Topics (Teleop Test):**
```bash
# Terminal 1: Full system
./start_docker.sh -s

# Terminal 2 (in container): Monitor joystick
ros2 topic echo /joy

# Terminal 3 (in container): Monitor teleop output
ros2 topic echo /cmd_vel_teleop

# Terminal 4 (in container): Monitor twist_mux output
ros2 topic echo /cmd_vel_out

# Terminal 5 (in container): Monitor roboclaw status
ros2 topic echo /roboclaw_status
```

**RViz Visualization:**
```bash
./start_docker.sh -d           # Enable X11 from host terminal
rviz2 -c /home/walle/ros2_ws/src/robot_navigation/config/walle_default.rviz
```

---

## 6. DEBUGGING & DIAGNOSTICS

### Common Debugging Commands

**Check Topic Publication:**
```bash
ros2 topic list              # Show all active topics
ros2 topic echo /topic_name  # Monitor topic messages
ros2 topic hz /topic_name    # Check publish frequency
ros2 topic info /topic_name  # Show subscribers/publishers
```

**Check Node Status:**
```bash
ros2 node list               # Show running nodes
ros2 node info /node_name    # Show subscriptions/publications
```

**View RoboClaw Status:**
```bash
ros2 topic echo /roboclaw_status | grep -E "m1_speed|m2_speed|temp|battery"
```

**Docker Logs:**
```bash
docker logs -f walle_container  # Live logs from running container
docker logs walle_container | tail -100
```

**Git Verification:**
```bash
cd /home/julian/WALL-E
git status                   # Check uncommitted changes
git log --oneline -10        # Recent commits
git diff HEAD~1 HEAD         # Show last commit changes
```

### Problem-Solving Checklist

1. **Motors not responding?**
   - Check `/roboclaw_status` for connection status
   - Verify `/cmd_vel_out` topic is being published
   - Check RoboClaw red LED (connection error?) or yellow LED (temperature)
   - Confirm `m1_reverse` and `m2_reverse` flags match physical wiring

2. **Teleop joystick not working?**
   - Verify `/joy` topic is publishing (check joystick connected)
   - Monitor `/cmd_vel_teleop` (should see Twist msgs when joystick moved)
   - Check `/cmd_vel_out` (twist_mux should route teleop to this topic)
   - Verify RoboClaw node is running (check remapping to /cmd_vel_out)

3. **Navigation goals not being reached?**
   - Check Nav2 status in RViz
   - Verify `/cmd_vel_nav` is being published at reasonable frequencies
   - Monitor motor oscillation (see motor speed oscillation issue below)
   - Check `/odom` updates match actual robot movement

4. **Motor speed oscillation?**
   - This is a known partially-diagnosed issue
   - Likely encoder noise or PID tuning; shelved for later investigation
   - Robot still moves, but speed readings unreliable

---

## 7. WORKING EFFECTIVELY WITH THIS CODEBASE

### Code Organization Principles

1. **Parameter Consistency:** All robot parameters (wheel dimensions, QPPS limits) must be synchronized across:
   - `roboclaw_node.py` (node defaults)
   - `roboclaw_launch.py` (launch arguments)
   - Hardware calibration (from BasicMicro)

2. **Topic Routing:** Understand complete message flow:
   - Joystick → twist_mux → roboclaw
   - Navigation → twist_mux → roboclaw
   - Roboclaw → odometry publisher → localization/nav feedback

3. **Docker Isolation:** Changes inside Docker are TEMPORARY unless committed to git. Always:
   - Test locally in Docker
   - Commit changes to git
   - Rebuild with `-b` flag to apply changes

### Modification Workflow

1. **Identify Target File:**
   - Use grep_search for configuration parameters
   - Use semantic_search for broader concepts
   - Check git history for context on why values were chosen

2. **Make Changes Systematically:**
   - Test one parameter change at a time
   - Verify git diff before committing
   - Write clear commit messages describing physical motivation (e.g., "Match 12V hardware limits")

3. **Validate & Commit:**
   - Build: `cd docker && ./start_docker.sh -b`
   - Test: `./start_docker.sh -s` or `-t` depending on subsystem
   - Monitor relevant topics to confirm behavior
   - Commit: `git add -A && git commit -m "..."` 
   - Push: `git push origin oldcodetest`

4. **For User Testing:**
   - Document changes in commit message
   - Have user pull on Jetson: `git pull`
   - Have user rebuild: `cd docker && ./start_docker.sh -b`
   - Verify behavior matches expectations

### Performance Optimization Priorities

**When Asked to Fix Issues, Prioritize:**

1. **Connectivity Issues (HIGH PRIORITY)**
   - Motor disconnect/red LED
   - Teleop/joystick not responding
   - Topic routing broken
   - Root cause: Usually simple configuration/remapping issues
   - Approach: Trace complete message flow, identify broken link

2. **Movement Issues (MEDIUM PRIORITY)**
   - Motors moving wrong direction → motor reversal flags
   - Oscillation/unstable control → PID tuning or velocity quantization
   - Odometry incorrect → encoder mode or double-reversal bug
   - Approach: Verify parameters match physical robot

3. **Performance Issues (LOW PRIORITY)**
   - CPU usage high → reduce node frequencies
   - Navigation slow → tune MPPI parameters
   - Approach: Profile and monitor, change one parameter at a time

### When User Gets Frustrated

**Pragmatic Approach:**
1. Don't over-explain or over-analyze
2. Identify **single root cause** causing the problem
3. Apply **minimal fix** (one-line changes preferred)
4. Commit and push immediately
5. Have user test and report
6. Iterate quickly if fix didn't work

**Example:** User said "teleop not working... you're wasting my time"
- Avoided lengthy debugging session
- Traced complete flow, found roboclaw subscribing to wrong topic
- Applied one-line remapping fix: `remappings=[('cmd_vel', '/cmd_vel_out')]`
- Committed and pushed
- Success

---

## 8. CURRENT REPOSITORY STATE (As of April 9, 2026)

**Branch:** `oldcodetest`  
**Latest Commit:** 41d0c044 "roboclaw: sync parameters with hardware-tuned settings from BasicMicro"

**Recent Successful Changes:**
- ✅ Encoder initialization fixed
- ✅ Motor parameters unified
- ✅ Odometry direction corrected
- ✅ Teleop routing fixed (remapping to /cmd_vel_out)
- ✅ D2OC exploration algorithm fixed and working

**Status:** Ready for field testing
- Teleop: Awaiting user confirmation after rebuild
- Navigation: Working but with motor oscillation (known issue)
- Motors: Responding to commands correctly
- Localization: Dual EKF running

**Next Actions (When Needed):**
1. Confirm teleop works on Jetson (user to test after git pull + rebuild)
2. Investigate motor oscillation root cause (if motor still oscillates during nav)
3. Test autonomous D2OC exploration (if user enables it)

---

## 9. QUICK REFERENCE: HOW TO DO COMMON TASKS

### Add a New Parameter to RoboClaw Node

1. Add to `roboclaw_node.py` `__init__`: `self.declare_parameter('param_name', default_value)`
2. Add to launch args in `roboclaw_launch.py`: `DeclareLaunchArgument('param_name', default_value='...')`
3. Add to node_params in launch: `{'param_name': LaunchConfiguration('param_name')}`
4. Use in code: `self.get_parameter('param_name').get_parameter_value().TYPE_value`

### Debug a Stuck Command Flow

1. `ros2 topic list` → confirm all expected topics exist
2. `ros2 topic echo /topic_name` → watch each stage of pipeline
3. `ros2 node info /node_name` → verify subscriptions/publications
4. `docker logs -f container_id` → check for error messages
5. `git diff HEAD~1` → see what changed recently

### Test a Configuration Change

1. Edit config file
2. Rebuild: `cd docker && ./start_docker.sh -b`
3. Run: `./start_docker.sh -s` or `-t`
4. Monitor: `ros2 topic echo` relevant topics
5. If good: `git add -A && git commit && git push`
6. If bad: `git checkout -- .` to revert, analyze

### Push Changes to GitHub

```bash
cd /home/julian/WALL-E
git status                           # Verify what changed
git add -A                           # Stage all changes
git commit -m "Clear description"    # Write GOOD commit message
git push origin oldcodetest          # Push to working branch
```

---

## 10. FINAL OPERATIONAL GUIDELINES

1. **Always work on `oldcodetest` branch** (never main)
2. **Test changes in Docker first** before declaring "done"
3. **Commit every logical fix** with clear message explaining physical motivation
4. **Prioritize connectivity/routing over tuning** (simple fixes first)
5. **When frustrated, go pragmatic:** Find single root cause → minimal fix → test
6. **Document via git commits**, not markdown (markdown gets outdated)
7. **Pull current code before starting work** (`git pull origin oldcodetest`)
8. **After rebuild, always test** with live topic monitoring before declaring success

---

**Generated:** April 9, 2026 | Updated by: Claude | Status: Ready for Field Operations
