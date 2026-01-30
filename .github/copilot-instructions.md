# WALL-E Copilot Instructions

## Project Overview
WALL-E (Wildlife Activity Life Explorer) is a ROS2-based autonomous ground robot for wildlife monitoring. The project uses Docker containerization, dual Extended Kalman Filters (EKF) for localization, GPS/IMU fusion, and Nav2 for navigation.

## Architecture Essentials

### Workspace Structure
- **`ros2_ws/`** - ROS2 workspace with robot packages (sources in `src/`)
- **`docker/`** - Docker container configuration (Dockerfile + startup scripts)
- **`web_app/`** - Flask web interface for remote control
- **`scripts/`** - System utilities (udev rules, GPS initialization, service management)

### Key Components & Data Flow

**Localization Stack** (`ros2_ws/src/robot_navigation/`):
- Dual EKF filter setup (`dual_ekf_navsat.launch.py`):
  - `ekf_filter_node_odom`: Local odometry (world_frame: `odom`)
  - `ekf_filter_node_map`: Global GPS-fused (world_frame: `map`)
  - `navsat_transform_node`: Transforms GPS to map frame
- **Critical config**: `config/dual_ekf_navsat_params.yaml`
  - Two separate EKF instances with different frame hierarchies
  - `wait_for_datum: true` + `datum: [lat, lon, alt]` must be set (your actual starting GPS coordinates)
  - GPS initialization delay: `delay: 10.0` seconds (allow GPS lock time)

**Sensor Stack**:
- `sllidar_ros2/` - LIDAR scan matcher for odometry (publishes to `odom` frame)
- `bno085_driver/` - IMU providing orientation (yaw stabilization)
- `nmea_navsat_driver/` - GPS receiver (publishes to `fix` topic)
- `roboclaw_driver/` - Motor controller interface

**Bringup** (`robot_bringup/launch/robot_launch.py`):
- Launches LIDAR, IMU, GPS, motor controller
- 5-second timer before navigation launch (for system stabilization)
- URDF published by `robot_state_publisher`

### Frame Hierarchy (Critical for RViz)
```
map → odom → base_link → [sensors]
```
- `map`: GPS-based global frame (set by navsat_transform)
- `odom`: Local odometry frame (from scan matching)
- `base_link`: Robot body center

**Common Issue**: Map shifts in RViz when GPS initializes late → **Solution**: Ensure `wait_for_datum: true` and set `datum` to your test location's GPS coordinates.

## Build & Deployment

### Docker Commands
```bash
cd docker
./start_docker.sh -b              # Build container
./start_docker.sh -s              # Start robot (all nodes)
./start_docker.sh -w              # View map (RViz)
./start_docker.sh -c "ros2 topic list"  # Run custom command
```

### ROS2 Build
Inside container:
```bash
cd /root/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Critical Conventions & Patterns

1. **YAML Parameter Overrides**: EKF params split by filter (odom vs map). Changes to `dual_ekf_navsat_params.yaml` affect both node instances—verify both `ekf_filter_node_odom` and `ekf_filter_node_map` sections.

2. **Frame IDs**: Always `base_link` for robot body, `imu/data` for raw IMU (never `imu`). GPS topic remapped to `fix` in launch file.

3. **GPS Initialization**: 
   - Set `datum` in YAML to **your testing location's GPS coordinates**
   - `wait_for_datum: true` prevents premature map frame broadcasts
   - RViz should show stable map after ~10 sec startup

4. **Sensor Remappings**: Check `dual_ekf_navsat.launch.py` remappings—mismatch between launch remaps and YAML topic names causes filter failures.

5. **Docker Env**: 
   - `XDG_RUNTIME_DIR` not set in container (non-blocking RViz issue)
   - Use `./start_docker.sh -d` for X11 display forwarding

## Testing & Debugging

### Verify Localization
```bash
ros2 topic echo /odometry/local   # Odom-frame odometry
ros2 topic echo /odometry/global  # Map-frame odometry (GPS-fused)
ros2 topic echo /tf_static        # Frame transforms
```

### RViz Setup
1. Set Fixed Frame to `map`
2. Add TF visualization to see frame hierarchy
3. Odometry topics: `/odometry/local` (odom) and `/odometry/global` (map)
4. If map shifts: Check EKF diagnostics (`/diagnostics` topic)

### Common Errors
- **GLSL link errors in RViz**: Non-critical; renderer incompatibility with indexed 8-bit images
- **Map origin jumping**: GPS datum not set or wrong coordinates—update `dual_ekf_navsat_params.yaml`
- **No /map→/odom transform**: Check `broadcast_cartesian_transform: true` in navsat_transform config

## Integration Points
- **GPS Driver**: NMEA serial at `/dev/gps` (4800 baud)
- **Motor Control**: RoboClaw at `/dev/roboclaw`
- **Web Interface**: Flask app (`web_app/app.py`) communicates via ROS bridge
- **Launch Interdependencies**: `robot_launch.py` → `gps_waypoint_follower.launch.py` → `dual_ekf_navsat.launch.py`

## References
- robot_localization docs: Dual EKF filter pattern for GPS integration
- ROS2 TF2 frame conventions: https://docs.ros.org/en/humble/Concepts/Intermediate/Tf2/Frames.html
