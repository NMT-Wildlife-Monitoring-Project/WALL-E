## WALL-E Fix Implementation Guide

Follow these steps in order. Commands assume you are at the repo root (`/home/ndev/WALL-E`). Use a text editor you are comfortable with (VS Code, nano, etc.). After each change, save the file and rerun Docker rebuild/tests where noted.

---

### 1) Fix RoboClaw heartbeat safety (prevents runaway motors)
- File: `ros2_ws/src/roboclaw_driver/roboclaw_driver/roboclaw_node.py`
- What to change:
  1. In `stop_motors()`, after `SpeedAccelM1M2(...)`, set `self._last_cmd = (0, 0)` and record a flag like `self._cmd_valid = False`.
  2. In `cmd_vel_callback()`, after updating `_last_cmd`, set `self._cmd_valid = True`.
  3. In `_heartbeat()`, return immediately unless `self._cmd_valid` is `True`; this stops resending old motion once a timeout occurs.
- Quick test:
  1. Run the node in the container, send a `cmd_vel` once, then stop publishing.
  2. Confirm motors stop and do not resume (check logs: no SpeedAccel calls after timeout).

### 2) Unblock GPS waypoint handler (allow callbacks to run)
- File: `ros2_ws/src/waypoint_server/waypoint_server/gps_waypoint_handler_node.py`
- What to change:
  1. Replace `rclpy.spin_until_future_complete(self, future)` with `future.result()` awaited via `rclpy.spin_once(self, timeout_sec=...)` loop that yields, or switch the executable to use `MultiThreadedExecutor`.
  2. Replace the `while not self.navigator.isTaskComplete(): ... time.sleep(0.1)` loop with a timer that polls and logs feedback without blocking (`self.create_timer(0.1, callback)`).
- Quick test:
  1. Launch `gps_waypoint_follower.launch.py`.
  2. Confirm feedback logs continue while the node remains responsive (no warnings about blocked executor).

### 3) Fix Nav2 parameter rewrite helper
- File: `ros2_ws/src/robot_navigation/launch/gps_waypoint_follower.launch.py`
- What to change: Set `param_rewrites={}` (not an empty string) in the `RewrittenYaml` call.
- Quick test: Launch the file; verify no exception about `param_rewrites` type and Nav2 parameters load.

### 4) Align RoboClaw geometry defaults
- Files:
  - `ros2_ws/src/roboclaw_driver/roboclaw_driver/roboclaw_node.py` (defaults)
  - `ros2_ws/src/roboclaw_driver/launch/roboclaw_launch.py` (launch args)
- What to change: Set both defaults to the same values (recommended: `wheel_separation: 0.39`, `wheel_diameter: 0.095`) so running the node directly or via launch gives matching kinematics.
- Quick test: Start RoboClaw node via launch and directly; ensure odometry matches expected distances when wheels move a known distance.

### 5) Make GPS datum configurable
- Files:
  - `ros2_ws/src/robot_navigation/config/dual_ekf_navsat_params.yaml`
  - `ros2_ws/src/robot_navigation/launch/dual_ekf_navsat.launch.py`
- What to change:
  1. Add launch arguments for `datum_lat`, `datum_lon`, `datum_alt` with safe defaults.
  2. Pass those into the parameters (replace the hard-coded `[34.0658, -106.908, 0.0]`).
  3. Document in `README.md` how to set these for a new site before running.
- Quick test: Launch with a custom datum (any safe test numbers) and confirm `navsat_transform_node` starts without warnings about datum mismatch.

### 6) Refresh beginner-friendly README
- File: `README.md`
- What to change:
  1. Remove the old “options not implemented” block and TODOs.
  2. Add a short “From zero to first launch” section:
     - Install Docker, add user to docker group.
     - `cd docker && ./start_docker.sh -b` to build.
     - `./start_docker.sh -r` to bring up sensors/motors, or `-n` to start navigation.
     - How to enter the container: `./start_docker.sh -c "bash"`.
  3. Add a quick verification checklist (topics: `/odom`, `/scan`, `/tf` tree).
- Quick test: Ask a teammate with little ROS experience to follow it; they should reach a running container and see `ros2 topic list` without extra help.

---

After changes
1. Rebuild Docker: `cd docker && ./start_docker.sh -b`
2. Basic smoke test: `./start_docker.sh -r` (bringup) or `./start_docker.sh -n` (navigation). Watch logs for errors and ensure TF tree is stable.
