# WALL-E Web App Enhancement Plan: Power Monitoring & Bug Fixes

## Executive Summary

This plan covers:
1. **Critical Bug Fixes** - Fix hardcoded WebSocket URL blocking ZeroTier access
2. **Power Monitoring** - Add Jetson power consumption tracking and watt-hour integration
3. **Environment Configuration** - Proper .env setup for deployment

---

## Part 1: Critical Bug Fixes

### Issue 1: Hardcoded WebSocket URL
**Location**: `/home/ndev/WALL-E/web_app/frontend/src/App.tsx:19`

**Current Code**:
```typescript
connect('ws://localhost:9090');
```

**Fix Required**:
```typescript
const rosbridgeUrl = import.meta.env.VITE_ROSBRIDGE_URL || 'ws://localhost:9090';
connect(rosbridgeUrl);
```

**Impact**: Enables remote access over ZeroTier

---

### Issue 2: Missing Frontend Environment Configuration

**Action**: Create `/home/ndev/WALL-E/web_app/frontend/.env` from template

**Content**:
```env
# Google Maps API Key (optional - map features will be disabled without it)
VITE_GOOGLE_MAPS_API_KEY=

# ROSBridge WebSocket URL
# For local dev: ws://localhost:9090
# For Docker: ws://localhost:9090 (uses host network)
# For ZeroTier remote: ws://<jetson-zerotier-ip>:9090
VITE_ROSBRIDGE_URL=ws://localhost:9090

# Backend API Base URL (relative path works for same-origin)
VITE_API_BASE_URL=/api
```

---

## Part 2: Jetson Power Monitoring Architecture

### Overview

Add real-time Jetson power consumption monitoring and cumulative energy tracking (watt-hours) to the web dashboard.

### Data Flow

```
Jetson Hardware (INA3221 power monitors)
    ↓
jtop Python library (jetson_stats)
    ↓
New: power_monitor_node.py (ROS2 node)
    ↓
/jetson/power topic (custom PowerStatus message)
    ↓
Existing: roboclaw_node → /roboclaw_status (battery volts, motor currents, controller temp)
    ↓
ROSBridge WebSocket
    ↓
Backend: /api/robot/stats endpoint (enhanced)
    ↓
Frontend: StatsPanel component (enhanced)
```

### Existing Power Data Available

From Roboclaw motor controller (already implemented):
- Main battery voltage (`/roboclaw_status` topic)
- Logic battery voltage
- Motor currents (M1, M2)
- Temperature

**New Data to Add** (Jetson-specific):
- Total system power draw (watts)
- CPU power (watts)
- GPU power (watts)
- Total energy consumed (watt-hours) - persistent tracking
- Individual power rail readings (VDD_CPU_GPU_CV, etc.)
- Jetson temperatures (CPU, GPU, thermal zones)

---

## Part 3: Implementation Details

### 3.1 New ROS2 Message Type

**File**: `/home/ndev/WALL-E/ros2_ws/src/robot_messages/msg/JetsonPowerStatus.msg`

```python
# Jetson Power Status Message
std_msgs/Header header

# Instantaneous power consumption (watts)
float32 total_power          # Total system power (W)
float32 cpu_power            # CPU power consumption (W)
float32 gpu_power            # GPU power consumption (W)

# Power rails (model-specific)
string[] rail_names          # Names of power rails (e.g., "VDD_CPU_GPU_CV")
float32[] rail_power         # Power per rail in watts

# Cumulative energy tracking
float64 total_energy_wh      # Total energy consumed since boot (watt-hours)
float64 session_energy_wh    # Energy consumed this session (watt-hours)

# Thermal data
float32 cpu_temp             # CPU temperature (Celsius)
float32 gpu_temp             # GPU temperature (Celsius)
float32 thermal_temp         # Thermal zone temperature (Celsius)

# System info
uint8 power_mode             # Current power mode (0=MAX, 1=15W, etc.)
string power_mode_name       # Human-readable power mode name
```

---

### 3.2 New ROS2 Power Monitor Node

**File**: `/home/ndev/WALL-E/ros2_ws/src/robot_sensors/robot_sensors/power_monitor_node.py`

**Dependencies**:
```python
import rclpy
from rclpy.node import Node
from robot_messages.msg import JetsonPowerStatus
from jtop import jtop
import time
```

**Key Responsibilities**:
1. Read Jetson power stats via `jtop` library
2. Integrate power over time to calculate watt-hours
3. Publish on `/jetson/power` topic at configurable rate (default: 1 Hz)
4. Persist cumulative energy to file on shutdown
5. Handle Jetson model differences (Nano, Xavier, Orin, TX2)

**Energy Integration Algorithm**:
```python
# Trapezoidal integration
dt = current_time - last_time  # seconds
avg_power = (current_power + last_power) / 2.0  # watts
energy_wh += (avg_power * dt) / 3600.0  # convert to watt-hours
```

**Persistence**:
- Store cumulative energy in `/tmp/shared/jetson_energy.json`
- Load on startup, save on shutdown and periodically (every 5 minutes)
- Format: `{"total_energy_wh": 123.45, "last_update": "2025-12-03T10:30:00"}`

---

### 3.3 Backend API Enhancement

**File**: `/home/ndev/WALL-E/web_app/backend/routers/robot.py`

**Enhance `RobotStats` model** (around line 27):
```python
class RobotStats(BaseModel):
    # Existing fields
    battery_voltage: float = Field(default=0.0, description="Battery voltage in volts")
    battery_percentage: float = Field(default=0.0, description="Battery percentage 0-100")
    cpu_temperature: float = Field(default=0.0, description="CPU temperature in Celsius")

    # NEW: Jetson power monitoring
    jetson_power_watts: float = Field(default=0.0, description="Current Jetson power draw in watts")
    jetson_cpu_power_watts: float = Field(default=0.0, description="Jetson CPU power in watts")
    jetson_gpu_power_watts: float = Field(default=0.0, description="Jetson GPU power in watts")
    jetson_energy_wh: float = Field(default=0.0, description="Total energy consumed in watt-hours")
    jetson_session_energy_wh: float = Field(default=0.0, description="Session energy in watt-hours")
    jetson_cpu_temp: float = Field(default=0.0, description="Jetson CPU temperature in Celsius")
    jetson_gpu_temp: float = Field(default=0.0, description="Jetson GPU temperature in Celsius")
    jetson_thermal_temp: float = Field(default=0.0, description="Jetson thermal zone temperature in Celsius")

    # Roboclaw telemetry
    main_battery_voltage: float = Field(default=0.0, description="Main battery voltage in volts")
    logic_battery_voltage: float = Field(default=0.0, description="Logic rail voltage in volts")
    motor_current_m1: float = Field(default=0.0, description="Motor 1 current in amps")
    motor_current_m2: float = Field(default=0.0, description="Motor 2 current in amps")
    roboclaw_temperature: float = Field(default=0.0, description="Roboclaw controller temperature in Celsius")
    roboclaw_error: str = Field(default="", description="Roboclaw error string (if any)")
```

**Subscribe to new topic** in `robot.py`:
```python
# Subscribe to /jetson/power topic
async def handle_jetson_power(msg):
    # Update cached power stats
    pass

# Subscribe to /roboclaw_status for volts/amps/temp/error
async def handle_roboclaw_status(msg):
    # Update cached roboclaw stats
    pass
```

**Update `/api/robot/stats` endpoint** to return new fields from ROSBridge subscription.

---

### 3.4 Frontend UI Enhancement

**File**: `/home/ndev/WALL-E/web_app/frontend/src/components/StatsPanel.tsx`

**Add Power Monitoring Section**:
```typescript
interface PowerStats {
  jetsonPowerWatts: number;
  jetsonCpuPowerWatts: number;
  jetsonGpuPowerWatts: number;
  jetsonEnergyWh: number;
  jetsonSessionEnergyWh: number;
  jetsonCpuTemp: number;
  jetsonGpuTemp: number;
  jetsonThermalTemp: number;
  motorCurrentM1: number;
  motorCurrentM2: number;
   mainBatteryVoltage: number;
   logicBatteryVoltage: number;
   roboclawTemperature: number;
   roboclawError?: string;
}
```

**Display Elements**:
1. **Live Power Draw** - Large number with "W" unit, color-coded:
   - Green: < 10W
   - Yellow: 10-20W
   - Red: > 20W

2. **Power Breakdown Bar Chart**:
   - CPU power (blue bar)
   - GPU power (green bar)
   - System/Other (gray bar)

3. **Jetson Thermals**:
   - CPU temp, GPU temp, thermal zone temp with warning thresholds and badges

4. **Energy Consumption**:
   - Total: `XX.XX Wh` (since boot)
   - Session: `XX.XX Wh` (this session)

5. **Motor Load**:
   - M1: `X.XX A`
   - M2: `X.XX A`

6. **Roboclaw Status**:
   - Main battery: `XX.X V` (low/high indicator)
   - Logic battery: `XX.X V`
   - Controller temp: `XX.X °C`
   - Error string if present

**UI Layout** (add to existing stats panel):
```
┌─────────────────────────────────────┐
│ Robot Statistics                    │
├─────────────────────────────────────┤
│                                     │
│ 🔋 Battery: 24.5V (95%)            │
│ 🌡️  Temperature: 42°C               │
│                                     │
│ ⚡ POWER MONITORING                 │
│ ┌─────────────────────────────┐   │
│ │   Current Draw: 15.3 W      │   │  ← Large, prominent
│ └─────────────────────────────┘   │
│                                     │
│ CPU:    ████████░░ 7.2 W           │  ← Breakdown bars
│ GPU:    ███░░░░░░░ 3.1 W           │
│ Other:  ████░░░░░░ 5.0 W           │
│                                     │
│ Total Energy: 145.8 Wh             │  ← Cumulative
│ Session:      12.3 Wh              │
│                                     │
│ 🏍️ Motor Current                    │
│ M1: 2.4 A    M2: 2.6 A             │
└─────────────────────────────────────┘
```

---

## Part 4: Installation & Dependencies

### 4.1 System Dependencies

**Install jtop on Jetson**:
```bash
sudo pip3 install --break-system-packages jetson-stats
```

**Add to Dockerfile** (`/home/ndev/WALL-E/docker/Dockerfile` around line 160):
```dockerfile
# Install Jetson power monitoring
RUN pip3 install --break-system-packages jetson-stats || true
```

Note: The `|| true` allows build to succeed on non-Jetson platforms (development machines).

---

### 4.2 ROS2 Package Updates

**Add to `robot_sensors` package**:
- New file: `power_monitor_node.py`
- Update `setup.py` entry points
- Update `package.xml` dependencies

**Add to `robot_messages` package**:
- New message: `JetsonPowerStatus.msg`

---

### 4.3 Launch File Integration

**File**: `/home/ndev/WALL-E/ros2_ws/src/robot_sensors/launch/sensors_launch.py`

Add power monitor node to existing sensor launch file:
```python
power_monitor_node = Node(
    package='robot_sensors',
    executable='power_monitor_node',
    name='power_monitor',
    parameters=[{
        'publish_rate': 1.0,  # Hz
        'energy_save_interval': 300.0,  # seconds (5 min)
        'energy_file_path': '/tmp/shared/jetson_energy.json',
    }],
    output='screen'
)
```

---

## Part 5: Fallback & Error Handling

### 5.1 Graceful Degradation

**If jtop is not available** (non-Jetson platforms):
- Power monitor node should log warning and exit gracefully
- Backend should return 0.0 for power fields
- Frontend should hide/gray out power monitoring section

**If jtop fails to read data**:
- Log error but continue publishing with last known values
- Set error flag in message
- Display "N/A" or "Error" in frontend

---

### 5.2 Platform Detection

```python
def is_jetson_platform():
    """Check if running on Jetson hardware."""
    try:
        with open('/proc/device-tree/model', 'r') as f:
            model = f.read().lower()
            return 'jetson' in model or 'tegra' in model
    except:
        return False
```

---

## Part 6: Testing Strategy

### 6.1 Unit Tests
- Test energy integration math (trapezoidal rule)
- Test persistence save/load
- Test platform detection

### 6.2 Integration Tests
1. **On Jetson**: Verify power readings are reasonable (5-30W range)
2. **On x86**: Verify graceful fallback (no crashes)
3. **In Docker**: Verify node starts and publishes
4. **Over ZeroTier**: Verify frontend receives power data remotely

### 6.3 Manual Verification
- Run `ros2 topic echo /jetson/power` and verify data
- Load the robot, verify power draw increases
- Check energy file persists across restarts
- Verify frontend updates in real-time

---

## Part 7: File Modifications Summary

### Files to CREATE:
1. `/home/ndev/WALL-E/ros2_ws/src/robot_messages/msg/JetsonPowerStatus.msg`
2. `/home/ndev/WALL-E/ros2_ws/src/robot_sensors/robot_sensors/power_monitor_node.py`
3. `/home/ndev/WALL-E/web_app/frontend/.env`

### Files to MODIFY:
1. `/home/ndev/WALL-E/web_app/frontend/src/App.tsx` (line 19)
2. `/home/ndev/WALL-E/web_app/backend/routers/robot.py` (lines 27-93)
3. `/home/ndev/WALL-E/web_app/frontend/src/components/StatsPanel.tsx`
4. `/home/ndev/WALL-E/docker/Dockerfile` (line ~160)
5. `/home/ndev/WALL-E/ros2_ws/src/robot_sensors/setup.py`
6. `/home/ndev/WALL-E/ros2_ws/src/robot_sensors/package.xml`
7. `/home/ndev/WALL-E/ros2_ws/src/robot_sensors/launch/sensors_launch.py`

---

## Part 8: Execution Order

### Phase 1: Critical Fixes (Minion)
1. Fix hardcoded WebSocket URL in App.tsx
2. Create .env file for frontend

### Phase 2: ROS2 Backend (Minion + Manual)
3. Create JetsonPowerStatus message
4. Create power_monitor_node.py
5. Update package.xml and setup.py
6. Update sensors launch file
7. Add jtop to Dockerfile

### Phase 3: Web Backend (Minion)
8. Enhance RobotStats model in robot.py
9. Add /jetson/power subscription
10. Update /api/robot/stats endpoint

### Phase 4: Web Frontend (Minion)
11. Enhance StatsPanel.tsx with power UI
12. Add power stats interface types
13. Wire up API calls

### Phase 5: Testing (Manual)
14. Build Docker image
15. Test on Jetson hardware
16. Test over ZeroTier
17. Verify energy persistence

---

## Part 9: Expected Behavior After Implementation

### On Jetson in Field:
- Web dashboard shows live power draw updating every second
- Power breakdown shows CPU/GPU/System distribution
- Total energy counter increments continuously
- Energy persists across container restarts
- Accessible over ZeroTier VPN

### On x86 Development Machine:
- Power monitoring section shows "N/A" or is hidden
- Other web app features work normally
- No crashes or errors from missing jtop

### Performance Impact:
- Minimal: 1 Hz publish rate, simple math operations
- Energy file I/O: once per 5 minutes + on shutdown
- Frontend update: same as other stats (polling or WebSocket)

---

## References

### Jetson Power Monitoring
- [jtop GitHub Repository](https://github.com/rbonghi/jetson_stats)
- [NVIDIA Jetson Power Optimization](https://developer.nvidia.com/blog/power-optimization-with-nvidia-jetson)
- [NVIDIA Developer Forums - Extract Power from jtop](https://forums.developer.nvidia.com/t/extract-power-mw-or-power-consumption-from-jtop-in-python/82001)

### Existing WALL-E Code References
- Roboclaw status: `/home/ndev/WALL-E/ros2_ws/src/roboclaw_driver/roboclaw_driver/roboclaw_node.py:379-388`
- RoboclawStatus message: `/home/ndev/WALL-E/ros2_ws/src/robot_messages/msg/RoboclawStatus.msg`
- Robot API: `/home/ndev/WALL-E/web_app/backend/routers/robot.py:27-120`

---

## Success Criteria

✅ WebSocket URL uses environment variable (ZeroTier access works)
✅ Live Jetson power draw displays on dashboard
✅ Power breakdown shows CPU/GPU/System components
✅ Total energy (Wh) tracks cumulative consumption
✅ Session energy (Wh) tracks per-session consumption
✅ Energy persists across container restarts
✅ Graceful fallback on non-Jetson platforms
✅ Motor current from Roboclaw integrated
✅ All features testable in Docker on any computer
✅ Remote access over ZeroTier verified

---

## Estimated Complexity

- **Critical Fixes**: Simple (30 min)
- **ROS2 Message & Node**: Medium (2-3 hours)
- **Backend API Enhancement**: Simple-Medium (1 hour)
- **Frontend UI Enhancement**: Medium (2 hours)
- **Testing & Integration**: Medium (2 hours)

**Total**: ~1 day of focused development

---

## Next Steps

This plan is ready for execution. I recommend:

1. **Review this plan** - Confirm the approach and UI design
2. **Clarify requirements** - Any specific power monitoring features needed?
3. **Execute in phases** - Start with critical fixes, then add power monitoring
4. **Delegate to Minion** - Most implementation is mechanical and suitable for Haiku

Would you like me to proceed with implementation?
