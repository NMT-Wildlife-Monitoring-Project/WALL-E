# RViz Map Shift Issue - Root Cause Analysis

## Problem Summary
The RViz map display shifts off-screen immediately after startup, making it difficult to view the robot's navigation environment.

## Root Causes Identified

### 1. **Incorrect Camera View Settings in `rviz2/default.rviz`**

The RViz configuration file contains problematic camera/view parameters:

**File:** `/home/emiliano-mota/WALL-E/rviz2/default.rviz` (lines 252-271)

```yaml
Views:
  Current:
    Class: rviz_default_plugins/Orbit
    Distance: 10.235515594482422      # ⚠️ Camera zoom distance is very small
    Focal Point:
      X: 0
      Y: 0
      Z: 0
    Pitch: 1.5697963237762451        # ⚠️ Pitch ~1.57 radians = 90°, looking straight down
    Yaw: 6.263583660125732           # ⚠️ Yaw angle is non-standard
    Target Frame: <Fixed Frame>
    Value: Orbit (rviz)
```

**Key Issues:**

1. **Distance: 10.24** - This is extremely close to the origin. The camera is zoomed in too tightly.
   - When the robot starts at GPS origin or odometry origin (0,0,0), this close zoom means the map extends far beyond the visible view
   - As the robot moves away from origin, the camera doesn't follow properly

2. **Pitch: ~1.57 radians (90°)** - Looking straight down
   - This is actually correct for a top-down map view
   - However, combined with the small distance, it can cause issues

3. **Yaw: 6.26 radians** - Non-standard rotation
   - This is about 358°, which is nearly a full rotation
   - Creates an odd viewing angle relative to the robot frame

### 2. **Reference Frame Configuration**

**File:** `/home/emiliano-mota/WALL-E/rviz2/default.rviz` (line 243)

```yaml
Global Options:
  Background Color: 48; 48; 48
  Fixed Frame: map        # ✓ Correct - should be 'map'
  Frame Rate: 30
```

The fixed frame is correctly set to `map`, which is good. However, the camera focal point is at (0,0,0) in the map frame.

### 3. **Map Display Topic Issue**

**File:** `/home/emiliano-mota/WALL-E/rviz2/default.rviz` (lines 99-112)

```yaml
- Class: rviz_default_plugins/Map
  Enabled: true
  Name: Map
  Topic:
    Value: /local_costmap/costmap
  Update Topic:
    Value: /local_costmap/costmap_updates
```

**Potential Issue:** The map is displaying `/local_costmap/costmap` instead of `/global_costmap/costmap`
- Local costmap is centered on the robot and moves with it
- If there's a shift in the local costmap or frame alignment, it appears to shift in RViz
- Global costmap maintains a fixed coordinate frame and may be better for this use case

## Why the Map Shifts Off-Screen at Startup

**Sequence of Events:**

1. **RViz starts** → Loads default.rviz configuration
2. **Camera position initialized** → Placed at distance 10.24 from origin (0,0,0)
3. **Robot starts** → May start at origin or at last known GPS/odometry position
4. **Map/costmap publishes** → Local costmap centered on robot publishes its data
5. **Camera follows focal point?** → But focal point is at (0,0,0) while robot is elsewhere
6. **Result** → Map appears to shift because:
   - Camera is zoomed in too tight relative to the map extent
   - Camera focal point doesn't properly track the robot
   - Map is in local costmap frame which can have sudden frame transformations

## Solutions

### Solution 1: Adjust Camera View Settings (Recommended)

Modify `rviz2/default.rviz` to have sensible defaults:

```yaml
Views:
  Current:
    Class: rviz_default_plugins/Orbit
    Distance: 50.0                    # Zoom out significantly (from 10.24)
    Enable Stereo Rendering:
      Stereo Eye Separation: 0.05999999865889549
      Stereo Focal Distance: 1
      Swap Stereo Eyes: false
      Value: false
    Focal Point:
      X: 0                            # Keep at origin
      Y: 0
      Z: 0
    Focal Shape Fixed Size: true
    Focal Shape Size: 0.05000000074505806
    Invert Z Axis: false
    Name: Current View
    Near Clip Distance: 0.009999999776482582
    Pitch: 1.5707963267948966         # 90°, straight down (correct)
    Target Frame: <Fixed Frame>
    Value: Orbit (rviz)
    Yaw: 0                            # Normalized angle (0°)
```

**Key Changes:**
- **Distance: 10.24 → 50.0** - Zooms out to see more of the map
- **Yaw: 6.26 → 0** - Removes the odd angle rotation

### Solution 2: Use Global Costmap Instead of Local Costmap

Change the map display to use global costmap (fixed frame):

```yaml
- Class: rviz_default_plugins/Map
  Enabled: true
  Name: Map
  Topic:
    Value: /global_costmap/costmap    # Changed from /local_costmap/costmap
  Update Topic:
    Value: /global_costmap/costmap_updates
```

**Pros:**
- Global costmap stays in fixed map frame (less frame jumping)
- Provides broader context of mapped areas

**Cons:**
- Requires `/global_costmap/costmap` to be published
- May show empty areas if mapping hasn't covered them

### Solution 3: Enable Camera Follow Mode

If using newer RViz2 versions, enable the "Target Frame" tracking:

```yaml
Views:
  Current:
    Target Frame: base_link           # Track robot base_link instead of <Fixed Frame>
    # ... rest of settings
```

This makes the camera orbit around the robot, solving the "shift" problem entirely.

## Implementation Recommendations

### Step 1: Quick Fix (5 minutes)
Apply Solution 1 only - adjust camera distance and yaw:

```bash
# Edit the default.rviz file
# Change line 253: Distance: 10.235515594482422 → Distance: 50.0
# Change line 271: Yaw: 6.263583660125732 → Yaw: 0.0
```

### Step 2: Optional Enhancement (if local_costmap behavior is undesirable)
Apply Solution 2 - switch to global costmap display

### Step 3: Testing

After making changes:

```bash
# In Docker container:
source install/setup.bash
ros2 launch robot_bringup robot_launch.py

# In another terminal:
ros2 launch robot_navigation gps_waypoint_follower.launch.py use_rviz:=true
```

Then manually verify in RViz:
- [ ] Map appears centered at startup
- [ ] As robot moves, map view stays visible
- [ ] Zoom level is appropriate for navigation monitoring
- [ ] No excessive panning required to see robot position

## Verification Checklist

After applying the fix, verify:

- [ ] RViz starts with map visible (not off-screen)
- [ ] Camera focal point is reasonable for navigation
- [ ] Both `Distance` and `Yaw` values are sensible
- [ ] Robot appears in the center or near-center of view
- [ ] No jerky camera movements when robot moves

## Prevention for Future Issues

**Best Practices for RViz Configurations:**

1. Always save RViz config after manually adjusting the view
2. Use reasonable defaults:
   - Distance: 20-100 (depending on robot size)
   - Pitch: 0.5-1.57 radians (30-90 degrees down)
   - Yaw: 0 or π (normalized angles)
3. Set Target Frame to either:
   - `map` (if tracking global navigation)
   - `base_link` (if tracking robot movements)
   - `<Fixed Frame>` (default, but ensure focal point makes sense)

## Additional Notes

### Why This Affects Startup Specifically

The issue is most noticeable at startup because:
1. Robots typically initialize at origin (0,0,0) in their local frame
2. GPS may take time to acquire, causing sudden coordinate jumps
3. If robot starts far from saved focal point, the shift is dramatic
4. RViz doesn't interpolate camera movement, so sudden jumps appear jarring

### Interaction with Transform Tree

The shift can be exacerbated by:
- Delays in `/map` → `/base_link` transform publishing
- GPS datum mismatch between startup and saved configuration
- Odometry reset or discontinuity at startup

This is why the staging launch (3-second delay before nav2) in `robot_launch.py` helps.

