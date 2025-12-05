# Boat Not Moving Toward Goal - Troubleshooting Guide

## Quick Checklist

### ✓ 1. Is the node running?
```bash
ros2 node list | grep all_in_one_stack
```
Expected: `/all_in_one_stack`

If not found, start it:
```bash
ros2 launch control all_in_one_bringup.launch.py
```

---

### ✓ 2. Is pose data available?
```bash
ros2 topic echo /wamv/pose_filtered --once
```
Expected: See position (x, y) and orientation (w: 1.0)

If empty or timeout:
- Check GPS is working: `ros2 topic echo /wamv/sensors/gps/gps/fix --once`
- Check IMU is working: `ros2 topic echo /wamv/sensors/imu/imu/data --once`
- If GPS/IMU working but pose not, restart pose_filter node

---

### ✓ 3. Is lidar data available?
```bash
ros2 topic echo /wamv/sensors/lidars/lidar_wamv_sensor/scan --once
```
Expected: See ranges array with distance values

If timeout, try alternative:
```bash
ros2 topic echo /wamv/sensors/lidars/lidar_wamv/scan --once
```

---

### ✓ 4. Are thrusters getting commands?
```bash
ros2 topic echo /wamv/thrusters/left/thrust
ros2 topic echo /wamv/thrusters/right/thrust
```
Expected: Non-zero values when goal is sent

If always 0.0:
- Problem: Control logic not activating
- Solution: Check if goal was received

---

### ✓ 5. Did the goal get received?
```bash
ros2 topic echo /planning/goal --once
```

Then send goal:
```bash
ros2 topic pub -1 /planning/goal geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: world}, pose: {position: {x: 10.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'
```

---

## Common Problems & Solutions

### Problem 1: Node not activating (never sends thrusters)
**Symptoms**: Thrusters always 0.0, no log output
**Cause**: Goal not being received or pose missing

**Solution**:
```bash
# Check active status
ros2 param get /all_in_one_stack | grep active

# Manually send goal
ros2 topic pub -1 /planning/goal geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: world}, pose: {position: {x: 20.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'

# Watch logs
ros2 launch control all_in_one_bringup.launch.py 2>&1 | grep -E "goal|active|path"
```

---

### Problem 2: Thrusters active but boat not moving
**Symptoms**: Thrust commands non-zero, but no physical movement
**Cause**: Thruster saturation or hardware issue

**Solution**:
```bash
# Check thrust magnitude
ros2 topic echo /wamv/thrusters/left/thrust

# If very small (<50), increase forward_thrust
ros2 param set /all_in_one_stack forward_thrust 600.0

# Test with stronger goal
ros2 topic pub -1 /planning/goal geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: world}, pose: {position: {x: 50.0, y: 50.0, z: 0.0}, orientation: {w: 1.0}}}'
```

---

### Problem 3: Boat moves but wrong direction
**Symptoms**: Thrusters active, boat moving but away from goal
**Cause**: Coordinate frame mismatch or heading error

**Solution**:
```bash
# Check frame_id matches
ros2 topic echo /wamv/pose_filtered --once | grep frame_id
# Should be: frame_id: world

# Send goal in same frame
ros2 topic pub -1 /planning/goal geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: world, stamp: {sec: 0}}, pose: {position: {x: 10.0, y: 0.0}, orientation: {w: 1.0}}}'

# Check heading (yaw)
ros2 topic echo /wamv/pose_filtered --once | grep -A5 orientation
```

---

### Problem 4: Boat moving but slowly/unreliably
**Symptoms**: Some thrusts work, some don't, inconsistent movement
**Cause**: Parameters too conservative or obstacles blocking

**Solution**:
```bash
# Increase forward thrust
ros2 param set /all_in_one_stack forward_thrust 700.0

# Reduce waypoint tolerance for closer goal following
ros2 param set /all_in_one_stack waypoint_tolerance 2.0

# Check if stuck in avoidance
ros2 launch control all_in_one_bringup.launch.py 2>&1 | grep -i "obstacle\|avoid\|stuck"
```

---

## Step-by-Step Diagnostic

Run this sequence:

```bash
# Terminal 1: Start the system
ros2 launch control all_in_one_bringup.launch.py

# Terminal 2: Check pose
ros2 topic echo /wamv/pose_filtered --once

# Terminal 3: Check lidar
ros2 topic echo /wamv/sensors/lidars/lidar_wamv_sensor/scan --once | head -20

# Terminal 4: Send goal
ros2 topic pub -1 /planning/goal geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: world}, pose: {position: {x: 10.0, y: 0.0}, orientation: {w: 1.0}}}'

# Terminal 5: Monitor thrusters
watch -n 0.1 'ros2 topic echo /wamv/thrusters/left/thrust --once'
```

---

## Key Parameters to Check

| Parameter | What to do |
|-----------|-----------|
| `forward_thrust` | Increase if moving too slow (default 400) |
| `goal_tolerance` | Decrease if boat stops too far (default 1.0) |
| `kp_yaw` | Decrease if oscillating (default 600) |
| `control_rate` | Increase for smoother (default 20 Hz) |
| `obstacle_slow_dist` | Increase if avoiding when shouldn't (default 15) |

Quick tune:
```bash
ros2 param set /all_in_one_stack forward_thrust 600.0
ros2 param set /all_in_one_stack goal_tolerance 0.5
```

---

## If Still Not Working

**Enable verbose logging**:
```bash
# Set env var and run
ROS_LOG_LEVEL=debug ros2 launch control all_in_one_bringup.launch.py
```

**Check active flag** (node must have received a goal):
```bash
ros2 param get /all_in_one_stack | grep -E "active|waypoint"
```

**Manually inspect internal state** (if you have debug features):
```bash
# Some nodes publish debug info
ros2 topic list | grep debug
ros2 topic list | grep status
```

