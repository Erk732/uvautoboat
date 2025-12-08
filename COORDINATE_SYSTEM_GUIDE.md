# Coordinate System & Goal Targeting Guide

## Current Setup

### Frame Definition
```
World Frame (ENU - East-North-Up):
├─ Origin: First GPS fix location (automatically set)
├─ X-axis: East (positive = east)
├─ Y-axis: North (positive = north)
└─ Z-axis: Up (positive = up)
```

### Key Points
1. **Ship starting position** = NOT (0, 0) necessarily
2. **Goal coordinates** = Must match ship's current position frame
3. **Frame ID** = Must be "world" in both pose and goal messages

---

## How to Find Ship's Starting Position

```bash
# Check current position (run after launch)
ros2 topic echo /wamv/pose_filtered --once

# You'll see something like:
# header:
#   frame_id: 'world'
# pose:
#   position:
#     x: -0.975        ← Ship's X coordinate
#     y: 25.282        ← Ship's Y coordinate
#     z: 0.0           ← Ship's Z coordinate
```

**This is your starting position. Use this as reference.**

---

## How to Send Correct Goal

### Option 1: Relative Goal (from starting position)
If ship starts at (-0.975, 25.282) and you want it to go:
- 10m East and 10m North from start

```bash
# Calculate: start + offset
# X = -0.975 + 10 = 9.025
# Y = 25.282 + 10 = 35.282

ros2 topic pub -1 /planning/goal geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: world}, pose: {position: {x: 9.025, y: 35.282, z: 0.0}, orientation: {w: 1.0}}}'
```

### Option 2: Absolute Goal (from origin if using (0,0) as reference)
If you want to use (0, 0) as starting position reference:
- Set goal relative to (0, 0)
- **BUT** need to offset by starting position!

```bash
# If start is at (-0.975, 25.282), to reach (0, 0):
# X = -0.975
# Y = 25.282

ros2 topic pub -1 /planning/goal geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: world}, pose: {position: {x: -0.975, y: 25.282, z: 0.0}, orientation: {w: 1.0}}}'
```

---

## Verify Goal Direction (Important!)

After sending goal, check if path points toward it:

```bash
# Send goal
ros2 topic pub -1 /planning/goal geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: world}, pose: {position: {x: 50.0, y: 50.0, z: 0.0}, orientation: {w: 1.0}}}'

# Check generated path
ros2 topic echo /planning/path --once 2>/dev/null | head -50

# Look at the poses - they should progressively move from:
# Start (x: ..., y: ...) → Goal (x: 50.0, y: 50.0)
```

---

## Manual Direction Verification

```bash
# Get current position
BOAT_X=$(ros2 topic echo /wamv/pose_filtered --once 2>/dev/null | grep -A1 "position:" | tail -1 | grep -oP '(?<=x: )\-?[0-9.]+')
BOAT_Y=$(ros2 topic echo /wamv/pose_filtered --once 2>/dev/null | grep -A2 "position:" | tail -1 | grep -oP '(?<=y: )\-?[0-9.]+')

# Your goal
GOAL_X=50.0
GOAL_Y=50.0

# Direction vector
DX=$(echo "$GOAL_X - $BOAT_X" | bc)
DY=$(echo "$GOAL_Y - $BOAT_Y" | bc)
DISTANCE=$(echo "sqrt($DX*$DX + $DY*$DY)" | bc -l)
ANGLE=$(echo "atan2($DY, $DX) * 180 / 3.14159" | bc -l)

echo "Ship at: ($BOAT_X, $BOAT_Y)"
echo "Goal at: ($GOAL_X, $GOAL_Y)"
echo "Direction: ${ANGLE}° (bearing)"
echo "Distance: $DISTANCE m"
```

---

## Common Mistakes

❌ **WRONG**: Assuming (0, 0) is the origin
- The actual origin is the first GPS fix, which may be anywhere

❌ **WRONG**: Sending goal in different frame than ship
- Goal frame_id must match /wamv/pose_filtered frame_id (should be "world")

❌ **WRONG**: Not checking if path points toward goal
- Always verify /planning/path after sending goal

✅ **CORRECT**: Get ship position, calculate goal offset, verify path

---

## Complete Diagnostic Sequence

```bash
# Step 1: Start system
ros2 launch control all_in_one_bringup.launch.py &
sleep 3

# Step 2: Get ship position
echo "=== Ship Starting Position ==="
POSE=$(ros2 topic echo /wamv/pose_filtered --once 2>/dev/null)
echo "$POSE" | grep -A3 "position:"

# Step 3: Extract coordinates
SHIP_X=$(echo "$POSE" | grep -A1 "position:" | tail -1 | grep -oP '(?<=x: )\-?[0-9.]+')
SHIP_Y=$(echo "$POSE" | grep -A2 "position:" | tail -1 | grep -oP '(?<=y: )\-?[0-9.]+')
echo "Ship at: X=$SHIP_X, Y=$SHIP_Y"

# Step 4: Calculate goal (50m east, 50m north from start)
GOAL_X=$(echo "$SHIP_X + 50" | bc)
GOAL_Y=$(echo "$SHIP_Y + 50" | bc)
echo "Sending goal to: X=$GOAL_X, Y=$GOAL_Y"

# Step 5: Send goal
ros2 topic pub -1 /planning/goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: $GOAL_X, y: $GOAL_Y, z: 0.0}, orientation: {w: 1.0}}}" 2>/dev/null

# Step 6: Verify path
sleep 1
echo "=== Planned Path (first 5 points) ==="
ros2 topic echo /planning/path --once 2>/dev/null | grep -A50 "poses:" | head -40

# Step 7: Check thrusters
echo -e "\n=== Thruster Commands ==="
timeout 3 bash -c 'for i in {1..3}; do echo "Left: $(ros2 topic echo /wamv/thrusters/left/thrust --once 2>/dev/null | grep data)"; sleep 1; done'
```

---

## Summary

| Question | Answer |
|----------|--------|
| What is origin (0,0)? | First GPS fix (auto-set, not controllable) |
| Where does ship start? | Get from `/wamv/pose_filtered` |
| How to send goal? | Use ship position + offset in world frame |
| How to verify direction? | Check `/planning/path` - should point toward goal |
| Frame ID to use? | Always "world" to match `/wamv/pose_filtered` |

