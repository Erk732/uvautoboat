# How to Use atlantis.launch.yaml

## Overview

Your friend can now use either the Python or YAML launch file - they're equivalent!

---

## Usage Examples

### Basic Launch (All Defaults)

```bash
# Old way (still works):
ros2 launch plan atlantis.launch.py

# New way (YAML format):
ros2 launch plan atlantis.launch.yaml
```

### Custom Parameters

#### Example 1: More Aggressive Obstacle Detection
```bash
ros2 launch plan atlantis.launch.yaml min_safe_distance:=6.0
```
- Detects obstacles closer (6.0m instead of 8.0m)
- Boat turns later (not too early, not too late)

#### Example 2: Bigger Mission Area
```bash
ros2 launch plan atlantis.launch.yaml \
  scan_length:=200.0 \
  scan_width:=30.0 \
  lanes:=5
```
- Creates larger lawnmower pattern
- 200m forward × 30m width × 5 lanes

#### Example 3: Faster/Slower Boat
```bash
# Faster (risky with obstacles):
ros2 launch plan atlantis.launch.yaml base_speed:=700.0

# Slower (safer, more precise):
ros2 launch plan atlantis.launch.yaml base_speed:=300.0
```

#### Example 4: Looser Steering Control
```bash
ros2 launch plan atlantis.launch.yaml \
  kp:=300.0 \
  ki:=15.0 \
  kd:=80.0
```
- More gentle turns
- Less aggressive heading correction

#### Example 5: Full Custom Configuration
```bash
ros2 launch plan atlantis.launch.yaml \
  scan_length:=150.0 \
  scan_width:=25.0 \
  lanes:=4 \
  base_speed:=500.0 \
  min_safe_distance:=8.0 \
  kp:=400.0 \
  ki:=20.0 \
  kd:=100.0
```

---

## YAML vs Python - Differences

### Python Format (atlantis.launch.py)
- More powerful (can use Python logic)
- Used by developers making complex launches
- Has full programming capabilities

### YAML Format (atlantis.launch.yaml)
- ✅ **Simpler to read**
- ✅ **Simpler to edit**
- ✅ **No programming knowledge needed**
- ✅ **Your friend will like this!**
- Less flexible (no conditionals)

Both produce identical results.

---

## Parameter Reference

Copy-paste this when making custom launches:

```bash
ros2 launch plan atlantis.launch.yaml \
  scan_length:=150.0 \          # Forward distance per lane (meters)
  scan_width:=20.0 \            # Width between lanes (meters)
  lanes:=4 \                    # Number of parallel lanes
  frame_id:=map \               # TF frame for visualization
  kp:=400.0 \                   # PID Proportional gain
  ki:=20.0 \                    # PID Integral gain
  kd:=100.0 \                   # PID Derivative gain
  base_speed:=500.0 \           # Forward thrust speed
  max_speed:=800.0 \            # Maximum speed limit
  waypoint_tolerance:=2.0 \     # Distance to waypoint (meters)
  min_safe_distance:=8.0 \      # Obstacle detection distance (meters)
  critical_distance:=2.0 \      # Emergency reverse distance (meters)
  obstacle_slow_factor:=0.1 \   # Speed reduction when avoiding
  hysteresis_distance:=1.0 \    # Oscillation prevention buffer
  reverse_timeout:=10.0 \       # Max reverse duration (seconds)
  stuck_timeout:=5.0 \          # Stuck detection timeout (seconds)
  stuck_threshold:=1.0 \        # Min movement to not be stuck (meters)
  no_go_zone_radius:=8.0 \      # Stuck zone avoidance radius
  drift_compensation_gain:=0.3 \# Wind/current compensation
  probe_angle:=45.0 \           # Escape direction probing angle
  detour_distance:=12.0         # Escape maneuver distance (meters)
```

---

## Best Practices for Your Friend

### For Safe Testing:
```bash
# Conservative - easier to debug
ros2 launch plan atlantis.launch.yaml \
  min_safe_distance:=10.0 \
  base_speed:=300.0
```

### For Proven Settings (Current Working):
```bash
# This is what works right now:
ros2 launch plan atlantis.launch.yaml
# (all defaults are the proven working values)
```

### For Coverage Speed:
```bash
# Balanced - good speed without risky behavior
ros2 launch plan atlantis.launch.yaml \
  base_speed:=600.0 \
  min_safe_distance:=8.0
```

---

## Troubleshooting

### "File not found" error?
Make sure you're in the right directory:
```bash
cd ~/seal_ws
source install/setup.bash
ros2 launch plan atlantis.launch.yaml
```

### Both .py and .yaml are in same folder?
Yes! You can use either one:
```bash
# These do the same thing:
ros2 launch plan atlantis.launch.py
ros2 launch plan atlantis.launch.yaml
```

### Want to keep Python version?
You can! We didn't delete it. Use whichever you prefer.

---

## For Your Friend

Tell them:
- ✅ New YAML format is simpler
- ✅ Parameters are clearly labeled
- ✅ Easy to read and modify
- ✅ Copy-paste friendly
- ✅ No Python knowledge needed

Example for your friend:
```bash
# Start here (simple):
ros2 launch plan atlantis.launch.yaml

# Then try this (if you want faster):
ros2 launch plan atlantis.launch.yaml base_speed:=700.0

# Or bigger area:
ros2 launch plan atlantis.launch.yaml scan_length:=200.0 lanes:=6
```

---

**File Location:** `/home/bot/seal_ws/src/uvautoboat/launch/atlantis.launch.yaml`

**Status:** ✅ Ready to use
