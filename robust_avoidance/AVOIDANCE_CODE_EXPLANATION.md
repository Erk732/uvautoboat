# all_in_one_stack.py Obstacle Avoidance Detailed Explanation

## Code Location Summary

```text
Function declarations:
├─ analyze_lidar()             Line 508  ← Core! Get obstacle distances
├─ _analyze_pointcloud()       Line 556  ← Process 3D point cloud data
├─ _polar_bias_from_scan()     Line 641  ← Polar histogram steering
├─ _vfh_steer()                Line 670  ← VFH vector field steering
└─ control_loop()              Line 741  ← Main control loop (includes avoidance logic)

Key parameters: Lines 110-140
Key states: Lines 224-237
```

---

## Part 1: Parameter Declaration and Initialization (Lines 60-145)

### Obstacle Avoidance Parameters

```python
# Lines 123-137 - Obstacle avoidance parameter definitions

self.declare_parameter('obstacle_slow_dist', 15.0)     # Distance to activate soft avoidance (m)
self.declare_parameter('obstacle_stop_dist', 8.0)      # Distance to activate hard avoidance (m)
self.declare_parameter('avoid_turn_thrust', 350.0)     # Turning thrust during hard avoidance (N)
self.declare_parameter('avoid_diff_gain', 40.0)        # Left/right steering gain
self.declare_parameter('avoid_clear_margin', 3.0)      # Safety margin to exit avoidance (m)
self.declare_parameter('avoid_max_turn_time', 5.0)     # Max duration for hard avoidance (s)
self.declare_parameter('full_clear_distance', 60.0)    # [X] Key issue! Global clear distance

# Line 140 - Lidar parameters
self.declare_parameter('front_angle_deg', 30.0)        # Front scan angle (deg)
self.declare_parameter('side_angle_deg', 60.0)         # Side scan angle (deg)
self.declare_parameter('min_range_filter', 1.5)        # Minimum distance filter (m)
```

### Parameter Meaning Explanation

```text
obstacle_slow_dist: 15.0
  Meaning: When front obstacle < 15m, trigger "soft avoidance"
  Effect: Reduce thrust to 20-100%, while steering

obstacle_stop_dist: 8.0
  Meaning: When front obstacle < 8m, trigger "hard avoidance"
  Effect: Rotate in place (large thrust differential), wait for obstacle to clear

avoid_turn_thrust: 350.0
  Meaning: Turning thrust magnitude during hard avoidance
  Usage: Calculate difference between left_cmd and right_cmd

avoid_diff_gain: 40.0
  Meaning: Steering gain based on left/right lidar distance difference
  Range: [-40, +40]
  Rule: Left side clear → turn left (positive gain)
        Right side clear → turn right (negative gain)

full_clear_distance: 60.0 [X][X][X]
  Meaning: Only exit avoidance mode when all directions > 60m
  Problem: Lidar range is only ~30m, cannot reach 60m!
  Result: Avoidance mode permanently activated!
```

---

## Part 2: Lidar Analysis (Lines 508-639)

### Core Function: analyze_lidar()

```python
def analyze_lidar(self):
    """
    Returns (front_min, left_min, right_min) three distance values

    Front:   From -front_angle to +front_angle
             Default: -30 deg to +30 deg (60 deg total)

    Left:    From 0 deg to +side_angle
             Default: 0 deg to +60 deg

    Right:   From -side_angle to 0 deg
             Default: -60 deg to 0 deg

    Diagram (top view):
                Front 30 deg
              /      |      \
            /        |        \
          /          |          \
        Left 60 deg  Boat  Right 60 deg
          \          |          /
            \        |        /
              \      |      /

    """

    # Line 516: First try to get from 3D point cloud
    front_min_cloud, left_min_cloud, right_min_cloud = self._analyze_pointcloud()

    # Line 519: Then get from 2D laser scan
    if self.latest_scan is not None:
        scan = self.latest_scan
        angle = scan.angle_min          # Start angle (usually -pi)
        f = l = r = float('inf')        # Initialize to infinity

        for rng in scan.ranges:          # Iterate through each laser ray
            # Line 525: Filter invalid data
            if math.isinf(rng) or math.isnan(rng) or rng <= 0.0:
                angle += scan.angle_increment
                continue

            # Line 528: Filter too-close data (might be self)
            if rng < self.min_range_filter:  # min_range_filter = 1.5m
                angle += scan.angle_increment
                continue

            # Lines 531-539: Classify by angle (front/left/right)
            if -self.front_angle <= angle <= self.front_angle:
                f = min(f, rng)          # Record front minimum distance

            if 0.0 <= angle <= self.side_angle:
                l = min(l, rng)          # Record left minimum distance

            if -self.side_angle <= angle <= 0.0:
                r = min(r, rng)          # Record right minimum distance

            angle += scan.angle_increment

    # Line 546: Merge point cloud and scan data
    # Prefer point cloud (more accurate), otherwise use scan
    front_min = front_min_cloud or front_min_scan
    left_min = left_min_cloud or left_min_scan
    right_min = right_min_cloud or right_min_scan

    return front_min, left_min, right_min
```

### Sub-function: _analyze_pointcloud()

```python
def _analyze_pointcloud(self):
    """
    Process 3D point cloud data (PointCloud2 format)

    Why need 3D point cloud?
    └─ LaserScan is 2D (only range and angle)
    └─ Point cloud has Z coordinate, can detect obstacles below water (like dock piles)

    """

    if self.latest_cloud is None:
        return None, None, None

    # Line 568: Read each point in the point cloud
    for p in point_cloud2.read_points(self.latest_cloud,
                                      field_names=('x', 'y', 'z')):
        x, y, z = p

        # Line 571: Z-axis filtering
        if z < self.cloud_z_min or z > self.cloud_z_max:
            # cloud_z_min = -10.0 (below water surface)
            # cloud_z_max = 3.0   (above water surface)
            # Only keep points within this height range
            continue

        # Line 574: Calculate horizontal distance
        dist = math.hypot(x, y)          # sqrt(x^2 + y^2)

        # Lines 575-576: Distance filtering
        if dist <= 0.0 or dist < self.min_range_filter:
            continue

        # Line 577: Calculate angle
        angle = math.atan2(y, x)        # Measured counterclockwise from positive X-axis

        # Lines 579-589: Classify by angle (same as analyze_lidar)
        if -self.front_angle <= angle <= self.front_angle:
            front_min = dist if front_min is None else min(front_min, dist)

        if 0.0 <= angle <= self.side_angle:
            left_min = dist if left_min is None else min(left_min, dist)

        if -self.side_angle <= angle <= 0.0:
            right_min = dist if right_min is None else min(right_min, dist)

    return front_min, left_min, right_min
```

---

## Part 3: Steering Algorithms (Lines 641-720)

### Sub-function: _polar_bias_from_scan() (Polar Histogram)

```python
def _polar_bias_from_scan(self):
    """
    Simple polar histogram: Compare "free space" on left and right sides

    Principle:
    └─ Iterate through all rays in the laser scan
    └─ Left side rays (angle > 0) contribute to left_score
    └─ Right side rays (angle < 0) contribute to right_score
    └─ Weight is range^power (farther distance = higher weight)
    └─ Finally calculate bias: (left - right) / total

    Return value range: [-1, 1]
    └─ +1:  Strong turn left (left side is spacious)
    └─ 0:   No preference (left and right equally spacious)
    └─ -1:  Strong turn right (right side is spacious)
    """

    if not self.polar_use_scan or self.latest_scan is None:
        return 0.0

    scan = self.latest_scan
    angle = scan.angle_min              # Start angle -pi
    step = scan.angle_increment         # Angle increment ~0.006 rad
    left_score = 0.0
    right_score = 0.0
    power = max(self.polar_weight_power, 0.0)  # Usually 1.0

    for r in scan.ranges:               # Iterate through each ray
        # Line 656: Distance filtering
        if r < self.polar_min_range:    # Minimum 0.5m
            r = self.polar_min_range

        # Line 657: Calculate weight (power of distance)
        w = r ** power                  # Usually r^1 = r

        # Lines 658-661: Left/right classification and accumulation
        if angle > 0.0:
            left_score += w             # Accumulate left side
        else:
            right_score += w            # Accumulate right side

        angle += step

    # Line 662: Normalize
    total = left_score + right_score
    if total <= 0.0:
        return 0.0

    bias = (left_score - right_score) / total
    # Result is in [-1, 1] range
    return bias
```

### Sub-function: _vfh_steer() (Vector Field Histogram)

```python
def _vfh_steer(self, desired_yaw: float):
    """
    Vector Field Histogram (VFH) algorithm: Find the "free" direction closest to target heading

    Working steps:
    1. Divide 360 deg scan into multiple "bins" (usually 5 deg each)
    2. Mark bins that have obstacles (distance < vfh_block_dist)
    3. Mark areas around blocked bins as "blocked" (inflation processing)
    4. Among all free bins, select the one closest to target heading

    """

    if not self.vfh_enabled or self.latest_scan is None:
        return None

    scan = self.latest_scan
    bin_rad = math.radians(max(self.vfh_bin_deg, 1e-3))  # 5 deg = 0.087 rad
    num_bins = int(math.ceil((scan.angle_max - scan.angle_min) / bin_rad))
    blocked = [False] * num_bins

    # Lines 691-702: Mark blocked bins
    angle = scan.angle_min
    step = scan.angle_increment
    for r in scan.ranges:
        idx = int((angle - scan.angle_min) / bin_rad)
        if 0 <= idx < num_bins:
            if r > 0.0 and r < self.vfh_block_dist:  # vfh_block_dist = 10.0m
                blocked[idx] = True
        angle += step

    # Lines 703-714: Inflate blocked bins (add safety margin)
    clearance = math.radians(self.vfh_clearance_deg)
    inflate_bins = int(math.ceil(clearance / bin_rad))
    if inflate_bins > 0:
        blocked_inf = blocked[:]
        for i, b in enumerate(blocked):
            if not b:
                continue
            # Mark surrounding bins as also blocked
            for k in range(-inflate_bins, inflate_bins + 1):
                j = i + k
                if 0 <= j < num_bins:
                    blocked_inf[j] = True
        blocked = blocked_inf

    # Lines 715-729: Find free bin closest to target heading
    desired_idx = int((desired_yaw - scan.angle_min) / bin_rad)
    best_idx = None
    best_err = None

    for i, b in enumerate(blocked):
        if b:                          # Skip blocked bins
            continue
        center_ang = scan.angle_min + (i + 0.5) * bin_rad
        err = abs(math.atan2(math.sin(center_ang - desired_yaw),
                             math.cos(center_ang - desired_yaw)))
        if best_err is None or err < best_err:
            best_err = err
            best_idx = i

    if best_idx is None:
        return None

    return scan.angle_min + (best_idx + 0.5) * bin_rad
```

---

## Part 4: Main Obstacle Avoidance Control Logic (Lines 887-1015)

This is the most critical part! Let me explain the three stages of obstacle avoidance in detail:

### Stage 1: Global Avoidance Mode Activation/Exit (Lines 887-895)

```python
# ========== KEY CODE! ==========

# Force avoidance whenever any sector is below full_clear_distance;
# resume only when all clear
clear_val = self.full_clear_distance       # Default 60.0 [X][X][X]

f_val = front_min if front_min is not None else clear_val
l_val = left_min if left_min is not None else clear_val
r_val = right_min if right_min is not None else clear_val

if f_val < clear_val or l_val < clear_val or r_val < clear_val:
    self.force_avoid_active = True      # Activate global avoidance mode
elif f_val >= clear_val and l_val >= clear_val and r_val >= clear_val:
    self.force_avoid_active = False     # Exit global avoidance mode
```

**Problem Analysis:**

```python
# Typical lidar data:
front_min = 25.0    # Front 25m (no obstacle)
left_min = 30.0     # Left 30m (end of scan range)
right_min = 30.0    # Right 30m (end of scan range)

# Evaluation (clear_val = 60.0):
f_val = 25.0 < 60.0 -> True     [X] Activated!
l_val = 30.0 < 60.0 -> True     [X] Stay activated!
r_val = 30.0 < 60.0 -> True     [X] Stay activated!

Result: force_avoid_active = True  (Permanently stuck!)

To exit requires: 25 >= 60 AND 30 >= 60 AND 30 >= 60
This can never be satisfied (lidar can only see up to 30m)
```

### Stage 2: Hard Avoidance State Machine (Lines 908-925)

```python
# Two phases of hard avoidance: Reverse + Turn

if self.avoid_mode in ('reverse', 'turn'):
    if self.avoid_mode == 'reverse':
        # Phase 1: Reverse (for recover_reverse_time = 3.0 seconds)
        if (now_s - self.avoid_start_time) < self.recover_reverse_time:
            self.publish_thrust(self.recover_reverse_thrust,    # -200.0
                               self.recover_reverse_thrust)    # -200.0
            # Both thrusts are negative, boat moves backward
            return

        # After 3 seconds, switch to turning
        self.avoid_mode = 'turn'
        self.avoid_start_time = now_s

    if self.avoid_mode == 'turn':
        # Phase 2: Turn (direction based on left/right obstacles)
        clear_dist = self.obstacle_stop_dist + self.avoid_clear_margin
        time_in_turn = now_s - self.avoid_start_time

        if (front_min is None or front_min > clear_dist) or (time_in_turn > self.avoid_max_turn_time):
            # Front clear or turn timeout, end hard avoidance
            self.avoid_mode = ''
            self.avoid_start_time = 0.0
        else:
            # Continue turning
            turn_cmd = self.avoid_turn_thrust * self.avoid_turn_dir
            self.publish_thrust(-turn_cmd, turn_cmd)  # Left thruster push, right pull (or vice versa)
            return
```

### Stage 3: Soft Avoidance (Lines 934-1010)

```python
# When obstacle is between obstacle_slow_dist (12m) and obstacle_stop_dist (6m)

if front_min_eff < self.obstacle_slow_dist:  # 12.0
    # Soft avoidance: Reduce thrust + steering

    denom = max(self.obstacle_slow_dist - self.obstacle_stop_dist, 0.1)
    # denom = 12 - 6 = 6

    scale = (front_min_eff - self.obstacle_stop_dist) / denom
    # When front_min = 6m, scale = 0 / 6 = 0
    # When front_min = 12m, scale = 6 / 6 = 1

    scale = max(0.2, min(1.0, scale))
    # Clamp to [0.2, 1.0], reduce to minimum 20% thrust

    left_cmd *= scale       # [X] Thrust is weakened
    right_cmd *= scale      # [X] Thrust is weakened

    # ===================== KEY: Steering Calculation =====================

    # Method 1: Steer based on left/right distance difference
    diff_bias = (right_min_eff - left_min_eff) / norm * self.avoid_diff_gain
    # Left side clear -> right_min > left_min -> diff_bias > 0 -> turn left
    # Right side clear -> right_min < left_min -> diff_bias < 0 -> turn right

    # Method 2: VFH steering (select safest direction)
    vfh_angle = self._vfh_steer(desired_yaw)
    if vfh_angle is not None:
        rel = normalize_angle(vfh_angle)
        diff_bias += max(-1.0, min(1.0, rel / max(self.front_angle, 1e-3))) * self.avoid_diff_gain

    # Method 3: Polar histogram (compare left/right free space)
    if self.force_avoid_active or front_min_eff < self.obstacle_slow_dist:
        polar_bias = self._polar_bias_from_scan()
        # polar_bias in [-1, 1]
        # +1: Left side spacious
        # -1: Right side spacious
        diff_bias += polar_bias * self.avoid_diff_gain

    # [X] Problem: When force_avoid_active=True
    # polar_bias will override the intent to steer toward goal!

    # Finally: Apply steering bias
    left_cmd -= diff_bias       # When turning left, left thruster weakens
    right_cmd += diff_bias      # When turning left, right thruster strengthens
```

---

## Root Cause of Avoidance Getting Stuck

```text
Timeline:

T=0s: Receive goal -> Path generated -> Start navigation

T=1s: No obstacle ahead
     front_min = 25m > 60m? NO
     force_avoid_active = TRUE  [X] Activated

T=2s: Still no obstacle ahead, but still activated
     left_min = 30m > 60m? NO
     right_min = 30m > 60m? NO
     force_avoid_active = TRUE  [X] Stuck!

T=3-10s: Continuously stuck
     Thrust weakened to 20%
     left_cmd *= 0.2
     right_cmd *= 0.2
     |
     v
     Boat crawls, appears to not move

User sees: Boat stops after avoidance [X]

Actually: Thrust stuck at 20%, continuously trying to avoid, resulting in very slow speed
```

---

## The Fix

Change one number to solve the problem:

```python
# Before:
'full_clear_distance': 60.0      # [X] Lidar cannot detect 60m

# After:
'full_clear_distance': 20.0      # [OK] Lidar can detect 20m

Reason:
  Lidar maximum range: ~30m
  New threshold: 20m < 30m
  Result: Can properly exit avoidance mode!
```

---

## Complete Control Flow Diagram

```text
┌─────────────────────────────────────┐
│  control_loop() runs every 50ms     │
└──────────────┬──────────────────────┘
               │
        ┌──────▼──────┐
        │ Get lidar   │
        │ data        │
        └──────┬──────┘
               │
        ┌──────▼────────────────────────┐
        │ analyze_lidar()                │
        │ (returns front_min, left_min,  │
        │  right_min)                    │
        └──────┬────────────────────────┘
               │
    ┌──────────▼──────────────┐
    │ Check force_avoid_active │
    └──────────┬──────────────┘
               │
        ┌──────▼──────────────────┐
        │ Is < full_clear_distance? │
        └──────┬──────────────────┘
               │
        ┌──────▼──────────────────┐
        │ Activate/Exit avoidance │
        │ mode                    │
        └──────┬──────────────────┘
               │
        ┌──────▼──────────────────┐
        │ Control thrust based on │
        │ avoidance stage         │
        │ Hard: Turn + Reverse    │
        │ Soft: Slow + Steer      │
        │ Normal: Full + To goal  │
        └──────┬──────────────────┘
               │
        ┌──────▼──────────────────┐
        │ publish_thrust()         │
        │ (Send left/right         │
        │  thruster commands)      │
        └──────────────────────────┘
```

---

## Key Parameter Quick Reference Table

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `forward_thrust` | 400.0 | Base forward thrust |
| `kp_yaw` | 600.0 | Heading control gain |
| `obstacle_slow_dist` | 12.0 | Soft avoidance activation distance |
| `obstacle_stop_dist` | 6.0 | Hard avoidance activation distance |
| `avoid_turn_thrust` | 350.0 | Hard avoidance turn thrust |
| `avoid_diff_gain` | 40.0 | Avoidance steering gain |
| `full_clear_distance` | 20.0 | Global avoidance exit distance |
| `front_angle_deg` | 30.0 | Front scan angle |
| `side_angle_deg` | 60.0 | Side scan angle |
| `cloud_z_min` | -10.0 | Point cloud Z lower limit |
| `cloud_z_max` | 3.0 | Point cloud Z upper limit |
| `min_range_filter` | 3.0 | Minimum distance filter |

---

## Summary

The obstacle avoidance system consists of three parts:

1. **Sensor Data Processing** (analyze_lidar)
   - Extract minimum distances in front/left/right directions from lidar data

2. **Steering Decision** (VFH + Polar Histogram)
   - Select the safest direction or the one closest to the goal

3. **Thrust Control** (control_loop)
   - Adjust left/right thruster thrust based on avoidance stage

The problem was in part 3: `full_clear_distance: 60.0` was too large, causing avoidance mode to get stuck.
Fix: Change to 20.0 to match the actual lidar range.
