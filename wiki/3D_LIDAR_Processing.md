# 3D LIDAR Processing (OKO v2.0)

Deep dive into the OKO perception system — 3D LIDAR point cloud processing for obstacle detection.

---

## Overview

**OKO** (Russian: "eye") is the perception subsystem that processes 3D LIDAR data to detect obstacles in real-time. It uses advanced filtering techniques to provide reliable obstacle information to the navigation system.

---

## What is LIDAR?

**LIDAR** (Light Detection and Ranging) uses laser pulses to create a 3D map of the environment. The WAM-V's 3D LIDAR returns thousands of points per scan, each with X, Y, Z coordinates.

**Key Specifications:**

- **Scan Rate**: ~10-20 Hz
- **Points per Scan**: 10,000-50,000
- **Range**: 0-100m (configurable)
- **Field of View**: 360° horizontal, variable vertical

---

## Processing Pipeline

OKO v2.0 uses an 8-step processing pipeline:

```text
┌─────────────────────────────────────────────────────────┐
│  1. RAW POINT CLOUD                                     │
│     ↓ (Gazebo simulation: ~20K points/scan)            │
├─────────────────────────────────────────────────────────┤
│  2. HEIGHT FILTER (-15m to +10m)                        │
│     ↓ Removes sky and extreme water reflections        │
├─────────────────────────────────────────────────────────┤
│  3. RANGE FILTER (5m to 50m)                            │
│     ↓ Ignores boat structure and distant irrelevant    │
├─────────────────────────────────────────────────────────┤
│  4. WATER PLANE REMOVAL                                 │
│     ↓ Estimates water surface, filters reflections     │
├─────────────────────────────────────────────────────────┤
│  5. SECTOR ANALYSIS (Front/Left/Right)                  │
│     ↓ Calculates minimum distance per sector           │
├─────────────────────────────────────────────────────────┤
│  6. TEMPORAL FILTERING (5-scan history)                 │
│     ↓ Requires 3/5 detections to confirm               │
├─────────────────────────────────────────────────────────┤
│  7. OBSTACLE CLUSTERING (DBSCAN, eps=2m)                │
│     ↓ Groups points into distinct obstacles            │
├─────────────────────────────────────────────────────────┤
│  8. GAP DETECTION & VELOCITY ESTIMATION                 │
│     ↓ Finds passable gaps, tracks moving obstacles     │
└─────────────────────────────────────────────────────────┘
```

---

## Step-by-Step Explanation

### Step 1: Height Filter

**Purpose**: Remove irrelevant points (sky, extreme water reflections)

| Surface | Typical Z | Action |
|:--------|:----------|:-------|
| Sky | > +10m | Filter out |
| Obstacles on dock | 0 to +2m | ✅ Keep |
| Lake bank/terrain | ≈ -2.5m | ✅ Keep |
| Water surface | ≈ -3m | Keep for water plane estimation |
| Extreme reflections | < -15m | Filter out |

**Parameters:**

- `min_height`: -15.0m (default)
- `max_height`: +10.0m (default)

**Rationale**: LIDAR mounted ~2-3m above water, so water appears at negative Z.

---

### Step 2: Range Filter

**Purpose**: Ignore nearby boat structure and distant clutter

| Range | Reason |
|:------|:-------|
| **< 5m** | Boat hull, sensors, dock at spawn point |
| **5-50m** | ✅ Relevant obstacle detection range |
| **> 50m** | Too distant for navigation decisions |

**Parameters:**

- `min_range`: 5.0m (default)
- `max_range`: 50.0m (default)

**Note**: Adjust `min_range` if you see "CRITICAL" warnings at spawn due to nearby dock.

---

### Step 3: Water Plane Removal

**Purpose**: Filter water surface reflections that aren't actual obstacles

**Algorithm:**

1. Calculate **5th percentile** of Z values (low points = likely water)
2. Set water plane Z estimate
3. Remove points within ±0.5m of water plane

**Why 5th percentile?**

- Robust to outliers (not affected by obstacles above water)
- Adapts to varying water surface height in simulation

**Parameter:**

- `water_plane_threshold`: 0.5m (tolerance)

---

### Step 4: Sector Analysis

**Purpose**: Divide 360° view into actionable sectors

#### Sector Definitions

| Sector | Angle Range | Purpose | Width |
|:-------|:------------|:--------|:------|
| **FRONT** | -45° to +45° | Forward collision detection | 90° (adaptive) |
| **LEFT** | +45° to +135° | Left side clearance | 90° |
| **RIGHT** | -135° to -45° | Right side clearance | 90° |

**Adaptive Front Sector:**
The front sector width adjusts based on target heading error:

- Small error → Narrow sector (focused ahead)
- Large error (turning) → Wide sector (check periphery)

#### Minimum Distance Calculation

For each sector, OKO finds the **closest point**:

```python
front_clear = min(distance_to_point for point in front_sector)
left_clear = min(distance_to_point for point in left_sector)
right_clear = min(distance_to_point for point in right_sector)
```

If sector is empty → clearance = `max_range` (50m)

---

### Step 5: Temporal Filtering

**Purpose**: Reduce false positives from noise or transient reflections

**Algorithm:**

- Maintain **5-scan history** (rolling window)
- For each sector, count detections in last 5 scans
- Confirm obstacle only if detected in **≥3 out of 5 scans** (60%)

**Parameters:**

- `temporal_history_size`: 5 (default)
- `temporal_threshold`: 3 (default)

**Benefits:**

- **Reduces flickering**: Transient reflections don't trigger false alarms
- **Maintains responsiveness**: 3/5 threshold allows quick detection (≤500ms)
- **Adapts to scan rate**: Works with varying LIDAR frequencies

**Example:**

```text
Scan 1: Front obstacle at 8m ✓
Scan 2: Front clear (noise)
Scan 3: Front obstacle at 7.5m ✓
Scan 4: Front obstacle at 7m ✓
Scan 5: Front clear (noise)
→ 3/5 detections → CONFIRMED obstacle
```

---

### Step 6: Distance-Weighted Urgency

**Purpose**: Smooth control response instead of binary "obstacle yes/no"

**Urgency Score**: 0.0 (safe) to 1.0 (critical)

```python
if distance > safe_distance:
    urgency = 0.0  # No concern
elif distance < critical_distance:
    urgency = 1.0  # Maximum urgency
else:
    # Linear interpolation
    urgency = 1.0 - (distance - critical_distance) / (safe_distance - critical_distance)
```

| Distance | Urgency | Meaning |
|:---------|:--------|:--------|
| > 15m | 0.0 | ✅ Clear |
| 10m | 0.5 | ⚠️ Caution |
| 5m | 1.0 | 🚨 Critical |

**Benefits:**

- **Smooth thrust reduction**: Gradual slowdown instead of sudden stop
- **Proportional response**: Closer obstacles → stronger reaction
- **Better control**: No oscillation between "go" and "stop"

---

### Step 7: Obstacle Clustering

**Purpose**: Group nearby points into distinct obstacles

**Algorithm**: DBSCAN (Density-Based Spatial Clustering)

**Parameters:**

- `cluster_distance` (eps): 2.0m (max distance between cluster points)
- `min_cluster_size` (min_samples): 3 (min points to form obstacle)

**Output**: List of obstacles with:

- **Centroid**: Average (x, y) position
- **Size**: Number of points
- **Distance**: Range from boat
- **Angle**: Bearing (degrees)

**Example JSON:**

```json
"clusters": [
  {"x": 8.2, "y": 1.5, "size": 25, "distance": 8.5, "angle_deg": 10.3},
  {"x": 12.0, "y": -3.0, "size": 18, "distance": 12.4, "angle_deg": -14.0}
]
```

---

### Step 8: Gap Detection

**Purpose**: Find passable gaps between obstacles for navigation

**Algorithm:**

1. Sort obstacles by angle
2. Calculate angular gap between consecutive obstacles
3. Filter gaps > minimum width (3m default)

**Output**: List of gaps with:

- **Angle**: Direction (degrees)
- **Width**: Gap size (meters)
- **Distance**: Range to gap

**Example JSON:**

```json
"gaps": [
  {"angle_deg": -25.0, "width": 5.2, "distance": 15.0},
  {"angle_deg": 45.0, "width": 8.1, "distance": 20.0}
]
```

**Use Case**: Controller can steer toward gaps when obstacles block direct path.

---

### Step 9: Velocity Estimation (Moving Obstacles)

**Purpose**: Track obstacles across frames to detect movement

**Algorithm:**

1. Match obstacles between consecutive scans (nearest neighbor)
2. Calculate displacement over time
3. Estimate velocity vector (vx, vy)

**Parameters:**

- `velocity_history_size`: 5 frames (default)

**Output**: Moving obstacles with velocity:

```json
"moving_obstacles": [
  {"id": "obs_0", "vx": 0.5, "vy": 0.1, "speed": 0.51}
]
```

**Future Use**: Predictive avoidance for dynamic environments (other boats, wildlife).

---

## Output Message Format

OKO publishes obstacle information to `/perception/obstacle_info` as JSON:

```json
{
  "obstacle_detected": true,
  "min_distance": 8.5,
  "front_clear": 10.2,
  "left_clear": 45.0,
  "right_clear": 12.3,
  "is_critical": false,
  "front_urgency": 0.45,
  "left_urgency": 0.0,
  "right_urgency": 0.35,
  "overall_urgency": 0.45,
  "clusters": [
    {"x": 8.2, "y": 1.5, "size": 25, "distance": 8.5, "angle_deg": 10.3}
  ],
  "gaps": [
    {"angle_deg": -25.0, "width": 5.2, "distance": 15.0}
  ],
  "moving_obstacles": [
    {"id": "obs_0", "vx": 0.5, "vy": 0.1, "speed": 0.51}
  ],
  "water_plane_z": -2.8,
  "temporal_confidence": 1.0
}
```

---

## Configuration Parameters

### In `vostok1.launch.yaml` (OKO node)

| Parameter | Default | Description |
|:----------|:--------|:------------|
| `min_safe_distance` | 12.0 | Safe clearance distance (m) |
| `critical_distance` | 4.0 | Critical stop distance (m) |
| `min_height` | -15.0 | Minimum Z to keep (m) |
| `max_height` | 10.0 | Maximum Z to keep (m) |
| `min_range` | 5.0 | Minimum detection range (m) |
| `max_range` | 50.0 | Maximum detection range (m) |
| `temporal_history_size` | 5 | Scans in history |
| `temporal_threshold` | 3 | Min detections to confirm |
| `cluster_distance` | 2.0 | DBSCAN eps (m) |
| `min_cluster_size` | 3 | DBSCAN min samples |
| `water_plane_threshold` | 0.5 | Water filtering tolerance (m) |
| `velocity_history_size` | 5 | Frames for velocity tracking |

---

## Tuning Guide

### Problem: Too Many False Alarms

**Solution**: Increase temporal filtering

```yaml
temporal_history_size: 7
temporal_threshold: 5  # Require 5/7 detections
```

### Problem: Slow Reaction to Obstacles

**Solution**: Reduce temporal filtering

```yaml
temporal_history_size: 3
temporal_threshold: 2  # Require 2/3 detections
```

### Problem: Water Reflections Detected as Obstacles

**Solution**: Adjust water plane removal

```yaml
water_plane_threshold: 0.8  # More aggressive filtering
```

### Problem: Missing Small Obstacles

**Solution**: Reduce cluster size requirement

```yaml
min_cluster_size: 2  # Detect obstacles with only 2 points
```

### Problem: "CRITICAL" Warning at Spawn

**Solution**: Increase minimum range

```yaml
min_range: 7.0  # Ignore nearby dock
```

---

## Monitoring OKO

### Check LIDAR Data

```bash
# Check scan rate
ros2 topic hz /wamv/sensors/lidars/lidar_wamv/points

# View raw point cloud (first scan)
ros2 topic echo /wamv/sensors/lidars/lidar_wamv/points --once
```

### Monitor Obstacle Info

```bash
ros2 topic echo /perception/obstacle_info
```

### Visualize in RViz

```bash
rviz2
# Add PointCloud2 display
# Topic: /wamv/sensors/lidars/lidar_wamv/points
# Fixed Frame: wamv/base_link
```

---

## Performance Characteristics

| Metric | Value |
|:-------|:------|
| **Processing Latency** | ~10-50ms per scan |
| **Detection Range** | 5-50m |
| **Update Rate** | 10-20 Hz |
| **False Positive Rate** | < 5% (with temporal filtering) |
| **False Negative Rate** | < 2% (for obstacles > 0.5m diameter) |

---

## Related Pages

- **[Obstacle Avoidance Loop](Obstacle-Avoidance-Loop)** — How OKO integrates with control
- **[System Overview](System_Overview)** — High-level architecture
- **[Configuration & Tuning](Configuration-and-Tuning)** — Parameter reference
- **[Modular Architecture](Modular-Architecture)** — OKO's role in the system
