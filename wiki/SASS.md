# Smart Anti-Stuck System (SASS)

Intelligent recovery system that frees the boat when it becomes trapped or immobilized by obstacles, currents, or navigation errors.

---

## Overview

The **Smart Anti-Stuck System (SASS)** is an advanced recovery mechanism implemented in both Vostok1 and Modular (BURAN) architectures. When the boat detects it's stuck (minimal movement despite thrust), SASS executes a multi-phase escape maneuver while learning from the experience.

---

## Key Features

| Feature | Description |
|:--------|:------------|
| **Adaptive Escape** | 10-20s duration based on severity |
| **Multi-Direction Probe** | Scans L/R/Back before choosing escape direction |
| **No-Go Zones** | Remembers stuck locations (max 20, 8m radius) |
| **Kalman Drift Compensation** | Estimates current/wind with uncertainty |
| **Detour Insertion** | Auto-adds waypoints around obstacles |
| **Learning Memory** | Records successful escapes for future reference |

---

## When SASS Activates

SASS triggers when the boat is **stuck**:

| Condition | Value |
|:----------|:------|
| **Time without progress** | > 3.0 seconds (default `stuck_timeout`) |
| **Movement threshold** | < 0.5 meters (default `stuck_threshold`) |
| **Thrust applied** | Yes (not idle) |

**Example Log:**
```text
🚨 BLOQUÉ! | STUCK! - No progress for 3.5s
🔧 SASS PHASE 0: PROBE - Scanning best escape direction...
```

---

## Escape Sequence (4 Phases)

SASS executes a carefully designed sequence:

### Phase 0: PROBE (0-2s)
**Multi-direction scanning to find best escape route**

| Direction | Check |
|:----------|:------|
| **LEFT** | Front-left clearance |
| **RIGHT** | Front-right clearance |
| **BACK** | Rear clearance |

The system chooses the direction with **maximum clearance**.

```text
🔧 SASS PHASE 0: PROBE
   LEFT: 12.5m | RIGHT: 8.3m | BACK: 15.0m
   → Best direction: BACK
```

### Phase 1: REVERSE (2s-~6s)
**Back away from obstacle**

- **Thrust**: Both thrusters in reverse
- **Power**: -400 to -500 N (configurable)
- **Duration**: ~4 seconds

```text
🔧 SASS PHASE 1: REVERSE - Backing away...
```

### Phase 2: TURN (~6s-~10s)
**Rotate toward best escape direction**

- **Left turn**: Left=-500, Right=+500
- **Right turn**: Left=+500, Right=-500
- **Duration**: ~4 seconds

```text
🔧 SASS PHASE 2: TURN - Rotating toward clearest path...
```

### Phase 3: FORWARD (~10s-~12s)
**Test forward movement with drift compensation**

- **Thrust**: Forward with Kalman-filtered drift correction
- **Monitoring**: Checks if movement resumes
- **Duration**: ~2 seconds

```text
🔧 SASS PHASE 3: FORWARD - Testing escape...
✅ SASS SUCCESS: Escaped! Adding no-go zone at (23.4, 12.1)
```

---

## Adaptive Duration Calculation

SASS adjusts escape duration based on **severity**:

```text
Base Duration: 10 seconds

+ 4s if obstacle < critical_distance (5m)
+ 2s if obstacle < safe_distance (15m)
+ 2s per consecutive stuck attempt

Maximum: 20 seconds
```

**Example:**
- **First stuck attempt**, obstacle at 8m → 10s + 2s = **12s total**
- **Second attempt**, obstacle at 4m → 10s + 4s + 2s + 2s = **18s total**
- **Third attempt**, obstacle at 3m → **20s** (capped at maximum)

---

## No-Go Zones

After each successful escape, SASS **remembers** the stuck location:

| Parameter | Default | Description |
|:----------|:--------|:------------|
| **Radius** | 8.0m | Avoidance radius |
| **Maximum zones** | 20 | Memory limit (oldest removed first) |
| **Persistence** | Mission duration | Cleared on reset |

**Visualization in Dashboard:**
- 🔴 Red circles on trajectory map
- Boat avoids returning to these areas

**Log Output:**
```text
✅ SASS SUCCESS: Adding no-go zone #3 at (45.2, -12.8) [radius: 8.0m]
```

---

## Kalman Filter Drift Compensation

SASS uses a **2D Kalman filter** to estimate environmental drift (currents, wind):

### State Estimation

```python
x = [drift_x, drift_y]    # State estimate (m/s)
P = uncertainty           # Covariance matrix (confidence)
Q = 0.001                 # Process noise (drift changes slowly)
R = 0.1                   # Measurement noise (GPS/IMU error)
```

### Update Cycle

1. **Predict**: Uncertainty grows (`P = P + Q`)
2. **Measure**: Compare expected vs actual movement
3. **Update**: Correct estimate with measurement
   - `K = P / (P + R)` (Kalman gain)
   - `x = x + K(z - x)` (new estimate)
   - `P = (1 - K) * P` (reduced uncertainty)

### Dashboard Display

**Uncertainty Colors:**
- 🟢 **< 0.05** — High confidence
- 🟡 **0.05 - 0.15** — Moderate confidence
- 🔴 **> 0.15** — Low confidence (more measurements needed)

**Example:**
```text
Drift Estimate: vx=0.12 m/s, vy=-0.05 m/s
Uncertainty: 0.03 (🟢 confident)
```

---

## Detour Insertion

When SASS fails **3 times** at the same waypoint, it requests a **detour**:

### Detour Request

```json
{
  "type": "stuck_detour",
  "x": 45.2,
  "y": -12.8
}
```

The planner (SPUTNIK) calculates a detour waypoint:
- **Distance**: 12m from current position (configurable)
- **Direction**: Perpendicular to obstacle (90° from blocked direction)

### Detour Behavior

| Attempt | Action |
|:--------|:-------|
| **1st stuck** | Execute SASS Phase 0-3 |
| **2nd stuck** | Extended SASS + add no-go zone |
| **3rd stuck** | Request detour waypoint |
| **4th+ stuck** | Skip waypoint (normal mode) or insert another detour (go home mode) |

---

## Configuration Parameters

### In `vostok1.launch.yaml` (BURAN node)

| Parameter | Default | Description |
|:----------|:--------|:------------|
| `stuck_timeout` | 3.0 | Seconds without progress to trigger SASS |
| `stuck_threshold` | 0.5 | Minimum movement (meters) to not be stuck |
| `no_go_zone_radius` | 8.0 | Avoidance radius around stuck locations |
| `detour_distance` | 12.0 | Distance for detour waypoints |
| `critical_distance` | 5.0 | Obstacle distance for severity calculation |

### Runtime Tuning (via CLI)

```bash
# Example: Increase stuck detection sensitivity
ros2 param set /buran_controller stuck_timeout 2.0
ros2 param set /buran_controller stuck_threshold 0.3
```

---

## SASS vs Waypoint Skip

Two complementary strategies for handling blocked waypoints:

| Strategy | Trigger | Action |
|:---------|:--------|:-------|
| **SASS** | Boat physically stuck (no movement) | Multi-phase escape maneuver |
| **Waypoint Skip** | Obstacle blocking for 45s | Skip to next waypoint |

**Typical Flow:**
1. Boat approaches waypoint
2. Obstacle detected → slow down and navigate around
3. If stuck → SASS activates (Phase 0-3)
4. If still can't reach after 45s → Skip waypoint
5. Continue to next waypoint

---

## Real-World Example

**Scenario**: Boat gets wedged between two buoys

```text
T=0s:   Boat stuck between buoys
        → SASS activates

T=0-2s: PHASE 0 (PROBE)
        LEFT: 6m | RIGHT: 5m | BACK: 12m
        → Choose BACK

T=2-6s: PHASE 1 (REVERSE)
        Backing away from buoys...

T=6-10s: PHASE 2 (TURN)
         Rotating toward open water...

T=10-12s: PHASE 3 (FORWARD)
          Testing forward movement...
          → Success! Movement resumed

T=12s:  SASS SUCCESS
        → Add no-go zone at (25.3, 15.7)
        → Resume normal navigation
```

---

## Monitoring SASS

### Dashboard Panel

The dashboard shows:
- **Current phase** (Probe/Reverse/Turn/Forward)
- **No-go zones** (red circles on map)
- **Drift vector** (arrow showing current/wind)
- **Kalman uncertainty** (color-coded confidence)

### Terminal Output

```text
🚨 BLOQUÉ! | STUCK! - No progress for 3.2s (moved only 0.3m)
🔧 SASS PHASE 0: PROBE - Scanning escape directions...
🔧 SASS PHASE 1: REVERSE - Backing away from obstacle
🔧 SASS PHASE 2: TURN - Rotating RIGHT (clearer side)
🔧 SASS PHASE 3: FORWARD - Testing escape with drift compensation
✅ SASS SUCCESS: Escaped! No-go zone added at (X, Y)
```

### ROS 2 Topic

```bash
ros2 topic echo /vostok1/anti_stuck_status
# OR for modular:
ros2 topic echo /control/anti_stuck_status
```

**JSON Format:**
```json
{
  "active": true,
  "phase": 2,
  "phase_name": "TURN",
  "elapsed": 7.5,
  "no_go_zones": [
    {"x": 23.4, "y": 12.1, "radius": 8.0},
    {"x": 45.2, "y": -8.3, "radius": 8.0}
  ],
  "drift_estimate": {"vx": 0.12, "vy": -0.05},
  "drift_uncertainty": 0.03
}
```

---

## Troubleshooting

### SASS Activates Too Often

**Cause**: Stuck detection too sensitive

**Solution**: Increase timeout or threshold
```bash
ros2 param set /buran_controller stuck_timeout 5.0
ros2 param set /buran_controller stuck_threshold 1.0
```

### SASS Doesn't Escape

**Cause**: Escape duration too short for complex obstacles

**Solution**: Severity calculation should automatically extend duration, but you can manually increase:
- Check if `critical_distance` is appropriate for your obstacles
- Verify drift compensation is working (check Kalman uncertainty)

### Too Many No-Go Zones

**Cause**: Boat getting stuck repeatedly in obstacle-dense areas

**Solution**:
- Enable A* path planning to avoid obstacle fields
- Reduce `no_go_zone_radius` if zones overlap too much
- Use waypoint skip strategy to move past difficult areas

---

## Related Pages

- **[Obstacle Avoidance Loop](Obstacle-Avoidance-Loop)** — How continuous obstacle detection works
- **[Waypoint Skip Strategy](Waypoint-Skip-Strategy)** — Complementary recovery mechanism
- **[Kalman Filtering](Kalman-Filtering)** — State estimation theory
- **[Configuration & Tuning](Configuration-and-Tuning)** — Parameter reference
