# 🚤 Atlantis Dashboard

A clean, ergonomic web interface for the Atlantis Controller and Planner, styled to match the Vostok1 dashboard.

## Prerequisites

### ROS2 Rosbridge Suite (Required for WebSocket on port 9090)

If you're having trouble connecting to port 9090, install the rosbridge suite:

```bash
# For ROS2 Jazzy (Ubuntu 24.04)
sudo apt update
sudo apt install ros-jazzy-rosbridge-suite

# For ROS2 Humble (Ubuntu 22.04)
sudo apt install ros-humble-rosbridge-suite
```

After installation, source your ROS2 environment:

```bash
source /opt/ros/jazzy/setup.bash  # or humble
```

### Troubleshooting Port 9090

If the dashboard shows "Disconnected" or can't connect:

1. **Check if rosbridge is running:**

   ```bash
   ros2 node list | grep rosbridge
   ```

2. **Check if port 9090 is listening:**

   ```bash
   ss -tuln | grep 9090
   ```

3. **Check for firewall issues:**

   ```bash
   sudo ufw allow 9090
   ```

4. **Restart rosbridge:**

   ```bash
   ros2 launch rosbridge_server rosbridge_websocket_launch.xml delay_between_messages:=0.0
   ```

## Features

- **Real-time Status Display**: Mission state, waypoint progress, GPS position
- **Obstacle Detection Visualization**: 3-sector display (Left/Front/Right) with distance
- **Anti-Stuck System Monitoring**: SASS status, escape attempts, no-go zones
- **Live Thruster Output**: Visual bars showing left/right thrust
- **Interactive Map**:
  - Leaflet-based with boat tracking
  - **Follow Boat toggle** (🎯/🔓) - auto-center on boat or free pan
  - Waypoint markers with color coding (purple=pending, orange=current, green=passed)
  - Waypoint tooltips showing coordinates and status
  - Trajectory line showing actual path traveled
- **Terminal Output**: Real-time ROS2 node logs from atlantis nodes
- **Full Parameter Control**: All ROS 2 parameters adjustable via inputs
- **Theme Switcher**: Toggle between Modern and 1980s Soviet/TNO style

## 🎨 Theme Switcher - 1980s Soviet Mode

The dashboard includes a unique **1980s Soviet/TNO theme** that transforms the interface into an authentic Cold War-era Soviet computer terminal aesthetic.

### Features of Soviet Theme

- **CRT Phosphor Green**: Authentic green phosphor display colors
- **Scanline Effect**: Simulated CRT scanlines overlay
- **Screen Glow**: Subtle CRT screen glow effect
- **VT323 Font**: Retro terminal-style monospace typography
- **Cyrillic Labels**: Russian text for header elements ("СИСТЕМА", "СССР")
- **Amber Warnings**: 1980s-style amber warning colors
- **Angular UI**: Sharp, utilitarian Soviet design aesthetic

### How to Use

1. Click the **"☭ USSR 1980"** button in the header to activate Soviet theme
2. Click **"🌐 MODERN"** to switch back to the modern gradient style
3. Theme preference is saved in localStorage and persists across sessions

### Visual Style Comparison

| Element | Modern Theme | Soviet Theme |
|---------|--------------|--------------|
| Background | Purple-blue gradient | Pure black |
| Accent Color | Purple (#667eea) | Phosphor green (#00ff00) |
| Font | Roboto Condensed | VT323 (CRT terminal) |
| Panels | White, rounded | Dark green, angular |
| Map | Normal colors | Green-tinted (hue-rotated) |
| Effects | Smooth shadows | Scanlines, glow |

## Quick Start

### 1. Start ROS Bridge

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml delay_between_messages:=0.0
```

> **Important:** The `delay_between_messages:=0.0` parameter is required for ROS 2 Jazzy due to a parameter type bug.

This starts a WebSocket server on `ws://localhost:9090`.

### 2. Start Atlantis Nodes

#### Option A: YAML Launch File (Recommended)

```bash
# From the workspace
ros2 launch ~/seal_ws/src/uvautoboat/launch/atlantis.launch.yaml

# Or from the launch directory
ros2 launch launch/atlantis.launch.yaml
```

> **Note:** To change parameters, edit the `atlantis.launch.yaml` file directly or use the web dashboard controls.

#### Option B: Python Launch File

```bash
ros2 launch launch/atlantis.launch.py
```

#### Option C: Separate Terminals (Manual)

```bash
# Terminal 1 - Planner
ros2 run plan atlantis_planner

# Terminal 2 - Controller  
ros2 run control atlantis_controller
```

### 3. Start Web Server

```bash
cd /path/to/web_dashboard/atlantis
python3 -m http.server 8080
```

### 4. Open Dashboard

Open browser to: **<http://localhost:8080>**

### 5. Connect & Start

1. Click **"Connect"** to connect to ROS bridge
2. Click **"Start"** to enable mission and generate path
3. Use **"Replan"** to regenerate waypoints with new parameters

## YAML Launch File Parameters

The `atlantis.launch.yaml` file supports all parameters via command-line overrides.

### Planner Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `scan_length` | 150.0 | Length of each scanning lane (m) |
| `scan_width` | 20.0 | Spacing between lanes (m) |
| `lanes` | 4 | Number of scanning lanes |
| `frame_id` | map | Coordinate frame for path |

### PID Control Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `kp` | 400.0 | Proportional gain |
| `ki` | 20.0 | Integral gain |
| `kd` | 100.0 | Derivative gain |
| `base_speed` | 500.0 | Normal cruising thrust (N) |
| `max_speed` | 800.0 | Maximum thrust limit (N) |
| `waypoint_tolerance` | 2.0 | Distance to consider waypoint reached (m) |

### Obstacle Avoidance Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `min_safe_distance` | 8.0 | Start slowing down distance (m) |
| `critical_distance` | 2.0 | Emergency stop distance (m) |
| `obstacle_slow_factor` | 0.1 | Speed reduction factor near obstacles |
| `hysteresis_distance` | 1.0 | Hysteresis to prevent mode oscillation (m) |
| `reverse_timeout` | 10.0 | Max time to reverse when stuck (s) |

### Stuck Recovery (SASS) Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `stuck_timeout` | 5.0 | Time before declaring stuck (s) |
| `stuck_threshold` | 1.0 | Movement threshold for stuck detection (m) |
| `no_go_zone_radius` | 8.0 | Radius of no-go zones around failed positions (m) |
| `drift_compensation_gain` | 0.3 | Gain for drift compensation |
| `probe_angle` | 45.0 | Angle for directional probing (degrees) |
| `detour_distance` | 12.0 | Distance for detour maneuvers (m) |

## Dashboard Parameters

Parameters can also be adjusted via the web dashboard inputs:

## ROS 2 Topics

### Subscribed

| Topic | Type | Description |
|-------|------|-------------|
| `/atlantis/mission_status` | String (JSON) | Mission state, waypoint info |
| `/atlantis/obstacle_status` | String (JSON) | Obstacle detection status |
| `/atlantis/anti_stuck_status` | String (JSON) | SASS status |
| `/wamv/sensors/gps/gps/fix` | NavSatFix | GPS position |
| `/wamv/thrusters/left/thrust` | Float64 | Left thruster output |
| `/wamv/thrusters/right/thrust` | Float64 | Right thruster output |
| `/atlantis/path` | Path | Planned waypoints |
| `/rosout` | Log | ROS2 node logs (filtered for atlantis) |

### Published

| Topic | Type | Description |
|-------|------|-------------|
| `/atlantis/replan` | Empty | Trigger path replanning |
| `/atlantis/start` | Empty | Enable mission |
| `/atlantis/stop` | Empty | Disable mission |

## Map Controls

| Control | Description |
|---------|-------------|
| 🎯 (top-right) | Follow boat mode - map auto-centers on boat |
| 🔓 (top-right) | Free mode - pan/zoom freely |
| Scroll | Zoom in/out |
| Drag | Pan map (in free mode) |
| Hover waypoint | Show waypoint details tooltip |

## Files

```bash
atlantis/
├── index.html   # Main HTML structure
├── style.css    # Vostok1-style CSS
├── app.js       # ROS 2 integration & UI logic
└── README.md    # This file
```

---
**Part of the AutoBoat Navigation System** | IMT Nord Europe
