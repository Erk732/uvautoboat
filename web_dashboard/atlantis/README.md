# 🚤 Atlantis Dashboard

A clean, ergonomic web interface for the Atlantis Controller and Planner, styled to match the Vostok1 dashboard.

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

## Quick Start

### 1. Start ROS Bridge

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml delay_between_messages:=0.0
```

### 2. Start Atlantis Nodes (Option A: Launch File)

```bash
ros2 launch atlantis.launch.py
```

### 2. Start Atlantis Nodes (Option B: Separate Terminals)

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

## Parameters

### Planner Parameters

| Parameter | Description | Range |
|-----------|-------------|-------|
| Lanes | Number of scanning lanes | 1-20 |
| Scan Length | Length of each lane (m) | 10-500 |
| Lane Width | Spacing between lanes (m) | 5-100 |

### PID Control

| Parameter | Description | Range |
|-----------|-------------|-------|
| Kp | Proportional gain | 0-1000 |
| Ki | Integral gain | 0-100 |
| Kd | Derivative gain | 0-500 |

### Speed Settings

| Parameter | Description | Range |
|-----------|-------------|-------|
| Base Speed | Normal cruising thrust (N) | 100-1000 |
| Max Speed | Maximum thrust limit (N) | 100-1000 |
| Safe Distance | Obstacle avoidance threshold (m) | 5-30 |

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
