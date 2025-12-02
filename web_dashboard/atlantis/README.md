# 🚤 Atlantis Dashboard

A clean, ergonomic web interface for the Atlantis Controller and Planner.

## Features

- **Real-time Status Display**: Mission state, waypoint progress, GPS position
- **Obstacle Detection Visualization**: 3-sector display (Left/Front/Right) with distance
- **Anti-Stuck System Monitoring**: SASS status, escape attempts, no-go zones
- **Live Thruster Output**: Visual bars showing left/right thrust
- **Interactive Map**: Leaflet-based with boat tracking, waypoint markers, trajectory
- **Full Parameter Control**: All ROS 2 parameters adjustable via sliders and inputs

## Parameters Available

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
| Waypoint Tolerance | Arrival threshold (m) | 1-10 |

### Obstacle Avoidance
| Parameter | Description | Range |
|-----------|-------------|-------|
| Safe Distance | Start slowing threshold (m) | 5-30 |
| Critical Distance | Emergency reverse threshold (m) | 2-15 |
| Slow Factor | Speed reduction near obstacles | 0.1-1.0 |

### Stuck Detection
| Parameter | Description | Range |
|-----------|-------------|-------|
| Stuck Timeout | Time before stuck detection (s) | 2-15 |
| Movement Threshold | Min distance to not be stuck (m) | 0.5-5 |
| No-Go Zone Radius | Area to avoid after stuck (m) | 3-20 |
| Detour Distance | Alternative waypoint offset (m) | 5-30 |

## Usage

### 1. Start ROS Bridge
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### 2. Start Atlantis Nodes
```bash
# Terminal 1 - Planner
ros2 run plan atlantis_planner

# Terminal 2 - Controller  
ros2 run control atlantis_controller
```

### 3. Open Dashboard
Open `index.html` in a web browser, or serve it:
```bash
cd web_dashboard/atlantis
python3 -m http.server 8080
# Open http://localhost:8080
```

### 4. Connect
Click the "Connect" button to establish connection to ROS bridge.

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

### Published
| Topic | Type | Description |
|-------|------|-------------|
| `/atlantis/replan` | Empty | Trigger path replanning |

## Design Philosophy

- **Clean & Modern**: Purple gradient theme with clear typography
- **Ergonomic**: Parameters grouped logically with sliders + number inputs
- **Responsive**: Works on different screen sizes
- **User-Friendly**: Hints on every parameter, visual feedback on changes

## Files

```
atlantis/
├── index.html   # Main HTML structure
├── style.css    # Clean modern CSS
├── app.js       # ROS 2 integration & UI logic
└── README.md    # This file
```

---
**Part of the AutoBoat Navigation System** | IMT Nord Europe
