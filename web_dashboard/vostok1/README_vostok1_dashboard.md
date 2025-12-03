# Vostok1 Web Dashboard

A real-time web-based monitoring dashboard for the Vostok1 autonomous boat system.

## Features

- **Real-time GPS tracking** with trajectory visualization on interactive map
- **Obstacle detection monitoring** showing minimum distances and clearance zones
- **Enhanced OKO v2.0 perception data** with urgency scores, clusters, and gaps
- **Thruster output visualization** with live thrust bars
- **Mission status display** including state, waypoint, and distance
- **SASS v2.0 Anti-Stuck Status** with visual escalation indicators
- **System logs** with color-coded severity levels
- **Dual-mode support**: Works with both integrated Vostok1 and modular navigation
- **4 Style modes**: Normal, Bureau (TNO), Terminal (CRT), **MilSpec (Warsaw Pact)**
- **Configuration panel**: Adjust mission parameters in real-time
- **Mission control buttons**: Generate, Confirm, Start, Stop, Resume, Go Home
- **Responsive design** works on desktop and mobile devices

## 🎨 Visual Themes

The dashboard includes 4 distinct visual themes, cycled via the toggle button:

### 1. Normal Mode (Default)

Modern gradient design with purple/blue accents, clean white panels, Roboto font.

### 2. Bureau Mode (TNO Soviet)

The New Order-inspired Cold War aesthetic with CRT effects, scanlines, socialist longtermism emblem, industrial Soviet bureaucracy colors.

### 3. Terminal Mode (CRT)

Classic green-phosphor computer terminal aesthetic, heavy scanlines, monospace fonts, retro command-line feel.

### 4. MilSpec Mode (ВМФ СССР / Warsaw Pact)

**New!** Authentic 1980s Soviet Navy military specification styling:

- **Phosphor green** primary display color
- **Soviet Red & Navy Gold** accent colors
- **☭ Hammer & Sickle** emblem in header
- **ВМФ СССР** (Soviet Navy) designation badge
- **Military grid overlay** for tactical feel
- **MIL-STD-1553** inspired data presentation
- **Russian labels** for mode switching (БЮРО, ТЕРМИНАЛ, ВМФ СССР, ОБЫЧНЫЙ)
- **Alert animations** for critical status indicators
- **Radar sweep** effect on Soviet emblem

### Theme Cycling Order

```bash
Normal → БЮРО TNO → ТЕРМИНАЛ → ВМФ СССР → Normal (repeat)
```

Click the theme toggle button in the header to cycle through modes. Each mode has:

- Unique color palette
- Custom animations and effects
- Styled panels, buttons, and inputs
- Map filter adjustments
- Custom scrollbar styling

## Prerequisites

1. **ROS 2 Jazzy** installed and configured
2. **Rosbridge Suite** for WebSocket communication:

   ```bash
   # For ROS2 Jazzy (Ubuntu 24.04)
   sudo apt update
   sudo apt install ros-jazzy-rosbridge-suite

   # For ROS2 Humble (Ubuntu 22.04)
   sudo apt install ros-humble-rosbridge-suite
   ```

3. **Vostok1 node** running (`vostok1.py`)

### Troubleshooting Port 9090

If the dashboard shows "Déconnecté" or can't connect to rosbridge:

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

## Usage

### 1. Rebuild Workspace (if code changed)

```bash
cd ~/seal_ws
colcon build --packages-select plan
source install/setup.bash
```

### 2. Launch VRX Gazebo Simulation

**Terminal 1:**

```bash
cd ~/seal_ws
source install/setup.bash
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta_custom
```

Wait for Gazebo to fully load with the WAM-V boat spawned.

### 3. Start Rosbridge Server

**Terminal 2:**

```bash
cd ~/seal_ws
source install/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml delay_between_messages:=0.0
```

> **Important:** The `delay_between_messages:=0.0` parameter is required for ROS 2 Jazzy due to a parameter type bug.

This starts a WebSocket server on `ws://localhost:9090`.

### 4. Launch Navigation System

**Terminal 3 — Choose ONE option:**

#### Option A: Modular Navigation (Recommended)

Uses OKO + SPUTNIK + BURAN distributed architecture with full configurability:

```bash
cd ~/seal_ws
source install/setup.bash
ros2 launch ~/seal_ws/src/uvautoboat/launch/vostok1.launch.yaml
```

> **Note:** The YAML launch file includes all OKO v2.0 enhanced perception parameters (temporal filtering, clustering, gap detection, velocity estimation).

#### Option B: Integrated Vostok1

Single-node implementation with all features built-in:

```bash
cd ~/seal_ws
source install/setup.bash
ros2 run plan vostok1
```

#### Option C: Python Launch File (Command-line args)

For custom PID tuning via command-line:

```bash
ros2 launch plan vostok1_modular_navigation.launch.py kp:=500.0 ki:=30.0 kd:=150.0
```

### 5. Open Dashboard

**Terminal 4:**

```bash
cd ~/seal_ws/src/uvautoboat/web_dashboard/vostok1
python3 -m http.server 8000
```

Then open your web browser and navigate to:

```text
http://localhost:8000
```

**Important:** Do NOT open `index.html` directly as a file (`file://`). You must serve it via HTTP server for WebSocket connections to work.

The dashboard will automatically connect to rosbridge on port 9090 and display real-time data.

## Dashboard Panels

### Connection Status

- **Green (Connected)**: Successfully connected to rosbridge
- **Red (Disconnected)**: Connection lost, will auto-retry every 5 seconds

### Mission Status

- Current state (INIT, MOVING_TO_WAYPOINT, STUCK_ESCAPING, MISSION_COMPLETE)
- Active waypoint number
- Distance to current waypoint

### GPS Position

- Latitude/Longitude in decimal degrees
- Local X/Y coordinates relative to mission origin
- Real-time position updates

### Obstacle Detection

- Minimum obstacle distance from LIDAR
- Front/Left/Right clearance indicators
- Color-coded status: Clear (>15m), Warning (5-15m), Critical (<5m)
- **OKO v2.0 Enhanced Data** (via `/perception/obstacle_info`):
  - Urgency scores (0.0-1.0) for smoother control
  - Obstacle clusters with centroids and sizes
  - Passable gaps between obstacles
  - Moving obstacle velocity tracking
  - Temporal confidence indicator

### Thruster Output

- Left and right thruster values (-1000 to 1000)
- Visual thrust bars with color coding:
  - Purple gradient: Forward thrust
  - Red gradient: Reverse thrust

### Boat Trajectory

- Interactive OpenStreetMap with boat position marker
- Blue trajectory line showing last 100 GPS points
- Auto-panning to keep boat in view

### System Logs

- Timestamped log entries
- Color-coded by severity (info/warning/error)
- Auto-scrolling with last 50 entries kept

## ROS Topics Monitored

### Core Topics (Always Active)

| Topic | Message Type | Data |
|-------|-------------|------|
| `/wamv/sensors/gps/gps/fix` | `sensor_msgs/NavSatFix` | GPS latitude/longitude |
| `/wamv/thrusters/left/thrust` | `std_msgs/Float64` | Left thruster command |
| `/wamv/thrusters/right/thrust` | `std_msgs/Float64` | Right thruster command |
| `/rosout` | `rcl_interfaces/Log` | System logs (filtered) |

### Integrated Mode Topics (Vostok1)

| Topic | Message Type | Data |
|-------|-------------|------|
| `/vostok1/mission_status` | `std_msgs/String` | Mission state, waypoint, distance |
| `/vostok1/obstacle_status` | `std_msgs/String` | Obstacle distances and clearance |
| `/vostok1/anti_stuck_status` | `std_msgs/String` | SASS v2.0 escalation status |
| `/vostok1/config` | `std_msgs/String` | Current configuration values |
| `/vostok1/set_config` | `std_msgs/String` | Configuration updates (publish) |

### Modular Mode Topics (Sputnik/Oko/Buran)

| Topic | Message Type | Data |
|-------|-------------|------|
| `/planning/mission_status` | `std_msgs/String` | Sputnik planner status |
| `/perception/obstacle_info` | `std_msgs/String` | OKO v2.0 enhanced perception (see below) |
| `/control/status` | `std_msgs/String` | Buran controller status |
| `/control/anti_stuck_status` | `std_msgs/String` | Buran anti-stuck status |
| `/sputnik/config` | `std_msgs/String` | Sputnik configuration |
| `/sputnik/set_config` | `std_msgs/String` | Configuration updates (publish) |
| `/sputnik/mission_command` | `std_msgs/String` | Mission commands (publish) |

### OKO v2.0 Enhanced Perception Data

The `/perception/obstacle_info` topic now includes enhanced fields:

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
  "clusters": [{"x": 8.2, "y": 1.5, "size": 25, "distance": 8.5, "angle_deg": 10.3}],
  "gaps": [{"angle_deg": -25.0, "width": 5.2, "distance": 15.0}],
  "moving_obstacles": [{"id": "obs_0", "vx": 0.5, "vy": 0.1, "speed": 0.51}],
  "water_plane_z": -2.8,
  "temporal_confidence": 1.0
}
```

| Field | Description |
|-------|-------------|
| `*_urgency` | Distance-weighted score (0.0=safe, 1.0=critical) |
| `clusters` | Grouped obstacles with centroids and sizes |
| `gaps` | Passable gaps between obstacles (>3m width) |
| `moving_obstacles` | Tracked obstacles with velocity estimates |
| `water_plane_z` | Estimated water surface height |
| `temporal_confidence` | Detection confidence (scans received / history size) |

## Customization

### Adding Custom Topics

To subscribe to additional vostok1 topics, edit `app.js`:

```javascript
// Example: Subscribe to custom obstacle topic
const obstacleTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/vostok1/obstacles',
    messageType: 'your_custom_msg/ObstacleData'
});

obstacleTopic.subscribe((message) => {
    // Update obstacle data
    currentState.obstacles.min = message.min_distance;
    currentState.obstacles.front = message.front_clear;
});
```

### Changing Map Center

Edit the initial map center in `app.js`:

```javascript
map = L.map('map').setView([YOUR_LAT, YOUR_LON], 16);
```

### Adjusting Update Rates

Modify the update interval (default 1 second):

```javascript
setInterval(() => {
    // Update code
}, 1000); // milliseconds
```

## Troubleshooting

### Dashboard shows "Disconnected"

- Verify rosbridge is running:

  ```bash
  ros2 launch rosbridge_server rosbridge_websocket_launch.xml delay_between_messages:=0.0
  ```

- Check rosbridge is on port 9090: `ss -tlnp | grep 9090`
- If port 9090 is in use, kill previous instance: `pkill -9 -f rosbridge`
- Ensure you're accessing the dashboard via `http://localhost:8000`, NOT opening the file directly
- Open browser console (F12) to see WebSocket errors

### Libraries not loading (ROSLIB is not defined)

- Ensure you have internet connection for CDN libraries (roslibjs, Leaflet)
- Check browser console for failed resource loads
- The dashboard uses CDN for roslib.js and Leaflet.js - these require internet access

### No GPS data updating

- Confirm vostok1 node is publishing: `ros2 topic echo /wamv/sensors/gps/gps/fix`
- Check topic names match between vostok1 and dashboard

### Map not loading

- Requires internet connection for OpenStreetMap tiles
- Check browser console for tile loading errors

### Thruster bars not moving

- Verify thruster topics are publishing: `ros2 topic list | grep thrust`
- Ensure vostok1 is actively controlling thrusters

## Architecture

```text
┌─────────────────────────────────────────────────────────────────────┐
│                     Navigation Nodes                                │
├─────────────────────────────┬───────────────────────────────────────┤
│   Integrated Mode           │   Modular Mode (Recommended)          │
│   ┌─────────────────┐       │   ┌─────────┐ ┌───────┐ ┌───────┐     │
│   │   Vostok1 Node  │       │   │ Sputnik │ │  OKO  │ │ Buran │     │
│   │   (All-in-one)  │       │   │ Planner │ │ v2.0  │ │ Ctrl  │     │
│   └────────┬────────┘       │   └────┬────┘ └───┬───┘ └───┬───┘     │
│            │                │        │          │         │         │
│   /vostok1/* topics         │   /planning/*  /perception/* /control/*
└────────────┴────────────────┴────────┴──────────┴─────────┴─────────┘
                              │
                      ┌───────▼───────┐
                      │   Rosbridge   │
                      │  WebSocket    │
                      │  Port 9090    │
                      └───────┬───────┘
                              │ WS
                      ┌───────▼───────┐
                      │  Web Browser  │
                      │   Dashboard   │
                      │  (HTML/JS)    │
                      └───────────────┘
```

## Quick Start (4 Terminals)

```bash
# T1: Gazebo simulation
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta

# T2: Rosbridge WebSocket server
ros2 launch rosbridge_server rosbridge_websocket_launch.xml delay_between_messages:=0.0

# T3: Modular navigation (recommended)
ros2 launch ~/seal_ws/src/uvautoboat/launch/vostok1.launch.yaml

# T4: Web dashboard
cd ~/seal_ws/src/uvautoboat/web_dashboard/vostok1 && python3 -m http.server 8000
```

Then open: **<http://localhost:8000>**

---

## 🚀 Mission Workflow Guide

This section describes the typical workflow for running autonomous missions using the web dashboard.

### Modular Mode Workflow (OKO + SPUTNIK + BURAN)

The modular architecture uses three separate nodes for perception, planning, and control. This is the **recommended** mode for production use.

#### Step 1: Launch the System (4 Terminals)

```bash
# Terminal 1: Gazebo simulation
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta

# Terminal 2: Rosbridge WebSocket server
ros2 launch rosbridge_server rosbridge_websocket_launch.xml delay_between_messages:=0.0

# Terminal 3: Modular navigation stack
ros2 launch ~/seal_ws/src/uvautoboat/launch/vostok1.launch.yaml

# Terminal 4: Web dashboard server
cd ~/seal_ws/src/uvautoboat/web_dashboard/vostok1 && python3 -m http.server 8000
```

#### Step 2: Open Dashboard & Wait for GPS

1. Open **<http://localhost:8000>** in your browser
2. Verify connection status shows **Connected** (green)
3. Wait for GPS coordinates to appear (may take 5-10 seconds after simulation starts)
4. The Mission Control panel will show state: **INIT**

#### Step 3: Configure Mission Parameters (Optional)

In the **Configuration** panel, you can adjust:

- **Scan Length/Width**: Coverage area dimensions in meters
- **Lanes**: Number of lawnmower lanes
- **PID Gains (Kp, Ki, Kd)**: Heading control tuning
- **Base/Max Speed**: Thruster power limits
- **Safe Distance**: Obstacle detection threshold

Click **Send Config** to apply changes.

#### Step 4: Generate Waypoints

1. Click **📍 Generate Waypoints** button
2. Waypoints will appear on the map as blue dots
3. The planned path is shown as a blue line
4. State changes to: **WAITING_CONFIRM**
5. Review the waypoints on the map

#### Step 5: Confirm & Start Mission

1. Click **✓ Confirm** to lock in the waypoints
2. State changes to: **READY**
3. Click **▶ Start** to begin the mission
4. State changes to: **DRIVING**
5. The boat will begin navigating to waypoints

#### Step 6: Monitor Progress

- **Current waypoint** indicator shows progress (e.g., "3/15")
- **Distance to target** shows meters to current waypoint
- **Obstacle status** shows clearance in each direction
- **Trajectory** is drawn on the map as the boat moves
- Watch the terminal for log messages

#### Step 7: Mission Control During Operation

| Button | Action |
|--------|--------|
| **⏸ Stop** | Pause mission, stop motors immediately |
| **▶ Resume** | Continue mission from current waypoint |
| **🏠 Go Home** | Cancel mission, navigate to spawn point |
| **🔄 Reset** | Clear all waypoints, return to INIT |

#### Step 8: Handle Blocked/Stuck Situations

If the boat gets stuck:

1. BURAN automatically enters **escape mode** (SASS v2.0)
2. The anti-stuck panel shows escalation level
3. After 4 failed attempts, waypoint is automatically skipped
4. You can manually click **⏸ Stop** then **🏠 Go Home** to return

#### Step 9: Mission Complete

- When all waypoints are reached, state changes to: **FINISHED**
- Click **🏠 Go Home** to return to spawn
- Or click **▶ Start** to run the mission again

---

### Integrated Mode Workflow (Vostok1 Single Node)

The integrated Vostok1 node combines all functionality in one process. Useful for simpler deployments.

#### Step 1: Launch the System (4 Terminals)

```bash
# Terminal 1: Gazebo simulation
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta

# Terminal 2: Rosbridge WebSocket server
ros2 launch rosbridge_server rosbridge_websocket_launch.xml delay_between_messages:=0.0

# Terminal 3: Integrated Vostok1 navigation
ros2 run plan vostok1

# Terminal 4: Web dashboard server
cd ~/seal_ws/src/uvautoboat/web_dashboard/vostok1 && python3 -m http.server 8000
```

#### Step 2-9: Same as Modular Mode

The workflow is identical - the dashboard automatically detects which mode is running and subscribes to the appropriate topics.

---

### CLI Alternative (No Web Dashboard)

You can also control missions via the command-line interface:

```bash
# Modular mode (default)
ros2 run plan vostok1_cli generate --lanes 8 --length 50 --width 20
ros2 run plan vostok1_cli confirm
ros2 run plan vostok1_cli start
ros2 run plan vostok1_cli status
ros2 run plan vostok1_cli stop
ros2 run plan vostok1_cli resume
ros2 run plan vostok1_cli home
ros2 run plan vostok1_cli reset

# Integrated mode
ros2 run plan vostok1_cli --mode vostok1 generate
ros2 run plan vostok1_cli --mode vostok1 start

# Interactive mode (menu-driven)
ros2 run plan vostok1_cli interactive
```

---

### Troubleshooting Common Issues

| Problem | Solution |
|---------|----------|
| Stop button doesn't work | Ensure latest code is built (`colcon build --packages-select plan control`) |
| Boat stuck in escape mode after stop | Fixed in latest version - escape state resets on stop/resume |
| Go Home doesn't respond | Check GPS is available, ensure state is not INIT |
| Waypoints don't appear on map | Wait for GPS fix, check browser console for errors |
| Mission state stuck | Try Reset button, then Generate new waypoints |

---

## Files

- `index.html` - Main dashboard structure
- `style_merged.css` - Unified stylesheet with 3 style modes
- `app.js` - ROS connection and data handling logic
- `README_vostok1_dashboard.md` - This file

### Style Modes

The dashboard supports 4 visual styles (click the toggle button to cycle):

| Mode | Description |
|------|-------------|
| **Normal** | Clean purple gradient, modern look |
| **Bureau** | TNO Soviet industrial aesthetic with CRT effects |
| **Terminal** | Green phosphor CRT retro computer style |
| **MilSpec** | Warsaw Pact military specification (ВМФ СССР) |

## Future Enhancements

- [x] ~~Add parameter configuration panel~~ ✅ Implemented
- [x] ~~Implement emergency stop button~~ ✅ Mission control buttons added
- [ ] Add waypoint editing/planning interface
- [ ] Display LIDAR point cloud visualization
- [ ] Display OKO v2.0 cluster/gap visualization on map
- [ ] Mission recording and playback
- [ ] Multi-boat support
- [ ] Historical data charts

## License

Part of the uvautoboat project. See main repository for license details.

## Credits

Built for the Vostok1 autonomous boat navigation system using:

- [roslibjs](http://robotwebtools.org/) for ROS communication
- [Leaflet.js](https://leafletjs.com/) for map visualization
- [OpenStreetMap](https://www.openstreetmap.org/) for map tiles
