# Vostok1 Web Dashboard

A real-time web-based monitoring dashboard for the Vostok1 autonomous boat system.

## Features

- **Real-time GPS tracking** with trajectory visualization on interactive map
- **Obstacle detection monitoring** showing minimum distances and clearance zones
- **Thruster output visualization** with live thrust bars
- **Mission status display** including state, waypoint, and distance
- **SASS v2.0 Anti-Stuck Status** with visual escalation indicators
- **System logs** with color-coded severity levels
- **Dual-mode support**: Works with both integrated Vostok1 and modular navigation
- **4 Style modes**: Normal, Bureau (TNO), Terminal (CRT), **MilSpec (Warsaw Pact)**
- **Configuration panel**: Adjust mission parameters in real-time
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

### 4. Launch Vostok1

**Terminal 3:**

```bash
cd ~/seal_ws
source install/setup.bash
ros2 run plan vostok1
```

Or use your launch file if available.

### 5. Open Dashboard

**Terminal 4:**

```bash
cd ~/seal_ws/src/uvautoboat/web_dashboard
python3 -m http.server 8000
```

Then open your web browser and navigate to:

```bash
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
| `/perception/obstacle_info` | `std_msgs/String` | Oko perception data |
| `/control/status` | `std_msgs/String` | Buran controller status |
| `/control/anti_stuck_status` | `std_msgs/String` | Buran anti-stuck status |

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
┌─────────────────────────────────────────────────────────────────┐
│                     Navigation Nodes                            │
├─────────────────────────────┬───────────────────────────────────┤
│   Integrated Mode           │   Modular Mode                    │
│   ┌─────────────────┐       │   ┌─────────┐ ┌─────┐ ┌───────┐   │
│   │   Vostok1 Node  │       │   │ Sputnik │ │ Oko │ │ Buran │   │
│   │   (All-in-one)  │       │   │ Planner │ │Perc.│ │ Ctrl  │   │
│   └────────┬────────┘       │   └────┬────┘ └──┬──┘ └───┬───┘   │
│            │                │        │         │        │       │
│   /vostok1/* topics         │   /planning/*  /perception/* /control/*
└────────────┴────────────────┴────────┴─────────┴────────┴───────┘
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

## Files

- `index.html` - Main dashboard structure
- `style_merged.css` - Unified stylesheet with 3 style modes
- `app.js` - ROS connection and data handling logic
- `README_vostok1_dashboard.md` - This file

### Style Modes

The dashboard supports 3 visual styles (click the toggle button to cycle):

| Mode | Description |
|------|-------------|
| **Normal** | Clean purple gradient, modern look |
| **Bureau** | TNO Soviet industrial aesthetic with CRT effects |
| **Terminal** | Green phosphor CRT retro computer style |

## Future Enhancements

- [ ] Add waypoint editing/planning interface
- [ ] Implement emergency stop button
- [ ] Add parameter configuration panel
- [ ] Display LIDAR point cloud visualization
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
