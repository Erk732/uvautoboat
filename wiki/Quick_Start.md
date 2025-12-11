# Quick Start

Get AutoBoat running in 5 minutes! This guide assumes you've already completed the [Installation Guide](Installation_Guide).

---

## Two-Terminal Quick Start

The fastest way to get AutoBoat running.

### Terminal 1: Launch Simulation

```bash
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
```

**Wait for Gazebo to fully load** (you should see the WAM-V boat in the water).

### Terminal 2: Run Navigation

Choose one of the following navigation systems:

#### Option A: Vostok1 (Recommended)

```bash
ros2 run plan vostok1
```

#### Option B: Modular System

```bash
ros2 launch ~/seal_ws/src/uvautoboat/launch/vostok1.launch.yaml
```

---

## Expected Output

You should see terminal output like:

```text
[INFO] PROJET-17 — Vostok 1 Navigation System
[INFO] Waiting for GPS signal...
[INFO] GPS initialized: lat=XX.XXXX, lon=XX.XXXX
[INFO] MISSION DÉMARRÉE ! | MISSION STARTED!
[INFO] PT 1/19 | Pos: (5.2, 3.1) | Cible: (15.0, 0.0) | Dist: 10.2m | Cap: 45°
[INFO] ✅ DÉGAGÉ | CLEAR (F:50.0 L:50.0 R:50.0)
```

The boat should start moving autonomously through waypoints!

---

## Full Setup with Web Dashboard

For the complete experience with real-time monitoring:

### Terminal 1: Gazebo Simulation

```bash
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
```

### Terminal 2: rosbridge (WebSocket)

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml delay_between_messages:=0.0
```

**Note**: The `delay_between_messages:=0.0` parameter is required for ROS 2 Jazzy.

### Terminal 3: Camera Stream (Optional)

```bash
ros2 run web_video_server web_video_server
```

### Terminal 4: Navigation System

```bash
# Choose one:
ros2 run plan vostok1
# OR
ros2 launch ~/seal_ws/src/uvautoboat/launch/vostok1.launch.yaml
```

### Terminal 5: Dashboard Web Server

```bash
cd ~/seal_ws/src/uvautoboat/web_dashboard/vostok1
python3 -m http.server 8000
```

### Open Dashboard

Open your browser and navigate to:

```bash
http://localhost:8000
```

---

## One-Click Launch (Experimental)

Use the convenience script to launch everything at once:

```bash
cd ~/seal_ws/src/uvautoboat/one_click_launch_all
./launch_vostok1_complete.sh
```

This script opens multiple terminals automatically.

---

## Basic Mission Control

Once the system is running, you can control the mission using the CLI:

### Generate Waypoints

```bash
ros2 run plan vostok1_cli generate --lanes 10 --length 50 --width 20
```

### Start Mission

```bash
ros2 run plan vostok1_cli start
```

### Pause Mission

```bash
ros2 run plan vostok1_cli stop
```

### Resume Mission

```bash
ros2 run plan vostok1_cli resume
```

### Go Home

```bash
ros2 run plan vostok1_cli home
```

---

## What You Should See

### In Gazebo

- The WAM-V boat moving through the water
- Boat navigating toward waypoints
- Thrusters creating water effects

### In Terminal

- GPS coordinates and heading updates
- Waypoint progress (e.g., "PT 3/19")
- Obstacle detection status
- Distance to target

### In Dashboard (if running)

- Real-time boat position on map
- GPS coordinates
- Mission status and progress
- Obstacle clearance values
- Thruster outputs
- Camera feed (if web_video_server running)

---

## Stopping Everything

### Graceful Shutdown

Press `Ctrl+C` in each terminal to stop the nodes.

### Force Kill (if needed)

```bash
# Kill Gazebo
pkill -9 -f "gz sim"

# Kill ROS nodes
pkill -9 -f vostok1
pkill -9 -f rosbridge
```

---

## Next Steps

- **[First Mission Tutorial](First-Mission-Tutorial)** — Detailed step-by-step walkthrough
- **[Terminal Mission Control](Terminal-Mission-Control)** — Advanced CLI usage
- **[Web Dashboard Guide](Web-Dashboard-Guide)** — Dashboard features and configuration
- **[Configuration & Tuning](Configuration-and-Tuning)** — Optimize performance

---

## Troubleshooting Quick Fixes

### Boat Not Moving

```bash
# Check GPS signal
ros2 topic echo /wamv/sensors/gps/gps/fix --once
```

### Dashboard Not Connecting

- Ensure rosbridge is running on port 9090
- Check browser console for connection errors
- Verify WebSocket URL: `ws://localhost:9090`

### Build Errors

```bash
cd ~/seal_ws
rm -rf build install log
colcon build --merge-install
```

For more issues, see **[Common Issues](Common_Issues)**.
