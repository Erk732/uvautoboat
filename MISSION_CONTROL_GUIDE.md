# Atlantis Mission Control - Auto-Start vs Manual Start

## Overview
The controller now supports **dual-mode mission activation**:
- **Auto-Start Mode (Default)**: Boat automatically starts driving when GPS + path are ready
- **Manual-Start Mode**: Boat waits for explicit `/atlantis/start` signal (via dashboard or manual trigger)

## Mode 1: AUTO-START (Default)

### When to Use
- Autonomous missions
- Testing & development
- Reliable GPS signal in all conditions

### How It Works
1. Planner generates path → publishes to `/atlantis/path`
2. Controller receives GPS fix → sets home position
3. ✅ **Mission auto-enables** → Boat starts moving immediately

### Terminal Output
```
[INFO] Home Position Set: (-33.72276965153148, 150.67398261300283)
[INFO] Received new plan with 11 waypoints
✅ Mission AUTO-ENABLED (GPS ready + path received)
```

### Launch Command
```bash
ros2 run control atlantis_controller
# OR explicitly enable auto-start:
ros2 run control atlantis_controller --ros-args -p auto_start_mission:=true
```

---

## Mode 2: MANUAL-START (Safe/Backup)

### When to Use
- Safety-critical operations
- Uncertain GPS conditions
- Manual verification needed before mission starts
- Testing waypoint generation without moving

### How It Works
1. Planner generates path → publishes to `/atlantis/path`
2. Controller receives GPS fix → sets home position
3. ⏸️ **Mission stays paused** → You control when to start
4. Send `/atlantis/start` signal → Boat starts moving

### Terminal Output (Waiting for Start)
```
[INFO] Home Position Set: (-33.72276965153148, 150.67398261300283)
[INFO] Received new plan with 11 waypoints
[WARN] Mission PAUSED - waiting for /atlantis/start signal
```

### Launch Command
```bash
ros2 run control atlantis_controller --ros-args -p auto_start_mission:=false
```

### Trigger Manual Start
**Option A: Dashboard**
- Open web dashboard → Click "START MISSION" button
- Sends `/atlantis/start` Empty message

**Option B: Terminal Command**
```bash
ros2 topic pub -1 /atlantis/start std_msgs/Empty
```

**Option C: ROS2 Python Script**
```python
import rclpy
from std_msgs.msg import Empty

rclpy.init()
node = rclpy.create_node('mission_starter')
pub = node.create_publisher(Empty, '/atlantis/start', 10)
pub.publish(Empty())
rclpy.shutdown()
```

---

## Recommended Workflow

### Development/Testing
```bash
# Terminal 1: Launch environment
ros2 launch gazebo_ros gazebo.launch.py

# Terminal 2: Start planner (always auto-generates path)
ros2 run plan atlantis_planner

# Terminal 3: Start controller in MANUAL mode (safe)
ros2 run control atlantis_controller --ros-args -p auto_start_mission:=false

# Terminal 4: Verify mission, then manually start
ros2 topic pub -1 /atlantis/start std_msgs/Empty
```

### Production Autonomous
```bash
# Terminal 1: Environment
ros2 launch gazebo_ros gazebo.launch.py

# Terminal 2: Planner
ros2 run plan atlantis_planner

# Terminal 3: Controller (auto-start enabled)
ros2 run control atlantis_controller
# Or explicit: --ros-args -p auto_start_mission:=true
```

### Safe Checkpoint Testing
```bash
# Controller in manual mode
ros2 run control atlantis_controller --ros-args -p auto_start_mission:=false

# Verify conditions
# - GPS locked? → Check logs
# - Path reasonable? → Check waypoints
# - Obstacles clear? → Check lidar output
# Then start manually:
ros2 topic pub -1 /atlantis/start std_msgs/Empty
```

---

## Implementation Details

### Parameter Declaration
```python
self.declare_parameter('auto_start_mission', True)  # Default: True
```

### Logic in path_callback()
```python
if self.get_parameter('auto_start_mission').value and self.start_gps is not None:
    self.mission_enabled = True
    self.get_logger().info("✅ Mission AUTO-ENABLED (GPS ready + path received)")
```

### Logic in gps_callback()
```python
if self.get_parameter('auto_start_mission').value and len(self.waypoints) > 0:
    self.mission_enabled = True
    self.get_logger().info("✅ Mission AUTO-ENABLED (path ready + GPS acquired)")
```

### Manual Start (Always Available)
```python
def start_callback(self, msg):
    """Enable mission from dashboard or manual trigger"""
    self.mission_enabled = True
    if self.start_gps is not None and len(self.waypoints) > 0:
        self.state = "DRIVING"
        self.start_time = self.get_clock().now()
    self.get_logger().info("🚀 Mission ENABLED from dashboard/manual trigger")
```

---

## Changing Modes at Runtime

### Dynamic Parameter Updates (ROS2 2.0+)
```bash
# Switch TO manual mode while running
ros2 param set /atlantis_controller auto_start_mission false

# Switch TO auto-start while running
ros2 param set /atlantis_controller auto_start_mission true

# Check current setting
ros2 param get /atlantis_controller auto_start_mission
```

---

## Troubleshooting

### "Boat not moving but I enabled auto-start"
- ✅ Check: GPS locked? → `[INFO] Home Position Set: ...`
- ✅ Check: Path received? → `[INFO] Received new plan with X waypoints`
- ✅ Check: Parameter set? → `ros2 param get /atlantis_controller auto_start_mission`

### "Manual start not working"
- ✅ Check: Controller in manual mode? → `auto_start_mission:=false`
- ✅ Check: GPS locked? → Needed before manual start works
- ✅ Retry: `ros2 topic pub -1 /atlantis/start std_msgs/Empty`

### "Want to verify waypoints before moving"
- ✅ Launch in manual mode: `auto_start_mission:=false`
- ✅ Check planner output: `GENERATED: Mission Path (11 points)`
- ✅ Inspect waypoints: `ros2 topic echo /atlantis/waypoints`
- ✅ Then start: `ros2 topic pub -1 /atlantis/start std_msgs/Empty`

---

## Summary Table

| Feature | Auto-Start | Manual-Start |
|---------|-----------|--------------|
| Startup Speed | Immediate | Controlled |
| Safety | Good for proven setups | Better for untested missions |
| Manual Override | Always available | Primary control |
| Best For | Autonomous, tested missions | Development, safety-critical |
| Parameter | `auto_start_mission:=true` | `auto_start_mission:=false` |

---

## Default Configuration
- **Auto-Start is ENABLED by default** (`auto_start_mission:=true`)
- Manual start always available as backup
- Dashboard `/atlantis/start` signal always works regardless of mode
