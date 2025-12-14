# Common Issues & Solutions

Troubleshooting guide for frequent problems with AutoBoat.

---

## Navigation Issues

### Boat Not Moving

**Symptoms**: Boat sits idle despite mission started

**Diagnosis**:

```bash
# Check GPS signal
ros2 topic echo /wamv/sensors/gps/gps/fix --once
```

**Solutions**:

1. **GPS not initialized**: Wait 5-10 seconds after Gazebo launches
2. **Mission not started**: Run `ros2 run plan vostok1_cli start`
3. **Waypoints not generated**: Run `ros2 run plan vostok1_cli generate`
4. **Node not running**: Check `ros2 node list | grep vostok`

---

### Boat Spinning in Circles

**Symptoms**: Boat rotates continuously without making progress

**Cause**: PID gains too high (over-responsive)

**Solution**: Reduce PID proportional gain

```bash
# For Vostok1
ros2 param set /vostok1_node kp 300.0

# For Modular (BURAN)
ros2 param set /buran_controller kp 300.0
```

**Alternative**: Increase derivative gain for damping

```bash
ros2 param set /vostok1_node kd 150.0
```

---

### Boat Oscillates Around Heading

**Symptoms**: Boat zigzags toward target instead of straight line

**Cause**: Insufficient damping (Kd too low)

**Solution**: Increase derivative gain

```bash
ros2 param set /vostok1_node kd 150.0
```

---

### Boat Drifts Off Course

**Symptoms**: Boat consistently misses waypoints to one side

**Causes**:

1. **Simulated current/wind**: Normal behavior, simple anti-stuck will compensate
2. **Integral windup**: Ki too high

**Solutions**:

1. Enable simple anti-stuck (should be active by default)
2. Reduce integral gain if overshooting:

   ```bash
   ros2 param set /vostok1_node ki 10.0
   ```

---

### "CRITICAL" Warning at Spawn

**Symptoms**: Boat immediately detects critical obstacle at start

**Cause**: Dock/harbor structure within minimum LIDAR range

**Solution**: Increase minimum detection range

```yaml
# In vostok1.launch.yaml
- name: min_range
  value: 7.0  # Increase from default 5.0
```

**Alternative**: Teleport boat away from dock

```bash
gz service -s /world/sydney_regatta/set_pose \
  --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 1000 \
  --req 'name: "wamv", position: {x: 10, y: 10, z: 0.5}'
```

---

### Simple Anti-Stuck Activates Too Frequently

**Symptoms**: Simple anti-stuck system triggers during normal navigation

**Cause**: Stuck detection too sensitive

**Solutions**:

1. Increase timeout:

   ```bash
   ros2 param set /buran_controller stuck_timeout 5.0
   ```

2. Increase movement threshold:

   ```bash
   ros2 param set /buran_controller stuck_threshold 1.0
   ```

---

### Waypoints Skip Too Often

**Symptoms**: Mission skips waypoints that appear reachable

**Cause**: Waypoint skip timeout too short for obstacle avoidance

**Solution**: Increase timeout

```yaml
# In vostok1.launch.yaml
- name: waypoint_skip_timeout
  value: 60.0  # Increase from default 45.0
```

---

## Obstacle Detection Issues

### No Obstacles Detected

**Symptoms**: Boat shows "CLEAR" even near obstacles

**Diagnosis**:

```bash
# Check LIDAR data rate
ros2 topic hz /wamv/sensors/lidars/lidar_wamv/points

# Check obstacle info
ros2 topic echo /perception/obstacle_info
```

**Solutions**:

1. **LIDAR not publishing**: Restart Gazebo
2. **Height filter too restrictive**: Adjust range

   ```yaml
   min_height: -20.0
   max_height: 15.0
   ```

3. **Range filter too restrictive**: Increase max_range

   ```yaml
   max_range: 100.0
   ```

---

### False Obstacle Detections

**Symptoms**: Boat detects obstacles in empty water

**Cause**: Water reflections or noise

**Solutions**:

1. **Enable water plane removal** (should be default)
2. **Increase temporal filtering**:

   ```yaml
   temporal_history_size: 7
   temporal_threshold: 5
   ```

3. **Adjust water plane threshold**:

   ```yaml
   water_plane_threshold: 0.8
   ```

---

### Boat Collides with Obstacles

**Symptoms**: Boat hits obstacles despite LIDAR detection

**Causes**:

1. **Safe distance too small**
2. **Control loop too slow**
3. **Speed too high near obstacles**

**Solutions**:

1. Increase safe distance:

   ```yaml
   min_safe_distance: 15.0  # Increase from 12.0
   ```

2. Reduce obstacle slow factor:

   ```yaml
   obstacle_slow_factor: 0.2  # More aggressive slowdown
   ```

3. Reduce max speed:

   ```yaml
   max_speed: 600.0  # Reduce from 800.0
   ```

---

## Dashboard Issues

### Dashboard Not Connecting

**Symptoms**: "Disconnected" status in dashboard

**Diagnosis**:

```bash
# Check if rosbridge is running
ros2 node list | grep rosbridge

# Check port 9090
netstat -tuln | grep 9090
```

**Solutions**:

1. **Start rosbridge**:

   ```bash
   ros2 launch rosbridge_server rosbridge_websocket_launch.xml delay_between_messages:=0.0
   ```

2. **Check WebSocket URL**: Should be `ws://localhost:9090`
3. **Check browser console** (F12): Look for WebSocket errors
4. **Firewall blocking**: Allow port 9090

---

### Camera Panel Shows "No Image"

**Symptoms**: Dashboard camera panel blank

**Solutions**:

1. **Start web_video_server**:

   ```bash
   ros2 run web_video_server web_video_server
   ```

2. **Check camera topic**: Default is `/wamv/sensors/cameras/front_left_camera_sensor/image_raw`
3. **Verify topic exists**:

   ```bash
   ros2 topic list | grep camera
   ```

4. **Refresh stream**: Click "Refresh Stream" button in dashboard

---

### Dashboard Shows Old Data

**Symptoms**: Dashboard not updating, stale information

**Solutions**:

1. **Refresh browser page** (Ctrl+Shift+R)
2. **Restart rosbridge**
3. **Check ROS topics publishing**:

   ```bash
   ros2 topic hz /vostok1/mission_status
   ```

---

### Map Not Showing Boat Position

**Symptoms**: Trajectory map empty or boat icon missing

**Causes**:

1. GPS not publishing
2. WebSocket not connected
3. Browser blocking Leaflet.js

**Solutions**:

1. Check GPS:

   ```bash
   ros2 topic echo /wamv/sensors/gps/gps/fix --once
   ```

2. Check browser console for JavaScript errors
3. Ensure internet connection (Leaflet loads tiles from online)

---

## Build & Compilation Issues

### Build Failures

**Symptoms**: `colcon build` fails with errors

**Solutions**:

1. **Clean build**:

   ```bash
   cd ~/seal_ws
   rm -rf build install log
   colcon build --merge-install
   ```

2. **Update dependencies**:

   ```bash
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Check Python version**:

   ```bash
   python3 --version  # Should be 3.10+
   ```

---

### Missing Package Errors

**Symptoms**: `ModuleNotFoundError` or `No module named 'X'`

**Solutions**:

1. **Install Python packages**:

   ```bash
   pip3 install numpy scipy matplotlib
   ```

2. **Source ROS 2 environment**:

   ```bash
   source /opt/ros/jazzy/setup.bash
   source ~/seal_ws/install/setup.bash
   ```

---

### Gazebo Plugin Errors

**Symptoms**: Gazebo fails to load plugins or world

**Solutions**:

1. **Source Gazebo setup**:

   ```bash
   source /usr/share/gazebo/setup.sh
   ```

2. **Check plugin paths**:

   ```bash
   echo $GZ_SIM_RESOURCE_PATH
   ```

3. **Rebuild workspace**:

   ```bash
   colcon build --packages-select vrx_gz --merge-install
   ```

---

## Performance Issues

### Gazebo Running Slow

**Symptoms**: Low FPS, stuttering simulation

**Solutions**:

1. **Reduce real-time factor**: Accept slower-than-real-time
2. **Close other applications**: Free up CPU/GPU
3. **Reduce sensor resolution** (advanced):
   - Edit LIDAR parameters in URDF/XACRO
   - Reduce point cloud density
4. **Disable GUI**:

   ```bash
   ros2 launch vrx_gz competition.launch.py world:=sydney_regatta gui:=false
   ```

---

### High CPU Usage

**Symptoms**: 100% CPU utilization

**Causes**: Normal for Gazebo + navigation nodes

**Solutions**:

1. **Monitor specific nodes**:

   ```bash
   top -p $(pgrep -d, gz)
   ```

2. **Reduce LIDAR processing**:
   - Increase `min_range` (process fewer points)
   - Reduce temporal history size
3. **Use haiku model** for agents (if using Claude Code features)

---

## A* Path Planning Issues

### A* Not Finding Paths

**Symptoms**: "No path found" errors, waypoint skip

**Causes**:

1. Safety margin too large
2. Grid resolution too coarse
3. Max expansions too low

**Solutions**:

1. Reduce safety margin:

   ```bash
   ros2 topic pub /sputnik/set_config std_msgs/String \
     "data: '{\"astar_safety_margin\": 8.0}'" --once
   ```

2. Decrease grid resolution (more precise):

   ```bash
   ros2 topic pub /sputnik/set_config std_msgs/String \
     "data: '{\"astar_resolution\": 2.0}'" --once
   ```

3. Increase max expansions:

   ```bash
   ros2 topic pub /sputnik/set_config std_msgs/String \
     "data: '{\"astar_max_expansions\": 50000}'" --once
   ```

---

### A* Planning Too Slow

**Symptoms**: Long delays before boat moves

**Cause**: Grid too fine or search space too large

**Solutions**:

1. Increase grid resolution (coarser, faster):

   ```yaml
   astar_resolution: 5.0  # Increase from 3.0
   ```

2. Reduce max expansions:

   ```yaml
   astar_max_expansions: 10000  # Reduce from 20000
   ```

3. Disable hybrid mode (only use runtime A*):

   ```bash
   ros2 topic pub /sputnik/set_config std_msgs/String \
     "data: '{\"astar_hybrid_mode\": false}'" --once
   ```

---

## System-Level Issues

### "use_sim_time" Warnings

**Symptoms**: TF warnings about time synchronization

**Solution**: Ensure all nodes use simulation time

```yaml
# In launch file
- name: use_sim_time
  value: true
```

**Note**: Currently only `control/launch/all_in_one_bringup.launch.py` sets this. Other launches may need updates.

---

### Nodes Crashing on Start

**Symptoms**: Node starts then immediately exits

**Diagnosis**:

```bash
# Check node logs
ros2 run plan vostok1 --ros-args --log-level debug
```

**Solutions**:

1. **Check dependencies installed**
2. **Source workspace**:

   ```bash
   source ~/seal_ws/install/setup.bash
   ```

3. **Verify Gazebo running** before starting navigation

---

### Cannot Kill Processes

**Symptoms**: `Ctrl+C` doesn't stop nodes

**Nuclear Option**:

```bash
# Kill all Gazebo
pkill -9 -f "gz sim" && pkill -9 -f "gzserver" && pkill -9 -f "gzclient"

# Kill ROS nodes
pkill -9 -f vostok1 && pkill -9 -f rosbridge

# Kill everything (last resort)
pkill -9 -f ros && pkill -9 -f gz && pkill -9 -f gazebo
```

---

## Debug Commands

Useful commands for diagnosing issues:

### Check Node Status

```bash
ros2 node list
ros2 node info /vostok1_node
```

### Check Topics

```bash
ros2 topic list
ros2 topic hz /wamv/sensors/gps/gps/fix
ros2 topic echo /perception/obstacle_info
```

### Check Parameters

```bash
ros2 param list /vostok1_node
ros2 param get /vostok1_node kp
```

### Check Transforms

```bash
ros2 run tf2_tools view_frames
evince frames.pdf
```

### Monitor System Resources

```bash
htop
nvidia-smi  # If using GPU
```

---

## Still Having Issues?

If your problem isn't listed here:

1. **Check logs**: Look for ERROR or WARN messages
2. **Enable debug logging**:

   ```bash
   ros2 run plan vostok1 --ros-args --log-level debug
   ```

3. **Report issue**: [GitHub Issues](https://github.com/Erk732/uvautoboat/issues)
   - Include system info (Ubuntu version, ROS 2 version)
   - Paste relevant logs
   - Describe steps to reproduce

---

## Related Pages

- **[FAQ](FAQ)** — Frequently asked questions
- **[Debug Commands](Debug-Commands)** — Advanced diagnostic tools
- **[Configuration & Tuning](Configuration-and-Tuning)** — Parameter reference
- **[Installation Guide](Installation_Guide)** — Setup troubleshooting
