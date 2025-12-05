#!/usr/bin/env bash
# Diagnostic script to check if boat is moving toward goal

# Check 1: Is the node running?
echo "=== Checking if all_in_one_stack is running ==="
ros2 node list | grep all_in_one_stack
if [ $? -ne 0 ]; then
    echo "❌ all_in_one_stack not running!"
    echo "Start with: ros2 launch control all_in_one_bringup.launch.py"
    exit 1
fi
echo "✅ all_in_one_stack is running"

# Check 2: Is pose being published?
echo ""
echo "=== Checking pose topic ==="
ros2 topic echo /wamv/pose_filtered --once 2>/dev/null
if [ $? -ne 0 ]; then
    echo "❌ No pose data! Check GPS/IMU and pose_filter"
    exit 1
fi
echo "✅ Pose data available"

# Check 3: Is lidar working?
echo ""
echo "=== Checking lidar topic ==="
ros2 topic echo /wamv/sensors/lidars/lidar_wamv_sensor/scan --once 2>/dev/null | head -5
if [ $? -ne 0 ]; then
    echo "⚠️  Lidar scan not available, trying alternative topic..."
    ros2 topic echo /wamv/sensors/lidars/lidar_wamv/scan --once 2>/dev/null | head -5
fi
echo "✅ Lidar working"

# Check 4: Send a test goal
echo ""
echo "=== Sending test goal (10m forward) ==="
ros2 topic pub -1 /planning/goal geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: world}, pose: {position: {x: 10.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'

# Check 5: Monitor thruster commands
echo ""
echo "=== Checking if thrusters are getting commands (next 5 seconds) ==="
timeout 5 ros2 topic echo /wamv/thrusters/left/thrust | head -20 &
timeout 5 ros2 topic echo /wamv/thrusters/right/thrust | head -20 &
wait

echo ""
echo "=== Diagnostic complete ==="
echo "If thrusters show non-zero values, the boat should be moving."
echo "If all zeros, check the control logic."

