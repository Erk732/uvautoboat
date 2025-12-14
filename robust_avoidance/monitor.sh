#!/bin/bash
# Real-time system monitoring dashboard
# Usage: ./monitor_boat.sh

# Check if ROS2 is running
check_ros() {
    ros2 node list &>/dev/null
    if [ $? -ne 0 ]; then
        echo "❌ ROS2 not started, please run first: ros2 launch control all_in_one_bringup.launch.py"
        exit 1
    fi
}

# Get position information
get_position() {
    ros2 topic echo -n 1 /wamv/pose_filtered 2>/dev/null | grep -E "x:|y:" | head -2 | tr '\n' ' '
}

# Get goal distance (inferred from path)
get_goal_distance() {
    path_data=$(ros2 topic echo -n 1 /planning/path 2>/dev/null | grep -A 50 "poses:")
    if [ -n "$path_data" ]; then
        echo "$path_data" | grep "position:" | wc -l
    else
        echo "N/A"
    fi
}

# Get thruster status
get_thrusters() {
    left=$(ros2 topic echo -n 1 /wamv/thrusters/left/thrust 2>/dev/null | grep data | awk '{print $NF}')
    right=$(ros2 topic echo -n 1 /wamv/thrusters/right/thrust 2>/dev/null | grep data | awk '{print $NF}')
    echo "L:$left R:$right"
}

# Get lidar frequency
get_lidar_hz() {
    ros2 topic hz -c 1 /wamv/sensors/lidars/lidar_wamv_sensor/scan 2>/dev/null | grep "average" | awk '{print $2}' | cut -d'.' -f1
}

# Main loop
check_ros

echo ""
echo "UVAutoboat Real-time Monitor (updates every 2 seconds)"
echo "Press Ctrl+C to exit"
echo ""

counter=0
while true; do
    clear
    echo "=================================================="
    echo "       UVAutoboat System Monitor Dashboard        "
    echo "=================================================="
    echo ""

    # Runtime
    counter=$((counter + 1))
    echo "Runtime: $(($counter * 2)) seconds"
    echo ""

    # Node status
    echo "Node Status:"
    if ros2 node list 2>/dev/null | grep -q "all_in_one_stack"; then
        echo "   [OK] all_in_one_stack (controller)"
    else
        echo "   [X]  all_in_one_stack (controller)"
    fi

    if ros2 node list 2>/dev/null | grep -q "gps_imu_pose"; then
        echo "   [OK] gps_imu_pose (GPS/IMU)"
    else
        echo "   [X]  gps_imu_pose (GPS/IMU)"
    fi
    echo ""

    # Position
    echo "Current Position:"
    position=$(get_position)
    if [ -n "$position" ]; then
        echo "   $position"
    else
        echo "   Waiting for data..."
    fi
    echo ""

    # Thrusters
    echo "Thruster Commands:"
    thrusters=$(get_thrusters)
    echo "   $thrusters"
    echo ""

    # Lidar
    echo "Lidar:"
    lidar_hz=$(get_lidar_hz)
    if [ -n "$lidar_hz" ] && [ "$lidar_hz" != "" ]; then
        echo "   Frequency: ${lidar_hz} Hz"
    else
        echo "   Frequency: Detecting..."
    fi
    echo ""

    # Path
    echo "Navigation Path:"
    waypoints=$(get_goal_distance)
    if [ "$waypoints" != "N/A" ]; then
        echo "   Waypoints: $waypoints"
    else
        echo "   Waypoints: Waiting for goal..."
    fi
    echo ""

    # Bottom tips
    echo "=================================================="
    echo "Tip: Send goal in another terminal:"
    echo "   ros2 topic pub /planning/goal ..."
    echo "=================================================="

    sleep 2
done
