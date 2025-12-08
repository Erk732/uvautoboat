#!/bin/bash
# 实时系统监控仪表板
# 使用: ./monitor_boat.sh

# 检查 ROS2 是否运行
check_ros() {
    ros2 node list &>/dev/null
    if [ $? -ne 0 ]; then
        echo "❌ ROS2 未启动，请先运行: ros2 launch control all_in_one_bringup.launch.py"
        exit 1
    fi
}

# 获取位置信息
get_position() {
    ros2 topic echo -n 1 /wamv/pose_filtered 2>/dev/null | grep -E "x:|y:" | head -2 | tr '\n' ' '
}

# 获取目标距离（从路径推断）
get_goal_distance() {
    path_data=$(ros2 topic echo -n 1 /planning/path 2>/dev/null | grep -A 50 "poses:")
    if [ -n "$path_data" ]; then
        echo "$path_data" | grep "position:" | wc -l
    else
        echo "N/A"
    fi
}

# 获取推进器状态
get_thrusters() {
    left=$(ros2 topic echo -n 1 /wamv/thrusters/left/thrust 2>/dev/null | grep data | awk '{print $NF}')
    right=$(ros2 topic echo -n 1 /wamv/thrusters/right/thrust 2>/dev/null | grep data | awk '{print $NF}')
    echo "L:$left R:$right"
}

# 获取激光雷达频率
get_lidar_hz() {
    ros2 topic hz -c 1 /wamv/sensors/lidars/lidar_wamv_sensor/scan 2>/dev/null | grep "average" | awk '{print $2}' | cut -d'.' -f1
}

# 主循环
check_ros

echo ""
echo "🚀 UVAutoboat 实时监控 (每 2 秒更新一次)"
echo "按 Ctrl+C 退出"
echo ""

counter=0
while true; do
    clear
    echo "╔════════════════════════════════════════════════╗"
    echo "║         UVAutoboat 系统监控仪表板              ║"
    echo "╚════════════════════════════════════════════════╝"
    echo ""
    
    # 运行时间
    counter=$((counter + 1))
    echo "⏱️  运行时间: $(($counter * 2)) 秒"
    echo ""
    
    # 节点状态
    echo "📍 节点状态:"
    if ros2 node list 2>/dev/null | grep -q "all_in_one_stack"; then
        echo "   ✅ all_in_one_stack (控制器)"
    else
        echo "   ❌ all_in_one_stack (控制器)"
    fi
    
    if ros2 node list 2>/dev/null | grep -q "gps_imu_pose"; then
        echo "   ✅ gps_imu_pose (GPS/IMU)"
    else
        echo "   ❌ gps_imu_pose (GPS/IMU)"
    fi
    echo ""
    
    # 位置
    echo "🗺️  当前位置:"
    position=$(get_position)
    if [ -n "$position" ]; then
        echo "   $position"
    else
        echo "   ⏳ 等待数据..."
    fi
    echo ""
    
    # 推进器
    echo "⚙️  推进器命令:"
    thrusters=$(get_thrusters)
    echo "   $thrusters"
    echo ""
    
    # 激光雷达
    echo "📡 激光雷达:"
    lidar_hz=$(get_lidar_hz)
    if [ -n "$lidar_hz" ] && [ "$lidar_hz" != "" ]; then
        echo "   频率: ${lidar_hz} Hz"
    else
        echo "   频率: 检测中..."
    fi
    echo ""
    
    # 路径
    echo "🛣️  导航路径:"
    waypoints=$(get_goal_distance)
    if [ "$waypoints" != "N/A" ]; then
        echo "   路径点数: $waypoints"
    else
        echo "   路径点数: 等待目标..."
    fi
    echo ""
    
    # 底部提示
    echo "╔════════════════════════════════════════════════╗"
    echo "💡 提示: 在另一个终端发送目标:"
    echo "   ros2 topic pub /planning/goal ..."
    echo "════════════════════════════════════════════════"
    
    sleep 2
done
