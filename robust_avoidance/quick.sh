#!/bin/bash
# Quick system diagnostic script
# Usage: ./quick_test.sh

set -e

echo "================================"
echo "UVAutoboat Quick Diagnostics"
echo "================================"
echo ""

# Color definitions
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check 1: ROS2 installation
echo "1. Checking ROS2 installation..."
if command -v ros2 &> /dev/null; then
    ros2_version=$(ros2 --version | cut -d' ' -f2)
    echo -e "${GREEN}[OK] ROS2 ${ros2_version} installed${NC}"
else
    echo -e "${RED}[X]  ROS2 not installed${NC}"
    exit 1
fi
echo ""

# Check 2: Workspace compilation
echo "2. Checking workspace compilation..."
if [ -d "build" ] && [ -d "install" ]; then
    echo -e "${GREEN}[OK] Workspace compiled${NC}"
else
    echo -e "${YELLOW}[!]  Workspace not compiled, suggest running: colcon build${NC}"
fi
echo ""

# Check 3: Launch file syntax
echo "3. Checking launch file syntax..."
if python3 -m py_compile control/launch/all_in_one_bringup.launch.py 2>/dev/null; then
    echo -e "${GREEN}[OK] Launch file syntax correct${NC}"
else
    echo -e "${RED}[X]  Launch file has syntax errors${NC}"
fi
echo ""

# Check 4: Check if critical files exist
echo "4. Checking critical files..."
files=(
    "control/control/all_in_one_stack.py"
    "control/control/gps_imu_pose.py"
    "control/control/pose_filter.py"
    "control/launch/all_in_one_bringup.launch.py"
)

all_exist=true
for file in "${files[@]}"; do
    if [ -f "$file" ]; then
        echo -e "${GREEN}[OK] $file${NC}"
    else
        echo -e "${RED}[X]  $file missing${NC}"
        all_exist=false
    fi
done
echo ""

if [ "$all_exist" = false ]; then
    echo -e "${RED}[X]  Some files missing, please check${NC}"
    exit 1
fi

echo ""
echo "================================"
echo "[OK] All basic checks passed!"
echo "================================"
echo ""

echo "Suggested next steps:"
echo ""
echo "1. Start the system:"
echo "   ros2 launch control all_in_one_bringup.launch.py"
echo ""
echo "2. In another terminal, verify nodes are running:"
echo "   ros2 node list"
echo ""
echo "3. Send a test goal:"
echo "   ros2 topic pub /planning/goal geometry_msgs/PoseStamped \\"
echo "     \"{header: {frame_id: 'world'}, pose: {position: {x: 100.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}\" --once"
echo ""
echo "4. Monitor system output:"
echo "   ros2 launch control all_in_one_bringup.launch.py 2>&1 | grep -E 'goal|avoid|distance'"
echo ""
echo "For more diagnostic commands, see: TESTING_DIAGNOSTIC_GUIDE.md"
