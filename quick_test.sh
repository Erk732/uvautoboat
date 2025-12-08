#!/bin/bash
# 快速系统诊断脚本
# 使用: ./quick_test.sh

set -e

echo "================================"
echo "UVAutoboat 快速诊断"
echo "================================"
echo ""

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 检查 1: ROS2 是否安装
echo "1️⃣  检查 ROS2 安装..."
if command -v ros2 &> /dev/null; then
    ros2_version=$(ros2 --version | cut -d' ' -f2)
    echo -e "${GREEN}✅ ROS2 ${ros2_version} 已安装${NC}"
else
    echo -e "${RED}❌ ROS2 未安装${NC}"
    exit 1
fi
echo ""

# 检查 2: 工作空间是否编译
echo "2️⃣  检查工作空间编译..."
if [ -d "build" ] && [ -d "install" ]; then
    echo -e "${GREEN}✅ 工作空间已编译${NC}"
else
    echo -e "${YELLOW}⚠️  工作空间未编译，建议运行: colcon build${NC}"
fi
echo ""

# 检查 3: launch 文件语法
echo "3️⃣  检查 launch 文件语法..."
if python3 -m py_compile control/launch/all_in_one_bringup.launch.py 2>/dev/null; then
    echo -e "${GREEN}✅ launch 文件语法正确${NC}"
else
    echo -e "${RED}❌ launch 文件有语法错误${NC}"
fi
echo ""

# 检查 4: 检查关键文件是否存在
echo "4️⃣  检查关键文件..."
files=(
    "control/control/all_in_one_stack.py"
    "control/control/gps_imu_pose.py"
    "control/control/pose_filter.py"
    "control/launch/all_in_one_bringup.launch.py"
)

all_exist=true
for file in "${files[@]}"; do
    if [ -f "$file" ]; then
        echo -e "${GREEN}✅ $file${NC}"
    else
        echo -e "${RED}❌ $file 缺失${NC}"
        all_exist=false
    fi
done
echo ""

if [ "$all_exist" = false ]; then
    echo -e "${RED}❌ 某些文件缺失，请检查${NC}"
    exit 1
fi

echo ""
echo "================================"
echo "✅ 所有基础检查通过！"
echo "================================"
echo ""

echo "📝 建议的下一步操作："
echo ""
echo "1️⃣  启动系统："
echo "   ros2 launch control all_in_one_bringup.launch.py"
echo ""
echo "2️⃣  在另一个终端，验证节点是否运行："
echo "   ros2 node list"
echo ""
echo "3️⃣  发送测试目标："
echo "   ros2 topic pub /planning/goal geometry_msgs/PoseStamped \\"
echo "     \"{header: {frame_id: 'world'}, pose: {position: {x: 100.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}\" --once"
echo ""
echo "4️⃣  监看系统输出："
echo "   ros2 launch control all_in_one_bringup.launch.py 2>&1 | grep -E 'goal|avoid|distance'"
echo ""
echo "更多诊断命令，查看: TESTING_DIAGNOSTIC_GUIDE.md"
