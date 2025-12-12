#!/bin/bash

# ============================================================================
# PROJET-17 — Vostok1 Complete One-Click Launch Script
# ============================================================================
#
# This script launches the complete Vostok1 autonomous navigation system:
#   - VRX Gazebo Simulation (Sydney Regatta World)
#   - ROS Bridge (WebSocket bridge for web dashboard)
#   - OKO Perception (3D LIDAR obstacle detection)
#   - SPUTNIK Planner (GPS waypoint planning)
#   - BURAN Controller (PID control with obstacle avoidance)
#   - Web Video Server (camera stream for dashboard)
#   - Web Dashboard (real-time monitoring interface)
# Default options
# ----------------------------------------------------------------------------
# Author: IMT Nord Europe UVAutoBoat Team
#
# Usage:
#   chmod +x launch_vostok1_complete.sh
#   ./launch_vostok1_complete.sh [OPTIONS]
#
# Options:
#   --world <world_name>       Gazebo world (default: sydney_regatta_smoke)
#   For example: ./launch_vostok1_complete.sh --world sydney_regatta_DEFAULT
#
#   --skip-rviz                Skip RViz launch (default: launch RViz)
#   --skip-dashboard           Skip web dashboard (default: launch dashboard)
#   --skip-camera              Skip web video server (default: launch camera)
#   --no-browser               Do not open browser (default: open browser)
#   --help                     Show this help message
#
# Prerequisites:
#   - ROS 2 Jazzy installed and sourced
#   - VRX package installed
#   - AutoBoat workspace built: cd ~/seal_ws && colcon build --merge-install # (seal_ws is example workspace)
#   - rosbridge-suite: sudo apt install ros-jazzy-rosbridge-suite (Install if not already)
#   - web_video_server: sudo apt install ros-jazzy-web-video-server (Install if not already)
#   - GNOME Terminal installed (required for multi-tab launch)
# ============================================================================

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'  # No Color

# Graceful exit helper
die() { echo -e "${RED}$*${NC}"; exit 1; }

# Cleanup on exit (kills all spawned components)
cleanup() {
    echo -e "${YELLOW}Stopping all Vostok1 components...${NC}"
    pkill -9 -f "gz sim" || true
    pkill -9 -f "gzserver" || true
    pkill -9 -f "gzclient" || true
    pkill -9 -f "rosbridge_websocket" || true
    pkill -9 -f "web_video_server" || true
    pkill -9 -f "vostok1.launch.yaml" || true
    pkill -9 -f "plan/vostok1" || true
    pkill -9 -f "sputnik_planner" || true
    pkill -9 -f "buran_controller" || true
    pkill -9 -f "oko_perception" || true
    pkill -9 -f "http.server 8000" || true
    pkill -9 -f "rviz" || true
    pkill -9 -f "web_dashboard/vostok1" || true
    # Close gnome-terminal tabs we opened (if still running)
    for pid in $GAZEBO_PID $ROSBRIDGE_PID $NAV_PID $CAMERA_PID $RVIZ_PID $DASHBOARD_PID; do
        if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
        fi
    done
}

# Only clean up on user/system signals, not on normal exit
trap cleanup INT TERM

# Default options
WORLD="sydney_regatta_smoke"
LAUNCH_RVIZ=true
LAUNCH_DASHBOARD=true
LAUNCH_CAMERA=true
OPEN_BROWSER=true

# Parse command-line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --world)
            WORLD="$2"
            shift 2
            ;;
        --skip-rviz)
            LAUNCH_RVIZ=false
            shift
            ;;
        --skip-dashboard)
            LAUNCH_DASHBOARD=false
            shift
            ;;
        --skip-camera)
            LAUNCH_CAMERA=false
            shift
            ;;
        --no-browser)
            OPEN_BROWSER=false
            shift
            ;;
        --help)
            cat << 'EOF'
PROJET-17 Vostok1 Complete Launch Script

Usage: ./launch_vostok1_complete.sh [OPTIONS]

Options:
  --world <world_name>       Gazebo world (default: sydney_regatta_smoke)
  --skip-rviz                Skip RViz launch
  --skip-dashboard           Skip web dashboard launch
  --skip-camera              Skip web video server launch
  --no-browser               Do not open browser
  --help                     Show this help message

Example:
  ./launch_vostok1_complete.sh --world sydney_regatta_DEFAULT --skip-rviz

Worlds available:
  - sydney_regatta_smoke_wildlife (recommended, with obstacles + wildlife)
  - sydney_regatta_smoke (smoke task only)
  - sydney_regatta_DEFAULT (clean environment for testing)
EOF
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# Helper function to print messages
print_header() {
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}========================================${NC}"
}

print_status() {
    echo -e "${GREEN}✓${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}⚠${NC} $1"
}

print_error() {
    echo -e "${RED}✗${NC} $1"
}

# Recheck if a launched tab/process is still alive and report status
recheck() {
    local pid="$1"
    local label="$2"
    sleep 2
    if kill -0 "$pid" 2>/dev/null; then
        print_status "$label is running (PID: $pid)"
    else
        print_warning "$label appears to have exited. Check the tab for errors; consider stopping this script."
    fi
}

# Check prerequisites
print_header "Checking Prerequisites"

if ! command -v ros2 &> /dev/null; then
    die "ROS 2 not found. Please install and source ROS 2 Jazzy."
fi
print_status "ROS 2 found"

if ! command -v gnome-terminal &> /dev/null; then
    print_warning "GNOME Terminal not found. Some features may not work as expected."
fi

if ! command -v gz &> /dev/null; then
    die "Gazebo not found. Please install Gazebo Harmonic."
fi
print_status "Gazebo found"

# Detect workspace root dynamically
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
WS_ROOT="$SCRIPT_DIR"
while [ "$WS_ROOT" != "/" ]; do
    if [ -f "$WS_ROOT/install/setup.bash" ]; then break; fi
    WS_ROOT="$(dirname "$WS_ROOT")"
done
INSTALL_DIR="$WS_ROOT/install"
if [ ! -f "$INSTALL_DIR/setup.bash" ]; then
    die "AutoBoat workspace not built or install/setup.bash not found. Run: colcon build --merge-install"
fi
print_status "AutoBoat workspace found at $WS_ROOT"

# Source environment
source "$INSTALL_DIR/setup.bash"
print_status "Sourced AutoBoat environment"

# Check if VRX is available
if ! ros2 pkg prefix vrx_gz &> /dev/null 2>&1; then
    print_warning "VRX package not found in ROS 2 path. Gazebo launch may fail."
fi

print_header "Launching Vostok1 Complete System"
echo "World: $WORLD"
echo "RViz: $([ "$LAUNCH_RVIZ" = true ] && echo 'Yes' || echo 'No')"
echo "Dashboard: $([ "$LAUNCH_DASHBOARD" = true ] && echo 'Yes' || echo 'No')"
echo "Camera Stream: $([ "$LAUNCH_CAMERA" = true ] && echo 'Yes' || echo 'No')"
echo "Auto-open Browser: $([ "$OPEN_BROWSER" = true ] && echo 'Yes' || echo 'No')"
echo ""
echo "Tip: If tabs close immediately, check their terminal output for missing packages or model downloads."
echo "Shortcuts: skip RViz with --skip-rviz, skip dashboard with --skip-dashboard, skip camera with --skip-camera, change world with --world <name>."
echo ""

# Kill any old processes
print_status "Cleaning up old processes..."
pkill -9 -f "gz sim" &> /dev/null || true
pkill -9 -f "gzserver" &> /dev/null || true
pkill -9 -f "gzclient" &> /dev/null || true
pkill -9 -f "rosbridge" &> /dev/null || true
pkill -9 -f "web_video_server" &> /dev/null || true
pkill -9 -f "vostok1.launch.yaml" &> /dev/null || true
pkill -9 -f "sputnik_planner" &> /dev/null || true
pkill -9 -f "buran_controller" &> /dev/null || true
pkill -9 -f "oko_perception" &> /dev/null || true
pkill -9 -f "http.server 8000" &> /dev/null || true
sleep 4

# T1: Launch Gazebo (VRX Simulation)
print_status "Launching Gazebo (Sydney Regatta - $WORLD)..."
gnome-terminal --wait --tab --title="gazebo" -- bash -i -c "
source \"$INSTALL_DIR/setup.bash\"
echo 'Starting Gazebo with world: $WORLD'
echo 'Note: GPS may take 10-30 seconds to initialize'
ros2 launch vrx_gz competition.launch.py world:=$WORLD
" &
GAZEBO_PID=$!
sleep 28  # Wait for Gazebo to start and physics to initialize
print_status "Gazebo launched (PID: $GAZEBO_PID)"
recheck "$GAZEBO_PID" "Gazebo"

# T2: Launch ROS Bridge (WebSocket for dashboard)
print_status "Launching ROS Bridge (WebSocket on ws://localhost:9090)..."
gnome-terminal --wait --tab --title="rosbridge" -- bash -i -c "
source \"$INSTALL_DIR/setup.bash\"
echo 'Starting ROS Bridge WebSocket server...'
ros2 launch rosbridge_server rosbridge_websocket_launch.xml delay_between_messages:=0.0
" &
ROSBRIDGE_PID=$!
sleep 8  # Wait for rosbridge to initialize
print_status "ROS Bridge launched (PID: $ROSBRIDGE_PID)"
recheck "$ROSBRIDGE_PID" "ROS Bridge"

# T3: Launch Navigation Stack (OKO-SPUTNIK-BURAN)
print_status "Launching Navigation Stack (OKO-SPUTNIK-BURAN)..."
gnome-terminal --wait --tab --title="navigation" -- bash -i -c "
source \"$INSTALL_DIR/setup.bash\"
echo 'Starting Vostok1 Modular Navigation System...'
echo 'Components: OKO (perception) | SPUTNIK (planner) | BURAN (control)'
ros2 launch \"$WS_ROOT/src/uvautoboat/launch/vostok1.launch.yaml\" \
    world:=${WORLD} \
    sputnik_planner_node.world_name:=${WORLD} \
    sputnik_planner_node.pollutant_sdf_glob:=$WS_ROOT/src/uvautoboat/test_environment/${WORLD}.sdf
" &
NAV_PID=$!
sleep 8  # Wait for navigation stack to initialize
print_status "Navigation stack launched (PID: $NAV_PID)"
recheck "$NAV_PID" "Navigation stack"

# T4: Launch Web Video Server (camera stream)
if [ "$LAUNCH_CAMERA" = true ]; then
    print_status "Launching Web Video Server (http://localhost:8080)..."
gnome-terminal --wait --tab --title="camera" -- bash -i -c "
source \"$INSTALL_DIR/setup.bash\"
echo 'Starting Web Video Server for camera feed...'
echo 'Stream available at: http://localhost:8080'
ros2 run web_video_server web_video_server
" &
    CAMERA_PID=$!
    sleep 8
    print_status "Web Video Server launched (PID: $CAMERA_PID)"
    recheck "$CAMERA_PID" "Web Video Server"
fi

# T5: Launch RViz (optional visualization)
if [ "$LAUNCH_RVIZ" = true ]; then
    print_status "Launching RViz visualization..."
gnome-terminal --wait --tab --title="rviz" -- bash -i -c "
source \"$INSTALL_DIR/setup.bash\"
echo 'Starting RViz...'
sleep 3
cd \"$WS_ROOT\"
ros2 launch vrx_gazebo rviz.launch.py
" &
    RVIZ_PID=$!
    sleep 8
    print_status "RViz launched (PID: $RVIZ_PID)"
    recheck "$RVIZ_PID" "RViz"
fi

# T6: Launch Web Dashboard
if [ "$LAUNCH_DASHBOARD" = true ]; then
    print_status "Launching Web Dashboard (http://localhost:8000)..."
gnome-terminal --wait --tab --title="dashboard" -- bash -i -c "
cd \"$WS_ROOT/src/uvautoboat/web_dashboard/vostok1\"
echo 'Starting Web Dashboard HTTP server...'
echo 'Dashboard available at: http://localhost:8000'
echo 'Note: Wait for GPS to initialize (~10-30s) before opening dashboard'
python3 -m http.server 8000
" &
    DASHBOARD_PID=$!
    sleep 8
    print_status "Web Dashboard launched (PID: $DASHBOARD_PID)"
fi

# Open browser if requested
if [ "$OPEN_BROWSER" = true ] && [ "$LAUNCH_DASHBOARD" = true ]; then
    print_status "Waiting for systems to stabilize before opening browser..."
    sleep 8
    if command -v xdg-open &> /dev/null; then
        xdg-open http://localhost:8000 >/dev/null 2>&1 &
    elif command -v open &> /dev/null; then
        open http://localhost:8000 >/dev/null 2>&1 &
    elif command -v google-chrome &> /dev/null; then
        google-chrome http://localhost:8000 >/dev/null 2>&1 &
    else
        print_warning "Could not auto-open browser. Manually visit http://localhost:8000"
    fi
fi

print_header "Vostok1 System Launched Successfully!"
echo ""
echo -e "${GREEN}System Status:${NC}"
echo "  Gazebo:          http://localhost:11345 (Gazebo GUI)"
echo "  ROS Bridge:      ws://localhost:9090"
if [ "$LAUNCH_CAMERA" = true ]; then
    echo "  Camera Stream:   http://localhost:8080"
fi
if [ "$LAUNCH_DASHBOARD" = true ]; then
    echo "  Web Dashboard:   http://localhost:8000"
fi
echo ""
echo -e "${YELLOW}Quick Tips:${NC}"
echo "  - GPS takes 10-30s to initialize (be patient!)"
echo "  - Check terminal tabs for error messages"
echo "  - Use web dashboard to control and monitor the boat"
echo "  - Run 'ros2 run plan vostok1_cli generate' to create waypoints (if the web dashboard is not used)"
echo "  - Run 'ros2 run plan vostok1_cli start' to begin mission (if the web dashboard is not used)"
echo ""
echo -e "${BLUE}Useful Commands:${NC}"
echo "  Monitor GPS:        ros2 topic echo /wamv/sensors/gps/gps/fix"
echo "  Check LIDAR:        ros2 topic hz /wamv/sensors/lidars/lidar_wamv/points"
echo "  View obstacles:     ros2 topic echo /perception/obstacle_info"
echo "  List all nodes:     ros2 node list"
echo "  Kill everything:    pkill -9 gz && pkill -9 ros2 && pkill -9 rosbridge"
echo ""
echo -e "${BLUE}Press Ctrl+C in any terminal tab to stop that component${NC}"
echo -e "${BLUE}Press Ctrl+C here in this terminal to stop EVERYTHING (cleanup will kill all components)${NC}"
echo ""

# Keep script running until user interrupts (Ctrl+C triggers cleanup trap)
while true; do
    sleep 6
done
