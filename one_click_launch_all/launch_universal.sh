#!/bin/bash

# ============================================================================
# PROJET-17 — Vostok1 Complete One-Click Launch Script (Universal)
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
#   - Optional: RViz visualization
#
# Usage:
#   chmod +x launch_universal.sh
#   ./launch_universal.sh [OPTIONS]
#
# Options:
#   --world <world_name>       Gazebo world (default: sydney_regatta_smoke_wildlife)
#   --skip-rviz                Skip RViz launch
#   --skip-dashboard           Skip web dashboard launch
#   --skip-camera              Skip web video server launch
#   --no-browser               Do not open browser
#   --help                     Show this help message
#
# Prerequisites:
#   - ROS 2 Jazzy installed and sourced
#   - VRX package installed
#   - AutoBoat workspace built: colcon build --merge-install
#   - Gazebo Harmonic installed
#   - rosbridge-suite & web_video_server installed
#   - GNOME Terminal available for multi-tab launch
#
# ============================================================================

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'  # No Color

# ----------------------------------------------------------------------------
# Cleanup on exit (kills all spawned components)
# ----------------------------------------------------------------------------
cleanup() {
    echo -e "${YELLOW}Stopping all Vostok1 components...${NC}"
    pkill -9 -f "gz sim" || true
    pkill -9 -f "gzserver" || true
    pkill -9 -f "gzclient" || true
    pkill -9 -f "rosbridge_websocket" || true
    pkill -9 -f "web_video_server" || true
    pkill -9 -f "vostok1.launch.yaml" || true
    pkill -9 -f "plan/vostok1" || true
    pkill -9 -f "rviz" || true
    pkill -9 -f "http.server 8000" || true
    pkill -9 -f "web_dashboard/vostok1" || true
    # Close gnome-terminal tabs we opened
    for pid in $GAZEBO_PID $ROSBRIDGE_PID $NAV_PID $CAMERA_PID $RVIZ_PID $DASHBOARD_PID; do
        if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
        fi
    done
}
trap cleanup INT TERM

# ----------------------------------------------------------------------------
# Default options
# ----------------------------------------------------------------------------
WORLD="sydney_regatta_smoke_wildlife"
LAUNCH_RVIZ=true
LAUNCH_DASHBOARD=true
LAUNCH_CAMERA=true
OPEN_BROWSER=true

# ----------------------------------------------------------------------------
# Parse command-line arguments
# ----------------------------------------------------------------------------
while [[ $# -gt 0 ]]; do
    case $1 in
        --world) WORLD="$2"; shift 2 ;;
        --skip-rviz) LAUNCH_RVIZ=false; shift ;;
        --skip-dashboard) LAUNCH_DASHBOARD=false; shift ;;
        --skip-camera) LAUNCH_CAMERA=false; shift ;;
        --no-browser) OPEN_BROWSER=false; shift ;;
        --help)
            cat << 'EOF'
PROJET-17 Vostok1 Complete Launch Script (Universal)

Usage: ./launch_universal.sh [OPTIONS]

Options:
  --world <world_name>       Gazebo world (default: sydney_regatta_smoke_wildlife)
  --skip-rviz                Skip RViz launch
  --skip-dashboard           Skip web dashboard launch
  --skip-camera              Skip web video server launch
  --no-browser               Do not open browser
  --help                     Show this help message

Example:
  ./launch_universal.sh --world sydney_regatta_DEFAULT --skip-rviz

Worlds available:
  - sydney_regatta_smoke_wildlife
  - sydney_regatta_smoke
  - sydney_regatta_DEFAULT
EOF
            exit 0
            ;;
        *)
            echo "Unknown option: $1"; echo "Use --help for usage information"; exit 1 ;;
    esac
done

# ----------------------------------------------------------------------------
# Helper functions
# ----------------------------------------------------------------------------
print_header() { echo -e "${BLUE}========================================${NC}"; echo -e "${BLUE}$1${NC}"; echo -e "${BLUE}========================================${NC}"; }
print_status() { echo -e "${GREEN}✓${NC} $1"; }
print_warning() { echo -e "${YELLOW}⚠${NC} $1"; }
print_error() { echo -e "${RED}✗${NC} $1"; }
recheck() {
    local pid="$1"; local label="$2"; sleep 2
    if kill -0 "$pid" 2>/dev/null; then print_status "$label is running (PID: $pid)"; else print_warning "$label appears to have exited."; fi
}

# ----------------------------------------------------------------------------
# Check prerequisites
# ----------------------------------------------------------------------------
print_header "Checking Prerequisites"
command -v ros2 &> /dev/null || { print_error "ROS 2 not found. Please install and source ROS 2 Jazzy."; exit 1; }
print_status "ROS 2 found"
command -v gz &> /dev/null || { print_error "Gazebo not found. Please install Gazebo Harmonic."; exit 1; }
print_status "Gazebo found"

# ----------------------------------------------------------------------------
# Detect workspace root dynamically
# ----------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
WS_ROOT="$SCRIPT_DIR"
while [ "$WS_ROOT" != "/" ]; do
    if [ -f "$WS_ROOT/install/setup.bash" ]; then break; fi
    WS_ROOT="$(dirname "$WS_ROOT")"
done
INSTALL_DIR="$WS_ROOT/install"
if [ ! -f "$INSTALL_DIR/setup.bash" ]; then
    print_error "Workspace not built or install/setup.bash not found. Run: colcon build --merge-install"
    exit 1
fi
print_status "AutoBoat workspace found at $WS_ROOT"

# Source workspace environment
source "$INSTALL_DIR/setup.bash"
print_status "Sourced AutoBoat environment"

# ----------------------------------------------------------------------------
# Launch system
# ----------------------------------------------------------------------------
print_header "Launching Vostok1 Complete System"
echo "World: $WORLD"
echo "RViz: $([ "$LAUNCH_RVIZ" = true ] && echo 'Yes' || echo 'No')"
echo "Dashboard: $([ "$LAUNCH_DASHBOARD" = true ] && echo 'Yes' || echo 'No')"
echo "Camera Stream: $([ "$LAUNCH_CAMERA" = true ] && echo 'Yes' || echo 'No')"
echo "Auto-open Browser: $([ "$OPEN_BROWSER" = true ] && echo 'Yes' || echo 'No')"
echo ""

# Cleanup old processes
print_status "Cleaning up old processes..."
pkill -9 -f "gz sim" &> /dev/null || true
pkill -9 -f "gzserver" &> /dev/null || true
pkill -9 -f "gzclient" &> /dev/null || true
pkill -9 -f "rosbridge" &> /dev/null || true
pkill -9 -f "web_video_server" &> /dev/null || true
sleep 4

# --- Launch Gazebo ---
print_status "Launching Gazebo (Sydney Regatta - $WORLD)..."
gnome-terminal --tab --title="gazebo" -- bash -i -c "
source \"$INSTALL_DIR/setup.bash\"
ros2 launch vrx_gz competition.launch.py world:=$WORLD
" &
GAZEBO_PID=$!
sleep 28
recheck "$GAZEBO_PID" "Gazebo"

# --- Launch ROS Bridge ---
print_status "Launching ROS Bridge..."
gnome-terminal --tab --title="rosbridge" -- bash -i -c "
source \"$INSTALL_DIR/setup.bash\"
ros2 launch rosbridge_server rosbridge_websocket_launch.xml delay_between_messages:=0.0
" &
ROSBRIDGE_PID=$!
sleep 8
recheck "$ROSBRIDGE_PID" "ROS Bridge"

# --- Launch Navigation Stack ---
print_status "Launching Navigation Stack..."
gnome-terminal --tab --title="navigation" -- bash -i -c "
source \"$INSTALL_DIR/setup.bash\"
ros2 launch \"$WS_ROOT/src/uvautoboat/launch/vostok1.launch.yaml\"
" &
NAV_PID=$!
sleep 8
recheck "$NAV_PID" "Navigation stack"

# --- Launch Web Video Server ---
if [ "$LAUNCH_CAMERA" = true ]; then
    print_status "Launching Web Video Server..."
    gnome-terminal --tab --title="camera" -- bash -i -c "
source \"$INSTALL_DIR/setup.bash\"
ros2 run web_video_server web_video_server
" &
    CAMERA_PID=$!
    sleep 8
    recheck "$CAMERA_PID" "Web Video Server"
fi

# --- Launch RViz ---
if [ "$LAUNCH_RVIZ" = true ]; then
    print_status "Launching RViz..."
    gnome-terminal --tab --title="rviz" -- bash -i -c "
source \"$INSTALL_DIR/setup.bash\"
ros2 launch vrx_gazebo rviz.launch.py
" &
    RVIZ_PID=$!
    sleep 8
    recheck "$RVIZ_PID" "RViz"
fi

# --- Launch Web Dashboard ---
if [ "$LAUNCH_DASHBOARD" = true ]; then
    print_status "Launching Web Dashboard..."
    gnome-terminal --tab --title="dashboard" -- bash -i -c "
cd \"$WS_ROOT/src/uvautoboat/web_dashboard/vostok1\"
python3 -m http.server 8000
" &
    DASHBOARD_PID=$!
    sleep 8
    recheck "$DASHBOARD_PID" "Web Dashboard"
fi

# --- Open browser ---
if [ "$OPEN_BROWSER" = true ] && [ "$LAUNCH_DASHBOARD" = true ]; then
    sleep 8
    if command -v xdg-open &> /dev/null; then
        xdg-open http://localhost:8000 >/dev/null 2>&1 &
    fi
fi

print_header "Vostok1 System Launched Successfully!"
echo "  Gazebo GUI:       http://localhost:11345"
echo "  ROS Bridge:       ws://localhost:9090"
[ "$LAUNCH_CAMERA" = true ] && echo "  Camera Stream:    http://localhost:8080"
[ "$LAUNCH_DASHBOARD" = true ] && echo "  Web Dashboard:    http://localhost:8000"

# ----------------------------------------------------------------------------
# Keep script running
# ----------------------------------------------------------------------------
while true; do sleep 6; done
