# Robust Avoidance Stack

This folder contains the **Robust Avoidance** system - a comprehensive all-in-one navigation stack for autonomous boat control with LiDAR-based obstacle avoidance.

## Overview

The Robust Avoidance stack (formerly `all_in_one_stack`) is a monolithic ROS 2 node that combines:

- **GPS/IMU localization** (local ENU frame)
- **Pose filtering** (noise reduction)
- **Path planning** (A* detour planning)
- **Obstacle avoidance** (LiDAR-based VFH)
- **Thrust control** (PID heading control)
- **Stuck recovery** (automatic backup and turn)

## Architecture

```bash
┌─────────────────┐      ┌──────────────┐      ┌────────────────────┐
│  GPS/IMU Data   │─────▶│ gps_imu_pose │─────▶│   pose_filter      │
│ (VRX Sensors)   │      │   (plan)     │      │    (plan)          │
└─────────────────┘      └──────────────┘      └─────────┬──────────┘
                                                          │
                                                          ▼ /wamv/pose_filtered
┌─────────────────┐                              ┌────────────────────┐
│  LiDAR 3D Scan  │─────────────────────────────▶│ robust_avoidance   │
│ (VRX Sensors)   │                              │    (control)       │
└─────────────────┘                              └─────────┬──────────┘
                                                          │
                                                          ▼ Thruster Commands
                                                  ┌────────────────────┐
                                                  │  WAM-V Thrusters   │
                                                  └────────────────────┘
```

## Package Organization

| Component | Package | Executable | Description |
|-----------|---------|-----------|-------------|
| GPS/IMU → Pose | `plan` | `gps_imu_pose` | Converts GPS/IMU to local ENU pose |
| Pose Filter | `plan` | `pose_filter` | Filters noisy pose data |
| Main Controller | `control` | `robust_avoidance` | All-in-one navigation stack |

## Files in This Folder

### Documentation

- **[readme_for_all_in_one_bringup_launch.md](readme_for_all_in_one_bringup_launch.md)** - Detailed launch configuration guide
- **readme_robust_avoidance.md** (this file) - Overview and quick reference

### Utility Scripts

- **[diagnose_boat.sh](diagnose_boat.sh)** - System diagnostic tool
  - Checks if nodes are running
  - Verifies topics are publishing
  - Sends test goals
  - Monitors thruster commands

- **[monitor.sh](monitor.sh)** - Real-time monitoring dashboard
  - Live system status display
  - Position tracking
  - Thruster status
  - LiDAR frequency
  - Navigation path info

- **[quick.sh](quick.sh)** - Quick system validation
  - ROS 2 installation check
  - Workspace compilation check
  - Critical file existence check
  - Launch file syntax validation

## Quick Start

### 1. Build the Packages

```bash
cd /home/ghostzero/seal_ws
colcon build --packages-select control plan
source install/setup.bash
```

### 2. Launch the System

```bash
ros2 launch uvautoboat robust_avoidance.launch.yaml
```

### 3. Send a Test Goal

```bash
ros2 topic pub /planning/goal geometry_msgs/PoseStamped \
  "{header: {frame_id: 'world'}, pose: {position: {x: 100.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}" --once
```

### 4. Monitor the System

In a separate terminal:

```bash
cd /home/ghostzero/seal_ws/src/uvautoboat/robust_avoidance
./monitor.sh
```

## Diagnostics

### Run Quick Diagnostics

```bash
cd /home/ghostzero/seal_ws/src/uvautoboat/robust_avoidance
./quick.sh
```

### Run Full Diagnostics

```bash
cd /home/ghostzero/seal_ws/src/uvautoboat/robust_avoidance
./diagnose_boat.sh
```

## Key Features

### 1. Obstacle Avoidance

- **VFH (Vector Field Histogram)**: 360° LiDAR-based steering
- **Adaptive Sectors**: Front (30°), Side (60°) obstacle detection
- **Clearance Margin**: 4m safety buffer before resuming navigation

### 2. Stuck Recovery

- **Detection**: 8s timeout with 0.1m minimum movement
- **Recovery Sequence**:
  1. Reverse for 3-6 seconds
  2. Turn for 3 seconds
  3. Resume navigation

### 3. Path Planning

- **A* Detour**: Automatic obstacle detour planning
- **Dynamic Replanning**: Updates when obstacles detected
- **Waypoint Sequencing**: Automatic multi-goal navigation

### 4. Thrust Control

- **Base Forward Thrust**: 320N
- **PID Heading Control**: Kp=600.0
- **Adaptive Speed**: Slowdown near obstacles and waypoints

## Configuration

Launch file: [../launch/robust_avoidance.launch.yaml](../launch/robust_avoidance.launch.yaml)

Key parameters:

```yaml
forward_thrust: 320.0          # Base thrust (N)
kp_yaw: 600.0                  # Heading gain
obstacle_slow_dist: 10.0       # Slowdown distance (m)
obstacle_stop_dist: 7.0        # Hard avoid distance (m)
stuck_timeout: 8.0             # Stuck detection (s)
vfh_enabled: true              # Enable VFH steering
```

## Troubleshooting

### Boat Not Moving

1. Check if nodes are running: `ros2 node list`
2. Verify pose data: `ros2 topic echo /wamv/pose_filtered`
3. Check thrusters: `ros2 topic echo /wamv/thrusters/left/thrust`
4. Send test goal (see Quick Start #3)

### False Obstacle Detections

- Adjust `cloud_z_min` and `cloud_z_max` in launch file
- Increase `min_range_filter` to ignore close objects
- Check LiDAR topic: `ros2 topic echo /wamv/sensors/lidars/lidar_wamv_sensor/scan`

### Stuck in Avoidance Mode

- Reduce `full_clear_distance` (default: 20.0m)
- Increase `avoid_clear_margin` for earlier exit
- Check VFH parameters: `vfh_bin_deg`, `vfh_block_dist`

## Related Systems

- **Vostok1 Modular System**: [../launch/vostok1.launch.yaml](../launch/vostok1.launch.yaml)
  - OKO (Perception)
  - SPUTNIK (Planning)
  - BURAN (Control)
  - Includes LiDAR smoke detection

- **Legacy Atlantis System**: [../legacy/](../legacy/)
  - Original all-in-one stack
  - Python launch files
  - Deprecated but functional

## Support

For questions or issues:

- Check [readme_for_all_in_one_bringup_launch.md](readme_for_all_in_one_bringup_launch.md)
- Review main project [README.md](../README.md)
- Run diagnostic scripts in this folder

---

**Last Updated**: 14.12.2025
**Version**: Robust Avoidance v1.0
**Maintainer**: ghostzero <yinpuchen0@gmail.com>
