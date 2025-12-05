# AutoBoat — Autonomous Navigation for Unmanned Surface Vehicles

[![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-blue)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange)](https://gazebosim.org/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
![Status](https://img.shields.io/badge/Status-Active-green)

> **PROJET-17 — Autonomous Navigation System for the Virtual RobotX (VRX) Competition**  
> A comprehensive ROS 2 framework for GPS-based waypoint navigation with 3D LIDAR obstacle avoidance

---

## 📋 Table of Contents

1. [Abstract](#abstract)
2. [Project Overview](#project-overview)
   - [Project Status](#project-status)
   - [Project Structure](#project-structure)
   - [Additional Documentation](#additional-documentation)
   - [System Requirements](#system-requirements)
   - [Key Features](#key-features)
3. [Background Concepts](#background-concepts-for-new-users)
4. [Installation](#installation)
5. [Quick Start](#quick-start)
6. [Coordinate System](#coordinate-system)
7. [System Architecture](#system-architecture)
8. [Usage Guide](#usage-guide)
9. [Web Dashboard](#web-dashboard)
10. [Smart Anti-Stuck System (SASS)](#smart-anti-stuck-system-sass)
11. [Waypoint Skip Strategy](#waypoint-skip-strategy)
12. [Terminal Mission Control (CLI)](#terminal-mission-control)
13. [Technical Documentation](#technical-documentation)
14. [Troubleshooting](#troubleshooting)
15. [Command Cheatsheet](#command-cheatsheet)
16. [Future Roadmap](#future-roadmap)
17. [Contributing](#contributing)
18. [References](#references)
19. [Acknowledgments](#acknowledgments)
20. [License](#license)

---

## Abstract

AutoBoat is an autonomous navigation system for unmanned surface vehicles (USVs) developed for the Virtual RobotX ([VRX](https://github.com/osrf/vrx)) competition. The system integrates advanced path planning, real-time obstacle avoidance, and precise trajectory tracking algorithms optimized for the WAM-V maritime platform. Built on **[ROS 2 Jazzy](https://docs.ros.org/en/jazzy/)** and **[Gazebo Harmonic](https://gazebosim.org/docs/harmonic/)**, the framework provides a robust foundation for autonomous maritime navigation in simulated environments.

The project implements a hierarchical autonomous navigation framework combining perception, planning, and control subsystems to enable intelligent waypoint navigation while dynamically responding to environmental constraints. By processing sensor data streams and mission objectives in real-time, the architecture generates collision-free trajectories that account for static obstacles, operational boundaries, and vehicle dynamics, ensuring safe and efficient autonomous operation.

**Key Contributions:**

- **Vostok1 Navigation System**: Integrated autonomous navigation with 3D LIDAR perception
- **Modular Architecture**: Distributed nodes (OKO-SPUTNIK-BURAN) for flexible deployment
- **Smart Anti-Stuck System (SASS)**: Intelligent recovery with Kalman-filtered drift compensation
- **Web Dashboard**: Real-time monitoring with better visualization
- **Waypoint Skip Strategy**: Automatic skip for blocked waypoints ensuring mission completion

---

## Project Overview

### Project Status

| Phase | Description | Status |
|:------|:------------|:------:|
| Phase 1 | Architecture & MVP | ✅ 100% |
| Phase 2 | Obstacle Avoidance | ✅ 100% |
| Phase 3 | Coverage & Search | ⏸️ Planned |
| Phase 4 | Integration & Testing | 🔄 80% |

See [Board.md](Board.md) for detailed milestones and progress tracking.

### Project Structure

```text
uvautoboat/
├── control/                    # ROS 2 Control Package
│   └── control/
│       ├── atlantis_controller.py   # Integrated controller (Atlantis team)
│       ├── buran_controller.py      # Modular controller (BURAN)
│       ├── keyboard_teleop.py       # Manual control interface
│       ├── lidar_obstacle_avoidance.py
│       └── gps_imu_pose.py
├── plan/                       # ROS 2 Planning Package
│   ├── brain/
│   │   ├── vostok1.py               # Integrated navigation system
│   │   ├── oko_perception.py        # 3D LIDAR perception (OKO)
│   │   ├── sputnik_planner.py       # Waypoint planner (SPUTNIK)
│   │   ├── atlantis_planner.py      # Alternative planner (Atlantis)
│   │   ├── vostok1_cli.py           # Terminal mission control
│   │   └── lidar_obstacle_avoidance.py
│   └── launch/
│       └── vostok1_modular_navigation.launch.py
├── launch/                     # Top-level launch files
│   ├── vostok1.launch.yaml         # Modular system configuration
│   └── atlantis.launch.yaml        # Atlantis system configuration
├── web_dashboard/              # Real-time monitoring interfaces
│   ├── vostok1/                     # Vostok1 dashboard
│   └── atlantis/                    # Atlantis dashboard
├── test_environment/           # Custom Gazebo worlds and models
│   ├── sydney_regatta_DEFAULT.sdf  # Original VRX world (reference/template)
│   ├── sydney_regatta_custom.sdf   # Custom world with obstacles
│   ├── wamv_3d_lidar.xacro         # Default 3D LIDAR config (backup reference)
│   └── cardboardbox/                # Custom obstacle model
├── images/                     # Documentation images
├── Board.md                    # Development progress tracking
└── README.md                   # This file
```

> **Note:** The `test_environment/` folder contains reference copies of VRX default files:
>
> 1. **Quick reference** - No need to navigate through VRX package folders
> 2. **Template base** - Starting point for creating custom worlds with obstacles, buoys, etc.
> 3. **Parameter backup** - The `wamv_3d_lidar.xacro` contains default LIDAR parameters. If you modify your LIDAR config and need to reset, refer to this file.

### Additional Documentation

| Document | Description |
|:---------|:------------|
| [LIDAR_OBSTACLE_AVOIDANCE_GUIDE.md](LIDAR_OBSTACLE_AVOIDANCE_GUIDE.md) | Detailed LIDAR processing and obstacle detection |
| [MISSION_CONTROL_GUIDE.md](MISSION_CONTROL_GUIDE.md) | Mission control interface usage |
| [LAUNCH_YAML_GUIDE.md](LAUNCH_YAML_GUIDE.md) | YAML launch file configuration |
| [CODE_REVIEW.md](CODE_REVIEW.md) | Code review notes and standards |

### System Requirements

| Component | Minimum | Recommended |
|:----------|:--------|:------------|
| OS | Ubuntu 24.04 LTS | Ubuntu 24.04 LTS |
| RAM | 8 GB | 16 GB |
| Storage | 40 GB | 60 GB |
| Python | 3.10+ | 3.12 |
| GPU | Integrated | Dedicated (for Gazebo) |

### Key Features

| Feature | Description |
|:--------|:------------|
| **Autonomous Navigation** | GPS-based waypoint following with lawnmower pattern generation |
| **3D Obstacle Avoidance** | Real-time LIDAR point cloud processing with sector analysis |
| **Differential Thrust** | Independent left/right thruster control with PID heading |
| **Smart Anti-Stuck (SASS)** | 4-phase escape with no-go zones and drift compensation |
| **Waypoint Skip** | Automatic skip for blocked waypoints after timeout |
| **Go Home** | One-click return to spawn point |
| **Web Dashboard** | Real-time monitoring with interactive map |
| **Bilingual Interface** | French/English terminal output |

---

## Background Concepts for New Users

If you're new to ROS 2 or autonomous navigation, this section provides essential background knowledge.

### What is ROS 2?

**ROS 2 (Robot Operating System 2)** is a framework for building robot software. Think of it as a communication system that allows different parts of a robot (sensors, planning algorithms, controllers) to talk to each other.

**Key ROS 2 Concepts:**

| Concept | Description | Example |
|:--------|:------------|:--------|
| **Node** | Independent program for a specific task | `vostok1_node` (navigation) |
| **Topic** | Named channel for messages | `/wamv/sensors/gps/gps/fix` |
| **Message** | Data structure sent over topics | GPS coordinates, thrust commands |
| **Package** | Collection of related nodes | `plan`, `control` |
| **Parameter** | Runtime-configurable values | PID gains, speed limits |

### GPS Navigation

The WAM-V uses GPS for absolute position tracking. The system converts GPS coordinates to local meters using **equirectangular projection**:

```text
Local X = (latitude - start_lat) × Earth_radius
Local Y = (longitude - start_lon) × Earth_radius × cos(start_lat)
```

**How it works:**

1. First GPS fix becomes local origin (0, 0)
2. Waypoints defined in local meters from origin
3. Earth radius: 6,371,000 meters

### 3D LIDAR Point Cloud Processing

**LIDAR** (Light Detection and Ranging) uses laser pulses to create a 3D map of the environment. The WAM-V's 3D LIDAR returns thousands of points per scan.

**Processing Pipeline (OKO v2.0):**

| Step | Filter | Purpose |
|:-----|:-------|:--------|
| 1 | Height: -15m to +10m | Exclude water/sky, catch lake banks |
| 2 | Range: 5m to 50m | Ignore dock at spawn, focus on obstacles |
| 3 | Water Plane Removal | Filter water surface reflections (5th percentile Z) |
| 4 | Sector Analysis | Front/Left/Right clearance (adaptive width) |
| 5 | Temporal Filtering | Require 3/5 scans to confirm detection |
| 6 | Obstacle Clustering | Group points into distinct obstacles |
| 7 | Gap Detection | Find passable gaps between obstacles |
| 8 | Velocity Estimation | Track moving obstacles over time |

**Enhanced Features (OKO v2.0):**

| Feature | Description |
|:--------|:------------|
| **Temporal Filtering** | 5-scan history, requires 3/5 detections to confirm (reduces flickering) |
| **Distance-Weighted Urgency** | Score 0.0 (safe) to 1.0 (critical) for smoother control |
| **Obstacle Clustering** | Groups nearby points (2m eps) to identify individual obstacles |
| **Gap Detection** | Finds passable gaps (>3m width) between obstacles |
| **Adaptive Sectors** | Front sector width adjusts based on target heading |
| **Water Plane Removal** | Estimates water surface Z, filters reflections |
| **Velocity Estimation** | Tracks obstacles across frames to detect movement |

**Enhanced `/perception/obstacle_info` JSON:**

```json
{
  "obstacle_detected": true,
  "min_distance": 8.5,
  "front_clear": 10.2,
  "left_clear": 45.0,
  "right_clear": 12.3,
  "is_critical": false,
  "front_urgency": 0.45,
  "left_urgency": 0.0,
  "right_urgency": 0.35,
  "overall_urgency": 0.45,
  "clusters": [{"x": 8.2, "y": 1.5, "size": 25, "distance": 8.5, "angle_deg": 10.3}],
  "gaps": [{"angle_deg": -25.0, "width": 5.2, "distance": 15.0}],
  "moving_obstacles": [{"id": "obs_0", "vx": 0.5, "vy": 0.1, "speed": 0.51}],
  "water_plane_z": -2.8,
  "temporal_confidence": 1.0
}
```

### IMU Heading

The **IMU** (Inertial Measurement Unit) provides orientation via quaternion. Yaw (heading) is extracted:

```text
siny_cosp = 2 × (w × z + x × y)
cosy_cosp = 1 - 2 × (y² + z²)
yaw = atan2(siny_cosp, cosy_cosp)
```

### Differential Thrust Control

The WAM-V uses two independent thrusters:

| Maneuver | Left | Right | Result |
|:---------|:-----|:------|:-------|
| Forward | +500 | +500 | Straight ahead |
| Reverse | -500 | -500 | Straight back |
| Turn Left | +200 | +500 | Gradual left |
| Spin Left | -500 | +500 | Rotate in place |

**Thrust Range:** -1000 to +1000 Newtons

### PID Control

The **PID controller** provides smooth heading adjustments:

```text
error = target_heading - current_heading
correction = Kp × error + Ki × ∫error + Kd × d(error)/dt
```

| Parameter | Default | Effect |
|:----------|:--------|:-------|
| Kp | 400.0 | Proportional response (fast correction) |
| Ki | 20.0 | Integral accumulation (eliminate steady-state error) |
| Kd | 100.0 | Derivative damping (prevent overshoot) |

### Simulation Time (use_sim_time)

When running in Gazebo, time moves differently than real-world time. The `use_sim_time` parameter synchronizes ROS nodes with simulation time.

 Currently only `control/launch/all_in_one_bringup.launch.py` sets this flag for its nodes; other launches run on wall time unless updated.

---

## Installation

### Prerequisites

- **ROS 2 Jazzy**: [Installation Guide](https://docs.ros.org/en/jazzy/Installation.html)
- **Gazebo Harmonic**: [Installation Guide](https://gazebosim.org/docs/harmonic/install_ubuntu/)
- **VRX Simulation**: [GitHub Repository](https://github.com/osrf/vrx)
- **rosbridge-suite**: Required for web dashboard

### Step-by-Step Installation

```bash
# 1. Create workspace
mkdir -p ~/seal_ws/src && cd ~/seal_ws/src

# 2. Clone repositories
git clone https://github.com/Erk732/uvautoboat.git
git clone https://github.com/osrf/vrx.git

# 3. Install dependencies
cd ~/seal_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y

# 4. Install rosbridge for web dashboard
sudo apt install ros-jazzy-rosbridge-suite

# 5. Build workspace
colcon build --merge-install

# 6. Source environment
source ~/seal_ws/install/setup.bash

# 7. (Recommended) Add to ~/.bashrc for auto-sourcing
echo "source ~/seal_ws/install/setup.bash" >> ~/.bashrc
```

---

## Quick Start

### Two-Terminal Quick Start

**Terminal 1** — Launch Simulation:

```bash
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
```

**Terminal 2** — Run Navigation (choose one):

```bash
# Integrated Vostok1 (recommended)
ros2 run plan vostok1

# Modular system (configurable)
ros2 launch ~/seal_ws/src/uvautoboat/launch/vostok1.launch.yaml
```

### Four-Terminal Full Setup (with Web Dashboard)

| Terminal | Command | Purpose |
|:---------|:--------|:--------|
| **T1** | `ros2 launch vrx_gz competition.launch.py world:=sydney_regatta` | Gazebo simulation |
| **T2** | `ros2 launch rosbridge_server rosbridge_websocket_launch.xml delay_between_messages:=0.0` | WebSocket bridge |
| **T3** | `ros2 run plan vostok1` | Vostok1 navigation |
| **T4** | `cd ~/seal_ws/src/uvautoboat/web_dashboard/vostok1 && python3 -m http.server 8000` | Web server |

> **Important:** The `delay_between_messages:=0.0` parameter is required for ROS 2 Jazzy due to a parameter type bug.

This starts a WebSocket server on `ws://localhost:9090`.

> **Note:** T4 must run in a separate terminal — it's a simple HTTP server, not a ROS node.

Then open: **<http://localhost:8000>**

### Expected Output

[![VRX Simulation](images/sydney_regatta_gzsim.png)](https://vimeo.com/851696025)

*Sydney Regatta simulation environment in Gazebo. Credit: [VRX Project](https://github.com/osrf/vrx/wiki/running_vrx_tutorial)*

---

## Coordinate System

Understanding the coordinate system is essential for working with VRX simulation.

![3D Cartesian Coordinate System](images/3d_coordinate_system.jpg)

*3D Cartesian coordinate system. [Primalshell](https://commons.wikimedia.org/wiki/File:3D_Cartesian_Coodinate_Handedness.jpg), [CC BY-SA 4.0](https://creativecommons.org/licenses/by-sa/4.0/)*

### Position (x, y, z)

| Axis | Direction | Maritime Term |
|:-----|:----------|:--------------|
| **X** | Forward / Backward | Ahead / Astern |
| **Y** | Left / Right | Port / Starboard |
| **Z** | Up / Down | Above / Below waterline |

### Orientation (Roll, Pitch, Yaw)

| Rotation | Axis | Description | Maritime Term |
|:---------|:-----|:------------|:--------------|
| **Roll** | X-axis | Side-to-side tilt | Heel to port/starboard |
| **Pitch** | Y-axis | Front-to-back tilt | Bow up / Bow down |
| **Yaw** | Z-axis | Horizontal rotation | Turn to port/starboard |

**Common Values:**

| Pose | Roll | Pitch | Yaw | Result |
|:-----|:----:|:-----:|:---:|:-------|
| Default | 0 | 0 | 0 | Upright, facing +X |
| 90° right turn | 0 | 0 | -1.57 | Facing +Y |
| 90° left turn | 0 | 0 | 1.57 | Facing -Y |
| 180° turn | 0 | 0 | 3.14 | Facing -X |

---

## System Architecture

### Navigation Systems Comparison

AutoBoat provides multiple navigation systems:

| Aspect | Vostok1 (Integrated) | Modular (TNO) | Atlantis (Control Group) |
|:-------|:---------------------|:--------------|:-------------------------|
| **Approach** | Self-contained node | Distributed nodes | Integrated controller |
| **LIDAR** | 3D PointCloud2 | 3D PointCloud2 | 3D PointCloud2  |
| **Detection** | Full 3D volume | Full 3D volume | 3D Section Analysis |
| **Control** | PID heading | PID (configurable) | PID heading |
| **Monitoring** | Terminal + Web | Terminal (bilingual) | Web Dashboard |
| **Anti-Stuck** | SASS v2.0 | SASS v2.0 | Adaptive Escape with SASS (Work in progress) |
| **Best For** | Production use | Custom tuning | Robust Path Validation |

### Modular Architecture (TNO Style)

The modular system uses below programs to work:

| Node | Name | Function |
|:-----|:-----|:---------|
| **OKO** | `oko_perception` | 3D LIDAR obstacle detection |
| **SPUTNIK** | `sputnik_planner` | GPS waypoint planning |
| **BURAN** | `buran_controller` | PID heading control + SASS |

| Component | Script Name | Function |
|:-----|:-----|:---------|
| **Perception** | `lidar_obstacle_avoidance.py` | Zero-latency 3D PointCloud2 processing & Sector Analysis |
| **Planner** | `atlantis_planner` | checks between waypoints |
| **Controller** | `atlantis_controller` | Smart Anti Stuck System|

The additional feauture for Atlantis method is that' unlike distributed architectures, Atlantis embeds the LidarObstacleDetector class directly within the controller loop. This ensures zero-latency obstacle reaction, allowing the boat to make steering decisions in the exact same millisecond that the Lidar scan is received.

### Modular Topic Flow Diagram

Detailed ROS 2 topic connections between the modular nodes:

```text
┌─────────────────────────────────────────────────────────────────────────────────┐
│                          VOSTOK1 MODULAR SYSTEM                                 │
├─────────────────────────────────────────────────────────────────────────────────┤
│                                                                                 │
│  ┌──────────┐                                                                   │
│  │   OKO    │ /perception/obstacle_info ────────────► BURAN (obstacle_callback) │
│  │ (LiDAR)  │                           ────────────► SPUTNIK (obstacle_callback)│
│  └──────────┘                                                                   │
│       ▲                                                                         │
│       │ /wamv/sensors/lidars/lidar_wamv/points                                  │
│                                                                                 │
│  ┌──────────┐  /planning/current_target ────────────► BURAN (target_callback)   │
│  │ SPUTNIK  │  /planning/mission_status ────────────► BURAN (mission_status_cb) │
│  │(Planner) │  /sputnik/config          ────────────► BURAN (sputnik_config_cb) │
│  └──────────┘                                                                   │
│       ▲  ▲                                                                      │
│       │  │ /planning/detour_request ◄────────────────── BURAN (pub_detour)      │
│       │  │                                                                      │
│       │  └─ /wamv/sensors/gps/gps/fix                                           │
│       └──── /sputnik/mission_command ◄───────────────── CLI / Dashboard         │
│                                                                                 │
│  ┌──────────┐  /wamv/thrusters/left/thrust  ────────► Gazebo Simulator          │
│  │  BURAN   │  /wamv/thrusters/right/thrust ────────► Gazebo Simulator          │
│  │(Control) │  /buran/status                ────────► Web Dashboard             │
│  └──────────┘                                                                   │
│       ▲  ▲                                                                      │
│       │  └─ /wamv/sensors/imu/imu/data                                          │
│       └──── /vostok1/set_config ◄────────────────────── Dashboard (runtime PID) │
│                                                                                 │
│  External Control:                                                              │
│  ├─ /sputnik/set_config         ◄─────────── Dashboard (waypoint radius, etc.) │
│  └─ /sputnik/mission_command    ◄─────────── CLI: start, pause, stop, go_home  │
└─────────────────────────────────────────────────────────────────────────────────┘
```
### Atlantis Topic Flow Diagram

Detailed ROS 2 connections for the Atlantis architecture. Note the direct LIDAR ingestion by both nodes:

```text
┌─────────────────────────────────────────────────────────────────────────────────┐
│                          ATLANTIS INTEGRATED SYSTEM                             │
├─────────────────────────────────────────────────────────────────────────────────┤
│                                                                                 │
│               (Shared Library: lidar_obstacle_avoidance.py)                     │
│               ┌───────────────────────────────────────────┐                     │
│               │                                           │                     │
│  ┌──────────┐ ▼ /wamv/sensors/lidars/lidar_wamv.../points ▼    ┌──────────┐     │
│  │ PLANNER  │◄───────────────────────┬───────────────────────► │CONTROLLER│     │
│  │(Atlantis)│                        │                         │(Atlantis)│     │
│  └──────────┘                        │                         └──────────┘     │
│       │                              │                              ▲  ▲        │
│       │ /atlantis/path ──────────────┼──────────────────────────────┘  │        │
│       │                              │                                 │        │
│       ▼                              │   /wamv/sensors/gps/gps/fix ────┘        │
│  /atlantis/waypoints                 │   /wamv/sensors/imu/imu/data ───┐        │
│  /atlantis/obstacle_map              │                                 │        │
│       │                              │                                 │        │
│       ▼                              │                                 │        │
│  Web Dashboard                       │  /wamv/thrusters/left/thrust ───┼──► GZ  │
│                                      │  /wamv/thrusters/right/thrust ──┼──► GZ  │
│                                      │                                 │        │
│  External Control:                   │  /atlantis/mission_status ──────► DASH   │
│  ├─ /atlantis/replan    ───────────► │  /atlantis/anti_stuck_status ───► DASH   │
│  └─ /atlantis/start     ───────────► │                                          │
└──────────────────────────────────────┴──────────────────────────────────────────┘


**JSON Message Formats:**

| Topic | Format |
|:------|:-------|
| `/perception/obstacle_info` | `{obstacle_detected, min_distance, front_clear, left_clear, right_clear, is_critical}` |
| `/planning/current_target` | `{current_position, target_waypoint, distance_to_target, waypoint_index, target_heading}` |
| `/planning/mission_status` | `{state, current_waypoint, total_waypoints, progress_percent, elapsed_time}` |
| `/planning/detour_request` | `{type, x, y}` |

### Data Flow Diagram

```text
┌─────────────────────────────────────────────────────────────┐
│                    SENSORS (Gazebo)                         │
├─────────────────┬─────────────────┬─────────────────────────┤
│   GPS           │      IMU        │        3D LIDAR         │
│ (NavSatFix)     │    (Imu)        │    (PointCloud2)        │
└────────┬────────┴────────┬────────┴────────────┬────────────┘
         │                 │                     │
         └─────────────────┼─────────────────────┘
                           ▼
              ┌────────────────────────┐
              │   Navigation System    │
              │   ──────────────────   │
              │   • Position tracking  │
              │   • Heading control    │
              │   • Obstacle avoidance │
              │   • Waypoint planning  │
              │   • SASS recovery      │
              └───────────┬────────────┘
                          │
            ┌─────────────┴─────────────┐
            ▼                           ▼
    Left Thruster              Right Thruster
    (-1000 to +1000)           (-1000 to +1000)
```

### Continuous Obstacle Avoidance Loop

The obstacle avoidance runs **continuously** - not as a one-time decision. The boat constantly scans, evaluates, and adjusts:

```text
┌─────────────────────────────────────────────────────────────────┐
│                      PERCEPTION (OKO)                           │
│           LiDAR scans 360° continuously (~10-20 Hz)             │
│                            ↓                                    │
│      ┌─────────────────────────────────────────────┐            │
│      │  For each scan:                             │            │
│      │  1. Filter points (height, range)           │            │
│      │  2. Check FRONT sector → min distance       │            │
│      │  3. Check LEFT sector  → min distance       │            │
│      │  4. Check RIGHT sector → min distance       │            │
│      │  5. Publish obstacle_info                   │            │
│      └─────────────────────────────────────────────┘            │
└─────────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────────┐
│                      CONTROL (BURAN)                            │
│           Receives obstacle_info every ~100ms                   │
│                            ↓                                    │
│      ┌─────────────────────────────────────────────┐            │
│      │  Decision Logic (runs every control loop):  │            │
│      │                                             │            │
│      │  IF front_clear AND distance > safe:        │            │
│      │     → Continue toward waypoint              │            │
│      │                                             │            │
│      │  IF obstacle detected:                      │            │
│      │     → Slow down (obstacle_slow_factor)      │            │
│      │     → Check which side is clearer           │            │
│      │     → Turn toward clearer side              │            │
│      │                                             │            │
│      │  IF critical distance:                      │            │
│      │     → STOP immediately                      │            │
│      │     → Initiate SASS (anti-stuck) if stuck   │            │
│      └─────────────────────────────────────────────┘            │
└─────────────────────────────────────────────────────────────────┘
```

**Real-Time Decision Example:**

| Time | Front | Left | Right | Action |
|:-----|:------|:-----|:------|:-------|
| 0.0s | 15m | 50m | 8m | ⚠️ Obstacle ahead → Turn LEFT (clearer) |
| 0.1s | 20m | 45m | 10m | 🔄 Continue LEFT turn |
| 0.2s | 35m | 40m | 15m | 📐 Front clearing, reduce turn |
| 0.3s | 50m | 50m | 25m | ✅ CLEAR! Resume to waypoint |

> **Key Point:** The boat is **always** scanning, calculating distances, and adjusting - even while dodging an obstacle. This enables navigation through complex obstacle fields.

### ROS 2 Topics

#### Sensor Inputs

| Topic | Type | Description |
|:------|:-----|:------------|
| `/wamv/sensors/gps/gps/fix` | `NavSatFix` | GPS coordinates |
| `/wamv/sensors/imu/imu/data` | `Imu` | Orientation quaternion |
| `/wamv/sensors/lidars/.../points` | `PointCloud2` | 3D LIDAR data |

#### Control Outputs

| Topic | Type | Description |
|:------|:-----|:------------|
| `/wamv/thrusters/left/thrust` | `Float64` | Left thruster (-1000 to +1000 N) |
| `/wamv/thrusters/right/thrust` | `Float64` | Right thruster (-1000 to +1000 N) |

#### Dashboard Topics (Vostok1/Modular)

| Topic | Description |
|:------|:------------|
| `/vostok1/mission_status` | Mission state (JSON) |
| `/vostok1/config` | Current parameters |
| `/planning/mission_status` | Modular mission state |
| `/control/anti_stuck_status` | SASS status |
| `/perception/obstacle_info` | Obstacle detection |

---

## Usage Guide

### Option A: Integrated Vostok1 (Recommended)

Full-featured single-node implementation:

```bash
ros2 run plan vostok1
```

**Terminal Output:**

```text
[INFO] PROJET-17 — Vostok 1 Navigation System
[INFO] Waiting for GPS signal...
[INFO] MISSION DÉMARRÉE ! | MISSION STARTED!
[INFO] PT 1/19 | Pos: (5.2, 3.1) | Cible: (15.0, 0.0) | Dist: 10.2m | Cap: 45°
[INFO] ✅ DÉGAGÉ | CLEAR (F:50.0 L:50.0 R:50.0)
```

### Option B: Modular Navigation (YAML Launch)

For configurable PID and distributed architecture:

```bash
ros2 launch ~/seal_ws/src/uvautoboat/launch/vostok1.launch.yaml
```

> **Note:** The YAML launch file contains all default parameters. Edit the file directly to customize, or use the Python launch file for command-line arguments.

**Alternative with Python Launch (supports command-line args):**

```bash
ros2 launch plan vostok1_modular_navigation.launch.py kp:=500.0 ki:=30.0 kd:=150.0
```

**Available Parameters (in `vostok1.launch.yaml`):**

| Node | Parameter | Default | Description |
|:-----|:----------|:--------|:------------|
| **OKO** | `min_safe_distance` | 12.0 | Obstacle safe distance (m) |
| | `critical_distance` | 4.0 | Critical obstacle distance (m) |
| | `min_height` | -15.0 | Min Z to detect (lake bank, water) |
| | `max_height` | 10.0 | Max Z to detect |
| | `min_range` | 5.0 | Ignore obstacles closer (boat structure) |
| | `max_range` | 50.0 | Max detection range (m) |
| | `temporal_history_size` | 3 | Scans to keep in history (faster response) |
| | `temporal_threshold` | 2 | Min detections to confirm obstacle (2/3) |
| | `cluster_distance` | 2.0 | Max distance between cluster points (m) |
| | `min_cluster_size` | 3 | Min points per cluster (detect small obstacles) |
| | `water_plane_threshold` | 0.5 | Tolerance for water plane removal (m) |
| | `velocity_history_size` | 5 | Frames for velocity estimation |
| **SPUTNIK** | `scan_length` | 15.0 | Lawnmower lane length (m) |
| | `scan_width` | 30.0 | Lane spacing (m) |
| | `lanes` | 10 | Number of lawnmower lanes |
| | `waypoint_tolerance` | 2.0 | Arrival radius (m) |
| | `waypoint_skip_timeout` | 45.0 | Skip blocked waypoint after (s) |
| **BURAN** | `kp` | 400.0 | PID Proportional gain |
| | `ki` | 20.0 | PID Integral gain |
| | `kd` | 100.0 | PID Derivative gain |
| | `base_speed` | 500.0 | Base thrust speed (N) |
| | `max_speed` | 800.0 | Maximum thrust (N) |
| | `obstacle_slow_factor` | 0.3 | Speed reduction near obstacles |
| | `critical_distance` | 5.0 | Stop distance (m) |
| | `stuck_timeout` | 3.0 | SASS: stuck detection time (s) |
| | `stuck_threshold` | 0.5 | SASS: min movement to not be stuck (m) |
| | `no_go_zone_radius` | 8.0 | SASS: no-go zone radius (m) |
| | `detour_distance` | 12.0 | SASS: detour waypoint distance (m) |

### Keyboard Teleop

Manual control for testing — **War Thunder / GTA5 naval style** with persistent throttle and auto-centering rudder.

```bash
ros2 run control keyboard_teleop
```

**Throttle (persists like a lever):**

| Key | Action |
|:---:|:-------|
| `W` / `↑` | Increase throttle (speed up) |
| `S` / `↓` | Decrease throttle (slow down / reverse) |
| `Space` | All stop (zero throttle + center rudder) |
| `X` | Emergency full reverse |

**Rudder (auto-returns to center):**

| Key | Action |
|:---:|:-------|
| `A` / `←` | Steer left |
| `D` / `→` | Steer right |
| `Q` | Hard left turn |
| `E` | Hard right turn |
| `R` | Center rudder |

**Power:**

| Key | Action |
|:---:|:-------|
| `+` / `=` | Increase max thrust |
| `-` | Decrease max thrust |
| `H` | Show help |
| `Ctrl+C` | Quit |

**Behavior:**

- Throttle persists between keypresses (like a real throttle lever)
- Rudder automatically returns to center when released
- Rudder effect scales with speed (more responsive at higher speeds)
- Visual HUD shows throttle %, rudder position, and thrust values

---

## Web Dashboard

Real-time monitoring interface with TNO Cold War aesthetic.

### Prerequisites

```bash
sudo apt install ros-jazzy-rosbridge-suite
```

### Dashboard Panels

| Panel | Description |
|:------|:------------|
| **Connection Status** | WebSocket connection indicator |
| **GPS Position** | Latitude, longitude, local coordinates |
| **Mission Status** | State, waypoint progress, distance |
| **Obstacle Detection** | Front/Left/Right clearance with status badge |
| **Thruster Output** | Left/Right thrust with visual bars |
| **Anti-Stuck (SASS)** | Escape phase, no-go zones, drift vector |
| **Trajectory Map** | Interactive Leaflet map with boat position |
| **Configuration** | Path, PID, Speed parameter controls |
| **Terminal Output** | Live ROS log feed |

### Configuration Panel

Runtime parameter tuning:

| Section | Parameters | Description |
|:--------|:-----------|:------------|
| **Path** | Lanes, Length, Width | Lawnmower pattern |
| **PID** | Kp, Ki, Kd | Heading controller gains |
| **Speed** | Base, Max | Motion control |

**Buttons:**

| Button | Action |
|:-------|:-------|
| **Apply Config** | Send all parameters |
| **Go Home** | Return to spawn point |

---

## Smart Anti-Stuck System (SASS)

Intelligent recovery system when the boat becomes trapped or immobilized. Both the Vostok1 and Atlantis controllers implement the Smart Anti-Stuck System, features multi phase escape maneuvers (Probe -> Reverse -> Turn -> Forward)

### SASS Features

| Feature | Description |
|:--------|:------------|
| **Adaptive Escape** | 10-20s duration based on severity |
| **Multi-Direction Probe** | Scans L/R/Back before choosing escape |
| **No-Go Zones** | Remembers stuck locations (max 20, 8m radius) |
| **Kalman Drift Compensation** | Estimates current/wind with uncertainty |
| **Detour Insertion** | Auto-adds waypoints around obstacles |
| **Learning** | Records successful escapes for future |

### Adaptive Duration Calculation

```text
Base Duration: 10s
+ 4s if obstacle < critical distance (5m)
+ 2s if obstacle < safe distance (15m)  
+ 2s per consecutive stuck attempt
Maximum: 20s
```

### Escape Sequence

| Phase | Duration | Action |
|:------|:---------|:-------|
| **0: PROBE** | 0-2s | Multi-direction scan (L/R/Back) |
| **1: REVERSE** | 2s-~6s | Backward thrust |
| **2: TURN** | 6s-~10s | Rotate toward best direction |
| **3: FORWARD** | 10s-~12s | Forward test with drift compensation |

### Kalman Filter for Drift

The system uses a 2D Kalman filter to estimate environmental drift:

```python
x = [drift_x, drift_y]    # State estimate
P = uncertainty           # Covariance (lower = more confident)
Q = 0.001                 # Process noise (drift changes slowly)
R = 0.1                   # Measurement noise (GPS/IMU)
```

**Dashboard Uncertainty Colors:** 🟢 < 0.05 (confident) | 🟡 0.05-0.15 | 🔴 > 0.15

---

## Waypoint Skip Strategy

The Atlantis controller includes a Waypoint Timeout feature (deafult is 60 seconds) that automatically skips a target if the vessel cannot reach it due to currents or persistent obstacles. This was a update for Atlantis Controller.

When obstacles block waypoints, the system uses two strategies to continue the mission:

### 1. Stuck-Based Skip

When the boat is physically stuck and cannot move:

| Attempt | Action |
|:--------|:-------|
| 1st | Smart escape maneuver (probe + reverse + turn) |
| 2nd | Request detour waypoint around obstacle |
| 3rd | Extended escape with no-go zone |
| 4th+ | **Skip waypoint** and continue to next |

### 2. Obstacle Blocking Skip

When the boat keeps circling near a waypoint but can't reach it:

| Condition | Action |
|:----------|:-------|
| Distance < 20m | Start tracking obstacle blocking time |
| Obstacle detected | Accumulate blocking time |
| Blocking time ≥ 45s | **Skip waypoint** automatically |

### 3. Go Home Mode

When returning home encounters obstacles, the system uses **detour insertion** instead of skipping:

| Condition | Action |
|:----------|:-------|
| Distance < 20m | Start tracking obstacle blocking time |
| Blocking time ≥ 15s | **Insert detour waypoint** perpendicular to obstacle |
| Detour reached | Continue toward home |

This ensures the boat always reaches home, even through buoy fields.

**Log Output:**

```text
🏠 HOME MODE: Obstacle blocking for 15s - Inserting detour
DÉTOUR! Inserting detour waypoint LEFT at (45.2, -12.8)
```

**Configuration:**

```yaml
# In vostok1.launch.yaml
- name: waypoint_skip_timeout
  value: 45.0  # Seconds of obstacle blocking before skip (normal mode)
```

**Normal Mode Log Output:**

```text
⏭️ SAUT PT 3/10 | SKIP WP - Obstacle blocking for 45s (target was 8.2m away)
```

> **Note:** Skipping ensures mission completion even when waypoints are placed among obstacles like buoy lines.

---

## Terminal Mission Control

The **vostok1_cli** provides terminal-based mission control when the web dashboard is unavailable or for scripted automation. It supports both navigation architectures and includes automatic readiness checking.

### Features

| Feature | Description |
|:--------|:------------|
| **Auto-Ready Check** | Waits for navigation system before sending commands |
| **All-in-One Generate** | Waypoints + PID + Speed in a single command |
| **Dual Mode** | Works with both modular and integrated systems |
| **Interactive Shell** | Rapid command entry without retyping prefixes |

### Modes

| Mode | Flag | Description |
|:-----|:-----|:------------|
| **Modular** | `--mode modular` (default) | Sputnik + Buran |
| **Sputnik** | `--mode sputnik` | Alias for modular |
| **Vostok1** | `--mode vostok1` | Integrated navigation |

> **Note:** Default mode is `modular` since the Sputnik + Buran architecture is the primary system.

### Waypoint Generation

The `generate` command automatically waits for the navigation system to be ready before sending commands.

```bash
# Default: 8 lanes, 50m length, 20m width
ros2 run plan vostok1_cli generate

# Custom parameters
ros2 run plan vostok1_cli generate --lanes 10 --length 50 --width 20

# All-in-one: waypoints + PID + speed in one command
ros2 run plan vostok1_cli generate --lanes 10 --length 60 --width 25 --kp 400 --ki 20 --kd 100 --base 500 --max 800
```

**Output:**

```text
⏳ Waiting for navigation system...
✅ Navigation system ready!
✅ Waypoints generated: 10 lanes × 60m length × 25m width
   Estimated waypoints: 19
   Estimated distance: 825m
   PID: Kp=400, Ki=20, Kd=100
   Speed: base=500, max=800
```

| Parameter | Default | Description |
|:----------|:--------|:------------|
| `--lanes`, `-l` | 8 | Number of parallel scan lines |
| `--length`, `-L` | 50.0 | Length of each lane (meters) |
| `--width`, `-w` | 20.0 | Spacing between lanes (meters) |
| `--kp` | - | PID Proportional gain (optional) |
| `--ki` | - | PID Integral gain (optional) |
| `--kd` | - | PID Derivative gain (optional) |
| `--base` | - | Base speed in N (optional) |
| `--max` | - | Max speed in N (optional) |

### Mission Control

```bash
ros2 run plan vostok1_cli start     # 🚀 Start mission
ros2 run plan vostok1_cli stop      # 🛑 Pause mission
ros2 run plan vostok1_cli resume    # ▶️ Resume mission
ros2 run plan vostok1_cli home      # 🏠 Return to spawn
ros2 run plan vostok1_cli reset     # 🔄 Clear waypoints and reset
ros2 run plan vostok1_cli confirm   # ✅ Confirm waypoints
ros2 run plan vostok1_cli status    # 📊 Show current status
```

### Parameter Tuning

```bash
# PID gains
ros2 run plan vostok1_cli pid --kp 400 --ki 20 --kd 100

# Speed limits
ros2 run plan vostok1_cli speed --base 500 --max 800
```

### Interactive Mode

Launch an interactive shell for rapid command entry:

```bash
ros2 run plan vostok1_cli interactive
```

| Command | Action |
|:--------|:-------|
| `g [lanes] [length] [width]` | Generate waypoints |
| `c` | Confirm waypoints |
| `s` | Start mission |
| `x` | Stop/pause |
| `r` | Resume |
| `home` | 🏠 Go to spawn |
| `reset` | Reset mission |
| `status` | Show status |
| `pid <kp> <ki> <kd>` | Set PID parameters |
| `speed <base> <max>` | Set speed limits |
| `q` | Quit interactive mode |

### Vostok1 Mode Examples

For the integrated Vostok1 architecture (single node):

```bash
# Generate waypoints
ros2 run plan vostok1_cli --mode vostok1 generate --lanes 8 --length 15 --width 5

# Start mission
ros2 run plan vostok1_cli --mode vostok1 start

# Interactive mode
ros2 run plan vostok1_cli --mode vostok1 interactive
```

### Typical Workflow

> **Prerequisites:** Make sure Gazebo and the navigation system are running first!
>
> ```bash
> # Terminal 1: Start Gazebo simulation
> ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
> 
> # Terminal 2: Start modular navigation system
> ros2 launch ~/seal_ws/src/uvautoboat/launch/vostok1.launch.yaml
> ```

```bash
# 1. Generate waypoints with PID and speed (all-in-one)
#    CLI will wait for navigation system to be ready
ros2 run plan vostok1_cli generate --lanes 10 --length 60 --width 25 --kp 400 --ki 20 --kd 100 --base 500 --max 800

# 2. Confirm waypoints
ros2 run plan vostok1_cli confirm

# 3. Start mission
ros2 run plan vostok1_cli start

# 4. Monitor (optional)
ros2 run plan vostok1_cli status

# 5. Pause if needed
ros2 run plan vostok1_cli stop

# 6. Resume
ros2 run plan vostok1_cli resume

# 7. Return home when done
ros2 run plan vostok1_cli home
```

> **Auto-Ready Check:** The `generate` command automatically waits up to 5 seconds for the navigation system to respond. If not ready, it will show which command to run to start the navigation system.

---

## Technical Documentation

### GPS Navigation

**Equirectangular Projection:**

```text
Local X = (lat - start_lat) × 6,371,000m
Local Y = (lon - start_lon) × 6,371,000m × cos(start_lat)
```

### 3D LIDAR Processing

**Height Reference (LiDAR mounted ~2-3m above water):**

| Surface | Z Value |
|:--------|:--------|
| Water surface | ≈ -3m |
| Lake bank/terrain | ≈ -2.5m |
| Concrete harbour | ≈ -1 to -0.5m |
| Obstacles on dock | ≈ 0 to +2m |

**Sector Analysis:**

| Sector | Angle Range | Purpose |
|:-------|:------------|:--------|
| Front | -45° to +45° | Forward detection |
| Left | +45° to +135° | Left clearance |
| Right | -135° to -45° | Right clearance |

### Bayesian Fundamentals

The navigation system uses Bayesian inference for state estimation:

**Bayes' Theorem:**

```text
P(State | Data) = P(Data | State) × P(State) / P(Data)
     ↓                  ↓              ↓
  Posterior         Likelihood       Prior
```

| Term | Meaning | Example |
|:-----|:--------|:--------|
| **Prior** | Belief before measurement | "Drift was ~0.1 m/s" |
| **Likelihood** | Probability of data | "GPS shows velocity mismatch" |
| **Posterior** | Updated belief | "Drift is now ~0.15 m/s" |

### Kalman Filter

The Kalman filter is Bayes' theorem for continuous states with Gaussian distributions:

**Predict:** `P = P + Q` (uncertainty grows)  
**Update:** `K = P/(P+R)`, `x = x + K(z-x)`, `P = (1-K)P` (uncertainty shrinks)

---

### Atlantis Planner Safety Line Check

Unlike standard waypoints planners, the Atlantis planner verifies the entire path segment between waypoints (every 2 meters) to prevent planning trajectories that intersect with obstacles.

## Troubleshooting

### Expected Log Messages

| Stage | Expected Output |
|:------|:----------------|
| Startup | "MISSION DÉMARRÉE / MISSION STARTED" |
| Navigation | "PT X/19 \| Pos: (x, y) \| Cible: (tx, ty)" |
| Obstacle | "🚨 OBSTACLE DETECTED!" |
| Clear | "✅ DÉGAGÉ \| CLEAR" |
| Stuck | "🚨 BLOQUÉ! \| STUCK!" |
| Skip | "⏭️ SAUT PT \| SKIP WP" |
| Complete | "MISSION TERMINÉE!" |

### Common Issues

| Problem | Solution |
|:--------|:---------|
| **Boat not moving** | Check GPS: `ros2 topic echo /wamv/sensors/gps/gps/fix --once` |
| **Spinning in circles** | Reduce PID: `ros2 param set /vostok1_node kp 300` |
| **Dashboard disconnected** | Restart rosbridge, check port 9090 |
| **No obstacles detected** | Check LIDAR: `ros2 topic hz /wamv/sensors/lidars/lidar_wamv/points` |
| **Critical at spawn** | Increase `min_range` to 5.0 in launch file |
| **Build failures** | Clean: `rm -rf build install log && colcon build` |

### Debug Commands

```bash
# Check GPS
ros2 topic echo /wamv/sensors/gps/gps/fix --once

# Check LIDAR rate
ros2 topic hz /wamv/sensors/lidars/lidar_wamv/points

# Check node running
ros2 node list | grep vostok

# List parameters
ros2 param list /vostok1_node

# Check SASS status
ros2 topic echo /vostok1/anti_stuck_status
```

---

## Command Cheatsheet

### Kill Processes

```bash
# Kill all Gazebo
pkill -9 -f "gz sim" && pkill -9 -f "gzserver" && pkill -9 -f "gzclient"

# Kill ROS nodes
pkill -9 -f vostok1 && pkill -9 -f rosbridge

# Nuclear option
pkill -9 -f ros && pkill -9 -f gz && pkill -9 -f gazebo
```

### Build

```bash
# Full build
cd ~/seal_ws && colcon build --merge-install

# Specific packages
colcon build --packages-select plan control --merge-install

# Clean build
rm -rf build install log && colcon build --merge-install

# Run unit tests
colcon test --packages-select plan control
colcon test-result --verbose
```

### Launch

```bash
# Simulation
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta

# Navigation
ros2 run plan vostok1
ros2 launch ~/seal_ws/src/uvautoboat/launch/vostok1.launch.yaml

# Dashboard
ros2 launch rosbridge_server rosbridge_websocket_launch.xml delay_between_messages:=0.0
cd ~/seal_ws/src/uvautoboat/web_dashboard/vostok1 && python3 -m http.server 8000
```

### Teleport Boat

```bash
gz service -s /world/sydney_regatta/set_pose \
  --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 1000 \
  --req 'name: "wamv", position: {x: 0, y: 0, z: 0.5}'
```

---

## Future Roadmap

### Planned Features

| Feature | Priority | Description |
|:--------|:---------|:------------|
| **A* Path Planning** | High | Pre-compute obstacle-free paths to any destination |
| **Dynamic Replanning** | High | Replan path when new obstacles detected |
| **Coverage Planning** | Medium | Boustrophedon pattern for area coverage |
| **Multi-Goal Navigation** | Medium | Navigate through sequence of random points |

### A* Path Planning (Proposed)

For navigating to arbitrary points through obstacle fields:

```text
1. Create occupancy grid (10m cells) from LIDAR data
2. Mark known obstacles (buoys, docks) on grid
3. Use A* algorithm to find optimal path
4. Generate waypoints along path
5. If new obstacle detected → replan dynamically
```

**Proposed Architecture:**

```text
/goal_point ──────┐
                  ├──→ [pathfinder.py] ──→ /planned_waypoints ──→ [vostok1/sputnik]
/oko/obstacles ───┘
```

**Benefits:**

- Works for any destination point
- Avoids obstacles from the start
- No circling behavior
- Efficient paths through complex environments

### Technical Debt

| Issue | Description |
|:------|:------------|
| **ROS 2 Parameter Migration** | Hardcoded parameters can be migrated to ROS 2 parameter server, but these settings may not always work reliably |
| **Multi-Terminal Launch** | Lack of shell script (.sh file) to open multiple terminals and run multiple nodes simultaneously |
| **Debugging Required** | Complex planning and obstacle detection still need debugging |

---

## Contributing

### Development Guidelines

1. **Code Style**: Follow PEP 8 for Python
2. **Documentation**: Update README for significant changes
3. **Testing**: Include unit tests for new features
4. **Commits**: Use clear, descriptive messages

### Reporting Issues

Open an issue on [GitHub](https://github.com/Erk732/uvautoboat/issues) with:

- Problem description
- Steps to reproduce
- Expected vs actual behavior
- System information

---

## References

### Documentation

- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [Gazebo Harmonic Documentation](https://gazebosim.org/docs/harmonic)
- [VRX Wiki](https://github.com/osrf/vrx/wiki)
- [Kalman Filter Illustrated](https://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/)

### Message Types

- [sensor_msgs/NavSatFix](http://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html)
- [sensor_msgs/PointCloud2](http://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html)
- [sensor_msgs/Imu](http://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html)

### Related Projects

- [Virtual RobotX (VRX)](https://github.com/osrf/vrx)
- [ros2_control](https://github.com/ros-controls/ros2_control)

---

## Acknowledgments

**Maintained By**: AutoBoat Development Team

**Institution**: [IMT Nord Europe](https://imt-nord-europe.fr/) — Industry 4.0 Students & Faculty

**Special Thanks**:

- [Open Source Robotics Foundation (OSRF)](https://www.openrobotics.org/) for VRX and Gazebo
- [ROS 2 Community](https://www.ros.org/) for the robotics middleware
- Contributors and testers who helped improve this project

**Development Teams**:

- Planning Team: Atlantis Planner
- Control Team: Atlantis Controller

---

## License

This project is licensed under the **Apache License 2.0**.

See [LICENSE](LICENSE) for details.

---

**AutoBoat** — Autonomous Navigation for VRX Competition

Built with [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/) + [Gazebo Harmonic](https://gazebosim.org/docs/harmonic)

[Report Bug](https://github.com/Erk732/uvautoboat/issues) · [Request Feature](https://github.com/Erk732/uvautoboat/issues)
