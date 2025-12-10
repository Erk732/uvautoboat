# System Overview

High-level architecture and design philosophy of the AutoBoat autonomous navigation system.

---

## Abstract

AutoBoat is an autonomous navigation system for unmanned surface vehicles (USVs) developed for the Virtual RobotX ([VRX](https://github.com/osrf/vrx)) competition. The system integrates advanced path planning, real-time obstacle avoidance, and precise trajectory tracking algorithms optimized for the WAM-V maritime platform.

Built on **ROS 2 Jazzy** and **Gazebo Harmonic**, the framework provides a robust foundation for autonomous maritime navigation in simulated environments.

---

## Key Contributions

- **Vostok1 Navigation System**: Integrated autonomous navigation with 3D LIDAR perception
- **Modular Architecture**: Distributed nodes (OKO-SPUTNIK-BURAN) for flexible deployment
- **Smart Anti-Stuck System (SASS)**: Intelligent recovery with Kalman-filtered drift compensation
- **Web Dashboard**: Real-time monitoring with visualization
- **Waypoint Skip Strategy**: Automatic skip for blocked waypoints ensuring mission completion
- **A* Path Planning**: Grid-based pathfinding for obstacle avoidance

---

## Navigation Systems Comparison

AutoBoat provides three navigation architectures to suit different needs:

| Aspect | Vostok1 (Integrated) | Modular (TNO) | Atlantis (Control Group) |
|:-------|:---------------------|:--------------|:-------------------------|
| **Approach** | Self-contained node | Distributed nodes | Integrated controller |
| **LIDAR** | 3D PointCloud2 | 3D PointCloud2 | 3D PointCloud2 |
| **Detection** | Full 3D volume | Full 3D volume | 3D Section Analysis |
| **Control** | PID heading | PID (configurable) | PID heading |
| **Monitoring** | Terminal + Web | Terminal (bilingual) | Web Dashboard |
| **Anti-Stuck** | SASS v2.0 | SASS v2.0 | Adaptive Escape with SASS |
| **Best For** | Production use | Custom tuning | Robust Path Validation |

### Vostok1 (Recommended)
- **Single-node** integrated system
- All functionality in one executable
- Simplest to deploy and use
- Ideal for **production missions**

### Modular (TNO)
- **Distributed architecture** with three nodes:
  - **OKO**: Perception (3D LIDAR)
  - **SPUTNIK**: Planning (waypoints)
  - **BURAN**: Control (PID + SASS)
- Highly **configurable via YAML**
- Ideal for **research and tuning**

### Atlantis
- Control group's approach
- Zero-latency obstacle reaction
- Embedded LIDAR processing in controller
- Ideal for **path validation research**

---

## High-Level Data Flow

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
    (-1000 to +1000N)          (-1000 to +1000N)
```

---

## Core Subsystems

### 1. Perception
- **3D LIDAR Processing** (OKO v2.0)
- Real-time point cloud filtering
- Sector-based obstacle detection
- Temporal filtering for reliability
- Obstacle clustering and gap detection
- Moving obstacle tracking

### 2. Planning
- **GPS Waypoint Navigation** (SPUTNIK)
- Lawnmower pattern generation
- A* path planning for obstacles
- Hazard zone avoidance
- Waypoint skip logic
- Detour insertion

### 3. Control
- **PID Heading Control** (BURAN)
- Differential thrust control
- Obstacle reaction
- Smart Anti-Stuck System (SASS)
- Drift compensation with Kalman filter
- Speed adaptation near obstacles

### 4. Monitoring
- **Web Dashboard**
- Real-time position visualization
- Mission status and progress
- Obstacle detection display
- Parameter configuration
- Camera feed integration

---

## Key Features Explained

### Autonomous Navigation
- GPS-based waypoint following
- Automatic lawnmower pattern generation
- Dynamic path adjustment for obstacles

### 3D Obstacle Avoidance
- Real-time LIDAR point cloud processing
- Front/Left/Right sector clearance analysis
- Continuous obstacle monitoring (~10 Hz)
- Distance-weighted urgency scoring

### Smart Anti-Stuck System (SASS)
- 4-phase escape sequence (Probe → Reverse → Turn → Forward)
- Multi-direction scanning before escape
- No-go zone memory (up to 20 zones)
- Kalman filter for drift estimation
- Adaptive duration based on severity

### Waypoint Skip Strategy
- Automatic skip after timeout (default: 45s)
- Go Home mode uses detour insertion instead
- Ensures mission completion in complex environments

### A* Path Planning
- Grid-based pathfinding (default: 3m cells)
- Obstacle inflation for safe clearance
- Pre-defined hazard zones
- Hybrid mode: pre-plan routes at generation
- Runtime mode: plan detours when stuck

---

## Software Stack

```text
┌───────────────────────────────────┐
│      Application Layer            │
│  (AutoBoat Navigation Nodes)      │
├───────────────────────────────────┤
│         ROS 2 Jazzy               │
│  (Middleware & Communication)     │
├───────────────────────────────────┤
│      Gazebo Harmonic              │
│   (Physics Simulation)            │
├───────────────────────────────────┤
│       Ubuntu 24.04 LTS            │
└───────────────────────────────────┘
```

### Technologies Used
- **ROS 2 Jazzy**: Robot middleware
- **Gazebo Harmonic**: 3D simulation
- **Python 3.10+**: Implementation language
- **NumPy/SciPy**: Numerical computations
- **rosbridge**: WebSocket bridge for dashboard
- **Leaflet.js**: Interactive map visualization

---

## Design Philosophy

### Modularity
- Clear separation of perception, planning, and control
- Reusable components
- Multiple architecture options

### Robustness
- Temporal filtering reduces false detections
- SASS ensures recovery from stuck states
- Waypoint skip prevents mission failures
- Kalman filtering for state estimation

### Flexibility
- Runtime parameter tuning
- Multiple control interfaces (CLI, dashboard, manual)
- Configurable via YAML launch files
- Support for custom waypoint patterns

### Real-Time Performance
- Optimized point cloud processing
- Efficient sector-based detection
- Continuous perception-control loop
- Sub-100ms control cycle

---

## Project Structure

```text
uvautoboat/
├── control/           # Control nodes (BURAN, teleop)
├── plan/              # Planning nodes (SPUTNIK, OKO, Vostok1)
├── launch/            # Top-level launch files
├── web_dashboard/     # Web monitoring interfaces
├── test_environment/  # Custom Gazebo worlds and models
├── environment_plugins/ # Gazebo plugins
└── wiki/              # This documentation
```

---

## Next Steps

Learn more about specific components:

- **[Vostok1 Architecture](Vostok1-Architecture)** — Integrated system details
- **[Modular Architecture](Modular-Architecture)** — OKO-SPUTNIK-BURAN design
- **[ROS 2 Topic Flow](ROS2-Topic-Flow)** — Communication patterns
- **[3D LIDAR Processing](3D-LIDAR-Processing)** — OKO perception deep-dive
- **[SASS](SASS)** — Smart Anti-Stuck System explained
- **[A* Path Planning](Astar-Path-Planning)** — Grid-based pathfinding
