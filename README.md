# AutoBoat

[![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-blue)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange)](https://gazebosim.org/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

> Autonomous Navigation System for Unmanned Surface Vehicles (USVs) — Virtual RobotX Competition

## Table of Contents

- [AutoBoat](#autoboat)
  - [Table of Contents](#table-of-contents)
  - [Overview](#overview)
  - [Installation](#installation)
  - [Quick Start](#quick-start)
  - [Keyboard Teleop](#keyboard-teleop)
    - [Joystick Control (VRX)](#joystick-control-vrx)
  - [Coordinate System](#coordinate-system)
  - [System Architecture](#system-architecture)
    - [Navigation Systems](#navigation-systems)
    - [ROS 2 Topics](#ros-2-topics)
  - [Usage Guide](#usage-guide)
    - [Vostok1 (Recommended)](#vostok1-recommended)
    - [Modular Navigation](#modular-navigation)
  - [Web Dashboard](#web-dashboard)
    - [Quick Start (4 Terminals)](#quick-start-4-terminals)
    - [Dashboard Panels](#dashboard-panels)
    - [Configuration](#configuration)
  - [Smart Anti-Stuck System (SASS)](#smart-anti-stuck-system-sass)
    - [Features](#features)
    - [Escape Sequence](#escape-sequence)
    - [Kalman Filter for Drift](#kalman-filter-for-drift)
  - [Terminal Mission Control](#terminal-mission-control)
    - [Modes](#modes)
    - [Commands Reference](#commands-reference)
      - [Waypoint Generation](#waypoint-generation)
      - [Mission Control](#mission-control)
      - [Parameter Tuning](#parameter-tuning)
      - [Interactive Mode](#interactive-mode)
    - [Modular Mode Examples](#modular-mode-examples)
    - [Typical Workflow](#typical-workflow)
  - [Technical Reference](#technical-reference)
    - [GPS Navigation](#gps-navigation)
    - [IMU Heading](#imu-heading)
    - [3D LIDAR Processing](#3d-lidar-processing)
    - [PID Control](#pid-control)
    - [Differential Thrust](#differential-thrust)
    - [Bayesian Fundamentals](#bayesian-fundamentals)
    - [Kalman Filter (Drift Estimation)](#kalman-filter-drift-estimation)
  - [Troubleshooting](#troubleshooting)
    - [Common Issues](#common-issues)
    - [Debug Commands](#debug-commands)
  - [Command Reference](#command-reference)
    - [Essential Commands](#essential-commands)
    - [ROS 2 Introspection](#ros-2-introspection)
    - [Mission CLI](#mission-cli)
    - [Teleport Boat](#teleport-boat)
  - [Contributing](#contributing)
  - [References](#references)
  - [📜 Acknowledgments](#-acknowledgments)
  - [License](#license)

---

## Overview

AutoBoat provides GPS-based waypoint navigation with LIDAR obstacle avoidance for the WAM-V platform. Built on **[ROS 2 Jazzy](https://docs.ros.org/en/jazzy/)** and **[Gazebo Harmonic](https://gazebosim.org/docs/harmonic)**.

**Key Features:**

- Autonomous lawnmower-pattern navigation
- 3D point cloud obstacle detection
- Smart Anti-Stuck System (SASS) with Kalman-filtered drift compensation
- Real-time web dashboard
- Differential thrust control with PID heading

| Component | Requirement |
|-----------|-------------|
| OS | Ubuntu 24.04 LTS |
| RAM | 8 GB (16 GB recommended) |
| Dependencies | ROS 2 Jazzy, Gazebo Harmonic, VRX |

---

## Installation

```bash
# 1. Create workspace
mkdir -p ~/seal_ws/src && cd ~/seal_ws/src

# 2. Clone repositories
git clone https://github.com/Erk732/uvautoboat.git
git clone https://github.com/osrf/vrx.git

# 3. Install dependencies and build
cd ~/seal_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --merge-install

# 4. Add to ~/.bashrc (recommended)
echo "source ~/seal_ws/install/setup.bash" >> ~/.bashrc
```

---

## Quick Start

**Terminal 1** — Simulation:

```bash
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
```

**Terminal 2** — Navigation (choose one):

```bash
ros2 run plan vostok1           # Integrated (recommended)
ros2 run plan apollo11          # Simple 2D LIDAR
ros2 launch plan vostok1_modular_navigation.launch.py  # Modular
```

[![VRX Simulation](images/sydney_regatta_gzsim.png)](https://vimeo.com/851696025)

*Sydney Regatta simulation environment in Gazebo. Credit to [VRX Project](https://github.com/osrf/vrx/wiki/running_vrx_tutorial)*

---

## Keyboard Teleop

Manual control for testing — **War Thunder / GTA5 naval style** with persistent throttle and auto-centering rudder.

```bash
ros2 run control keyboard_teleop
```

**Throttle (persists like a lever):**

| Key | Action |
|-----|--------|
| `W` / `↑` | Increase throttle (speed up) |
| `S` / `↓` | Decrease throttle (slow down / reverse) |
| `Space` | All stop (zero throttle + center rudder) |
| `X` | Emergency full reverse |

**Rudder (auto-returns to center):**

| Key | Action |
|-----|--------|
| `A` / `←` | Steer left |
| `D` / `→` | Steer right |
| `Q` | Hard left turn |
| `E` | Hard right turn |
| `R` | Center rudder |

**Power:**

| Key | Action |
|-----|--------|
| `+` / `=` | Increase max thrust |
| `-` | Decrease max thrust |
| `H` | Show help |
| `Ctrl+C` | Quit |

**Behavior:**

- Throttle persists between keypresses (like a real throttle lever)
- Rudder automatically returns to center when released
- Rudder effect scales with speed (more responsive at higher speeds)
- Visual HUD shows throttle %, rudder position, and thrust values

### Joystick Control (VRX)

The VRX project also supports gamepad/joystick control. See [VRX Teleoperation Tutorial](https://github.com/osrf/vrx/wiki/teleop_tutorial) for setup.

---

## Coordinate System

![3D Cartesian Coordinate System](images/3d_coordinate_system.jpg)

*3D Cartesian coordinate system. [Primalshell](https://commons.wikimedia.org/wiki/File:3D_Cartesian_Coodinate_Handedness.jpg), [CC BY-SA 4.0](https://creativecommons.org/licenses/by-sa/4.0/)*

| Axis | Direction | Maritime |
|------|-----------|----------|
| **X** | Forward/Back | Ahead/Astern |
| **Y** | Left/Right | Port/Starboard |
| **Z** | Up/Down | Above/Below waterline |

| Rotation | Axis | Description |
|----------|------|-------------|
| **[Roll](https://en.wikipedia.org/wiki/Flight_dynamics#Roll)** | X | Side-to-side tilt |
| **[Pitch](https://en.wikipedia.org/wiki/Flight_dynamics#Pitch)** | Y | Front-to-back tilt |
| **[Yaw](https://en.wikipedia.org/wiki/Flight_dynamics#Yaw)** | Z | Horizontal rotation |

> 📚 Learn more: [Aircraft Principal Axes (Wikipedia)](https://en.wikipedia.org/wiki/Aircraft_principal_axes)

---

## System Architecture

### Navigation Systems

| System | LIDAR | Control | Best For |
|--------|-------|---------|----------|
| **Vostok1** | 3D PointCloud | PID + SASS | Production |
| **Apollo11** | 2D LaserScan | Direct thrust | Simple testing |
| **Modular** | 3D PointCloud | Configurable PID | Custom tuning |

### ROS 2 Topics

**Sensors:**

| Topic | Type |
|-------|------|
| `/wamv/sensors/gps/gps/fix` | NavSatFix |
| `/wamv/sensors/imu/imu/data` | Imu |
| `/wamv/sensors/lidars/.../points` | PointCloud2 |

**Actuators:**

| Topic | Type |
|-------|------|
| `/wamv/thrusters/left/thrust` | Float64 |
| `/wamv/thrusters/right/thrust` | Float64 |

**Dashboard (Vostok1):**

| Topic | Description |
|-------|-------------|
| `/vostok1/mission_status` | Mission state (JSON) |
| `/vostok1/config` | Current parameters |
| `/vostok1/set_config` | Parameter updates |

---

## Usage Guide

### Vostok1 (Recommended)

```bash
ros2 run plan vostok1
```

### Modular Navigation

```bash
ros2 launch plan vostok1_modular_navigation.launch.py kp:=500.0 ki:=30.0 kd:=150.0
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `kp/ki/kd` | 400/20/100 | PID gains |
| `base_speed` | 500.0 | Base thrust |
| `max_speed` | 800.0 | Max thrust |
| `lanes` | 10 | Scan lanes |

**Modular Nodes:** ОКО (perception) → СПУТНИК (planning) → БУРАН (control)

---

## Web Dashboard

Real-time monitoring and control interface.

### Quick Start (4 Terminals)

| Terminal | Command |
|----------|---------|
| T1 | `ros2 launch vrx_gz competition.launch.py world:=sydney_regatta` |
| T2 | `ros2 launch rosbridge_server rosbridge_websocket_launch.xml delay_between_messages:=0.0` |
| T3 | `ros2 run plan vostok1` |
| T4 | `cd ~/seal_ws/src/uvautoboat/web_dashboard && python3 -m http.server 8000` |

Open: <http://localhost:8000>

### Dashboard Panels

| Panel | Description |
|-------|-------------|
| GPS/Mission | Position, waypoint progress |
| Obstacles | Front/Left/Right clearance |
| SASS | Anti-stuck status, drift, no-go zones |
| Configuration | PID, speed, path parameters |
| Terminal | Live ROS log feed |

### Configuration

```bash
# Runtime parameter tuning
ros2 param set /vostok1_node kp 500.0
ros2 param set /vostok1_node ki 30.0
```

---

## Smart Anti-Stuck System (SASS)

Intelligent recovery system when the boat becomes trapped.

### Features

| Feature | Description |
|---------|-------------|
| **Adaptive Escape** | 10-20s duration based on severity |
| **Multi-Direction Probe** | Scans L/R/Back before choosing escape direction |
| **No-Go Zones** | Remembers stuck locations (max 20, 8m radius) |
| **Kalman Drift Compensation** | Estimates current/wind with uncertainty |
| **Detour Insertion** | Auto-adds waypoints around obstacles |
| **Learning** | Records successful escapes for future decisions |

### Escape Sequence

| Phase | Action |
|-------|--------|
| 0: PROBE | Scan left/right clearance |
| 1: REVERSE | Back away from obstacle |
| 2: TURN | Rotate toward best direction |
| 3: FORWARD | Test forward with drift compensation |

### Kalman Filter for Drift

The system uses a 2D [Kalman filter](https://en.wikipedia.org/wiki/Kalman_filter) to estimate environmental drift. See also: [How a Kalman Filter Works (Illustrated)](https://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/).

```python
# Simplified implementation
x = [drift_x, drift_y]    # State estimate
P = uncertainty           # Covariance (lower = more confident)
Q = 0.001                 # Process noise (drift changes slowly)
R = 0.1                   # Measurement noise (GPS/IMU)
```

**Dashboard Uncertainty Colors:** 🟢 < 0.05 (high confidence) | 🟡 0.05-0.15 | 🔴 > 0.15

---

## Terminal Mission Control

The **Mission CLI** provides terminal-based mission control when the web dashboard is unavailable or for scripted automation. It supports both navigation architectures.

### Modes

| Mode | Flag | Description |
|------|------|-------------|
| **Vostok1** | `--mode vostok1` (default) | Integrated navigation node |
| **Modular** | `--mode modular` | Sputnik planner + Buran controller |

### Commands Reference

#### Waypoint Generation

```bash
# Default: 4 lanes, 150m length, 20m width this is the best route for now!
ros2 run plan mission_cli generate

# Custom parameters
ros2 run plan mission_cli generate --lanes 8 --length 50 --width 20
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `--lanes` | 8 | Number of parallel scan lines |
| `--length` | 50.0 | Length of each lane (meters) |
| `--width` | 20.0 | Spacing between lanes (meters) |

#### Mission Control

```bash
ros2 run plan mission_cli start    # 🚀 Start mission
ros2 run plan mission_cli stop     # 🛑 Pause mission
ros2 run plan mission_cli resume   # ▶️ Resume mission
ros2 run plan mission_cli reset    # 🔄 Clear waypoints and reset
ros2 run plan mission_cli status   # 📊 Show current status
```

#### Parameter Tuning

```bash
# PID gains
ros2 run plan mission_cli pid --kp 400 --ki 20 --kd 100

# Speed limits
ros2 run plan mission_cli speed --base 500 --max 800
```

#### Interactive Mode

Launch an interactive shell for rapid command entry:

```bash
ros2 run plan mission_cli interactive
```

**Interactive Commands:**

| Command | Action |
|---------|--------|
| `g [lanes] [length] [width]` | Generate waypoints |
| `c` | Confirm waypoints |
| `s` | Start mission |
| `x` | Stop/pause mission |
| `r` | Resume mission |
| `reset` | Reset mission |
| `status` | Show status |
| `pid <kp> <ki> <kd>` | Set PID parameters |
| `speed <base> <max>` | Set speed limits |
| `q` | Quit interactive mode |

### Modular Mode Examples

For the modular architecture (Sputnik planner + Buran controller):

```bash
# Generate waypoints
ros2 run plan mission_cli --mode modular generate --lanes 8 --length 15 --width 5

# Start mission
ros2 run plan mission_cli --mode modular start

# Interactive mode
ros2 run plan mission_cli --mode modular interactive
```

### Typical Workflow

```bash
# 1. Generate waypoints
ros2 run plan mission_cli generate --lanes 10 --length 60 --width 25

# 2. Start mission
ros2 run plan mission_cli start

# 3. Monitor (optional)
ros2 run plan mission_cli status

# 4. Pause if needed
ros2 run plan mission_cli stop

# 5. Resume
ros2 run plan mission_cli resume
```

---

## Technical Reference

### GPS Navigation

Converts GPS to local coordinates using [equirectangular projection](https://en.wikipedia.org/wiki/Equirectangular_projection):

```text
Local X = (lat - start_lat) × 6,371,000m
Local Y = (lon - start_lon) × 6,371,000m × cos(start_lat)
```

### IMU Heading

[Quaternion](https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation) to yaw: `yaw = atan2(2(wz + xy), 1 - 2(y² + z²))`

### 3D LIDAR Processing

1. Height filter: -0.2m to 3.0m (excludes water/sky)
2. Distance filter: 50m detection range
3. Sector analysis: Front (±45°), Left/Right (45°-135°)

### PID Control

[PID Controller](https://en.wikipedia.org/wiki/PID_controller) for smooth heading adjustments:

```text
correction = Kp × error + Ki × ∫error + Kd × d(error)/dt
Default: Kp=800, Ki=50, Kd=100
```

> 📚 Learn more: [PID Control Explained (YouTube)](https://www.youtube.com/watch?v=wkfEZmsQqiA)

### Differential Thrust

| Maneuver | Left | Right |
|----------|------|-------|
| Forward | +500 | +500 |
| Turn Left | +200 | +500 |
| Spin Left | -500 | +500 |

Range: -1000 to +1000 N

### Bayesian Fundamentals

The navigation system uses [Bayesian inference](https://en.wikipedia.org/wiki/Bayesian_inference) for state estimation.

**[Bayes' Theorem](https://en.wikipedia.org/wiki/Bayes%27_theorem):**

```text
P(State | Data) = P(Data | State) × P(State) / P(Data)
     ↓                  ↓              ↓
  Posterior         Likelihood       Prior
 (new belief)     (sensor model)  (old belief)
```

| Term | Meaning | Example |
|------|---------|----------|
| **Prior** | Belief before measurement | "Drift was ~0.1 m/s" |
| **Likelihood** | Probability of seeing this data | "GPS shows velocity mismatch" |
| **Posterior** | Updated belief after data | "Drift is now ~0.15 m/s" |

> 📚 Learn more: [Bayesian Inference (Wikipedia)](https://en.wikipedia.org/wiki/Bayesian_inference) | [Bayes' Theorem Explained (3Blue1Brown)](https://www.youtube.com/watch?v=HZGCoVF3YvM)

### Kalman Filter (Drift Estimation)

The [Kalman filter](https://en.wikipedia.org/wiki/Kalman_filter) is Bayes' theorem applied to **continuous states** with **Gaussian distributions**. It's the optimal estimator for linear systems.

**Predict:** `P = P + Q` (uncertainty grows)  
**Update:** `K = P/(P+R)`, `x = x + K(z-x)`, `P = (1-K)P` (uncertainty shrinks)

> 📚 Learn more: [Kalman Filter (Wikipedia)](https://en.wikipedia.org/wiki/Kalman_filter) | [Illustrated Guide](https://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/)

| Parameter | Value | Purpose |
|-----------|-------|---------|
| Q (process) | 0.001 | Drift changes slowly |
| R (measurement) | 0.1 | GPS/IMU noise |

---

## Troubleshooting

### Common Issues

| Problem | Solution |
|---------|----------|
| Boat not moving | Check GPS: `ros2 topic echo /wamv/sensors/gps/gps/fix --once` |
| Spinning in circles | Reduce PID: `ros2 param set /vostok1_node kp 300` |
| Dashboard disconnected | Restart rosbridge: `pkill -f rosbridge && ros2 launch rosbridge_server rosbridge_websocket_launch.xml` |
| No obstacles detected | Check LIDAR: `ros2 topic hz /wamv/sensors/lidars/lidar_wamv/points` |
| Build failures | Clean build: `rm -rf build install log && colcon build --merge-install` |

### Debug Commands

```bash
ros2 topic echo /wamv/sensors/gps/gps/fix --once     # Check GPS
ros2 topic hz /wamv/sensors/lidars/lidar_wamv/points # Check LIDAR rate
ros2 node list | grep vostok                          # Check node running
ros2 param list /vostok1_node                         # List parameters
```

---

## Command Reference

### Essential Commands

```bash
# Kill everything
pkill -9 -f gazebo && pkill -9 -f gz && pkill -9 -f ros

# Build
colcon build --packages-select plan control && source install/setup.bash

# Launch simulation
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta

# Start navigation
ros2 run plan vostok1

# Start dashboard
ros2 launch rosbridge_server rosbridge_websocket_launch.xml delay_between_messages:=0.0
cd ~/seal_ws/src/uvautoboat/web_dashboard && python3 -m http.server 8000
```

### ROS 2 Introspection

```bash
ros2 node list                           # List nodes
ros2 topic list                          # List topics
ros2 topic echo /topic --once            # Read once
ros2 topic hz /topic                     # Check frequency
ros2 param list /node                    # List parameters
ros2 param set /node param value         # Set parameter
```

### Mission CLI

```bash
ros2 run plan mission_cli generate --lanes 8 --length 50 --width 20
ros2 run plan mission_cli start
ros2 run plan mission_cli stop
ros2 run plan mission_cli resume
ros2 run plan mission_cli reset
ros2 run plan mission_cli pid --kp 400 --ki 20 --kd 100
ros2 run plan mission_cli interactive
```

### Teleport Boat

```bash
gz service -s /world/sydney_regatta/set_pose \
  --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 1000 \
  --req 'name: "wamv", position: {x: 0, y: 0, z: 0.5}'
```

---

## Contributing

1. Follow PEP 8 for Python code
2. Update README for significant changes
3. Include unit tests for new features
4. Use clear commit messages

Report issues: [GitHub Issues](https://github.com/Erk732/uvautoboat/issues)

---

## References

- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/)
- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic)
- [VRX Wiki](https://github.com/osrf/vrx/wiki)

---

## 📜 Acknowledgments

**Maintained By**: AutoBoat Development Team

**Institution**: [IMT Nord Europe](https://imt-nord-europe.fr/) — Industry 4.0 Students & Faculty

**Special Thanks**:

- [Open Source Robotics Foundation (OSRF)](https://www.openrobotics.org/) for VRX and Gazebo
- [ROS 2 Community](https://www.ros.org/) for the robotics middleware
- Contributors and testers who helped improve this project

---

## License

Apache License 2.0 — See [LICENSE](LICENSE)

---

**AutoBoat** — Autonomous Navigation for VRX Competition  
Built with [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/) + [Gazebo Harmonic](https://gazebosim.org/docs/harmonic)
