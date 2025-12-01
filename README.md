# AutoBoat

[![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-blue)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange)](https://gazebosim.org/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
![Status](https://img.shields.io/badge/Status-Active-green)

> **Autonomous Navigation System for Unmanned Surface Vehicles (USVs)**  
> Developed for the Virtual RobotX (VRX) Competition

---

## 📋 Table of Contents

1. [Overview](#overview)
2. [Installation](#installation)
3. [Quick Start](#quick-start)
4. [Coordinate System](#coordinate-system)
5. [Features](#features)
6. [System Architecture](#system-architecture)
7. [Usage Guide](#usage-guide)
8. [Smart Anti-Stuck System (SASS)](#smart-anti-stuck-system-sass)
9. [Technical Documentation](#technical-documentation)
10. [Testing](#testing)
11. [Troubleshooting](#troubleshooting)
12. [Command Cheatsheet](#command-cheatsheet)
13. [Contributing](#contributing)
14. [References](#references)
15. [Acknowledgments](#acknowledgments)
16. [License](#license)

---

## Overview

AutoBoat is an autonomous navigation system designed for maritime surface vehicles competing in the Virtual RobotX (VRX) competition. Built on **ROS 2 Jazzy** and **Gazebo Harmonic**, the system provides:

- Real-time GPS-based waypoint navigation
- LIDAR-based obstacle detection and avoidance
- Differential thrust control for precise maneuvering
- Two distinct autonomous implementations for different use cases

### Project Status

| Phase | Description | Status |
|:------|:------------|:------:|
| Phase 1 | Architecture & MVP | ✅ Completed |
| Phase 2 | Autonomous Navigation | ✅ Completed |
| Phase 3 | Coverage & Search | ⏸️ Planned |
| Phase 4 | Integration & Testing | 🔄 60% |

See [Board.md](Board.md) for detailed milestones and progress tracking.

---

## Installation

### System Requirements

| Component | Minimum | Recommended |
|:----------|:--------|:------------|
| OS | Ubuntu 24.04 LTS | Ubuntu 24.04 LTS |
| RAM | 8 GB | 16 GB |
| Storage | 40 GB | 60 GB |
| Python | 3.10+ | 3.12 |

### Prerequisites

- Ubuntu 24.04 LTS
- ROS 2 Jazzy ([Installation Guide](https://docs.ros.org/en/jazzy/Installation.html))
- Gazebo Harmonic ([Installation Guide](https://gazebosim.org/docs/harmonic/install_ubuntu/))
- VRX Simulation ([GitHub](https://github.com/osrf/vrx))

### Step-by-Step Installation

1. **Create Workspace**

   ```bash
   mkdir -p ~/seal_ws/src
   cd ~/seal_ws/src
   ```

2. **Clone Repositories**

   ```bash
   git clone https://github.com/Erk732/uvautoboat.git
   git clone https://github.com/osrf/vrx.git
   ```

3. **Install Dependencies**

   ```bash
   cd ~/seal_ws
   source /opt/ros/jazzy/setup.bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

4. **Build Workspace**

   ```bash
   colcon build --merge-install
   ```

5. **Source Environment**

   ```bash
   source ~/seal_ws/install/setup.bash
   ```

   **Optional (Recommended)**: Add to `~/.bashrc` so you don't need to source in every new terminal:

   ```bash
   echo "source ~/seal_ws/install/setup.bash" >> ~/.bashrc
   ```

   To verify or edit your bashrc:

   ```bash
   gedit ~/.bashrc
   ```

   > **Note**: If you've added the source command to `~/.bashrc`, new terminals will automatically have the workspace sourced. No need to run `source` manually each time.

---

## Quick Start

After installation, launch in 2 steps (each step in a **separate terminal window**):

**Terminal 1** — Start Simulation:

```bash
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
```

**Terminal 2** — Run Autonomous Navigation:

```bash
source ~/seal_ws/install/setup.bash  # Required in each new terminal
# Choose one:
ros2 run plan apollo11    # Option A: 2D LIDAR approach
ros2 run plan atlantis_planner     # Option B: 3D LIDAR approach
```

### Expected Output

After launching the simulation, you should see the Sydney Regatta environment with the WAM-V boat:

[![VRX Simulation Environment](images/sydney_regatta_gzsim.png)](https://vimeo.com/851696025 "Gazebo Virtual RobotX v. 2.3 - Click to Watch!")
*Figure: Sydney Regatta simulation environment in Gazebo. Click to watch video demo. Source: [VRX Project](https://github.com/osrf/vrx/wiki/running_vrx_tutorial)*

The WAM-V boat will autonomously navigate through predefined waypoints while avoiding obstacles.

---

## Coordinate System

Understanding the coordinate system is essential for working with VRX simulation.

![3D Cartesian Coordinate System](images/3d_coordinate_system.jpg)
*Figure: 3D Cartesian coordinate system. Image: [Primalshell](https://commons.wikimedia.org/wiki/File:3D_Cartesian_Coodinate_Handedness.jpg), [CC BY-SA 4.0](https://creativecommons.org/licenses/by-sa/4.0/)*

### Position (x, y, z)

Coordinates in 3D space measured in **meters**:

| Axis | Direction | Maritime Term |
|:-----|:----------|:--------------|
| **X** | Forward / Backward | Ahead / Astern |
| **Y** | Left / Right | Port / Starboard |
| **Z** | Up / Down | Above / Below waterline |

### Orientation (Roll, Pitch, Yaw)

Rotation angles measured in **radians**:

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

**Note**: π/2 radians = 90°, π radians = 180°

---

## Features

### ✅ Core Capabilities

| Feature | Description |
|:--------|:------------|
| **Autonomous Navigation** | GPS-based waypoint following with lawnmower pattern |
| **Obstacle Avoidance** | Real-time LIDAR-based detection and reactive maneuvering |
| **Differential Thrust** | Independent left/right thruster control for precise steering |
| **Stuck Detection** | Automatic recovery when boat becomes trapped |
| **Web Dashboard** | Real-time monitoring interface (Vostok1 integrated & modular) |

### 🔧 Technical Highlights

- **Dual Implementations**: Apollo11 (modular) and Vostok1 (integrated) approaches
- **3D Point Cloud Processing**: Height-filtered obstacle detection
- **PID Control**: Configurable heading controller with tunable gains
- **Bilingual Interface**: Russian/English terminal output and dashboard
- **VRX Compatible**: Full integration with Virtual RobotX simulation

---

## System Architecture

### Implementation Comparison

AutoBoat provides three autonomous navigation systems:

| Aspect | Apollo11 | Vostok1 | Modular (TNO) |
|:-------|:---------|:--------|:--------------|
| **Approach** | Modular subsystems | Integrated autonomy | Distributed nodes |
| **LIDAR** | 2D LaserScan | 3D PointCloud2 | 3D PointCloud2 |
| **Detection** | Horizontal plane | Full 3D volume | Full 3D volume |
| **Control** | Direct thrust | PID-based heading | PID (configurable) |
| **Monitoring** | Terminal only | Terminal + Web | Terminal (bilingual) |
| **PID Tuning** | N/A | Parameters | Launch arguments |
| **Best For** | Simple environments | Dynamic obstacles | Custom tuning |

### Data Flow

```text
┌─────────────────────────────────────────────────────────────┐
│                    SENSORS (Gazebo)                         │
├─────────────────┬─────────────────┬─────────────────────────┤
│   GPS           │      IMU        │        LIDAR            │
│ (NavSatFix)     │    (Imu)        │  (LaserScan/PointCloud) │
└────────┬────────┴────────┬────────┴────────────┬────────────┘
         │                 │                     │
         └─────────────────┼─────────────────────┘
                           ▼
              ┌────────────────────────┐
              │   Apollo11 / Vostok1   │
              │   ──────────────────   │
              │   • Position tracking  │
              │   • Heading control    │
              │   • Obstacle avoidance │
              │   • Waypoint planning  │
              └───────────┬────────────┘
                          │
            ┌─────────────┴─────────────┐
            ▼                           ▼
    Left Thruster              Right Thruster
    (-1000 to +1000)           (-1000 to +1000)
```

### ROS 2 Topics

#### Sensor Inputs

| Topic | Type | Description |
|:------|:-----|:------------|
| `/wamv/sensors/gps/gps/fix` | `NavSatFix` | GPS coordinates (lat, lon, alt) |
| `/wamv/sensors/imu/imu/data` | `Imu` | Orientation quaternion |
| `/wamv/sensors/lidars/.../scan` | `LaserScan` | 2D LIDAR (Apollo11) |
| `/wamv/sensors/lidars/.../points` | `PointCloud2` | 3D LIDAR (Vostok1) |

#### Control Outputs

| Topic | Type | Description |
|:------|:-----|:------------|
| `/wamv/thrusters/left/thrust` | `Float64` | Left thruster command |
| `/wamv/thrusters/right/thrust` | `Float64` | Right thruster command |

#### Vostok1 Dashboard Topics

| Topic | Type | Description |
|:------|:-----|:------------|
| `/vostok1/mission_status` | `String` | Mission state (JSON) |
| `/vostok1/obstacle_status` | `String` | Obstacle data (JSON) |
| `/vostok1/config` | `String` | Current configuration (JSON, 1Hz) |
| `/vostok1/set_config` | `String` | Configuration updates from web |
| `/rosout` | `rcl_interfaces/Log` | ROS logs for terminal panel |

---

## Usage Guide

This section covers how to run the autonomous navigation systems. Choose based on your needs:

| System | Best For | Complexity | Features |
|:-------|:---------|:-----------|:---------|
| **Apollo11** | Simple testing | Low | 2D LIDAR, basic stuck recovery |
| **Vostok1** | Production use | Medium | 3D LIDAR, SASS, web dashboard |
| **Modular** | Custom tuning | High | Distributed nodes, full config |

### Before Running

Before running any navigation system, start the Gazebo simulation:

```bash
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
```

> **⚠️ Important**: Wait for Gazebo to fully load (you should see the WAM-V boat floating in the water) before running any navigation node. The boat needs GPS and sensor data to start navigating.
>
> **💡 Tip**: Each new terminal requires sourcing the workspace. See [Build & Environment](#-build--environment) in Command Cheatsheet for auto-sourcing setup via `~/.bashrc`.
>
> ```bash
> source ~/seal_ws/install/setup.bash
> ```

---

### Option A: Apollo11 (2D LIDAR)

Simple, lightweight implementation for basic testing.

```bash
ros2 run plan apollo11
```

**Terminal Output:**

```text
[INFO] Apollo 11 - Autonomous Navigation System
[INFO] Waiting for GPS signal...
[INFO] Mission Started!
[INFO] WP 1/19 | Pos: (5.2, 3.1) | Target: (15.0, 0.0) | Dist: 10.2m
```

---

### Option B: atlantis_planner (3D LIDAR + SASS)

Full-featured implementation with Smart Anti-Stuck System and web dashboard support.

```bash
ros2 run plan atlantis_planner
```

**Terminal Output :**

```text
[INFO] [1764598711.682310894] [atlantis_planner]: Atlantis Planner Started - Press ENTER to generate path
Press ENTER to generate and publish path...
```
after press enter this terminal output will follow:

---

**Terminal Output :**
[INFO] [1764598786.432758347] [atlantis_planner]: Generated Path with X waypoints
[INFO] [1764598786.433813041] [atlantis_planner]: Path generated! Controller will start driving.


---

### Option C: Modular Navigation (Vostok1 Edition)

For advanced users who need configurable PID control and modular architecture:

```bash
ros2 launch plan vostok1_modular_navigation.launch.py
```

**With Custom PID Gains:**

```bash
ros2 launch plan vostok1_modular_navigation.launch.py kp:=500.0 ki:=30.0 kd:=150.0
```

**Available Launch Parameters:**

| Parameter | Default | Description |
|:----------|:--------|:------------|
| `kp` | 400.0 | PID Proportional gain |
| `ki` | 20.0 | PID Integral gain |
| `kd` | 100.0 | PID Derivative gain |
| `base_speed` | 500.0 | Base thrust speed |
| `max_speed` | 800.0 | Maximum thrust limit |
| `min_safe_distance` | 15.0 | Obstacle safe distance (m) |
| `scan_length` | 15.0 | Lawnmower lane length (m) |
| `scan_width` | 30.0 | Lawnmower lane width (m) |
| `lanes` | 10 | Number of scan lanes |

**Modular Nodes (TNO/Post-Soviet Names):**

| Node | Name | Function |
|:-----|:-----|:---------|
| **ОКО** (Oko) | `oko_perception` | 3D LIDAR obstacle detection |
| **СПУТНИК** (Sputnik) | `sputnik_planner` | GPS waypoint planning |
| **БУРАН** (Buran) | `buran_controller` | PID heading control |

**Terminal Output:**

```text
[INFO] ОКО (OKO) - Система Обнаружения Препятствий
[INFO] СПУТНИК (SPUTNIK) - Система Планирования Маршрута
[INFO] БУРАН (BURAN) - Система Управления Движением
[INFO] ТМ 1/19 | Поз: (5.2, 3.1) | Цель: (15.0, 0.0) | Дист: 10.2m | Курс: 45°
[INFO] ✅ СВОБОДНО | CLEAR (F:50.0 L:50.0 R:50.0)
```

### Web Dashboard

The web dashboard provides real-time monitoring and control of the autonomous navigation system. It supports **both integrated and modular** navigation versions.

> 📖 **Full Documentation:** See [web_dashboard/Readme_webdashboard.md](web_dashboard/Readme_webdashboard.md) for detailed setup, customization, and troubleshooting.

| Topic Namespace | Source | Features |
|:----------------|:-------|:---------|
| `/vostok1/*` | Integrated Vostok1 | Full mission status, SASS, config |
| `/planning/*`, `/control/*`, `/perception/*` | Modular Navigation | Mission status, SASS, obstacles |

#### Requirements

Install rosbridge (once per machine):

```bash
sudo apt install ros-jazzy-rosbridge-suite
```

#### Quick Start - Integrated Vostok1 (4 Terminals)

> **Note**: Open 4 separate terminal windows. Each terminal runs one command continuously. Make sure to run `source ~/seal_ws/install/setup.bash` in each terminal first.

| Terminal | Command | Purpose |
|:---------|:--------|:--------|
| **T1** | `ros2 launch vrx_gz competition.launch.py world:=sydney_regatta` | Gazebo simulation |
| **T2** | `ros2 launch rosbridge_server rosbridge_websocket_launch.xml delay_between_messages:=0.0` | WebSocket bridge |
| **T3** | `ros2 run plan vostok1` | Vostok1 navigation |
| **T4** | `cd ~/seal_ws/src/uvautoboat/web_dashboard && python3 -m http.server 8000` | Web server |

Then open: <http://localhost:8000>

#### Quick Start - Modular Navigation (4 Terminals)

| Terminal | Command | Purpose |
|:---------|:--------|:--------|
| **T1** | `ros2 launch vrx_gz competition.launch.py world:=sydney_regatta` | Gazebo simulation |
| **T2** | `ros2 launch rosbridge_server rosbridge_websocket_launch.xml delay_between_messages:=0.0` | WebSocket bridge |
| **T3** | `ros2 launch plan vostok1_modular_navigation.launch.py` | Modular navigation |
| **T4** | `cd ~/seal_ws/src/uvautoboat/web_dashboard && python3 -m http.server 8000` | Web server |

Then open: <http://localhost:8000>

#### Dashboard Panels

| Panel | Description |
|:------|:------------|
| **Connection Status** | WebSocket connection indicator |
| **GPS Position** | Latitude, longitude, local coordinates |
| **Mission Status** | State, waypoint progress, distance |
| **Obstacle Detection** | Front/Left/Right clearance, status badge |
| **Thruster Output** | Left/Right thrust with visual bars |
| **🛡️ Anti-Stuck (SASS)** | Escape phase, no-go zones, drift, probe results |
| **Trajectory Map** | Interactive Leaflet map with boat position |
| **Configuration** | Path, PID, Speed parameter controls |
| **Terminal Output** | Live ROS log feed |
| **System Logs** | Dashboard event history |

#### Smart Anti-Stuck Panel (SASS)

The SASS panel displays real-time anti-stuck system status:

| Field | Description |
|:------|:------------|
| **Статус \| Status** | Normal (green) or STUCK with attempt count (red) |
| **Фаза \| Phase** | Current escape phase: PROBE → REVERSE → TURN → FORWARD → IDLE |
| **Направление \| Direction** | Best escape direction (← LEFT / → RIGHT) |
| **No-Go Зоны \| Zones** | Number of remembered stuck locations |
| **Дрейф \| Drift** | Estimated drift magnitude and direction |
| **История \| History** | Escape attempt records count |
| **Зонд \| Probe** | Left/Right clearance from last probe scan |

#### Visual Modes (TNO Aesthetic)

The dashboard features authentic TNO (The New Order) Cold War aesthetic with two switchable modes:

| Mode | Description | Color Palette |
|:-----|:------------|:--------------|
| **Bureau Mode** | TNO propaganda poster style | Soviet red accents, muted blue-gray (#0D0F11) |
| **Terminal Mode** | Cold War technocratic CRT | Bureau teal (#3A91A8), deep blue-gray background |

Both modes include:

- CRT scanline effects and screen flicker
- Analog signal glitch animations
- Soviet-style typography (Cyrillic headers)
- Oppressive Cold War bureaucratic atmosphere

#### Terminal Output Panel

Real-time ROS log display showing navigation feedback:

```text
[VOSTOK1] ТМ 1/19 | Поз: (5.2, 3.1) | Цель: (15.0, 0.0) | Дист: 10.2m
[VOSTOK1] ✅ СВОБОДНО | CLEAR (F:50.0 L:50.0 R:50.0)
[VOSTOK1] 🚨 ЗАСТРЯЛ! | STUCK! Smart escape initiating (Attempt 1)
[VOSTOK1] Phase 0: PROBING - L:25.0m R:18.0m → Best: LEFT
[VOSTOK1] Phase 2: TURN LEFT (escalation: 1)
[VOSTOK1] ✅ Escape successful! Resuming navigation
```

Features:

- Auto-scroll with toggle checkbox
- Clear button to reset output
- Color-coded log levels (INFO/WARN/ERROR)
- Filters for vostok1/buran/sputnik/oko nodes
- SASS escape phase logging

#### Live Parameter Configuration

The web dashboard includes a configuration panel for real-time parameter tuning:

| Section | Parameters | Description |
|:--------|:-----------|:------------|
| **Path** | Lanes, Length, Width | Lawnmower pattern configuration |
| **PID** | Kp, Ki, Kd | Heading controller gains |
| **Speed** | Base, Max, Safe Distance | Motion control parameters |

**Configuration Buttons:**

| Button | Action |
|:-------|:-------|
| **Apply** | Send all parameters, regenerate path |
| **PID Only** | Update only PID gains (immediate effect) |
| **Restart Mission** | Reset waypoint index and regenerate path |

**Command Line Parameter Tuning:**

```bash
# Set parameters at launch
ros2 run plan vostok1 --ros-args -p kp:=500.0 -p ki:=30.0 -p kd:=150.0
```

```bash
# Set parameters at runtime
ros2 param set /vostok1_node kp 500.0
ros2 param set /vostok1_node ki 30.0
ros2 param set /vostok1_node kd 150.0
```

```bash
# List all parameters
ros2 param list /vostok1_node
```

---

## Smart Anti-Stuck System (SASS)

The Vostok series (Vostok1 and Modular/Buran) implements an advanced **Smart Anti-Stuck System (SASS)** that enables intelligent recovery when the boat becomes trapped or immobilized.

### Evolution of Anti-Stuck Strategies

| Feature | Apollo11 (Basic) | Vostok1/Buran (SASS v2.0) |
|:--------|:-----------------|:--------------------------|
| **Detection** | Fixed 5s interval, 1m threshold | Configurable 3s interval, 0.5m threshold |
| **Escape Duration** | Fixed 4s (reverse + turn) | Adaptive 10-20s based on situation |
| **Turn Direction** | Fixed rotation | Multi-direction probe + LIDAR-guided |
| **Memory** | None | No-go zones (remembers stuck locations) |
| **Drift Compensation** | None | Real-time current/wind compensation |
| **Learning** | None | Records successful escapes |
| **Detour Waypoints** | None | Auto-inserts detour waypoints |
| **Skip Logic** | After 3 attempts | After 4 attempts (more patient) |

### SASS Features

#### 1. Adaptive Escape Duration

The escape duration adjusts based on the situation severity:

```text
Base Duration: 10s
+ 4s if obstacle < critical distance (5m)
+ 2s if obstacle < safe distance (15m)  
+ 2s per consecutive stuck attempt
Maximum: 20s
```

#### 2. Multi-Direction Probe

Before committing to an escape direction, SASS probes left, right, and backward:

| Phase | Time | Action |
|:------|:-----|:-------|
| Probe Left | 0-0.6s | Turn left, record max clearance |
| Probe Right | 0.6-1.2s | Turn right, record max clearance |
| Assess | 1.2-2.0s | Compare results, select best direction |

#### 3. No-Go Zone Memory

SASS remembers where the boat got stuck and avoids those areas:

- **Zone Radius**: 8m (configurable)
- **Zone Expansion**: Grows if stuck nearby again
- **Max Zones**: 20 (oldest removed to prevent memory issues)
- **Forward Check**: Before moving forward, checks if heading leads to no-go zone

#### 4. Drift/Current Compensation

Estimates and compensates for environmental drift:

```text
1. Track position history (last 100 samples at 20Hz)
2. Calculate drift vector using exponential moving average
3. Apply thrust bias to counteract drift during escape
```

#### 5. Detour Waypoint Insertion

On the 2nd consecutive stuck attempt, SASS inserts a detour waypoint:

```text
1. Calculate detour perpendicular to obstacle (12m offset)
2. Verify detour doesn't lead to no-go zone
3. Insert waypoint before current target
4. Resume normal navigation through detour
```

#### 6. Simple Escape Learning

SASS learns from successful escapes:

- Records: direction, success/failure, obstacle distance
- Stores last 50 escape attempts
- Recent successes weighted 2x when choosing direction
- Falls back to learned direction when probe results are ambiguous

### SASS Escape Sequence

| Phase | Duration | Action | Details |
|:------|:---------|:-------|:--------|
| **0: PROBE** | 0-2s | Multi-direction scan | L/R/Back clearance assessment |
| **1: REVERSE** | 2s-~6s | Backward thrust | Adaptive power based on obstacle distance |
| **2: TURN** | 6s-~10s | Rotate | Direction from probing, escalating power |
| **3: FORWARD** | 10s-~12s | Forward test | With drift compensation, no-go zone check |

### SASS Parameters (Configurable)

| Parameter | Default | Description |
|:----------|:--------|:------------|
| `stuck_timeout` | 3.0s | Time window to detect stuck |
| `stuck_threshold` | 0.5m | Minimum movement to not be stuck |
| `no_go_zone_radius` | 8.0m | Radius of avoided zones |
| `drift_compensation_gain` | 0.3 | Drift correction aggressiveness |
| `detour_distance` | 12.0m | Distance for detour waypoints |

### Web Dashboard SASS Panel

The web dashboard displays real-time SASS status:

| Field | Description |
|:------|:------------|
| **Status** | Normal / STUCK (with attempt count) |
| **Phase** | Current escape phase (PROBE/REVERSE/TURN/FORWARD/IDLE) |
| **Direction** | Best escape direction (LEFT/RIGHT) |
| **No-Go Zones** | Number of remembered stuck locations |
| **Drift** | Estimated drift magnitude and direction |
| **History** | Number of recorded escape attempts |
| **Probe** | Left/Right clearance from last probe |

---

## Technical Documentation

### GPS Navigation

The system uses GPS for absolute position tracking with local coordinate conversion using equirectangular projection:

```text
Local X = (latitude - start_lat) × Earth_radius
Local Y = (longitude - start_lon) × Earth_radius × cos(start_lat)
```

**Key Concepts:**

- First GPS fix becomes local origin (0, 0)
- Waypoints defined in local meters from origin
- Earth radius: 6,371,000 meters
- See [Coordinate System](#coordinate-system) for axis orientation

**GPS Message Structure** (`sensor_msgs/NavSatFix`):

| Field | Type | Range | Description |
|:------|:-----|:------|:------------|
| latitude | float64 | -90° to +90° | Degrees north/south |
| longitude | float64 | -180° to +180° | Degrees east/west |
| altitude | float64 | meters | Height above sea level |

### IMU (Inertial Measurement Unit)

The IMU provides the boat's orientation (heading) using a combination of accelerometers and gyroscopes. The WAM-V's IMU publishes data to `/wamv/sensors/imu/imu/data`.

**How the System Uses IMU:**

1. **Heading Calculation**: Extracts yaw angle from quaternion orientation
2. **Target Heading**: Calculates desired heading to next waypoint using `atan2(dy, dx)`
3. **Heading Error**: Computes difference between current and target heading
4. **PID Control**: Uses heading error to adjust differential thrust

**Quaternion to Yaw Conversion:**

```text
siny_cosp = 2 × (w × z + x × y)
cosy_cosp = 1 - 2 × (y² + z²)
yaw = atan2(siny_cosp, cosy_cosp)
```

**IMU Message Structure** (`sensor_msgs/Imu`):

| Field | Type | Description |
|:------|:-----|:------------|
| orientation | Quaternion | x, y, z, w components |
| angular_velocity | Vector3 | Roll, pitch, yaw rates (rad/s) |
| linear_acceleration | Vector3 | X, Y, Z acceleration (m/s²) |

**Note**: The system primarily uses the `orientation` quaternion for heading control. Angular velocity and acceleration are available but not currently used.

### 3D LIDAR Processing (Vostok1)

**Point Cloud Filtering Pipeline:**

1. **Height Filter**: Keep points between -0.2m and 3.0m
   - Excludes water surface reflections
   - Excludes sky and distant objects

2. **Distance Filter**: Within configurable detection range (default: 50m)

3. **Sector Analysis**:

   | Sector | Angle Range | Purpose |
   |:-------|:------------|:--------|
   | Front | -45° to +45° | Forward obstacle detection |
   | Left | +45° to +135° | Left-side clearance |
   | Right | -135° to -45° | Right-side clearance |

**Decision Logic:**

```text
IF min_front_distance < safety_threshold:
    IF left_clearance > right_clearance:
        Turn LEFT
    ELSE:
        Turn RIGHT
```

**Hysteresis**: Entry threshold differs from exit threshold to prevent detection flickering.

### Differential Thrust Control

The WAM-V uses two independent thrusters for differential steering:

| Maneuver | Left Thruster | Right Thruster | Result |
|:---------|:--------------|:---------------|:-------|
| Forward | +500 | +500 | Straight ahead |
| Reverse | -500 | -500 | Straight back |
| Turn Left | +200 | +500 | Gradual left turn |
| Turn Right | +500 | +200 | Gradual right turn |
| Spin Left | -500 | +500 | Rotate in place |
| Spin Right | +500 | -500 | Rotate in place |

**Thrust Range**: -1000 to +1000 Newtons

### PID Heading Control (Vostok1)

Vostok1 uses a PID controller for smooth heading adjustments:

```text
error = target_heading - current_heading
correction = Kp × error + Ki × ∫error + Kd × d(error)/dt
```

**Default Gains:**

| Parameter | Value | Effect |
|:----------|:------|:-------|
| Kp | 800 | Proportional response |
| Ki | 50 | Integral accumulation |
| Kd | 100 | Derivative damping |

---

## Testing

### Quick Validation

**Step 1** — Launch Simulation:

```bash
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
```

**Step 2** — Run Navigation:

```bash
ros2 run plan vostok1
```

**Step 3** — Monitor Topics:

```bash
# Verify thruster commands
ros2 topic echo /wamv/thrusters/left/thrust
```

```bash
# Verify GPS data
ros2 topic echo /wamv/sensors/gps/gps/fix
```

```bash
# Verify LIDAR data
ros2 topic echo /wamv/sensors/lidars/lidar_wamv_sensor/points --no-arr
```

### Expected Behavior

| Stage | Expected Output |
|:------|:----------------|
| Startup | "МИССИЯ НАЧАТА! (Mission Started!)" |
| Navigation | "ТМ X/19 \| Поз: (x, y) \| Цель: (tx, ty)" |
| Obstacle | "🚨 ПРЕПЯТСТВИЕ! \| OBSTACLE DETECTED!" |
| Clear | "✅ Путь свободен \| Path CLEAR" |
| **Stuck Detected** | "🚨 ЗАСТРЯЛ! \| STUCK! Smart escape initiating" |
| **Escape Phase** | "Phase 0: PROBING" → "Phase 1: REVERSE" → "Phase 2: TURN" → "Phase 3: FORWARD" |
| **Escape Success** | "✅ Escape successful! Resuming navigation" |
| Complete | "МИССИЯ ЗАВЕРШЕНА! (MISSION COMPLETE!)" |

### Testing SASS (Anti-Stuck System)

To test the Smart Anti-Stuck System:

1. **Create obstacle scenario**: Place obstacles in Gazebo that trap the boat
2. **Monitor stuck detection**: Watch for "ЗАСТРЯЛ!" messages
3. **Observe escape phases**: Verify 4-phase escape sequence
4. **Check no-go zones**: Boat should avoid returning to stuck locations

```bash
# Monitor anti-stuck status (Vostok1)
ros2 topic echo /vostok1/anti_stuck_status
```

```bash
# Monitor anti-stuck status (Modular/Buran)
ros2 topic echo /control/anti_stuck_status
```

### Performance Benchmarks

| Metric | Target | Achieved |
|:-------|:-------|:---------|
| Waypoint Accuracy | < 3m | ~2m |
| Control Loop Rate | 20 Hz | 20 Hz |
| Detection Range | 50m | 50m |
| Collision Avoidance | 100% | 100% |

### Unit Tests

```bash
cd ~/seal_ws
colcon test --packages-select plan control
colcon test-result --verbose
```

---

## Troubleshooting

### Common Issues

#### Boat Not Moving

**Symptoms**: Gazebo running but boat stationary.

**Solutions**:

1. Check GPS signal:

   ```bash
   ros2 topic echo /wamv/sensors/gps/gps/fix
   ```

2. Check thruster commands:

   ```bash
   ros2 topic echo /wamv/thrusters/left/thrust
   ```

3. Verify node is running:

   ```bash
   ros2 node list | grep -E "apollo|vostok"
   ```

4. Check if waypoints are loaded:

   ```bash
   ros2 topic echo /vostok1/mission_status --once
   ```

5. Verify LIDAR is publishing (obstacle detection may be blocking):

   ```bash
   ros2 topic hz /wamv/sensors/lidars/lidar_wamv/points
   ```

#### Boat Spinning in Circles

**Symptoms**: Boat rotates continuously without making progress.

**Solutions**:

1. Check IMU orientation data:

   ```bash
   ros2 topic echo /wamv/sensors/imu/imu/data --field orientation
   ```

2. Verify heading calculation - yaw should be in radians:

   ```bash
   ros2 topic echo /wamv/sensors/imu/imu/data --field orientation.z
   ```

3. Check for reversed thruster commands (left/right swapped):

   ```bash
   ros2 topic echo /wamv/thrusters/left/thrust &
   ros2 topic echo /wamv/thrusters/right/thrust &
   ```

4. Reduce PID gains if oscillating:

   ```bash
   ros2 param set /vostok1_node kp 0.3
   ros2 param set /vostok1_node kd 0.05
   ```

#### Stuck Detection Not Working

**Symptoms**: Boat gets stuck but doesn't trigger escape maneuver.

**Solutions**:

1. Check SASS status:

   ```bash
   ros2 topic echo /vostok1/anti_stuck_status
   # or for modular:
   ros2 topic echo /control/anti_stuck_status
   ```

2. Verify GPS is updating (stuck detection uses position delta):

   ```bash
   ros2 topic hz /wamv/sensors/gps/gps/fix
   ```

3. Check stuck threshold parameters:

   ```bash
   ros2 param get /vostok1_node stuck_distance_threshold
   ros2 param get /vostok1_node stuck_time_threshold
   ```

4. Monitor position changes manually:

   ```bash
   watch -n 1 "ros2 topic echo /wamv/sensors/gps/gps/fix --once 2>/dev/null | grep -E 'latitude|longitude'"
   ```

#### Web Dashboard Not Connecting

**Symptoms**: Dashboard shows "Disconnected" or no data updating.

**Solutions**:

1. Verify rosbridge is running:

   ```bash
   ros2 node list | grep rosbridge
   ```

2. Check rosbridge port (default 9090):

   ```bash
   ss -tlnp | grep 9090
   ```

3. Restart rosbridge:

   ```bash
   pkill -f rosbridge
   ros2 launch rosbridge_server rosbridge_websocket_launch.xml delay_between_messages:=0.0
   ```

4. Clear browser cache (Ctrl+Shift+R) or use incognito mode

5. Check browser console for WebSocket errors (F12 → Console)

6. Verify correct URL in browser: `http://localhost:8000` (not https)

7. Check firewall isn't blocking ports 8000 or 9090:

   ```bash
   sudo ufw status
   ```

#### LIDAR Not Detecting Obstacles

**Symptoms**: Boat collides with obstacles despite LIDAR being active.

**Solutions**:

1. Verify LIDAR topic is publishing:

   ```bash
   ros2 topic hz /wamv/sensors/lidars/lidar_wamv/points
   ```

2. Check point cloud has data:

   ```bash
   ros2 topic echo /wamv/sensors/lidars/lidar_wamv/points --field height
   ```

3. Verify obstacle detection parameters:

   ```bash
   ros2 param get /vostok1_node obstacle_distance_threshold
   ros2 param get /vostok1_node min_obstacle_height
   ros2 param get /vostok1_node max_obstacle_height
   ```

4. Visualize in RViz2:

   ```bash
   ros2 run rviz2 rviz2 -d ~/seal_ws/src/uvautoboat/plan/rviz/default.rviz
   ```

5. Check TF transforms are valid:

   ```bash
   ros2 run tf2_tools view_frames
   ```

#### Build Failures

**Solutions**:

1. Install missing dependencies:

   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

2. Clean and rebuild:

   ```bash
   rm -rf build install log
   colcon build --merge-install
   ```

3. Check Python package versions:

   ```bash
   pip list | grep -E "numpy|transforms3d|rclpy"
   ```

4. Source the workspace after building:

   ```bash
   source install/setup.bash
   ```

5. Check for conflicting installations:

   ```bash
   pip check
   ```

#### ROS 2 Nodes Not Found

**Symptoms**: `ros2 run plan vostok1` fails with "Package not found".

**Solutions**:

1. Verify workspace is sourced:

   ```bash
   echo $AMENT_PREFIX_PATH | grep seal_ws
   ```

2. Re-source after every new terminal:

   ```bash
   source ~/seal_ws/install/setup.bash
   ```

3. Add to `.bashrc` for automatic sourcing:

   ```bash
   echo "source ~/seal_ws/install/setup.bash" >> ~/.bashrc
   ```

4. Verify package is installed:

   ```bash
   ros2 pkg list | grep -E "plan|control"
   ```

5. Check entry points in `setup.py`:

   ```bash
   cat ~/seal_ws/src/uvautoboat/plan/setup.py | grep console_scripts -A 10
   ```

#### Gazebo Crashes on Startup

**Symptoms**: Gazebo window opens then immediately closes.

**Solutions**:

1. Check GPU drivers:

   ```bash
   nvidia-smi  # or glxinfo | grep "OpenGL"
   ```

2. Run with software rendering:

   ```bash
   export LIBGL_ALWAYS_SOFTWARE=1
   ros2 launch vrx_gz competition.launch.py
   ```

3. Check available memory:

   ```bash
   free -h
   ```

4. Kill zombie processes:

   ```bash
   pkill -9 -f gazebo && pkill -9 -f gz && pkill -9 -f ruby
   ```

5. Clear Gazebo cache:

   ```bash
   rm -rf ~/.gz/fuel ~/.gazebo/models
   ```

6. Check for missing models (first run downloads them):

   ```bash
   gz fuel download -u https://fuel.gazebosim.org/1.0/OpenRobotics/models/sydney_regatta
   ```

#### Simulation Performance

| Issue | Solution |
|:------|:---------|
| Low FPS | Reduce Gazebo graphics quality |
| Crashes | Verify 8GB+ RAM available |
| Lag | Use headless mode for testing |
| GPU overheating | Limit FPS with `--render-engine-server-plugin` |
| High CPU | Close unnecessary applications |

**Headless Mode:**

```bash
ros2 launch vrx_gz competition.launch.py headless:=true
```

**Reduce Physics Rate (if lagging):**

```bash
# Lower real-time factor
gz service -s /world/sydney_regatta/set_physics --reqtype gz.msgs.Physics --reptype gz.msgs.Boolean --req "real_time_factor: 0.5"
```

### Environment Issues

#### VRX/Gazebo Not Installed

**Solutions**:

1. Follow VRX installation guide:

   ```bash
   # Add OSRF packages
   sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list'
   wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
   sudo apt update
   ```

2. Install VRX packages:

   ```bash
   sudo apt install ros-jazzy-vrx-gz
   ```

3. Verify installation:

   ```bash
   ros2 pkg list | grep vrx
   ```

#### Python Import Errors

**Symptoms**: `ModuleNotFoundError` when running nodes.

**Solutions**:

1. Install missing Python packages:

   ```bash
   pip install numpy transforms3d scipy
   ```

2. For ROS packages:

   ```bash
   sudo apt install ros-jazzy-tf-transformations python3-transforms3d
   ```

3. Check Python path:

   ```bash
   python3 -c "import sys; print('\n'.join(sys.path))"
   ```

### Debug Tips

#### Enable Verbose Logging

```bash
# Set log level for specific node
ros2 run plan vostok1 --ros-args --log-level debug

# Or via environment variable
export RCUTILS_LOGGING_MIN_SEVERITY=DEBUG
ros2 run plan vostok1
```

#### Record a Bag for Analysis

```bash
# Record all relevant topics
ros2 bag record /wamv/sensors/gps/gps/fix \
                /wamv/sensors/imu/imu/data \
                /wamv/sensors/lidars/lidar_wamv/points \
                /wamv/thrusters/left/thrust \
                /wamv/thrusters/right/thrust \
                /vostok1/mission_status \
                /vostok1/anti_stuck_status \
                -o debug_session
```

#### Replay for Debugging

```bash
ros2 bag play debug_session --clock
```

#### Monitor System Resources

```bash
# Watch CPU/Memory usage
htop

# Monitor GPU (NVIDIA)
watch -n 1 nvidia-smi

# Check disk space
df -h
```

---

## Command Cheatsheet

### 🔴 Kill Processes

```bash
# Kill all Gazebo processes completely
pkill -9 -f gazebo && pkill -9 -f gz && pkill -9 -f ruby
```

```bash
# Kill specific ROS 2 nodes
pkill -9 -f apollo11
pkill -9 -f vostok1
pkill -9 -f rosbridge
```

```bash
# Kill rosbridge and verify port 9090 is released
pkill -9 -f rosbridge && sleep 1 && ss -tlnp | grep 9090
# (no output = port is free)
```

```bash
# Kill all ROS 2 daemon (if nodes not responding)
ros2 daemon stop
ros2 daemon start
```

```bash
# Nuclear option - kill everything ROS/Gazebo related
pkill -9 -f ros && pkill -9 -f gz && pkill -9 -f gazebo && pkill -9 -f ruby
```

### 📊 ROS 2 Introspection

```bash
# List all active nodes
ros2 node list
```

```bash
# Get info about a specific node
ros2 node info /vostok1_node
```

```bash
# List all active topics
ros2 topic list
```

```bash
# Monitor topic data in real-time
ros2 topic echo /wamv/sensors/gps/gps/fix
ros2 topic echo /wamv/thrusters/left/thrust
```

```bash
# Check topic publishing frequency
ros2 topic hz /wamv/sensors/gps/gps/fix
```

```bash
# List all parameters of a node
ros2 param list /vostok1_node
```

```bash
# Get a parameter value
ros2 param get /vostok1_node kp
```

```bash
# Set a parameter at runtime
ros2 param set /vostok1_node kp 500.0
```

### 🔧 Build & Environment

```bash
# Source ROS 2 environment (required in each new terminal)
source /opt/ros/jazzy/setup.bash
source ~/seal_ws/install/setup.bash
```

```bash
# Edit bashrc to add auto-sourcing (so you don't need to source manually)
gedit ~/.bashrc
# Add these lines at the end of the file:
#   export PATH="$HOME/.local/bin:$PATH"
#   source /opt/ros/jazzy/setup.bash
#   source ~/seal_ws/install/setup.bash
#   export ROS_DOMAIN_ID=56 # The domain ID for VRX simulation can be any number between 0-1013.
```

```bash
# Or add to bashrc via command line (one-time setup)
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/seal_ws/install/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=56" >> ~/.bashrc # The domain ID for VRX simulation can be any number between 0-1013.
```

```bash
# Build all packages
cd ~/seal_ws && colcon build --merge-install
```

```bash
# Build specific package only
colcon build --packages-select plan control
```

```bash
# Build with symlink (faster rebuilds for Python)
colcon build --symlink-install
```

```bash
# Build with parallel jobs (faster on multi-core)
colcon build --parallel-workers 4
```

```bash
# Build with verbose output (debugging build issues)
colcon build --event-handlers console_direct+
```

```bash
# Build with CMake arguments (e.g., Release mode)
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

```bash
# Build only packages that changed
colcon build --packages-up-to plan
```

```bash
# Clean build (when things go wrong)
rm -rf build install log && colcon build --merge-install
```

```bash
# Clean specific package only
rm -rf build/plan install/plan && colcon build --packages-select plan
```

```bash
# Install dependencies
rosdep install --from-paths src --ignore-src -r -y
```

```bash
# Check package dependencies
rosdep check --from-paths src --ignore-src
```

### 🚀 Launch Commands

```bash
# Start Gazebo simulation
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
```

```bash
# Start Gazebo headless (no GUI, faster)
ros2 launch vrx_gz competition.launch.py headless:=true
```

```bash
# Run autonomous navigation
ros2 run plan vostok1
ros2 run plan apollo11
```

```bash
# Launch modular navigation with custom PID
ros2 launch plan vostok1_modular_navigation.launch.py kp:=500.0 ki:=30.0 kd:=150.0
```

```bash
# Start rosbridge for web dashboard (delay_between_messages fixes Jazzy bug)
ros2 launch rosbridge_server rosbridge_websocket_launch.xml delay_between_messages:=0.0
```

```bash
# Start web server for dashboard
cd ~/seal_ws/src/uvautoboat/web_dashboard && python3 -m http.server 8000
```

```bash
# Teleport boat to starting position (reset without restarting simulation)
gz service -s /world/sydney_regatta/set_pose \
  --reqtype gz.msgs.Pose \
  --reptype gz.msgs.Boolean \
  --timeout 1000 \
  --req 'name: "wamv", position: {x: 0, y: 0, z: 0.5}, orientation: {x: 0, y: 0, z: 0, w: 1}'
```

### 🐛 Debugging

```bash
# Check if GPS is publishing
ros2 topic echo /wamv/sensors/gps/gps/fix --once
```

```bash
# Check LIDAR data (without array flood)
ros2 topic echo /wamv/sensors/lidars/lidar_wamv_sensor/points --no-arr
```

```bash
# View TF tree
ros2 run tf2_tools view_frames
```

```bash
# Check transform between frames
ros2 run tf2_ros tf2_echo world wamv/wamv/base_link
```

```bash
# Run unit tests
colcon test --packages-select plan control
colcon test-result --verbose
```

### 🖥️ RQT Tools

> 📚 RQT is a Qt-based GUI framework for ROS 2. See [RQT Documentation](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-RQt.html)

```bash
# Launch RQT main window (access all plugins from menu)
rqt
```

```bash
# Topic Monitor - view all topics and their data
rqt_topic
```

```bash
# Node Graph - visualize node connections
rqt_graph
```

```bash
# Plot - real-time data plotting
rqt_plot /wamv/thrusters/left/thrust/data /wamv/thrusters/right/thrust/data
```

```bash
# Console - view ROS logs with filtering
rqt_console
```

```bash
# TF Tree - visualize transform hierarchy
rqt_tf_tree
```

```bash
# Image View - display camera feeds
rqt_image_view
```

```bash
# Parameter Reconfigure - tune parameters live
rqt_reconfigure
```

> 💡 **Better Plotting UI:** For advanced real-time plotting, install PlotJuggler:
>
> ```bash
> sudo apt install ros-jazzy-plotjuggler-ros
> ros2 run plotjuggler plotjuggler
> ```
>
> Then: **Streaming → ROS2 Topic Subscriber → Start → Select topics → Drag to plot**

### 💾 Git Quick Commands

> 📚 For comprehensive Git documentation, see [Git Reference](https://git-scm.com/docs)

```bash
# Check status
git status
```

```bash
# Stage and commit
git add -A && git commit -m "Your message"
```

```bash
# Push to remote
git push origin main
```

```bash
# Pull latest changes
git pull origin main
```

```bash
# Discard local changes
git checkout -- .
```

```bash
# Stash changes temporarily (save work without committing)
git stash
```

```bash
# Stash with a descriptive message
git stash push -m "WIP: feature description"
```

```bash
# List all stashes
git stash list
```

```bash
# Apply most recent stash (keep stash in list)
git stash apply
```

```bash
# Apply and remove most recent stash
git stash pop
```

```bash
# Apply a specific stash
git stash apply stash@{1}
```

```bash
# Drop a specific stash
git stash drop stash@{0}
```

```bash
# Clear all stashes
git stash clear
```

---

## Contributing

This project is developed as part of the ROS 2 Autonomous Systems course at IMT Nord Europe.

### Development Guidelines

1. **Code Style**: Follow PEP 8 for Python
2. **Documentation**: Update README for significant changes
3. **Testing**: Include unit tests for new features
4. **Commits**: Use clear, descriptive commit messages

### Reporting Issues

Open an issue on [GitHub](https://github.com/Erk732/uvautoboat/issues) with:

- Problem description
- Steps to reproduce
- Expected vs actual behavior
- System information (OS, ROS version, etc.)

---

## References

### Documentation

- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [Gazebo Harmonic Documentation](https://gazebosim.org/docs/harmonic)
- [VRX Wiki](https://github.com/osrf/vrx/wiki)
- [RQT Tools Documentation](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-RQt.html)
- [Git Reference Manual](https://git-scm.com/docs)

### Message Types

- [sensor_msgs/NavSatFix](http://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html)
- [sensor_msgs/PointCloud2](http://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html)
- [sensor_msgs/Imu](http://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html)

### Related Projects

- [Virtual RobotX (VRX)](https://github.com/osrf/vrx)
- [ros2_control](https://github.com/ros-controls/ros2_control)

---

## Acknowledgments

- **Open Source Robotics Foundation**: VRX simulation environment
- **IMT Nord Europe**: Academic support and guidance
- **Development Teams**:
  - Apollo11 — Planning Team
  - Vostok1 — Control Team

---

## License

This project is licensed under the Apache License 2.0.

See [LICENSE](LICENSE) for details.

---

**AutoBoat** — Autonomous Navigation for VRX Competition

Built with ROS 2 Jazzy + Gazebo Harmonic

[Report Bug](https://github.com/Erk732/uvautoboat/issues) · [Request Feature](https://github.com/Erk732/uvautoboat/issues)
