# AutoBoat (uvautoboat)

[![ROS 2 Version](https://img.shields.io/badge/ROS_2-Jazzy-blue)](https://docs.ros.org/en/jazzy/)
[![Gazebo Version](https://img.shields.io/badge/Gazebo-Harmonic-orange)](https://gazebosim.org/)
![ROS 2 CI](https://github.com/osrf/vrx/workflows/ROS%202%20CI/badge.svg)

## Abstract

This repository contains an autonomous navigation system for unmanned surface vehicles (USVs) designed for the Virtual RobotX (VRX) competition. The system implements path planning, obstacle avoidance, and control algorithms for the WAM-V maritime platform using ROS 2 and Gazebo simulation environments.

## Overview

AutoBoat is a comprehensive autonomous navigation framework developed for the VRX project. The system provides intelligent path planning capabilities for WAM-V surface vessels by integrating perception, planning, and control modules. The architecture processes real-time odometry data and mission objectives while accounting for environmental constraints such as static obstacles and operational boundaries to generate safe, optimal trajectories.

## Core Capabilities

- **Point-to-Point Navigation**: Efficient trajectory generation between specified waypoints using optimization-based planning algorithms.
- **Coverage Planning**: Systematic area coverage using boustrophedon (lawn-mower) patterns for search and surveillance missions.
- **Obstacle Avoidance**: Dynamic path replanning using A* (A-Star) algorithm with grid-based environmental representation to avoid static obstacles including buoys, islands, and other maritime structures.

---

## System Architecture

The navigation system follows a hierarchical architecture with three primary layers:

- **Mission Layer**: High-level task coordination and goal specification
- **Planning Layer**: Path generation and obstacle avoidance (`plan` package)
- **Control Layer**: Trajectory tracking and thruster control (`control` package)

The planning module serves as the bridge between mission objectives and vehicle control, transforming high-level goals into executable trajectories.

### ROS 2 Communication Interfaces

| Topic Name | Message Type | I/O | Description |
| :--- | :--- | :--- | :--- |
| `/wamv/sensors/odometry` | `nav_msgs/Odometry` | Sub | Current boat position/orientation. |
| `/planning/goal` | `geometry_msgs/PoseStamped` | Sub | Desired destination. |
| `/planning/path` | `nav_msgs/Path` | Pub | Computed trajectory (waypoints). |

---

## Installation

### System Requirements

- **Operating System**: Ubuntu 22.04 LTS or Ubuntu 24.04 LTS
- **Python**: Version 3.10 or higher
- **Memory**: Minimum 8GB RAM (16GB recommended)
- **Storage**: At least 10GB available disk space

### Prerequisites

⚠️ **Important**: This package has been tested with ROS 2 Jazzy and Gazebo Harmonic. Using different versions may result in compatibility issues or unexpected behavior.

**Required Software:**

- [ROS 2 Jazzy Jalisco](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) - Robot Operating System 2
- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install_ubuntu/) - Physics-based simulation environment
- [VRX Simulation](https://github.com/osrf/vrx) - Virtual RobotX competition environment
- Python 3.10+ with pip

**Python Dependencies:**

The following Python packages are automatically installed during the build process:

- `numpy` - Numerical computing
- `matplotlib` - Visualization (optional)
- Other dependencies specified in package configurations

### Installation Instructions

⚠️ **Note**: Please follow these instructions carefully. Skipping steps may result in build failures or runtime errors.

1. **Create and initialize workspace**

   Create a ROS 2 workspace and clone this repository into the `src` directory:

   ```bash
   mkdir -p ~/seal_ws/src
   cd ~/seal_ws/src
   git clone https://github.com/Erk732/uvautoboat.git
   ```

2. **Clone VRX dependencies**

   The VRX simulation environment is required for testing:

   ```bash
   cd ~/seal_ws/src
   git clone https://github.com/osrf/vrx.git
   ```

3. **Source ROS 2 environment**

   Source the ROS 2 installation (default location: `/opt/ros/jazzy/setup.bash`):

   ```bash
   source /opt/ros/jazzy/setup.bash
   ```

4. **Build the workspace**

   Navigate to the workspace root and build all packages:

   ```bash
   cd ~/seal_ws
   colcon build --merge-install
   ```

   For parallel builds (faster compilation):

   ```bash
   colcon build --merge-install --parallel-workers 4
   ```

5. **Source the workspace**

   After successful build, source the workspace overlay:

   ```bash
   source ~/seal_ws/install/setup.bash
   ```

   **Tip**: Add this line to your `~/.bashrc` for automatic sourcing:

   ```bash
   echo "source ~/seal_ws/install/setup.bash" >> ~/.bashrc
   ```

## Usage

### Quick Start

Launch the complete navigation system with the default Sydney Regatta environment:

```bash
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
```

In a separate terminal, launch the navigation demo:

```bash
ros2 launch plan demo.launch.py
```

### Running Individual Components

**Planning Nodes:**

```bash
# A* path planner with obstacle avoidance
ros2 run plan astar_planner

# Time-stamped dynamic obstacle avoidance
ros2 run plan avoidingobs_ts_planner

# Perception and obstacle detection
ros2 run plan simple_perception

# Mission coordination
ros2 run plan mission_trigger
```

**Control Nodes:**

```bash
# Simple thruster controller
ros2 run control simple_controller

# Path following controller
ros2 run control path_follower
```

## Simulation Environment

### Default Environment

The system is designed to operate in the standard VRX Sydney Regatta environment:

```bash
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
```

### Legacy Test Environment (Deprecated)

⚠️ **Deprecation Notice**: The custom test environment is maintained for backward compatibility but is no longer actively developed. Users are advised to utilize the default Sydney Regatta environment for current development and testing.

The `test_environment` directory contains legacy simulation worlds and custom models:

**Custom Worlds:**

- `sydney_regatta_custom.sdf` - Modified environment with additional obstacles and test scenarios

**Custom Models:**

- `cardboardbox/` - Simple obstacle model for collision avoidance testing

**Legacy Environment Launch** (Optional):

1. Configure Gazebo resource path:

   ```bash
   export GZ_SIM_RESOURCE_PATH=$HOME/seal_ws/src/uvautoboat/test_environment:$GZ_SIM_RESOURCE_PATH
   ```

2. Launch custom environment:

   ```bash
   source ~/seal_ws/install/setup.bash
   ros2 launch vrx_gz competition.launch.py world:=sydney_regatta_custom
   ```

### Expected Output

After launching the simulation environment, you should observe the following:

[![VRX Simulation Environment](images/sydney_regatta_gzsim.png)](https://vimeo.com/851696025 "Gazebo Virtual RobotX v. 2.3 - Click to Watch!")

*Figure 1: Sydney Regatta simulation environment in Gazebo. Source: [VRX Project](https://github.com/osrf/vrx/wiki/running_vrx_tutorial)*

---

## Package Documentation

### Plan Package

The `plan` package implements the planning and perception subsystems for autonomous navigation. This package provides multiple planning algorithms and supporting utilities for environment representation and obstacle detection.

#### Planning Nodes

**1. A* Planner** (`astar_planner`)

Implements the A* search algorithm for optimal path planning with obstacle avoidance. Uses grid-based environmental representation with configurable resolution and inflation radius.

**Run:**

```bash
ros2 run plan astar_planner
```

**2. Time-Stamped Obstacle Avoidance Planner** (`avoidingobs_ts_planner`)

Enhanced planning algorithm incorporating temporal information for dynamic obstacle prediction and avoidance.

**Run:**

```bash
ros2 run plan avoidingobs_ts_planner
```

**3. Perception Module** (`simple_perception`)

Processes sensor data streams to construct environmental representations, including obstacle detection and classification.

**Run:**

```bash
ros2 run plan simple_perception
```

**4. Mission Coordinator** (`mission_trigger`)

High-level mission management node responsible for goal specification and planning behavior coordination.

**Run:**

```bash
ros2 run plan mission_trigger
```

**5. Transform Broadcaster** (`tf_broadcaster`)

Manages coordinate frame transformations required for navigation stack operation.

**Run:**

```bash
ros2 run plan tf_broadcaster
```

### Launch Files

**Demo Launch:**

```bash
ros2 launch plan demo.launch.py
```

### Plan Package Modules

- `astar_planner.py` - A* path planning implementation
- `avoidingOBS_planner.py` - Obstacle avoidance planner
- `avoidingobs_ts_planner.py` - Time-stamped obstacle avoidance
- `grid_map.py` - Grid-based environment representation
- `simple_perception.py` - Obstacle detection and processing
- `mission_trigger.py` - Mission coordination
- `tf_broadcaster.py` - Transform broadcasting
- `FREE.py` - Free space utilities
- `OUT.py` - Out-of-bounds detection

---

### Control Package

The `control` package implements trajectory tracking and low-level thruster control for the WAM-V platform.

#### Control Nodes

**1. Simple Controller** (`simple_controller`)

Baseline controller implementation for thruster command testing and validation.

**Run:**

```bash
ros2 run control simple_controller
```

**Launch file:**

```bash
ros2 launch control simple_controller.launch.py
```

**2. Path Following Controller** (`path_follower`)

Advanced trajectory tracking controller implementing waypoint following with feedback control for precise path execution.

**Run:**

```bash
ros2 run control path_follower
```

### Control Interfaces

| Topic Name | Message Type | I/O | Description |
| :--- | :--- | :--- | :--- |
| `/planning/path` | `nav_msgs/Path` | Sub | Trajectory waypoints from planner. |
| `/wamv/sensors/odometry` | `nav_msgs/Odometry` | Sub | Current boat state. |
| `/wamv/thrusters/...` | (varies) | Pub | Thruster commands. |

---

## Development

### Project Status

For detailed development status, milestones, and task tracking, refer to [Board.md](Board.md).

### Contributing

This project is part of an ROS 2 course at IMT Nord Europe. For contribution guidelines and development roadmap, please refer to the project board.

## References

- [Virtual RobotX (VRX) Competition](https://github.com/osrf/vrx)
- [ROS 2 Documentation](https://docs.ros.org/en/jazzy/)
- [Gazebo Simulation](https://gazebosim.org/)

## Acknowledgments

This project utilizes the Virtual RobotX (VRX) simulation environment developed by Open Source Robotics Foundation (OSRF). Simulation assets and base environment courtesy of the VRX project.

## License

This project is licensed under the Apache License 2.0 - see individual package files for details.

## Contact

For questions or issues, please open an issue on the [GitHub repository](https://github.com/Erk732/uvautoboat).
