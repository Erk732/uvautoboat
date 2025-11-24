# AutoBoat (uvautoboat)

## Overview

AutoBoat is the path planning module for the VRX (Virtual RobotX) project. This ROS 2 node acts as the "brain" of the WAM-V surface vessel: it ingests the boat's current state and mission goals, processes environmental data (obstacles, boundaries), and generates safe trajectories for execution.

## Core Responsibilities

- Point-to-point planning: navigate efficiently from start pose A to goal pose B.
- Coverage/search planning: create systematic patterns (e.g., lawn-mower) to sweep a defined region for pollution or objects.
- Obstacle avoidance: detect and route around static obstacles (e.g., buoys, islands) using planners such as A*.

---

## Architecture

- Node name: `path_planner`
- Role: bridge between the Mission/Perception layer and the Control layer.

### Interfaces

| Topic Name | Message Type | I/O | Description |
| :--- | :--- | :--- | :--- |
| `/wamv/sensors/odometry` | `nav_msgs/Odometry` | Sub | Current boat position/orientation. |
| `/planning/goal` | `geometry_msgs/PoseStamped` | Sub | Desired destination. |
| `/planning/path` | `nav_msgs/Path` | Pub | Computed trajectory (waypoints). |

---

## Installation

### Prerequisites

- [ROS 2 (Jazzy)](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- [Gazebo(Harmonic)](https://gazebosim.org/docs/harmonic/install_ubuntu/)
- [VRX Environment](https://github.com/osrf/vrx/wiki/installation_tutorial)
- Python 3

### Setup
# PLEASE NOTE THAT FOLLOW SETUP INSTRUCTIONS STRICTLY !

1. Clone the repository into your workspace `src` folder (workspace name example `seal_ws`):
```bash
mkdir -p ~/vrx_ws/src
cd ~/seal_ws/src
git clone https://github.com/Erk732/uvautoboat.git
```
2. Clone VRX environment from the repository given below:
```bash
git clone https://github.com/osrf/vrx.git
```
3. Source the main ROS 2 installation.:
```bash
source /opt/ros/jazzy/setup.bash
```

4. Build your "seal" workspace:
```bash
cd ~/seal_ws
colcon build --merge-install
```
5. Build the package:
```bash
cd ~/seal_ws
colcon build --packages-select path
. ~/seal_ws/install/setup.bash
```

---

## How to Run

Start the planner node:

```bash
ros2 run path simple_planner
```
