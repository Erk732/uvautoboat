# AutoBoat (uvautoboat)

## Overview

AutoBoat is the path planning module for the VRX (Virtual RobotX) project. This ROS 2 node acts as the "brain" of the WAM-V surface vessel: it ingests the boat's current state and mission goals, processes environmental data (obstacles, boundaries), and generates safe trajectories for execution.

## Core Responsibilities

- Point-to-point planning: Navigate efficiently from start pose A to goal pose B.
- Coverage/search planning: Create systematic patterns (e.g., lawn-mower) to sweep a defined region for pollution or objects.
- Obstacle avoidance: Detect and route around static obstacles (e.g., buoys, islands) using planners such as A*.

---

## Architecture

- Node name: `path_planner`
- Role: Bridge between the Mission / Perception layer and the Control layer.

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
- Python 3 (Google it if you don't know what it is)

### Setup

#### Please note that you should follow setup instructions strictly, and if any step is missed, the package may not work as intended

1. Clone the repository into your workspace `src` folder (workspace name example `seal_ws` in our case):

   ```bash
   mkdir -p ~/seal_ws/src
   cd ~/seal_ws/src
   git clone https://github.com/Erk732/uvautoboat.git
   ```

2. Clone VRX environment from the repository given below:

   ```bash
   git clone https://github.com/osrf/vrx.git
   ```

3. Source the main ROS 2 installation: (if you don't know where your ROS 2 is installed, most probably it is in `/opt/ros/<distro_name>/setup.bash`, in our case `jazzy`, and if you don't know what this sentence means, just copy the line below)

   ```bash
   source /opt/ros/jazzy/setup.bash
   ```

4. Build your "seal" workspace: (Again `seal_ws` is just an example name, if you named your workspace differently, please change it accordingly)

   ```bash
   cd ~/seal_ws
   colcon build --merge-install
   ```

5. Set up the environment:

   ```bash
   cd ~/seal_ws
   . ~/seal_ws/install/setup.bash
   ```

## How to Run (🚧 WORK IN PROGRESS 🚧)

Start the planner node:

```bash
ros2 run path simple_planner
```

## Using the Test Environment for path planning

1.Let `Gazebo` know you have a new environment to load.

```bash
 export GZ_SIM_RESOURCE_PATH=/home/bot/seal_ws/src/uvautoboat/test_environment:$GZ_SIM_RESOURCE_PATH
```

2.Source and launch the test environment.

```bash
source ~/seal_ws/install/setup.bash
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta_custom
```

---

## Control Package

The `control` package provides path following and thruster control capabilities for the WAM-V vessel.

### Available Nodes

#### 1. Simple Controller

A basic controller node for testing thruster commands.

**Run:**

```bash
ros2 run control simple_controller
```

**Launch file:**

```bash
ros2 launch control simple_controller.launch.py
```

#### 2. Path Follower

Advanced controller that follows computed paths from the planner.

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

## Project Status

For detailed development status, milestones, and task tracking, see [Board.md](Board.md).
