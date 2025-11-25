# AutoBoat (uvautoboat)

## Overview

AutoBoat is the path planning module for the VRX (Virtual RobotX) project. This ROS 2 node acts as the "brain" of the WAM-V surface vessel: it ingests the boat's current state and mission goals, processes environmental data (obstacles, boundaries), and generates safe trajectories for execution.

## Core Responsibilities

- Point-to-point planning: Navigate efficiently from start point A to goal point B.
- Coverage / search planning: Create systematic patterns (e.g., lawn-mower) to sweep a defined region for pollution or objects.
- Obstacle avoidance: Detect and route around static obstacles (e.g., buoys, islands) using planners such as A* (Which is A-Star).

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

❗ **Attention:** It is strongly recommended to use ROS 2 (Jazzy) paired with Gazebo (Harmonic). If you plan to use other versions of these 2 setups, some unintended problems may occur.

- [ROS 2 (Jazzy)](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- [Gazebo (Harmonic)](https://gazebosim.org/docs/harmonic/install_ubuntu/)
- Python 3

### Setup

❗ **Attention:** Please note that if you are expecting a relatively good result, it is highly recommended to follow setup instructions strictly, and if any step is missed, the package may not work as intended.

1. Clone the repository into your workspace `src` folder (workspace name example `seal_ws` in our case, and you can use your own workspace name):

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

## How to Run

### Running the Demo

Launch the complete navigation demo:

```bash
ros2 launch plan demo.launch.py
```

### Running Individual Nodes

Start specific planner nodes:

```bash
# A* planner
ros2 run plan astar_planner

# Time-stamped obstacle avoidance planner
ros2 run plan avoidingobs_ts_planner

# Simple controller
ros2 run control simple_controller
```

## Using the Test Environment for path planning

⚠️ **Note:** The custom test environment is deprecated. It is recommended to use the default Sydney Regatta world instead.

The `test_environment` folder contains legacy `Gazebo` worlds and models for testing navigation algorithms.

### Custom World (Deprecated)

- **sydney_regatta_custom.sdf** - Modified Sydney Regatta environment with custom obstacles and test scenarios

### Custom Models (Deprecated)

- **cardboardbox** - Obstacle model for testing collision avoidance

### Setup and Launch (⚠️End of Life)

1. Let `Gazebo` know you have a new environment to load.

   ```bash
   export GZ_SIM_RESOURCE_PATH=$HOME/seal_ws/src/uvautoboat/test_environment:$GZ_SIM_RESOURCE_PATH
   ```

1. Source and launch the test environment.

   ```bash
   source ~/seal_ws/install/setup.bash
   ros2 launch vrx_gz competition.launch.py world:=sydney_regatta_custom
   ```

❗ **Attention:** After testing it is recommended to directly use the default world which is the Sydney Regatta environment.

```bash
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
```

Put this command in the terminal and run it. If you have followed the previous instrucions carefully, you will most likely see this:

[![VRX](images/sydney_regatta_gzsim.png)](https://vimeo.com/851696025 "Gazebo Virtual RobotX v. 2.3 - Click to Watch!")

*Image source: [VRX Project](https://github.com/osrf/vrx/wiki/running_vrx_tutorial)*

![ROS 2 CI](https://github.com/osrf/vrx/workflows/ROS%202%20CI/badge.svg)

---

## Plan Package

The `plan` package contains the core path planning and perception logic for autonomous navigation.

### Planning Nodes

#### 1. A* Planner

Obstacle-avoiding path planner using A* algorithm with grid map representation.

**Run:**

```bash
ros2 run plan astar_planner
```

#### 2. Avoiding Obstacles Planner (Time-Stamped)

Advanced planner with temporal obstacle avoidance.

**Run:**

```bash
ros2 run plan avoidingobs_ts_planner
```

#### 3. Simple Perception

Processes sensor data to detect obstacles and update the environment representation.

**Run:**

```bash
ros2 run plan simple_perception
```

#### 4. Mission Trigger

Coordinates mission goals and triggers appropriate planning behaviors.

**Run:**

```bash
ros2 run plan mission_trigger
```

#### 5. TF Broadcaster

Publishes coordinate frame transformations for the navigation stack.

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

## Control Package

The `control` package provides path following and thruster control capabilities for the WAM-V vessel.

### Control Nodes

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
