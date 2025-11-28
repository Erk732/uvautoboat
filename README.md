# AutoBoat (uvautoboat)

[![ROS 2 Version](https://img.shields.io/badge/ROS_2-Jazzy-blue)](https://docs.ros.org/en/jazzy/)
[![Gazebo Version](https://img.shields.io/badge/Gazebo-Harmonic-orange)](https://gazebosim.org/)
![ROS 2 CI](https://github.com/osrf/vrx/workflows/ROS%202%20CI/badge.svg)

## Abstract

AutoBoat is an autonomous navigation system for unmanned surface vehicles (USVs) developed for the Virtual RobotX (VRX) competition. The system integrates advanced path planning, real-time obstacle avoidance, and precise trajectory tracking algorithms optimized for the WAM-V maritime platform. Built on ROS 2 Jazzy and Gazebo Harmonic, the framework provides a robust foundation for autonomous maritime navigation in simulated environments.

## Overview

AutoBoat implements a hierarchical autonomous navigation framework designed for maritime surface vehicles operating in complex environments. The system combines perception, planning, and control subsystems to enable intelligent waypoint navigation while dynamically responding to environmental constraints. By processing sensor data streams and mission objectives in real-time, the architecture generates collision-free trajectories that account for static obstacles, operational boundaries, and vehicle dynamics, ensuring safe and efficient autonomous operation.

## Background Concepts for New Users

If you're new to ROS 2 or autonomous navigation, here are some key concepts that will help you understand this system:

### What is ROS 2?

**ROS 2 (Robot Operating System 2)** is a framework for building robot software. Think of it as a communication system that allows different parts of a robot (sensors, planning algorithms, controllers) to talk to each other.

**Key ROS 2 Concepts:**

- **Node**: An independent program that performs a specific task (e.g., path planning, motor control)
- **Topic**: A named channel where nodes send and receive messages (like `/wamv/pose` for boat position)
- **Message**: Data structure sent over topics (e.g., position coordinates, sensor readings)
- **Package**: A collection of related nodes and code organized together

**Example**: In this system, the `astar_planner` node publishes path messages to the `/planning/path` topic, and the `path_follower` node subscribes to that topic to receive the path.

### What are Coordinate Frames (TF)?

**Coordinate frames** define where things are in 3D space. Different parts of the robot have their own reference frames.

**In this system:**

- `world` or `map`: The global reference frame (like GPS coordinates)
- `wamv/base_link`: The boat's center point
- `wamv/imu_wamv_link`: Where the IMU sensor is located

**Why it matters**: To plan a path, the system needs to know where the boat is relative to the world. The TF (Transform) system automatically converts between these different frames.

### What is use_sim_time?

When running in simulation (Gazebo), time moves differently than real-world time. The simulation can be paused, slowed down, or sped up.

**The `use_sim_time` parameter** tells ROS 2 nodes to use simulation time instead of your computer's clock. This keeps everything synchronized.

**Always use** `--ros-args -p use_sim_time:=true` when running nodes alongside Gazebo simulation.

### What is Gazebo?

**Gazebo** is a 3D physics simulator that creates realistic virtual environments for testing robots. It simulates:

- Physics (gravity, buoyancy, collisions)
- Sensors (cameras, GPS, IMU)
- Actuators (thrusters, motors)

**Why simulate?** Testing on real boats is expensive and dangerous. Gazebo lets you test your algorithms safely before deploying to hardware.

### How Does Path Planning Work?

Path planning is like using Google Maps for your robot:

1. **Input**: Current position + Goal position
2. **Process**: Algorithm finds obstacle-free path
3. **Output**: Series of waypoints to follow

**A* Algorithm** (used in this system):

- Searches through a grid of possible positions
- Finds the shortest path while avoiding obstacles
- Uses a "heuristic" (educated guess) to search efficiently

### How Does the Control System Work?

The control system is like a driver following GPS directions:

1. **Input**: Desired path from planner
2. **Process**: Calculate steering and throttle commands
3. **Output**: Thruster commands (left/right thrust)

**Differential Thrust Control**:

- Two independent thrusters (left and right)
- To go straight: Both thrusters equal power
- To turn left: Right thruster more power than left
- To turn right: Left thruster more power than right

### Topic Communication Example

```text
┌─────────────┐         /planning/goal        ┌──────────────┐
│    User     │ ──────────────────────────────│ astar_planner│
└─────────────┘                               └──────────────┘
                                                       │
                                                       │ /planning/path
                                                       ↓
                                                ┌──────────────┐
                                                │path_follower │
                                                └──────────────┘
                                                       │
                                          ┌────────────┴──────────────┐
                                          ↓                           ↓
                                /wamv/thrusters/left    /wamv/thrusters/right
                                          ↓                           ↓
                                    ┌─────────────────────────────────┐
                                    │   WAM-V Boat (Gazebo)           │
                                    └─────────────────────────────────┘
```

This visualization shows how messages flow from goal input to boat movement.

## Core Capabilities

- **Point-to-Point Navigation**: Generates efficient trajectories between specified waypoints using optimization-based planning algorithms, enabling precise autonomous navigation in structured maritime environments.

- **Coverage Planning**: Implements systematic area coverage using boustrophedon (lawn-mower) patterns optimized for search and surveillance missions, ensuring complete environmental exploration.

- **Obstacle Avoidance**: Provides dynamic path replanning capabilities using the A* search algorithm with grid-based environmental representation to safely navigate around static obstacles including buoys, islands, and other maritime structures.

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
| `/wamv/pose` | `geometry_msgs/PoseStamped` | Sub | Current boat position/orientation. |
| `/planning/goal` | `geometry_msgs/PoseStamped` | Sub | Desired destination. |
| `/planning/path` | `nav_msgs/Path` | Pub | Computed trajectory (waypoints). |

---

## Installation

### System Requirements

- **Operating System**: Ubuntu 24.04 LTS
- **Python**: Version 3.10 or higher
- **Memory**: Minimum 8GB RAM (16GB or 32GB recommended)
- **Storage**: At least 40GB available disk space

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

⚠️ **Important**: When running nodes with Gazebo simulation, you must use the `use_sim_time` parameter to synchronize with simulation time.

**Planning Nodes:**

***A star path planner with obstacle avoidance***

```bash
ros2 run plan astar_planner --ros-args -p use_sim_time:=true
```

***Time-stamped dynamic obstacle avoidance***

```bash
ros2 run plan avoidingobs_ts_planner --ros-args -p use_sim_time:=true
```

***Perception and obstacle detection***

```bash
ros2 run plan simple_perception --ros-args -p use_sim_time:=true
```

***Mission coordination***

```bash
ros2 run plan mission_trigger
```

**Control Nodes:**

***Simple thruster controller***

```bash
ros2 run control simple_controller --ros-args -p use_sim_time:=true
```

***Path following controller***

```bash
ros2 run control path_follower --ros-args -p use_sim_time:=true
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

### Customizing the Simulation World

⚠️ **Important Note**: The VRX repository (`~/seal_ws/src/vrx`) is itself a Git repository. If you modify VRX world files (e.g., `sydney_regatta.sdf`) or other VRX content and don't want to accidentally push changes to the upstream VRX repository, consider these options:

- **Option 1**: Create custom world files in your own package (e.g., `test_environment/`) and reference them instead
- **Option 2**: Add modified files to `.git/info/exclude` in the VRX directory to ignore them locally without affecting `.gitignore`
- **Option 3**: Use `git update-index --assume-unchanged <file>` to tell Git to ignore changes to specific files
- **Best Practice**: Keep VRX unmodified and create custom worlds in your own workspace directories

#### Adding Models from Gazebo Fuel

To add additional obstacles or objects to your simulation environment, you can download models from the Gazebo Fuel model repository, which is the recommended source.

**Model Repositories:**

- **General Models**: [Gazebo Fuel Collections](https://app.gazebosim.org/OpenRobotics/fuel/collections)
- **VRX-Specific Models**: [VRX Model Collection](https://app.gazebosim.org/OpenRobotics/fuel/collections/VRX)

**Steps to Add Models to Your World:**

1. **Browse and select a model** from Gazebo Fuel (e.g., `cardboard_box`, `construction_barrel`, etc.)

2. **Download the model** (optional - Gazebo can auto-download):

   Models are automatically downloaded when referenced in your SDF file, or you can manually download them to your local model path.

3. **Add the model to your world file**:

   Open your world SDF file (e.g., `sydney_regatta.sdf` or create a custom one) and add the model using the `<include>` tag:

   ```xml
   <world name="sydney_regatta">
     <!-- Existing world content -->
     
     <!-- Add your custom model -->
     <include>
       <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/cardboard_box</uri>
       <name>obstacle_box_1</name>
       <pose>10 5 0 0 0 0</pose>  <!-- x y z roll pitch yaw -->
     </include>
     
     <!-- Add multiple instances with different names and positions -->
     <include>
       <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/cardboard_box</uri>
       <name>obstacle_box_2</name>
       <pose>15 -3 0 0 0 1.57</pose>
     </include>
   </world>
   ```

4. **Launch with your customized world**:

   ```bash
   ros2 launch vrx_gz competition.launch.py world:=your_custom_world
   ```

**Common Useful Models:**

- `cardboard_box` - Simple box obstacle
- `construction_barrel` - Traffic barrel
- `jersey_barrier` - Concrete barrier
- `brick_wall` - Wall obstacle
- `wooden_peg` - Small marker

**Note**: The `<pose>` tag specifies the model's position (x, y, z) and orientation (roll, pitch, yaw) in meters and radians respectively.

![3D Cartesian Coordinate System](images/3d_coordinate_system.jpg)
*Figure: 3D Cartesian coordinate system showing x, y, z axes. Image: [Primalshell](https://commons.wikimedia.org/wiki/File:3D_Cartesian_Coodinate_Handedness.jpg), [CC BY-SA 4.0](https://creativecommons.org/licenses/by-sa/4.0/)*

- **Position** (x, y, z): Coordinates in 3D space measured in **meters**
  - `x`: Forward/backward (正向/反向)
  - `y`: Left/right (左/右)
  - `z`: Up/down (上/下)
- **Orientation** (roll, pitch, yaw): Rotation angles measured in **radians**
  - `roll`: Rotation around x-axis (绕x轴旋转)
  - `pitch`: Rotation around y-axis (绕y轴旋转)
  - `yaw`: Rotation around z-axis (绕z轴旋转)
  - **Common usage**: `0 0 0` for default upright orientation (no rotation)
  - **Example**: `0 0 1.57` rotates 90° (π/2 radians) around z-axis

**Note**: The `<pose>` tag specifies the model's position in (x, y, z) and orientation (roll, pitch, yaw)，in meters and radians respectively.

- **Position** (x, y, z): Coordinates in 3D space in**m**
  - `x`: Used for going forward (all engines ahead) or backward (all engines sternway)
  - `y`: Left hand turning or righr hand turning
  - `z`: Up or down 
- **Orientation** (roll, pitch, yaw): Rotation angle in**rad**
  - `roll`: Rotation around the x-axis (roll to port side or starboard side)
  - `pitch`: Rotation around the y-axis（pitch up or pitch down）
  - `yaw`: Rotation around the z-axis（yaw to port direction or starboard direction ）
  - **Common Values**: `0 0 0` These values are default upright orinetation of the boat（无旋转）
  - **For better understanding**: `0 0 1.57` values will show a 90° (π/2 in radians) rotation around z-axis(a simple yaw) 

### Expected Output

After launching the simulation environment, you should observe the following:

[![VRX Simulation Environment](images/sydney_regatta_gzsim.png)](https://vimeo.com/851696025 "Gazebo Virtual RobotX v. 2.3 - Click to Watch!")

*Figure: Sydney Regatta simulation environment in Gazebo. Source: [VRX Project](https://github.com/osrf/vrx/wiki/running_vrx_tutorial)*

---

## Package Documentation

### Plan Package

The `plan` package implements the planning and perception subsystems for autonomous navigation. This package provides multiple planning algorithms and supporting utilities for environment representation and obstacle detection.

#### Planning Nodes

**1. A* Planner** (`astar_planner`)

Implements the A* search algorithm for optimal path planning with obstacle avoidance. Uses grid-based environmental representation with configurable resolution and inflation radius.

**Algorithm Overview:**

A* (A-star) is an informed search algorithm that finds the shortest path between two points while avoiding obstacles. It combines two key metrics:

- **g(n)**: Actual cost from start to current node
- **h(n)**: Estimated cost (heuristic) from current node to goal
- **f(n) = g(n) + h(n)**: Total estimated cost through current node

The algorithm maintains a priority queue of nodes to explore, always expanding the node with the lowest f(n) value. This ensures both optimality (finding the shortest path) and efficiency (exploring fewer nodes than uninformed search methods like Dijkstra's algorithm).

**Further Reading:** [A* Search Algorithm - Wikipedia](https://en.wikipedia.org/wiki/A*_search_algorithm)

**Key Features:**

- Guaranteed to find the optimal path if one exists
- Grid-based discretization of the environment
- Obstacle inflation for safety margins
- Euclidean distance heuristic for maritime navigation

**Run:**

```bash
ros2 run plan astar_planner --ros-args -p use_sim_time:=true
```

**2. Time-Stamped Obstacle Avoidance Planner** (`avoidingobs_ts_planner`)

Enhanced planning algorithm incorporating temporal information for dynamic obstacle prediction and avoidance.

**Run:**

```bash
ros2 run plan avoidingobs_ts_planner --ros-args -p use_sim_time:=true
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
ros2 run control simple_controller --ros-args -p use_sim_time:=true
```

**Launch file:**

```bash
ros2 launch control simple_controller.launch.py
```

**2. Path Following Controller** (`path_follower`)

Advanced trajectory tracking controller implementing waypoint following with feedback control for precise path execution.

**Current Implementation:**

⚠️ **Note**: The current path follower uses a simplified control strategy rather than full PID control. The controller employs direct y-axis adjustment for heading correction, setting the y-axis value in the final goal command sent to the boat. This approach provides basic trajectory tracking functionality while more advanced PID-based control methods are under development.

**Run:**

```bash
ros2 run control path_follower --ros-args -p use_sim_time:=true
```

### Control Interfaces

| Topic Name | Message Type | I/O | Description |
| :--- | :--- | :--- | :--- |
| `/planning/path` | `nav_msgs/Path` | Sub | Trajectory waypoints from planner. |
| `/wamv/pose` | `geometry_msgs/PoseStamped` | Sub | Current boat state. |
| `/wamv/thrusters/left/thrust` | `std_msgs/Float64` | Pub | Left thruster command. |
| `/wamv/thrusters/right/thrust` | `std_msgs/Float64` | Pub | Right thruster command. |

---

## Troubleshooting

### Common Issues and Solutions

#### Issue: Boat Not Moving in Simulation

**Symptoms**: Nodes are running but the WAM-V boat remains stationary.

**Solutions**:

1. **Verify `use_sim_time` parameter**:
   - Ensure all nodes are launched with `--ros-args -p use_sim_time:=true`
   - This synchronizes node time with Gazebo simulation time

2. **Check topic connections**:

   ```bash
   # Verify thrust commands are being published
   ros2 topic echo /wamv/thrusters/left/thrust
   ros2 topic echo /wamv/thrusters/right/thrust

   # Check if path is being generated
   ros2 topic echo /planning/path

   # Verify pose data is available
   ros2 topic echo /wamv/pose
   ```

3. **Verify TF transforms**:

   ```bash
   # Check if required transforms are available
   ros2 run tf2_ros tf2_echo map wamv/base_link
   ```

4. **Check node status**:

   ```bash
   # List active nodes
   ros2 node list

   # Check specific node info
   ros2 node info /astar_planner_node
   ros2 node info /thruster_path_follower
   ```

#### Issue: A* Planner Not Publishing Path

**Symptoms**: No path messages on `/planning/path` topic after sending a goal.

**Solutions**:

1. **Verify goal message**:

   ```bash
   # Ensure goal is in correct frame
   ros2 topic pub --once /planning/goal geometry_msgs/msg/PoseStamped \
   "{header: {frame_id: 'map'}, pose: {position: {x: -520.0, y: 190.0, z: 0.0}}}"
   ```

2. **Check planner logs**:
   - Look for warnings like "Start or Goal outside Grid Map!"
   - Verify TF is available: "Waiting for TF..."

3. **Verify grid map configuration**:
   - Default grid: 300m × 300m
   - Ensure goal coordinates are within grid bounds

#### Issue: Build Failures

**Symptoms**: `colcon build` fails with errors.

**Solutions**:

1. **Missing dependencies**:

   ```bash
   # Install ROS 2 dependencies
   cd ~/seal_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

2. **Clean build**:

   ```bash
   # Remove build artifacts and rebuild
   rm -rf build install log
   colcon build --merge-install
   ```

3. **Python import errors**:
   - Ensure `grid_map.py` exists in `plan/brain/` directory
   - Verify relative imports use correct syntax

#### Issue: Simulation Crashes or Freezes

**Symptoms**: Gazebo becomes unresponsive or crashes.

**Solutions**:

1. **Check system resources**:
   - Verify sufficient RAM (minimum 8GB, recommended 16GB)
   - Monitor CPU usage

2. **Reduce graphics quality**:
   - Lower rendering settings in Gazebo
   - Disable shadows and reflections

3. **Use headless mode** (for testing without GUI):

   ```bash
   ros2 launch vrx_gz competition.launch.py world:=sydney_regatta headless:=true
   ```

### Getting Help

If you encounter issues not covered here:

1. **Check logs**: Review terminal output for error messages and warnings
2. **Verify configuration**: Ensure all parameters match the documentation
3. **Consult VRX documentation**: [VRX Wiki](https://github.com/osrf/vrx/wiki)
4. **Report issues**: Open an issue on the [GitHub repository](https://github.com/Erk732/uvautoboat/issues)

---

## Testing

### Unit Testing

The project includes unit tests for critical components:

```bash
# Run all tests
cd ~/seal_ws
colcon test --packages-select plan control

# View test results
colcon test-result --verbose
```

### Integration Testing

### Test 1: Path Planning and Following

1. Launch simulation:

   ```bash
   ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
   ```

2. Start navigation stack:

   ```bash
   # Terminal 2
   ros2 run plan astar_planner --ros-args -p use_sim_time:=true

   # Terminal 3
   ros2 run control path_follower --ros-args -p use_sim_time:=true
   ```

3. Send test goal:

   ```bash
   # Terminal 4
   ros2 topic pub --once /planning/goal geometry_msgs/msg/PoseStamped \
   "{header: {frame_id: 'map'}, pose: {position: {x: -520.0, y: 190.0, z: 0.0}}}"
   ```

4. **Expected behavior**:
   - A* planner computes path within 1-2 seconds
   - Path follower begins tracking waypoints
   - Boat moves smoothly toward goal
   - Boat stops within 2m of goal position

### Test 2: Obstacle Avoidance

1. Launch simulation with custom world:

   ```bash
   export GZ_SIM_RESOURCE_PATH=$HOME/seal_ws/src/uvautoboat/test_environment:$GZ_SIM_RESOURCE_PATH
   gz sim ~/seal_ws/src/uvautoboat/test_environment/sydney_regatta_custom.sdf
   ```

2. Start obstacle avoidance node:

   ```bash
   ros2 run plan avoidingobs_ts_planner --ros-args -p use_sim_time:=true
   ```

3. **Expected behavior**:
   - Node detects cardboard box obstacles
   - Boat adjusts trajectory to avoid collisions
   - Thrust commands modulate based on obstacle proximity

### Performance Metrics

Expected system performance benchmarks:

| Metric | Target | Description |
|:-------|:-------|:------------|
| Planning Time | < 2s | Time to compute initial path |
| Control Loop Rate | 10 Hz | Path follower update frequency |
| Position Error | < 2m | Distance to goal at completion |
| Collision Avoidance | 100% | Success rate in test scenarios |

---

## Development

### Project Status

For detailed development status, milestones, and task tracking, refer to [Board.md](Board.md).

### Contributing

This project is developed as part of the ROS 2 Autonomous Systems course at IMT Nord Europe. Contributions are welcome from the community.

**Development Guidelines**:

1. **Code Style**: Follow PEP 8 for Python code
2. **Documentation**: Update README and inline comments for significant changes
3. **Testing**: Include unit tests for new functionality
4. **Pull Requests**: Provide clear descriptions of changes and their purpose

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
