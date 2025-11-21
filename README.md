# AutoBoat (uvautoboat)

<<<<<<< HEAD
## Short Description
=======
[cite_start]AutoBoat is the Path Planning module for the VRX (Virtual RobotX) project[cite: 45, 47]. [cite_start]This ROS 2 node acts as the "brain" of the WAM-V surface vessel[cite: 48]. [cite_start]It receives the boat's current state and mission goals, processes environmental data (obstacles, boundaries), and calculates safe trajectories for the Control Group to execute[cite: 49].
>>>>>>> bb5b0f2 (prof review)

[cite_start]AutoBoat is the Path Planning module for the VRX (Virtual RobotX) project[cite: 45, 47]. [cite_start]This ROS 2 node acts as the "brain" of the WAM-V surface vessel[cite: 48]. [cite_start]It receives the boat's current state and mission goals, processes environmental data (obstacles, boundaries), and calculates safe trajectories for the Control Group to execute[cite: 49].

## Core Responsibilities

1. **Point-to-Point Planning**
    * [cite_start]Navigating efficiently from start pose A to goal pose B[cite: 52, 54].
2. **Coverage/Search Planning**
    * [cite_start]Generating systematic patterns (e.g., lawn-mower) to search a defined region for pollution or objects[cite: 55, 56].
3. **Obstacle Avoidance**
    * [cite_start]Using algorithms (like A*) to detect and navigate around static obstacles such as buoys and islands[cite: 57, 58].

---

## Architecture

* [cite_start]**Node Name:** `path_planner` [cite: 60]
* [cite_start]**Role:** The planner serves as the bridge between the Mission/Perception layer and the Control layer[cite: 61].

### Interface

| Topic Name | Message Type | I/O | Description |
| :--- | :--- | :--- | :--- |
| `/wamv/sensors/odometry` | `nav_msgs/Odometry` | Sub | [cite_start]Current boat position/orientation[cite: 67, 68, 69]. |
| `/planning/goal` | `geometry_msgs/PoseStamped` | Sub | [cite_start]The desired destination[cite: 71, 72, 73]. |
| `/planning/path` | `nav_msgs/Path` | Pub | [cite_start]The computed trajectory [list of waypoints](cite: 75, 76, 78). |

---

## Installation Instructions

### Prerequisites

* [cite_start]ROS 2 (Jazzy) [cite: 81]
* [cite_start]VRX Simulation Environment [cite: 82]
* [cite_start]Python 3 [cite: 83]

### Setup Steps

<<<<<<< HEAD
1. **Clone the repository into your workspace src folder:**

=======
1.  **Clone the repository into your workspace src folder:** Here the worspace name is `seal_ws`
>>>>>>> bb5b0f2 (prof review)
    ```bash
    cd ~/seal_ws/src
    git clone [https://github.com/Erk732/uvautoboat.git](https://github.com/Erk732/uvautoboat.git)
    ```

    [cite: 90, 91]

2. **Build the package:**

    ```bash
    cd ~/seal_ws
    colcon build --packages-select path
    source install/setup.bash
    ```

    [cite: 93, 94, 95]

---

## How to Run

### 1. Start the Planner Node

```bash
ros2 run path simple_planner


## Install instructions

## Get Started


