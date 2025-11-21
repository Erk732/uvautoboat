# AutoBoat
AutoBoat (uvautoboat) 🛥️

📖 Short Description

AutoBoat is the Path Planning module for the VRX (Virtual RobotX) project.

This ROS 2 node acts as the "brain" of the WAM-V surface vessel. It receives the boat's current state and mission goals, processes environmental data (obstacles, boundaries), and calculates safe trajectories for the Control Group to execute.

🎯 Core Responsibilities

📍 Point-to-Point Planning

Navigating efficiently from start pose A to goal pose B.

🔍 Coverage/Search Planning

Generating systematic patterns (e.g., lawn-mower) to search a defined region for pollution or objects.

🚧 Obstacle Avoidance

Using algorithms (like A*) to detect and navigate around static obstacles such as buoys and islands.

⚙️ Architecture

Node Name: path_planner

The planner serves as the bridge between the Mission/Perception layer and the Control layer.

Interface

Topic Name

Message Type

Description

Sub (Input)

/wamv/sensors/odometry

nav_msgs/Odometry

Current boat position/orientation.

Sub (Input)

/planning/goal

geometry_msgs/PoseStamped

The desired destination.

Pub (Output)

/planning/path

nav_msgs/Path

The computed trajectory (list of waypoints).

🚀 Installation Instructions

Prerequisites

ROS 2 (Jazzy)

VRX Simulation Environment

Python 3

Setup Steps

Clone the repository into your workspace src folder:

cd ~/seal_ws/src
git clone [https://github.com/Erk732/uvautoboat.git](https://github.com/Erk732/uvautoboat.git)


Build the package:

cd ~/seal_ws
colcon build --packages-select uvautoboat
source install/setup.bash


⚡ How to Run

1. Start the Planner Node

ros2 run uvautoboat simple_planner


2. Send a Test Goal (Debugging)

To simulate a mission request without the full system running, open a new terminal and publish a dummy goal:

ros2 topic pub /planning/goal geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 50.0, y: 20.0, z: 0.0}}}"


🧠 Algorithms

Version

Status

Algorithm

Description

v1.0

✅ Current

Straight Line

Basic Euclidean interpolation for initial testing.

v2.0

🚧 In Progress

*A (A-Star)**

Grid-based search for optimal pathfinding around obstacles.

v3.0

📅 Planned

Boustrophedon

"Lawn-mower" patterns for full area coverage.

👥 Team Members

Erk Cayhan

Zhiyan Piao

Harish Tannira



Short Description

## Install instructions

## Get Started


