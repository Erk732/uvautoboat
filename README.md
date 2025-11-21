# AutoBoat
AutoBoat (uvautoboat) 🛥️

Short Description

AutoBoat is the Path Planning module for our VRX (Virtual RobotX) project.

This ROS 2 node is responsible for the autonomous decision-making of the WAM-V surface vessel. It receives the boat's current state and a mission goal, processes the environment (obstacles, boundaries), and outputs a trajectory for the Control Group to follow.

🎯 Core Responsibilities

Point-to-Point Planning: Navigating from start pose A to goal pose B.

Coverage/Search Planning: Generating patterns (lawn-mower) to search a defined region for pollution or objects.

Obstacle Avoidance: Using algorithms (A-Star) to navigate around static obstacles (buoys, islands).

⚙️ Architecture

Node Name: path_planner

The planner acts as the bridge between the Mission/Perception layer and the Control layer.

Interface

Topic Name

Message Type

Description

Sub (Input)

/wamv/sensors/odometry

nav_msgs/Odometry

Current boat position (Start).

Sub (Input)

/planning/goal

geometry_msgs/PoseStamped

Desired destination.

Pub (Output)

/planning/path

nav_msgs/Path

The computed trajectory (waypoints).

🚀 Install Instructions

Prerequisites

ROS 2 (Jazzy)

VRX Simulation Environment

Python 3

Installation

Clone this repository into your ROS 2 workspace src folder:

cd ~/seal_ws/src
git clone [https://github.com/Erk732/uvautoboat.git](https://github.com/Erk732/uvautoboat.git)


Build the package:

cd ~/seal_ws
colcon build --packages-select uvautoboat
source install/setup.bash


⚡ Get Started

To run the planner node:

ros2 run uvautoboat simple_planner


To send a test goal (for debugging without the Mission node), open a new terminal and run:

ros2 topic pub /planning/goal geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 50.0, y: 20.0, z: 0.0}}}"


🧠 Algorithms Used

v1.0 (Current): Straight Line Planner (Euclidean interpolation).

v2.0 (In Progress): A* (A-Star) Grid Search for static obstacle avoidance.

v3.0 (Planned): Boustrophedon patterns for area coverage.

👥 Team Members

Erk Cayhan
Zhiyan Piao
Harish Tannira



Short Description

## Install instructions

## Get Started


