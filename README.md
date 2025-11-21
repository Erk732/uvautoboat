# AutoBoat
## AutoBoat

AutoBoat (uvautoboat)

### Short Description

AutoBoat is the Path Planning module for the VRX (Virtual RobotX) project.

This ROS 2 node acts as the “brain” of the WAM-V surface vessel. It receives the boat’s current state and mission goals, processes environmental data (obstacles, boundaries), and calculates safe trajectories for the Control layer to execute.

### Core Responsibilities
Point-to-Point Planning

Navigating efficiently from start pose A to goal pose B.

### Coverage / Search Planning

Generating systematic patterns (e.g., lawn-mower sweeps) to search a region for pollution, debris, or mission-specific objects.

### Obstacle Avoidance

Using algorithms such as A* to detect and navigate around static obstacles like buoys, docks, and islands.

### Architecture

Node Name: path_planner

The planner acts as the bridge between the Mission/Perception layer and the Control layer.

## Interface
Input Topic

/planning/goal
Type: geometry_msgs/PoseStamped
Used to set the target pose of the vessel.




## Install instructions

## Get Started


