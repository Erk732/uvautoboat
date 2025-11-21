# Project Status Board: AutoBoat

## Phase 1: Architecture & MVP (Minimum Viable Product)

> **Goal:** Establish communication with Control Group and move the boat in a straight line.

### Define Interfaces

- [ ] Agree on Topic names (`/planning/path`) with Control Group.
- [ ] Agree on Message types (`nav_msgs/Path`).
- [ ] Set Frame ID (Map, Odom) as parameter.

### Setup Workspace

- [x] Create GitHub Repository (`uvautoboat`).
- [ ] Create ROS 2 Package structure in seal ws.

### Implement v1.0: Straight Line Planner

- [ ] Write Subscriber for Odometry.
- [ ] Write Subscriber for Goal.
- [ ] Implement Linear Interpolation Logic.
- [ ] Test with VRX simulation (visualize path in Rviz).

### Implement v1.0: Path Follow Node

- [ ] Write Subscriber for Path.
- [ ] Handle transform from odom/map to baselink
- [ ] Publish local next goal in path ((with rviz visualisation))
- [ ] PID control toward the next goal
- [ ] Test with VRX simulation (visualize path in Rviz).

---

## Phase 2: Obstacle Avoidance (A*)

> **Goal:** Navigate around buoys and islands without collision.

### Environment Representation

- [ ] Create a class to convert (x, y) coordinates to Grid Map.
- [ ] Implement "Inflation" (pad obstacles so boat doesn't hit edges).

### Implement v2.0: A* Algorithm

- [ ] Implement Heuristic function (Euclidean distance).
- [ ] Implement core A* search loop.
- [ ] Integrate Grid Map with A* logic.

### Path Smoothing

- [ ] (Optional) Implement post-processing to round off sharp 90-degree turns.

---

## Phase 3: Coverage & Search
>
> **Goal:** Search a defined area for pollution/objects.

### Region Definition

- [ ] Define how to receive Area boundaries (Polygon or Rectangle).

### Implement v3.0: Coverage Planner

- [ ] Implement Boustrophedon (Lawn-mower) pattern generator.
- [ ] Ensure path stays strictly within boundaries.
