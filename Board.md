# Project Status Board: AutoBoat

## Phase 1: Architecture & MVP (Minimum Viable Product)

> **Goal:** Establish communication with Control Group and move the boat in a straight line.

### Define Interfaces

- [x] Agree on Topic names (`/planning/path`) with Control Group.
- [x] Agree on Message types (`nav_msgs/Path`).
- [x] Set Frame ID (Map, Odom) as parameter.

### Setup Workspace

- [x] Create GitHub Repository (`uvautoboat`).
- [x] Create ROS 2 package structure in `seal_ws`.

### Implement v1.0: Straight Line Planner

- [x] Write Subscriber for Odometry.
- [x] Write Subscriber for Goal.
- [x] Implement Linear Interpolation Logic.
- [x] Test with VRX simulation (visualize path in Rviz). **Done when:** path shows as a straight line between start/goal and endpoints are within tolerance.

### Implement v1.1: Path Follow Node

- [ ] Write Subscriber for Path.
- [ ] Handle transform from `map`/`odom` to `base_link` (verify TF tree map → odom → base_link).
- [ ] Publish local next goal in path (with Rviz visualisation).
- [ ] PID control toward the next goal
- [ ] Test with VRX simulation (visualize path in Rviz). **Done when:** boat follows path within positional error threshold.

---

## Phase 2: Obstacle Avoidance (A*)

> **Goal:** Navigate around buoys and islands without collision.

### Environment Representation

- [x] Create a class to convert (x, y) coordinates to Grid Map.
- [x] Implement "Inflation" (pad obstacles so boat doesn't hit edges).

### Implement v2.0: A* Algorithm

- [x] Implement Heuristic function (Euclidean distance).
- [x] Implement core A* search loop.
- [x] Integrate Grid Map with A* logic. **Done when:** generated paths avoid inflated obstacles in sim.

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
- [ ] Ensure path stays strictly within boundaries. **Done when:** simulated tracks remain inside provided polygon.
- [ ] Add test scenario with multiple obstacles to validate coverage completion without boundary violations.
