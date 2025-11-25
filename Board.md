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

- [x] Understand how to add obstacles
- [x] Download some obstacles from Fuel
- [x] Create a class to convert (x, y) coordinates to Grid Map (`grid_map.py`).
- [x] Implement "Inflation" (pad obstacles so boat doesn't hit edges).
- [x] Create custom test environment with obstacles (`test_environment/sydney_regatta_custom.sdf`).
- [x] Add cardboardbox obstacle model for testing.

### Implement v2.0: A* Algorithm

- [x] Implement Heuristic function (Euclidean distance).
- [x] Implement core A* search loop.
- [x] Integrate Grid Map with A* logic (`astar_planner.py`).
- [x] Create obstacle-aware planner (`avoidingOBS_planner.py`).
- [x] Add time-stamped obstacle avoidance (`avoidingobs_ts_planner.py`). **Done when:** generated paths avoid inflated obstacles in sim.

### Path Smoothing

- [x] (Optional) Implement post-processing to round off sharp 90-degree turns.

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

---

## Phase 4: Integration & Testing

> **Goal:** Ensure all components work together seamlessly.

### Plan Package Components

- [x] Create `plan` ROS 2 package structure.
- [x] Implement A* planner nodes (`astar_planner.py`, `avoidingOBS_planner.py`, `avoidingobs_ts_planner.py`).
- [x] Implement perception node (`simple_perception.py`).
- [x] Implement mission coordination (`mission_trigger.py`).
- [x] Implement TF broadcasting (`tf_broadcaster.py`).
- [x] Add utility modules (`grid_map.py`, `FREE.py`, `OUT.py`).
- [x] Create demo launch file (`demo.launch.py`).
- [x] Add RViz configuration (`default.rviz`).

### Control Package Integration

- [x] Create `control` ROS 2 package structure.
- [x] Implement `simple_controller` node for basic thruster testing.
- [x] Implement `path_follower` node for advanced path tracking.
- [x] Add launch files for control nodes.
- [ ] Integrate PID controller for precise waypoint following.
- [ ] Test end-to-end: planner → controller → simulation.

### System Validation

- [ ] Test complete pipeline: odometry → goal → path planning → path following → thruster commands.
- [ ] Verify TF tree is correct (map → odom → base_link).
- [ ] Measure path following accuracy (position error < 0.5m, heading error < 5°).
- [ ] Test with different scenarios (straight line, obstacle avoidance, coverage).

### Documentation

- [x] Update README.md with control package documentation.
- [x] Add usage examples for all nodes.
- [ ] Create parameter configuration guide.
- [ ] Add troubleshooting section.

---

## Known Issues & Fixes

### Git Issues

- [x] **Fixed:** Invalid Windows paths (`path/path/**FREE**` and `path/path/**OUT`) renamed to `FREE.py` and `OUT.py`.
- [x] **Fixed:** Sparse checkout preventing file visibility - disabled with `git sparse-checkout disable`.

### Configuration

- [x] **Fixed:** Markdown linting error MD046 - configured `.markdownlint.json` to use fenced code blocks.
