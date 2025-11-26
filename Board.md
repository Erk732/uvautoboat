# Project Development Board: AutoBoat Navigation System

**Project**: AutoBoat Navigation System
**Repository**: [uvautoboat](https://github.com/Erk732/uvautoboat)  
**Last Updated**: 26/11/2025  
**Status**: Phase 1 In Progress - Basic Navigation Implemented

---

## Progress Overview

| Phase | Title | Status | Completion |
|-------|-------|--------|------------|
| 1 | Architecture & MVP | 🔄 In Progress | 70% |
| 2 | Obstacle Avoidance (A*) | 🔄 In Progress | 10% |
| 3 | Coverage & Search | ⏸️ Not Started | 0% |
| 4 | Integration & Testing | ⏸️ Not Started | 0% |

---

## Phase 1: Architecture & Minimum Viable Product

**Objective**: Establish foundational architecture and demonstrate basic straight-line navigation.

**Status**: 🔄 **IN PROGRESS** | **Priority**: High | **Completion**: 70%

### 1.1 Interface Definition

**Goal**: Establish standardized communication protocols between planning and control modules.

**Status**: ✅ **COMPLETED**

- [x] Define topic naming conventions (`/planning/path`) in coordination with Control Team
- [x] Specify message types (`nav_msgs/Path`, `geometry_msgs/PoseStamped`)
- [x] Configure coordinate frame parameters (Map, Odom)

### 1.2 Workspace Configuration

**Goal**: Initialize project repository and ROS 2 workspace structure.

**Status**: ✅ **COMPLETED**

- [x] Create GitHub repository (`uvautoboat`) with proper .gitignore
- [x] Initialize ROS 2 package structure in workspace (`seal_ws`)
- [x] Configure package dependencies and build system

### 1.3 Straight Line Planner (v1.0)

**Goal**: Implement basic point-to-point navigation without obstacle consideration.

**Status**: 🔄 **IN PROGRESS** | **Priority**: High

**Last Updated**: 26/11/2025

**Implementation Tasks:**

- [x] Implement odometry subscriber (`nav_msgs/Odometry`)
- [x] Implement goal pose subscriber (`geometry_msgs/PoseStamped`)
- [x] Develop linear interpolation algorithm for waypoint generation (simple version)
- [x] Integrate RViz visualization for path display
- [ ] Refine complex version with advanced features (currently failing)

**Progress Notes:**

- ✅ Simple version working successfully in trials
- ⚠️ More complex version encounters failures - requires debugging

**Acceptance Criteria:**

- [x] Generated path appears as straight line between start and goal
- [x] Endpoint accuracy within specified tolerance (±0.5m)
- [x] Path successfully visualized in RViz
- [ ] Complex version stable and reliable

### 1.4 Path Following Controller (v1.1)

**Goal**: Implement trajectory tracking for autonomous navigation.

**Status**: 🔄 **IN PROGRESS** | **Priority**: High

**Last Updated**: 26/11/2025

**Implementation Tasks:**

- [x] Implement path subscriber (`nav_msgs/Path`)
- [x] Configure basic TF tree transformations (`map` → `odom` → `base_link`)
- [x] Develop simple waypoint selection algorithm
- [x] Implement basic controller for heading and velocity control
- [ ] Add RViz markers for next waypoint visualization
- [ ] Refine advanced controller with PID tuning (complex version failing)

**Progress Notes:**

- ✅ Simple path follower working well in trials
- ⚠️ Advanced version with complex control logic experiencing failures
- Need to debug and stabilize complex controller implementation

**Acceptance Criteria:**

- [x] Basic vessel navigation along generated path
- [ ] Position error consistently < 0.5m
- [ ] Heading error maintained below 5°
- [ ] Smooth transitions between waypoints without oscillation

---

## Phase 2: Obstacle Avoidance & Path Planning

**Objective**: Implement intelligent path planning with static obstacle avoidance using A* algorithm.

**Status**: 🔄 **IN PROGRESS** | **Priority**: Medium | **Completion**: 10%

### 2.1 Environment Representation

**Goal**: Develop grid-based environmental model for obstacle mapping.

**Status**: 🔄 **IN PROGRESS**

**Last Updated**: 26/11/2025

**Implementation Tasks:**

- [x] Research and understand Gazebo obstacle integration
- [x] Experiment with world customization (adding cardboard box to default world)
- [ ] Source obstacle models from Gazebo Fuel library
- [ ] Implement `GridMap` class for coordinate-to-grid conversion (`grid_map.py`)
- [ ] Develop obstacle inflation algorithm for safety margins
- [ ] Create custom simulation environment (`test_environment/sydney_regatta_custom.sdf`)
- [ ] Integrate obstacle models for testing

**Progress Notes:**

- 🛠️ Experimental work on world customization in progress
- ⚠️ Team still learning Gazebo world customization workflow
- Successfully added cardboard box to default world as proof-of-concept
- Need more practice with SDF file structure and model integration

**Next Steps:**

- Formalize world customization process
- Document steps for adding obstacles
- Build comprehensive test environment
- [ ] Source obstacle models from Gazebo Fuel library
- [ ] Implement `GridMap` class for coordinate-to-grid conversion (`grid_map.py`)
- [ ] Develop obstacle inflation algorithm for safety margins
- [ ] Create custom simulation environment (`test_environment/sydney_regatta_custom.sdf`)
- [ ] Integrate obstacle models for testing

**Expected Deliverables:**

- [ ] Functional grid-based environment representation
- [ ] Configurable obstacle inflation radius
- [ ] Custom test world with multiple obstacle configurations

### 2.2 A* Path Planning Algorithm (v2.0)

**Goal**: Implement optimal path planning with guaranteed obstacle avoidance.

**Implementation Tasks:**

- [ ] Implement Euclidean distance heuristic function
- [ ] Develop core A* search algorithm with priority queue
- [ ] Integrate grid map with A* planning logic (`astar_planner.py`)
- [ ] Create obstacle-aware planner variant (`avoidingOBS_planner.py`)
- [ ] Implement time-stamped version for dynamic environments (`avoidingobs_ts_planner.py`)

**Acceptance Criteria:**

- [ ] Generated paths successfully avoid all inflated obstacles
- [ ] Path optimality verified (near-optimal given grid discretization)
- [ ] Planner handles multiple obstacle scenarios

### 2.3 Path Smoothing

**Goal**: Post-process A* output for smoother, more natural trajectories.

**Implementation Tasks:**

- [ ] Implement path smoothing algorithm to reduce sharp angles
- [ ] Add configurable smoothing parameters

**Expected Deliverables:**

- [ ] Smoothed paths with gradual turns suitable for maritime vessel dynamics

---

## Phase 3: Coverage Planning & Area Search

**Objective**: Implement systematic area coverage for search and surveillance missions.

**Status**: ⏸️ **NOT STARTED** | **Priority**: Low

### 3.1 Region Definition

**Goal**: Develop interface for specifying coverage areas.

**Implementation Tasks:**

- [ ] Define message type for area boundaries (Polygon vs. Rectangle)
- [ ] Implement polygon boundary subscriber
- [ ] Add boundary validation and preprocessing
- [ ] Create test scenarios with various boundary shapes

**Acceptance Criteria:**

- [ ] System accepts arbitrary polygon boundaries
- [ ] Invalid boundaries rejected with informative error messages

### 3.2 Boustrophedon Coverage Planner (v3.0)

**Goal**: Generate efficient lawn-mower patterns for complete area coverage.

**Implementation Tasks:**

- [ ] Implement boustrophedon decomposition algorithm
- [ ] Develop coverage path generation with configurable line spacing
- [ ] Add boundary constraint enforcement
- [ ] Optimize turn sequences for minimal transit time
- [ ] Integrate obstacle avoidance within coverage area

**Acceptance Criteria:**

- [ ] Complete coverage of specified area (>95% coverage)
- [ ] Generated paths remain within boundary constraints
- [ ] Successful navigation around obstacles within coverage area
- [ ] Path efficiency metrics documented (distance, time)

### 3.3 Validation & Testing

**Goal**: Verify coverage completeness and performance.

**Test Scenarios:**

- [ ] Simple rectangular area (baseline)
- [ ] Irregular polygon with concave sections
- [ ] Coverage area with multiple internal obstacles
- [ ] Large-scale area requiring extended operation

**Performance Metrics:**

- [ ] Coverage percentage
- [ ] Path length efficiency
- [ ] Execution time
- [ ] Boundary violation incidents

---

## Phase 4: System Integration & Validation

**Objective**: Ensure seamless integration of all subsystems and validate complete navigation stack.

**Status**: ⏸️ **NOT STARTED** | **Priority**: Low | **Completion**: 0%

### 4.1 Plan Package Implementation

**Goal**: Finalize planning module architecture and components.

**Status**: ⏸️ **NOT STARTED**

**Planned Components:**

- [ ] ROS 2 package structure with proper dependencies (`package.xml`, `setup.py`)
- [ ] A* planning nodes:
  - [ ] `astar_planner.py` - Core A* implementation
  - [ ] `avoidingOBS_planner.py` - Obstacle-aware variant
  - [ ] `avoidingobs_ts_planner.py` - Time-stamped version
- [ ] Perception module (`simple_perception.py`)
- [ ] Mission coordination (`mission_trigger.py`)
- [ ] Transform management (`tf_broadcaster.py`)
- [ ] Utility modules:
  - [ ] `grid_map.py` - Grid-based mapping
  - [ ] `FREE.py` - Free space utilities
  - [ ] `OUT.py` - Boundary detection
- [ ] Launch configuration (`demo.launch.py`)
- [ ] RViz visualization config (`default.rviz`)

### 4.2 Control Package Implementation

**Goal**: Develop trajectory tracking and low-level control.

**Status**: ⏸️ **NOT STARTED** | **Completion**: 0%

**Planned Components:**

- [ ] ROS 2 package structure (`control` package)
- [ ] Simple controller node (`simple_controller.py`) - Basic thruster testing
- [ ] Path follower node (`path_follower.py`) - Trajectory tracking
- [ ] Launch files (`simple_controller.launch.py`)
- [ ] Tune PID controller parameters for optimal performance
- [ ] Implement adaptive control for varying environmental conditions
- [ ] Add velocity profiling for smooth acceleration/deceleration

### 4.3 End-to-End System Testing

**Goal**: Validate complete navigation pipeline under realistic conditions.

**Status**: ⏸️ **NOT STARTED**

**Test Suite:**

- [ ] **Pipeline Integration Test**
  - [ ] Verify data flow: Odometry → Planning → Control → Thrusters
  - [ ] Measure end-to-end latency (<100ms target)
  - [ ] Validate message synchronization

- [ ] **Coordinate Frame Validation**
  - [ ] Verify TF tree structure (`map` → `odom` → `base_link`)
  - [ ] Test transform accuracy and timing
  - [ ] Monitor for transform timeout errors

- [ ] **Performance Benchmarking**
  - [ ] Position tracking error: Target <0.5m RMS
  - [ ] Heading error: Target <5° RMS
  - [ ] Path following smoothness (jerk metrics)
  - [ ] Computational resource usage (CPU, memory)

**Test Scenarios:**

- [ ] **Scenario 1**: Straight-line navigation (10m, 50m, 100m distances)
- [ ] **Scenario 2**: Obstacle avoidance (single obstacle, multiple obstacles)
- [ ] **Scenario 3**: Complex waypoint navigation (8-point circuit)
- [ ] **Scenario 4**: Coverage mission (rectangular search area)
- [ ] **Scenario 5**: Long-duration test (15+ minute operation)

### 4.4 Documentation & Knowledge Transfer

**Goal**: Provide comprehensive documentation for users and developers.

**Status**: 🔄 **IN PROGRESS** | **Completion**: 30%

**Completed Documentation:**

- [x] Project README with basic project description
- [x] Development board tracking (this document)

**Remaining Documentation:**

- [ ] Complete installation instructions
- [ ] Package-level documentation (plan & control)
- [ ] Node usage examples and command reference
- [ ] **Configuration Guide**
  - [ ] Parameter descriptions and tuning guidelines
  - [ ] Launch file customization
  - [ ] RViz setup and visualization guide

- [ ] **Troubleshooting Guide**
  - [ ] Common errors and solutions
  - [ ] Debug procedures
  - [ ] Performance optimization tips

- [ ] **Developer Documentation**
  - [ ] Code architecture diagrams
  - [ ] API documentation
  - [ ] Contributing guidelines

---

## Issue Tracking & Resolution Log

### Resolved Issues

#### Repository & Version Control

**Issue #1**: Invalid Windows file paths  
**Status**: ✅ **RESOLVED**  
**Description**: Files `path/path/**FREE**` and `path/path/**OUT` contained invalid characters (`**`) for Windows filesystem.  
**Resolution**: Renamed to `FREE.py` and `OUT.py`. Updated all imports and references.  
**Date**: November 2025

**Issue #2**: Sparse checkout preventing file visibility  
**Status**: ✅ **RESOLVED**  
**Description**: Git sparse-checkout configuration prevented renamed files from appearing in workspace.  
**Resolution**: Disabled sparse checkout with `git sparse-checkout disable`.  
**Date**: November 2025

#### Development Tools & Configuration

**Issue #3**: Markdown linting error MD046  
**Status**: ✅ **RESOLVED**  
**Description**: Inconsistent code block styles (fenced vs. indented) causing linter failures.  
**Resolution**: Created `.markdownlint.json` configuration to standardize on fenced code blocks.  
**Date**: November 2025

### Active Issues

#### Navigation Implementation

**Issue #4**: Complex planner version failures  
**Status**: 🔴 **ACTIVE** | **Priority**: High  
**Description**: While simple straight-line planner works successfully, the more complex version with advanced features encounters runtime failures.  
**Impact**: Blocking progression to advanced planning features.  
**Next Steps**: Debug complex planner implementation, isolate failure points, review algorithm logic.  
**Date Identified**: 26/11/2025

**Issue #5**: Advanced path follower instability  
**Status**: 🔴 **ACTIVE** | **Priority**: High  
**Description**: Basic path following controller operates correctly, but advanced version with complex control logic experiences failures during operation.  
**Impact**: Limiting trajectory tracking accuracy and reliability.  
**Next Steps**: Review PID tuning, check controller state transitions, validate control loop timing.  
**Date Identified**: 26/11/2025

#### Simulation Environment

**Issue #6**: Limited world customization expertise  
**Status**: 🟡 **ACTIVE** | **Priority**: Medium  
**Description**: Team requires additional familiarity with Gazebo SDF file structure and world customization workflow. Successfully added cardboard box to default world as proof-of-concept; comprehensive understanding of process still in development.  
**Impact**: Reduced velocity in developing custom test environments for obstacle avoidance validation.  
**Next Steps**: Study SDF documentation thoroughly, practice model integration techniques, document standardized customization workflow.  
**Date Identified**: 26/11/2025

### Future Considerations

- [ ] **Performance Optimization**: Profile planning algorithms for computational efficiency
- [ ] **Multi-Agent Support**: Extend architecture for coordinated multi-vessel operations
- [ ] **Dynamic Obstacle Handling**: Implement prediction and avoidance for moving obstacles
- [ ] **Robustness Testing**: Extended stress testing under adverse simulation conditions

---

## Timeline & Milestones

| Milestone | Target Date | Status |
|-----------|-------------|--------|
| Phase 1.1-1.2 Complete | ✅ Completed | Done |
| Phase 1.3-1.4 Basic Implementation | ✅ 26/11/2025 | Done |
| Phase 1.3-1.4 Advanced Versions | TBD | Debugging |
| Phase 2.1 Environment Setup | TBD | In Progress |
| Phase 2 Complete | TBD | 10% |
| Phase 3 Complete | TBD | Not Started |
| Phase 4 Complete | TBD | Not Started |
| System Demo Ready | TBD | Pending |
| Documentation Finalized | TBD | 30% |

---

## Notes & Observations

### Lessons Learned

1. **Cross-platform compatibility**: Filename restrictions vary by OS - maintain strict naming conventions
2. **Coordinate frame management**: Proper TF tree configuration is critical for navigation accuracy
3. **Modular architecture**: Separation of planning and control enables independent development and testing
4. **Documentation early**: Comprehensive documentation from the start reduces technical debt
5. **Iterative development approach** (26/11/2025): Starting with simple, working implementations before adding complexity proves valuable - allows verification of basic functionality before troubleshooting advanced features
6. **Simulation environment learning curve** (26/11/2025): Gazebo world customization requires dedicated learning investment; hands-on experimentation is essential for comprehensive understanding of SDF structure and model integration

### Technical Debt

- Parameter configuration currently hardcoded - needs migration to ROS 2 parameter server
- Limited unit test coverage - should implement automated testing framework
- Complex planner and controller versions need debugging and stabilization
- World customization workflow needs formalization and documentation

### Recent Progress Summary (26/11/2025)

**Achievements:**

- ✅ Basic straight-line planner operational
- ✅ Simple path follower successfully tested
- ✅ Initial world customization experiments completed

**Current Challenges:**

- ⚠️ Complex algorithm versions experiencing runtime failures
- ⚠️ Team developing expertise in Gazebo environment customization
- ⚠️ Debugging strategy required for advanced implementations

**Focus Areas for Next Development Session:**

- Debug and resolve complex planner and controller failures
- Formalize and document world customization workflow
- Document troubleshooting procedures for identified issues

---

**Document Version**: 2.1  
**Maintained By**: AutoBoat Development Team (IMT Nord Europe Industry 4.0 students and faculty)
**Review Frequency**: Weekly
