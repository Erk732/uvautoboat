# 📋 AutoBoat Development Board

[![Status](https://img.shields.io/badge/Status-Active-green)](https://github.com/Erk732/uvautoboat)
[![Progress](https://img.shields.io/badge/Progress-90%25-blue)](https://github.com/Erk732/uvautoboat)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

| | |
|---|---|
| **Project** | AutoBoat Navigation System |
| **Repository** | [Erk732/uvautoboat](https://github.com/Erk732/uvautoboat) |
| **Last Updated** | 04/12/2025 |
| **Status** | 🟢 Vostok1 Production Ready |

---

## 📊 Progress Overview

| Phase | Description | Status | Progress |
|:-----:|-------------|:------:|:--------:|
| 1 | Architecture & MVP | ✅ | 100% |
| 2 | Autonomous Navigation | ✅ | 100% |
| 3 | Coverage Planning | ⏸️ | 0% |
| 4 | Integration & Testing | 🔄 | 90% |

### Active Systems

| System | Architecture | Sensors | Features |
|--------|--------------|---------|----------|
| **Vostok1** | Integrated | 3D PointCloud | PID control, SASS v2.0, waypoint skip, web dashboard |
| **Modular** | Distributed | 3D PointCloud | ОКО + СПУТНИК + БУРАН, runtime config |

---

## Phase 1: Architecture & MVP ✅

**Completed**: 27/11/2025

| Task | Status |
|------|:------:|
| ROS 2 topic conventions (\`/planning/path\`) | ✅ |
| Message types (Path, PoseStamped) | ✅ |
| Workspace structure (\`seal_ws\`) | ✅ |
| Straight-line planner v1.0 | ✅ |
| Path following controller v1.1 | ✅ |
| TF tree configuration | ✅ |

---

## Phase 2: Autonomous Navigation ✅

**Completed**: 28/11/2025

### Apollo11 (Planning Team)

- Modular architecture with external planner/controller
- 2D LaserScan obstacle detection
- GPS-based waypoint navigation

### Vostok1 (Control Team)

- Integrated perception + planning + control
- 3D PointCloud processing (height/distance filtering)
- Smart Anti-Stuck System (SASS) v2.0
  - Kalman-filtered drift compensation
  - No-go zone memory
  - Multi-direction probing
- **Waypoint Skip Strategy** (NEW)
  - Stuck-based skip after 4 attempts
  - Obstacle blocking skip after 45s timeout
- Runtime PID/speed configuration
- Real-time web dashboard
- Terminal Mission CLI (vostok1_cli)

---

## Phase 3: Coverage Planning ⏸️

**Status**: Not Started | **Priority**: Low

| Task | Status |
|------|:------:|
| Region definition (polygon boundaries) | ⬜ |
| Boustrophedon coverage planner | ⬜ |
| Coverage validation (>95% target) | ⬜ |

---

## Phase 4: Integration & Testing 🔄

**Progress**: 75%

### Completed ✅

| Test | Vostok1 | Modular |
|------|:-------:|:-------:|
| GPS waypoint following | ✅ | ✅ |
| Obstacle detection (3D) | ✅ | ✅ |
| Multi-waypoint missions | ✅ | ✅ |
| Stuck detection/recovery | ✅ | ✅ |
| Waypoint skip strategy | ✅ | ✅ |
| Runtime config updates | ✅ | ✅ |
| Web dashboard | ✅ | ✅ |
| Terminal CLI | ✅ | ✅ |
| Min-range spawn fix (5m) | ✅ | ✅ |

### Pending ⬜

| Task | Priority |
|------|:--------:|
| Performance benchmarking (RMS error) | Medium |
| Obstacle stress testing | Medium |
| Long-duration test (15+ min) | Low |
| Complex waypoint circuit (8-point) | Low |

### Documentation ✅

| Document | Status |
|----------|:------:|
| README.md | ✅ |
| Board.md | ✅ |
| Code comments | ✅ |
| Troubleshooting guide | ✅ |

---

## 📝 Issue Tracking

### Resolved ✅

| Issue | Resolution |
|-------|------------|
| Invalid Windows file paths | Renamed to \`FREE.py\`, \`OUT.py\` |
| Sparse checkout blocking | \`git sparse-checkout disable\` |
| Markdown lint errors | Added \`.markdownlint.json\` |
| Spawn dock obstacle detection | Increased min_range from 0.5m → 5.0m |
| Runtime config not updating | Added config_callback to buran_controller |
| Boat circling around buoys | Added waypoint skip strategy (45s timeout) |
| Missing numpy dependency | Added python3-numpy to package.xml |
| Invalid setup.py entries | Removed non-existent apollo11, atlantis |

### Active 🔄

| Issue | Priority | Description |
|-------|:--------:|-------------|
| #4 | Medium | Advanced planner debugging |
| #5 | Medium | PID tuning refinement |
| #6 | Low | Gazebo SDF customization |

---

## 📅 Timeline

| Date | Milestone | Status |
|------|-----------|:------:|
| 25/11/2025 | Project Kickoff | ✅ |
| 26/11/2025 | Basic Navigation | ✅ |
| 27/11/2025 | End-to-End Pipeline | ✅ |
| 28/11/2025 | Apollo11 & Vostok1 Complete | ✅ |
| 01/12/2025 | SASS v2.0 + Mission CLI | ✅ |
| 03/12/2025 | Waypoint Skip + Runtime Config | ✅ |
| 03/12/2025 | Go Home Optimization (detour insertion) | ✅ |
| 03/12/2025 | README Consolidation + Cleanup | ✅ |
| TBD | A* Path Planning | ⏸️ |
| TBD | Coverage Planning | ⏸️ |

---

## 🎯 Next Priorities

1. Long-duration stress testing (15+ min missions)
2. Complex waypoint circuits with obstacles
3. Performance benchmarking (RMS error analysis)
4. Coverage planning algorithms (boustrophedon)

---

## 🚀 Future Ideas

| Feature | Priority | Description |
|---------|:--------:|-------------|
| **A* Path Planning** | High | Pre-compute obstacle-free paths using occupancy grid |
| **Dynamic Replanning** | High | Replan when new obstacles detected mid-route |
| **Go-To-Point** | Medium | Navigate to arbitrary GPS coordinate with obstacle avoidance |
| **Multi-Goal Navigation** | Medium | Sequence of random points (patrol mode) |
| **Coverage Planning** | Low | Boustrophedon pattern for area scanning |

### A* Path Planning (Proposed)

```text
/goal_point ──────┐
                  ├──→ [pathfinder.py] ──→ /planned_waypoints ──→ [vostok1/sputnik]
/oko/obstacles ───┘
```

- Occupancy grid (10m cells) from LIDAR
- A* algorithm for optimal path
- Dynamic replanning on obstacle detection

---

## 📚 Lessons Learned

| # | Lesson |
|---|--------|
| 1 | Cross-platform naming conventions are critical |
| 2 | TF tree configuration requires careful attention |
| 3 | Start simple, add complexity incrementally |
| 4 | Document early to reduce technical debt |

### Technical DebtTesting

- Hardcoded parameters → migrate to ROS 2 parameter server
- Limited unit test coverage → add automated testing
- Complex planner versions need debugging

---

## 📜 Acknowledgments

**Document Version**: 6.0 | **Last Updated**: 03/12/2025

**Maintained By**: AutoBoat Development Team

**Institution**: [IMT Nord Europe](https://imt-nord-europe.fr/) — Industry 4.0 Students & Faculty
