# 📋 AutoBoat Development Board

[![Status](https://img.shields.io/badge/Status-Active-green)](https://github.com/Erk732/uvautoboat)
[![Progress](https://img.shields.io/badge/Progress-90%25-blue)](https://github.com/Erk732/uvautoboat)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

| | |
|---|---|
| **Project** | AutoBoat Navigation System |
| **Repository** | [Erk732/uvautoboat](https://github.com/Erk732/uvautoboat) |
| **Last Updated** | 11/12/2025 |
| **Status** | 🟢 Vostok1 Production Ready (A* path planning + one-click launcher + wiki docs) |

---

## 📊 Progress Overview

| Phase | Description | Status | Progress |
|:-----:|-------------|:------:|:--------:|
| 1 | Architecture & MVP | ✅ | 100% |
| 2 | Autonomous Navigation | ✅ | 100% |
| 3 | Coverage Planning | ⏸️ | 0% |
| 4 | Integration & Testing | 🔄 | 90% |

### Active System

| System | Architecture | Sensors | Features |
|--------|--------------|---------|----------|
| **Vostok1 Modular** | Distributed (OKO + SPUTNIK + BURAN) | 3D PointCloud | A* path planning, LiDAR smoke detection, simple anti-stuck, runtime config, web dashboard + camera, waypoint persistence |

> **Note:** The integrated Vostok1 has been deprecated and moved to `legacy/`. Use the modular system.

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

### Vostok1 Navigation System

- Integrated perception + planning + control
- Modular variant: OKO + SPUTNIK + BURAN distributed architecture
- 3D PointCloud processing (height/distance filtering)
- Simple Anti-Stuck System
  - Turn left until clear
  - Kalman-filtered drift compensation
  - Skip detection during obstacle avoidance
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

**Progress**: 90%

### Completed ✅

| Test | Status |
|------|:------:|
| GPS waypoint following | ✅ |
| Obstacle detection (3D) | ✅ |
| Multi-waypoint missions | ✅ |
| Stuck detection/recovery | ✅ |
| Waypoint skip strategy | ✅ |
| Runtime config updates | ✅ |
| Web dashboard (map, mission, camera) | ✅ |
| Terminal CLI | ✅ |
| Min-range spawn fix (5m) | ✅ |
| A* path planning (hybrid + runtime) | ✅ |
| One-click launcher script | ✅ |
| LiDAR smoke detection (spatial density filtering) | ✅ |

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
| 28/11/2025 | Vostok1 Navigation Complete | ✅ |
| 01/12/2025 | Simple Anti-Stuck + Mission CLI | ✅ |
| 03/12/2025 | Waypoint Skip + Runtime Config | ✅ |
| 03/12/2025 | Go Home Optimization (detour insertion) | ✅ |
| 03/12/2025 | README Consolidation + Cleanup | ✅ |
| 08/12/2025 | A* Path Planning (Hybrid + Runtime modes) | ✅ |
| 09/12/2025 | One-Click Launcher Script | ✅ |
| 11/12/2025 | Wiki Documentation + README Update | ✅ |
| 14/12/2025 | LiDAR Smoke Detection (Spatial Density Filtering) | ✅ |
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
| **Dynamic Replanning** | High | Replan when new obstacles detected mid-route |
| **Go-To-Point** | Medium | Navigate to arbitrary GPS coordinate with obstacle avoidance |
| **Multi-Goal Navigation** | Medium | Sequence of random points (patrol mode) |
| **Coverage Planning** | Low | Boustrophedon pattern for area scanning |

### Recently Completed ✅

| Feature | Status | Description |
|---------|:------:|-------------|
| **A* Path Planning** | ✅ Done | Hybrid mode (pre-plan) + Runtime mode (detours) in SPUTNIK |
| **One-Click Launcher** | ✅ Done | `launch_vostok1_complete.sh` for full system startup |
| **Wiki Documentation** | ✅ Done | Comprehensive wiki pages in `wiki/` folder |

### A* Path Planning (Implemented)

```text
/oko/obstacles ────>┌─────────────────────┐
                    │  AStarSolver        │
Hazard boxes ──────>│  (in SPUTNIK)       │────> Detour waypoints inserted into /planning/waypoints
                    │                     │
Current position ──>└─────────────────────┘
```

- Occupancy grid (3m cells) with 8-connected A*
- **Hybrid Mode**: Pre-plan routes between lawnmower waypoints
- **Runtime Mode**: Plan detours when stuck or blocked

---

## 📚 Lessons Learned

| # | Lesson |
|---|--------|
| 1 | Cross-platform naming conventions are critical |
| 2 | TF tree configuration requires careful attention |
| 3 | Start simple, add complexity incrementally |
| 4 | Document early to reduce technical debt |

### Technical Debt

| Issue | Status | Description |
|:------|:------:|:------------|
| **ROS 2 Parameter Migration** | ✅ Done | Parameters now configurable via `vostok1.launch.yaml` |
| **Multi-Terminal Launch** | ✅ Done | `one_click_launch_all/launch_vostok1_complete.sh` available |
| **Debugging Required** | 🔄 In Progress | Complex planning and obstacle detection still need debugging |

---

## 📜 Acknowledgments

**Document Version**: 8.0 | **Last Updated**: 14/12/2025

**Maintained By**: AutoBoat Development Team

**Institution**: [IMT Nord Europe](https://imt-nord-europe.fr/) — Industry 4.0 Students & Faculty
