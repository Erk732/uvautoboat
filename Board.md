# 📋 AutoBoat Development Board

[![Status](https://img.shields.io/badge/Status-Active-green)](https://github.com/Erk732/uvautoboat)
[![Progress](https://img.shields.io/badge/Progress-75%25-blue)](https://github.com/Erk732/uvautoboat)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

| | |
|---|---|
| **Project** | AutoBoat Navigation System |
| **Repository** | [Erk732/uvautoboat](https://github.com/Erk732/uvautoboat) |
| **Last Updated** | 01/12/2025 |
| **Status** | 🟢 Dual Systems Operational (Apollo11 & Vostok1) |

---

## 📊 Progress Overview

| Phase | Description | Status | Progress |
|:-----:|-------------|:------:|:--------:|
| 1 | Architecture & MVP | ✅ | 100% |
| 2 | Autonomous Navigation | ✅ | 100% |
| 3 | Coverage Planning | ⏸️ | 0% |
| 4 | Integration & Testing | 🔄 | 75% |

### Active Systems

| System | Architecture | Sensors | Features |
|--------|--------------|---------|----------|
| **Apollo11** | Modular | 2D LaserScan | GPS waypoints, clean separation |
| **Vostok1** | Integrated | 3D PointCloud | PID control, SASS, web dashboard |

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
- Real-time web dashboard
- Terminal Mission CLI

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

| Test | Apollo11 | Vostok1 |
|------|:--------:|:-------:|
| GPS waypoint following | ✅ | ✅ |
| Obstacle detection | ✅ 2D | ✅ 3D |
| Multi-waypoint missions | ✅ | ✅ |
| Stuck detection/recovery | — | ✅ |
| Web dashboard | — | ✅ |
| Terminal CLI | — | ✅ |
| TF tree validation | ✅ | ✅ |

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
| TBD | Coverage Planning | ⏸️ |

---

## 🎯 Next Priorities

1. Performance benchmarking (Apollo11 vs Vostok1)
2. PID parameter optimization
3. Long-duration stress testing
4. Coverage planning algorithms

---

## 📚 Lessons Learned

| # | Lesson |
|---|--------|
| 1 | Cross-platform naming conventions are critical |
| 2 | TF tree configuration requires careful attention |
| 3 | Start simple, add complexity incrementally |
| 4 | Document early to reduce technical debt |

### Technical Debt

- Hardcoded parameters → migrate to ROS 2 parameter server
- Limited unit test coverage → add automated testing
- Complex planner versions need debugging

---

## 📜 Acknowledgments

**Document Version**: 5.0 | **Last Updated**: 01/12/2025

**Maintained By**: AutoBoat Development Team

**Institution**: [IMT Nord Europe](https://imt-nord-europe.fr/) — Industry 4.0 Students & Faculty
