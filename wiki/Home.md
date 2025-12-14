# AutoBoat Wiki — Autonomous Navigation for Unmanned Surface Vehicles

![AutoBoat Banner](../images/image1.png)
[![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-blue)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange)](https://gazebosim.org/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
![Status](https://img.shields.io/badge/Status-Active-green)

Welcome to the **AutoBoat Wiki**! This documentation provides comprehensive guides for using and developing the autonomous navigation system for the Virtual RobotX (VRX) competition.

---

## 📚 Quick Navigation

### 🚀 Getting Started

- **[Installation Guide](Installation_Guide)** — Set up ROS 2, Gazebo, and AutoBoat
- **[Quick Start](Quick_Start)** — Get your first mission running in 5 minutes
- **[First Mission Tutorial](First-Mission-Tutorial)** — Step-by-step walkthrough

### 🏗️ Architecture

- **[System Overview](System_Overview)** — High-level architecture and design philosophy
- **[Vostok1 Architecture](Vostok1-Architecture)** — Integrated single-node system
- **[Modular Architecture](Modular-Architecture)** — OKO-SPUTNIK-BURAN distributed system
- **[Atlantis Architecture](Atlantis-Architecture)** — Control group approach
- **[ROS 2 Topic Flow](ROS2-Topic-Flow)** — Inter-node communication diagram

### 📖 User Guides

- **[Terminal Mission Control (CLI)](Terminal-Mission-Control)** — Command-line interface
- **[Web Dashboard Guide](Web-Dashboard-Guide)** — Real-time monitoring interface
- **[Keyboard Teleop](Keyboard-Teleop)** — Manual control for testing
- **[Configuration & Tuning](Configuration-and-Tuning)** — Parameter reference
- **[Launch Files Reference](Launch-Files-Reference)** — YAML and Python launch files

### 🧠 Core Concepts

- **[GPS Navigation](GPS-Navigation)** — Coordinate systems and equirectangular projection
- **[3D LIDAR Processing](3D_LIDAR_Processing)** — OKO perception system explained
- **[PID Control](PID-Control)** — Heading controller fundamentals
- **[Differential Thrust](Differential-Thrust)** — Two-thruster control system
- **[Kalman Filtering](Kalman-Filtering)** — State estimation and Bayesian inference

### 🛠️ Advanced Features

- **[Simple Anti-Stuck System](SASS)** — Simple recovery maneuvers (deprecated wiki, see README)
- **[A* Path Planning](Astar-Path-Planning)** — Grid-based obstacle avoidance
- **[Waypoint Skip Strategy](Waypoint-Skip-Strategy)** — Handling blocked waypoints
- **[Obstacle Avoidance Loop](Obstacle-Avoidance-Loop)** — Continuous perception-control cycle
- **[Hazard Zone Management](Hazard-Zone-Management)** — Pre-defined no-go areas

### 🧪 Development

- **[Contributing Guidelines](Contributing)** — How to contribute code
- **[Code Review Standards](Code-Review-Standards)** — Best practices
- **[Testing Guide](Testing-Guide)** — Unit tests and integration tests
- **[API Reference](API-Reference)** — ROS 2 topics, services, and parameters

### 🐛 Troubleshooting

- **[Common Issues](Common_Issues)** — Solutions to frequent problems
- **[Debug Commands](Debug-Commands)** — Diagnostic tools
- **[FAQ](FAQ)** — Frequently asked questions

### 📚 References

- **[ROS 2 Resources](ROS2-Resources)** — External documentation
- **[VRX Competition](VRX-Competition)** — Competition information
- **[Related Projects](Related-Projects)** — Similar work and inspiration

---

## 🎯 Project Status

| Phase | Description | Status |
|:------|:------------|:------:|
| Phase 1 | Architecture & MVP | ✅ DONE |
| Phase 2 | Obstacle Avoidance | ✅ DONE |
| Phase 3 | Coverage & Search | ⏸️ Planned |
| Phase 4 | Integration & Testing | 🔄 80% |

See [Board.md](https://github.com/Erk732/uvautoboat/blob/main/Board.md) for detailed milestones.

---

## 🤝 About

**Maintained By**: AutoBoat Development Team
**Institution**: [IMT Nord Europe](https://imt-nord-europe.fr/) — Industry 4.0 Students & Faculty
**License**: Apache 2.0

**Last Updated**: December 2025

---

## 🔗 External Links

- **[GitHub Repository](https://github.com/Erk732/uvautoboat)**
- **[Report Issues](https://github.com/Erk732/uvautoboat/issues)**
- **[VRX Official Wiki](https://github.com/osrf/vrx/wiki)**
- **[ROS 2 Documentation](https://docs.ros.org/en/jazzy/)**
