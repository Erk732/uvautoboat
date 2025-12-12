# Installation Guide

Complete guide to installing AutoBoat and its dependencies.

---

## System Requirements

| Component | Minimum | Recommended |
|:----------|:--------|:------------|
| OS | Ubuntu 24.04 LTS | Ubuntu 24.04 LTS |
| RAM | 8 GB | 16 GB |
| Storage | 40 GB | 60 GB |
| Python | 3.10+ | 3.12 |
| GPU | Integrated | Dedicated (for Gazebo) |

---

## Prerequisites

Before installing AutoBoat, you need to install the following dependencies:

### 1. ROS 2 Jazzy

Follow the official installation guide:

- **[ROS 2 Jazzy Installation](https://docs.ros.org/en/jazzy/Installation.html)**

```bash
# Quick install (Ubuntu 24.04)
sudo apt update
sudo apt install ros-jazzy-desktop-full
```

### 2. Gazebo Harmonic

Follow the official installation guide:

- **[Gazebo Harmonic Installation](https://gazebosim.org/docs/harmonic/install_ubuntu/)**

```bash
# Quick install
sudo apt-get update
sudo apt-get install gz-harmonic
```

### 3. VRX Simulation

Clone the VRX repository:

- **[VRX GitHub Repository](https://github.com/osrf/vrx)**

```bash
cd ~/seal_ws/src
git clone https://github.com/osrf/vrx.git
```

### 4. rosbridge-suite

Required for web dashboard WebSocket communication:

```bash
sudo apt install ros-jazzy-rosbridge-suite
```

### 5. web_video_server

Required for dashboard camera panel:

```bash
sudo apt install ros-jazzy-web-video-server
```

---

## Installation Steps

### Step 1: Create Workspace

```bash
mkdir -p ~/seal_ws/src
cd ~/seal_ws/src
```

### Step 2: Clone AutoBoat Repository

```bash
git clone https://github.com/Erk732/uvautoboat.git
```

### Step 3: Clone VRX (if not already done)

```bash
git clone https://github.com/osrf/vrx.git
```

### Step 4: Install Dependencies

```bash
cd ~/seal_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

### Step 5: Build Workspace

```bash
colcon build --merge-install
```

**Note**: This may take 5-10 minutes on first build.

### Step 6: Source Environment

```bash
source ~/seal_ws/install/setup.bash
```

### Step 7: Add to ~/.bashrc (Recommended)

For automatic sourcing in new terminals:

```bash
echo "source ~/seal_ws/install/setup.bash" >> ~/.bashrc
```

---

## Verification

### Verify ROS 2 Installation

```bash
ros2 --version
# Expected: ros2 cli version: 0.32.x
```

### Verify Gazebo Installation

```bash
gz sim --version
# Expected: Gazebo Sim, version 8.x.x
```

### Verify AutoBoat Installation

```bash
ros2 pkg list | grep plan
ros2 pkg list | grep control
```

You should see `plan` and `control` packages listed.

### Test Launch VRX

```bash
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
```

You should see the Gazebo simulator open with the Sydney Regatta world and WAM-V boat.

---

## Troubleshooting

### Build Failures

If you encounter build errors, try a clean build:

```bash
cd ~/seal_ws
rm -rf build install log
colcon build --merge-install
```

### Missing Dependencies

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### Gazebo Plugin Errors

Make sure Gazebo environment variables are set:

```bash
source /usr/share/gazebo/setup.sh
```

### Python Package Issues

Install additional Python packages if needed:

```bash
pip3 install numpy scipy matplotlib
```

---

## Next Steps

Once installation is complete:

1. **[Quick Start](Quick_Start)** — Launch your first mission
2. **[First Mission Tutorial](First-Mission-Tutorial)** — Detailed walkthrough
3. **[Configuration & Tuning](Configuration-and-Tuning)** — Customize parameters

---

## Additional Resources

- **[ROS 2 Jazzy Tutorials](https://docs.ros.org/en/jazzy/Tutorials.html)**
- **[Gazebo Harmonic Tutorials](https://gazebosim.org/docs/harmonic/tutorials)**
- **[VRX Installation Guide](https://github.com/osrf/vrx/wiki/installation)**
