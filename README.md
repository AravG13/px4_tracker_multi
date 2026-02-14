# px4_tracker_multi

**Adaptive, Safe, and Trustworthy Real-Time Task Management for Mobile Autonomous Robots**

A trust-based multi-drone object tracking system built on PX4, ROS2, and Gazebo. Three Typhoon-H480 drones autonomously track multiple targets in a realistic simulated environment, coordinating via a combined direct/indirect trust algorithm and active collision avoidance.

---

## Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Features](#features)
- [Tech Stack](#tech-stack)
- [Repository Structure](#repository-structure)
- [Prerequisites](#prerequisites)
- [Setup & Installation](#setup--installation)
- [Running the Simulation](#running-the-simulation)
- [Tracking System](#tracking-system)
- [Control Pipeline](#control-pipeline)
- [Trust System](#trust-system)
- [Collision Avoidance](#collision-avoidance)
- [Logs & Data](#logs--data)
- [References](#references)

---

## Overview

This project implements a complete end-to-end framework for multi-drone autonomous tracking in dynamic, GPS-compromised environments. Each drone independently tracks a target using visual feedback, while a trust-based coordination layer ensures safe inter-drone spacing and robust task handover.

The simulation environment is built from real-world OpenStreetMap data, processed through Blender and imported into Gazebo, providing realistic urban geometry for testing.

---

## System Architecture

```
┌────────────────────────────────────────────────────────┐
│                     Gazebo Classic                     │
│   (OSM-based world + 3× Typhoon-H480 drones)           │
└───────────────────┬────────────────────────────────────┘
                    │ uORB topics
            ┌───────▼────────┐
            │ μXRCE-DDS      │  (PX4 ↔ ROS2 bridge)
            │ Client / Agent │
            └───────┬────────┘
                    │ FAST-DDS
        ┌───────────▼───────────┐
        │        ROS2           │
        │  ┌─────────────────┐  │
        │  │  Camera Node    │  │  (GStreamer-based, per drone)
        │  │  Tracker Node   │  │  (OpenCV detection + bbox)
        │  │  Controller     │  │  (PID + NED transform)
        │  │  Trust Manager  │  │  (Direct + Indirect trust)
        │  └─────────────────┘  │
        └───────────────────────┘
```

---

## Features

- **Multi-drone multi-target tracking** — each drone is assigned a target and maintains continuous visual lock
- **Visual tracking via OpenCV** — GStreamer camera feed → bounding box detection → normalized error computation
- **Per-axis PID control** — independent X/Y/Z + distance PIDs with exponential smoothing
- **NED frame transformation** — camera-frame pixel errors correctly mapped to PX4 NED velocity setpoints via yaw rotation
- **Trust-based coordination** — combined direct and indirect trust scores govern task delegation and prioritisation
- **Collision avoidance** — repulsive velocity adjustment maintains minimum drone separation at all times
- **Backing-away mode** — drones autonomously reverse if a target enters the close-proximity violation zone
- **Manual control interface** — per-drone keyboard controller for arm/disarm, takeoff, land, and tracking toggle
- **Persistent CSV logging** — timestamped trust and tracking data for post-run analysis

---

## Tech Stack

| Component | Technology |
|---|---|
| Flight controller | PX4 Autopilot |
| Simulation | Gazebo Classic |
| Middleware | ROS2 (Humble) |
| PX4–ROS2 bridge | MicroXRCE-DDS |
| Visual tracking | OpenCV |
| Camera streaming | GStreamer |
| World modelling | OpenStreetMap + Blender (Blosm extension) |
| Language | Python (65%), C++ (30%), Shell (3%), CMake (1%) |

---

## Repository Structure

```
px4_tracker_multi/
├── src/                                    # ROS2 package source
│   ├── camera_node/                        # GStreamer camera publisher
│   ├── tracker_node/                       # OpenCV tracker + bbox publisher
│   ├── controller_node/                    # PID controller + NED transform
│   └── trust_manager/                      # Trust computation + collision avoidance
├── drone_tracking_logs_20251003_122609/    # Sample session logs
├── comprehensive_data_20251004_114927.csv  # Full telemetry + trust log (session 1)
├── comprehensive_data_20251004_115118.csv  # Full telemetry + trust log (session 2)
├── tracking_data_20251003_123103.csv       # Tracking-only log (drone 1)
├── tracking_data_20251003_123113.csv       # Tracking-only log (drone 2)
├── setup_drone_tracker.sh                  # One-shot environment setup script
└── .gitignore
```

---

## Prerequisites

- Ubuntu 22.04
- ROS2 Humble
- PX4 Autopilot (built from source)
- Gazebo Classic (Gazebo 11)
- MicroXRCE-DDS Agent
- Python 3.10+
- OpenCV (`pip install opencv-python`)
- GStreamer 1.0 (`gstreamer1.0-tools`, `gstreamer1.0-plugins-*`)
- Blender 3.x with [Blosm extension](https://github.com/vvoovv/blosm) *(for world generation only)*

---

## Setup & Installation

```bash
# 1. Clone the repository
git clone https://github.com/AravG13/px4_tracker_multi.git
cd px4_tracker_multi

# 2. Run the setup script (installs dependencies and builds the ROS2 workspace)
chmod +x setup_drone_tracker.sh
./setup_drone_tracker.sh

# 3. Source the workspace
source install/setup.bash
```

> **Note:** Ensure PX4 firmware is built and the Gazebo world file is in place before launching. Refer to the [PX4 Simulation docs](https://docs.px4.io/main/en/) for first-time setup.

---

## Running the Simulation

Open four terminals and run each command in order:

**Terminal 1 — Start Gazebo + PX4 multi-vehicle simulation**
```bash
cd <PX4-Autopilot>
Tools/simulation/gazebo-classic/sitl_multiple_run.sh -m typhoon_h480 -n 3
```

**Terminal 2 — Start MicroXRCE-DDS Agent**
```bash
MicroXRCEAgent udp4 -p 8888
```

**Terminal 3 — Launch ROS2 nodes**
```bash
ros2 launch px4_tracker_multi multi_drone_tracking.launch.py
```

**Terminal 4 — Manual controller (optional)**
```bash
ros2 run px4_tracker_multi manual_controller --ros-args -p drone_id:=1
```

### Manual Controller Commands

| Key | Action |
|---|---|
| `a` | Arm drone |
| `d` | Disarm drone |
| `t` | Takeoff |
| `l` | Land |
| `e` | Emergency stop |
| `k` | Start tracking (select target first) |
| `p` | Pause tracking |
| `s` | Show current status |
| `h` | Show help |
| `q` | Quit |

---

## Tracking System

Each drone runs a GStreamer-based camera node that publishes frames to ROS2. The tracker node processes these frames to produce:

- **Detection** — locates the target object in the camera frame
- **Bounding box** — centre coordinates, size, and confidence score
- **Normalised error** — horizontal (`eₓ`) and vertical (`eᵧ`) pixel offset from frame centre:

$$e_x = \frac{x_\text{pixel} - c_x}{c_x}, \quad e_y = \frac{y_\text{pixel} - c_y}{c_y}$$

- **Distance proxy** — bounding box area used to approximate range to target
- **Boundary violation flag** — raised when target enters the close-proximity zone

---

## Control Pipeline

Commands are published at **50 Hz** via `OffboardControlMode` and `TrajectorySetpoint`.

### PID Controllers

| Axis | Controls |
|---|---|
| X | Forward / backward |
| Y | Left / right (yaw-corrected) |
| Z | Altitude |
| Distance PID | Approach / retreat to maintain safe follow distance |

### Velocity Smoothing

PID outputs are low-pass filtered to prevent jitter:

$$V_\text{smooth} = \alpha \cdot V_\text{new} + (1 - \alpha) \cdot V_\text{old}$$

### Frame Transformation (Body → NED)

PX4 uses the NED frame (X=North, Y=East, Z=Down). Camera-frame corrections are rotated to NED using the drone's current yaw (ψ):

$$v_\text{NED,x} = v_x \cos\psi - v_y \sin\psi$$
$$v_\text{NED,y} = v_x \sin\psi + v_y \cos\psi$$

---

## Trust System

Trust scores quantify each drone's reliability and govern task prioritisation. The trust model combines two components:

### Direct Trust

First-hand evaluation of a drone's own observable state, incorporating factors such as distance to its assigned target, battery level, environmental disturbance, and sensor/communication noise.

### Indirect Trust

Peer reputation aggregated from neighbouring drones, weighted by their own trustworthiness, link quality, and consistency of past recommendations.

---

## Collision Avoidance

Each drone's velocity is adjusted by a repulsive term that pushes it away from all other drones:

$$V_D = V_D + k \cdot \sum_j \frac{P_D - P_{D_j}}{\|P_D - P_{D_j}\|^2}$$

This maintains a minimum separation distance in real time without requiring centralised coordination.

---

## Logs & Data

Trust and telemetry values are logged per timestep to CSV for analysis and debugging. Each row records:

- Timestamp
- Observer drone and target drone IDs
- Target position (x, y, z) and velocity
- Collision flags
- Communication quality
- Distance to target
- Battery level
- Environmental and noise factors
- Direct trust, indirect trust, and combined trust scores

Sample logs are included in the repository root and in `drone_tracking_logs_20251003_122609/`.

---

## References

1. [PX4 Autopilot Documentation](https://docs.px4.io/main/en/)
2. [PX4 ROS2 User Guide](https://docs.px4.io/main/en/ros2/user_guide)
3. [PX4 Video Streaming Guide](https://docs.px4.io/v1.12/en/companion_computer/video_streaming.html)

---

*Project report: ECE F366 Laboratory Project — BITS Pilani, Pilani Campus (December 2025)*  
*Supervisor: Dr. Meetha V. Shenoy*
