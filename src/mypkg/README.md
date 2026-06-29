<div align="center">



# 🤖 URDFBot — Coffee Shop Autonomous Navigation System

**A production-ready ROS 2 mobile robot platform with SLAM mapping and Nav2 autonomous navigation**

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble-blue?style=for-the-badge&logo=ros)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Gazebo-11+-orange?style=for-the-badge)](http://gazebosim.org/)
[![SLAM Toolbox](https://img.shields.io/badge/SLAM-Toolbox-green?style=for-the-badge)](https://github.com/SteveMacenski/slam_toolbox)
[![Nav2](https://img.shields.io/badge/Nav2-Stack-red?style=for-the-badge)](https://navigation.ros.org/)
[![Python](https://img.shields.io/badge/Python-3.8+-yellow?style=for-the-badge&logo=python)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-purple?style=for-the-badge)](LICENSE)

</div>

---

## 📡 System Overview

> **`ros_ws4/src/mypkg`** — A fully integrated autonomous mobile robot system designed to map and navigate indoor environments (coffee shop layout) using Simultaneous Localization and Mapping (SLAM) and autonomous path planning via Nav2.

The robot perceives its environment through a 360° LiDAR sensor and an RGB camera, communicates over standard ROS 2 topics, and is physically simulated in Gazebo with accurate friction, inertia, and differential drive dynamics.

### Robot in Simulation

![URDFBot in Gazebo Coffee Shop](images/01-gazebo-simulation.png)

---

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                        ros_ws4 Workspace                            │
│                                                                     │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────────────┐  │
│  │  URDF Model  │    │   Gazebo     │    │   ROS 2 Topic Bus    │  │
│  │  urdfbot_    │───▶│  Simulation  │───▶│                      │  │
│  │  simple.urdf │    │  (Physics)   │    │  /scan  /odom        │  │
│  └──────────────┘    └──────┬───────┘    │  /cmd_vel /tf        │  │
│                             │            │  /camera/image_raw   │  │
│  ┌──────────────┐           │            └──────────┬───────────┘  │
│  │  Teleop /    │           │                       │              │
│  │  Nav2 Goals  │──▶ /cmd_vel                       │              │
│  └──────────────┘           │              ┌────────▼───────┐      │
│                             │              │  SLAM Toolbox  │      │
│  ┌──────────────┐           │              │  (Map Builder) │      │
│  │  RViz2       │◀──────────┘              └────────┬───────┘      │
│  │  Visualizer  │◀─────────────────────────────────▼───────────┐  │
│  └──────────────┘              Nav2 Stack (Path Planner)        │  │
│                                 ┌──────────────────────────────┘  │
│                                 │  BT Navigator → Controller       │
│                                 │  Costmap → Planner → Recovery    │
└─────────────────────────────────┴──────────────────────────────────┘
```

### System Architecture Diagram

![System Block Diagram](images/02-system-architecture.png)

---

## 🤖 Robot Hardware Specifications

### Chassis

| Parameter | Value |
|-----------|-------|
| Dimensions | 0.65 × 0.45 × 0.16 m |
| Total Mass | 14 kg |
| Drive Type | Differential Drive |
| Color | Orange |

### Drive Wheels

| Parameter | Value |
|-----------|-------|
| Wheel Radius | 0.12 m |
| Wheel Length | 0.07 m |
| Wheel Separation | 0.54 m |
| Mass (each) | 1.5 kg |
| Friction Coefficient (μ) | 1.5 |
| Joint Type | Continuous (no limits) |

### Caster Wheels (Stability)

| Position | X | Y | Friction |
|----------|---|---|---------|
| Front | +0.25 m | 0.00 m | μ = 0.0 |
| Rear-Left | −0.25 m | +0.15 m | μ = 0.0 |
| Rear-Right | −0.25 m | −0.15 m | μ = 0.0 |

> Frictionless casters allow smooth 360° rotation without resistance.

---

## 🔭 Sensor Suite

### LiDAR Scan Visualization

![360° LiDAR Point Cloud](images/03-lidar-scan.png)

### 🟢 LiDAR — Primary SLAM Sensor

```
┌────────────────────────────────────┐
│  Sensor: 2D Rotating LiDAR        │
│  Topic:  /scan                     │
│  Type:   sensor_msgs/LaserScan     │
├────────────────────────────────────┤
│  Scan Range:    360°               │
│  Samples/Scan:  360  (1°/sample)   │
│  Update Rate:   10 Hz              │
│  Min Range:     0.12 m             │
│  Max Range:     10.0 m             │
│  Resolution:    0.01 m             │
│  Noise (σ):     0.015 m (Gaussian) │
└────────────────────────────────────┘
```

### 📷 RGB Camera — Visual Perception

```
┌────────────────────────────────────┐
│  Sensor: Front-Facing RGB Camera   │
│  Topic:  /camera/image_raw         │
│  Type:   sensor_msgs/Image         │
├────────────────────────────────────┤
│  Resolution:   640 × 480 px        │
│  Frame Rate:   30 Hz               │
│  H-FOV:        80°                 │
│  Frame:        camera_optical_frame│
│  Convention:   Z-fwd, X-right      │
└────────────────────────────────────┘
```

---

## 🧩 Transform Tree (TF Frames)

```
odom
 └── base_footprint          ← Ground-level navigation reference
      └── base_link           ← Chassis center (0.12 m above ground)
           ├── wheel_left      ← Left drive wheel joint
           ├── wheel_right     ← Right drive wheel joint
           ├── caster_front    ← Front stability caster
           ├── caster_rear_l   ← Rear-left stability caster
           ├── caster_rear_r   ← Rear-right stability caster
           ├── mast_link       ← Vertical sensor mast
           │    └── lidar_link ← LiDAR sensor (0.22 m above mast)
           └── camera_link
                └── camera_optical_frame ← ROS standard camera axes
```

### RViz2 Transform Visualization

![TF Tree in RViz2](images/04-rviz2-tf-tree.png)

---

## 🚀 Launch Files

| File | Purpose | Key Nodes Started |
|------|---------|-------------------|
| `gazebo.launch.py` | Basic simulation + teleop | Gazebo, robot spawner, teleop_twist_keyboard |
| `slam.launch.py` | Full SLAM mapping pipeline | Gazebo + SLAM Toolbox (async) + RViz2 |
| `nav2.launch.py` | Autonomous navigation | Nav2 stack, map server, AMCL localizer |
| `display.launch.py` | Visualization only | robot_state_publisher + RViz2 |

---

## 🗂️ Project Structure

```
ros_ws4/
└── src/
    └── mypkg/
        ├── launch/
        │   ├── gazebo.launch.py      ← Basic simulation
        │   ├── slam.launch.py        ← SLAM mapping
        │   ├── nav2.launch.py        ← Autonomous nav
        │   └── display.launch.py     ← RViz only
        ├── urdf/
        │   └── urdfbot_simple.urdf   ← Robot model
        ├── config/
        │   ├── *.yaml                ← Nav2/SLAM configs
        │   └── *.rviz                ← RViz2 configs
        ├── worlds/
        │   └── coffee_shop.world     ← Gazebo environment
        ├── map/
        │   ├── coffee_shop_map.yaml  ← Saved map metadata
        │   └── coffee_shop_map.pgm   ← Occupancy grid image
        ├── package.xml
        └── setup.py
```

---

## ⚙️ Gazebo Plugin Configuration

### Differential Drive Plugin
```xml
<plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
  <commandTopic>cmd_vel</commandTopic>
  <odometryTopic>odom</odometryTopic>
  <odometryFrame>odom</odometryFrame>
  <robotBaseFrame>base_footprint</robotBaseFrame>
  <wheelSeparation>0.54</wheelSeparation>
  <wheelDiameter>0.24</wheelDiameter>
</plugin>
```

### Physics Parameters (Wheels)
```xml
<surface>
  <friction>
    <ode>
      <mu>1.5</mu>     <!-- Drive wheels: high grip -->
      <mu2>1.5</mu2>
    </ode>
  </friction>
  <contact>
    <ode>
      <kp>500000</kp>  <!-- Spring constant -->
      <kd>10.0</kd>    <!-- Damping coefficient -->
    </ode>
  </contact>
</surface>
```

---

## 🗺️ Operational Workflow

### Phase 1 — Environment Mapping (SLAM)

```bash
# 1. Launch the SLAM pipeline
ros2 launch mypkg slam.launch.py

# 2. Open a new terminal and start teleoperation
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 3. Drive the robot around the coffee shop
#    SLAM Toolbox builds the map in real-time (visible in RViz2)
```

### SLAM Mapping in Progress

![RViz2 SLAM Mapping - Map Building](images/05-slam-mapping.png)

### Phase 2 — Save the Map

```bash
# Save the completed occupancy grid map
ros2 run nav2_map_server map_saver_cli \
  -f ~/ros_ws4/src/mypkg/map/coffee_shop_map

# Output files:
#   coffee_shop_map.pgm  ← Grayscale occupancy grid
#   coffee_shop_map.yaml ← Map metadata (resolution, origin)
```

### Phase 3 — Autonomous Navigation

```bash
# 1. Launch Nav2 with pre-built map
ros2 launch mypkg nav2.launch.py

# 2. In RViz2:
#    → Click "2D Pose Estimate" to set initial robot pose
#    → Click "2D Nav Goal"  to send navigation target

# Nav2 handles everything autonomously:
#   Path Planning (A* / Dijkstra) → Trajectory Control → Obstacle Avoidance
```

### Nav2 Path Planning

![Autonomous Navigation with Path Planning](images/06-nav2-planning.png)

---

## 🔄 Data Flow Diagram

```
┌─────────────────────────────────────────────────────────┐
│                    SLAM MODE                            │
│                                                         │
│  Teleop ──▶ /cmd_vel ──▶ DiffDrive Plugin               │
│                              │                          │
│                              ▼                          │
│                    Gazebo Physics Engine                 │
│                         │        │                      │
│                    /odom         /scan                   │
│                         │        │                      │
│                         ▼        ▼                      │
│              robot_state_pub   SLAM Toolbox             │
│                    │               │                    │
│                   /tf          /map (occupancy_grid)    │
│                    │               │                    │
│                    └───────┬───────┘                    │
│                            ▼                            │
│                          RViz2                          │
└─────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────┐
│                   NAV2 MODE                             │
│                                                         │
│  RViz2 Goal ──▶ /goal_pose ──▶ BT Navigator             │
│                                    │                   │
│                        ┌───────────┴──────────┐        │
│                        ▼                      ▼        │
│                 Global Planner          Local Planner   │
│                  (A*/Dijkstra)         (DWB Controller) │
│                        │                      │        │
│                        └───────────┬──────────┘        │
│                                    ▼                   │
│                              /cmd_vel                  │
│                                    │                   │
│                                    ▼                   │
│                            Gazebo / Real Robot         │
└─────────────────────────────────────────────────────────┘
```

---

## 🛠️ Build & Installation

### Prerequisites

```bash
# ROS 2 (Humble or later)
sudo apt install ros-humble-desktop

# Required packages
sudo apt install \
  ros-humble-slam-toolbox \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-teleop-twist-keyboard
```

### Build

```bash
# Clone / set up workspace
cd ~/ros_ws4

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install

# Source
source install/setup.bash
```

---

## 📊 Key ROS 2 Topics

| Topic | Message Type | Direction | Description |
|-------|-------------|-----------|-------------|
| `/scan` | `sensor_msgs/LaserScan` | OUT | 360° LiDAR scan data |
| `/cmd_vel` | `geometry_msgs/Twist` | IN | Velocity commands |
| `/odom` | `nav_msgs/Odometry` | OUT | Wheel odometry |
| `/map` | `nav_msgs/OccupancyGrid` | OUT | SLAM-built map |
| `/tf` | `tf2_msgs/TFMessage` | OUT | Transform tree |
| `/camera/image_raw` | `sensor_msgs/Image` | OUT | RGB camera feed |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | OUT | Camera calibration |
| `/goal_pose` | `geometry_msgs/PoseStamped` | IN | Nav2 target goal |
| `/local_costmap/costmap` | `nav_msgs/OccupancyGrid` | OUT | Local obstacle map |
| `/global_costmap/costmap` | `nav_msgs/OccupancyGrid` | OUT | Global plan costmap |

---

## 🧠 Technology Stack

| Layer | Technology | Version |
|-------|-----------|---------|
| Middleware | ROS 2 | Humble+ |
| Simulation | Gazebo | 11+ |
| Mapping | SLAM Toolbox | Latest |
| Navigation | Nav2 | Latest |
| Visualization | RViz2 | Latest |
| Robot Model | URDF/XML | — |
| Launch System | Python 3 | 3.8+ |
| Physics Engine | ODE (via Gazebo) | — |

---

## 📌 Notes & Extension Points

- **Object Detection:** The RGB camera (`/camera/image_raw`) is ready for integration with OpenCV or a YOLO model for detecting cups, people, or obstacles.
- **Multi-Robot:** The package structure supports multi-robot extension via namespace isolation.
- **Real Hardware:** Replace Gazebo plugins with actual hardware drivers (e.g., `rplidar_ros`, `v4l2_camera`) to deploy on physical hardware.
- **Custom Worlds:** Add new Gazebo `.world` files under `worlds/` and update the launch file path to switch environments.

---

## 📄 License

```
MIT License — Free to use, modify, and distribute.
See LICENSE file for full terms.
```

---

<div align="center">

**Built with ROS 2 · Gazebo · SLAM Toolbox · Nav2**

*Autonomous robots for the real world — one coffee shop at a time.*

</div>