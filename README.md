# 🤖 Heuristic and Local-Optimization-Based Controller for Efficient Segment Visitation

**Short Description:**  
Combines heuristic nearest-neighbor planning with local 2-opt optimization for path refinement.

---

## 🧭 Overview

This project implements an **autonomous controller** for a simulated robot in **ROS 2** that efficiently visits multiple target line segments.  
It combines **heuristic planning** and **local optimization** to minimize traversal time and reduce unnecessary turns while ensuring complete coverage of all segments.

The controller interacts with the provided **sim2** simulator to visualize the robot, its path, and visited/unvisited targets in **RViz 2**.

---

## ⚙️ Key Features

- **Heuristic Nearest-Neighbor Planning** – Quickly determines an initial visiting order for segments.  
- **2-Opt Local Optimization** – Refines the route to reduce overall path length.  
- **Smart Segment Crossing** – Chooses the optimal crossing point (not always endpoints) to minimize heading changes.  
- **Smooth Motion Control** – Proportional velocity control for linear and angular movement with minimal oscillation.  
- **ROS 2 Integration** – Uses publishers, subscribers, and services (`/pose`, `/cmd_vel`, `/unvisited_targets`, `/reset`).  
- **RViz 2 Visualization** – Displays robot path, visited segments, and traversal trail.

---

## 🧩 System Architecture

```
+-------------------------+
|      Controller Node    |
|-------------------------|
| Subscribes:             |
|  • /pose                |
|  • /unvisited_targets   |
| Publishes:              |
|  • /cmd_vel             |
| Calls Service:          |
|  • /reset               |
+-------------------------+
           |
           v
+-------------------------+
|         sim2            |
|  (Simulation Node)      |
| Publishes visualization |
|  markers & robot state  |
+-------------------------+
```

---

## 🧠 Algorithm Workflow

1. **Acquire Targets**  
   Subscribe to `/unvisited_targets` for all segment endpoints.

2. **Form Segments**  
   Pair consecutive endpoints to form line segments.

3. **Heuristic Planning**  
   Start at the nearest segment to the robot and iteratively choose the nearest unvisited segment.

4. **Local Optimization (2-Opt)**  
   Apply 2-opt swaps to improve total path efficiency.

5. **Execution**  
   For each segment:
   - Select an execution point minimizing:  
     `Cost = TURN_WEIGHT * heading_change + DIST_WEIGHT * distance`
   - Align heading and move smoothly toward it.

6. **Completion**  
   Stop when all segments are crossed.

---

## 🚀 Run Instructions

### 1️⃣ Setup ROS 2 Workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2️⃣ Clone Packages
```bash
# sim2 package (provided for the course)
git clone <path_or_link_to_sim_package>

# this controller
git clone https://github.com/<your-username>/segment-visitation-controller.git
```

### 3️⃣ Build and Source
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### 4️⃣ Launch Simulation and Controller
In **Terminal 1**:
```bash
ros2 run sim sim2 --ros-args -p targets:=<target_file_name>
```

In **Terminal 2**:
```bash
ros2 run segment_visitation controller
```

---

## 🧪 Visualization in RViz 2

Add these displays:
- **RobotModel** → shows the simulated robot.  
- **MarkerArray (/sim_markers)** → shows line segments and traversal history.  
- **Path Trace** → displays robot movement.

---

## 📸 Example Screenshots

| Description | Screenshot |
|--------------|-------------|
| **Initial Setup** – Robot and unvisited targets | ![Initial Setup](assets/rviz_initial.png) |
| **During Traversal** – Robot crossing segments | ![Mid Execution](assets/rviz_mid.png) |
| **Completed Path** – All segments visited efficiently | ![Completed Path](assets/rviz_complete.png) |

> *(Save your screenshots from RViz as PNGs in an `assets/` folder.)*

---

## 🧮 Configuration Parameters

| Parameter | Meaning | Default |
|------------|----------|----------|
| `CONTROL_RATE_HZ` | Control loop frequency | 20 Hz |
| `DIST_THRESHOLD` | Distance to mark a segment as reached | 0.05 m |
| `ANGLE_THRESHOLD` | Heading error before rotation priority | 0.1 rad |
| `TURN_WEIGHT` | Cost weight per radian turn | 6.0 |
| `DIST_WEIGHT` | Cost weight per meter travel | 1.0 |
| `MAX_LINEAR_SPEED` | Maximum forward speed | 25 m/s |
| `MAX_ANGULAR_SPEED` | Maximum rotational speed | 25 rad/s |

---

## 📚 Dependencies

- ROS 2 (Humble / Iron)
- Python ≥ 3.8  
- `rclpy`, `geometry_msgs`, `sensor_msgs`, `std_srvs`  
- `tf2_ros` (via sim2 package)

---

## 🧑‍💻 Author

**Dev Pankajbhai Goti**  
CSCE 452/752 – Robotics and Spatial Intelligence, Fall 2025  
University of South Carolina  

---

## 🏁 Acknowledgments

- **Instructor:** Dr. Jason M. O’Kane  
- **Simulation:** Provided `sim2` package for ROS 2.

---

## 🧠 Summary

This project demonstrates a controller that integrates **heuristic planning** with **local optimization** to efficiently visit all target segments while minimizing unnecessary rotation and travel distance.  
It showcases practical skills in **motion planning, control design, and ROS 2 integration**, making it a standout project for any robotics portfolio.
