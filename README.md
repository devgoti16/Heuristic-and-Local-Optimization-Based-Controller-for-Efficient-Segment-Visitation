# Heuristic and Local-Optimization-Based Controller for Efficient Segment Visitation

**Short Description:**
Combines heuristic nearest-neighbor planning with local 2-opt optimization for path refinement and smooth execution.

## 🔹 Overview

This controller is designed for a **simulated robot in ROS 2** that must traverse multiple target line segments efficiently.
It integrates **heuristic planning** with **local optimization** to determine an optimal visitation order and crossing points for each segment.
The controller uses **ROS 2 topics** for robot pose (`/pose`), unvisited segments (`/unvisited_targets`), and publishes velocity commands (`/cmd_vel`).

**Key Features:**

* Fast initial path planning using nearest-neighbor heuristic.
* Path refinement with 2-opt local optimization to minimize total distance.
* Smart selection of segment crossing points to reduce unnecessary rotation.
* Smooth proportional control of linear and angular velocities.
* Full integration with ROS 2 for simulation and visualization in RViz.

---

## 🔹 Full Logic and Workflow

### **1. Target Acquisition**

* Subscribe to `/unvisited_targets` to receive all segment endpoints.
* Consecutive points are paired to form line segments.
* Segments are stored as a list: `[[(x1,y1),(x2,y2)], [(x3,y3),(x4,y4)], ...]`
* Robot maintains an internal map of visited vs unvisited segments.

### **2. Heuristic Planning (Nearest-Neighbor)**

* Robot starts at current pose `(x_r, y_r)`.
* Iteratively selects the nearest unvisited segment endpoint using **Euclidean distance**:

```text
d = sqrt((x_r - x_s)^2 + (y_r - y_s)^2)
```

* Produces a **fast initial visiting order** for all segments.
* Ensures the robot does not make long unnecessary detours.
* The heuristic provides a good initial approximation but may not be globally optimal.

### **3. Local Optimization (2-Opt Algorithm)**

* Refines the initial path using **2-opt swaps** to improve efficiency.
* For segments `i` and `j`, a swap is applied if it reduces the total path distance:

```text
D_total,new = sum(d_k,k+1) < D_total,old
```

* This step reduces total traversal distance and minimizes backtracking.
* 2-opt is iteratively applied until no further improvement is possible.

### **4. Segment Crossing and Execution**

* For each segment, select an **optimal crossing point** `P` minimizing combined cost of heading change and distance:

```text
Cost(P) = W_turn * Δθ + W_dist * d
```

Where:

* `Δθ` = heading change required to reach `P`
* `d` = distance from current robot position
* `W_turn`, `W_dist` = configurable weights for rotation vs distance

- **Motion Control:**

  * Linear velocity: `v = K_linear * d_target`
  * Angular velocity: `ω = K_angular * Δθ`
  * Smooth control ensures minimal oscillation and stable path tracking.

- **Segment Completion:**

  * Segment is marked visited when the robot is within a distance threshold and heading alignment is within tolerance.
  * Updates `/unvisited_targets` to remove completed segments.

- **Adaptive Decision Making:**

  * If multiple candidate crossing points exist, the one minimizing total cost and future path disruption is chosen.

### **5. Path Execution Summary**

1. Acquire all targets from simulator.
2. Form segments by pairing consecutive endpoints.
3. Plan initial path using nearest-neighbor heuristic.
4. Refine path with 2-opt optimization until no further improvement.
5. For each segment, select optimal crossing point minimizing heading change + distance.
6. Use smooth proportional control to move along path.
7. Mark segment as visited and continue until all segments are completed.
8. Optionally reset using `/reset` service to start new session.

### **6. Visualization in RViz**

* Robot model shows current pose and heading.
* MarkerArray displays all segments with visited/unvisited status.
* Path trace shows trajectory history.
* Colors differentiate completed segments from remaining ones.

![Traversal Screenshot](image.png)

### **7. Additional Notes**

* Configurable parameters such as `W_turn`, `W_dist`, `K_linear`, and `K_angular` allow tuning for different robot dynamics.
* The system is designed to be modular, allowing easy replacement of the heuristic or optimization method.
* Supports dynamic updates: if segments are added or removed during execution, the path is recalculated using the same logic.

---

This expanded README provides a complete explanation of the controller's logic, algorithms, and execution flow, suitable for GitHub display and understanding by other developers.
