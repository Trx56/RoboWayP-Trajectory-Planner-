# Robo-WayP: Robotic Arm Trajectory Planner 

[![Python](https://img.shields.io/badge/Python-3.8%2B-blue.svg)](https://www.python.org/)
[![NumPy](https://img.shields.io/badge/NumPy-Scientific%20Computing-orange.svg)](https://numpy.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

**Robo-WayP** is an interactive simulation and trajectory planning engine designed for a 3-link planar robotic arm. Developed as part of the Robotics and Automation curriculum at Saintgits College of Engineering, the project implements core robotic mathematics from scratch—ranging from analytical inverse kinematics to multi-polynomial trajectory smoothing and real-time obstacle avoidance[cite: 1, 13, 14].

---

##  Key Features

* **Analytical 3-DOF Inverse & Forward Kinematics:** Computes exact joint angles (`th1`, `th2`, `th3`) for target end-effector coordinates with support for **Elbow Up / Elbow Down** configuration switching[cite: 1, 14].
* **Advanced Trajectory Interpolation:** Features Quintic (5th-order polynomial for smooth jerk-free motion), Cubic (3rd-order), and Cartesian straight-line waypoint planners[cite: 1, 14].
* **Collision Detection & Clearance Mapping:** Dynamically tracks the minimum Euclidean distance between all three arm segments and workspace circular obstacles, triggering real-time collision alerts[cite: 1, 14].
* **2D & 3D Spatial Mapping:** Translates 2D planar kinematics into a 3D spatial manipulator model featuring turret base-yaw rotation and a 2-finger end-effector.

---

##  Tech Stack & Libraries
* **Language:** Python[cite: 1]
* **Core Math & Arrays:** NumPy[cite: 1]
* **Visualization:** Matplotlib / Interactive Interface Design[cite: 1, 13]

---

##  Code Architecture (`robowayp.py`)
```text
 Robo-WayP-Trajectory-Planner
 ┣  robowayp.py         # Core kinematics engine, IK solver, and interpolator
 ┗  README.md           # Project documentation

Core Mathematical Implementation Overview:Inverse Kinematics (solve_ik): Uses geometric law of cosines to resolve redundant link coordinates and continuous angle wrapping (_unwrap) to prevent sudden motor jumps.Quintic Trajectory Profiling (evaluate_at): Applies 5th-order polynomial equations ($10\tau^3 - 15\tau^4 + 6\tau^5$) to control position, velocity, and acceleration boundaries smoothly across task-space waypoints[cite: 14].
