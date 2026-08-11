# Formal Problem Statement: Autonomous UAV Navigation in GPS-Denied Environments

## Executive Summary
This document outlines the formal problem definition, technical bottlenecks, mathematical formulation, and evaluation metrics for autonomous Unmanned Aerial Vehicle (UAV) navigation, state estimation, and obstacle avoidance in GPS-denied, indoor, and EW-contested operational domains.

---

## 1. Context and Operational Domain

Modern Unmanned Aerial Vehicles (UAVs) rely fundamentally on Global Navigation Satellite Systems (GNSS—including GPS, GLONASS, Galileo, and BeiDou) paired with low-cost Micro-Electro-Mechanical Systems (MEMS) Inertial Measurement Units (IMUs) to estimate position, velocity, and orientation (attitude). 

However, operational deployment in critical scenarios inherently blocks or corrupts direct line-of-sight to satellite constellations:
* **Physical Obstruction:** Subterranean mines, tunnels, indoor industrial facilities, high-density urban canyons, and dense forest canopies.
* **Electronic Warfare (EW):** Contested defense operational environments featuring active RF jamming, spoofing, and electromagnetic interference (EMI).

In these environments, UAVs must achieve high-precision autonomous navigation, path planning, and hover stability purely through onboard sensing and computation.

---

## 2. Technical Bottlenecks & Core Problem

When GNSS signals become unavailable or corrupted, standard flight control stacks default to unconstrained **dead-reckoning**. In this state, small high-frequency sensor errors (bias, thermal noise, and scale factor inaccuracies) within onboard MEMS IMUs accumulate rapidly upon integration:
* **Position drift** scales quadratically ($O(t^2)$) over time.
* **Orientation error** leads to unbounded velocity estimation errors.

Within seconds to minutes, this drift causes catastrophic loss of control, mission failure, or high-speed collisions with obstacles. Alternative single-sensor modalities present distinct failure modes:
1. **Visual Systems (VIO / Visual SLAM):** Suffer complete failure in dark, smoke-filled, motion-blurred, or featureless environments (e.g., bare walls, uniform water).
2. **Active Range Sensing (LiDAR):** Imposes significant size, weight, power, and cost (SWaP-C) penalties while remaining susceptible to degraded performance in airborne dust, rain, or fog.
3. **RF / Beacon-Based Systems:** Active transmissions violate low-observable (stealth) operational requirements and require pre-installed infrastructure.

---

## 3. Mathematical Problem Formalization

### 3.1 State Representation
Let the 6-DOF (Degree of Freedom) state vector of the UAV at time step $t$ be defined as:

$$\mathbf{x}_t = \begin{bmatrix} \mathbf{p}_t \\ \mathbf{v}_t \\ \mathbf{q}_t \\ \mathbf{b}_a \\ \mathbf{b}_g \end{bmatrix} \in \mathbb{R}^{16}$$

Where:
* $\mathbf{p}_t = [p_x, p_y, p_z]^T \in \mathbb{R}^3$ represents 3D position in the World Frame ($\\mathcal{W}$).
* $\mathbf{v}_t = [v_x, v_y, v_z]^T \in \mathbb{R}^3$ represents linear velocity in the World Frame.
* $\mathbf{q}_t = [q_w, q_x, q_y, q_z]^T \in \mathbb{SO}(3)$ represents orientation as a unit quaternion relative to the Body Frame ($\\mathcal{B}$).
* $\mathbf{b}_a \in \mathbb{R}^3$ and $\mathbf{b}_g \in \mathbb{R}^3$ represent time-varying accelerometer and gyroscope biases, respectively.

### 3.2 Formal Problem Objective
> **Design, construct, and validate a fully onboard, real-time navigation and state-estimation stack for a resource-constrained UAV that continuously estimates its state trajectory $\mathbf{x}_{0:T}$ within bounded error thresholds ($\\epsilon < \\delta$) over extended operational durations ($T_{mission} \ge 30 \text{ mins}$) without relying on external satellite signals or pre-installed infrastructure.**

Mathematically, this corresponds to solving the maximum a posteriori (MAP) estimation problem over the factor graph / state history:

$$\mathbf{x}_{0:T}^* = \arg\max_{\mathbf{x}_{0:T}} \left[ p(\mathbf{x}_0) \prod_{t=1}^{T} p(\mathbf{x}_t \mid \mathbf{x}_{t-1}, \mathbf{u}_t) \prod_{k \in \mathcal{Z}_t} p(\mathbf{z}_{t,k} \mid \mathbf{x}_t) \right]$$

Where:
* $\mathbf{u}_t$ represents internal control inputs / IMU measurements.
* $\mathbf{z}_{t,k}$ represents heterogeneous exteroceptive sensor measurements (e.g., camera features, LiDAR point clouds, rangefinder readings).

---

## 4. Key Performance Indicators (KPIs) & Evaluation Criteria

| Performance Metric | Target Benchmark | Baseline (Standard Fallback) |
| :--- | :--- | :--- |
| **Absolute Trajectory Error (ATE)** | $< 0.5\%$ of total path length | $> 10\%$ (Pure IMU Dead-Reckoning) |
| **Stationary Hover Drift** | $\le \pm 5 \text{ cm}$ over 10 minutes | Unbounded ($\ge 2 \text{ m/min}$) |
| **System Processing Latency** | State updates $\ge 100 \text{ Hz}$; Vision $\ge 30 \text{ Hz}$ | Frame drop leading to EKF filter divergence |
| **Onboard Compute Envelope** | Power consumption $\le 15 \text{ W}$ | Relies on off-board cloud streaming |
| **Lighting Adaptability** | Operational from $0 \text{ lux}$ to $10,000+ \text{ lux}$ | Fails below $50 \text{ lux}$ |

---

## 5. System Architecture Overview

```
                      +-----------------------------+
                      |    Heterogeneous Sensors    |
                      | (IMU, Cameras, LiDAR, ToF)  |
                      +--------------+--------------+
                                     |
                                     v
                      +-----------------------------+
                      | Sensor Pre-processing &     |
                      | Feature Extraction          |
                      +--------------+--------------+
                                     |
                                     v
                      +-----------------------------+
                      |   State Estimator (EKF /    |
                      |   Factor Graph Optimization)|
                      +--------------+--------------+
                                     |
                                     v
                      +-----------------------------+
                      | Local Map & Loop Closure    |
                      | (SLAM / Point Cloud Grid)   |
                      +--------------+--------------+
                                     |
                                     v
                      +-----------------------------+
                      | Trajectory Planner &        |
                      | Flight Controller (PX4)     |
                      +-----------------------------+
```

---

## 6. Scope Boundaries & Constraints

### In Scope
* Real-time Multi-Sensor Fusion (Visual-Inertial Odometry, LiDAR SLAM, Optical Flow).
* Onboard edge computing (e.g., NVIDIA Jetson Orin Nano, Raspberry Pi Compute Module 4).
* Closed-loop flight control integration via MAVLink / ROS 2 with PX4 or ArduPilot.

### Out of Scope
* Reliance on ground control station (GCS) offloaded processing.
* Reliance on active RF beacons (UWB, Wi-Fi, Cellular) in unmapped hostile spaces.
* Continuous reliance on manual pilot intervention.