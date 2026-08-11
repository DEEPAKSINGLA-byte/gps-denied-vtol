# Autonomous Drone: Sensor Architecture & BOM Evaluation

This repository documents the hardware architectures, bill of materials (BOM), and proposed workflows for our autonomous drone project. The goal is to evaluate three distinct sensor suites for ROS 2 navigation, balancing cost, computational overhead, and stability. 

## 1. 2D LiDAR + Optical Flow (The Hybrid Architecture)

This architecture isolates mapping data (geometry) from velocity data. This is optimal for achieving exceptional hover stability independent of visual odometry drift. 

### Budget Breakdown

| # | Component | Price (INR) | Link |
| :--- | :--- | :--- | :--- |
| 1 | RPLiDAR A1M8 | ₹8,899 | [Link](#) |
| 2 | PMW3901 Optical Flow Sensor | ₹3,212 | [Link](#) |
| 3 | Raspberry Pi 5 — 8GB | ₹17,500 | [Link](#) |
| 4 | Holybro Kakute H7 V1.3 | ₹10,217 | [Link](#) |
| 5 | RadioMaster Pocket ELRS | ₹7,000 | [Link](#) |
| 6 | RadioMaster RP1 ELRS Nano | ₹2,600 | [Link](#) |
| 7 | Tattu R-Line 850mAh 14.8V 150C | ₹2,300 | [Link](#) |
| | **Total** | **₹51,728.00** | |
*(Note: Dropping to a 4GB Pi reduces this to roughly ₹47,600).*

### Development Workflow 
*   **Hardware & Analog Interfacing:** Focuses heavily on the electrical side, including wiring the SPI/I2C buses for the optical flow sensor and designing custom power distribution and PWM control circuits for the LiDAR motor.
*   **Software & Middleware:** Requires writing custom C++ and Python scripts for PID control loops to maintain optical flow velocity hold, alongside managing Nav2 and Gazebo simulations in an Ubuntu Linux environment.

---

## 2. Stereo Camera + Raspberry Pi 5 8GB (Pure V-SLAM)

This setup relies entirely on visual data to map the environment in dense 3D point clouds, leveraging maximum memory overhead.

### Budget Breakdown

| # | Component | Price (INR) | Link |
| :--- | :--- | :--- | :--- |
| 1 | SC132GS Binocular Stereo Camera | ₹6,061.66 | [Link](#) |
| 2 | Raspberry Pi 5 — 8GB | ₹17,500 | [Link](#) |
| 3 | Holybro Kakute H7 V1.3 | ₹10,217 | [Link](#) |
| 4 | RadioMaster Pocket ELRS | ₹7,000 | [Link](#) |
| 5 | RadioMaster RP1 ELRS Nano | ₹2,600 | [Link](#) |
| 6 | Tattu R-Line 850mAh 14.8V 150C | ₹2,300 | [Link](#) |
| | **Total** | **₹45,678.66** | |

### Development Workflow 
*   This architecture shifts focus away from custom electrical hardware and heavily into middleware. The primary challenge is configuring and tuning ROS 2 packages (like RTAB-Map) to prevent visual odometry drift in featureless environments like hallways.

---

## 3. Monocular RGB Camera + 2D LiDAR (Vision + Geometry)

This hybrid approach leverages LiDAR for geometric mapping while freeing up the camera for visual tasks, avoiding the extreme processing overhead of stereo depth-mapping.

### Budget Breakdown

| # | Component | Price (INR) | Link |
| :--- | :--- | :--- | :--- |
| 1 | RPLiDAR A1M8 | ₹8,899 | [Link](#) |
| 2 | Raspberry Pi Camera Module 3 | ₹3,999 | [Link](#) |
| 3 | Raspberry Pi 5 — 8GB | ₹17,500 | [Link](#) |
| 4 | Holybro Kakute H7 V1.3 | ₹10,217 | [Link](#) |
| 5 | RadioMaster Pocket ELRS | ₹7,000 | [Link](#) |
| 6 | RadioMaster RP1 ELRS Nano | ₹2,600 | [Link](#) |
| 7 | Tattu R-Line 850mAh 14.8V 150C | ₹2,300 | [Link](#) |
| | **Total** | **₹52,515.00** | |

### Development Workflow
*   **Hardware & Geometry:** The spinning LiDAR handles all the heavy lifting for 2D mapping and obstacle avoidance in Nav2.
*   **Software & Vision:** The single RGB camera is utilized to run custom OpenCV Python scripts for ArUco tag detection, color tracking, or basic visual odometry to correct LiDAR drift.

---

## System Requirements
*   **OS:** Ubuntu Linux (Native or WSL)
*   **Frameworks:** ROS 2 (Humble/Iron), Nav2
*   **Simulation:** Gazebo 
*   **Languages:** C++, Python
