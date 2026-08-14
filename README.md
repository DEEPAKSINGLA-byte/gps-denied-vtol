# GPS-Denied Autonomous UAV Navigation

A research-oriented repository for designing, analyzing, and prototyping an **autonomous UAV navigation stack that operates without GNSS/GPS**, targeted at indoor, subterranean, and RF-contested (electronic-warfare) environments.

The project covers the full pipeline — from the formal problem statement and candidate hardware/software architectures, through quantitative decision analysis, to a concrete build budget — for a stereo-vision + Raspberry Pi 5 based drone.

---

## Table of Contents

- [Overview](#overview)
- [Why GPS-Denied Navigation?](#why-gps-denied-navigation)
- [Repository Contents](#repository-contents)
- [Google Drive — Documents](#google-drive--documents)
- [1. Problem Statement](#1-problem-statement)
- [2. Candidate Solutions](#2-candidate-solutions)
- [3. Quantitative Analysis](#3-quantitative-analysis)
- [4. Hardware Bill of Materials (BOM)](#4-hardware-bill-of-materials-bom)
- [5. Reproducing the Analysis](#5-reproducing-the-analysis)
- [System Architecture](#system-architecture)
- [Key Performance Indicators](#key-performance-indicators)
- [Roadmap](#roadmap)
- [Contributing](#contributing)
- [License](#license)

---

## Overview

Modern drones depend on GNSS (GPS/GLONASS/Galileo/BeiDou) for positioning. When that signal is blocked or jammed, low-cost MEMS IMUs default to dead-reckoning, and position error grows **quadratically** with time — a guaranteed crash in minutes.

This repository explores **15 candidate navigation modalities** for solving that problem and narrows them down through a statistical decision framework against a concrete, budget-constrained build.

**Selected architecture (per `BOM.md`):** Stereo Camera + Raspberry Pi 5 8GB + Holybro Kakute H7 flight controller (PX4) — a pure V-SLAM / visual-odometry configuration.

---

## Why GPS-Denied Navigation?

GNSS-denial arises from two broad operational domains:

| Domain | Examples |
| :--- | :--- |
| **Physical obstruction** | Subterranean mines, tunnels, indoor factories, urban canyons, forest canopies |
| **Electronic warfare (EW)** | RF jamming, spoofing, electromagnetic interference in contested airspace |

Without external position fixes, standard flight stacks suffer:

- **Quadratic position drift** — $O(t^2)$ accumulation of IMU bias/thermal noise.
- **Unbounded velocity error** from orientation drift.
- **Single-sensor failure modes** — vision fails in darkness/smoke; LiDAR is heavy and SWaP-hungry; active RF beacons violate stealth.

See [PROBLEM_STATEMENT.md](PROBLEM_STATEMENT.md) for the full formalization.

---

## Repository Contents

| File | Description |
| :--- | :--- |
| `PROBLEM_STATEMENT.md` | Formal problem definition, state-space math, MAP formulation, KPIs |
| `SOLUTIONS.md` | 15 hardware/software architectures for GPS-denied navigation |
| `ANALYSIS_SOLUTIONS.md` | Statistical decision-matrix analysis of those 15 solutions |
| `analysis.py` | Python script that generates the comparative charts |
| `1_swap_tradeoff.png` | SWaP chart — payload mass vs. power draw (bubble = drift) |
| `2_drift_rate_comparison.png` | Drift rate benchmark (bar chart) |
| `3_cost_benchmark_inr.png` | Hardware cost comparison in INR (bar chart) |
| `4_robustness_heatmap.png` | Environmental robustness heatmap |
| `BOM.md` | Hardware bill of materials with pricing and rationale |
| `images/` | Supporting figures |

---

## Google Drive — Documents

All related documentation and design artifacts are stored in the shared Google Drive folder:

**📁 [Project Documents on Google Drive](https://drive.google.com/drive/folders/1eQoDXTlVj3ZABpU9uxMFRTQ5T66ZaTJB?usp=sharing)**

---

## 1. Problem Statement

**Formal objective:**

> Design, construct, and validate a fully onboard, real-time navigation and state-estimation stack for a resource-constrained UAV that continuously estimates its state trajectory $\mathbf{x}_{0:T}$ within bounded error thresholds ($\epsilon < \delta$) over extended operational durations ($T_{mission} \ge 30 \text{ mins}$) without relying on external satellite signals or pre-installed infrastructure.

This is solved as a **maximum a posteriori (MAP) estimation** problem over the factor graph:

$$\mathbf{x}_{0:T}^* = \arg\max_{\mathbf{x}_{0:T}} \left[ p(\mathbf{x}_0) \prod_{t=1}^{T} p(\mathbf{x}_t \mid \mathbf{x}_{t-1}, \mathbf{u}_t) \prod_{k \in \mathcal{Z}_t} p(\mathbf{z}_{t,k} \mid \mathbf{x}_t) \right]$$

Where $\mathbf{u}_t$ are IMU/control inputs and $\mathbf{z}_{t,k}$ are exteroceptive measurements (camera features, LiDAR point clouds, rangefinder readings).

Full details — state vector, biases, constraints, and scope boundaries — in [PROBLEM_STATEMENT.md](PROBLEM_STATEMENT.md).

---

## 2. Candidate Solutions

[SOLUTIONS.md](SOLUTIONS.md) catalogs **15 distinct navigation architectures**, summarized below:

### Vision-Based
| # | Solution | Key Idea |
| :--- | :--- | :--- |
| 2 | Stereo Camera + Raspberry Pi 5 | Pure V-SLAM; dense 3D point cloud from stereo disparity |
| 4 | Event-Based Camera Odometry | Neuromorphic cameras; zero motion blur, low latency |
| 5 | Visual SLAM | Landmark memory with global loop closure |
| 7 | Optical Flow + Rangefinder | Downward camera + laser altimeter for perfect hover |
| 8 | ArUco / Fiducial Tag Tracking | Pre-printed tags as vision-based localization anchors |
| 9 | Three RGB Cameras + RPi 5 | Panoramic field-of-view for feature retention |
| 10 | Depth Camera + RPi 5 | Active IR projection; ready-to-use depth/point clouds |

### Laser & Range-Based
| # | Solution | Key Idea |
| :--- | :--- | :--- |
| 1 | 2D LiDAR + Optical Flow | Hybrid: LiDAR for layout, flow sensor for velocity |
| 3 | 2D LiDAR + Monocular RGB | LiDAR for 2D mapping, camera for vision tasks |
| 6 | Acoustic / Ultrasonic Array | Echolocation-style; cheap, works in pitch-black |
| 11 | 3D LiDAR + RPi 5 | 360° topographical map; full obstacle avoidance |
| 12 | FMCW mmWave Radar | Radio-frequency ranging; immune to smoke/dust |

### Beacon & Geophysical
| # | Solution | Key Idea |
| :--- | :--- | :--- |
| 13 | UWB Beacon Triangulation | Miniature indoor GPS via radio ping timing |
| 14 | Magnetic Anomaly Navigation (MAGNAV) | Passive matching of Earth's magnetic field |
| 15 | TERCOM / SITAN | Terrain contour matching against a DEM |

---

## 3. Quantitative Analysis

[ANALYSIS_SOLUTIONS.md](ANALYSIS_SOLUTIONS.md) provides a statistical and decision-matrix framework over the 15 solutions, including:

- **Quantitative performance matrix** — drift rate, power draw, payload mass, update rate, operational range.
- **Accuracy vs. SWaP clustering** — three Pareto-frontier clusters (ultra-low SWaP / balanced / high-reliability).
- **Robustness matrix** — 1–5 scores across darkness, smoke/dust, glare, RF jamming, and unmapped territory.
- **Multi-Criteria Decision Analysis (MCDA)** — weighted top-3 rankings for three mission profiles.

### Top Rankings by Mission Profile

| Mission Profile | 1st | 2nd | 3rd |
| :--- | :--- | :--- | :--- |
| **Defense / Contested** | FMCW mmWave Radar | 3D LiDAR SLAM | Event-Camera Odometry |
| **Underground Inspection** | 3D LiDAR SLAM | FMCW mmWave Radar | Depth Camera (IR) |
| **Sub-250g Micro UAV** | Optical Flow + Rangefinder | UWB Beacon Triangulation | ArUco Tag Tracking |

The charts below are generated by [`analysis.py`](analysis.py).

![SWaP Trade-Off](1_swap_tradeoff.png)
![Drift Rate Benchmark](2_drift_rate_comparison.png)
![Cost Benchmark](3_cost_benchmark_inr.png)
![Robustness Heatmap](4_robustness_heatmap.png)

---

## 4. Hardware Bill of Materials (BOM)

The selected build is a **stereo vision + edge-compute** configuration. Full details and rationale in [BOM.md](BOM.md).

| Component | Purpose | Price (INR) |
| :--- | :--- | ---: |
| SC132GS Binocular Stereo Camera | Visual odometry, depth, ArUco detection | ₹6,061.66 |
| Raspberry Pi 5 — 8GB | Perception, depth, costmap, navigation | ₹17,500 |
| Holybro Kakute H7 V1.3 | Flight controller (PX4) | ₹10,217 |
| RadioMaster Pocket ELRS | Manual override / pilot control | ₹7,000 |
| RadioMaster RP1 ELRS Nano | Receiver on drone | ₹2,600 |
| Tattu R-Line 850mAh 4S 150C | Power (~10 min flight time) | ₹2,300 |
| | **Total** | **₹45,678.66** |

### Why this stack?
- **Stereo camera** provides both visual odometry *and* depth — no active lasers needed.
- **Raspberry Pi 5 (8GB)** has the compute headroom for V-SLAM, point clouds, and Nav2-style planning on-board.
- **Kakute H7** runs PX4 in real-time for stabilization while the Pi handles perception.
- **ELRS radio link** guarantees a manual safety override during autonomous flight.

---

## 5. Reproducing the Analysis

### Prerequisites

- Python 3.8+
- Dependencies: `pandas`, `matplotlib`, `seaborn`

```bash
pip install pandas matplotlib seaborn
```

### Generate the charts

```bash
python3 analysis.py
```

This regenerates all four PNG charts in the repository root:

1. `1_swap_tradeoff.png` — SWaP trade-off scatter (log-log)
2. `2_drift_rate_comparison.png` — drift benchmark
3. `3_cost_benchmark_inr.png` — cost benchmark
4. `4_robustness_heatmap.png` — robustness heatmap

---

## System Architecture

```
              +-----------------------------+
              |    Heterogeneous Sensors    |
              | (Stereo Cam, IMU, ...)      |
              +--------------+--------------+
                             |
                             v
              +-----------------------------+
              |  Raspberry Pi 5             |
              |  Feature Extraction /       |
              |  Depth & Point Cloud        |
              |  V-SLAM / ArUco             |
              +--------------+--------------+
                             |
                             v
              +-----------------------------+
              |  State Estimator            |
              |  (EKF / Factor Graph)       |
              +--------------+--------------+
                             |
                             v
              +-----------------------------+
              |  Local Map & Loop Closure   |
              |  (SLAM / Costmap)           |
              +--------------+--------------+
                             |
                             v
              +-----------------------------+
              |  Kakute H7 (PX4)            |
              |  Flight Controller          |
              +-----------------------------+

  RadioMaster Pocket (ELRS)  --manual override-->  Kakute H7
```

---

## Key Performance Indicators

| Metric | Target | Baseline (Dead-Reckoning) |
| :--- | :--- | :--- |
| Absolute Trajectory Error (ATE) | $< 0.5\%$ of path length | $> 10\%$ |
| Stationary Hover Drift | $\le \pm 5 \text{ cm}$ / 10 min | Unbounded ($\ge 2$ m/min) |
| State Update Rate | $\ge 100 \text{ Hz}$; Vision $\ge 30 \text{ Hz}$ | Frame drop → filter divergence |
| Onboard Compute Envelope | $\le 15 \text{ W}$ | Off-board cloud reliance |
| Lighting Adaptability | $0$ – $10{,}000+$ lux | Fails below 50 lux |

---

## Roadmap

- [ ] Baseline IMU-only dead-reckoning harness for drift measurement
- [ ] Stereo camera intrinsic calibration + ROS 2 camera pipeline on RPi 5
- [ ] V-SLAM / visual-odometry bring-up (e.g., ORB-SLAM3, VINS-Fusion)
- [ ] Depth & point-cloud generation for obstacle avoidance
- [ ] PX4 integration via MAVLink; offboard velocity control
- [ ] ArUco marker localization testbed
- [ ] Hover-drift and ATE benchmarking against KPIs above

---

## Contributing

This is an exploratory research repository. Suggestions, test data, and implementation PRs are welcome.

1. Fork the repository.
2. Create a feature branch: `git checkout -b feature/my-idea`
3. Commit your changes and open a pull request.

---

## License

Not yet specified. See the repository owner for licensing terms.
