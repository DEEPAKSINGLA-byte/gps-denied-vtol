# Project Timeline — GPS-Denied Autonomous UAV Navigation


## Phase 0 — Concept & Requirements (Complete)

- [x] Problem statement and formal MAP formulation
- [x] 15 candidate solution survey (`SOLUTIONS.md`)
- [x] Quantitative decision-matrix analysis + MCDA (`ANALYSIS_SOLUTIONS.md`)
- [x] Architecture selection: Stereo Camera + Raspberry Pi 5 + PX4
- [x] Hardware BOM with pricing (`BOM.md`)
- [x] Objectives definition (`OBJECTIVES.md`)

## Phase 1 — Hardware Bring-up & Bench Setup (Weeks 1–3)

- [ ] Procure all BOM components (RPi 5 8GB, SC132GS stereo cam, Kakute H7, ELRS radios, battery)
- [ ] Flash PX4 on the Kakute H7; verify sensor logs + manual flight mode
- [ ] Set up bench rig / test frame for hover-safe development
- [ ] Install Ubuntu on RPi 5; ROS 2 Humble + mavros + Nav2 baseline
- [ ] Verify ELRS link, RC override path, and failsafe triggers (lost-link, low-battery)

## Phase 2 — Sensing & Perception (Weeks 3–6)

- [ ] Stereo camera intrinsics/extrinsics calibration
- [ ] ROS 2 camera pipeline: rectified stereo publish at ≥30 Hz
- [ ] Depth map + point-cloud generation (OpenCV / stereo-matching)
- [ ] IMU-only dead-reckoning harness to measure baseline drift (KPI comparison)
- [ ] Optical-flow / rangefinder (optional) integration

## Phase 3 — State Estimation & SLAM (Weeks 6–9)

- [ ] V-SLAM / visual-inertial odometry bring-up (ORB-SLAM3, VINS-Fusion)
- [ ] EKF / factor-graph fusion of vision + IMU (+ flow/rangefinder)
- [ ] Loop closure and map reuse for bounded drift
- [ ] Hover-drift and ATE benchmarking against KPIs
- [ ] Graceful-degradation tests: blocked camera, saturated IMU, low light

## Phase 4 — Planning & Obstacle Avoidance (Weeks 9–11)

- [ ] Costmap generation from depth/point cloud
- [ ] Nav2 global + local planner tuning for cluttered environments
- [ ] Obstacle-avoidance validation against static and dynamic objects
- [ ] Offboard velocity setpoints via MAVLink to PX4

## Phase 5 — LLM Command Following (Weeks 11–13)

- [ ] Semantic target grounding: natural-language command → local map target
- [ ] Onboard / low-latency LLM integration and prompt/fallback design
- [ ] "Go to the window"-style end-to-end demo in the lab
- [ ] Safety gating: LLM intent verified against costmap before execution

## Phase 6 — Integration & Validation (Weeks 13–16)

- [ ] Full-stack flight: perception → estimation → planning → control
- [ ] Indoor GNSS-denied mission test (mapping + commanded navigation)
- [ ] Dark / smoke / glare robustness passes
- [ ] KPI sign-off: ATE, hover drift, latency, power (≤15 W), mission ≥30 min
- [ ] Documentation, reproducibility notes, and handoff

## Milestones

| Milestone | Target Date (Week) |
| :--- | :--- |
| Hardware assembled + PX4 manual flight | W3 |
| Stereo depth pipeline live at ≥30 Hz | W6 |
| V-SLAM with bounded drift in a mapped room | W9 |
| Obstacle-avoidance flight in cluttered space | W11 |
| LLM "go to the window" demo | W13 |
| Full KPI-validated GNSS-denied mission | W16 |
