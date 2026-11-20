# Timeline

## Phase 0: Concept (done)

- [x] Problem statement and math ([PROBLEM_STATEMENT.md](PROBLEM_STATEMENT.md))
- [x] Survey of 15 solutions ([SOLUTIONS.md](SOLUTIONS.md))
- [x] Comparison and scoring ([ANALYSIS_SOLUTIONS.md](ANALYSIS_SOLUTIONS.md))
- [x] Design chosen: stereo camera + Raspberry Pi 5 + PX4
- [x] Parts list ([BOM.md](BOM.md)) and objectives ([OBJECTIVES.md](OBJECTIVES.md))

## Phase 1: Hardware on the bench (weeks 1–3)

- [ ] Buy the parts (Pi 5, stereo camera, Kakute H7, ELRS radios, battery)
- [ ] Flash PX4 on the Kakute H7; check sensors and manual flight
- [ ] Build a safe test rig for hover tests
- [ ] Ubuntu + ROS 2 Humble + mavros + Nav2 on the Pi 5
- [ ] Check the ELRS link, manual override and failsafes (lost link, low battery)

## Phase 2: Sensing (weeks 3–6)

- [ ] Calibrate the stereo camera
- [ ] Publish rectified stereo images in ROS 2 at ≥ 30 Hz
- [ ] Depth maps and point clouds
- [ ] Measure IMU-only drift as the baseline
- [ ] Add optical flow / rangefinder (optional)

## Phase 3: Position estimate and SLAM (weeks 6–9)

- [ ] Visual-inertial odometry (ORB-SLAM3 or VINS-Fusion)
- [ ] Fuse vision + IMU (+ flow / rangefinder) with an EKF or factor graph
- [ ] Loop closure and map reuse to keep drift bounded
- [ ] Measure hover drift and trajectory error against the KPIs
- [ ] Failure tests: blocked camera, saturated IMU, low light

## Phase 4: Planning and obstacle avoidance (weeks 9–11)

- [ ] Costmaps from depth / point clouds
- [ ] Tune Nav2 for cluttered spaces
- [ ] Test avoidance of still and moving obstacles
- [ ] Velocity commands to PX4 over MAVLink

## Phase 5: Language commands (weeks 11–13)

- [ ] Turn a command ("go to the window") into a target in the map
- [ ] Run an LLM onboard or over a fast link, with fallbacks
- [ ] Lab demo: "go to the window"
- [ ] Safety: check every LLM target against the costmap before flying

## Phase 6: Full system (weeks 13–16)

- [ ] Fly the whole chain: perception → estimation → planning → control
- [ ] Indoor GPS-free mission: map and fly to commanded targets
- [ ] Dark / smoke / glare tests
- [ ] Sign off the KPIs: trajectory error, hover drift, latency, ≤ 15 W, ≥ 30 min
- [ ] Documentation and handover

## Milestones

| Week | Milestone |
| :--- | :--- |
| 3 | Hardware built, PX4 manual flight |
| 6 | Stereo depth at ≥ 30 Hz |
| 9 | Visual SLAM with bounded drift in a mapped room |
| 11 | Obstacle-avoiding flight in a cluttered space |
| 13 | LLM "go to the window" demo |
| 16 | Full GPS-free mission meeting the KPIs |
