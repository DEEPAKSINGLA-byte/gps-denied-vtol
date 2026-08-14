# Project Objectives — GPS-Denied Autonomous UAV Navigation

## Primary Objective
Develop a fully onboard navigation and state-estimation stack that enables a resource-constrained UAV to localize, navigate, and avoid obstacles in GNSS-denied environments — without reliance on satellite signals, pre-installed infrastructure, or offboard processing.

## Core Side Objectives

### 1. Obstacle Avoidance in Unstructured Space
Detect and avoid static and dynamic obstacles in cluttered, unmapped, and unstructured environments (industrial interiors, tunnels, mines, urban canyons) in real time.

### 2. Simultaneous Localization and Mapping
Build and reuse maps of unmapped territory, with loop-closure correction to bounded position drift over time.

### 3. Multi-Sensor Fusion and Graceful Degradation
Fuse heterogeneous sensors (stereo vision, IMU, optional flow/rangefinder) with cross-checking so the system degrades gracefully when a modality fails (blocked camera, saturated IMU) rather than diverging or crashing.

### 4. Real-Time Performance Within an Onboard Compute Envelope
Deliver high-rate state updates and low end-to-end latency entirely on onboard edge compute (Raspberry Pi 5), within a tight power and thermal budget — with no cloud or ground-station dependency.

### 5. Electronic Warfare and RF Resilience
Remain immune to RF jamming and spoofing, and prefer passive sensing modalities to preserve low observability (stealth) in contested airspace.

### 6. Precision Hover and Low-Speed Maneuvering
Hold stable station-keeping and execute precise takeoff, landing, and low-speed maneuvers without GPS assistance.

### 7. Energy Management for Sustained Missions
Complete extended missions (target ≥ 30 minutes) within battery and payload constraints, budgeting power across sensing, compute, and actuation.

### 8. Perception Robustness Across Environmental Conditions
Maintain reliable state estimation and obstacle detection across the full lighting spectrum — pitch-black, smoke/dust-filled, featureless, glare-heavy, and motion-blurred conditions — not just well-lit, textured environments.

### 9. Safety, Failsafes, and Manual Override
Provide continuous pilot override (ELRS radio link) and robust failsafe behaviors (lost-link, low-battery, sensor-failure) that transition to safe landing or return-to-base without GNSS.

### 10. Budget- and SWaP-Constrained Build
Achieve all of the above on a sub-500 g payload and a budget-constrained (~₹45.6k) hardware platform, keeping the architecture reproducible and scalable.

### 11. Natural-Language Command Following via LLMs
Integrate an onboard (or low-latency-linked) LLM so an operator can issue natural-language commands — e.g., "go to the window" — and the drone maps that intent to a semantic target in the local map, plans a route, and autonomously navigates there while avoiding obstacles, all without GPS.
