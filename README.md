<p align="center">
<img width="633" height="473" alt="image" src="https://github.com/user-attachments/assets/6d49e17d-afb5-40ae-9434-eaf62bd4b33f" />
</p>

<h1 align="center">GPS-Denied Autonomous UAV Navigation</h1>

<p align="center"><em>A drone that finds its way and avoids obstacles without GPS: indoors, underground, or under jamming.</em></p>

## Start here

| I want to… | Read |
| :--- | :--- |
| **Fly the real drone** | [docs/QUICKSTART.md](docs/QUICKSTART.md) |
| Set up a laptop or Pi (once) | [docs/SETUP.md](docs/SETUP.md) |
| Look up ports, topics, options | [docs/REFERENCE.md](docs/REFERENCE.md) |
| Use the Pi's tools | [raspberry_pi/README.md](raspberry_pi/README.md) |
| Run the simulator | [docs/SIMULATION.md](docs/SIMULATION.md) |
| Build and test code | [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md) |
| Read the research | [docs/RESEARCH.md](docs/RESEARCH.md) |

## Quick start

```bash
# 1. Laptop (first)
cd ~/gps-denied-vtol/ros2_ws && source install/setup.bash
ros2 launch x500_navigation navigation_real.launch.py pi_ip:=10.184.3.36

# 2. Raspberry Pi
ssh deimos@10.184.3.36
cd ~/raspberry_pi && python3 start_streams.py --nav-cmds
```

Then check, arm and fly: [QUICKSTART.md](docs/QUICKSTART.md#3-check-it-works).

## How the current system works

```
 Drone                                              Laptop (ROS 2 Humble, GPU)
 ┌───────────────────────────┐   Wi-Fi             ┌────────────────────────────────────┐
 │ Pixhawk (ArduPilot) ─USB─┐│ ──telemetry 14551─▶ │ /odom + TF                         │
 │ YDLIDAR ────────────USB──┤│ ──lidar     14552─▶ │ /scan            ┐                 │
 │ USB camera ──────────────┤│ ──camera  TCP 5000▶ │ /camera/image_raw┼▶ Nav2, YOLO+SAM │
 │        Raspberry Pi ◀────┘│ ◀─cmd_vel   14553── │ /cmd_vel ◀───────┘                 │
 └───────────────────────────┘                     └────────────────────────────────────┘
```

- **The Pi** (`raspberry_pi/`) sends the Pixhawk data, lidar and camera to the laptop, and
  passes speed commands back to the Pixhawk.
- **The laptop** (`ros2_ws/`) works out where the drone is, plans paths around obstacles
  (Nav2), recognises objects in the camera (YOLO + SAM), and shows everything in RViz.
- **The simulator** uses PX4 + Gazebo ([SIMULATION.md](docs/SIMULATION.md)).

The end goal is to do all of this on the drone itself; see [RESEARCH.md](docs/RESEARCH.md).

## Why

Without GPS, a drone only has its motion sensors, and their errors grow fast: it drifts within
minutes. We compared 15 ways to navigate without GPS ([analysis](docs/ANALYSIS_SOLUTIONS.md))
and are building one on a ₹45.6k budget ([parts](docs/BOM.md)).

## Folders

```text
docs/                    guides and research
raspberry_pi/            code for the drone's Pi (+ laptop tools that need no ROS)
ros2_ws/src/
  gps_denied_bridge/     connects Pi and laptop (/odom, /scan, camera, /cmd_vel), health metrics
  x500_navigation/       Nav2 settings, launch files, RViz
  quadcopter_perception/ YOLO + SAM segmentation
  x500_px4_bridge/       simulator: Nav2 → PX4
  quadcopter_bringup/    simulator: launch + keyboard control
scripts/                 analysis charts and small tools
benchmarking/            generated charts
```

## More

[Design documents (Google Drive)](https://drive.google.com/drive/folders/1eQoDXTlVj3ZABpU9uxMFRTQ5T66ZaTJB?usp=sharing) ·
[Segmentation demo video](https://youtu.be/XxDwwBQxVR0) · Contributions welcome (one branch per
feature) · License: not chosen yet.
