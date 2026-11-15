<h1 align="center">Raspberry Pi Onboard Compute & Ground Telemetry Stack</h1>

<p align="center">
  <em>Onboard flight control, MAVLink telemetry, YDLIDAR + camera streaming, ROS 2 / Nav2 bridging, persistent udev rules, and RViz / PlotJuggler visualization for GPS-denied UAVs</em>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/Platform-Raspberry_Pi_4-C51A4A?logo=raspberrypi&logoColor=white" alt="Raspberry Pi 4"/>
  <img src="https://img.shields.io/badge/Autopilot-Pixhawk_/_ArduPilot-blue" alt="Pixhawk"/>
  <img src="https://img.shields.io/badge/MAVLink-2.0-orange" alt="MAVLink"/>
  <img src="https://img.shields.io/badge/ROS_2-Humble-FC4050?logo=ros&logoColor=white" alt="ROS 2"/>
  <img src="https://img.shields.io/badge/GPU-NVIDIA_RTX_4060-76B900?logo=nvidia&logoColor=white" alt="RTX 4060"/>
  <img src="https://img.shields.io/badge/Visualization-PlotJuggler_3-green" alt="PlotJuggler"/>
</p>

---

## Overview

This directory provides the complete software stack for using a **Raspberry Pi** as the onboard companion computer for a **Pixhawk flight controller**, coupled with a wireless telemetry link to a ground laptop over Wi-Fi (tested on **Redmi 10** network).

It addresses key real-world drone integration challenges:
* **Persistent Hardware Links:** Dedicated udev rules create `/dev/pixhawk` to prevent port hopping (`ttyACM0` $\leftrightarrow$ `ttyACM1`) during vibration and motion.
* **Flight Control & Safety:** Full programmatic and CLI control to arm/disarm (with pre-arm bypass `--force` for GPS-denied bench testing) and switch modes (`ALT_HOLD`, `POSHOLD`, `LAND`, `STABILIZE`, `GUIDED`).
* **Dual Telemetry Streaming:** Transmits high-rate parsed JSON telemetry (UDP 14551) alongside standard MAVLink packets (UDP 14550) for QGroundControl/Mission Planner.
* **Live Plotting in PlotJuggler:** Real-time multi-window timeseries plotting of Roll, Pitch, Yaw, Accelerometer, Gyro, Barometer, and Battery with a pre-configured XML layout.
* **LiDAR Streaming:** YDLIDAR scans are streamed from the Pi as JSON over UDP (14552) and republished on the laptop as `sensor_msgs/LaserScan` on `/scan` for Nav2 costmaps.
* **ROS 2 / Nav2 Bridge:** Telemetry becomes `/odom` + TF, camera becomes `/camera/image_raw`, and Nav2 `/cmd_vel` is sent back to the Pixhawk through a Pi-side velocity listener (UDP 14553).
* **Validation in RViz:** Latency, rate, TF-age and costmap-vs-lidar metrics are published under `/metrics/*` (see [Ground-side ROS 2 stack](#ground-side-ros-2-stack-rviz--nav2)).
* **Perception & Segmentation:** Live RGB camera matrix streaming (`/dev/video0`) with real-time semantic segmentation on the laptop's NVIDIA RTX 4060 GPU via CUDA.

---

## File Manifest

| File | Type | Role |
| :--- | :--- | :--- |
| [`99-pixhawk.rules`](99-pixhawk.rules) | Config | Persistent udev rules creating symlink `/dev/pixhawk` with `0666` permissions. |
| [`install_udev_rules.sh`](install_udev_rules.sh) | Script | Automated installer for Pixhawk udev rules. |
| [`pixhawk_controller.py`](pixhawk_controller.py) | Library | Core MAVLink engine with port auto-discovery, auto-reconnect, arm/disarm, and streaming. |
| [`arm_disarm.py`](arm_disarm.py) | Utility | Motor safety arm/disarm/kill utility with `--force` override. |
| [`set_mode.py`](set_mode.py) | Utility | Flight mode changer (`alt_hold`, `poshold`, `land`, `stabilize`, `guided`, `rtl`). |
| [`sensor_telemetry.py`](sensor_telemetry.py) | Utility | Real-time terminal dashboard, formatted snapshots, and CSV logging on the Pi. |
| [`drone_cli.py`](drone_cli.py) | CLI | Unified interactive console (`drone [ARMED | ALT_HOLD] >`) and batch CLI. |
| [`start_streams.py`](start_streams.py) | Launcher | **One command on the Pi** (`python3 start_streams.py <laptop_ip>`): runs + supervises telemetry, lidar and camera streamers, optional Nav2 velocity listener. |
| [`telemetry_streamer.py`](telemetry_streamer.py) | Service | Transmits live telemetry (10 Hz JSON + raw MAVLink) over Wi-Fi to laptop. |
| [`laptop_telemetry_receiver.py`](laptop_telemetry_receiver.py) | Utility | Ground receiver verifying packet integrity, latency, and sensor telemetry. |
| [`plotjuggler_bridge.py`](plotjuggler_bridge.py) | Service | Ground bridge forwarding telemetry to PlotJuggler UDP (port 9870) and ROS 2 topics. |
| [`plotjuggler_drone_layout.xml`](plotjuggler_drone_layout.xml) | Layout | Pre-configured 4-window PlotJuggler layout for Attitude, Accel, Gyro, and Altitude. |
| [`launch_plotjuggler.sh`](launch_plotjuggler.sh) | Launcher | 1-click launcher starting the bridge and opening PlotJuggler with drone layout. |
| [`camera_streamer.py`](camera_streamer.py) | Streamer | Streams RGB camera image matrices `(480, 640, 3)` over Wi-Fi (Redmi 10). |
| [`camera_receiver.py`](camera_receiver.py) | Receiver | Receives and verifies camera matrices on the laptop. |
| [`segmentation_pipeline.py`](segmentation_pipeline.py) | Pipeline | CUDA-accelerated YOLOv8 segmentation engine on NVIDIA RTX 4060 GPU. |
| [`live_segmentation.py`](live_segmentation.py) | Pipeline | Real-time segmentation receiver extracting object masks, bounding boxes, and overlays. |
| [`vibration_motor_halt.py`](vibration_motor_halt.py) | Safety Guard | Monitors vibration and clamps motors at 40% while locking out FlySky RC commands. |
| [`launch_vibration_guard.sh`](launch_vibration_guard.sh) | Launcher | 1-click launcher for the Vibration Guard (supports local or SSH remote execution). |
| [`98-camera.rules`](98-camera.rules) | Config | Persistent udev rule creating the `/dev/camera` symlink for the RGB camera. |
| [`test_connection.py`](test_connection.py) | Utility | Quick Pixhawk connectivity / heartbeat check. |
| [`lidar_udp_streamer.py`](lidar_udp_streamer.py) | Streamer | **Pi:** reads the YDLIDAR serial stream and sends one JSON packet per revolution (`timestamp`, 360 `ranges`, `intensities`) to UDP 14552. |
| [`lidar_ros2_receiver.py`](lidar_ros2_receiver.py) | Receiver | **Laptop:** UDP 14552 -> `sensor_msgs/LaserScan` on `/scan` (`laser_frame`). Standalone version of the `gps_denied_bridge` node. |
| [`camera_ros2_receiver.py`](camera_ros2_receiver.py) | Receiver | **Laptop:** TCP 5000 camera stream -> `sensor_msgs/Image` on `/camera/image_raw` for RViz. Standalone version of the `gps_denied_bridge` node. |
| [`mavlink_cmd_listener.py`](mavlink_cmd_listener.py) | Service | **Pi:** listens on UDP 14553 for Nav2 velocity setpoints (`{"action":"velocity","vx",...}`) and forwards them to the Pixhawk in GUIDED mode. |
| [`ros2_ydlidar/`](ros2_ydlidar) | ROS 2 pkg | On-board YDLIDAR G2 driver package (C++ / Python nodes, launch + params). |
| [`nav2_ws/config/costmap_params.yaml`](nav2_ws/config/costmap_params.yaml) | Config | Standalone costmap/DWB parameters (odom-frame rolling window, `/scan` source). |
| [`instruction.md`](instruction.md) | Docs | **Exhaustive step-by-step operating guide and reference manual.** |

---

## Quickstart

### 1. Install Persistent udev Rule (On Pi or Laptop)
```bash
sudo ./install_udev_rules.sh
ls -la /dev/pixhawk
```

### 2. Verify Pixhawk Connection
```bash
python3 test_connection.py
```

### 3. Stream Telemetry to Ground Laptop (Over Redmi 10 Wi-Fi)
On the Raspberry Pi (`10.184.3.36`):
```bash
python3 telemetry_streamer.py --target-ip 10.184.3.231 --rate 10
```

On the Laptop (`10.184.3.231`):
```bash
# Verify incoming packets
python3 laptop_telemetry_receiver.py --verify

# Launch real-time PlotJuggler visualization
./launch_plotjuggler.sh
```

### 4. Stream & Segment Live Camera Feed
On the Laptop:
```bash
python3 live_segmentation.py --stream
```
On the Raspberry Pi:
```bash
python3 camera_streamer.py --target-ip 10.184.3.231 --stream --fps 20
```

### 5. One Command: Stream Everything from the Pi
On the Raspberry Pi (e.g. after `ssh deimos@10.184.3.36`):
```bash
python3 start_streams.py 10.184.3.231          # <- laptop IP; omit it to use the SSH client IP
```
This starts and supervises (auto-restart with back-off) telemetry (UDP 14551/14550, commands 14552), the YDLIDAR stream (UDP 14552) and the camera (TCP 5000). Ctrl-C stops all of them.

Options: `--no-lidar --no-camera --no-telemetry`, `--lidar-port /dev/ttyUSB0`, `--camera-device /dev/camera`, `--fps 20 --quality 70`, `--rate 10`, `--pixhawk-port`.

Add `--nav-cmds` to also accept Nav2 velocity commands (UDP 14553). It runs inside the telemetry process and shares its Pixhawk connection, so there is no serial-port conflict. Velocity setpoints are forwarded **only while the vehicle is already armed and in GUIDED**; arming stays a deliberate operator action (`drone_cli.py` / `arm_disarm.py`). Test with props off.

Start the laptop side first (the camera receiver listens on TCP 5000; the Pi reconnects automatically):
```bash
ros2 launch x500_navigation navigation_real.launch.py pi_ip:=10.184.3.36
```
The individual scripts (`telemetry_streamer.py`, `lidar_udp_streamer.py`, `camera_streamer.py`, `mavlink_cmd_listener.py`) can still be run on their own.

### 6. Stopping Ongoing Processes
```bash
# On Laptop
pkill -f plotjuggler_bridge.py; pkill -f plotjuggler; pkill -f camera_receiver.py; pkill -f rviz2

# On Raspberry Pi
ssh deimos@10.184.3.36 "pkill -f start_streams.py; pkill -f telemetry_streamer.py; pkill -f camera_streamer.py; pkill -f lidar_udp_streamer.py"
```

---

## Ground-side ROS 2 stack (RViz + Nav2)

Data flow:

| Stream | Pi program | Transport | Laptop node (`gps_denied_bridge`) | ROS output |
| :--- | :--- | :--- | :--- | :--- |
| Telemetry | `telemetry_streamer.py` | UDP 14551 (JSON) | `mavlink_to_ros2_bridge` | `/odom`, TF `odom->base_link` |
| LiDAR | `lidar_udp_streamer.py` | UDP 14552 (JSON) | `lidar_udp_receiver` | `/scan` |
| Camera | `camera_streamer.py` | TCP 5000 | `camera_tcp_receiver` | `/camera/image_raw` |
| Commands | `mavlink_cmd_listener.py` | UDP 14553 (JSON) | `mavlink_cmd_bridge` | subscribes `/cmd_vel` |

Launch everything (Nav2, bridges, RViz, metrics) from `ros2_ws`:
```bash
cd ../ros2_ws && colcon build --packages-select gps_denied_bridge x500_navigation && source install/setup.bash
ros2 launch x500_navigation navigation_real.launch.py pi_ip:=10.184.3.36
# RViz + metrics only:  ros2 launch x500_navigation visualization.launch.py
# options: rviz:=false  metrics:=false  camera:=false
```

RViz uses `ros2_ws/src/x500_navigation/rviz/gps_denied_real.rviz` and shows the scan, camera, TF, local/global costmaps, plans, odometry trail, range rings, nearest obstacle and a status box.

**Validation metrics** (`std_msgs/Float32`, plottable in PlotJuggler / `rqt_plot`; summary on `/diagnostics`):

| Topic | Meaning |
| :--- | :--- |
| `/metrics/{lidar,telemetry,camera}/latency_ms` | Pi -> laptop one-way latency from the Pi's `timestamp`. Needs synced clocks (chrony/NTP on both) or the `clock_offset_s` parameter. |
| `/metrics/*/latency_excess_ms` | Latency minus its 10 s minimum: Wi-Fi jitter, independent of clock offset. |
| `/metrics/{lidar,telemetry,camera,odom}/rate_hz` | Stream rates (decay to 0 when a stream stalls). |
| `/metrics/camera/{decode_ms,bandwidth_mbps}` | JPEG decode cost and link load. |
| `/metrics/lidar/{nearest_obstacle_m,valid_ratio,tf_fail_ratio}` | Scan quality; `tf_fail_ratio` = scans Nav2 could not transform. |
| `/metrics/tf/odom_base_age_ms` | Staleness of `odom -> base_link`. |
| `/metrics/costmap/local/scan_agreement` | Fraction of lidar hits on lethal costmap cells (~1 = costmap matches the lidar). |
| `/metrics/odom/{speed_mps,distance_from_start_m,path_length_m}` | Odometry / drift tracking. |

> The camera and lidar mount offsets in `navigation_real.launch.py` (`base_link -> laser_frame / camera_frame`) are placeholders; set them to the real mounting.

---

## Detailed Documentation

For exhaustive configuration parameters, MAVLink stream rates, GPS-denied bench testing, wiring pinouts, and troubleshooting, refer to **[`instruction.md`](instruction.md)**.
