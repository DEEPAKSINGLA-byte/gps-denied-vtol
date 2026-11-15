<h1 align="center">Raspberry Pi Onboard Compute & Ground Telemetry Stack</h1>

<p align="center">
  <em>Onboard flight control, MAVLink telemetry streaming, persistent udev rules, camera perception, and real-time PlotJuggler visualization for GPS-denied UAVs</em>
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
| [`telemetry_streamer.py`](telemetry_streamer.py) | Service | Transmits live telemetry (10 Hz JSON + raw MAVLink) over Wi-Fi to laptop. |
| [`laptop_telemetry_receiver.py`](laptop_telemetry_receiver.py) | Utility | Ground receiver verifying packet integrity, latency, and sensor telemetry. |
| [`plotjuggler_bridge.py`](plotjuggler_bridge.py) | Service | Ground bridge forwarding telemetry to PlotJuggler UDP (port 9870) and ROS 2 topics. |
| [`plotjuggler_drone_layout.xml`](plotjuggler_drone_layout.xml) | Layout | Pre-configured 4-window PlotJuggler layout for Attitude, Accel, Gyro, and Altitude. |
| [`launch_plotjuggler.sh`](launch_plotjuggler.sh) | Launcher | 1-click launcher starting the bridge and opening PlotJuggler with drone layout. |
| [`camera_streamer.py`](camera_streamer.py) | Streamer | Streams RGB camera image matrices `(480, 640, 3)` over Wi-Fi (Redmi 10). |
| [`camera_receiver.py`](camera_receiver.py) | Receiver | Receives and verifies camera matrices on the laptop. |
| [`segmentation_pipeline.py`](segmentation_pipeline.py) | Pipeline | CUDA-accelerated YOLOv8 segmentation engine on NVIDIA RTX 4060 GPU. |
| [`live_segmentation.py`](live_segmentation.py) | Pipeline | Real-time segmentation receiver extracting object masks, bounding boxes, and overlays. |
| [`ground_station_gui.py`](ground_station_gui.py) | GCS GUI | Full-featured Ground Station GUI with PFD horizon, vibration oscilloscope, live video, and direct drone controls. |
| [`launch_ground_station.sh`](launch_ground_station.sh) | Launcher | 1-click launcher starting the unified Ground Control Station. |
| [`vibration_motor_halt.py`](vibration_motor_halt.py) | Safety Guard | Monitors vibration and clamps motors at 40% while locking out FlySky RC commands. |
| [`launch_vibration_guard.sh`](launch_vibration_guard.sh) | Launcher | 1-click launcher for the Vibration Guard (supports local or SSH remote execution). |
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

### 5. Stopping Ongoing Processes
```bash
# On Laptop
pkill -f plotjuggler_bridge.py; pkill -f plotjuggler; pkill -f camera_receiver.py

# On Raspberry Pi
ssh deimos@10.184.3.36 "pkill -f telemetry_streamer.py; pkill -f camera_streamer.py"
```

---

## Detailed Documentation

For exhaustive configuration parameters, MAVLink stream rates, GPS-denied bench testing, wiring pinouts, and troubleshooting, refer to **[`instruction.md`](instruction.md)**.
