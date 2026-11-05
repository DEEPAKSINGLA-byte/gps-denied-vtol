# Raspberry Pi Onboard Drone Compute & Pixhawk Control Stack

This folder contains the complete onboard flight control and telemetry software stack running on the **Raspberry Pi companion computer**, interfacing directly with the **Pixhawk flight controller** over MAVLink.

The suite provides programmatic and interactive control to:
- **Arm / Disarm** motors with safety feedback and pre-arm check bypass options.
- **Switch flight modes** dynamically (`ALT_HOLD`, `POSHOLD` / `LOITER`, `LAND`, `STABILIZE`, `GUIDED`, `RTL`, etc.).
- **Acquire and stream live sensor telemetry** (Attitude, IMU, Barometer, Battery, GPS, Local Position NED, Rangefinder/Lidar, Optical Flow, and VFR HUD).
- **Log flight telemetry** into structured CSV/JSON formats for offline analysis.
- **Perform hardware link diagnostics** and packet throughput verification.

---

## Hardware Architecture & Connectivity

```
 ┌──────────────────────────────────────────────────────────────┐
 │               Raspberry Pi (Onboard Companion)               │
 │                                                              │
 │   • pixhawk_controller.py (Core MAVLink Controller Engine)   │
 │   • drone_cli.py          (Interactive Console & CLI)        │
 │   • sensor_telemetry.py   (Real-Time Dashboard & CSV Logger) │
 │   • set_mode.py           (Flight Mode Switcher)             │
 │   • arm_disarm.py         (Motor Arm / Disarm / Kill Switch) │
 └──────────────────────────────┬───────────────────────────────┘
                                │ MAVLink Protocol
                                │ USB (/dev/ttyACM0) or UART (/dev/serial0)
                                │ Baud: 115200 (Default)
 ┌──────────────────────────────┴───────────────────────────────┐
 │                  Pixhawk Flight Controller                   │
 │                                                              │
 │   • IMU (3x Accel, 3x Gyro, 3x Magnetometer)                 │
 │   • Barometer (MS5611 / BMP280 Air Pressure & Temp)          │
 │   • Battery Monitor (Voltage & Current Sensing)              │
 │   • Distance Sensor / Lidar (Rangefinder for GPS-denied alt) │
 │   • Optical Flow Sensor (Indoor XY Position Hold)            │
 │   • Motor ESC Outputs & Actuation                            │
 └──────────────────────────────────────────────────────────────┘
```

### Connection Interfaces

| Connection Type | Physical Port on RPi | Pixhawk Port | Linux Device Node | Default Baud |
| :--- | :--- | :--- | :--- | :--- |
| **USB Cable (Default)** | Standard USB 2.0 / 3.0 | Micro-USB Port | `/dev/ttyACM0` | `115200` |
| **UART Telemetry Cable** | GPIO 14 (TX) & 15 (RX) | TELEM1 or TELEM2 | `/dev/serial0` | `57600` or `921600` |

> [!NOTE]
> All scripts in this directory feature **automatic port detection**. They probe `/dev/ttyACM*`, `/dev/ttyUSB*`, and `/dev/serial0*` at common baud rates (`115200`, `57600`, `921600`) and lock onto the active MAVLink heartbeat automatically.

---

## Installation & Setup on Raspberry Pi

### 1. User Permissions
Ensure the current Linux user belongs to the `dialout` group to access serial ports without `sudo`:
```bash
sudo usermod -aG dialout $USER
```
*(Log out and back in, or run `newgrp dialout` for changes to take effect).*

### 2. Install Python Dependencies
Install the required packages (`pymavlink` and `pyserial`):
```bash
pip install -r requirements.txt --break-system-packages
```
Or create an isolated virtual environment:
```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

---

## Verification & Link Diagnostics

Before executing flight commands, run the automated diagnostic script to verify the physical link, MAVLink stream health, and sensor responses:

```bash
python3 test_connection.py
```

**Expected output:**
```text
======================================================================
 PIXHAWK COMPANION HARDWARE & LINK DIAGNOSTICS
======================================================================
Detected serial devices in /dev: ['/dev/ttyACM0']
[*] Probing /dev/ttyACM0 at 115200 baud...
[+] Successfully connected to Pixhawk on /dev/ttyACM0 at 115200 baud!

--- Flight Controller Metadata ---
  Autopilot:       ArduPilot (ArduCopter/Plane/VTOL)
  Vehicle Frame:   Quadrotor
  System ID:       1
  Component ID:    1
  Current Mode:    STABILIZE
  ...
  Throughput: 142 packets in 3.0s (47.3 packets/sec)
  [ OK ] Attitude & Orientation (IMU EKF)         (ATTITUDE)
  [ OK ] Raw Accelerometer / Gyro / Magnetometer  (RAW_IMU)
  [ OK ] Barometer (Air Pressure & Altitude)      (SCALED_PRESSURE)
  [ OK ] Battery Voltage & Power Monitoring       (SYS_STATUS)
======================================================================
 [PASSED] Pixhawk communication link is verified and fully operational!
======================================================================
```

---

## Script Usage Guide

### 1. Arming & Disarming (`arm_disarm.py`)

Controls the motor safety arm state. Provides pre-arm failure diagnostic feedback and emergency motor cutoff.

```bash
# Check current motor armed state
python3 arm_disarm.py --status

# Arm motors (runs pre-arm safety checks)
python3 arm_disarm.py --arm

# Force-arm motors (bypasses pre-arm checks like GPS fix for indoor testing)
python3 arm_disarm.py --arm --force

# Disarm motors
python3 arm_disarm.py --disarm

# Emergency motor cutoff (forces immediate disarm)
python3 arm_disarm.py --kill
```

> [!WARNING]
> **Safety Notice:** Always remove propellers during bench and indoor development before issuing `--arm` or `--arm --force` commands.

---

### 2. Flight Mode Switching (`set_mode.py`)

Changes the autopilot mode to Altitude Hold, Position Hold, Land, Stabilize, Guided, etc. Supports aliases (`alt_mode`, `position_mode`, `land_mode`).

```bash
# List all available flight modes supported by your firmware
python3 set_mode.py --list

# Show currently active mode
python3 set_mode.py --current

# Switch to Altitude Hold Mode (Barometer/Lidar altitude lock, manual horizontal)
python3 set_mode.py alt_hold
# (Aliases: alt, alt_mode, althold)

# Switch to Position Hold Mode (Maintains 3D position lock via GPS / Optical Flow)
python3 set_mode.py poshold
# (Aliases: pos, position, position_mode, loiter)

# Switch to Land Mode (Automated smooth vertical descent and disarm upon touchdown)
python3 set_mode.py land

# Switch to Stabilize / Manual Mode (Self-leveling attitude control)
python3 set_mode.py stabilize

# Switch to Guided Mode (Enables autonomous waypoint/velocity companion control)
python3 set_mode.py guided

# Switch to Return to Launch (Autonomous return to home takeoff coordinates)
python3 set_mode.py rtl
```

---

### 3. Sensor Telemetry Monitoring & Logging (`sensor_telemetry.py`)

Extracts telemetry streams and presents them in real-time dashboards or logs to disk.

```bash
# 1. Launch real-time interactive terminal dashboard (refreshes at 4 Hz)
python3 sensor_telemetry.py

# 2. Print a single formatted telemetry snapshot to stdout and exit
python3 sensor_telemetry.py --snapshot

# 3. Print a single JSON snapshot (ideal for piping into other tools)
python3 sensor_telemetry.py --snapshot --json

# 4. Stream and log live telemetry to a CSV file for flight analysis
python3 sensor_telemetry.py --log flight_data.csv --rate 10
```

#### Monitored Telemetry Channels:
1. **Attitude:** Roll, Pitch, Yaw (degrees), and 3-axis angular rotation rates (°/s).
2. **IMU Sensors:** 3-axis Accelerometer (mg), 3-axis Gyroscope (°/s), 3-axis Magnetometer (mG).
3. **Barometer:** Static air pressure (hPa) and ambient temperature (°C).
4. **Power System:** Battery voltage (V), current draw (A), and remaining capacity (%).
5. **GPS:** Fix status (3D Fix, RTK, No Fix), satellites visible, HDOP/VDOP.
6. **Local Position NED:** Local coordinate estimates (X, Y, Z in meters) and velocities ($V_x, V_y, V_z$ in m/s) — **crucial for GPS-denied state estimation**.
7. **Rangefinder / Lidar:** Ground clearance distance (meters) for obstacle detection and precision altitude holding.
8. **Optical Flow:** Ground velocity vector and surface texture quality score.
9. **VFR HUD:** Airspeed, groundspeed, climb rate, magnetic heading, throttle percentage.

---

### 4. Wireless Telemetry Streaming over Wi-Fi (Redmi 10 Network)

You can wirelessly stream live Pixhawk flight telemetry and raw MAVLink packets directly from the Raspberry Pi to your ground laptop over any Wi-Fi network (e.g. **Redmi 10** hotspot).

```
 ┌────────────────────────────────────┐             ┌────────────────────────────────────┐
 │  Raspberry Pi (10.184.3.36)        │             │  Ground Laptop (10.184.3.231)      │
 │  (Connected to Pixhawk /dev/ttyACM)│             │                                    │
 │                                    │  Redmi 10   │  • laptop_telemetry_receiver.py    │
 │  • telemetry_streamer.py           │─── Wi-Fi ──▶│  • QGroundControl / MissionPlanner │
 │    - MAVLink stream (port 14550)   │    (UDP)    │    (Listens on UDP 14550)          │
 │    - JSON stream    (port 14551)   │             │  • Custom Python / ROS 2 nodes     │
 └────────────────────────────────────┘             └────────────────────────────────────┘
```

#### Step 1: On the Raspberry Pi
Run the telemetry streamer specifying your laptop's IP address:
```bash
# Stream both MAVLink (port 14550) and JSON telemetry (port 14551) at 10 Hz
python3 telemetry_streamer.py --target-ip 10.184.3.231 --rate 10

# Or run in the background as a daemon
nohup python3 telemetry_streamer.py --target-ip 10.184.3.231 --rate 10 > ~/streamer.log 2>&1 &
```

#### Step 2: On Your Laptop
Verify data reception and inspect live flight telemetry:
```bash
# 1. Run automated telemetry integrity verification
python3 laptop_telemetry_receiver.py --verify

# 2. Open live interactive dashboard on your laptop
python3 laptop_telemetry_receiver.py

# 3. Stream and log incoming flight telemetry directly to CSV on your laptop
python3 laptop_telemetry_receiver.py --log flight_log.csv
```

#### Step 3: Connecting Ground Control Stations (QGroundControl / Mission Planner)
Because the Raspberry Pi forwards the raw MAVLink stream over UDP port `14550`, standard Ground Control Stations (QGC / Mission Planner) will automatically detect the vehicle and connect wirelessly as soon as they open!

---

### 5. RGB Camera Image Matrix Streaming & Verification

The Raspberry Pi captures live frames from the onboard RGB camera (`/dev/video0`) and transmits the underlying **NumPy image matrix** $(H, W, 3)$ across the **Redmi 10** network to the laptop for computer vision, perception, or inspection.

```
 ┌──────────────────────────────────────┐             ┌──────────────────────────────────────┐
 │  Raspberry Pi (10.184.3.36)          │             │  Ground Laptop (10.184.3.231)        │
 │  • /dev/video0 (USB RGB Camera)      │  Redmi 10   │                                      │
 │  • OpenCV + NumPy                    │─── Wi-Fi ──▶│  • camera_receiver.py                │
 │  • camera_streamer.py                │    (TCP)    │    - Reconstructs (H, W, 3) matrix   │
 │    - Sends raw or encoded matrix     │  Port 5000  │    - Validates pixel integrity       │
 └──────────────────────────────────────┘             └──────────────────────────────────────┘
```

#### Step 1: On the Laptop (Start the Receiver)
Start the receiver to listen on port `5000` and automatically verify the matrix:
```bash
# Verify image matrix structure, pixel values, and save snapshot to disk
python3 camera_receiver.py --verify

# Or receive continuous video stream at 20-30 FPS
python3 camera_receiver.py --stream
```

#### Step 2: On the Raspberry Pi (Transmit the Matrix)
Send the image matrix to the laptop:
```bash
# Send a single verified snapshot matrix
python3 camera_streamer.py --target-ip 10.184.3.231 --snapshot

# Stream raw uncompressed NumPy matrix
python3 camera_streamer.py --target-ip 10.184.3.231 --mode raw --snapshot

# Continuous live camera stream (20 FPS)
python3 camera_streamer.py --target-ip 10.184.3.231 --stream --fps 20
```

---

### 6. Real-Time Segmentation on Camera Matrix Stream

You can run semantic / instance segmentation models directly on the live image matrices streamed from the drone across the **Redmi 10** network, accelerated by the ground laptop's **NVIDIA RTX 4060 GPU** via CUDA.

```
 ┌──────────────────────────────────────┐             ┌──────────────────────────────────────────────┐
 │  Raspberry Pi 4 (10.184.3.36)        │             │  Ground Laptop (10.184.3.231)                │
 │  • /dev/video0 (USB RGB Camera)      │  Redmi 10   │  • NVIDIA GeForce RTX 4060 Laptop GPU        │
 │  • camera_streamer.py                │─── Wi-Fi ──▶│  • live_segmentation.py                      │
 │    - Sends (480, 640, 3) matrix      │    (TCP)    │    - Ingests image matrix                    │
 │                                      │  Port 5000  │    - YOLOv8-seg / PyTorch inference (CUDA)   │
 │                                      │             │    - Extracts object masks, bboxes, classes  │
 └──────────────────────────────────────┘             └──────────────────────────────────────────────┘
```

#### Step 1: Start the Segmentation Receiver on Your Laptop
Run the live segmentation processor:
```bash
cd ~/gps-denied-vtol/raspberry_pi

# 1. Automated verification on a single incoming frame from Pi
python3 live_segmentation.py --verify

# 2. Continuous real-time segmentation stream (prints GPU inference ms & FPS)
python3 live_segmentation.py --stream

# 3. Continuous segmentation with live OpenCV visual display window
python3 live_segmentation.py --stream --gui

# 4. Save segmented frames to an output folder
python3 live_segmentation.py --stream --save-dir ./segmentation_output
```

#### Step 2: Stream Live Frames from the Raspberry Pi
```bash
# Send single frame snapshot
ssh deimos@10.184.3.36 "cd ~/raspberry_pi && python3 camera_streamer.py --target-ip 10.184.3.231 --snapshot"

# Or stream live continuous frames (20 FPS)
ssh deimos@10.184.3.36 "cd ~/raspberry_pi && python3 camera_streamer.py --target-ip 10.184.3.231 --stream --fps 20"
```

#### Step 3: Programmatic API for Downstream Autonomy & Navigation
You can import `SegmentationEngine` in your custom scripts or ROS 2 nodes:
```python
import cv2
from segmentation_pipeline import SegmentationEngine

# Loads on CUDA (RTX 4060) automatically
engine = SegmentationEngine(model_path="yolov8n-seg.pt")

# Pass any (H, W, 3) uint8 image matrix directly
frame = cv2.imread("received_image_matrix.jpg")
results = engine.process_matrix(frame)

print(f"Inference latency: {results['inference_time_ms']} ms")
print(f"Detected {results['num_objects']} objects with {results['num_masks']} segmentation masks")

# Access binary masks, bounding boxes, and annotated visualization
annotated_frame = results["annotated_frame"]
isolated_masks = results["isolated_mask_frame"]
for mask_info in results["masks"]:
    print(f"Found {mask_info['class_name']} with {mask_info['pixel_area']} pixels")
```

---

### 7. Unified Drone CLI (`drone_cli.py`)

A single tool that offers both an **interactive control shell** and convenient single-command flags.

#### Interactive Shell Mode:
Run without arguments:
```bash
python3 drone_cli.py
```
You will enter the live interactive prompt with real-time status indication:
```text
drone [DISARMED | STABILIZE] > mode alt_hold
[SUCCESS] Flight mode successfully set to ALT_HOLD

drone [DISARMED | ALT_HOLD] > telemetry
--- Sensor Snapshot ---
  Attitude: Roll=-1.2°, Pitch=-0.4°, Yaw=154.2°
  Baro:     Press=899.2 hPa, Temp=28.9°C
  Battery:  Volt=0.0V, Rem=None%
  GPS:      No GPS Connected (0 sats)
  HUD:      Alt=0.3m, Hdg=207°, Thr=0%

drone [DISARMED | ALT_HOLD] > arm --force
[SUCCESS] Vehicle successfully ARMED

drone [ARMED | ALT_HOLD] > disarm
[SUCCESS] Vehicle successfully DISARMED

drone [DISARMED | ALT_HOLD] > exit
```

#### Direct Batch Commands:
```bash
python3 drone_cli.py --status
python3 drone_cli.py --arm --force
python3 drone_cli.py --mode ALT_HOLD
python3 drone_cli.py --mode LAND
python3 drone_cli.py --disarm
python3 drone_cli.py --telemetry
```

---

## Programmatic Python API (`pixhawk_controller.py`)

You can import `PixhawkVehicle` directly in your own autonomous scripts or ROS 2 navigation nodes:

```python
from pixhawk_controller import PixhawkVehicle
import time

# Auto-connects to Pixhawk on /dev/ttyACM0 or /dev/serial0
drone = PixhawkVehicle()

try:
    print(f"Vehicle: {drone.vehicle_type} running {drone.autopilot_type}")
    print(f"Current mode: {drone.flight_mode}")

    # Set mode to ALT_HOLD
    success, msg = drone.set_mode("ALT_HOLD")
    print(f"Set ALT_HOLD: {success} ({msg})")

    # Fetch live telemetry dictionary
    telem = drone.get_telemetry()
    roll = telem["attitude"]["roll_deg"]
    pitch = telem["attitude"]["pitch_deg"]
    yaw = telem["attitude"]["yaw_deg"]
    print(f"Current Attitude: Roll={roll}°, Pitch={pitch}°, Yaw={yaw}°")

    # Arm vehicle (force=True bypasses GPS check for indoor tests)
    ok, msg = drone.arm(force=True)
    print(f"Arm result: {ok} ({msg})")

    time.sleep(2.0)

    # Disarm
    drone.disarm()

finally:
    drone.disconnect()
```

---

## GPS-Denied Navigation & Bench Testing Guide

In GPS-denied environments (indoors, tunnels, underground, or under electronic warfare jamming), Pixhawk's Extended Kalman Filter (EKF) will not receive GNSS satellite fixes.

### 1. Why Normal Arming Is Blocked Indoors
ArduPilot runs **pre-arm checks** to ensure flight safety. If GPS is expected but no satellite lock exists, arming will fail with:
`PreArm: Need 3D Fix` or `PreArm: High GPS HDOP`.

### 2. How to Overcome GPS Pre-Arm Blocks

#### Method A: Software Override Flag (`--force`)
Use the `--force` flag in the provided scripts:
```bash
python3 arm_disarm.py --arm --force
# or in interactive shell:
# drone> arm --force
```
This passes parameter `21196` to MAVLink's `MAV_CMD_COMPONENT_ARM_DISARM`, instructing the autopilot to bypass pre-arm checks.

#### Method B: Firmware Parameter Configuration
If performing extensive indoor testing, connect to the flight controller via Mission Planner / QGroundControl / MAVProxy and modify:
- **`ARMING_CHECK`**:
  - Set to `0` to disable all pre-arm checks (strictly for bench testing without propellers).
  - Or set to `1048558` (disables GPS and compass checks while keeping voltage and hardware checks active).
- **`AHRS_GPS_USE`**: Set to `0` (disables GPS from being used by the attitude and heading reference system).
- **`EK3_SRC1_POSXY`**:
  - Set to `0` (None) for pure Altitude Hold (`ALT_HOLD`) flight.
  - Set to `4` (OpticalFlow) when an optical flow sensor (e.g. CX-OF, PMW3901, or camera-based V-SLAM) is connected.
- **`EK3_SRC1_POSZ`**: Set to `1` (Barometer) or `2` (RangeFinder / Lidar).

### 3. Recommended Flight Modes for GPS-Denied Flight
- **`ALT_HOLD` (Altitude Hold):** Uses the barometer and ultrasonic/lidar rangefinder for vertical height holding. Pitch and roll remain manual/stabilized. Highly recommended for initial indoor testing.
- **`POSHOLD` (Position Hold):** Requires either GPS or a valid indoor optical flow / visual-inertial odometry source. If no position sensor is configured, Pixhawk will reject switching to `POSHOLD`.
- **`LAND`:** Automatic controlled descent using barometer/rangefinder, shuts down motors upon touchdown.

---

## Troubleshooting Checklist

| Issue | Root Cause | Solution |
| :--- | :--- | :--- |
| `Permission denied: '/dev/ttyACM0'` | Current user lacks serial port access permissions | Run `sudo usermod -aG dialout $USER`, log out and re-login |
| `Timed out waiting for heartbeat` | Pixhawk is not powered, or incorrect port/baud rate | Check USB cable connection; ensure Pixhawk LED is pulsing; run `python3 test_connection.py` |
| `Device or resource busy: '/dev/ttyACM0'` | Another process (e.g. MAVProxy, QGroundControl, ROS 2 MAVROS node) has opened the port | Check open serial locks: `lsof /dev/ttyACM0` and terminate conflicting processes |
| `Arming DENIED by Pixhawk` | Pre-arm safety check failed (GPS lock, calibration, or safety switch) | Use `--arm --force` to bypass check, or press the physical GPS safety switch button (if equipped) |
| `Mode change failed` | Mode not supported or prerequisites unmet (e.g. POSHOLD without optical flow/GPS) | Run `python3 set_mode.py --list` to check firmware-supported modes; switch to `ALT_HOLD` or `STABILIZE` instead |
| `No attitude/telemetry packets received` | Pixhawk stream rate is disabled or set to 0 | `PixhawkVehicle` automatically sends `request_data_stream_send`. Verify baud rate is at least 115200 |
