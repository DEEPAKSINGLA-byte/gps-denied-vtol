# Raspberry Pi code

Runs on the drone's Pi (plain Python, no ROS). A few tools here also run on the laptop without ROS.

- To fly: [docs/QUICKSTART.md](../docs/QUICKSTART.md)
- To install: [docs/SETUP.md](../docs/SETUP.md#2-raspberry-pi)
- Ports and options: [docs/REFERENCE.md](../docs/REFERENCE.md)

> **Only one program can talk to the Pixhawk at a time.** While `start_streams.py` runs, don't
> start the bench tools or the vibration guard. Stop the streams first.

## Files

| Where | File | What it does |
| :--- | :--- | :--- |
| Pi | `start_streams.py` | **The one command:** starts and watches the three streams below |
| Pi | `telemetry_streamer.py` | Pixhawk data → laptop; also takes arm/mode commands |
| Pi | `lidar_udp_streamer.py` | Lidar → laptop |
| Pi | `camera_streamer.py` | Camera → laptop (newest frame only) |
| Pi | `mavlink_cmd_listener.py` | Nav2 speed commands → Pixhawk (only when armed in GUIDED) |
| Pi | `pixhawk_controller.py` | Shared Pixhawk library |
| Pi | `test_connection.py` | Checks the Pixhawk link |
| Pi | `drone_cli.py`, `arm_disarm.py`, `set_mode.py` | Arm, disarm, change mode |
| Pi | `sensor_telemetry.py` | Live sensor view and CSV logging |
| Pi | `vibration_motor_halt.py`, `launch_vibration_guard.sh` | Vibration guard |
| Laptop | `live_segmentation.py`, `segmentation_pipeline.py` | Camera + YOLO/SAM segmentation, no ROS |
| Laptop | `camera_receiver.py` | Camera viewer, no ROS |
| Laptop | `laptop_telemetry_receiver.py` | Telemetry viewer / logger, no ROS |
| Laptop | `launch_plotjuggler.sh`, `plotjuggler_bridge.py` | Telemetry graphs in PlotJuggler |
| Laptop | `fake_pi.py` | Fake drone data for testing |
| Setup | `install_udev_rules.sh`, `99-pixhawk.rules`, `98-camera.rules`, `requirements.txt` | Install |

Also here: `camera_ros2_receiver.py` / `lidar_ros2_receiver.py` (stand-alone copies of the
laptop's ROS receivers), `ros2_ydlidar/` (lidar driver if you run ROS on the Pi),
`nav2_ws/config/costmap_params.yaml`.

## Bench tools (props off!)

```bash
python3 test_connection.py              # ends with [PASSED] if the link is good

python3 drone_cli.py                    # interactive: mode alt_hold / telemetry / arm --force / disarm / exit
python3 drone_cli.py --status           # or one-shot: --telemetry, --mode ALT_HOLD, --arm [--force], --disarm, --kill

python3 arm_disarm.py --arm [--force]   # --force skips pre-arm checks (no GPS indoors)
python3 arm_disarm.py --disarm          # --kill = emergency motor stop, --status = armed?

python3 set_mode.py --list              # modes your Pixhawk supports
python3 set_mode.py alt_hold            # also: poshold, land, stabilize, guided, rtl

python3 sensor_telemetry.py             # live view; --snapshot [--json]; --log flight.csv
```

They find the Pixhawk automatically; use `--port` / `--baud` to choose.

## Laptop tools without ROS

Use these **instead of** the ROS stack (they use the same ports).

```bash
# Camera + segmentation (start the laptop first, then the Pi)
python3 live_segmentation.py                    # add --sam-imgsz 768 (faster) or --sam-model none (YOLO only)
python3 live_segmentation.py --verify           # one frame, report + saved images
python3 camera_receiver.py --stream             # plain viewer
#   on the Pi: python3 start_streams.py --no-lidar --no-telemetry

# Telemetry
python3 laptop_telemetry_receiver.py --verify   # check data arrives; without --verify: live view (--log file.csv)
./launch_plotjuggler.sh                         # then in PlotJuggler: Streaming → UDP Server → Start
#   on the Pi: python3 start_streams.py --no-lidar --no-camera
```

QGroundControl can watch the drone on UDP 14550, but can't control it in this setup.

## Vibration guard

If vibration gets dangerous (default above 35 m/s²), it cuts throttle to 40% and holds the drone
level. While it's active, the RC sticks are ignored so a panicking pilot can't make things worse.

```bash
python3 vibration_motor_halt.py                       # on the Pi
python3 vibration_motor_halt.py --threshold 30 --throttle-pct 40 --auto-alt-hold
./launch_vibration_guard.sh                           # from the laptop, runs it on the Pi
```

Keys: `t` test, `r` give the sticks back, `d` emergency disarm, `q` quit.

## Use it from Python

```python
from pixhawk_controller import PixhawkVehicle
drone = PixhawkVehicle()                  # finds /dev/pixhawk
print(drone.flight_mode, drone.get_telemetry()["attitude"])
drone.set_mode("ALT_HOLD"); drone.arm(force=True); drone.disarm()
drone.disconnect()
```

```python
from segmentation_pipeline import SegmentationEngine      # laptop
out = SegmentationEngine().process_matrix(frame)          # frame: OpenCV BGR image
out["detections"], out["class_mask"], out["annotated_frame"]
```

## Something wrong?

| Problem | Fix |
| :--- | :--- |
| `Permission denied: /dev/ttyACM0` | `sudo usermod -aG dialout $USER`, then log in again |
| `Timed out waiting for heartbeat` | Pixhawk powered? `ls -la /dev/pixhawk`, `python3 test_connection.py` |
| `Device or resource busy` | Something else has the Pixhawk open (often `start_streams.py`): `lsof /dev/pixhawk` |
| `Arming DENIED: <reason>` | Fix the reason, or see [SETUP.md → ArduPilot](../docs/SETUP.md#3-ardupilot-settings) |
| `Mode change failed` | That mode needs a position sensor; use `ALT_HOLD` |
| Camera gone after re-plugging | `sudo ./install_udev_rules.sh` and re-plug firmly |
| Sticks ignored, throttle stuck at 40% | The vibration guard triggered: press `r` or `d`, then check props and motors |
