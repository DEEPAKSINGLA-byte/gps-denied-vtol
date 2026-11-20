# Reference

Look-up tables for the real drone. To fly, see [QUICKSTART.md](QUICKSTART.md).

## Ports

| Data | Direction | Port | Pi program | Laptop node | ROS topic |
| :--- | :--- | :--- | :--- | :--- | :--- |
| Telemetry (JSON) | Pi → laptop | UDP 14551 | `telemetry_streamer.py` | `mavlink_to_ros2_bridge` | `/odom`, TF |
| Raw MAVLink (watch only, e.g. QGC) | Pi → laptop | UDP 14550 | `telemetry_streamer.py` | | |
| Lidar | Pi → laptop | UDP 14552 | `lidar_udp_streamer.py` | `lidar_udp_receiver` | `/scan` |
| Camera | Pi → laptop | TCP 5000 | `camera_streamer.py` | `camera_tcp_receiver` | `/camera/image_raw` |
| Speed commands | laptop → Pi | UDP 14553 | `mavlink_cmd_listener.py` | `mavlink_cmd_bridge` | `/cmd_vel` |
| Mode / arm commands | laptop → Pi | UDP 14552 | `telemetry_streamer.py` | `mavlink_cmd_bridge` | |

Only one program can use a port on the laptop. The standalone tools (`live_segmentation.py`,
`camera_receiver.py`, `laptop_telemetry_receiver.py`, PlotJuggler bridge) use the same ports
as the ROS stack, so run them **instead of** it.

## `start_streams.py` options

| Option | Default | What it does |
| :--- | :--- | :--- |
| *(laptop IP)* | your SSH client | Where to send the data |
| `--nav-cmds` | off | Let Nav2 steer the drone (only while armed in GUIDED) |
| `--no-telemetry`, `--no-lidar`, `--no-camera` | | Leave a stream out |
| `--pixhawk-port` | auto (`/dev/pixhawk`) | Pixhawk device |
| `--lidar-port`, `--lidar-baud` | `/dev/ttyUSB0`, `115200` | Lidar serial port |
| `--camera-device` | `/dev/camera` | Camera |
| `--camera-size` | `640x480` | Smaller (e.g. `424x240`) = less Wi-Fi load |
| `--fps`, `--quality` | `20`, `70` | Camera frame rate, JPEG quality |
| `--camera-passthrough` | off | Send the camera's own JPEGs (less Pi CPU) |
| `--rate` | `30` | Max telemetry rate (Hz) |
| `--keep-powersave` | off | Don't turn off Wi-Fi power saving |
| `--no-time-sync` | off | Don't sync the Pi clock to the laptop |

Each stream can also run alone; run any script with `--help`.

## Launch files

| Command | Starts |
| :--- | :--- |
| `ros2 launch x500_navigation navigation_real.launch.py` | Everything for the real drone |
| `ros2 launch x500_navigation visualization.launch.py` | Only RViz and the health metrics |
| `ros2 launch quadcopter_perception segmentation.launch.py` | Only segmentation |

`navigation_real.launch.py` options: `pi_ip`, `camera`, `segmentation`, `seg_model`,
`seg_sam_imgsz` (768), `seg_sam_model`, `rviz`,
`metrics`, `use_sim_time`, `params_file` (default `config/nav2_real.yaml`). Fixed inside it:
max speed 0.5 m/s, command timeout 0.5 s, never auto-arms.

## ROS topics

| Topic | What |
| :--- | :--- |
| `/odom`, TF `odom → base_link` | Drone position: Pixhawk's estimate if it has one, else from optical flow |
| `/scan` | Lidar, 360 readings, one per degree |
| `/camera/image_raw` | Camera image |
| `/segmentation/detections` | Each object: class name, score, box |
| `/segmentation/class_mask` | Image where each pixel = class id + 1 (0 = nothing) |
| `/segmentation/instance_mask` | Image where each pixel = object number + 1 |
| `/segmentation/image_annotated` | Picture with masks drawn (only made if someone watches it) |
| `/segmentation/class_names` | Class id → name |
| `/cmd_vel` | Nav2's speed command to the drone |

## Health metrics

Numbers under `/metrics/…` (plot with PlotJuggler or `rqt_plot`). A summary (OK / WARN / FAIL)
is on `/diagnostics` and shown in RViz.

| Topic | Meaning |
| :--- | :--- |
| `/metrics/{lidar,telemetry,camera}/latency_ms` | Pi → laptop delay (exact once `start_streams.py` has synced the clocks) |
| `/metrics/*/latency_excess_ms` | Extra delay from Wi-Fi hiccups (works without `chrony`) |
| `/metrics/*/rate_hz`, `/metrics/lidar/jitter_ms` | How often data arrives, how evenly |
| `/metrics/camera/{decode_ms,bandwidth_mbps,dropped_frames}` | Camera cost and load |
| `/metrics/lidar/{nearest_obstacle_m,valid_ratio,tf_fail_ratio}` | Lidar quality |
| `/metrics/tf/odom_base_age_ms` | How old the position is |
| `/metrics/costmap/local/scan_agreement` | ≈ 1 when the map matches the lidar |
| `/metrics/costmap/{local,global}/{rate_hz,lethal_cells}` | Map updates / obstacles |
| `/metrics/odom/{speed_mps,distance_from_start_m,path_length_m,stationary}` | Movement and drift |
| `/metrics/flow/{quality,ground_distance_m}` | Optical-flow sensor |
| `/metrics/nav/{cmd_vel_rate_hz,cmd_speed_mps,plan_length_m}` | What Nav2 is doing |
| `/metrics/segmentation/{inference_ms,yolo_ms,sam_ms,sam_refined,…}` | Segmentation cost and output |

## Segmentation (YOLO + SAM)

YOLO finds and names objects. SAM 2.1 then redraws each object's outline from YOLO's box, which
is much sharper. If SAM's outline looks wrong, the object keeps YOLO's outline. In the picture,
`[S]` means SAM outline, `[Y]` means YOLO outline. The code is in
[`hybrid_segmenter.py`](../ros2_ws/src/quadcopter_perception/quadcopter_perception/hybrid_segmenter.py).

| ROS option | `live_segmentation.py` | Effect | Speed (640×480, RTX 4050) |
| :--- | :--- | :--- | :--- |
| `sam_imgsz:=1024` (default) | `--sam-imgsz 1024` | Sharpest | ~57 ms per frame (~14 FPS) |
| `sam_imgsz:=768` | `--sam-imgsz 768` | Almost the same | ~35 ms (20 FPS) |
| `sam_every_n:=2` | `--sam-every-n 2` | SAM on every 2nd frame | |
| `sam_model:=none` | `--sam-model none` | YOLO only | ~5–10 ms |
| `classes:="['person']"` | | Only these classes | |

In the full stack, use `seg_sam_imgsz` (default **768**, lowest latency) and `seg_sam_model`
(`none` = YOLO only) on `navigation_real.launch.py`. Webcam demo without a drone:
[`scripts/experimentation.py`](../ros2_ws/src/quadcopter_perception/scripts/README.md).

## Link latency

Already done in code: telemetry is sent the moment new Pixhawk data arrives (30 Hz), each
telemetry packet fits in one Wi-Fi packet, the laptop publishes on arrival, commands are
forwarded instantly, telemetry and commands get Wi-Fi priority, and the camera drops old frames
instead of queueing them.

| Path | Before | After |
| :--- | :--- | :--- |
| Pixhawk → telemetry sent | 28 ms (up to 100 ms) | 5 ms |
| Telemetry → `/odom` | 20 ms at 10 Hz | 2 ms at 30 Hz |
| `/cmd_vel` → Pixhawk | 54 ms (up to 100 ms) | 1.4 ms |

*(Measured on one machine, so Wi-Fi time is not included.)*

Up to you: turn off Wi-Fi power saving on the laptop ([SETUP.md](SETUP.md#1-laptop)),
connect Pi and laptop directly instead of via a phone hotspot, and use smaller camera frames
if Wi-Fi is busy.
