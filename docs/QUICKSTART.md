# Quick Start: flying the real drone

The Pi on the drone only **sends** data (telemetry, lidar, camera). The **laptop does the
thinking** (position, segmentation, path planning) and sends speed commands back.

```
 Drone                                              Laptop (ROS 2 Humble, GPU)
 ┌───────────────────────────┐   Wi-Fi             ┌────────────────────────────────────┐
 │ Pixhawk (ArduPilot) ─USB─┐│ ──telemetry 14551─▶ │ /odom + TF                         │
 │ YDLIDAR ────────────USB──┤│ ──lidar     14552─▶ │ /scan            ┐                 │
 │ USB camera ──────────────┤│ ──camera  TCP 5000▶ │ /camera/image_raw┼▶ Nav2, YOLO+SAM │
 │        Raspberry Pi ◀────┘│ ◀─cmd_vel   14553── │ /cmd_vel ◀───────┘                 │
 └───────────────────────────┘                     └────────────────────────────────────┘
```

First time? Do [SETUP.md](SETUP.md) once. The examples use Pi `deimos@10.184.3.36`; use your own.
**Take the propellers off** until you have tested everything once on the bench.

## 1. Start the laptop (always first)

```bash
cd ~/gps-denied-vtol/ros2_ws && source install/setup.bash
ros2 launch x500_navigation navigation_real.launch.py pi_ip:=10.184.3.36
```

This starts everything on the laptop: receivers, segmentation, Nav2, the command bridge, RViz
and the health checks. Add `camera:=false`, `segmentation:=false` or `rviz:=false` to leave
parts out.

## 2. Start the Pi

```bash
ssh deimos@10.184.3.36
cd ~/raspberry_pi && python3 start_streams.py --nav-cmds
```

This sends telemetry, lidar and camera to the laptop, restarts any stream that crashes, and
syncs the Pi's clock to the laptop (look for `clock synced to laptop … offset …`).
`--nav-cmds` lets Nav2 steer the drone; leave it out to only watch.

## 3. Check it works

- RViz shows the lidar scan, camera and costmaps, and the status text says **OK**.
- Pick the drone up and move it: it moves in RViz too, and the scan stays lined up with the map.
- Rates: `ros2 topic hz /odom` ≈ 30 Hz, `/scan` ≈ 7 Hz, `/camera/image_raw` ≈ 15 Hz (the current
  USB camera can't go faster), `/segmentation/detections` ≈ 15 Hz.

## 4. Fly

1. Arm and take off with the **RC transmitter** (e.g. in `ALT_HOLD`).
2. Switch to **GUIDED** with the RC mode switch. The laptop cannot arm the drone or change its mode for you.
3. In RViz, click **Nav2 Goal** and pick a spot. The drone only obeys while armed in GUIDED,
   goes at most 0.5 m/s, and stops if commands stop for 0.5 s.
4. To take over, flip the mode switch out of GUIDED.

## 5. Stop

Land and disarm with the RC. Then `Ctrl-C` on the Pi, then `Ctrl-C` on the laptop.

## Safety

- Make sure the RC can always take over. Test it before every autonomous flight.
- Set up the RC-loss and low-battery failsafes in ArduPilot.
- Fly in sight, in a clear area, with a spotter, never over people.

## Something wrong?

| Problem | Try |
| :--- | :--- |
| No telemetry / no `/odom` | Is `start_streams.py` running? Does its first line show the laptop's IP? Laptop firewall: `sudo ufw allow 14550:14553/udp && sudo ufw allow 5000/tcp` |
| `Timed out waiting for heartbeat` (Pi) | Pixhawk powered and plugged in? `ls -la /dev/pixhawk`, `python3 test_connection.py` |
| No `/scan` | Lidar plugged in? `ls /dev/ttyUSB0` (the Pi keeps retrying) |
| No camera | Start the laptop before the Pi. Don't run `live_segmentation.py` at the same time (same port) |
| Drone ignores Nav2 goals | Pi started with `--nav-cmds`? Drone armed **and** in GUIDED? |
| Laggy data | Wi-Fi: see [REFERENCE.md → Link latency](REFERENCE.md#link-latency) |
| Segmentation slow | Run it separately with `sam_imgsz:=768` ([REFERENCE.md](REFERENCE.md#segmentation-yolo--sam)) |
| `Arming DENIED` | The message says why. Indoors without GPS: [SETUP.md → ArduPilot](SETUP.md#3-ardupilot-settings) |

Other ways to run (no ROS, simulation, Pi bench tools): see [raspberry_pi/README.md](../raspberry_pi/README.md) and [SIMULATION.md](SIMULATION.md).
