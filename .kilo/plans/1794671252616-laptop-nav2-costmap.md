# Plan: Laptop-side Nav2 Costmap & Navigation from Pixhawk + LiDAR Data

## Goal

Enable the ground laptop to receive Pixhawk telemetry (including optical flow MTF01) and YDLIDAR data over Wi-Fi, build 2D costmaps, and run the full Nav2 navigation stack for GPS-denied VTOL flight — all on the laptop, with commands bridged back to the Pixhawk via the Raspberry Pi companion.

## Current State (inventory)

**Raspberry Pi** (`raspberry_pi/`):
- `pixhawk_controller.py` — MAVLink engine; auto-detects `/dev/pixhawk`, `/dev/ttyACM*`, `/dev/serial0` at 115200 baud; waits for HEARTBEAT; requests data streams at 10 Hz; background telemetry thread with auto-reconnect. Publishes arm/disarm/set_mode/takeoff via MAVLink commands.
- `telemetry_streamer.py` — Streams parsed JSON telemetry (UDP 14551) + raw MAVLink (UDP 14550) to laptop. Includes `raw_packet_callback` forwarding and a command listener on UDP 14552 (arm/disarm/set_mode/emergency_kill).
- `lidar_udp_streamer.py` — Streams YDLIDAR G2 scans as JSON over UDP 14552 to laptop.
- Optical flow data is already in the telemetry JSON (`optical_flow` key from `get_telemetry()`).

**Laptop** (`raspberry_pi/` + `ros2_ws/`):
- `laptop_telemetry_receiver.py` — Receives JSON telemetry on UDP 14551 (dashboard + verify modes).
- `lidar_ros2_receiver.py` — ROS 2 node; listens UDP 14552, publishes `sensor_msgs/LaserScan` to `/scan` (frame_id `laser_frame`).
- `plotjuggler_bridge.py` — Receives telemetry UDP 14551; optional ROS 2 topic publishing (`/drone/imu`, `/drone/attitude`, etc.).
- `ros2_ws/src/x500_navigation/` — Nav2 config (`nav2_x500.yaml`) with local/global costmap, planner, controller, BT navigator. **Uses `use_sim_time: true`** and PX4 uORB topics — designed for Gazebo/PX4 sim, NOT real ArduPilot Pixhawk.
- `ros2_ws/src/x500_px4_bridge/` — Publishes `TrajectorySetpoint` to PX4 uORB (`/fmu/in/*`). **Incompatible with ArduPilot over MAVLink.**
- `ros2_ws/src/quadcopter_localization/` — Visual odometry bridge params + EKF fusion stub.
- `ros2_ws/src/quadcopter_mapping/` — Empty stub package.

## Key Gaps

1. No laptop node converts MAVLink JSON telemetry → ROS 2 `/odom` + TF (`odom` → `base_link`) needed by Nav2 costmaps.
2. Optical flow MTF01 data is received but not fused into state estimation for GPS-denied hold.
3. Nav2 config uses `use_sim_time: true`; needs `false` for real hardware.
4. No bridge from Nav2 `/cmd_vel` back to Pixhawk MAVLink commands (ArduPilot `MAV_CMD_*` / GUIDED velocity setpoints).
5. No TF: `base_link` → `laser_frame` static transform for the YDLIDAR mount.
6. Costmap params tuned for sim (2.5 m scan range, 0.45 m robot radius); need retuning for real YDLIDAR (8–16 m range).

## Design Decisions

| Decision | Rationale |
|---|---|
| Telemetry path: Pi `telemetry_streamer.py` JSON UDP 14551 → laptop new `mavlink_to_ros2_bridge` node | Reuses existing streaming infrastructure; JSON is already parsed and includes optical flow. |
| Lidar path: Pi `lidar_udp_streamer.py` UDP 14552 → laptop `lidar_ros2_receiver.py` → `/scan` | Already implemented; just needs to be run. |
| Command path: laptop sends JSON command to Pi UDP 14552 → Pi `telemetry_streamer.py` `_command_listener_worker` → `PixhawkVehicle` MAVLink commands | Reuses existing command listener on the Pi; no new serial access needed on laptop. |
| State estimation: fuse optical flow (XY velocity) + barometer/rangefinder (Z) + IMU (attitude) into `/odom` on laptop | GPS-denied: optical flow provides XY velocity dead-reckoning; rangefinder provides altitude. |
| Nav2 `use_sim_time: false` | Real hardware, no Gazebo clock. |
| Local costmap: rolling window, `/scan` source, 0.05 m resolution | Standard Nav2 local costmap for obstacle avoidance. |
| Global costmap: static layer + obstacle layer + inflation, `/scan` source | Enables mapping + planning over known space. |

## Implementation Tasks

### Task 1: Verify data reception on laptop
- Run Pi `telemetry_streamer.py --target-ip <laptop-ip> --rate 10`
- Run Pi `lidar_udp_streamer.py --target-ip <laptop-ip>`
- On laptop, run `laptop_telemetry_receiver.py --verify` (confirm optical flow field present)
- On laptop, run `lidar_ros2_receiver.py` and confirm `/scan` topic publishes LaserScan
- **Exit criteria**: JSON telemetry with `optical_flow` key received; `/scan` topic active with valid ranges

### Task 2: Create `mavlink_to_ros2_bridge` ROS 2 node (new package `gps_denied_bridge` in `ros2_ws/src/`)
- Subscribe to nothing; create UDP socket listening on `0.0.0.0:14551` for JSON telemetry from Pi
- Parse each JSON packet into ROS 2 messages:
  - `nav_msgs/Odometry` on `/odom`:
    - `pose.pose.position.x/y/z`: integrate optical flow velocity over time (dead-reckoning from last known position), z from barometer/rangefinder altitude
    - `pose.pose.orientation`: from attitude roll/pitch/yaw (convert to quaternion)
    - `twist.twist.linear.x/y/z`: optical flow flow_x/flow_y (m/s) + rangefinder vertical velocity; z from baro rate
    - `twist.twist.angular.x/y/z`: from gyro (converted deg/s → rad/s)
    - `header.frame_id`: `odom`; `child_frame_id`: `base_link`
  - `tf2_ros.TransformBroadcaster`: `odom` → `base_link` from odometry pose
- Publish optical flow as `geometry_msgs/Vector3Stamped` on `/drone/optical_flow` (for logging/debug)
- **Exit criteria**: `/odom` topic publishing at ~10 Hz; TF `odom` → `base_link` available in `tf2_ros`

### Task 3: Add static TF `base_link` → `laser_frame`
- Run `tf2_ros static_transform_publisher 0.12 0 0.26 0 0 0 base_link laser_frame` (matching existing sim launch args; adjust for real lidar mount)
- **Exit criteria**: `tf2_ros` can resolve `base_link` → `laser_frame`

### Task 4: Create real-hardware Nav2 costmap config
- New file `ros2_ws/src/x500_navigation/config/nav2_real.yaml` (copy of `nav2_x500.yaml` with changes):
  - `use_sim_time: false` everywhere
  - `global_frame: odom` for local_costmap (rolling window, no map needed initially); `map` for global_costmap if static layer used, else `odom`
  - `robot_base_frame: base_link`
  - Local costmap: `width: 8`, `height: 8`, `resolution: 0.05`, `robot_radius: 0.25`, `obstacle_max_range: 8.0` (YDLIDAR G2 max 16 m, clamp to 8 for costmap)
  - Global costmap: `width: 20`, `height: 20`, `resolution: 0.05`, plugins `static_layer`, `obstacle_layer`, `inflation_layer`; `static_layer.map_subscribe_transient_local: true`
  - DWB critic limits: `max_vel_x: 0.5`, `max_speed_xy: 0.5` (safe indoor speeds)
- **Exit criteria**: YAML parses without Nav2 parameter errors

### Task 5: Create Nav2 bringup launch for real hardware
- New file `ros2_ws/src/x500_navigation/launch/navigation_real.launch.py`:
  - Launch `navigation_launch.py` from `nav2_bringup` with `use_sim_time:=false`, `params_file:=nav2_real.yaml`
  - Launch `mavlink_to_ros2_bridge` node
  - Launch `lidar_ros2_receiver_node` (from existing package or new)
  - Launch `static_transform_publisher` for `base_link` → `laser_frame`
- **Exit criteria**: `ros2 launch` brings up costmap nodes, `/odom`, `/scan`, `/cmd_vel` topics active

### Task 6: Create ROS 2 → MAVLink command bridge (new node in `gps_denied_bridge`)
- Subscribe to `/cmd_vel` (`geometry_msgs/Twist`) from Nav2 controller
- Convert to MAVLink commands sent to Pixhawk via the Pi's existing command listener (UDP to Pi IP:14552):
  - JSON commands: `{"action": "set_mode", "mode": "GUIDED"}`, `{"action": "arm", "force": true}`
  - For velocity control in GUIDED: use `MAV_CMD_DO_SET_MODE` to GUIDED, then send `MAV_CMD_NAV_VELOCITY` or `SET_POSITION_TARGET_LOCAL_NED` equivalent
- Alternative simpler path: publish `geometry_msgs/Twist` to a UDP socket that Pi `telemetry_streamer.py` forwards... but existing listener only handles discrete actions (arm/disarm/set_mode), not continuous velocity.
- **Recommended**: New Pi-side node `mavlink_cmd_listener.py` that listens UDP 14553 for `geometry_msgs/Twist`-like JSON and sends `MAV_CMD_DO_SET_MODE` (GUIDED) + `MAV_CMD_NAV_VELOCITY` via `PixhawkVehicle`. Laptop sends Twist JSON to Pi UDP 14553.
- **Exit criteria**: `/cmd_vel` from Nav2 results in Pixhawk mode change to GUIDED and velocity commands accepted

### Task 7: Build and source ROS 2 workspace
- `colcon build` in `ros2_ws`
- Source `install/setup.bash`
- **Exit criteria**: `ros2 node list` shows `mavlink_to_ros2_bridge`, `lidar_ros2_receiver_node`, Nav2 nodes

### Task 8: End-to-end validation
- Start Pi: `telemetry_streamer.py` + `lidar_udp_streamer.py` + `mavlink_cmd_listener.py`
- Start laptop: `ros2 launch x500_navigation navigation_real.launch.py`
- Open RViz2; subscribe to `/scan`, `/odom`, `/global_costmap/costmap`, `/local_costmap/costmap`
- Verify costmap shows obstacles from lidar
- Send a Nav2 goal (`geometry_msgs/PoseStamped` to `/goal_pose`) in RViz2
- Verify drone moves (in GUIDED) toward goal while avoiding obstacles
- **Exit criteria**: Costmap populates from lidar scans; Nav2 computes path; `/cmd_vel` flows; Pixhawk receives and acts on commands

## Risks & Mitigations

| Risk | Mitigation |
|---|---|
| Wi-Fi latency/jitter (Redmi 10) causes odometry drift | Limit costmap update frequency (5 Hz local); use short prediction horizon; rely on Pixhawk's onboard EKF for inner-loop stability |
| Optical flow fails over textureless surfaces | Fall back to barometer/rangefinder altitude hold only; drift horizontally — use `ALT_HOLD` instead of `POSHOLD` |
| `use_sim_time` mismatch prevents Nav2 from starting | Ensure all nodes (including Nav2 bringup) use `use_sim_time: false`; remove `/clock` dependency |
| ArduPilot GUIDED velocity mode behavior differs from PX4 OFFBOARD | Test with propellers removed; start with `ALT_HOLD` + manual; verify `MAV_CMD_NAV_VELOCITY` acceptance |
| TF chain broken (`odom` → `base_link` → `laser_frame`) | Validate with `ros2 run tf2_tools view_frames` or `ros2 echo /tf_static` |

## Validation Plan

1. `laptop_telemetry_receiver.py --verify` → all sensor checks pass including `optical_flow`
2. `ros2 topic echo /scan` → LaserScan with valid finite ranges
3. `ros2 topic echo /odom` → odometry updates at ~10 Hz with position/velocity
4. `ros2 run tf2_tools view_frames` → tree: `odom` → `base_link` → `laser_frame`
5. RViz2: costmap layers visible, obstacles marked from lidar
6. RViz2 2D Nav Goal → `/cmd_vel` nonzero → Pi receives JSON command → Pixhawk switches to GUIDED

## Open Questions (to resolve before implementation)

1. What is the laptop's IP address on the Redmi 10 network? (Pi `telemetry_streamer.py` needs `--target-ip`)
2. What is the YDLIDAR mount offset (x, y, z, yaw) relative to `base_link` for the static TF?
3. Does the Pixhawk run ArduPilot (confirmed by `pixhawk_controller.py` docs) — what GUIDED velocity command does it accept (`MAV_CMD_NAV_VELOCITY` vs `SET_POSITION_TARGET_LOCAL_NED`)?
4. Should the local costmap use `global_frame: odom` (no map, rolling) or `map` (requires SLAM/global localization)?
