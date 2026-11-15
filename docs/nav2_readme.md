# Nav2 + PX4 Gazebo Simulation Setup

This guide provides the complete set of commands needed to launch the GPS-denied drone navigation stack using ROS 2 Nav2, SLAM Toolbox, and PX4 Autopilot in Gazebo.

## Prerequisites
Before running the simulation, ensure your ROS 2 workspace is built. If you make any changes to the `x500_px4_bridge` or other packages, build them using:
```bash
cd /home/deepak/gps-denied-vtol/ros2_ws
colcon build
source install/setup.bash
```

## Launch Sequence

You will need to open **5 separate terminals**. Run one block of commands in each terminal in the exact order below.

### Terminal 1: Start PX4 & Gazebo Simulation
This launches the Gazebo simulator, loads the floorplan world, and spawns the x500 drone equipped with a 2D LiDAR.
```bash
cd ~/PX4-Autopilot
PX4_GZ_NO_FOLLOW=1 PX4_GZ_MODEL_POSE="7.5,7.5,0.2" PX4_GZ_WORLD=floorplan make px4_sitl gz_x500_lidar_2d
```

### Terminal 2: Start ROS-Gazebo Bridges & Odometry TF
This establishes communication between PX4/Gazebo and ROS 2. It starts `MicroXRCEAgent` for PX4 telemetry, `ros_gz_bridge` for `/scan` and clock synchronization, and broadcasts the `odom` → `base_footprint` transform.

Standard ROS 2 command:
```bash
source /home/deepak/gps-denied-vtol/ros2_ws/install/setup.bash
ros2 launch x500_navigation bridge_and_tf.launch.py
```

### Terminal 3: Start SLAM Toolbox
This node processes the incoming 2D LiDAR scans to construct a map of the environment and localize the drone within it.
```bash
source /home/deepak/gps-denied-vtol/ros2_ws/install/setup.bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
```

### Terminal 4: Start Nav2
This brings up the full Nav2 stack (path planning, costmaps, and behavior trees).
```bash
source /home/deepak/gps-denied-vtol/ros2_ws/install/setup.bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true params_file:=/home/deepak/gps-denied-vtol/ros2_ws/src/x500_navigation/config/nav2_x500.yaml
```

### Terminal 5: Start the PX4 Control Bridge
This runs our custom node that converts `/cmd_vel_nav` twist commands from Nav2 into `TrajectorySetpoint` commands for PX4, and switches the drone into `OFFBOARD` mode.
```bash
source /home/deepak/gps-denied-vtol/ros2_ws/install/setup.bash
ros2 run x500_px4_bridge x500_px4_bridge --ros-args -p use_sim_time:=true
```
*(LiDAR safety is turned off by default. To re-enable it on demand, add `-p safety_enabled:=true`)*

---

### Optional: Autonomous Frontier Exploration
To automatically explore and map the environment without manually setting 2D nav goals in RViz:
```bash
source /home/deepak/gps-denied-vtol/ros2_ws/install/setup.bash
ros2 launch explore_lite explore.launch.py use_sim_time:=true
```

## Visualization
Once all terminals are running, open a new terminal and launch RViz2 to set navigation goals and visualize the map:
```bash
source /opt/ros/humble/setup.bash
rviz2
```
*Note: Make sure your RViz configuration is subscribed to the `/map` and `/scan` topics.*

## RViz validation dashboard & metrics

```bash
cd ros2_ws && colcon build --packages-select gps_denied_bridge x500_navigation && source install/setup.bash
ros2 launch x500_navigation navigation_real.launch.py pi_ip:=<PI_IP>      # everything, incl. RViz
ros2 launch x500_navigation visualization.launch.py                        # RViz + metrics only
# options: rviz:=false  metrics:=false  camera:=false
```

RViz (`x500_navigation/rviz/gps_denied_real.rviz`, fixed frame `odom`, view follows `base_link`):
YDLIDAR scan, RGB camera, TF tree, local/global costmaps, global/local plan, dead-reckoned
odometry trail, and a `/metrics/overlay` layer (range rings, nearest-obstacle line, status text).
Saved views: *Chase 3D*, *Global overview*.

Metrics (`gps_denied_bridge/metrics_monitor`) — all `std_msgs/Float32`, plot with PlotJuggler/`rqt_plot`:

| Topic | Meaning |
|---|---|
| `/metrics/{lidar,telemetry,camera}/latency_ms` | Pi->laptop one-way latency (needs chrony/NTP on both, or set `clock_offset_s`) |
| `/metrics/*/latency_excess_ms` | latency minus 10 s window minimum: network jitter, independent of clock offset |
| `/metrics/*/rate_hz`, `/metrics/lidar/jitter_ms` | stream rate / inter-arrival jitter |
| `/metrics/camera/{decode_ms,bandwidth_mbps}` | decode cost and link load |
| `/metrics/lidar/{nearest_obstacle_m,valid_ratio,tf_fail_ratio}` | scan quality; `tf_fail_ratio` = scans Nav2 could not transform |
| `/metrics/tf/odom_base_age_ms` | staleness of odom->base_link |
| `/metrics/costmap/local/scan_agreement` | fraction of lidar hits landing on lethal costmap cells (~1 = costmap matches the lidar) |
| `/metrics/costmap/{local,global}/{rate_hz,lethal_cells}` | costmap update rate / occupancy |
| `/metrics/odom/{speed_mps,distance_from_start_m,path_length_m}` | odometry/drift tracking |
| `/metrics/nav/{cmd_vel_rate_hz,cmd_speed_mps,plan_length_m}` | planner/controller output |

`/diagnostics` carries an OK/WARN/FAIL summary per subsystem (`rqt_robot_monitor`).
