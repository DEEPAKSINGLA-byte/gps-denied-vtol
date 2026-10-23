# quadcopter_navigation — Nav2 + PX4 Gazebo Simulation

ROS 2 package holding the Nav2 configuration and bringup launch for the
quadcopter GPS-denied VTOL platform (mirrors `x500_navigation`).

## Layout

```text
quadcopter_navigation/
  launch/bridge_and_tf.launch.py  # MicroXRCEAgent + ros_gz_bridge + odom/static TF
  config/nav2_quadcopter.yaml      # Nav2 params (DWB, costmaps, behavior servers)
```

## Prerequisites

Build the workspace first (from the repo root):

```bash
cd software/src
colcon build --symlink-install
source install/setup.bash
```

If you change `quadcopter_px4_bridge` or this package, rebuild them:

```bash
cd software/src
colcon build --packages-select quadcopter_px4_bridge quadcopter_navigation --symlink-install
source install/setup.bash
```

## Launch sequence

Open **5 separate terminals**. Run one block per terminal, in order.
(Source `software/src/install/setup.bash` in each ROS terminal, after
sourcing `/opt/ros/humble/setup.bash`.)

### Terminal 1: PX4 & Gazebo simulation

Launches Gazebo, loads the `floorplan` world, and spawns the x500 drone
with a 2D LiDAR.

```bash
cd ~/PX4-Autopilot
PX4_GZ_NO_FOLLOW=1 PX4_GZ_MODEL_POSE="7.5,7.5,0.2" PX4_GZ_WORLD=floorplan make px4_sitl gz_x500_lidar_2d
```

### Terminal 2: ROS–Gazebo bridges & TF

Starts `MicroXRCEAgent` (PX4 telemetry), `ros_gz_bridge` (`/scan` + clock),
and the static `base_footprint` → `link` transform.

```bash
source software/src/install/setup.bash
ros2 launch quadcopter_navigation bridge_and_tf.launch.py
```

> The launch bridges the floorplan model odometry to `/odom` and publishes
> `odom` → `base_footprint` TF. If your simulator uses a different model
> name or topic, inspect `gz topic -l` and update the mapping in
> `launch/bridge_and_tf.launch.py`.

### Terminal 3: SLAM Toolbox

Builds a map from 2D LiDAR scans and localizes the drone in it.

```bash
source software/src/install/setup.bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
```

### Terminal 4: Nav2

Brings up planning, costmaps, and behavior trees.

```bash
source software/src/install/setup.bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true \
  params_file:=$(ros2 pkg prefix quadcopter_navigation)/share/quadcopter_navigation/config/nav2_quadcopter.yaml
```

### Terminal 5: PX4 control bridge

Converts Nav2 `/cmd_vel_nav` commands into PX4 `TrajectorySetpoint`s and
switches the drone to `OFFBOARD` mode.

```bash
source software/src/install/setup.bash
ros2 run quadcopter_px4_bridge quadcopter_px4_bridge --ros-args -p use_sim_time:=true
```

## Visualization

Once all terminals run, launch RViz2 to set goals and visualize the map:

```bash
source /opt/ros/humble/setup.bash
rviz2
```

Subscribe RViz to `/map` and `/scan`.
