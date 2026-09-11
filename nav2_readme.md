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
This establishes the necessary communication between PX4/Gazebo and ROS 2. It starts the `MicroXRCEAgent` for PX4 telemetry, the `ros_gz_bridge` for `/scan` and clock synchronization, and the `tf2.py` script to broadcast the `odom` → `base_footprint` transform.
```bash
source /home/deepak/gps-denied-vtol/ros2_ws/install/setup.bash
ros2 launch /home/deepak/gps-denied-vtol/nav2/bridge_and_tf.launch.py
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

## Visualization
Once all terminals are running, open a new terminal and launch RViz2 to set navigation goals and visualize the map:
```bash
source /opt/ros/humble/setup.bash
rviz2
```
*Note: Make sure your RViz configuration is subscribed to the `/map` and `/scan` topics.*
