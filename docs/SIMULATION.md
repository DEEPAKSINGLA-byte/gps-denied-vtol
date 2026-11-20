# Simulation

The simulator runs **PX4** (not ArduPilot) in **Gazebo Harmonic** and talks to ROS 2 through
the **MicroXRCE-DDS agent**.

**You need:** the laptop setup from [SETUP.md](SETUP.md#1-laptop),
[PX4-Autopilot](https://docs.px4.io/main/en/dev_setup/building_px4.html) in `~/PX4-Autopilot`, and
the agent (`sudo apt install ros-humble-micro-ros-agent`).

In every ROS terminal: `source ~/gps-denied-vtol/ros2_ws/install/setup.bash`

## A. Navigation with mapping

A drone with a 2D lidar in a floor-plan world. SLAM builds the map, Nav2 plans, and
`x500_px4_bridge` flies PX4. Use 5 terminals, in this order:

```bash
# 1. Simulator
cd ~/PX4-Autopilot
PX4_GZ_NO_FOLLOW=1 PX4_GZ_MODEL_POSE="7.5,7.5,0.2" PX4_GZ_WORLD=floorplan make px4_sitl gz_x500_lidar_2d

# 2. Agent + bridges + odometry
ros2 launch x500_navigation bridge_and_tf.launch.py

# 3. Mapping
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true

# 4. Nav2
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true \
  params_file:=$HOME/gps-denied-vtol/ros2_ws/src/x500_navigation/config/nav2_x500.yaml

# 5. Nav2 → PX4 (add -p safety_enabled:=true for a lidar safety stop)
ros2 run x500_px4_bridge x500_px4_bridge --ros-args -p use_sim_time:=true
```

Then run `rviz2`, show `/map` and `/scan`, and click **Nav2 Goal**.
To explore on its own: `ros2 launch explore_lite explore.launch.py use_sim_time:=true`.

## B. Fly with the keyboard

```bash
ros2 launch quadcopter_bringup sitl_gz.launch.py      # simulator + agent + bridge
ros2 run quadcopter_bringup teleop                     # second terminal
```

Keys: `w`/`s` forward/back, `a`/`d` left/right, `r`/`f` up/down, `x` stop, `q`/`z` faster/slower.
More: [quadcopter_bringup/INSTRUCTIONS.md](../ros2_ws/src/quadcopter_bringup/INSTRUCTIONS.md).

## Something wrong?

| Problem | Try |
| :--- | :--- |
| No `/fmu/*` topics | Start the agent: `MicroXRCEAgent udp4 -p 8888` |
| Simulator won't start | Run it from `~/PX4-Autopilot`; Gazebo must be Harmonic (`gz`, not `ign`) |
| `package not found` | `source install/setup.bash` after `colcon build` |
| PX4 won't arm | In the `pxh>` shell: `param set NAV_DLL_ACT 0`, `param save`, `commander arm` |
| Drone ignores Nav2 | Is terminal 5 running? `use_sim_time:=true` everywhere? |
