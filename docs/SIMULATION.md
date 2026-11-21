# Simulation

The simulator runs **PX4** (not ArduPilot) in **Gazebo Harmonic** and talks to ROS 2 through
the **MicroXRCE-DDS agent**.

**You need:** the laptop setup from [SETUP.md](SETUP.md#1-laptop),
[PX4-Autopilot](https://docs.px4.io/main/en/dev_setup/building_px4.html) in `~/PX4-Autopilot`, and
the agent (`sudo apt install ros-humble-micro-ros-agent`).

In every ROS terminal: `source ~/gps-denied-vtol/ros2_ws/install/setup.bash`

## Baylands: lidar drone, SLAM and Nav2 in one command

The x500 with a 2D lidar (`x500_lidar_2d`, 30 m range, 270°) in PX4's **Baylands** park
world. The drone starts inside a grove in the west of the park: rows of trees about
10 m apart, 15 trees within 20 m. The trees are 3–4.6 m tall with leaves from about 1.2 to 3 m,
so at the default 2 m flying height every crown shows up in the lidar scan. slam_toolbox maps the park, Nav2 plans around trees and buildings, and
`x500_px4_bridge` flies PX4 in OFFBOARD mode. One launch file starts everything:

```bash
cd ~/gps-denied-vtol/ros2_ws && colcon build --packages-select x500_px4_bridge x500_navigation
source install/setup.bash
ros2 launch x500_navigation sim_baylands.launch.py
```

After about 30 s the drone arms by itself, climbs to 2 m and holds there (look for
`PX4 is in OFFBOARD and armed: taking off`). In RViz, click **2D Goal Pose** and click
on the map. Goals can be outside the area mapped so far. Ctrl+C stops everything,
including PX4 and Gazebo.

When the Gazebo window has loaded, the launch points the camera at the drone (from behind and
above, set by `camera_offset`), then leaves it free. Moving the Gazebo camera: **left-drag** pans,
**middle-drag** or **Shift + left-drag** rotates, **scroll** or **right-drag** zooms.
To jump to the drone, right-click it → **Move to**. For tracking, right-click
it → **Follow** (or launch with `follow:=true`). The **⋮** menu (top right) →
**View Angle** gives top, side and front views with one click.

| Argument | Default | Meaning |
| :--- | :--- | :--- |
| `headless` | `false` | `true`: no Gazebo window (faster) |
| `rviz` | `true` | Start RViz with `rviz/sim_baylands.rviz` |
| `follow` | `false` | `false`: free Gazebo camera that starts pointed at the drone. `true`: camera tracks the drone (view locked) |
| `camera_offset` | `-2.5,-2.5,2.5` | Start position of the free camera, `x,y,z` in m from the spawn point |
| `follow_offset` | `-4.0,-4.0,3.0` | With `follow:=true`, camera offset `x,y,z` from the drone |
| `altitude` | `2.0` | Flying height in m; the lidar scans about 0.3 m above it |
| `max_speed` | `1.0` | Horizontal speed limit in m/s |
| `safety` | `true` | Lidar safety in `x500_px4_bridge`: removes motion towards anything closer than 0.65 m and gently pushes away from anything closer than 0.45 m |
| `pose` | `-80.7,93.2,0.0,0,0,0` | Spawn pose `"x,y,z,roll,pitch,yaw"` in Gazebo metres. Default is inside the tree grove; `0,0,0,0,0,0` is PX4's open-field spawn |
| `px4_dir` | `~/PX4-Autopilot` | PX4 source folder |
| `nav2_params` | `config/nav2_baylands.yaml` | Nav2 settings |

Send a goal from a terminal instead of RViz:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 25.0, y: -5.0}, orientation: {w: 1.0}}}}"
```

How it fits together:

```
Gazebo (baylands) ──gz_bridge──▶ /clock, /scan ──▶ slam_toolbox ──▶ /map, map→odom
PX4 SITL ──MicroXRCE agent──▶ /fmu/out/vehicle_odometry ──tf2_broadcaster──▶ odom→base_footprint→base_link, /odom
Nav2 (nav2_baylands.yaml) ──/cmd_vel_nav──▶ x500_px4_bridge ──▶ /fmu/in/trajectory_setpoint (OFFBOARD)
```

Things to know:

- **Gazebo-to-ROS bridge.** The apt `ros-humble-ros-gz-bridge` is built for Gazebo
  Fortress and can't hear Gazebo Harmonic, which PX4 uses: it starts, but no message ever
  arrives. `x500_px4_bridge/gz_bridge.py` uses Harmonic's own Python bindings instead
  (`python3-gz-transport13`, `python3-gz-msgs10`).
- **PX4 settings.** The launch sets `NAV_DLL_ACT=0` (no ground station needed to arm) and
  `COM_RCL_EXCEPT=4` (OFFBOARD does not need an RC link).
- **GPS.** PX4's position estimate in this world still uses the simulated GPS. The lidar
  is used for mapping and obstacle avoidance; it is not yet fed back into PX4's estimator.

## Office: indoor rooms, corridor and doorways

The same drone and stack in an indoor office (`x500_navigation/worlds/office.sdf`):
a 24 × 16 m floor with an open-plan area (desk pods, pillars, shelves), a meeting room,
a kitchen, two private offices, a server room with racks and a lobby, joined by a 2.5 m
corridor and 1.6 m doorways. The walls are 3 m high and there is no ceiling, so the Gazebo
camera can look in from above.

```bash
ros2 launch x500_navigation sim_office.launch.py
```

The drone starts in the lobby, takes off to **1.5 m** and flies at up to **0.5 m/s**. It takes
the same arguments as Baylands (`headless`, `rviz`, `follow`, `camera_offset`, `pose`,
`altitude`, `max_speed`, `safety`, ...); the office uses `config/nav2_office.yaml`,
`config/slam_office.yaml` and `rviz/sim_office.rviz`.

**Autonomous exploration (on by default).** A few seconds after take-off, `explore_lite`
(the `m-explore-ros2` package in this workspace) starts: it looks at the SLAM map for
*frontiers*, the edges between mapped free space and unknown space, sends the most
promising one to Nav2 as a goal, and repeats until no frontier is left. Then it flies the
drone back to the lobby. In RViz the frontiers show as markers (**Exploration frontiers**).
The terminal prints `Exploration stopped.` when it is done.

| Argument | Default (office) | Meaning |
| :--- | :--- | :--- |
| `explore` | `true` | `false`: no exploring; send goals yourself with **2D Goal Pose** |
| `explore_delay` | `8.0` | Seconds between take-off and the first frontier goal |
| `explore_params` | `config/explore_office.yaml` | explore_lite settings (frontier size, timeouts, return to start) |

Don't send your own goals while it explores: Nav2 takes one goal at a time, and
explore_lite would replace yours with the next frontier. To pause and resume it:
`ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: false}"` (`true` to resume).
The same option works in Baylands (`sim_baylands.launch.py explore:=true`), but the park
has no walls, so it would keep finding frontiers for a very long time.

The lobby entrance is a closed glass door, so the floor is sealed: otherwise the explorer
would see the outdoors as unexplored space and leave the building.

What a 2D lidar can and cannot see indoors: it only sees the slice it scans, about 0.3 m
above the drone. The office is built so that this is safe at the default height: desks,
tables, chairs, sofas and counters are at most 1.1 m tall (the drone flies over them, the
lidar does not see them); cabinets, shelves, racks, the fridge, plants and pillars are at
least 2 m tall (the lidar sees them and Nav2 plans around them). Nothing sits in between.
If you fly lower than about 1.3 m, the drone can hit furniture the lidar never saw.

To change the layout, edit `worlds/make_office_world.py` and run it; it rewrites
`office.sdf`. Then `colcon build --packages-select x500_navigation`.

How it starts: PX4's own script only loads worlds from `~/PX4-Autopilot`, so for the office
the launch starts Gazebo itself (with PX4's environment from `gz_env.sh`), waits until the
world answers, then starts PX4, which attaches to the running world and spawns the drone.

## How the drone flies (both worlds)

- **It turns to face where it is going.** The lidar covers 270° and is blind over the rear
  90°, so `x500_px4_bridge` points the drone in its direction of travel and slows down
  while it is still turning (`yaw_follows_motion`, on by default).
- **Recovery without spinning or reversing.** Nav2's default recoveries spin the robot
  (useless here) and back it up (straight into the blind spot). The simulation uses
  `behavior_trees/navigate_drone.xml` instead: clear the costmaps, hover a few seconds,
  try again.
- **Lidar safety** (see `safety` above) only removes the part of a command that points at
  a close obstacle, so the drone can still slide along a wall or back out of a doorway.

## Frames (both worlds)

```
map ──slam_toolbox──▶ odom ──▶ base_footprint ──▶ base_link ──fixed──▶ link (lidar)
                               x, y, heading      height, roll, pitch
```

`map`, `odom` and `base_footprint` are at floor level; `base_link` is the real body.
SLAM and Nav2 work in 2D on `base_footprint`, and costmap obstacle heights are real heights
above the floor.

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
| No `/scan` or `/clock` (either world) | `gz topic -l` should list `/world/baylands/...`; `python3 -c "import gz.transport13"` must work |
| Baylands: drone never takes off | Check the `pxh>` lines for a failed preflight check; the bridge gives up after 120 s |
| Baylands: goal aborted at once | The goal is inside an obstacle or more than 40 m away (global costmap is 80 × 80 m) |
| `TF_OLD_DATA` warnings flood the terminal | Two simulations were running at once. The launch now refuses to start a second copy and stops nodes left over from a crashed run. If it says *already running*, stop the other one: `pkill -INT -f "ros2 launch x500_navigation"` |
| Office: `Gazebo world office did not start` | Check that PX4 is built (`make px4_sitl` once) and that `~/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh` exists |
