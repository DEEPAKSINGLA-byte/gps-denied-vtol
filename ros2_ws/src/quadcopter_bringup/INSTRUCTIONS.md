# quadcopter_bringup

Simulation bring-up for PX4: one launch for PX4 SITL + Gazebo + the ROS bridge, and a keyboard
teleop node. Full walkthrough: [docs/SIMULATION.md](../../../docs/SIMULATION.md).

## Build

```bash
cd ~/gps-denied-vtol/ros2_ws && source /opt/ros/humble/setup.bash
colcon build --packages-select quadcopter_bringup --symlink-install
source install/setup.bash
```

With `--symlink-install`, Python edits apply without rebuilding. Rebuild after changing
`setup.py`, `package.xml` or launch files.

## Run

```bash
ros2 launch quadcopter_bringup sitl_gz.launch.py                 # agent + PX4 SITL (x500, baylands) + bridge
ros2 launch quadcopter_bringup sitl_gz.launch.py world:=forest headless:=true
ros2 run quadcopter_bringup teleop                               # second terminal
```

The launch uses Gazebo Harmonic (`gz`, version 8+), which PX4 main requires. The bridge topics
are listed in `config/sitl_gz_bridge.yaml`; Humble's bridge takes them as command-line
arguments, so the launch file passes them.

**Teleop keys:** `w`/`s` forward/back, `a`/`d` left/right, `r`/`f` up/down, `x` stop,
`q`/`z` speed ±10%, `Ctrl+C` quit. It streams setpoints at 20 Hz, switches PX4 to Offboard
after ~15 setpoints and arms after ~30.

If PX4 won't arm in SITL (no RC or ground station), type in the `pxh>` shell:
`param set NAV_DLL_ACT 0`, `param save`, `commander arm`.

**On real PX4 hardware:** start the agent on the UART first
(`MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600`, with PX4 `UXRCE_DDS_CFG`=TELEM2 and
`SER_TEL2_BAUD`=921600), then run `teleop`.

Check: `ros2 topic echo /fmu/out/vehicle_attitude`.

## Files

```text
launch/sitl_gz.launch.py      PX4 SITL + agent + ros_gz_bridge
config/sitl_gz_bridge.yaml    bridge topic list
quadcopter_bringup/           Python nodes (teleop, teleop_gui)
test/                         flake8, pep257, copyright checks
```

New node: add `<name>.py` in `quadcopter_bringup/`, register it in `setup.py` under
`console_scripts`, rebuild. New launch file: put it in `launch/` (already installed by the
`glob('launch/*.launch.py')` rule in `setup.py`).

Tests: `colcon test --packages-select quadcopter_bringup && colcon test-result --verbose`

See also: [px4_ros_com](https://github.com/PX4/px4_ros_com), [px4_msgs](../../px4_msgs/README.md).
