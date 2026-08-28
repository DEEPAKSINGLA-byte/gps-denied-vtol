# quadcopter_bringup - Package INSTRUCTIONS

Build, launch, and development guidelines for the `quadcopter_bringup` ROS 2
package (autonomy bring-up for the Autonomous Quadcopter Framework).

---

## Table of Contents

1. [Overview](#1-overview)
2. [Prerequisites](#2-prerequisites)
3. [Building the Package](#3-building-the-package)
4. [Launching](#4-launching)
5. [Package Layout](#5-package-layout)
6. [Adding a Node or Launch File](#6-adding-a-node-or-launch-file)
7. [Testing](#7-testing)

---

## 1. Overview

`quadcopter_bringup` is the **ament_python** package that launches the autonomy
stack (planning, VIO fusion, detection, and the PX4 bridge) on top of the
`px4_msgs` / `px4_ros_com` interfaces. It is the entry point for both SITL
simulation and hardware deployment.

---

## 2. Prerequisites

- ROS 2 Humble installed and sourced (`source /opt/ros/humble/setup.bash`).
- A built workspace containing `px4_msgs` and `px4_ros_com`.
- The MicroXRCE-DDS agent running for PX4 connectivity (see the root
  [INSTRUCTIONS.md](../../INSTRUCTIONS.md)).

---

## 3. Building the Package

```bash
cd <workspace>/src
source /opt/ros/humble/setup.bash

# Build everything (fastest when starting fresh)
colcon build --symlink-install

# Build only this package
colcon build --packages-select quadcopter_bringup --symlink-install
```

After building, source the workspace:

```bash
source install/setup.bash
```

### 3.1 Rebuild After Changes

With `--symlink-install`, Python edits take effect on the next `ros2 launch`
/`ros2 run` call. Rebuild only when `setup.py`, `package.xml`, or launch-file
install rules change:

```bash
colcon build --packages-select quadcopter_bringup --symlink-install
```

---

## 4. Launching

### 4.1 Simulation (SITL)

Requires the MicroXRCE-DDS agent over UDP and PX4 SITL running:

```bash
# Terminal 1 - agent
MicroXRCEAgent udp4 -p 8888 -v

# Terminal 2 - PX4 SITL + Gazebo (from the PX4-Autopilot source directory)
make px4_sitl gazebo

# Terminal 3 - bring-up
cd <workspace>/src && source install/setup.bash
ros2 run quadcopter_bringup teleop
```

### 4.2 Hardware

Requires the agent on UART and PX4 powered (with `UXRCE_DDS_CFG` = TELEM2 and
`SER_TEL2_BAUD` = 921600 in QGroundControl):

```bash
MicroXRCEAgent serial --dev /dev/ttyTHS1 -b 921600

cd <workspace>/src && source install/setup.bash
ros2 run quadcopter_bringup teleop
```

### 4.3 Keyboard Teleop Control

The `teleop` executable drives the PX4 velocity setpoints in Offboard mode:

| Key      | Action                    |
| -------- | ------------------------- |
| `w`      | Move forward (+X)         |
| `s`      | Move back (-X)            |
| `a`      | Move left (+Y)            |
| `d`      | Move right (-Y)           |
| `r`      | Move up (-Z, NED)         |
| `f`      | Move down (+Z, NED)       |
| `x`      | Stop / hover in place     |
| `q`      | Increase max speed by 10% |
| `z`      | Decrease max speed by 10% |
| `Ctrl+C` | Exit safely               |

It publishes `OffboardControlMode` and `TrajectorySetpoint` at 20 Hz, switches
PX4 to **Offboard** mode after ~15 setpoints, and arms after ~30.

### 4.4 Verify

```bash
ros2 node list
ros2 topic list
ros2 topic echo /fmu/out/vehicle_attitude
```

---

## 5. Package Layout

```text
quadcopter_bringup/
|-- package.xml                      # Manifest (format 3)
|-- setup.py                         # Build config & console scripts
|-- setup.cfg                        # Ament Python metadata
|-- resource/quadcopter_bringup      # Resource marker
|-- quadcopter_bringup/
|   `-- __init__.py                  # Python package (nodes live here)
|-- launch/                          # Launch files (add yours here)
`-- test/                            # ament_lint tests (flake8, pep257, copyright)
```

> `launch/` files must be added to the `data_files` list in `setup.py` to be
> installed.

---

## 6. Adding a Node or Launch File

### 6.1 New Node

1. Create `<name>_node.py` under `quadcopter_bringup/`.
2. Register it in `setup.py`:

```python
entry_points={
    'console_scripts': [
        '<name>_node = quadcopter_bringup.<name>_node:main',
    ],
},
```

3. Rebuild and run:

```bash
colcon build --packages-select quadcopter_bringup --symlink-install
ros2 run quadcopter_bringup <name>_node
```

### 6.2 New Launch File

1. Create `launch/<name>.launch.py`.
2. Add to `data_files` in `setup.py`:

```python
('share/' + package_name + '/launch', glob('launch/*.launch.py')),
```

3. Rebuild and launch:

```bash
colcon build --packages-select quadcopter_bringup --symlink-install
ros2 launch quadcopter_bringup <name>.launch.py
```

---

## 7. Testing

```bash
cd <workspace>/src
source /opt/ros/humble/setup.bash

# Run lint + tests for this package
colcon test --packages-select quadcopter_bringup
colcon test-result --verbose
```

Tests cover `flake8` (style), `pep257` (docstrings), and `ament_copyright`
licensing. Keep them green before merging.

---

## See Also

- [../../INSTRUCTIONS.md](../../INSTRUCTIONS.md) - Root workspace build & launch
- [../px4_ros_com/README.md](../px4_ros_com/README.md) - PX4 bridge examples
- [../px4_msgs/README.md](../px4_msgs/README.md) - Message definitions