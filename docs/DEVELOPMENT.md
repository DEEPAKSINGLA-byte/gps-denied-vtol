# Development

## Build

```bash
cd ~/gps-denied-vtol/ros2_ws && source /opt/ros/humble/setup.bash
colcon build --symlink-install                                  # everything
colcon build --symlink-install --packages-select gps_denied_bridge   # one package
source install/setup.bash
```

With `--symlink-install`, Python edits apply without rebuilding. Rebuild after changing launch
files, configs or `setup.py`. Changed something in `raspberry_pi/`? Copy it to the Pi again
([SETUP.md](SETUP.md#2-raspberry-pi)).

## Packages

| Package | What it does |
| :--- | :--- |
| `gps_denied_bridge` | Connects Pi and laptop: telemetry → `/odom`, lidar → `/scan`, camera, `/cmd_vel` → Pi, health metrics |
| `x500_navigation` | Nav2 settings, launch files, RViz setup |
| `quadcopter_perception` | YOLO + SAM segmentation |
| `x500_px4_bridge`, `quadcopter_bringup` | Simulation only (PX4) |
| `quadcopter_localization`, `quadcopter_mapping` | Early work |
| `m-explore-ros2`, `px4_msgs` | Third-party |

## Test

```bash
colcon test --packages-select quadcopter_perception quadcopter_bringup && colcon test-result --verbose

# segmentation logic (no GPU needed)
cd ros2_ws/src/quadcopter_perception && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_hybrid_segmenter.py
```

No drone? `python3 raspberry_pi/fake_pi.py` sends fake telemetry and lidar to the laptop stack.

## Rules

- Style: PEP 8, max line length 99 (checked by the package tests).
- No hard-coded IPs, ports or paths in nodes: use parameters.
- ROS frames: ENU world, FLU body. Convert from the Pixhawk's NED/FRD in the bridge.
- New node: add it to the package, register it in `setup.py` (`console_scripts`), put launch
  files in `launch/`, then build and test.
- Git: one branch per feature, commit messages like `feat: …` / `fix: …`. Never commit
  `build/`, `install/`, `log/` or model files (`*.pt`).
