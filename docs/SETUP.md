# Setup (do once)

After this, use [QUICKSTART.md](QUICKSTART.md) to fly.

## 1. Laptop

You need Ubuntu 22.04, an NVIDIA GPU and [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html).

```bash
sudo apt install -y python3-colcon-common-extensions python3-rosdep
sudo rosdep init; rosdep update

git clone https://github.com/DEEPAKSINGLA-byte/gps-denied-vtol.git ~/gps-denied-vtol
cd ~/gps-denied-vtol/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y    # Nav2 and other ROS packages
colcon build --symlink-install

pip install -U ultralytics    # YOLO + SAM; model files download on first run
```

Add to `~/.bashrc`:

```bash
source /opt/ros/humble/setup.bash
source ~/gps-denied-vtol/ros2_ws/install/setup.bash
export GPS_DENIED_PI_IP=10.184.3.36    # your Pi's IP
```

Turn off Wi-Fi power saving (it delays incoming data):

```bash
printf '[connection]\nwifi.powersave = 2\n' | sudo tee /etc/NetworkManager/conf.d/wifi-powersave-off.conf
sudo systemctl restart NetworkManager
```

Serve time to the Pi, so both clocks agree and the latency numbers are real:

```bash
sudo ~/gps-denied-vtol/scripts/setup_laptop_timeserver.sh   # installs chrony, allows the local network, opens UDP 123
```

**Tip:** connect the Pi and laptop directly (e.g. the laptop hosts a 5 GHz hotspot). Going
through a phone hotspot sends every packet over the air twice.

Optional: PlotJuggler (`sudo apt install ros-humble-plotjuggler-ros`), QGroundControl (to watch the Pixhawk).

## 2. Raspberry Pi

The Pi only needs the `raspberry_pi/` folder (no ROS). It lives in `~/raspberry_pi`.

**Copy the code** from the laptop (repeat after every change):

```bash
rsync -av --exclude '*.pt' --exclude '__pycache__' \
  ~/gps-denied-vtol/raspberry_pi/ deimos@10.184.3.36:~/raspberry_pi/
```

**Install** (on the Pi):

```bash
cd ~/raspberry_pi
sudo apt install -y python3-opencv python3-numpy
pip install -r requirements.txt --break-system-packages
sudo ./setup_pi.sh       # then log out and back in once
```

`setup_pi.sh` installs the udev rules (`/dev/pixhawk`, `/dev/camera`), serial access, `chrony`
and `iw`, turns Wi-Fi power saving off for good, and lets `start_streams.py` run exactly these
commands without a password: `iw … set power_save off` and `chronyc add server / delete /
makestep` (to sync the Pi clock to the laptop at every start).

If the Pi has no internet for `apt` (e.g. a network login page), download the package on the
laptop and copy it over: `apt download chrony` won't work across architectures, so fetch the
`arm64` `.deb` for your Debian release from `deb.debian.org` and install it with `sudo dpkg -i`.

**Check:**

```bash
ls -la /dev/pixhawk /dev/camera /dev/ttyUSB0    # Pixhawk, camera, lidar
python3 test_connection.py                      # should end with [PASSED]
```

Connect the Pixhawk by **USB** (simplest). For a UART cable (`/dev/serial0`), the baud rate must
match the Pixhawk's `SERIALx_BAUD`. `start_streams.py` assumes 115200; for other rates run
`telemetry_streamer.py --port /dev/serial0 --baud <rate>`. For other Pixhawk or camera models,
add their USB IDs to `99-pixhawk.rules` / `98-camera.rules`.

## 3. ArduPilot settings

Set these with Mission Planner or QGroundControl:

| Setting | Value | Why |
| :--- | :--- | :--- |
| `FLTMODEx` | put `GUIDED` (4) on one RC switch position | The laptop can only steer in GUIDED |
| `EK3_SRC1_POSXY` | `0` (none) or `4` (optical flow) | No GPS indoors |
| `EK3_SRC1_POSZ` | `1` (barometer) or `2` (rangefinder) | Height source |
| `AHRS_GPS_USE` | `0` | Don't use GPS |
| `ARMING_CHECK` | `1048558` (skips GPS/compass checks) | Otherwise `PreArm: Need 3D Fix` blocks arming |

On the bench (props off) you can also force-arm: `python3 arm_disarm.py --arm --force`.

Which mode to fly: start with `ALT_HOLD` (holds height, you steer). `POSHOLD` needs optical flow.
`GUIDED` is for the laptop. `LAND` lands.

## 4. Sensor positions

`ros2_ws/src/x500_navigation/launch/navigation_real.launch.py` says where the lidar and camera
sit on the drone. The values there are **guesses**: measure yours, edit them, run `colcon build`.
If the lidar angle is wrong, `/metrics/costmap/local/scan_agreement` drops well below 1.
