# Setup Instructions

This guide helps a new user set up the PX4 + ROS 2 simulation bridge used by this project.

## What You Will Set Up

- Micro XRCE-DDS Agent for PX4-to-ROS 2 communication
- PX4 SITL simulation with Gazebo
- A ROS 2 workspace containing `px4_msgs` and `px4_ros_com`

## Prerequisites

Use Ubuntu 22.04 with ROS 2 Humble.

Install the basic build tools if they are not already installed:

```bash
sudo apt update
sudo apt install -y git cmake build-essential
```

You should also already have:

- PX4 Autopilot cloned and set up
- ROS 2 Humble installed
- `colcon` installed

If `colcon` is missing:

```bash
sudo apt install -y python3-colcon-common-extensions
```

## 1. Install Micro XRCE-DDS Agent

Open a terminal and run:

```bash
git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

Verify that the agent command is available:

```bash
MicroXRCEAgent --help
```

## 2. Start Micro XRCE-DDS Agent

Keep this terminal open:

```bash
MicroXRCEAgent udp4 -p 8888
```

The agent waits for PX4 to connect over UDP port `8888`.

## 3. Start PX4 SITL

Open a new terminal in the root of your PX4 Autopilot repository:

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

When PX4 connects to the agent, the PX4 terminal should show messages like:

```text
INFO  [uxrce_dds_client] synchronized with time offset ...
INFO  [uxrce_dds_client] successfully created rt/fmu/out/failsafe_flags data writer
INFO  [uxrce_dds_client] successfully created rt/fmu/out/sensor_combined data writer
INFO  [uxrce_dds_client] successfully created rt/fmu/out/timesync_status data writer
```

If these messages appear, PX4 and the Micro XRCE-DDS Agent are communicating correctly.

## 4. Create the ROS 2 Workspace

Open a new terminal:

```bash
mkdir -p ~/ws_sensor_combined/src
cd ~/ws_sensor_combined/src
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/PX4/px4_ros_com.git
```

Build the workspace:

```bash
cd ~/ws_sensor_combined
source /opt/ros/humble/setup.bash
colcon build
```

Source the workspace:

```bash
source install/setup.bash
```

Optional: add the ROS 2 and workspace setup commands to your shell startup file:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ws_sensor_combined/install/setup.bash" >> ~/.bashrc
```

## 5. Verify ROS 2 Topics

With these still running:

- Terminal 1: `MicroXRCEAgent udp4 -p 8888`
- Terminal 2: `make px4_sitl gz_x500`

Open another terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/setup.bash
ros2 topic list
```

You should see PX4 topics such as:

```text
/fmu/out/sensor_combined
/fmu/out/timesync_status
/fmu/out/vehicle_status
```

To inspect sensor data:

```bash
ros2 topic echo /fmu/out/sensor_combined
```

## 6. Recommended Terminal Layout

Use three terminals while developing:

| Terminal | Purpose | Command |
| :--- | :--- | :--- |
| 1 | Micro XRCE-DDS Agent | `MicroXRCEAgent udp4 -p 8888` |
| 2 | PX4 SITL | `make px4_sitl gz_x500` |
| 3 | ROS 2 tools/nodes | `source ~/ws_sensor_combined/install/setup.bash` |

## Troubleshooting

### `MicroXRCEAgent: command not found`

The agent was not installed or your shell cannot find it. Re-run:

```bash
cd Micro-XRCE-DDS-Agent/build
sudo make install
sudo ldconfig /usr/local/lib/
```

### PX4 does not create ROS 2 topics

Check that the agent is running before starting PX4:

```bash
MicroXRCEAgent udp4 -p 8888
```

Then restart PX4 SITL:

```bash
make px4_sitl gz_x500
```

### `ros2 topic list` does not show `/fmu/...` topics

Make sure the ROS 2 environment is sourced:

```bash
source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/setup.bash
```

Also confirm that PX4 printed `uxrce_dds_client` connection messages.

### `colcon build` fails

Make sure ROS 2 Humble is sourced before building:

```bash
source /opt/ros/humble/setup.bash
cd ~/ws_sensor_combined
colcon build
```

## Next Step

After setup is complete, use the ROS 2 workspace to run PX4 example nodes from `px4_ros_com` or add project-specific nodes for GPS-denied navigation experiments.
