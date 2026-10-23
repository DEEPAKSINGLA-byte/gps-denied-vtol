# Hardware demo quick start

Use this guide when the real camera and LiDAR are connected to your laptop. The laptop runs ROS 2, the sensor drivers, segmentation, SLAM, and RViz. PX4 SITL and Gazebo are not part of this hardware demo.

The repository does not contain a physical SC132GS camera driver or a driver for a named LiDAR model. Install and configure the drivers supplied for your exact sensors first. Driver launch commands depend on the model and connection type; see [DEMO.md](DEMO.md) for the full presentation checklist and live sequence.



## 0. Start the lidar
'''
cd ~/ydlidar_ws
source install/setup.bash
ros2 launch ydlidar_ros2_driver ydlidar_launch.py
'''
## 1. Prepare the laptop and sensors

- Ubuntu 22.04 with ROS 2 Humble, `cv_bridge`, RViz, and the sensor-driver dependencies installed.
- Connect and power the camera and LiDAR as their manufacturers specify. Some LiDARs need a separate power supply; do not assume laptop USB can power them.
- Install the manufacturer/ROS drivers so the camera publishes `sensor_msgs/Image` and the 2D LiDAR publishes `sensor_msgs/LaserScan`.
- Confirm the devices are visible to the laptop: `lsusb`, `v4l2-ctl --list-devices` for UVC cameras, and `ls -l /dev/serial/by-id/` for USB-serial LiDARs.
- Install Ultralytics in the Python environment used by ROS and download `yolov8n-seg.pt` before presenting.
- Use a stable table or hand-carried planar scan for mapping. A 2D SLAM demo assumes the LiDAR stays approximately level.

## 2. Build the workspace

```bash
source /opt/ros/humble/setup.bash
cd software/src
colcon build --packages-select quadcopter_perception quadcopter_navigation --symlink-install
source install/setup.bash
```

Source ROS and `software/src/install/setup.bash` in every terminal. Start the physical sensor drivers first; the hardware launch does not start drivers or create sensor transforms.

## 3. Start and verify sensor drivers

Start the camera and LiDAR drivers using the commands from their driver documentation. Keep the drivers on the laptop and check what they publish:

```bash
ros2 topic list
ros2 topic info /scan
ros2 topic info /camera/image_raw
```

The expected demo topics are `/scan` (`sensor_msgs/msg/LaserScan`) and `/camera/image_raw` (`sensor_msgs/msg/Image`). If the driver uses different topic names, pass those names to the launch commands below. Check rates with `ros2 topic hz /scan` and `ros2 topic hz /camera/image_raw`.

View the raw camera stream before inference:

```bash
ros2 run rqt_image_view rqt_image_view
```

Select the camera image topic in the viewer. Verify the lidar frame and transform tree with `ros2 topic echo /scan --once` and `ros2 run tf2_tools view_frames`.

## 4. Show live camera segmentation

After both sensor drivers are publishing, start camera segmentation and LiDAR SLAM together:

```bash
ros2 launch quadcopter_navigation hardware_sensor_demo.launch.py camera_image_topic:=/camera/image_raw segmentation_model:=yolov8n-seg.pt
```

View `/quadcopter_segmentation/annotated` and `/quadcopter_segmentation/class_mask` in `rqt_image_view`. Show object names, confidences, and image-space boxes with:

```bash
ros2 topic echo /quadcopter_segmentation/detections
```

For the SC132GS stereo camera, use one rectified or raw image stream for this 2D segmentation demo. This node does not perform stereo depth, and its masks are not fused with the LiDAR map.

## 5. Build and view a LiDAR map

SLAM Toolbox needs `/scan`, a valid transform from `base_link` to the LiDAR frame, and an `odom` → `base_link` transform. The hardware launch uses wall-clock time and the config in `software/src/quadcopter_navigation/config/slam_toolbox_hardware.yaml`. Edit `scan_topic`, `base_frame`, and `odom_frame` there if your driver/rig uses different names, or pass a custom YAML through `slam_params_file:=/path/to/params.yaml`. The Gazebo odometry publisher in `quadcopter_navigation/bridge_and_tf.launch.py` is for simulation and must not be used for real sensors.

Open RViz in another terminal, set the fixed frame to `map`, and add `Map`, `LaserScan`, and `TF` displays. Slowly move the level LiDAR around a small area and observe the occupancy map. Configure the driver to publish `/scan`, or set `scan_topic` in the SLAM YAML to its actual topic. Do not guess frame names or transforms.

This demo shows live sensor acquisition, 2D mapping, and camera segmentation as separate outputs. It does not demonstrate an autonomous flying drone or camera-to-LiDAR semantic fusion.

## If a physical sensor is unavailable

The prepared-image publisher is only a camera fallback; it is not live sensor input:

```bash
python3 demo/publish_demo_image.py /path/to/demo-image.jpg
```

Do not mix simulated odometry or sensor topics with real LiDAR measurements.
