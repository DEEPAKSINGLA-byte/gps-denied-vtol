# Hardware sensor demo plan

## Purpose

Demonstrate the real camera and LiDAR connected to the laptop: show the live camera stream, run object segmentation on camera frames, build a 2D occupancy map from live LiDAR scans, and inspect both in ROS 2/RViz.

This is a sensor/software demonstration. It does not require PX4, Gazebo, or a flying vehicle. Camera segmentation and LiDAR SLAM are separate pipelines here; the segmentation masks are not fused into the LiDAR map.

## Important driver note

The repo includes a ROS segmentation node and Gazebo LiDAR simulation support, but it does not include a physical SC132GS camera driver or a driver for a named LiDAR model. Use the vendors' ROS 2 drivers or SDK wrappers to publish ROS messages. The exact driver commands depend on the camera interface and LiDAR make/model.

The live ROS interfaces expected by this demo are:

| Sensor | Expected ROS data | Default topic |
|---|---|---|
| Camera | `sensor_msgs/msg/Image` | `/camera/image_raw` |
| 2D LiDAR | `sensor_msgs/msg/LaserScan` | `/scan` |

A 3D LiDAR that publishes `PointCloud2` needs a different mapping pipeline; the SLAM setup in this repo is for 2D `LaserScan` data.

## 8–10 minute presentation

1. **Show the physical setup (1 min).** Point out the camera and LiDAR connected and powered on the laptop. Explain that the laptop is running the ROS 2 drivers and processing nodes.
2. **Prove live sensor data (1 min).** Show the camera image in `rqt_image_view`, a live `/scan` rate, and the ROS topic list.
3. **Show camera segmentation (2 min).** Run the ROS YOLO node. Show its annotated image and class mask, then echo the detection messages with labels, confidence, and image-space boxes.
4. **Build a LiDAR map (3–4 min).** Start SLAM Toolbox with real-time clock, show the scan and map in RViz, and slowly move the level LiDAR around a small room or tabletop area. Keep the scan path short and avoid fast turns.
5. **Explain the result and current boundary (1 min).** The map comes from LiDAR; segmentation comes from the camera. Explain that camera-to-LiDAR calibration and semantic map fusion are future work. This demo does not command a drone or validate flight.

## Prepare before presenting

- Laptop running Ubuntu 22.04 and ROS 2 Humble, with `cv_bridge`, RViz, `rqt_image_view`, and `slam_toolbox` available.
- Camera driver installed and publishing `sensor_msgs/Image`. For the SC132GS, determine whether your module/adapter exposes a UVC stream or requires the vendor SDK/driver. The segmentation node can consume one camera view; it does not calculate stereo depth.
- LiDAR driver installed and publishing `sensor_msgs/LaserScan`. Confirm it is a 2D scanner. Check its power requirements and connect the required supply.
- Camera and LiDAR have distinct, known TF frame IDs. A transform from the SLAM base frame to the laser frame is required. For a moving scan, provide a valid `odom` → base transform or a scan-matching setup appropriate to your rig.
- Ultralytics, OpenCV, and NumPy installed in the Python environment used by ROS. Download `yolov8n-seg.pt` and run a warm-up before the demo so no internet download is needed live.
- Select a small, level area with visible walls/edges and enough room to move the LiDAR slowly. Keep the camera pointed at objects YOLO can recognize (for example, people, chairs, bottles, or bags).
- Have a prerecorded screen capture and one camera image ready as offline fallbacks.

## Build the added ROS packages

From the repository root:

```bash
source /opt/ros/humble/setup.bash
cd software/src
colcon build --packages-select quadcopter_perception quadcopter_navigation --symlink-install
source install/setup.bash
```

Source `/opt/ros/humble/setup.bash` and `software/src/install/setup.bash` in every ROS terminal. Do not start the Gazebo bridge launch for this real-sensor demo; it is hard-coded to the simulated floorplan model.

## Live run steps

### 1. Connect and start the sensor drivers

Start the real camera and LiDAR drivers using their own documented launch commands. Those commands are model-specific and are not supplied by this repository. Then verify the streams:

```bash
ros2 topic list
ros2 topic info /camera/image_raw
ros2 topic hz /camera/image_raw
ros2 topic info /scan
ros2 topic hz /scan
```

If your sensor topics have different names, use the actual names in the following steps.

Check the camera feed:

```bash
ros2 run rqt_image_view rqt_image_view
```

Check the LiDAR message and TF frames:

```bash
ros2 topic echo /scan --once
ros2 run tf2_tools view_frames
```

Before SLAM, confirm TF connects the laser frame to the base frame. Use `ros2 run tf2_ros tf2_echo base_link <laser_frame>` with your real frame names. If the base frame is not `base_link`, configure it consistently in the SLAM parameters.

### 2. Run camera segmentation

Start camera segmentation and LiDAR SLAM together after launching both real sensor drivers:

```bash
ros2 launch quadcopter_navigation hardware_sensor_demo.launch.py camera_image_topic:=/camera/image_raw segmentation_model:=yolov8n-seg.pt
```

In `rqt_image_view`, display:

- `/quadcopter_segmentation/annotated`
- `/quadcopter_segmentation/class_mask`

Show detected classes and confidence scores:

```bash
ros2 topic echo /quadcopter_segmentation/detections
```

If using a stereo camera, select one image stream (left or rectified-left). This node publishes 2D masks; it does not consume camera calibration or depth.

### 3. Run LiDAR mapping

The launch uses [slam_toolbox_hardware.yaml](../software/src/quadcopter_navigation/config/slam_toolbox_hardware.yaml), which defaults to `/scan`, `base_link`, `odom`, and `map`, with wall-clock time. Set `scan_topic`, `base_frame`, and `odom_frame` to match the real driver and rig. Use the actual frame IDs; do not guess them.

Open RViz and set the fixed frame to `map`. Add `Map`, `LaserScan`, and `TF` displays. Move the LiDAR slowly while keeping the scan plane level; pause at corners so scans overlap. The occupancy grid should grow as the sensor sees new areas.

SLAM requires a valid `odom` → base transform and base → laser transform. If the sensor is handheld and there is no odometry source, confirm that the chosen scan-matching setup supports this arrangement before presenting. The repo's `odom_tf_publisher` is wired to Gazebo odometry and is not a real-sensor odometry source.

### 4. Optional still-image fallback

If the real camera driver fails, feed a prepared image into the ROS segmentation node from another sourced terminal:

```bash
python3 demo/publish_demo_image.py /path/to/demo-image.jpg
```

This fallback only replaces the camera stream. It does not simulate LiDAR or create a map. The standalone webcam preview is also available:

```bash
python3 demo/camera_segmentation.py
```

## Troubleshooting checklist

- **No camera topic:** check the driver logs, USB permissions, camera power, and whether the module needs a vendor SDK rather than UVC.
- **No `/scan`:** check the LiDAR power supply, USB/serial device path and permissions, and the driver's configured port and baud rate.
- **Segmentation receives frames but shows no masks:** aim at common objects, check that model weights loaded, and verify image encoding with `ros2 topic info`/driver settings.
- **No map or TF warnings:** check the LaserScan `header.frame_id`, base-to-laser TF, scan topic, `use_sim_time:=false`, and any odometry/scan-matching requirements.
- **Map smears or jumps:** move more slowly, keep the scanner level, improve overlap between views, and check that transforms use consistent frame conventions.
- **Performance is poor:** lower the segmentation node's `max_rate_hz` parameter or reduce the camera resolution in its driver.

## What to claim

You can claim the demo shows physical camera and LiDAR data entering ROS 2, YOLO image segmentation, and a LiDAR occupancy map if those functions are running live. Do not claim the camera masks are fused into the map, that Nav2 is controlling a drone, or that this validates GPS-denied flight.
