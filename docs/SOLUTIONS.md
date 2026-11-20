# 15 Ways to Navigate Without GPS

The options we considered. They are compared with numbers in [ANALYSIS_SOLUTIONS.md](ANALYSIS_SOLUTIONS.md).

| # | Solution | How it works | Good | Watch out |
| :--- | :--- | :--- | :--- | :--- |
| 1 | 2D LiDAR + optical flow | LiDAR maps the room; a downward flow sensor measures speed over the floor | Flow sensor wires straight to the flight controller for fast, stable control | Room-scale only |
| 2 | Stereo camera + Pi 5 (visual SLAM) | Two cameras build a 3D point cloud of the surroundings | No lasers; light and cheap | Drifts in plain, textureless corridors |
| 3 | 2D LiDAR + one RGB camera | LiDAR does mapping and avoidance; the camera is free for vision tasks | Easy to add OpenCV jobs (colour tracking, landing pads) | Room-scale only |
| 4 | Event camera | Pixels report only when brightness changes, like the eye | No motion blur, very low latency | Needs special algorithms; normal OpenCV won't work |
| 5 | Visual SLAM | Remembers visual landmarks to build a map and locate itself | Corrects drift when it sees a known place again (loop closure) | Needs light and texture |
| 6 | Ultrasonic array | Sends sound pulses and times the echoes, like a bat | Cheap; works in total darkness | Only a few metres of range |
| 7 | Optical flow + rangefinder | A down-facing camera sees sideways motion; a laser measures height | Very little compute; great for steady hovering | No map; drift over distance |
| 8 | ArUco markers | Recognises printed square tags placed in the area | Very accurate and simple to code | Tags must be put up first |
| 9 | Three cameras + Pi 5 | Combines three cameras into a wide panoramic view | Rarely loses track of features in tight spaces | More weight, power and compute |
| 10 | Depth camera + Pi 5 | Projects an infrared pattern to measure distance directly | Ready-made 3D point cloud; low CPU load | Short range (< 5 m) |
| 11 | 3D LiDAR + Pi 5 | Many spinning laser beams scan 360° in 3D | Very accurate; works in darkness and smoke | Heavy, power-hungry, complex wiring |
| 12 | mmWave radar | Radio waves reveal speed and distance from frequency shifts | Sees through smoke and dust | Lower precision; industrial-grade hardware |
| 13 | UWB beacons | Times radio pings to beacons placed around the room, like indoor GPS | About ±2 cm accuracy, no vision needed | Beacons must be installed; can be jammed |
| 14 | Magnetic map (MAGNAV) | Compares the local magnetic field with a stored magnetic map | Passive and unjammable | Motors and wiring disturb the readings; needs a map |
| 15 | Terrain matching (TERCOM) | Measures the ground height below and matches it to a stored 3D map | Works over long distances | For high, long flights, not indoors |
