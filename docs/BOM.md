# Parts List (design target)

| # | Part | Why | Price (INR) | Link |
| :--- | :--- | :--- | ---: | :--- |
| 1 | SC132GS stereo camera | Visual odometry, depth for obstacle avoidance, point clouds, ArUco tags | ₹6,061.66 | [link](https://share.google/AHIbHjg7k9iasOi3g) |
| 2 | Raspberry Pi 5, 8 GB | Processes the stereo images: depth, costmaps, obstacle avoidance, visual odometry | ₹17,500 | [link](https://quartzcomponents.com/products/raspberry-pi-5-model-b-8-gb-ram) |
| 3 | Holybro Kakute H7 V1.3 | Flight controller; STM32H7 runs PX4 stabilisation | ₹10,217 | [link](https://robu.in/product/holybro-kakute-h7-v1.5-flight-controller/) |
| 4 | RadioMaster Pocket ELRS | Pilot's transmitter; low-latency manual override | ₹7,000 | [link](https://share.google/fzrNBatO92qi0966L) |
| 5 | RadioMaster RP1 ELRS Nano | Receiver on the drone, passes pilot commands to the flight controller | ₹2,600 | [link](https://share.google/8WYlHOsC5xzn7iG34) |
| 6 | Tattu R-Line 850 mAh 4S 150C | Light battery with high current; about 10 min of flight | ₹2,300 | [link](https://share.google/xTDTzNLacnPPxd84K) |
| | **Total** | | **₹45,678.66** | |

```
Stereo camera → Raspberry Pi 5 (perception + navigation) → Kakute H7 (flight control) → ESCs / motors
Pocket transmitter → ELRS → RP1 receiver → Kakute H7
```

> The current prototype uses a Pixhawk (ArduPilot), a YDLIDAR and a USB camera instead; see the
> [README](../README.md#how-the-current-system-works).
