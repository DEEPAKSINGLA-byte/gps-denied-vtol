## Drone Hardware Budget

| # | Component | Price (INR) | Link |
|---|---|---:|---|
| 1 | SC132GS Binocular Stereo Camera | ₹6,061.66 | [Link](https://share.google/AHIbHjg7k9iasOi3g) |
| 2 | Raspberry Pi 5 — 8GB | ₹17,500 | [Link](https://quartzcomponents.com/products/raspberry-pi-5-model-b-8-gb-ram) |
| 3 | Holybro Kakute H7 V1.3 | ₹10,217 | [Link](https://robu.in/product/holybro-kakute-h7-v1.5-flight-controller/) |
| 4 | RadioMaster Pocket ELRS | ₹7,000 | [Link](https://share.google/fzrNBatO92qi0966L) |
| 5 | RadioMaster RP1 ELRS Nano | ₹2,600 | [Link](https://share.google/8WYlHOsC5xzn7iG34) |
| 6 | Tattu R-Line 850mAh 14.8V 150C | ₹2,300 | [Link](https://share.google/xTDTzNLacnPPxd84K) |
| | **Total** | **₹45,678.66** | |

## Why These Components?

###  SC132GS Stereo Camera
- Visual odometry for better localization
- Depth information for obstacle avoidance
- Point cloud generation
- ArUco marker detection

###  Raspberry Pi 5 8GB
- Processes the two stereo camera images
- Handles depth and point cloud generation
- Runs obstacle avoidance and costmap processing
- Handles ArUco and visual-odometry processing

###  Holybro Kakute H7
- Main flight controller
- STM32H7 provides enough processing power for PX4
- Handles real-time flight stabilization and control

###  RadioMaster Pocket ELRS
- Manual pilot control
- ELRS provides very low latency and long-range communication
- Provides a manual override during autonomous flight

###  RadioMaster RP1 ELRS Nano
- Receives commands from the RadioMaster Pocket
- Sends pilot commands to the flight controller

###  Tattu R-Line 850mAh 14.8V 150C
- High **150C discharge rating** for high-current demands
- Small 850mAh capacity keeps the battery lightweight
- Expected flight time: **~10 minutes**, depending on drone weight and power consumption

## System Overview

```text
Stereo Camera
     ↓
Raspberry Pi 5
(Perception + Navigation)
     ↓
Kakute H7
(Flight Control)
     ↓
Motors / ESCs

RadioMaster Pocket
     ↓ ELRS
RP1 Receiver
     ↓
Kakute H7