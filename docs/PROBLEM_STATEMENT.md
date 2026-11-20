# Problem Statement

## The problem

Drones normally know where they are from **GPS** (or GLONASS, Galileo, BeiDou) plus an **IMU**
(accelerometers and gyros). GPS is not always available:

- **Blocked:** underground mines, tunnels, inside buildings, between tall buildings, under trees.
- **Attacked:** jamming, spoofing and radio interference in military areas.

Without GPS, the drone can only add up its IMU readings ("dead reckoning"). Small sensor errors
add up fast: the position error grows with the **square of time**. Within seconds to minutes
the drone drifts, loses control or hits something.

Single-sensor fixes each have a weak spot:

| Approach | Fails when |
| :--- | :--- |
| Cameras (visual odometry / SLAM) | It's dark, smoky, blurry, or the walls are plain |
| LiDAR | Weight, power and cost are tight; heavy dust, rain or fog |
| Radio beacons | You can't install beacons first, or must stay radio-silent |

**Goal:** a navigation system that runs **fully on the drone**, in real time, keeps its position
error small for **30+ minutes**, and needs **no satellites and no pre-installed equipment**.

## In math terms

The drone's state at time *t* is position **p**, velocity **v**, orientation **q** (quaternion),
and the accelerometer and gyro biases **b**<sub>a</sub>, **b**<sub>g</sub>:

$$\mathbf{x}_t = [\mathbf{p}_t,\ \mathbf{v}_t,\ \mathbf{q}_t,\ \mathbf{b}_a,\ \mathbf{b}_g]$$

We want the most likely path of states given the IMU/control inputs **u** and the sensor
measurements **z** (camera features, LiDAR, rangefinder):

$$\mathbf{x}_{0:T}^* = \arg\max_{\mathbf{x}_{0:T}} \; p(\mathbf{x}_0) \prod_{t=1}^{T} p(\mathbf{x}_t \mid \mathbf{x}_{t-1}, \mathbf{u}_t) \prod_{k} p(\mathbf{z}_{t,k} \mid \mathbf{x}_t)$$

This is solved with an EKF or factor-graph optimisation.

## Targets (KPIs)

| Metric | Target | Typical fallback today |
| :--- | :--- | :--- |
| Trajectory error | < 0.5% of distance flown | > 10% |
| Hover drift | ≤ ±5 cm over 10 min | ≥ 2 m per minute |
| Update rate | State ≥ 100 Hz, vision ≥ 30 Hz | Dropped frames break the filter |
| Compute power | ≤ 15 W, onboard | Needs off-board computing |
| Lighting | 0 to 10,000+ lux | Fails below 50 lux |

## How it fits together

```
Sensors (IMU, cameras, LiDAR, ToF)
   → feature extraction
   → state estimator (EKF / factor graph)
   → map + loop closure (SLAM)
   → path planner → flight controller (PX4 / ArduPilot)
```

## Scope

**In:** fusing several sensors (visual-inertial odometry, LiDAR SLAM, optical flow), onboard
computing, flight control over MAVLink / ROS 2 with PX4 or ArduPilot.

**Out:** off-board processing on a ground station, radio beacons (UWB, Wi-Fi, cellular) in
unknown hostile areas, constant manual piloting.

> The current prototype streams to a laptop for processing. That's a development step; the
> target is still fully onboard.
