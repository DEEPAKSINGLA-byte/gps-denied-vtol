# Comparing the 15 Solutions

How the options in [SOLUTIONS.md](SOLUTIONS.md) compare on accuracy, weight, power, speed,
range and robustness. Charts of this data: [RESEARCH.md](RESEARCH.md#charts).

## 1. The numbers

- **Drift:** position error as a % of distance flown (lower is better).
- **Range:** how far it keeps working (∞ = no hard limit).

| Solution | Drift | Power | Weight | Update rate | Range |
| :--- | :--- | :--- | :--- | :--- | :--- |
| 1. 2D LiDAR + optical flow | 0.8–2.0% | 4–10 W | 100–200 g | 50–100 Hz | < 15 m (room) |
| 2. Stereo camera visual SLAM (Pi 5) | 0.1–0.5% | 6–15 W | 50–120 g | 20–60 Hz | ~1 km (memory) |
| 3. 2D LiDAR + RGB camera | 1.0–2.0% | 5–11 W | 100–180 g | 20–50 Hz | < 15 m (room) |
| 4. Event camera | 0.8–2.0% | 3–8 W | 25–60 g | 500–1000 Hz | ∞ |
| 5. Visual SLAM | 0.1–0.5% | 10–25 W | 50–120 g | 20–60 Hz | ~1 km (memory) |
| 6. Ultrasonic array | 1.0–4.0% | 0.5–2 W | 15–40 g | 20–40 Hz | < 10 m |
| 7. Optical flow + rangefinder | 2.0–5.0% | 1–3 W | 15–35 g | 50–100 Hz | < 20 m (height) |
| 8. ArUco markers | ±2 cm (absolute) | 1–3 W | 10–30 g | 10–30 Hz | Marker area |
| 9. Three cameras (Pi 5) | 0.3–1.0% | 8–18 W | 60–150 g | 15–30 Hz | ∞ |
| 10. Depth camera (Pi 5) | 0.5–1.5% | 5–15 W | 40–100 g | 15–30 Hz | < 5 m |
| 11. 3D LiDAR SLAM (Pi 5) | 0.05–0.2% | 15–30 W | 250–500 g | 10–20 Hz | ~5 km (compute) |
| 12. mmWave radar | 1.0–3.0% | 4–10 W | 60–180 g | 20–50 Hz | ∞ |
| 13. UWB beacons | ±2 cm (absolute) | 1–3 W | 10–30 g | 50–100 Hz | < 200 m (beacons) |
| 14. MAGNAV | 0.2–1.0% | 2–5 W | 40–100 g | 10–30 Hz | ∞ (map) |
| 15. TERCOM | 0.1–0.8% | 5–15 W | 100–350 g | 5–20 Hz | ∞ (map) |

## 2. Three groups

```
 More accurate ^   [3D LiDAR SLAM]  [UWB]  [ArUco]
               |   [Visual SLAM / stereo]  [TERCOM]
               |   [Depth cam]  [mmWave]  [Event cam]  [MAGNAV]
               |   [Optical flow]  [Ultrasonic]  [2D LiDAR combos]
 Less accurate +-------------------------------------------->
                   Light / low power             Heavy / high power
```

| Group | Examples | Typical numbers | Best for |
| :--- | :--- | :--- | :--- |
| **A. Light and cheap** | Optical flow, UWB, ArUco, ultrasonic, event camera | < 60 g, < 8 W, > 30 Hz | Small indoor drones |
| **B. Balanced** | Visual SLAM, stereo / three cameras, 2D LiDAR + flow, depth camera, mmWave, MAGNAV | 50–200 g, 5–25 W, 0.1–2% drift | Standard multirotors, mid-size fixed wing |
| **C. Heavy and most reliable** | 3D LiDAR SLAM, TERCOM | 250–500 g, 15–30 W, < 0.1% drift | Inspection, heavy-lift, high altitude |

## 3. Robustness

Score 1 (fails) to 5 (unaffected).

| Solution | Darkness | Smoke / dust | Glare / moving things | Jamming | Unknown area |
| :--- | :---: | :---: | :---: | :---: | :---: |
| 2D LiDAR + optical flow | 5 | 2 | 5 | 5 | 5 |
| Stereo / three cameras | 1 | 1 | 2 | 5 | 5 |
| 2D LiDAR + RGB camera | 5 | 2 | 3 | 5 | 5 |
| Event camera | 2 | 1 | 5 | 5 | 5 |
| Visual SLAM | 1 | 1 | 2 | 5 | 5 |
| Ultrasonic array | 5 | 3 | 5 | 5 | 2 |
| Optical flow + rangefinder | 2 | 1 | 3 | 5 | 5 |
| ArUco markers | 2 | 2 | 3 | 5 | 1 |
| Depth camera | 2 | 1 | 3 | 5 | 5 |
| 3D LiDAR SLAM | 5 | 2 | 5 | 5 | 5 |
| mmWave radar | 5 | 5 | 5 | 5 | 5 |
| UWB / radio beacons | 5 | 5 | 5 | 1 | 1 |
| MAGNAV / TERCOM | 5 | 5 | 5 | 5 | 2 |

## 4. Best choice per mission

Each solution gets a 1–10 rating per criterion. The score is the weighted sum: score = Σ (weight × rating).

| Mission | Weights | 1st | 2nd | 3rd |
| :--- | :--- | :--- | :--- | :--- |
| **Defense / jammed airspace** | Jamming 30%, dark/smoke 25%, size/weight/power 25%, precision 20% | mmWave radar (8.75) | 3D LiDAR SLAM (8.30) | Event camera (7.95) |
| **Underground inspection** | Darkness 35%, precision 25%, size/weight/power 20%, unknown area 20% | 3D LiDAR SLAM (9.10) | mmWave radar (8.40) | Depth camera with IR (7.80) |
| **Tiny drone (< 250 g)** | Weight 40%, power 30%, update rate 15%, precision 15% | Optical flow + rangefinder (9.15) | UWB beacons (8.90) | ArUco markers (8.35) |
