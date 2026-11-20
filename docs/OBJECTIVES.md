# Objectives

**Main goal:** a drone that finds its position, navigates and avoids obstacles without GPS,
without pre-installed equipment, and with all processing onboard.

To get there, the system must:

1. **Avoid obstacles** (still and moving) in cluttered, unknown places: factories, tunnels, mines, cities.
2. **Map as it flies** (SLAM), and fix drift when it returns to a known place (loop closure).
3. **Combine sensors** (stereo camera, IMU, optional optical flow / rangefinder) and keep working
   when one fails, such as a blocked camera or a saturated IMU.
4. **Run in real time on the drone** (Raspberry Pi 5) within a small power and heat budget.
5. **Work in any light**: dark, smoke, dust, plain walls, glare, motion blur.
6. **Resist jamming and spoofing**, and prefer sensors that don't transmit (stay hard to detect).
7. **Hover and move precisely**, including takeoff and landing, without GPS.
8. **Fly 30+ minutes** within the battery and payload limits.
9. **Stay safe**: the pilot can always take over (ELRS radio); failsafes for lost link, low
   battery and sensor failure land or return the drone without GPS.
10. **Fit the budget**: under 500 g of payload and about ₹45.6k of hardware, easy to reproduce.
11. **Follow plain-language commands** through an LLM, e.g. "go to the window": find that target
    in the map, plan a route and fly there safely.
