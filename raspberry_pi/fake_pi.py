#!/usr/bin/env python3
"""
fake_pi.py - bench-test the LAPTOP stack (Nav2, costmaps, RViz, metrics) with no drone.

Impersonates the Raspberry Pi by sending the same UDP packets start_streams.py would:
  * telemetry JSON -> 127.0.0.1:14551   (drone hovering at 1.2 m, level, yaw = North)
  * LiDAR JSON     -> 127.0.0.1:14552   (YDLIDAR-format 360 bins of a simulated room)

Simulated world, in the DRONE's frame (x forward, y left), drone at the centre:
  a 6 m x 4 m room (walls 3 m ahead/behind, 2 m left/right) with a 0.6 m cabinet
  1.5 m ahead and 0.8 m to the left. Height matters (lidar = altitude - 0.15 m mount):
    * the cabinet is 1.5 m tall - fly above it and the lidar sees over it;
    * a doorway in the left wall (x -1.5..-0.5 m) is open below 2.0 m;
    * a window in the right wall (x 0.5..1.5 m) is open between 1.3 and 1.8 m.
  Use --sweep MIN MAX to climb/descend continuously and build a 3D map (scan_to_3d).
The drone faces North (yaw 0). In RViz the odom frame is ENU (x = East, y = North), so
"forward" points UP the screen: the room appears 4 m wide x 6 m tall with the box at
upper-left - which is correct.

Usage (laptop, two terminals):
    python3 raspberry_pi/fake_pi.py
    ros2 launch x500_navigation navigation_real.launch.py camera:=false pi_ip:=192.0.2.1
(pi_ip points the command bridge at an unused address so it doesn't send to this machine.)
"""
import argparse
import json
import math
import socket
import time

ROOM_X, ROOM_Y = 3.0, 2.0
BOX = (1.5, 0.8, 0.3)    # centre x, centre y, half-size (m)
BOX_TOP = 1.5            # cabinet height (m)
DOOR = (-1.5, -0.5, 2.0)          # left wall (y=+2): x range, open below this height
WINDOW = (0.5, 1.5, 1.3, 1.8)     # right wall (y=-2): x range, open between these heights
LIDAR_BELOW_BASE = 0.15           # lidar mount, m below base_link


def ray_box(ox, oy, dx, dy, x0, x1, y0, y1):
    """Distance along a ray to an axis-aligned box (slab method), or inf."""
    tmin, tmax = -math.inf, math.inf
    for o, d, lo, hi in ((ox, dx, x0, x1), (oy, dy, y0, y1)):
        if abs(d) < 1e-12:
            if not lo <= o <= hi:
                return math.inf
            continue
        t1, t2 = (lo - o) / d, (hi - o) / d
        tmin, tmax = max(tmin, min(t1, t2)), min(tmax, max(t1, t2))
    return tmin if tmax >= max(tmin, 0.0) and tmin > 0 else math.inf


def wall_open(x, y, z):
    """True if the wall point (x, y) at height z is a doorway/window opening."""
    if abs(y - ROOM_Y) < 1e-6 and DOOR[0] <= x <= DOOR[1] and z < DOOR[2]:
        return True
    if abs(y + ROOM_Y) < 1e-6 and WINDOW[0] <= x <= WINDOW[1] and WINDOW[2] <= z <= WINDOW[3]:
        return True
    return False


def scan_ranges(z, noise=0.01):
    """360 raw bins at lidar height z, index k = sensor angle in degrees CLOCKWISE
    (as the YDLIDAR sends)."""
    import random
    out = []
    for k in range(360):
        th = math.radians(-k)                     # receiver maps raw CW k -> ROS angle -k
        dx, dy = math.cos(th), math.sin(th)
        d = math.inf
        for t in ((ROOM_X / dx) if dx else math.inf, (-ROOM_X / dx) if dx else math.inf,
                  (ROOM_Y / dy) if dy else math.inf, (-ROOM_Y / dy) if dy else math.inf):
            if t > 0 and not wall_open(t * dx, t * dy, z):
                d = min(d, t)
        bx, by, h = BOX
        if z <= BOX_TOP:
            d = min(d, ray_box(0.0, 0.0, dx, dy, bx - h, bx + h, by - h, by + h))
        out.append(round(d + random.gauss(0.0, noise), 3) if d < 16.0 else 0.0)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--target-ip', default='127.0.0.1')
    ap.add_argument('--telemetry-port', type=int, default=14551)
    ap.add_argument('--lidar-port', type=int, default=14552)
    ap.add_argument('--altitude', type=float, default=1.2)
    ap.add_argument('--sweep', type=float, nargs=2, metavar=('MIN', 'MAX'),
                    help='climb/descend continuously between MIN and MAX altitude (m)')
    ap.add_argument('--sweep-speed', type=float, default=0.15, help='vertical speed (m/s)')
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f'[fake_pi] telemetry -> {args.target_ip}:{args.telemetry_port} (10 Hz), '
          f'lidar -> {args.target_ip}:{args.lidar_port} (7 Hz). Ctrl-C to stop.')
    t_tel = t_lid = 0.0
    t0 = time.time()

    def altitude(now):
        if not args.sweep:
            return args.altitude
        lo, hi = args.sweep
        span = max(hi - lo, 1e-3)
        p = ((now - t0) * args.sweep_speed) % (2 * span)   # triangle wave
        return lo + (p if p < span else 2 * span - p)

    try:
        while True:
            now = time.time()
            alt = altitude(now)
            if now - t_tel >= 0.1:
                t_tel = now
                telem = {
                    'timestamp': now, 'connected': True, 'armed': False, 'flight_mode': 'GUIDED',
                    'attitude': {'roll_deg': 0.0, 'pitch_deg': 0.0, 'yaw_deg': 0.0},
                    'imu': {'gyro_x_dps': 0.0, 'gyro_y_dps': 0.0, 'gyro_z_dps': 0.0},
                    'rangefinder': {'distance_m': alt},
                    'optical_flow': {'vel_x_ms': 0.0, 'vel_y_ms': 0.0, 'quality': 200,
                                     'ground_distance_m': alt},
                }
                sock.sendto(json.dumps(telem).encode(), (args.target_ip, args.telemetry_port))
            if now - t_lid >= 1 / 7.0:
                t_lid = now
                pkt = {'timestamp': now, 'ranges': scan_ranges(alt - LIDAR_BELOW_BASE),
                       'intensities': [100] * 360}
                sock.sendto(json.dumps(pkt).encode(), (args.target_ip, args.lidar_port))
            time.sleep(0.005)
    except KeyboardInterrupt:
        print('\n[fake_pi] stopped')


if __name__ == '__main__':
    main()
