#!/usr/bin/env python3
"""
metrics_monitor.py - validation / health metrics for the GPS-denied stack.

Passively watches the topics the stack already produces and republishes
scalar metrics under /metrics/*, a /diagnostics summary and an RViz overlay.

Scalars (std_msgs/Float32)
  /metrics/lidar/{rate_hz,jitter_ms,nearest_obstacle_m,valid_ratio,tf_fail_ratio}
  /metrics/telemetry/rate_hz           (latency_ms / latency_excess_ms come from the bridge)
  /metrics/camera/rate_hz              (latency/decode/bandwidth come from the camera receiver)
  /metrics/odom/{rate_hz,speed_mps,distance_from_start_m,path_length_m}
  /metrics/tf/odom_base_age_ms
  /metrics/costmap/local/{rate_hz,lethal_cells,inflated_cells,scan_agreement,scan_coverage}
  /metrics/costmap/global/{rate_hz,lethal_cells}
  /metrics/nav/{cmd_vel_rate_hz,cmd_speed_mps,plan_length_m}
  /metrics/system/{cpu_percent,mem_percent}        (only if psutil is installed)

Other outputs
  /diagnostics                 diagnostic_msgs/DiagnosticArray (rqt_robot_monitor)
  /metrics/overlay             visualization_msgs/MarkerArray  (range rings, nearest
                               obstacle, text dashboard)
  /metrics/odom_trail          nav_msgs/Path (dead-reckoned trajectory)

costmap `scan_agreement` = fraction of lidar returns (inside the costmap window,
projected into the costmap frame via TF) whose cell, or a direct neighbour, is
LETHAL (100) in the local costmap. ~1.0 means the costmap faithfully reproduces what
the lidar sees; drops reveal TF/timing problems or dropped scans.
"""

import math
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from rclpy.time import Time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA, Float32
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros

from gps_denied_bridge.metrics_utils import RateTracker

try:
    import psutil
except ImportError:  # optional
    psutil = None

# Nav2 publishes /*_costmap/costmap as an OccupancyGrid with lethal=100, inscribed=99, unknown=-1.
LETHAL = 100
OK, WARN, ERROR = DiagnosticStatus.OK, DiagnosticStatus.WARN, DiagnosticStatus.ERROR
LEVEL_RGB = {OK: (0.2, 0.9, 0.3), WARN: (1.0, 0.75, 0.1), ERROR: (1.0, 0.25, 0.2)}
LEVEL_TXT = {OK: 'OK', WARN: 'WARN', ERROR: 'FAIL'}


def quat_to_rot(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class MetricsMonitor(Node):
    def __init__(self):
        super().__init__('metrics_monitor')

        P = self.declare_parameter
        P('odom_frame', 'odom')
        P('base_frame', 'base_link')
        P('lidar_min_hz', 4.0)
        P('camera_min_hz', 8.0)
        P('telemetry_min_hz', 5.0)
        P('expect_camera', True)
        P('latency_warn_ms', 150.0)    # thresholds apply to *excess* latency
        P('latency_error_ms', 400.0)
        P('tf_age_warn_ms', 250.0)
        P('obstacle_warn_m', 2.0)
        P('obstacle_error_m', 0.8)
        P('scan_max_range_m', 8.0)     # = costmap obstacle_max_range
        P('trail_max_points', 600)
        P('dashboard_offset', [-0.3, 1.8, 0.4])
        P('dashboard_text_height', 0.13)
        P('ring_radii_m', [1.0, 2.0, 4.0, 8.0])
        g = lambda n: self.get_parameter(n).value
        self.odom_frame, self.base_frame = g('odom_frame'), g('base_frame')
        self.cfg = {k: g(k) for k in (
            'lidar_min_hz', 'camera_min_hz', 'telemetry_min_hz', 'expect_camera',
            'latency_warn_ms', 'latency_error_ms', 'tf_age_warn_ms',
            'obstacle_warn_m', 'obstacle_error_m', 'scan_max_range_m')}
        self.trail_max = int(g('trail_max_points'))
        self.dash_off = list(g('dashboard_offset'))
        self.dash_h = float(g('dashboard_text_height'))
        self.rings = list(g('ring_radii_m'))

        # ---- state ----
        self.r = {k: RateTracker() for k in (
            'lidar', 'telemetry', 'camera', 'odom', 'cmd', 'lcm', 'gcm')}
        self.lat = {'lidar': (0.0, 0.0), 'telemetry': (0.0, 0.0), 'camera': (0.0, 0.0)}
        self.cam_extra = {'decode_ms': 0.0, 'bandwidth_mbps': 0.0}
        self.last_scan = None
        self.scan_frame = 'laser_frame'
        self.nearest = float('nan')
        self.valid_ratio = 0.0
        self.tf_fail_ema = 0.0
        self.odom = None
        self.odom_origin = None
        self.path_len = 0.0
        self.prev_xy = None
        self.cmd_speed = 0.0
        self.plan_len = 0.0
        self.lcm = {'lethal': 0, 'inflated': 0, 'agree': float('nan'), 'cover': 0.0}
        self.gcm_lethal = 0
        self.trail = Path()
        self.trail.header.frame_id = self.odom_frame

        # ---- TF ----
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)

        # ---- subscriptions ----
        sensor = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST, depth=10)
        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(LaserScan, '/scan', self.on_scan, sensor)
        self.create_subscription(Odometry, '/odom', self.on_odom, sensor)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.create_subscription(Path, '/plan', self.on_plan, 10)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.on_local_cm, latched)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.on_global_cm, latched)
        # Rate/latency of camera + telemetry taken from the receivers' own metric
        # topics, so we never deserialise full camera frames here.
        for name in ('lidar', 'telemetry', 'camera'):
            self.create_subscription(
                Float32, f'/metrics/{name}/latency_ms',
                lambda m, n=name: self.on_lat(n, m), sensor)
            self.create_subscription(
                Float32, f'/metrics/{name}/latency_excess_ms',
                lambda m, n=name: self.on_lat_ex(n, m), sensor)
        self.create_subscription(
            Float32, '/metrics/camera/decode_ms',
            lambda m: self.cam_extra.__setitem__('decode_ms', m.data), sensor)
        self.create_subscription(
            Float32, '/metrics/camera/bandwidth_mbps',
            lambda m: self.cam_extra.__setitem__('bandwidth_mbps', m.data), sensor)

        # ---- publishers ----
        self._pubs = {}
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/metrics/overlay', 10)
        self.trail_pub = self.create_publisher(Path, '/metrics/odom_trail', 10)

        self.create_timer(0.5, self.publish_metrics)      # 2 Hz
        self.create_timer(1.0, self.publish_static_overlay)
        self.get_logger().info('metrics_monitor up: publishing /metrics/*, /diagnostics, /metrics/overlay')

    # ---------------------------------------------------------------- helpers
    def pub(self, topic, value):
        if topic not in self._pubs:
            self._pubs[topic] = self.create_publisher(Float32, topic, 10)
        v = float(value)
        self._pubs[topic].publish(Float32(data=v if math.isfinite(v) else float('nan')))

    def on_lat(self, name, msg):
        self.lat[name] = (msg.data, self.lat[name][1])
        self.r[name].tick() if name in ('telemetry', 'camera') else None

    def on_lat_ex(self, name, msg):
        self.lat[name] = (self.lat[name][0], msg.data)

    # -------------------------------------------------------------- callbacks
    def on_scan(self, msg: LaserScan):
        self.r['lidar'].tick()
        self.last_scan = msg
        self.scan_frame = msg.header.frame_id
        rng = np.asarray(msg.ranges, dtype=np.float32)
        ok = np.isfinite(rng) & (rng >= msg.range_min) & (rng <= msg.range_max)
        self.valid_ratio = float(ok.mean()) if rng.size else 0.0
        self.nearest = float(rng[ok].min()) if ok.any() else float('nan')
        # Would Nav2's message filter accept this scan? (TF available at its stamp)
        try:
            self.tf_buf.lookup_transform(
                self.odom_frame, msg.header.frame_id, Time.from_msg(msg.header.stamp))
            fail = 0.0
        except Exception:
            fail = 1.0
        self.tf_fail_ema += 0.05 * (fail - self.tf_fail_ema)

    def on_odom(self, msg: Odometry):
        self.r['odom'].tick()
        self.odom = msg
        p = msg.pose.pose.position
        if self.odom_origin is None:
            self.odom_origin = (p.x, p.y)
        if self.prev_xy is not None:
            self.path_len += math.hypot(p.x - self.prev_xy[0], p.y - self.prev_xy[1])
        self.prev_xy = (p.x, p.y)
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        self.trail.poses.append(ps)
        if len(self.trail.poses) > self.trail_max:
            del self.trail.poses[0]

    def on_cmd(self, msg: Twist):
        self.r['cmd'].tick()
        self.cmd_speed = math.hypot(msg.linear.x, msg.linear.y)

    def on_plan(self, msg: Path):
        pts = msg.poses
        self.plan_len = sum(
            math.hypot(b.pose.position.x - a.pose.position.x,
                       b.pose.position.y - a.pose.position.y)
            for a, b in zip(pts, pts[1:]))

    def on_global_cm(self, msg: OccupancyGrid):
        self.r['gcm'].tick()
        self.gcm_lethal = int(np.count_nonzero(np.asarray(msg.data, dtype=np.int8) == LETHAL))

    def on_local_cm(self, msg: OccupancyGrid):
        self.r['lcm'].tick()
        w, h, res = msg.info.width, msg.info.height, msg.info.resolution
        grid = np.asarray(msg.data, dtype=np.int8).reshape(h, w)
        lethal = grid == LETHAL
        self.lcm['lethal'] = int(lethal.sum())
        self.lcm['inflated'] = int(np.count_nonzero((grid > 0) & (grid < LETHAL)))

        scan = self.last_scan
        if scan is None:
            return
        try:
            tf = self.tf_buf.lookup_transform(msg.header.frame_id, scan.header.frame_id, Time())
        except Exception:
            return
        rng = np.asarray(scan.ranges, dtype=np.float32)
        ang = scan.angle_min + np.arange(rng.size, dtype=np.float32) * scan.angle_increment
        ok = np.isfinite(rng) & (rng >= scan.range_min) & (rng <= self.cfg['scan_max_range_m'])
        if not ok.any():
            self.lcm['agree'], self.lcm['cover'] = float('nan'), 0.0
            return
        pts = np.stack([rng[ok] * np.cos(ang[ok]), rng[ok] * np.sin(ang[ok]),
                        np.zeros(ok.sum())])
        R = quat_to_rot(tf.transform.rotation)
        t = tf.transform.translation
        world = R @ pts + np.array([[t.x], [t.y], [t.z]])
        ix = np.floor((world[0] - msg.info.origin.position.x) / res).astype(int)
        iy = np.floor((world[1] - msg.info.origin.position.y) / res).astype(int)
        inside = (ix >= 1) & (ix < w - 1) & (iy >= 1) & (iy < h - 1)
        self.lcm['cover'] = float(inside.mean())
        if not inside.any():
            self.lcm['agree'] = float('nan')
            return
        ix, iy = ix[inside], iy[inside]
        hit = np.zeros(ix.size, dtype=bool)
        for dx in (-1, 0, 1):          # 3x3 tolerance = +-1 cell (5 cm @ 0.05 m)
            for dy in (-1, 0, 1):
                hit |= lethal[iy + dy, ix + dx]
        self.lcm['agree'] = float(hit.mean())

    # ------------------------------------------------------------ diagnostics
    def _tf_age_ms(self):
        try:
            t = self.tf_buf.lookup_transform(self.odom_frame, self.base_frame, Time())
            return (self.get_clock().now() - Time.from_msg(t.header.stamp)).nanoseconds / 1e6
        except Exception:
            return float('inf')

    def _stream_level(self, name, min_hz, required=True):
        hz = self.r[name].rate_hz()
        if hz <= 0.0:
            return (ERROR if required else WARN), 'no data'
        lat_ex = self.lat.get(name, (0.0, 0.0))[1]
        if hz < min_hz or lat_ex > self.cfg['latency_error_ms']:
            return ERROR, f'{hz:.1f} Hz, +{lat_ex:.0f} ms'
        if hz < 1.5 * min_hz or lat_ex > self.cfg['latency_warn_ms']:
            return WARN, f'{hz:.1f} Hz, +{lat_ex:.0f} ms'
        return OK, f'{hz:.1f} Hz, +{lat_ex:.0f} ms'

    def publish_metrics(self):
        c = self.cfg
        tf_age = self._tf_age_ms()

        # scalars
        self.pub('/metrics/lidar/rate_hz', self.r['lidar'].rate_hz())
        self.pub('/metrics/lidar/jitter_ms', self.r['lidar'].jitter_ms())
        self.pub('/metrics/lidar/nearest_obstacle_m', self.nearest)
        self.pub('/metrics/lidar/valid_ratio', self.valid_ratio)
        self.pub('/metrics/lidar/tf_fail_ratio', self.tf_fail_ema)
        self.pub('/metrics/telemetry/rate_hz', self.r['telemetry'].rate_hz())
        self.pub('/metrics/camera/rate_hz', self.r['camera'].rate_hz())
        self.pub('/metrics/odom/rate_hz', self.r['odom'].rate_hz())
        speed = 0.0
        dist = 0.0
        if self.odom is not None:
            v = self.odom.twist.twist.linear
            speed = math.sqrt(v.x ** 2 + v.y ** 2)
            p = self.odom.pose.pose.position
            dist = math.hypot(p.x - self.odom_origin[0], p.y - self.odom_origin[1])
        self.pub('/metrics/odom/speed_mps', speed)
        self.pub('/metrics/odom/distance_from_start_m', dist)
        self.pub('/metrics/odom/path_length_m', self.path_len)
        self.pub('/metrics/tf/odom_base_age_ms', tf_age if math.isfinite(tf_age) else -1.0)
        self.pub('/metrics/costmap/local/rate_hz', self.r['lcm'].rate_hz())
        self.pub('/metrics/costmap/local/lethal_cells', self.lcm['lethal'])
        self.pub('/metrics/costmap/local/inflated_cells', self.lcm['inflated'])
        self.pub('/metrics/costmap/local/scan_agreement', self.lcm['agree'])
        self.pub('/metrics/costmap/local/scan_coverage', self.lcm['cover'])
        self.pub('/metrics/costmap/global/rate_hz', self.r['gcm'].rate_hz())
        self.pub('/metrics/costmap/global/lethal_cells', self.gcm_lethal)
        self.pub('/metrics/nav/cmd_vel_rate_hz', self.r['cmd'].rate_hz())
        self.pub('/metrics/nav/cmd_speed_mps', self.cmd_speed if self.r['cmd'].age_s() < 1.0 else 0.0)
        self.pub('/metrics/nav/plan_length_m', self.plan_len)
        cpu = mem = float('nan')
        if psutil:
            cpu, mem = psutil.cpu_percent(), psutil.virtual_memory().percent
            self.pub('/metrics/system/cpu_percent', cpu)
            self.pub('/metrics/system/mem_percent', mem)

        # diagnostics
        rows = []   # (name, level, summary, {kv})
        lv, sm = self._stream_level('lidar', c['lidar_min_hz'])
        if lv == OK and self.tf_fail_ema > 0.2:
            lv, sm = WARN, sm + f', TF miss {self.tf_fail_ema:.0%}'
        if lv == OK and math.isfinite(self.nearest) and self.nearest < c['obstacle_error_m']:
            lv, sm = WARN, sm + f', obstacle {self.nearest:.2f} m'
        rows.append(('LiDAR', lv, sm, {
            'rate_hz': self.r['lidar'].rate_hz(), 'jitter_ms': self.r['lidar'].jitter_ms(),
            'latency_ms': self.lat['lidar'][0], 'latency_excess_ms': self.lat['lidar'][1],
            'valid_ratio': self.valid_ratio, 'nearest_m': self.nearest,
            'tf_fail_ratio': self.tf_fail_ema}))
        lv, sm = self._stream_level('camera', c['camera_min_hz'], c['expect_camera'])
        rows.append(('Camera', lv, sm, {
            'rate_hz': self.r['camera'].rate_hz(), 'latency_ms': self.lat['camera'][0],
            'latency_excess_ms': self.lat['camera'][1], **self.cam_extra}))
        lv, sm = self._stream_level('telemetry', c['telemetry_min_hz'])
        rows.append(('Telemetry', lv, sm, {
            'rate_hz': self.r['telemetry'].rate_hz(), 'latency_ms': self.lat['telemetry'][0],
            'latency_excess_ms': self.lat['telemetry'][1]}))
        if not math.isfinite(tf_age):
            lv, sm = ERROR, 'odom->base_link missing'
        elif tf_age > c['tf_age_warn_ms']:
            lv, sm = WARN, f'stale {tf_age:.0f} ms'
        else:
            lv, sm = OK, f'age {tf_age:.0f} ms'
        rows.append(('TF', lv, sm, {'odom_base_age_ms': tf_age}))
        lrate = self.r['lcm'].rate_hz()
        agree = self.lcm['agree']
        if lrate <= 0.0:
            lv, sm = ERROR, 'no costmap'
        elif math.isfinite(agree) and agree < 0.5:
            lv, sm = WARN, f'{lrate:.1f} Hz, scan match {agree:.0%}'
        else:
            lv, sm = OK, f'{lrate:.1f} Hz, scan match {agree:.0%}' if math.isfinite(agree) \
                else f'{lrate:.1f} Hz'
        rows.append(('Costmap', lv, sm, {
            'local_rate_hz': lrate, 'global_rate_hz': self.r['gcm'].rate_hz(),
            'lethal_cells': self.lcm['lethal'], 'scan_agreement': agree,
            'scan_coverage': self.lcm['cover']}))

        stamp = self.get_clock().now().to_msg()
        arr = DiagnosticArray()
        arr.header.stamp = stamp
        for name, lv, sm, kv in rows:
            st = DiagnosticStatus(level=lv, name=f'gps_denied/{name}', message=sm,
                                  hardware_id='x500')
            st.values = [KeyValue(key=k, value=f'{v:.3f}' if isinstance(v, float) else str(v))
                         for k, v in kv.items()]
            arr.status.append(st)
        self.diag_pub.publish(arr)

        self.trail.header.stamp = stamp
        self.trail_pub.publish(self.trail)
        self.publish_dashboard(rows, speed, dist, cpu, stamp)
        self.publish_nearest(stamp)

    # ----------------------------------------------------------------- overlay
    def _marker(self, ns, mid, mtype, frame, stamp=None):
        m = Marker()
        m.header.frame_id = frame
        m.header.stamp = stamp if stamp is not None else Time().to_msg()
        m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def publish_dashboard(self, rows, speed, dist, cpu, stamp):
        worst = max(lv for _, lv, _, _ in rows)
        lines = [f'SYSTEM {LEVEL_TXT[worst]}']
        for name, lv, sm, _ in rows:
            lines.append(f'[{LEVEL_TXT[lv]:>4}] {name:<9} {sm}')
        lines.append(f'v {speed:.2f} m/s  d {dist:.1f} m  path {self.path_len:.1f} m')
        if math.isfinite(self.nearest):
            lines.append(f'nearest obstacle {self.nearest:.2f} m')
        if psutil:
            lines.append(f'GCS cpu {cpu:.0f}%')
        m = self._marker('dashboard', 0, Marker.TEXT_VIEW_FACING, self.base_frame, stamp)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = self.dash_off
        m.scale.z = self.dash_h
        r, g_, b = LEVEL_RGB[worst]
        m.color = ColorRGBA(r=r, g=g_, b=b, a=1.0)
        m.text = '\n'.join(lines)
        m.lifetime = Duration(seconds=2).to_msg()
        out = MarkerArray()
        out.markers.append(m)
        self.marker_pub.publish(out)

    def publish_nearest(self, stamp):
        scan = self.last_scan
        if scan is None or not math.isfinite(self.nearest):
            return
        rng = np.asarray(scan.ranges, dtype=np.float32)
        ok = np.isfinite(rng) & (rng >= scan.range_min) & (rng <= scan.range_max)
        i = int(np.argmin(np.where(ok, rng, np.inf)))
        a = scan.angle_min + i * scan.angle_increment
        m = self._marker('nearest', 1, Marker.LINE_LIST, scan.header.frame_id, stamp)
        m.scale.x = 0.025
        m.points = [Point(), Point(x=float(rng[i] * math.cos(a)), y=float(rng[i] * math.sin(a)))]
        lv = ERROR if self.nearest < self.cfg['obstacle_error_m'] else \
            WARN if self.nearest < self.cfg['obstacle_warn_m'] else OK
        r, g_, b = LEVEL_RGB[lv]
        m.color = ColorRGBA(r=r, g=g_, b=b, a=0.95)
        m.lifetime = Duration(seconds=1).to_msg()
        out = MarkerArray()
        out.markers.append(m)
        self.marker_pub.publish(out)

    def publish_static_overlay(self):
        """Range rings + labels around the drone (latched-ish, re-sent at 1 Hz)."""
        out = MarkerArray()
        for k, rad in enumerate(self.rings):
            m = self._marker('rings', k, Marker.LINE_STRIP, self.base_frame)
            m.scale.x = 0.01
            m.color = ColorRGBA(r=0.6, g=0.7, b=0.8, a=0.45)
            m.points = [Point(x=rad * math.cos(2 * math.pi * j / 72),
                              y=rad * math.sin(2 * math.pi * j / 72)) for j in range(73)]
            m.lifetime = Duration(seconds=3).to_msg()
            out.markers.append(m)
            t = self._marker('ring_labels', k, Marker.TEXT_VIEW_FACING, self.base_frame)
            t.pose.position.x = rad * 0.7071
            t.pose.position.y = rad * 0.7071
            t.scale.z = 0.12
            t.color = ColorRGBA(r=0.75, g=0.85, b=0.95, a=0.8)
            t.text = f'{rad:g} m'
            t.lifetime = Duration(seconds=3).to_msg()
            out.markers.append(t)
        self.marker_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MetricsMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
