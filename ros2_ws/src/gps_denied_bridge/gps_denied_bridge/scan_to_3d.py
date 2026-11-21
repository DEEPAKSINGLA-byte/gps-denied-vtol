#!/usr/bin/env python3
"""
scan_to_3d.py - build a 3D point cloud of the room from the 2D lidar + drone height.

Every /scan is a horizontal slice at the lidar's current height. Each return is placed in
the odom frame with the full TF (x, y, z = altitude, roll/pitch/yaw, and the base_link ->
laser_frame mount), so slices taken at different heights - or while the drone tilts -
stack up into a 3D map. Fly slow vertical sweeps (e.g. 0.3 -> 2 m) to fill in walls.

Points are de-duplicated into a voxel grid (one point per voxel_size cube). A voxel is
only published once it has been hit min_hits times, which removes single-scan noise.

Topics
  sub  /scan                  sensor_msgs/LaserScan
  pub  /map_3d                sensor_msgs/PointCloud2 (odom frame, x/y/z, 1 Hz, latched)
Services
  /map_3d/save   std_srvs/Trigger   write a .ply (opens in CloudCompare / MeshLab)
  /map_3d/clear  std_srvs/Trigger   forget everything
The map is also saved automatically on shutdown (if it has any points).

Accuracy is limited by the height estimate (baro drifts indoors; the flow sensor's
ground distance is better below ~2 m) and by optical-flow XY drift, which thickens walls
over long flights.
"""
import collections
import math
import os
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.time import Time

from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from std_srvs.srv import Trigger
import tf2_ros


def quat_to_rot(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class ScanTo3D(Node):
    def __init__(self):
        super().__init__('scan_to_3d')
        P = self.declare_parameter
        P('scan_topic', '/scan')
        P('cloud_topic', '/map_3d')
        P('map_frame', 'odom')
        P('voxel_size', 0.05)          # m
        P('min_hits', 2)               # hits before a voxel is published (noise filter)
        P('min_range', 0.15)           # m, ignore returns closer than this (props/frame)
        P('max_range', 8.0)            # m, YDLIDAR G2 is unreliable beyond ~8 m indoors
        P('max_tilt_deg', 30.0)        # skip scans while the drone is tilted more than this
        P('max_voxels', 2_000_000)
        P('publish_rate_hz', 1.0)
        P('tf_timeout_s', 0.5)          # drop a scan if no TF reaches its stamp by then
        P('save_dir', os.path.expanduser('~/drone_maps'))
        P('save_on_shutdown', True)
        g = lambda n: self.get_parameter(n).value
        self.frame = g('map_frame')
        self.vox = float(g('voxel_size'))
        self.min_hits = int(g('min_hits'))
        self.rmin, self.rmax = float(g('min_range')), float(g('max_range'))
        self.max_tilt_cos = math.cos(math.radians(float(g('max_tilt_deg'))))
        self.max_voxels = int(g('max_voxels'))
        self.tf_timeout = Duration(seconds=float(g('tf_timeout_s')))
        self.save_dir = g('save_dir')
        self.save_on_shutdown = bool(g('save_on_shutdown'))

        # voxel key (int64) -> hit count; plus the summed coordinates for the centroid
        self.hits = {}
        self.sums = {}
        self.scans_used = self.scans_skipped = 0
        self.dirty = False
        self.z_seen = [math.inf, -math.inf]

        self.tf_buf = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)
        # Scans are stamped on arrival, usually a few ms AFTER the newest odom TF, so they
        # are queued and processed (never blocking) once a TF at/after their stamp exists.
        self.pending = collections.deque(maxlen=50)

        self.create_subscription(LaserScan, g('scan_topic'), self.on_scan, qos_profile_sensor_data)
        self.create_timer(0.02, self.process_pending)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(PointCloud2, g('cloud_topic'), latched)
        self.create_service(Trigger, g('cloud_topic') + '/save', self.srv_save)
        self.create_service(Trigger, g('cloud_topic') + '/clear', self.srv_clear)
        self.create_timer(1.0 / float(g('publish_rate_hz')), self.publish)
        self.create_timer(10.0, self.report)
        self.get_logger().info(
            f'scan_to_3d: {g("scan_topic")} -> {g("cloud_topic")} in {self.frame}, '
            f'voxel {self.vox:.2f} m, min_hits {self.min_hits}; save with '
            f'`ros2 service call {g("cloud_topic")}/save std_srvs/srv/Trigger`')

    # ------------------------------------------------------------------ input
    def on_scan(self, scan: LaserScan):
        self.pending.append(scan)
        self.process_pending()

    def process_pending(self):
        now = self.get_clock().now()
        while self.pending:
            scan = self.pending[0]
            stamp = Time.from_msg(scan.header.stamp)
            if self.tf_buf.can_transform(self.frame, scan.header.frame_id, stamp):
                self.pending.popleft()
                tf = self.tf_buf.lookup_transform(self.frame, scan.header.frame_id, stamp)
                self.add_scan(scan, tf)
            elif now - stamp > self.tf_timeout:
                self.pending.popleft()       # TF never arrived (telemetry gap): drop it
                self.scans_skipped += 1
                if self.scans_skipped % 50 == 1:
                    self.get_logger().warn(
                        f'dropping scans: no TF {self.frame} <- {scan.header.frame_id} '
                        f'within {self.tf_timeout.nanoseconds / 1e9:.1f} s (is /odom running?)')
            else:
                break                        # wait for the next TF; keep order

    def add_scan(self, scan: LaserScan, tf):
        R = quat_to_rot(tf.transform.rotation)
        if R[2, 2] < self.max_tilt_cos:     # lidar plane normal vs. vertical
            self.scans_skipped += 1
            return
        t = tf.transform.translation

        r = np.asarray(scan.ranges, dtype=np.float64)
        a = scan.angle_min + np.arange(r.size) * scan.angle_increment
        ok = np.isfinite(r) & (r >= max(self.rmin, scan.range_min)) & \
            (r <= min(self.rmax, scan.range_max))
        if not ok.any():
            return
        r, a = r[ok], a[ok]
        local = np.stack([r * np.cos(a), r * np.sin(a), np.zeros_like(r)])   # 3 x N
        pts = (R @ local).T + np.array([t.x, t.y, t.z])                       # N x 3

        keys = np.floor(pts / self.vox).astype(np.int64)
        # pack 3 x 21-bit signed cell indices into one int64 key
        packed = ((keys[:, 0] & 0x1FFFFF) << 42) | ((keys[:, 1] & 0x1FFFFF) << 21) | \
            (keys[:, 2] & 0x1FFFFF)
        for k, p in zip(packed.tolist(), pts):
            n = self.hits.get(k)
            if n is None:
                if len(self.hits) >= self.max_voxels:
                    continue
                self.hits[k] = 1
                self.sums[k] = p.copy()
            else:
                self.hits[k] = n + 1
                self.sums[k] += p
        self.scans_used += 1
        self.dirty = True
        self.z_seen[0] = min(self.z_seen[0], t.z)
        self.z_seen[1] = max(self.z_seen[1], t.z)

    # ----------------------------------------------------------------- output
    def points(self):
        if not self.hits:
            return np.zeros((0, 3), np.float32)
        out = [self.sums[k] / n for k, n in self.hits.items() if n >= self.min_hits]
        return np.asarray(out, np.float32).reshape(-1, 3)

    def publish(self):
        if not self.dirty:
            return
        self.dirty = False
        hdr = Header(frame_id=self.frame, stamp=self.get_clock().now().to_msg())
        self.pub.publish(point_cloud2.create_cloud_xyz32(hdr, self.points()))

    def report(self):
        if self.scans_used == 0:
            return
        zl, zh = self.z_seen
        self.get_logger().info(
            f'3D map: {len(self.hits)} voxels ({len(self.points())} published), '
            f'{self.scans_used} scans used, {self.scans_skipped} skipped, '
            f'lidar heights {zl:.2f}..{zh:.2f} m')

    def save(self):
        pts = self.points()
        if len(pts) == 0:
            return None
        os.makedirs(self.save_dir, exist_ok=True)
        path = os.path.join(self.save_dir, time.strftime('map3d_%Y%m%d_%H%M%S.ply'))
        # Colour by height (blue low -> red high) so the file is readable in any viewer.
        z = pts[:, 2]
        u = (z - z.min()) / max(1e-6, float(z.max() - z.min()))
        rgb = np.stack([255 * u, 255 * (1 - np.abs(2 * u - 1)), 255 * (1 - u)], 1).astype(np.uint8)
        with open(path, 'w') as f:
            f.write('ply\nformat ascii 1.0\n'
                    f'comment gps-denied-vtol scan_to_3d, frame {self.frame}, voxel {self.vox}\n'
                    f'element vertex {len(pts)}\n'
                    'property float x\nproperty float y\nproperty float z\n'
                    'property uchar red\nproperty uchar green\nproperty uchar blue\n'
                    'end_header\n')
            for (x, y, zz), (cr, cg, cb) in zip(pts, rgb):
                f.write(f'{x:.3f} {y:.3f} {zz:.3f} {cr} {cg} {cb}\n')
        return path, len(pts)

    def srv_save(self, req, res):
        out = self.save()
        res.success = out is not None
        res.message = f'saved {out[1]} points to {out[0]}' if out else 'map is empty'
        self.get_logger().info(res.message)
        return res

    def srv_clear(self, req, res):
        self.hits.clear()
        self.sums.clear()
        self.scans_used = self.scans_skipped = 0
        self.z_seen = [math.inf, -math.inf]
        self.dirty = True
        res.success, res.message = True, '3D map cleared'
        self.get_logger().info(res.message)
        return res


def main(args=None):
    rclpy.init(args=args)
    node = ScanTo3D()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node.save_on_shutdown:
            out = node.save()
            if out:
                print(f'[scan_to_3d] saved {out[1]} points to {out[0]}', flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
