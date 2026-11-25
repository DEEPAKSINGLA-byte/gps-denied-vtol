#!/usr/bin/env python3
"""
mavlink_to_ros2_bridge.py

Receives parsed JSON Pixhawk telemetry over UDP (port 14551) from the Raspberry
Pi companion computer and publishes it as standard ROS 2 topics for Nav2:

  * nav_msgs/Odometry on /odom
    - position.x/y: integrated from optical flow XY velocity (dead-reckoning), frozen
      while the IMU reports the vehicle stationary (zero-velocity update, zupt_*)
    - position:     ENU (x=East, y=North, z=Up); NED/FRD inputs are converted here
    - orientation:  from attitude roll/pitch/yaw -> quaternion
    - twist.linear: optical flow vx/vy + baro/rangefinder vertical rate
    - twist.angular: from gyro (deg/s -> rad/s)
    - frame_id: odom, child_frame_id: base_link

  * tf2 transform: odom -> base_link (broadcast from odometry pose)

  * geometry_msgs/Vector3Stamped on /drone/optical_flow
    - raw optical flow flow_x/flow_y and ground_distance for logging/debug

This is the GPS-denied state estimation front-end: no GNSS, optical flow +
rangefinder/baro provide local position hold sufficient for Nav2 costmaps.

UDP JSON schema matches the output of PixhawkVehicle.get_telemetry() in
pixhawk_controller.py (telemetry_streamer.py on the Pi).

Latency: /odom + TF are published the moment a telemetry packet arrives (blocking
receive, backlog drained so only the newest packet is used). publish_rate_hz is only the
FALLBACK rate used while packets are not arriving, so TF stays alive and position holds
during a Wi-Fi dropout.
"""

import collections
import json
import math
import signal
import socket
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Vector3Stamped, TransformStamped
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster

from gps_denied_bridge.metrics_utils import LatencyTracker


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Convert roll/pitch/yaw (radians) to a ROS Quaternion."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


def wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def ned_to_enu_yaw(yaw_ned: float) -> float:
    """ArduPilot yaw (clockwise from North) -> ROS yaw (counter-clockwise from East)."""
    return wrap_pi(math.pi / 2.0 - yaw_ned)


# Frame conventions (ArduPilot/MAVLink -> ROS REP-103):
#   world : NED (x=North, y=East, z=Down)        -> ENU (x=East, y=North, z=Up)
#   body  : FRD (x=Forward, y=Right, z=Down)     -> FLU (x=Forward, y=Left, z=Up)
#   Euler : roll same; pitch_ros = -pitch_ned (nose-up is +ned, nose-down is +ros);
#           yaw_ros = pi/2 - yaw_ned (CW from N -> CCW from E)
#   body vectors (velocity, gyro): (x, y, z)_FLU = (x, -y, -z)_FRD
class MavlinkToRos2Bridge(Node):
    def __init__(self):
        super().__init__('mavlink_to_ros2_bridge')

        # ---------------- Parameters ----------------
        self.declare_parameter('listen_port', 14551)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_rate_hz', 10.0)   # fallback rate when no packets arrive
        self.declare_parameter('optical_flow_scale', 1.0)
        self.declare_parameter('z_source', 'auto')  # auto | rangefinder | flow | ekf | barometer
        self.declare_parameter('min_flow_quality', 20)  # ignore flow below this (0-255)
        # Baro altitude (m above sea level) that counts as z = 0. NaN (default) = use the
        # first barometer reading after startup, i.e. the ground the drone is sitting on.
        self.declare_parameter('home_alt_m', float('nan'))
        # Zero-velocity update: when the IMU says the vehicle is not moving, flow velocity is
        # forced to 0 and dead-reckoned position is frozen, so sensor noise/bias cannot
        # integrate into drift. Stationary (bench log): |a| std ~3 mg, |w| ~1 deg/s; hovering
        # vibration is >> these, so it never triggers in flight.
        self.declare_parameter('zupt_enable', True)
        self.declare_parameter('zupt_window_s', 1.0)
        self.declare_parameter('zupt_accel_std_mg', 15.0)
        self.declare_parameter('zupt_gyro_dps', 4.0)
        # Telemetry older than this is not dead-reckoned: on a Wi-Fi dropout the last flow
        # velocity must not keep being integrated (position holds until data returns).
        self.declare_parameter('telem_timeout_s', 0.5)

        self.listen_port = int(self.get_parameter('listen_port').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.publish_rate = float(self.get_parameter('publish_rate_hz').value)
        self.of_scale = float(self.get_parameter('optical_flow_scale').value)
        self.z_source = str(self.get_parameter('z_source').value)
        self.min_flow_q = int(self.get_parameter('min_flow_quality').value)
        self.home_alt_m = float(self.get_parameter('home_alt_m').value)
        self.home_hud_alt_m = float('nan')   # same, for the VFR_HUD altitude fallback
        self.zupt_enable = bool(self.get_parameter('zupt_enable').value)
        self.zupt_window_s = float(self.get_parameter('zupt_window_s').value)
        self.zupt_accel_std = float(self.get_parameter('zupt_accel_std_mg').value)
        self.zupt_gyro = float(self.get_parameter('zupt_gyro_dps').value)
        self.telem_timeout_s = float(self.get_parameter('telem_timeout_s').value)
        self._imu_hist = collections.deque()   # (monotonic, |accel| mg, |gyro| dps)
        self.stationary = False

        # ---------------- State ----------------
        self.lock = threading.Lock()
        self.latest_telem = None
        self.last_telem_time = None

        # Dead-reckoning position state (integrated from optical flow velocity)
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        self.last_integrate_time = None

        # Last known yaw for orienting flow velocity into world frame
        self.last_yaw = 0.0

        # ---------------- Publishers ----------------
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # /odom must be RELIABLE: Nav2's controller_server and bt_navigator subscribe with
        # RELIABLE QoS, and a BEST_EFFORT publisher is incompatible with them (DDS delivers
        # nothing - DWB then always assumes the drone is at rest). A RELIABLE publisher
        # still serves BEST_EFFORT subscribers (RViz, metrics_monitor).
        qos_odom = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos_odom)
        self.flow_pub = self.create_publisher(
            Vector3Stamped, '/drone/optical_flow', qos_sensor
        )

        self.declare_parameter('clock_offset_s', 0.0)
        self.lat = LatencyTracker(
            offset_s=float(self.get_parameter('clock_offset_s').value))
        self.lat_pub = self.create_publisher(Float32, '/metrics/telemetry/latency_ms', 10)
        self.lat_ex_pub = self.create_publisher(
            Float32, '/metrics/telemetry/latency_excess_ms', 10)

        self.flow_q_pub = self.create_publisher(Float32, '/metrics/flow/quality', 10)
        self.still_pub = self.create_publisher(Float32, '/metrics/odom/stationary', 10)
        self.flow_h_pub = self.create_publisher(Float32, '/metrics/flow/ground_distance_m', 10)

        # ---------------- TF ----------------
        self.tf_broadcaster = TransformBroadcaster(self)

        # ---------------- UDP socket ----------------
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', self.listen_port))
        self.sock.settimeout(0.2)   # blocking recv: wakes the instant a packet lands

        self.get_logger().info(
            f'mavlink_to_ros2_bridge listening on UDP 0.0.0.0:{self.listen_port}'
        )
        self.get_logger().info(
            f'odom_frame={self.odom_frame}, base_frame={self.base_frame}, '
            f'z_source={self.z_source}, of_scale={self.of_scale}'
        )

        # UDP receive thread
        self._stop_event = threading.Event()
        self._udp_thread = threading.Thread(
            target=self._udp_receive_loop, daemon=True, name='MavlinkUDPReceiver'
        )
        self._udp_thread.start()

        # Publishing happens on packet arrival (receive thread). The timer only covers
        # gaps: it publishes when nothing was published for a full period.
        self._pub_lock = threading.Lock()
        self._last_pub = 0.0
        self._period = 1.0 / max(self.publish_rate, 1.0)
        self.timer = self.create_timer(self._period, self._publish_timer_callback)

    # ------------------------------------------------------------
    # UDP receive
    # ------------------------------------------------------------
    def _latest_datagram(self):
        """Block for one datagram, then drain any backlog and return the newest, so a
        Wi-Fi hiccup never makes us replay stale telemetry."""
        data, _ = self.sock.recvfrom(65535)
        self.sock.setblocking(False)
        try:
            while True:
                data, _ = self.sock.recvfrom(65535)
        except (BlockingIOError, InterruptedError):
            pass
        finally:
            self.sock.settimeout(0.2)
        return data

    def _udp_receive_loop(self):
        while not self._stop_event.is_set():
            try:
                data = self._latest_datagram()
            except socket.timeout:
                continue
            except Exception:
                time.sleep(0.01)
                continue

            try:
                telem = json.loads(data.decode('utf-8'))
            except Exception:
                continue

            ts = telem.get('timestamp') if isinstance(telem, dict) else None
            if ts is not None:
                try:
                    self.lat.update(float(ts))
                    self.lat_pub.publish(Float32(data=float(self.lat.raw_ms)))
                    self.lat_ex_pub.publish(Float32(data=float(self.lat.excess_ms)))
                except Exception:
                    pass

            with self.lock:
                self.latest_telem = telem
                self.last_telem_time = time.time()
                self._update_stationary(telem)

            try:
                self._publish()                 # event-driven: no timer wait
            except Exception as e:
                self.get_logger().warn(f'odom publish failed: {e}', throttle_duration_sec=5.0)

    def _update_stationary(self, telem: dict):
        """Track IMU stillness over a sliding window (call with self.lock held)."""
        imu = telem.get('imu') if isinstance(telem, dict) else None
        now = time.monotonic()
        if imu:
            try:
                a = math.sqrt(sum(float(imu.get(f'accel_{k}_mg') or 0.0) ** 2 for k in 'xyz'))
                g = math.sqrt(sum(float(imu.get(f'gyro_{k}_dps') or 0.0) ** 2 for k in 'xyz'))
                self._imu_hist.append((now, a, g))
            except (TypeError, ValueError):
                pass
        while self._imu_hist and now - self._imu_hist[0][0] > self.zupt_window_s:
            self._imu_hist.popleft()
        h = self._imu_hist
        # Need a reasonably full window (>= 5 samples spanning most of it) to decide.
        if not self.zupt_enable or len(h) < 5 or h[-1][0] - h[0][0] < 0.6 * self.zupt_window_s:
            self.stationary = False
            return
        accs = [x[1] for x in h]
        mean_a = sum(accs) / len(accs)
        std_a = math.sqrt(sum((x - mean_a) ** 2 for x in accs) / len(accs))
        mean_g = sum(x[2] for x in h) / len(h)
        self.stationary = std_a < self.zupt_accel_std and mean_g < self.zupt_gyro

    # ------------------------------------------------------------
    # Telemetry parsing helpers
    # ------------------------------------------------------------
    @staticmethod
    def _get(nested: dict, *keys, default=None):
        cur = nested
        for k in keys:
            if not isinstance(cur, dict) or k not in cur:
                return default
            cur = cur[k]
        return cur

    def _flow_velocity(self, flow: dict):
        """Body-frame (FLU) metric velocity (m/s) from the optical-flow sensor, or (0, 0).

        Uses vel_x_ms/vel_y_ms (flow_comp_m_* from the Pixhawk). The raw flow_x/y
        fields are pixel units and must not be treated as m/s.
        """
        vx, vy = flow.get('vel_x_ms'), flow.get('vel_y_ms')
        if vx is None or vy is None or (flow.get('quality') or 0) < self.min_flow_q:
            return 0.0, 0.0
        # Sensor/body FRD -> ROS FLU: forward stays, right becomes left (negate y).
        # If forward/left motion has the wrong sign in /odom, fix FLOW_ORIENT_YAW on the
        # autopilot (or use of_scale = -1 for a pure x flip).
        return float(vx) * self.of_scale, -float(vy) * self.of_scale

    def _integrate_position(self, telem: dict, now: float, hold: bool = False):
        """Integrate optical flow velocity into world-frame X/Y position."""
        att = telem.get('attitude') or {}
        flow = telem.get('optical_flow') or {}
        lpos = telem.get('local_position_ned') or {}

        # Prefer EKF local position if available (already fused on Pixhawk)
        if lpos and all(k in lpos for k in ('x_m', 'y_m', 'z_m')):
            with self.lock:
                # NED (N, E, D) -> ENU (E, N, U)
                self.pos_x = float(lpos['y_m'])
                self.pos_y = float(lpos['x_m'])
                self.pos_z = -float(lpos['z_m'])
                self.last_integrate_time = now
                self.last_yaw = ned_to_enu_yaw(math.radians(att.get('yaw_deg', 0.0) or 0.0))
            return

        # Otherwise dead-reckon from optical flow velocity
        # Body (FLU) velocity rotated into the ENU world frame by the ROS yaw
        vx_body, vy_body = self._flow_velocity(flow)
        if hold:  # stationary (ZUPT) or telemetry stale
            vx_body = vy_body = 0.0

        yaw = self.last_yaw
        c = math.cos(yaw)
        s = math.sin(yaw)
        vx_world = c * vx_body - s * vy_body
        vy_world = s * vx_body + c * vy_body

        with self.lock:
            if self.last_integrate_time is not None:
                dt = now - self.last_integrate_time
                if 0.0 < dt < 1.0:  # ignore large gaps
                    self.pos_x += vx_world * dt
                    self.pos_y += vy_world * dt
            self.last_integrate_time = now

            self.last_yaw = ned_to_enu_yaw(math.radians(att.get('yaw_deg', 0.0) or 0.0))

    def _get_altitude(self, telem: dict) -> float:
        """Height above ground/home (m, positive up).

        'auto' tries, in order: dedicated rangefinder -> optical-flow sensor's own
        ground distance (the MTF-01 has a built-in ToF) -> EKF local position ->
        barometer -> VFR_HUD. Each is accepted only if it has a valid reading.
        """
        def rangefinder():
            d = (telem.get('rangefinder') or {}).get('distance_m')
            return float(d) if d is not None and d > 0 else None

        def flow():
            f = telem.get('optical_flow') or {}
            d = f.get('ground_distance_m')
            return float(d) if d and d > 0 and (f.get('quality') or 0) >= self.min_flow_q else None

        def ekf():
            z = (telem.get('local_position_ned') or {}).get('z_m')
            return -float(z) if z is not None else None   # NED z is down

        def baro():
            # Both are altitude above SEA LEVEL; subtract the reading at startup so z is
            # height above the takeoff spot (otherwise base_link floats tens/hundreds of
            # metres above odom).
            b = (telem.get('barometer') or {}).get('baro_alt_m')
            if b is not None:
                if not math.isfinite(self.home_alt_m):
                    self.home_alt_m = float(b)
                    self.get_logger().info(
                        f'Barometer zeroed: {b:.1f} m above sea level = z 0')
                return float(b) - self.home_alt_m
            a = (telem.get('vfr_hud') or {}).get('alt_m')
            if a is None:
                return None
            if not math.isfinite(self.home_hud_alt_m):
                self.home_hud_alt_m = float(a)
            return float(a) - self.home_hud_alt_m

        sources = {'rangefinder': [rangefinder], 'flow': [flow], 'ekf': [ekf],
                   'barometer': [baro],
                   'auto': [rangefinder, flow, ekf, baro]}.get(self.z_source, [rangefinder, flow, ekf, baro])
        for fn in sources:
            v = fn()
            if v is not None:
                return v
        return self.pos_z  # hold last

    # ------------------------------------------------------------
    # Publish timer
    # ------------------------------------------------------------
    def _publish_timer_callback(self):
        """Fallback only: publish if the receive thread has not for a full period."""
        if time.monotonic() - self._last_pub >= self._period:
            self._publish()

    def _publish(self):
        # Serialised: called from both the receive thread and the fallback timer.
        with self._pub_lock:
            self._publish_locked()
            self._last_pub = time.monotonic()

    def _publish_locked(self):
        with self.lock:
            telem = self.latest_telem
            if telem is None:
                return
            now = time.time()
            stale = now - (self.last_telem_time or 0.0) > self.telem_timeout_s
            hold = self.stationary or stale

        self._integrate_position(telem, now, hold)

        with self.lock:
            pos_x = self.pos_x
            pos_y = self.pos_y
            pos_z = self._get_altitude(telem)
            # Override z with integrated baro/rangefinder value
            self.pos_z = pos_z

            att = telem.get('attitude') or {}
            imu = telem.get('imu') or {}
            flow = telem.get('optical_flow') or {}
            lpos = telem.get('local_position_ned') or {}

            roll = math.radians(att.get('roll_deg', 0.0) or 0.0)
            pitch = math.radians(att.get('pitch_deg', 0.0) or 0.0)
            yaw = math.radians(att.get('yaw_deg', 0.0) or 0.0)

            # NED/FRD attitude -> ENU/FLU orientation (see convention note above)
            q = quaternion_from_euler(roll, -pitch, ned_to_enu_yaw(yaw))

            # Twist: linear from optical flow + vertical rate, angular from gyro
            vx_body, vy_body = self._flow_velocity(flow)
            stationary = self.stationary
            if hold:
                vx_body = vy_body = 0.0

            # Vertical velocity: prefer EKF local position vz, else baro rate (approx 0)
            vz = 0.0
            if lpos and lpos.get('vz_ms') is not None:
                vz = float(lpos['vz_ms'])  # NED vz (down positive); negated below

            # Prefer the EKF's fused velocity (NED world) over raw flow: NED -> ENU -> body FLU
            if lpos and lpos.get('vx_ms') is not None and lpos.get('vy_ms') is not None:
                v_e, v_n = float(lpos['vy_ms']), float(lpos['vx_ms'])
                yr = ned_to_enu_yaw(yaw)
                c, sn = math.cos(yr), math.sin(yr)
                vx_body = c * v_e + sn * v_n
                vy_body = -sn * v_e + c * v_n

            # Gyro is body FRD -> FLU: (x, -y, -z)
            gx = float(imu.get('gyro_x_dps', 0.0) or 0.0)
            gy = -float(imu.get('gyro_y_dps', 0.0) or 0.0)
            gz = -float(imu.get('gyro_z_dps', 0.0) or 0.0)
            deg2rad = math.pi / 180.0

        # Build odometry message
        odom = Odometry()
        stamp = self.get_clock().now().to_msg()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = pos_x
        odom.pose.pose.position.y = pos_y
        odom.pose.pose.position.z = pos_z
        odom.pose.pose.orientation = q

        # Covariance (tune as needed)
        odom.pose.covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.10, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.05,
        ]

        odom.twist.twist.linear.x = vx_body
        odom.twist.twist.linear.y = vy_body
        odom.twist.twist.linear.z = -vz  # NED -> up positive
        odom.twist.twist.angular.x = gx * deg2rad
        odom.twist.twist.angular.y = gy * deg2rad
        odom.twist.twist.angular.z = gz * deg2rad

        odom.twist.covariance = [
            0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.02, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.02, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.05,
        ]

        self.odom_pub.publish(odom)

        # Broadcast TF: odom -> base_link
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = pos_x
        t.transform.translation.y = pos_y
        t.transform.translation.z = pos_z
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

        # Publish optical flow for debugging
        flow_msg = Vector3Stamped()
        flow_msg.header.stamp = stamp
        flow_msg.header.frame_id = self.base_frame
        flow_msg.vector.x = vx_body
        flow_msg.vector.y = vy_body
        rf = telem.get('rangefinder') or {}
        flow_msg.vector.z = pos_z  # height used for /odom (see z_source)
        self.flow_pub.publish(flow_msg)

        fl = telem.get('optical_flow') or {}
        self.flow_q_pub.publish(Float32(data=float(fl.get('quality') or 0)))
        self.still_pub.publish(Float32(data=1.0 if stationary else 0.0))
        self.flow_h_pub.publish(Float32(data=float(fl.get('ground_distance_m') or 0.0)))

    def destroy_node(self):
        self._stop_event.set()
        if self._udp_thread and self._udp_thread.is_alive():
            self._udp_thread.join(timeout=1.0)
        self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MavlinkToRos2Bridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)   # 2nd Ctrl-C: no traceback
        node.destroy_node()
        if rclpy.ok():   # SIGINT already shut the context down
            rclpy.shutdown()


if __name__ == '__main__':
    main()
