#!/usr/bin/env python3
"""
pixhawk_controller.py - Modular MAVLink Controller for Pixhawk Autopilots
========================================================================
Designed for Raspberry Pi companion onboard compute on UAV / VTOL platforms.
Supports both ArduPilot (ArduCopter / ArduPlane / ArduVTOL) and PX4 Autopilot.

Capabilities:
  * Auto-detection of serial ports (/dev/ttyACM*, /dev/ttyUSB*, /dev/serial0)
  * Arm / Disarm with pre-arm feedback & force-arming option
  * Mode switching (ALT_HOLD, POSHOLD, LAND, STABILIZE, GUIDED, RTL, etc.)
  * Live sensor telemetry streaming (Attitude, IMU, GPS/GPS-denied, Baro, Battery, Distance/Lidar, Optical Flow)
  * Asynchronous background telemetry collection
"""

import sys
import os
import glob
import time
import math
import logging
import threading
from typing import Dict, Any, Optional, Tuple, List

try:
    from pymavlink import mavutil
except ImportError:
    print("ERROR: pymavlink is not installed. Install via: pip install pymavlink")
    sys.exit(1)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s"
)
logger = logging.getLogger("PixhawkController")


class PixhawkVehicle:
    """
    High-level interface to Pixhawk flight controller via MAVLink.
    """

    # Common default mode aliases for user friendliness
    MODE_ALIASES = {
        "alt": "ALT_HOLD",
        "alt_mode": "ALT_HOLD",
        "althold": "ALT_HOLD",
        "alt_hold": "ALT_HOLD",
        "altitude": "ALT_HOLD",
        "altitude_mode": "ALT_HOLD",
        "pos": "POSHOLD",
        "pos_mode": "POSHOLD",
        "poshold": "POSHOLD",
        "position": "POSHOLD",
        "position_mode": "POSHOLD",
        "loiter": "LOITER",
        "land": "LAND",
        "land_mode": "LAND",
        "stab": "STABILIZE",
        "stabilize": "STABILIZE",
        "stabilized": "STABILIZE",
        "guided": "GUIDED",
        "offboard": "GUIDED",  # Mapped according to autopilot
        "rtl": "RTL",
        "return_to_launch": "RTL",
        "auto": "AUTO",
        "brake": "BRAKE",
        "qland": "QLAND",
        "qhover": "QHOVER",
        "qloiter": "QLOITER",
        "qstabilize": "QSTABILIZE",
    }

    # Autopilot type names
    AUTOPILOT_MAP = {
        mavutil.mavlink.MAV_AUTOPILOT_GENERIC: "Generic",
        mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA: "ArduPilot",
        mavutil.mavlink.MAV_AUTOPILOT_PX4: "PX4",
    }

    # Vehicle type names
    VEHICLE_TYPE_MAP = {
        mavutil.mavlink.MAV_TYPE_QUADROTOR: "Quadrotor",
        mavutil.mavlink.MAV_TYPE_HEXAROTOR: "Hexarotor",
        mavutil.mavlink.MAV_TYPE_OCTOROTOR: "Octorotor",
        mavutil.mavlink.MAV_TYPE_FIXED_WING: "Fixed Wing",
        mavutil.mavlink.MAV_TYPE_VTOL_TILTROTOR: "VTOL (Tiltrotor)",
        mavutil.mavlink.MAV_TYPE_VTOL_DUOROTOR: "VTOL (Duorotor)",
        mavutil.mavlink.MAV_TYPE_VTOL_QUADROTOR: "VTOL (Quadplane)",
        mavutil.mavlink.MAV_TYPE_GROUND_ROVER: "Ground Rover",
    }

    # GPS Fix Types
    GPS_FIX_MAP = {
        0: "No GPS Connected",
        1: "No Fix",
        2: "2D Fix",
        3: "3D Fix",
        4: "DGPS Fix",
        5: "RTK Float",
        6: "RTK Fixed",
    }

    STALE_AFTER_S = 1.0  # navigation messages older than this are treated as missing

    # Messages the ground station's state estimate is built from. They are requested at
    # FAST_RATE_HZ via SET_MESSAGE_INTERVAL (the legacy REQUEST_DATA_STREAM groups only
    # give 10 Hz, i.e. up to 100 ms of extra age), and their arrival sets nav_data_event
    # so telemetry can be forwarded the moment it is fresh instead of on a fixed timer.
    NAV_MESSAGES = ("ATTITUDE", "LOCAL_POSITION_NED", "OPTICAL_FLOW", "RAW_IMU",
                    "DISTANCE_SENSOR")
    FAST_RATE_HZ = 30.0

    def __init__(
        self,
        connection_str: Optional[str] = None,
        baud: int = 115200,
        timeout: float = 10.0,
        auto_connect: bool = True
    ):
        """
        Initialize Pixhawk vehicle controller.

        :param connection_str: Serial port (e.g. /dev/ttyACM0), UDP (udp:127.0.0.1:14550), or None to auto-detect.
        :param baud: Baud rate for serial connections (default: 115200).
        :param timeout: Connection timeout in seconds.
        :param auto_connect: Whether to immediately establish connection.
        """
        self.connection_str = connection_str
        self.baud = baud
        self.timeout = timeout
        self.master = None
        self.target_system = 1
        self.target_component = 1
        self.autopilot_type = "Unknown"
        self.vehicle_type = "Unknown"
        self.is_connected = False

        # Latest raw messages store
        self._telemetry_lock = threading.Lock()
        self._raw_messages: Dict[str, Any] = {}
        self._msg_times: Dict[str, float] = {}   # monotonic receive time per message type
        self._recent_statustexts: List[str] = []
        # Per-command COMMAND_ACK cache: the periodic SET_MESSAGE_INTERVAL requests are
        # ACKed too, and with a single "last ACK" slot they could overwrite an ARM ACK.
        self._acks: Dict[int, Any] = {}
        # Set by the telemetry thread whenever a NAV_MESSAGES message arrives.
        self.nav_data_event = threading.Event()
        self.fast_rate_hz = self.FAST_RATE_HZ   # 0 disables the per-message rate requests

        # Background thread
        self._stop_telemetry_event = threading.Event()
        self._telemetry_thread: Optional[threading.Thread] = None
        self.raw_packet_callback = None

        if auto_connect:
            self.connect()

    @staticmethod
    def detect_serial_port() -> Optional[str]:
        """
        Scan Linux serial devices to find connected Pixhawk.
        Prioritizes persistent udev symlink /dev/pixhawk.
        """
        candidates = []
        # Priority 1: Persistent udev rule symlink
        if os.path.exists("/dev/pixhawk"):
            candidates.append("/dev/pixhawk")
        # Priority 2: Standard USB ACM devices (Pixhawk native USB)
        candidates.extend(sorted(glob.glob("/dev/ttyACM*")))
        # Priority 3: USB serial FTDI/CP210x devices
        candidates.extend(sorted(glob.glob("/dev/ttyUSB*")))
        # Priority 4: Raspberry Pi hardware UARTs
        if os.path.exists("/dev/serial0"):
            candidates.append("/dev/serial0")
        if os.path.exists("/dev/ttyAMA0"):
            candidates.append("/dev/ttyAMA0")

        # Never probe other devices' serial ports: opening the LiDAR's /dev/ttyUSB0 at
        # 115200 and reading it for 1.5 s steals its scan data (and toggles DTR/RTS).
        # start_streams.py lists them in GPS_DENIED_EXCLUDE_PORTS (':'-separated).
        excluded = {os.path.realpath(p) for p in
                    os.environ.get("GPS_DENIED_EXCLUDE_PORTS", "").split(":") if p}
        candidates = [p for p in dict.fromkeys(candidates)
                      if os.path.realpath(p) not in excluded]

        # Test each candidate
        for port in candidates:
            try:
                logger.debug("Testing candidate port: %s", port)
                conn = mavutil.mavlink_connection(port, baud=115200, timeout=1.0)
                hb = conn.wait_heartbeat(timeout=1.5)
                if hb:
                    logger.info("Found Pixhawk heartbeat on port: %s (Baud 115200)", port)
                    conn.close()
                    return port
                conn.close()
            except Exception:
                continue
        return None

    def connect(self) -> bool:
        """
        Connect to flight controller and verify heartbeat.
        """
        if not self.connection_str:
            logger.info("No connection string provided. Auto-detecting Pixhawk port...")
            detected = self.detect_serial_port()
            if detected:
                self.connection_str = detected
            else:
                self.connection_str = "/dev/ttyACM0"  # fallback default
                logger.warning("Auto-detection found no active heartbeat. Falling back to default %s", self.connection_str)

        logger.info("Connecting to Pixhawk at %s (Baud %d)...", self.connection_str, self.baud)
        try:
            self.master = mavutil.mavlink_connection(
                self.connection_str,
                baud=self.baud,
                source_system=255,
                source_component=0
            )
        except Exception as e:
            logger.error("Failed to open connection to %s: %s", self.connection_str, e)
            self.is_connected = False
            return False

        logger.info("Waiting for heartbeat from Pixhawk (timeout: %.1fs)...", self.timeout)
        hb = self.master.wait_heartbeat(timeout=self.timeout)
        if not hb:
            logger.error("Timed out waiting for heartbeat from Pixhawk.")
            self.is_connected = False
            return False

        # The first heartbeat may come from another component (GCS, gimbal, OSD, ...):
        # wait for the autopilot's own one.
        deadline = time.time() + self.timeout
        while not self._is_autopilot_heartbeat(hb) and time.time() < deadline:
            hb = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0) or hb
        # Take the IDs from the autopilot's heartbeat itself. pymavlink only learns the
        # SYSTEM id from heartbeats - master.target_component stays 0 - and the telemetry
        # thread drops heartbeats whose source != (target_system, target_component), so a
        # 0 here froze armed/flight_mode at their connect-time values forever.
        self.target_system = hb.get_srcSystem()
        self.target_component = hb.get_srcComponent()
        self.master.target_system = self.target_system
        self.master.target_component = self.target_component
        self.autopilot_type = self.AUTOPILOT_MAP.get(hb.autopilot, f"Autopilot ID {hb.autopilot}")
        self.vehicle_type = self.VEHICLE_TYPE_MAP.get(hb.type, f"Type ID {hb.type}")
        self.is_connected = True

        # Store initial heartbeat in message cache
        with self._telemetry_lock:
            self._raw_messages["HEARTBEAT"] = hb

        logger.info(
            "Connected to Pixhawk! Autopilot: %s | Vehicle: %s | System ID: %d | Comp ID: %d",
            self.autopilot_type, self.vehicle_type, self.target_system, self.target_component
        )

        # Request default data streams
        self.request_data_streams(rate_hz=10)

        # Start background telemetry receiver
        self.start_telemetry_thread()
        return True

    @staticmethod
    def _is_autopilot_heartbeat(hb) -> bool:
        return (hb is not None
                and hb.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID
                and hb.type not in (mavutil.mavlink.MAV_TYPE_GCS,
                                    mavutil.mavlink.MAV_TYPE_GIMBAL,
                                    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER))

    def disconnect(self):
        """
        Disconnect and stop background receiver.
        """
        self.stop_telemetry_thread()
        if self.master:
            try:
                self.master.close()
            except Exception:
                pass
            self.master = None
        self.is_connected = False
        logger.info("Disconnected from Pixhawk.")

    def request_data_streams(self, rate_hz: int = 10):
        """
        Request telemetry streams at specified frequency (Hz).
        Sends to target component 1 (autopilot), 0 (broadcast), and self.target_component.
        """
        if not self.master:
            return
        streams = [
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,
            mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
            mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,  # Attitude
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA2,  # VFR_HUD
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA3,  # Sensors / AHRS
        ]
        target_comps = {1, getattr(self, "target_component", 1), 0}
        for comp in target_comps:
            for s in streams:
                try:
                    self.master.mav.request_data_stream_send(
                        self.target_system,
                        comp,
                        s,
                        rate_hz,
                        1  # start stream
                    )
                except Exception as e:
                    logger.debug("Failed requesting stream %s on comp %s: %s", s, comp, e)
        # Must come AFTER the stream requests: ArduPilot resets per-message intervals of a
        # stream group whenever REQUEST_DATA_STREAM is (re)applied to that group.
        self.request_fast_messages()

    def request_fast_messages(self):
        """Ask the autopilot to send NAV_MESSAGES at fast_rate_hz (MAV_CMD_SET_MESSAGE_INTERVAL)."""
        if not self.master or not self.fast_rate_hz or self.fast_rate_hz <= 0:
            return
        interval_us = int(1e6 / self.fast_rate_hz)
        for name in self.NAV_MESSAGES:
            msg_id = getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{name}", None)
            if msg_id is None:
                continue
            try:
                self.master.mav.command_long_send(
                    self.target_system, self.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                    msg_id, interval_us, 0, 0, 0, 0, 0)
            except Exception as e:
                logger.debug("Failed setting interval for %s: %s", name, e)

    def start_telemetry_thread(self):
        """
        Start continuous background message listener.
        """
        if self._telemetry_thread and self._telemetry_thread.is_alive():
            return
        self._stop_telemetry_event.clear()
        self._telemetry_thread = threading.Thread(
            target=self._telemetry_worker,
            daemon=True,
            name="PixhawkTelemetryThread"
        )
        self._telemetry_thread.start()

    def stop_telemetry_thread(self):
        """
        Stop continuous background message listener.
        """
        self._stop_telemetry_event.set()
        if self._telemetry_thread and self._telemetry_thread.is_alive():
            self._telemetry_thread.join(timeout=1.0)
        self._telemetry_thread = None

    def _telemetry_worker(self):
        """
        Background loop reading incoming MAVLink packets.
        Periodically re-requests data streams to keep sensors streaming at full rate,
        and automatically handles USB re-enumeration (/dev/ttyACM*).
        """
        last_stream_req = time.time()
        last_msg_time = time.time()

        while not self._stop_telemetry_event.is_set():
            if not self.master:
                # Attempt auto-reconnect if disconnected
                if time.time() - last_msg_time > 2.0:
                    detected = self.detect_serial_port()
                    if detected and detected != self.connection_str:
                        logger.info("Auto-reconnecting to newly detected port: %s", detected)
                        self.connection_str = detected
                        self.connect()
                time.sleep(0.1)
                continue

            try:
                # Keep telemetry streams alive every 3.0 seconds
                now = time.time()
                if now - last_stream_req > 3.0:
                    self.request_data_streams(rate_hz=10)
                    last_stream_req = now

                msg = self.master.recv_msg()
                if msg is not None:
                    last_msg_time = now
                    mtype = msg.get_type()
                    # Only the autopilot's own HEARTBEAT defines armed state / flight mode;
                    # other components (camera, gimbal, OSD, ...) also send heartbeats.
                    foreign_hb = mtype == "HEARTBEAT" and (
                        msg.get_srcSystem() != self.target_system
                        or msg.get_srcComponent() != self.target_component)
                    with self._telemetry_lock:
                        if not foreign_hb:
                            self._raw_messages[mtype] = msg
                            self._msg_times[mtype] = time.monotonic()
                        if mtype == "COMMAND_ACK":
                            self._acks[getattr(msg, "command", -1)] = msg
                        if mtype == "STATUSTEXT":
                            text = getattr(msg, "text", "")
                            self._recent_statustexts.append(text)
                            if len(self._recent_statustexts) > 20:
                                self._recent_statustexts.pop(0)
                    if mtype in self.NAV_MESSAGES:
                        self.nav_data_event.set()

                    # Forward raw buffer to callback if registered
                    if self.raw_packet_callback:
                        try:
                            buf = msg.get_msgbuf()
                            if buf:
                                self.raw_packet_callback(buf)
                        except Exception:
                            pass
                else:
                    # If no message received for > 4 seconds, check if device vanished
                    if now - last_msg_time > 4.0 and self._try_reenumerate():
                        last_msg_time = time.time()
                    time.sleep(0.002)
            except Exception as e:
                # A vanished USB device makes the read RAISE (SerialException) rather than
                # return None, so the re-enumeration check must also run here.
                logger.debug("Telemetry worker error: %s", e)
                if time.time() - last_msg_time > 4.0 and self._try_reenumerate():
                    last_msg_time = time.time()
                time.sleep(0.05)

    def _try_reenumerate(self) -> bool:
        """The link has been silent for > 4 s (the autopilot sends HEARTBEAT at 1 Hz):
        re-open it. Also when the device path still exists - after a quick USB
        re-enumeration /dev/pixhawk (or ttyACM0) is back, but our file descriptor
        belongs to the vanished device and would never deliver data again."""
        now = time.monotonic()
        if now - getattr(self, "_last_reopen", 0.0) < 3.0:
            return False
        self._last_reopen = now
        if self.connection_str and os.path.exists(self.connection_str):
            logger.warning("No MAVLink data from %s for > 4 s; re-opening the port...",
                           self.connection_str)
            port = self.connection_str
        else:
            logger.warning("Device %s disappeared! Scanning for new port...", self.connection_str)
            port = None
        try:
            self.master.close()
        except Exception:
            pass
        if port is None:
            port = self.detect_serial_port()
            if not port:
                return False
            logger.info("Found re-enumerated Pixhawk on %s! Reconnecting...", port)
            self.connection_str = port
        try:
            self.master = mavutil.mavlink_connection(
                port, baud=self.baud, source_system=255, source_component=0)
        except Exception as e:
            logger.warning("Re-open of %s failed: %s", port, e)
            return False
        if not self.master.wait_heartbeat(timeout=3.0):
            logger.warning("Re-opened %s but no heartbeat yet; will retry", port)
            return False
        logger.info("Pixhawk link restored on %s", port)
        self.request_data_streams(rate_hz=10)
        return True

    def _last_ack(self, command):
        """Latest COMMAND_ACK received for ``command`` (None if none yet)."""
        with self._telemetry_lock:
            return self._acks.get(command)

    def _new_ack(self, prev, command):
        """COMMAND_ACK for ``command`` received since ``prev`` (from the telemetry thread's
        per-command cache). Never call master.recv_match() outside the telemetry thread: two
        threads reading the same port race and the ACK is swallowed by the other reader."""
        ack = self._last_ack(command)
        if ack is not None and ack is not prev:
            return ack
        return None

    @property
    def is_armed(self) -> bool:
        """
        Check if vehicle is currently armed.
        """
        with self._telemetry_lock:
            hb = self._raw_messages.get("HEARTBEAT")
        if hb:
            return bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        if self.master:
            return bool(self.master.motors_armed())
        return False

    @property
    def flight_mode(self) -> str:
        """
        Get current flight mode string name.
        """
        with self._telemetry_lock:
            hb = self._raw_messages.get("HEARTBEAT")

        if hb and self.master:
            mode_map = self.master.mode_mapping()
            if mode_map:
                inv_map = {v: k for k, v in mode_map.items()}
                if hb.custom_mode in inv_map:
                    return inv_map[hb.custom_mode]
            return f"MODE_{hb.custom_mode}"

        if self.master and hasattr(self.master, "flightmode") and self.master.flightmode:
            return str(self.master.flightmode).upper()

        return "UNKNOWN"

    def get_available_modes(self) -> List[str]:
        """
        Return list of supported flight mode names for this autopilot.
        """
        if self.master:
            mode_map = self.master.mode_mapping()
            if mode_map:
                return sorted(list(mode_map.keys()))
        return ["STABILIZE", "ALT_HOLD", "POSHOLD", "LAND", "GUIDED", "RTL", "AUTO", "LOITER"]

    def arm(self, force: bool = False, timeout: float = 5.0) -> Tuple[bool, str]:
        """
        Arm the vehicle motors.

        :param force: If True, bypass pre-arm safety checks (param2=21196 in ArduPilot).
        :param timeout: Maximum seconds to wait for confirmation.
        :return: (success: bool, status_message: str)
        """
        if not self.is_connected or not self.master:
            return False, "Not connected to Pixhawk"

        if self.is_armed:
            return True, "Vehicle is already armed"

        force_param = 21196 if force else 0
        logger.info("Sending ARM command (force=%s)...", force)

        # Clear recent statustexts to catch fresh error messages
        with self._telemetry_lock:
            self._recent_statustexts.clear()

        prev_ack = self._last_ack(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
        # Send command MAV_CMD_COMPONENT_ARM_DISARM (param1=1 to arm)
        self.master.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,             # confirmation
            1,             # param1: 1 = arm, 0 = disarm
            force_param,   # param2: force arm magic value
            0, 0, 0, 0, 0
        )

        start = time.time()
        ack_received = False
        ack_result = None

        while time.time() - start < timeout:
            if self.is_armed:
                return True, "Vehicle successfully ARMED"

            msg = self._new_ack(prev_ack, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
            if msg:
                ack_received = True
                ack_result = msg.result
                if ack_result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    # Give heartbeat a brief moment to update
                    time.sleep(0.3)
                    if self.is_armed:
                        return True, "Vehicle successfully ARMED (ACK accepted)"

            time.sleep(0.05)

        # If arming failed, fetch recent status texts from Pixhawk
        with self._telemetry_lock:
            reasons = list(self._recent_statustexts)

        reason_str = " | ".join(reasons) if reasons else "No PreArm error detail returned"
        if ack_result == mavutil.mavlink.MAV_RESULT_DENIED:
            return False, f"Arming DENIED by Pixhawk: {reason_str}"
        elif ack_result == mavutil.mavlink.MAV_RESULT_FAILED:
            return False, f"Arming FAILED: {reason_str}"
        return False, f"Arming timed out: {reason_str}"

    def disarm(self, force: bool = False, timeout: float = 5.0) -> Tuple[bool, str]:
        """
        Disarm the vehicle motors.

        :param force: If True, force disarm immediately even if airborne/in motion.
        :param timeout: Maximum seconds to wait for confirmation.
        :return: (success: bool, status_message: str)
        """
        if not self.is_connected or not self.master:
            return False, "Not connected to Pixhawk"

        if not self.is_armed:
            return True, "Vehicle is already disarmed"

        force_param = 21196 if force else 0
        logger.info("Sending DISARM command (force=%s)...", force)

        prev_ack = self._last_ack(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
        # Send command MAV_CMD_COMPONENT_ARM_DISARM (param1=0 to disarm)
        self.master.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,             # confirmation
            0,             # param1: 0 = disarm
            force_param,   # param2: force disarm
            0, 0, 0, 0, 0
        )

        start = time.time()
        while time.time() - start < timeout:
            if not self.is_armed:
                return True, "Vehicle successfully DISARMED"

            msg = self._new_ack(prev_ack, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
            if msg:
                if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    time.sleep(0.3)
                    if not self.is_armed:
                        return True, "Vehicle successfully DISARMED (ACK accepted)"

            time.sleep(0.05)

        with self._telemetry_lock:
            reasons = list(self._recent_statustexts)
        reason_str = " | ".join(reasons) if reasons else "Timeout waiting for disarm"
        return False, f"Disarming failed or timed out: {reason_str}"

    def set_mode(self, mode_name: str, timeout: float = 4.0) -> Tuple[bool, str]:
        """
        Change Pixhawk flight mode.

        :param mode_name: Mode name (e.g. ALT_HOLD, POSHOLD, LAND, STABILIZE, GUIDED).
                          User aliases like 'alt', 'position', 'land_mode' are automatically mapped.
        :param timeout: Max seconds to wait for confirmation.
        :return: (success: bool, message: str)
        """
        if not self.is_connected or not self.master:
            return False, "Not connected to Pixhawk"

        clean_name = mode_name.strip().upper()
        # Check alias
        alias = self.MODE_ALIASES.get(mode_name.strip().lower())
        target_mode = alias if alias else clean_name

        mode_map = self.master.mode_mapping()
        if not mode_map:
            return False, "Autopilot mode mapping unavailable"

        # Check if target mode exists directly in mode mapping
        if target_mode not in mode_map:
            # Fallback check: PX4 vs ArduPilot alternate names
            if target_mode == "POSHOLD" and "LOITER" in mode_map:
                target_mode = "LOITER"
            elif target_mode == "LOITER" and "POSHOLD" in mode_map:
                target_mode = "POSHOLD"
            elif target_mode == "ALT_HOLD" and "ALTCTL" in mode_map:
                target_mode = "ALTCTL"
            elif target_mode == "POSHOLD" and "POSCTL" in mode_map:
                target_mode = "POSCTL"
            elif target_mode == "GUIDED" and "OFFBOARD" in mode_map:
                target_mode = "OFFBOARD"

        if target_mode not in mode_map:
            avail = ", ".join(sorted(mode_map.keys()))
            return False, f"Invalid mode '{mode_name}'. Available modes: {avail}"

        mode_id = mode_map[target_mode]
        logger.info("Switching flight mode to %s (ID: %d)...", target_mode, mode_id)

        # Send set_mode command
        self.master.mav.set_mode_send(
            self.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )

        start = time.time()
        while time.time() - start < timeout:
            current = self.flight_mode
            if current.upper() == target_mode.upper():
                return True, f"Flight mode successfully set to {target_mode}"
            time.sleep(0.05)

        with self._telemetry_lock:
            reasons = list(self._recent_statustexts)
        reason_str = f" | Detail: {' '.join(reasons)}" if reasons else ""
        return False, f"Failed or timed out setting mode to {target_mode}. Current mode is {self.flight_mode}{reason_str}"

    def emergency_kill(self) -> Tuple[bool, str]:
        """
        Emergency Motor Kill / Cutoff. Forces immediate disarm.
        """
        logger.warning("EMERGENCY MOTOR CUTOFF TRIGGERED!")
        return self.disarm(force=True, timeout=3.0)

    def takeoff(self, altitude_m: float = 2.0, timeout: float = 8.0) -> Tuple[bool, str]:
        """
        Command autonomous takeoff to target altitude (requires GUIDED / NAV mode).

        :param altitude_m: Target takeoff altitude in meters.
        :param timeout: Seconds to wait for command acknowledgment.
        """
        if not self.is_connected or not self.master:
            return False, "Not connected to Pixhawk"

        if not self.is_armed:
            return False, "Vehicle must be armed before takeoff"

        logger.info("Sending TAKEOFF command to %.1f meters...", altitude_m)
        prev_ack = self._last_ack(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)
        self.master.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0,
            altitude_m
        )

        start = time.time()
        while time.time() - start < timeout:
            msg = self._new_ack(prev_ack, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)
            if msg:
                if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    return True, f"Takeoff command accepted for {altitude_m:.1f}m altitude"
                return False, f"Takeoff command rejected (ACK code: {msg.result})"
            time.sleep(0.05)

        return False, "Takeoff command timed out waiting for ACK"

    def get_telemetry(self) -> Dict[str, Any]:
        """
        Retrieve structured dictionary of all available sensor telemetry.
        """
        with self._telemetry_lock:
            msgs = dict(self._raw_messages)
            times = dict(self._msg_times)

        # A sensor that stops streaming must read as "missing", not as its last value
        # forever - otherwise the ROS bridge keeps publishing a frozen EKF position.
        now_m = time.monotonic()
        for k in ("LOCAL_POSITION_NED", "OPTICAL_FLOW", "OPTICAL_FLOW_RAD", "DISTANCE_SENSOR"):
            if k in msgs and now_m - times.get(k, 0.0) > self.STALE_AFTER_S:
                del msgs[k]

        hb = msgs.get("HEARTBEAT")
        att = msgs.get("ATTITUDE")
        imu = msgs.get("RAW_IMU") or msgs.get("SCALED_IMU") or msgs.get("HIGHRES_IMU")
        press = msgs.get("SCALED_PRESSURE")
        gps = msgs.get("GPS_RAW_INT")
        gpos = msgs.get("GLOBAL_POSITION_INT")
        lpos = msgs.get("LOCAL_POSITION_NED")
        hud = msgs.get("VFR_HUD")
        sys_stat = msgs.get("SYS_STATUS")
        pwr = msgs.get("POWER_STATUS")
        dist = msgs.get("DISTANCE_SENSOR")
        flow = msgs.get("OPTICAL_FLOW") or msgs.get("OPTICAL_FLOW_RAD")
        vibe = msgs.get("VIBRATION")
        ekf = msgs.get("EKF_STATUS_REPORT")
        rc = msgs.get("RC_CHANNELS")

        telemetry: Dict[str, Any] = {
            "timestamp": time.time(),
            "connected": self.is_connected,
            "armed": self.is_armed,
            "flight_mode": self.flight_mode,
            "autopilot": self.autopilot_type,
            "vehicle_type": self.vehicle_type,
        }

        # 1. Attitude
        if att:
            telemetry["attitude"] = {
                "roll_deg": round(math.degrees(att.roll), 2),
                "pitch_deg": round(math.degrees(att.pitch), 2),
                "yaw_deg": round(math.degrees(att.yaw), 2),
                "rollspeed_dps": round(math.degrees(att.rollspeed), 2),
                "pitchspeed_dps": round(math.degrees(att.pitchspeed), 2),
                "yawspeed_dps": round(math.degrees(att.yawspeed), 2),
            }
        else:
            telemetry["attitude"] = None

        # 2. IMU Sensors
        if imu:
            telemetry["imu"] = {
                "accel_x_mg": getattr(imu, "xacc", 0),
                "accel_y_mg": getattr(imu, "yacc", 0),
                "accel_z_mg": getattr(imu, "zacc", 0),
                "gyro_x_dps": round(getattr(imu, "xgyro", 0) * 0.0572958, 2),
                "gyro_y_dps": round(getattr(imu, "ygyro", 0) * 0.0572958, 2),
                "gyro_z_dps": round(getattr(imu, "zgyro", 0) * 0.0572958, 2),
                "mag_x_mgauss": getattr(imu, "xmag", 0),
                "mag_y_mgauss": getattr(imu, "ymag", 0),
                "mag_z_mgauss": getattr(imu, "zmag", 0),
            }
        else:
            telemetry["imu"] = None

        # 3. Barometer & Pressure
        if press:
            p_abs = press.press_abs
            std_alt = round(44330.0 * (1.0 - (p_abs / 1013.25) ** 0.190295), 2) if p_abs > 0 else 0.0
            telemetry["barometer"] = {
                "pressure_hpa": round(p_abs, 2),
                "temperature_c": round(press.temperature / 100.0, 2),
                "baro_alt_m": std_alt,
            }
        else:
            telemetry["barometer"] = None

        # 4. Battery & Power
        if sys_stat:
            telemetry["battery"] = {
                "voltage_v": round(sys_stat.voltage_battery / 1000.0, 2),
                "current_a": round(sys_stat.current_battery / 100.0, 2) if sys_stat.current_battery >= 0 else None,
                "remaining_pct": sys_stat.battery_remaining if sys_stat.battery_remaining >= 0 else None,
            }
        else:
            telemetry["battery"] = None

        # 5. GPS Status
        if gps:
            telemetry["gps"] = {
                "fix_type": gps.fix_type,
                "fix_name": self.GPS_FIX_MAP.get(gps.fix_type, f"Fix {gps.fix_type}"),
                "satellites_visible": gps.satellites_visible,
                "hdop": round(gps.eph / 100.0, 2) if getattr(gps, "eph", 65535) != 65535 else None,
                "vdop": round(gps.epv / 100.0, 2) if getattr(gps, "epv", 65535) != 65535 else None,
            }
        else:
            telemetry["gps"] = None

        # 6. Global Position (WGS-84)
        if gpos:
            telemetry["global_position"] = {
                "latitude_deg": round(gpos.lat / 1e7, 7),
                "longitude_deg": round(gpos.lon / 1e7, 7),
                "altitude_msl_m": round(gpos.alt / 1000.0, 2),
                "relative_alt_m": round(gpos.relative_alt / 1000.0, 2),
                "vx_ms": round(gpos.vx / 100.0, 2),
                "vy_ms": round(gpos.vy / 100.0, 2),
                "vz_ms": round(gpos.vz / 100.0, 2),
                "heading_deg": round(gpos.hdg / 100.0, 1),
            }
        else:
            telemetry["global_position"] = None

        # 7. Local Position NED (Essential for GPS-Denied flight)
        if lpos:
            telemetry["local_position_ned"] = {
                "x_m": round(lpos.x, 3),
                "y_m": round(lpos.y, 3),
                "z_m": round(lpos.z, 3),
                "vx_ms": round(lpos.vx, 3),
                "vy_ms": round(lpos.vy, 3),
                "vz_ms": round(lpos.vz, 3),
            }
        else:
            telemetry["local_position_ned"] = None

        # 8. VFR HUD
        if hud:
            telemetry["vfr_hud"] = {
                "airspeed_ms": round(hud.airspeed, 2),
                "groundspeed_ms": round(hud.groundspeed, 2),
                "heading_deg": int(hud.heading),
                "throttle_pct": int(hud.throttle),
                "alt_m": round(hud.alt, 2),
                "climb_ms": round(hud.climb, 2),
            }
        else:
            telemetry["vfr_hud"] = None

        # 9. Distance Sensor / Rangefinder (Lidar/Sonar for GPS-denied altitude)
        if dist:
            telemetry["rangefinder"] = {
                "distance_m": round(dist.current_distance / 100.0, 3),
                "min_distance_m": round(dist.min_distance / 100.0, 2),
                "max_distance_m": round(dist.max_distance / 100.0, 2),
                "orientation": getattr(dist, "orientation", 0),
            }
        else:
            telemetry["rangefinder"] = None

        # 10. Optical Flow (For GPS-denied position holding)
        # OPTICAL_FLOW.flow_x/y are raw sensor units (dpix), NOT m/s. Metric velocity is
        # flow_comp_m_x/y. For OPTICAL_FLOW_RAD we derive it: rad/s * distance.
        if flow:
            vel_x = vel_y = None
            if hasattr(flow, "flow_comp_m_x"):
                gdist = getattr(flow, "ground_distance", 0.0) or 0.0
                if self.autopilot_type == "ArduPilot":
                    # ArduPilot does NOT follow the spec here: flow_comp_m_x/y are the
                    # gyro-compensated angular rates (flowRate - bodyRate) in rad/s, each a
                    # rotation ABOUT that axis, and ground_distance is the EKF HAGL (0 when
                    # unknown, e.g. indoors without an EKF origin). Convert with the EKF3
                    # model (AP_NavEKF3_OptFlowFusion: comp_x = vy/h, comp_y = -vx/h, FRD).
                    h = gdist if gdist > 0 else (dist.current_distance / 100.0 if dist else 0.0)
                    if h > 0:
                        gdist = h
                        vel_x = -flow.flow_comp_m_y * h
                        vel_y = flow.flow_comp_m_x * h
                else:
                    vel_x, vel_y = flow.flow_comp_m_x, flow.flow_comp_m_y   # spec: m/s
            else:
                dt_us = getattr(flow, "integration_time_us", 0)
                gdist = getattr(flow, "distance", 0.0)
                if dt_us > 0 and gdist > 0:
                    # MAVLink spec: integrated_x is rotation ABOUT X (linear +Y motion -> negative
                    # flow); integrated_y is rotation about Y (linear +X motion -> positive flow).
                    # Subtract the gyro-integrated body rotation so only translation remains.
                    dt = dt_us * 1e-6
                    rot_x = flow.integrated_x - getattr(flow, "integrated_xgyro", 0.0)
                    rot_y = flow.integrated_y - getattr(flow, "integrated_ygyro", 0.0)
                    vel_x = rot_y / dt * gdist
                    vel_y = -rot_x / dt * gdist
            telemetry["optical_flow"] = {
                "flow_x": getattr(flow, "flow_x", getattr(flow, "integrated_x", None)),
                "flow_y": getattr(flow, "flow_y", getattr(flow, "integrated_y", None)),
                "vel_x_ms": None if vel_x is None else round(float(vel_x), 3),
                "vel_y_ms": None if vel_y is None else round(float(vel_y), 3),
                "quality": getattr(flow, "quality", 0),
                "ground_distance_m": round(float(gdist), 3),
            }
        else:
            telemetry["optical_flow"] = None

        # 11. Vibration
        if vibe:
            telemetry["vibration"] = {
                "vibe_x": round(vibe.vibration_x, 2),
                "vibe_y": round(vibe.vibration_y, 2),
                "vibe_z": round(vibe.vibration_z, 2),
                "clipping_0": getattr(vibe, "clipping_0", 0),
                "clipping_1": getattr(vibe, "clipping_1", 0),
                "clipping_2": getattr(vibe, "clipping_2", 0),
            }
        else:
            telemetry["vibration"] = None

        # 12. EKF Status
        if ekf:
            telemetry["ekf_status"] = {
                "flags": getattr(ekf, "flags", 0),
                "velocity_variance": round(getattr(ekf, "velocity_variance", 0.0), 3),
                "pos_horiz_variance": round(getattr(ekf, "pos_horiz_variance", 0.0), 3),
                "pos_vert_variance": round(getattr(ekf, "pos_vert_variance", 0.0), 3),
                "compass_variance": round(getattr(ekf, "compass_variance", 0.0), 3),
            }
        else:
            telemetry["ekf_status"] = None

        # 13. RC Channels
        if rc:
            telemetry["rc_channels"] = {
                f"ch{i}": getattr(rc, f"chan{i}_raw", None)
                for i in range(1, 9)
            }
        else:
            telemetry["rc_channels"] = None

        # 14. Status Messages / Alerts
        with self._telemetry_lock:
            telemetry["statustexts"] = list(self._recent_statustexts)

        return telemetry


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Pixhawk Controller Diagnostic & Status")
    parser.add_argument("--port", type=str, default=None, help="Serial port (default: auto-detect)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    args = parser.parse_args()

    print("=== Initializing Pixhawk Controller ===")
    vehicle = PixhawkVehicle(connection_str=args.port, baud=args.baud)
    if not vehicle.is_connected:
        print("Failed to connect to Pixhawk.")
        sys.exit(1)

    print("\nVehicle Status:")
    print(f"  Autopilot:     {vehicle.autopilot_type}")
    print(f"  Vehicle Type:  {vehicle.vehicle_type}")
    print(f"  Armed:         {vehicle.is_armed}")
    print(f"  Flight Mode:   {vehicle.flight_mode}")
    print(f"  Supported Modes: {', '.join(vehicle.get_available_modes())}")

    print("\nFetching Sensor Telemetry Snapshot...")
    time.sleep(1.0)
    telem = vehicle.get_telemetry()

    if telem["attitude"]:
        att = telem["attitude"]
        print(f"  Attitude: Roll={att['roll_deg']}°, Pitch={att['pitch_deg']}°, Yaw={att['yaw_deg']}°")
    if telem["barometer"]:
        baro = telem["barometer"]
        print(f"  Barometer: {baro['pressure_hpa']} hPa | Temp: {baro['temperature_c']}°C")
    if telem["battery"]:
        bat = telem["battery"]
        print(f"  Battery: {bat['voltage_v']}V | Remaining: {bat['remaining_pct']}%")
    if telem["gps"]:
        gps = telem["gps"]
        print(f"  GPS: {gps['fix_name']} | Satellites: {gps['satellites_visible']}")
    if telem["vfr_hud"]:
        hud = telem["vfr_hud"]
        print(f"  HUD: Alt={hud['alt_m']}m | Heading={hud['heading_deg']}° | Throttle={hud['throttle_pct']}%")

    vehicle.disconnect()
    print("\nDiagnostics complete.")
