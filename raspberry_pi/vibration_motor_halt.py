#!/usr/bin/env python3
"""
vibration_motor_halt.py - Pixhawk Vibration Abnormality Guard & Motor Halt Controller
====================================================================================
Monitors real-time Pixhawk IMU vibration and accelerometer clipping.
When vibration exceeds safety thresholds (indicating frame abnormality, loose prop,
motor failure, or mechanical resonance):
  1. Clamps all motors strictly to 40.0% throttle (calibrated PWM 1400 µs).
  2. Overrides and locks out FlySky RC transmitter input so pilot stick movements
     cannot command higher throttle or destabilize the vehicle.
  3. Levels Roll and Pitch attitude (1500 µs) to prevent uncontrolled tipping.
  4. Keeps streaming continuous 20 Hz MAVLink overrides to prevent autopilot timeout.
  5. Provides live interactive controls: [t] test trigger, [r] release override,
     [d] emergency disarm, [q] safe exit.

Can be run on the Raspberry Pi companion computer or directly on a Linux laptop.
"""

import sys
import os
import glob
import time
import math
import select
import termios
import tty
import signal
import threading
import argparse
from typing import Optional, Dict, Any

try:
    from pymavlink import mavutil
except ImportError:
    print("ERROR: pymavlink is not installed! Run: pip install pymavlink")
    sys.exit(1)


# ANSI Terminal Colors
C_RESET  = "\033[0m"
C_BOLD   = "\033[1m"
C_RED    = "\033[91m"
C_GREEN  = "\033[92m"
C_YELLOW = "\033[93m"
C_BLUE   = "\033[94m"
C_MAGENTA= "\033[95m"
C_CYAN   = "\033[96m"
C_WHITE  = "\033[97m"
C_BG_RED = "\033[41m"
C_BG_GREEN = "\033[42m"
C_CLEAR  = "\033[2J\033[H"


def detect_pixhawk_port() -> str:
    """Find connected Pixhawk device, prioritizing /dev/pixhawk udev symlink."""
    candidates = []
    if os.path.exists("/dev/pixhawk"):
        candidates.append("/dev/pixhawk")
    candidates.extend(sorted(glob.glob("/dev/ttyACM*")))
    candidates.extend(sorted(glob.glob("/dev/ttyUSB*")))
    if os.path.exists("/dev/serial0"):
        candidates.append("/dev/serial0")
    if os.path.exists("/dev/ttyAMA0"):
        candidates.append("/dev/ttyAMA0")

    for port in candidates:
        try:
            conn = mavutil.mavlink_connection(port, baud=115200, timeout=1.0)
            hb = conn.wait_heartbeat(timeout=1.2)
            conn.close()
            if hb:
                return port
        except Exception:
            continue
    return "/dev/pixhawk" if os.path.exists("/dev/pixhawk") else "/dev/ttyACM0"


class VibrationMotorGuard:
    def __init__(
        self,
        device: str,
        baud: int = 115200,
        threshold_vibe: float = 35.0,
        halt_throttle_pct: float = 40.0,
        consecutive_samples: int = 3,
        auto_alt_hold: bool = False,
    ):
        self.device = device
        self.baud = baud
        self.threshold_vibe = threshold_vibe
        self.halt_throttle_pct = halt_throttle_pct
        self.consecutive_samples = consecutive_samples
        self.auto_alt_hold = auto_alt_hold

        self.conn = None
        self.target_system = 1
        self.target_component = 1
        self.running = True

        # Throttle calibration defaults
        self.rc3_min = 1000
        self.rc3_max = 2000
        self.halt_pwm = int(self.rc3_min + (self.halt_throttle_pct / 100.0) * (self.rc3_max - self.rc3_min))

        # Real-time state
        self.lock = threading.Lock()
        self.armed = False
        self.flight_mode = "UNKNOWN"
        self.vibe_x = 0.0
        self.vibe_y = 0.0
        self.vibe_z = 0.0
        self.vibe_mag = 0.0
        self.max_vibe_mag = 0.0
        self.clipping_0 = 0
        self.clipping_1 = 0
        self.clipping_2 = 0
        self.init_clipping_total = None
        self.clipping_burst = 0

        # Raw FlySky RC inputs (channels 1-4)
        self.rc_in_roll = 1500
        self.rc_in_pitch = 1500
        self.rc_in_throttle = 1000
        self.rc_in_yaw = 1500

        # Guard status
        self.halt_triggered = False
        self.trigger_reason = ""
        self.trigger_time = 0.0
        self.samples_above_threshold = 0

        # Background threads
        self.telemetry_thread = None
        self.override_thread = None

    def connect(self) -> bool:
        """Establish MAVLink connection and fetch parameters."""
        print(f"{C_CYAN}[+] Connecting to Pixhawk on {self.device} (baud: {self.baud})...{C_RESET}")
        try:
            self.conn = mavutil.mavlink_connection(
                self.device,
                baud=self.baud,
                source_system=255,
                source_component=0
            )
        except Exception as e:
            print(f"{C_RED}[!] Error opening connection: {e}{C_RESET}")
            return False

        hb = self.conn.wait_heartbeat(timeout=6.0)
        if not hb:
            print(f"{C_RED}[!] Timed out waiting for Pixhawk heartbeat!{C_RESET}")
            return False

        self.target_system = self.conn.target_system
        self.target_component = self.conn.target_component
        print(f"{C_GREEN}[+] Heartbeat received! System ID: {self.target_system}, Component ID: {self.target_component}{C_RESET}")

        # Request calibration parameters (RC3 min/max for accurate 40% PWM)
        self._fetch_calibration_params()

        # Request continuous telemetry streams (20 Hz)
        self.request_data_streams(rate_hz=20)
        return True

    def _fetch_calibration_params(self):
        """Query Pixhawk RC3_MIN and RC3_MAX for exact throttle calculation."""
        try:
            self.conn.mav.param_request_read_send(self.target_system, self.target_component, b"RC3_MIN", -1)
            msg = self.conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.5)
            if msg and msg.param_id.startswith("RC3_MIN"):
                self.rc3_min = int(msg.param_value)

            self.conn.mav.param_request_read_send(self.target_system, self.target_component, b"RC3_MAX", -1)
            msg = self.conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.5)
            if msg and msg.param_id.startswith("RC3_MAX"):
                self.rc3_max = int(msg.param_value)

            self.halt_pwm = int(self.rc3_min + (self.halt_throttle_pct / 100.0) * (self.rc3_max - self.rc3_min))
            print(f"{C_CYAN}[+] RC3 Throttle Range: [{self.rc3_min} µs - {self.rc3_max} µs] -> {self.halt_throttle_pct:.1f}% Halt Target = {self.halt_pwm} µs{C_RESET}")
        except Exception as e:
            print(f"{C_YELLOW}[!] Using default throttle PWM: {self.halt_pwm} µs ({e}){C_RESET}")

    def request_data_streams(self, rate_hz: int = 20):
        """Request high-rate vibration, telemetry, and RC channel streams."""
        streams = [
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA3,      # VIBRATION & AHRS
            mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, # FlySky channel inputs
            mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS, # IMU sensors
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,      # Attitude
        ]
        for s in streams:
            try:
                self.conn.mav.request_data_stream_send(
                    self.target_system,
                    self.target_component,
                    s,
                    rate_hz,
                    1
                )
            except Exception:
                pass

    def start(self):
        """Start monitoring and override control threads."""
        self.telemetry_thread = threading.Thread(target=self._telemetry_worker, daemon=True, name="TelemetryWorker")
        self.override_thread = threading.Thread(target=self._override_worker, daemon=True, name="OverrideWorker")
        self.telemetry_thread.start()
        self.override_thread.start()

    def _telemetry_worker(self):
        """Continuously parse incoming MAVLink packets and check vibration levels."""
        last_req = time.time()
        while self.running and self.conn:
            try:
                # Keep telemetry stream alive every 3s
                now = time.time()
                if now - last_req > 3.0:
                    self.request_data_streams(rate_hz=20)
                    last_req = now

                msg = self.conn.recv_msg()
                if not msg:
                    time.sleep(0.005)
                    continue

                mtype = msg.get_type()

                if mtype == "HEARTBEAT":
                    armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    mode_map = self.conn.mode_mapping()
                    mode_name = "UNKNOWN"
                    if mode_map:
                        inv = {v: k for k, v in mode_map.items()}
                        mode_name = inv.get(msg.custom_mode, f"MODE_{msg.custom_mode}")
                    with self.lock:
                        self.armed = armed
                        self.flight_mode = mode_name

                elif mtype == "VIBRATION":
                    vx = msg.vibration_x
                    vy = msg.vibration_y
                    vz = msg.vibration_z
                    vmag = math.sqrt(vx * vx + vy * vy + vz * vz)
                    c0 = getattr(msg, "clipping_0", 0)
                    c1 = getattr(msg, "clipping_1", 0)
                    c2 = getattr(msg, "clipping_2", 0)
                    c_total = c0 + c1 + c2

                    with self.lock:
                        self.vibe_x = vx
                        self.vibe_y = vy
                        self.vibe_z = vz
                        self.vibe_mag = vmag
                        if vmag > self.max_vibe_mag:
                            self.max_vibe_mag = vmag
                        self.clipping_0 = c0
                        self.clipping_1 = c1
                        self.clipping_2 = c2

                        if self.init_clipping_total is None:
                            self.init_clipping_total = c_total
                        self.clipping_burst = c_total - self.init_clipping_total

                        # Check for abnormality
                        if not self.halt_triggered:
                            # Condition A: Sustained vibration above threshold
                            if vmag >= self.threshold_vibe:
                                self.samples_above_threshold += 1
                                if self.samples_above_threshold >= self.consecutive_samples:
                                    self._trigger_halt_internal(f"Excessive Vibration: {vmag:.1f} m/s² >= {self.threshold_vibe:.1f} m/s²")
                            # Condition B: Severe spike (>60 m/s²)
                            elif vmag >= 60.0:
                                self._trigger_halt_internal(f"Critical Shock/Vibration Spike: {vmag:.1f} m/s²")
                            # Condition C: Accelerometer clipping burst
                            elif self.clipping_burst > 15:
                                self._trigger_halt_internal(f"Accelerometer Clipping Burst: +{self.clipping_burst} events")
                            else:
                                self.samples_above_threshold = 0

                elif mtype == "RC_CHANNELS":
                    with self.lock:
                        # Raw values from FlySky receiver
                        self.rc_in_roll = getattr(msg, "chan1_raw", 1500)
                        self.rc_in_pitch = getattr(msg, "chan2_raw", 1500)
                        self.rc_in_throttle = getattr(msg, "chan3_raw", 1000)
                        self.rc_in_yaw = getattr(msg, "chan4_raw", 1500)

            except Exception:
                time.sleep(0.01)

    def _trigger_halt_internal(self, reason: str):
        """Activate the halt state (must be called with self.lock or sets flag safely)."""
        self.halt_triggered = True
        self.trigger_reason = reason
        self.trigger_time = time.time()

        # Optional switch to ALT_HOLD / STABILIZE mode
        if self.auto_alt_hold and self.conn:
            try:
                mode_map = self.conn.mode_mapping()
                if mode_map and "ALT_HOLD" in mode_map:
                    self.conn.mav.set_mode_send(
                        self.target_system,
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                        mode_map["ALT_HOLD"]
                    )
            except Exception:
                pass

    def trigger_halt(self, reason: str = "Manual Operator Trigger"):
        """Public method to manually trigger motor halt and FlySky lockout."""
        with self.lock:
            self._trigger_halt_internal(reason)

    def release_override(self):
        """Release RC override and return full control back to the FlySky transmitter."""
        with self.lock:
            self.halt_triggered = False
            self.trigger_reason = ""
            self.samples_above_threshold = 0
            self.init_clipping_total = self.clipping_0 + self.clipping_1 + self.clipping_2

        # Send release packets (0 disables override on that channel)
        if self.conn:
            for _ in range(5):
                try:
                    self.conn.mav.rc_channels_override_send(
                        self.target_system,
                        self.target_component,
                        0, 0, 0, 0, 0, 0, 0, 0
                    )
                except Exception:
                    pass
                time.sleep(0.02)

    def emergency_disarm(self) -> bool:
        """Immediately force-disarms Pixhawk motors."""
        if not self.conn:
            return False
        try:
            self.conn.mav.command_long_send(
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                0,     # param1: 0 = disarm
                21196, # param2: force disarm
                0, 0, 0, 0, 0
            )
            return True
        except Exception:
            return False

    def _override_worker(self):
        """
        High-rate override loop (20 Hz).
        When halt is active, continuously sends RC_CHANNELS_OVERRIDE packets
        with throttle clamped to 40% and roll/pitch/yaw centered.
        Because ArduPilot expects overrides at >= 2 Hz, this 20 Hz stream
        completely overrides and ignores all FlySky stick commands.
        """
        while self.running:
            is_halt = False
            with self.lock:
                is_halt = self.halt_triggered
                halt_pwm = self.halt_pwm

            if is_halt and self.conn:
                try:
                    # Ch1 (Roll): 1500 (Level attitude)
                    # Ch2 (Pitch): 1500 (Level attitude)
                    # Ch3 (Throttle): halt_pwm (Strictly 40.0% throttle)
                    # Ch4 (Yaw): 1500 (Neutral center)
                    # Ch5..8: 0 (No override)
                    self.conn.mav.rc_channels_override_send(
                        self.target_system,
                        self.target_component,
                        1500,     # Ch1 Roll
                        1500,     # Ch2 Pitch
                        halt_pwm, # Ch3 Throttle (40%)
                        1500,     # Ch4 Yaw
                        0, 0, 0, 0
                    )
                except Exception:
                    pass
            time.sleep(0.05)  # 20 Hz

    def close(self):
        """Safely clean up threads and release RC overrides."""
        self.running = False
        self.release_override()
        if self.conn:
            try:
                self.conn.close()
            except Exception:
                pass


def render_vibe_bar(val: float, max_scale: float = 60.0, width: int = 14) -> str:
    """Format an ASCII progress bar for vibration levels."""
    ratio = min(max(val / max_scale, 0.0), 1.0)
    filled = int(round(ratio * width))
    bar = "|" * filled + " " * (width - filled)

    if val < 30.0:
        return f"{C_GREEN}[{bar}]{C_RESET}"
    elif val < 45.0:
        return f"{C_YELLOW}[{bar}]{C_RESET}"
    else:
        return f"{C_RED}[{bar}]{C_RESET}"


def run_terminal_dashboard(guard: VibrationMotorGuard):
    """Interactive single-screen terminal UI."""
    # Setup terminal for non-blocking raw keypresses
    old_settings = None
    if sys.stdin.isatty():
        try:
            old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        except Exception:
            pass

    status_notice = ""
    last_render = 0.0

    try:
        while guard.running:
            now = time.time()

            # Process non-blocking keyboard input
            if sys.stdin.isatty():
                r, _, _ = select.select([sys.stdin], [], [], 0.02)
                if r:
                    key = sys.stdin.read(1).lower()
                    if key == "q":
                        status_notice = "Exiting program..."
                        break
                    elif key == "t":
                        guard.trigger_halt("Manual Operator Test Trigger [Key 't']")
                        status_notice = "Test Trigger Activated: Motors Halted to 40%!"
                    elif key == "r":
                        guard.release_override()
                        status_notice = "Override Released: FlySky Transmitter Restored!"
                    elif key == "d":
                        guard.emergency_disarm()
                        status_notice = "EMERGENCY DISARM COMMAND SENT!"

            # Render UI at 5 Hz
            if now - last_render >= 0.2:
                last_render = now
                with guard.lock:
                    is_halt = guard.halt_triggered
                    reason = guard.trigger_reason
                    v_x = guard.vibe_x
                    v_y = guard.vibe_y
                    v_z = guard.vibe_z
                    v_mag = guard.vibe_mag
                    max_mag = guard.max_vibe_mag
                    c0 = guard.clipping_0
                    c1 = guard.clipping_1
                    c2 = guard.clipping_2
                    burst = guard.clipping_burst
                    armed = guard.armed
                    mode = guard.flight_mode
                    rc_roll = guard.rc_in_roll
                    rc_pitch = guard.rc_in_pitch
                    rc_thr = guard.rc_in_throttle
                    rc_yaw = guard.rc_in_yaw
                    halt_pwm = guard.halt_pwm

                # Build Dashboard Screen
                lines = []
                lines.append(C_CLEAR)
                lines.append(f"{C_BOLD}{C_WHITE}==============================================================================={C_RESET}")
                lines.append(f"{C_BOLD}{C_WHITE}       PIXHAWK VIBRATION GUARD & 40% MOTOR OVERRIDE CONTROLLER{C_RESET}")
                lines.append(f"{C_BOLD}{C_WHITE}==============================================================================={C_RESET}")

                # Status Banner
                if is_halt:
                    lines.append(f"{C_BG_RED}{C_WHITE}{C_BOLD} >>> ALARM: ABNORMALITY DETECTED! MOTORS HALTED AT 40% (FLYSKY LOCKED OUT) <<< {C_RESET}")
                    lines.append(f"{C_RED}{C_BOLD} Trigger Reason: {reason}{C_RESET}")
                else:
                    lines.append(f"{C_BG_GREEN}{C_WHITE}{C_BOLD} [STATUS: ACTIVE MONITORING - FLYSKY HAS NORMAL MANUAL CONTROL] {C_RESET}")
                    lines.append(f"{C_CYAN} Armed: {C_GREEN if armed else C_YELLOW}{'ARMED (Motors Live)' if armed else 'DISARMED'}{C_RESET}  |  Flight Mode: {C_BOLD}{mode}{C_RESET}  |  Device: {guard.device}")

                lines.append(f"{C_BOLD}{C_WHITE}-------------------------------------------------------------------------------{C_RESET}")
                lines.append(f"{C_BOLD}1. REAL-TIME VIBRATION SENSORS (m/s²):{C_RESET}")
                lines.append(f"   X-Axis:     {render_vibe_bar(v_x)} {v_x:5.2f} m/s²")
                lines.append(f"   Y-Axis:     {render_vibe_bar(v_y)} {v_y:5.2f} m/s²")
                lines.append(f"   Z-Axis:     {render_vibe_bar(v_z)} {v_z:5.2f} m/s²")

                # Vibration Mag color
                mag_color = C_GREEN if v_mag < 30.0 else (C_YELLOW if v_mag < guard.threshold_vibe else C_RED)
                lines.append(f"   Total Mag:  {render_vibe_bar(v_mag)} {mag_color}{C_BOLD}{v_mag:5.2f} m/s²{C_RESET} (Peak: {max_mag:.2f} m/s² | Threshold: {guard.threshold_vibe:.1f} m/s²)")
                lines.append(f"   Clipping:   Acc0: {c0} | Acc1: {c1} | Acc2: {c2} (New Burst: {burst})")

                lines.append(f"{C_BOLD}{C_WHITE}-------------------------------------------------------------------------------{C_RESET}")
                lines.append(f"{C_BOLD}2. FLYSKY RC TRANSMITTER vs MOTOR CONTROL STATUS:{C_RESET}")
                if is_halt:
                    lines.append(f"   FlySky Input Status: {C_BG_RED}{C_WHITE}{C_BOLD} BLOCKED & IGNORED (Companion Computer Active) {C_RESET}")
                    lines.append(f"   Roll Command:        1500 µs (Level Attitude Lock)")
                    lines.append(f"   Pitch Command:       1500 µs (Level Attitude Lock)")
                    lines.append(f"   Throttle Command:    {C_RED}{C_BOLD}{halt_pwm} µs [FORCED AT 40.0%]{C_RESET} (FlySky Stick: {rc_thr} µs ignored)")
                    lines.append(f"   Yaw Command:         1500 µs (Neutral Center)")
                else:
                    lines.append(f"   FlySky Input Status: {C_GREEN}CONNECTED & CONTROLLING MOTORS{C_RESET}")
                    lines.append(f"   Roll (Ch 1):         {rc_roll:4d} µs")
                    lines.append(f"   Pitch (Ch 2):        {rc_pitch:4d} µs")
                    lines.append(f"   Throttle (Ch 3):     {rc_thr:4d} µs")
                    lines.append(f"   Yaw (Ch 4):          {rc_yaw:4d} µs")

                lines.append(f"{C_BOLD}{C_WHITE}-------------------------------------------------------------------------------{C_RESET}")
                lines.append(f"{C_BOLD}HOTKEY CONTROLS:{C_RESET}")
                lines.append(f"  {C_CYAN}[t]{C_RESET} Test Abnormality Trigger (Halt to 40%)     {C_CYAN}[r]{C_RESET} Release Override (Back to FlySky)")
                lines.append(f"  {C_CYAN}[d]{C_RESET} Force Emergency Disarm                    {C_CYAN}[q]{C_RESET} Quit Safely")
                if status_notice:
                    lines.append(f"\n{C_MAGENTA}>>> {status_notice}{C_RESET}")
                lines.append(f"{C_BOLD}{C_WHITE}==============================================================================={C_RESET}")

                sys.stdout.write("\n".join(lines) + "\n")
                sys.stdout.flush()

            time.sleep(0.02)

    finally:
        if old_settings:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            except Exception:
                pass


def main():
    parser = argparse.ArgumentParser(
        description="Pixhawk Vibration Abnormality Guard: Halts motors at 40% and overrides FlySky input."
    )
    parser.add_argument(
        "--device",
        type=str,
        default=None,
        help="Pixhawk serial port or UDP endpoint (default: auto-detects /dev/pixhawk)"
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Serial baud rate (default: 115200)"
    )
    parser.add_argument(
        "--threshold",
        type=float,
        default=35.0,
        help="Vibration threshold magnitude in m/s² to trigger halt (default: 35.0)"
    )
    parser.add_argument(
        "--throttle-pct",
        type=float,
        default=40.0,
        help="Target motor throttle percentage on abnormality halt (default: 40.0)"
    )
    parser.add_argument(
        "--consecutive",
        type=int,
        default=3,
        help="Number of consecutive samples above threshold to trigger halt (default: 3)"
    )
    parser.add_argument(
        "--auto-alt-hold",
        action="store_true",
        help="Automatically switch Pixhawk to ALT_HOLD mode when abnormality occurs"
    )

    args = parser.parse_args()

    # Auto-detect device if not specified
    device = args.device if args.device else detect_pixhawk_port()

    guard = VibrationMotorGuard(
        device=device,
        baud=args.baud,
        threshold_vibe=args.threshold,
        halt_throttle_pct=args.throttle_pct,
        consecutive_samples=args.consecutive,
        auto_alt_hold=args.auto_alt_hold,
    )

    # Handle Ctrl+C cleanly
    def sig_handler(sig, frame):
        print(f"\n{C_YELLOW}[!] Interrupted by user. Releasing overrides and stopping...{C_RESET}")
        guard.close()
        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    if not guard.connect():
        sys.exit(1)

    guard.start()
    print(f"{C_GREEN}[+] Vibration Guard active! Launching interactive dashboard...{C_RESET}")
    time.sleep(0.5)

    try:
        run_terminal_dashboard(guard)
    except KeyboardInterrupt:
        pass
    finally:
        guard.close()
        print(f"{C_GREEN}[+] Guard stopped and FlySky control cleanly restored.{C_RESET}")


if __name__ == "__main__":
    main()
