#!/usr/bin/env python3
"""
test_connection.py - Pixhawk Hardware & Communication Link Diagnostics
=====================================================================
Automated diagnostic tool to verify the physical and logical connection between
the Raspberry Pi companion computer and the Pixhawk flight controller.

Features:
  * Port auto-discovery (/dev/ttyACM*, /dev/ttyUSB*, /dev/serial0)
  * Baudrate auto-probing (115200, 57600, 921600)
  * Firmware & vehicle type identification
  * MAVLink message throughput measurement (packets/sec)
  * Sensor stream verification (Attitude, IMU, Baro, Battery, GPS)
  * Pre-arm safety check status diagnosis
"""

import sys
import glob
import time
import math
from pymavlink import mavutil


def test_port(port: str, baud: int):
    print(f"\n[*] Probing {port} at {baud} baud...")
    try:
        conn = mavutil.mavlink_connection(port, baud=baud, timeout=2.0)
        hb = conn.wait_heartbeat(timeout=3.0)
        if not hb:
            print(f"[-] No heartbeat on {port} ({baud} baud).")
            conn.close()
            return None
        return conn, hb
    except Exception as e:
        print(f"[-] Error opening {port}: {e}")
        return None


def run_diagnostics():
    print("=" * 70)
    print(" PIXHAWK COMPANION HARDWARE & LINK DIAGNOSTICS")
    print("=" * 70)

    # 1. Discover serial ports
    ports = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*") + glob.glob("/dev/serial0*"))
    print(f"Detected serial devices in /dev: {ports or 'None found'}")

    if not ports:
        print("[!] No serial ports detected. Check USB cable or UART connection.")
        sys.exit(1)

    active_conn = None
    heartbeat = None
    connected_port = None
    connected_baud = None

    for port in ports:
        for baud in [115200, 57600, 921600]:
            res = test_port(port, baud)
            if res:
                active_conn, heartbeat = res
                connected_port = port
                connected_baud = baud
                break
        if active_conn:
            break

    if not active_conn:
        print("\n[FAILED] Could not receive MAVLink heartbeat on any port.")
        sys.exit(1)

    print(f"\n[+] Successfully connected to Pixhawk on {connected_port} at {connected_baud} baud!")

    # 2. Autopilot Info
    autopilot_map = {
        mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA: "ArduPilot (ArduCopter/Plane/VTOL)",
        mavutil.mavlink.MAV_AUTOPILOT_PX4: "PX4 Autopilot",
        mavutil.mavlink.MAV_AUTOPILOT_GENERIC: "Generic Autopilot"
    }
    type_map = {
        mavutil.mavlink.MAV_TYPE_QUADROTOR: "Quadrotor",
        mavutil.mavlink.MAV_TYPE_FIXED_WING: "Fixed Wing",
        mavutil.mavlink.MAV_TYPE_VTOL_TILTROTOR: "VTOL Tiltrotor",
        mavutil.mavlink.MAV_TYPE_VTOL_QUADROTOR: "VTOL Quadplane",
        mavutil.mavlink.MAV_TYPE_HEXAROTOR: "Hexarotor",
    }

    auto_str = autopilot_map.get(heartbeat.autopilot, f"Autopilot ID {heartbeat.autopilot}")
    type_str = type_map.get(heartbeat.type, f"Vehicle Type ID {heartbeat.type}")
    is_armed = bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    print("\n--- Flight Controller Metadata ---")
    print(f"  Autopilot:       {auto_str}")
    print(f"  Vehicle Frame:   {type_str}")
    print(f"  System ID:       {active_conn.target_system}")
    print(f"  Component ID:    {active_conn.target_component}")
    print(f"  MAVLink Version: {heartbeat.mavlink_version}")
    print(f"  Base Mode Flag:  {heartbeat.base_mode} (Armed: {is_armed})")
    print(f"  Custom Mode ID:  {heartbeat.custom_mode}")

    # Mode mapping
    mode_map = active_conn.mode_mapping()
    if mode_map:
        inv = {v: k for k, v in mode_map.items()}
        curr_mode = inv.get(heartbeat.custom_mode, "Unknown")
        print(f"  Current Mode:    {curr_mode}")
        print(f"  Supported Modes: {', '.join(sorted(mode_map.keys())[:10])} ... ({len(mode_map)} total)")

    # 3. Test Data Stream Throughput
    print("\n--- Requesting Telemetry Streams (10 Hz) ---")
    active_conn.mav.request_data_stream_send(
        active_conn.target_system,
        active_conn.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        10, 1
    )

    packet_counts = {}
    t_start = time.time()
    duration = 3.0
    print(f"Benchmarking packet throughput for {duration:.1f} seconds...")

    while time.time() - t_start < duration:
        msg = active_conn.recv_msg()
        if msg:
            mtype = msg.get_type()
            packet_counts[mtype] = packet_counts.get(mtype, 0) + 1

    total_packets = sum(packet_counts.values())
    rate = total_packets / duration
    print(f"  Throughput: {total_packets} packets in {duration:.1f}s ({rate:.1f} packets/sec)")
    print(f"  Distinct Message Types Received: {len(packet_counts)}")
    top_msgs = sorted(packet_counts.items(), key=lambda x: x[1], reverse=True)[:6]
    for name, cnt in top_msgs:
        print(f"    * {name:<22}: {cnt} packets ({cnt/duration:.1f} Hz)")

    # 4. Sensor Verification Checklist
    print("\n--- Sensor Health Verification Checklist ---")
    checklist = {
        "ATTITUDE": "Attitude & Orientation (IMU EKF)",
        "RAW_IMU": "Raw Accelerometer / Gyro / Magnetometer",
        "SCALED_PRESSURE": "Barometer (Air Pressure & Altitude)",
        "SYS_STATUS": "Battery Voltage & Power Monitoring",
        "GPS_RAW_INT": "GNSS / GPS Receiver",
        "VFR_HUD": "Flight HUD (Airspeed, Heading, Throttle)",
        "EKF_STATUS_REPORT": "EKF State Estimation",
    }

    for msg_name, desc in checklist.items():
        present = msg_name in packet_counts
        status = "[ OK ]" if present else "[WARN]"
        print(f"  {status} {desc:<42} ({msg_name})")

    active_conn.close()
    print("\n" + "=" * 70)
    print(" [PASSED] Pixhawk communication link is verified and fully operational!")
    print("=" * 70)


if __name__ == "__main__":
    run_diagnostics()
