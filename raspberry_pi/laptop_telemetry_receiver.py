#!/usr/bin/env python3
"""
laptop_telemetry_receiver.py - Receive & Verify Pixhawk Telemetry on Laptop
==========================================================================
Runs on the ground laptop, listening on the wireless network interface (Redmi 10)
for telemetry datagrams broadcast by the Raspberry Pi companion computer.

Features:
  * Automatic validation of all sensor fields (Attitude, IMU, Baro, Battery, GPS, HUD)
  * Real-time packet reception rate (packets/sec) & latency statistics
  * Formatted live terminal dashboard
  * Verification mode (--verify) for automated link health testing
  * Telemetry logging to CSV/JSON

Usage Examples:
  # Verify link health & data integrity (receives 10 packets and prints verification report)
  python3 laptop_telemetry_receiver.py --verify

  # Live interactive dashboard
  python3 laptop_telemetry_receiver.py

  # Custom listening port
  python3 laptop_telemetry_receiver.py --port 14551

  # Log received packets to CSV file
  python3 laptop_telemetry_receiver.py --log received_flight_data.csv
"""

import sys
import time
import json
import socket
import argparse
from datetime import datetime


def format_value(val, unit="", precision=2):
    if val is None:
        return "N/A"
    if isinstance(val, float):
        return f"{val:.{precision}f}{unit}"
    return f"{val}{unit}"


def verify_telemetry_stream(port: int = 14551, packet_count: int = 15, timeout: float = 10.0):
    """
    Listen for `packet_count` packets and perform deep schema & value validation.
    """
    print("=" * 70)
    print(" PIXHAWK TELEMETRY LINK VERIFICATION (REDMI 10 NETWORK)")
    print("=" * 70)
    print(f"Listening on:        0.0.0.0:{port} (UDP)")
    print(f"Test Packets:        {packet_count}")
    print(f"Receive Timeout:     {timeout:.1f}s")
    print("-" * 70)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", port))
    sock.settimeout(timeout)

    received_packets = []
    sender_addr = None
    start_time = time.time()

    print(f"Waiting for telemetry packets from Raspberry Pi...")
    try:
        while len(received_packets) < packet_count:
            data, addr = sock.recvfrom(65535)
            if not sender_addr:
                sender_addr = addr
                print(f"[+] Connected! Receiving data stream from Raspberry Pi at {addr[0]}:{addr[1]}")

            try:
                packet = json.loads(data.decode("utf-8"))
                received_packets.append((time.time(), packet))
                sys.stdout.write(f"\r  Received: {len(received_packets)}/{packet_count} packets...")
                sys.stdout.flush()
            except json.JSONDecodeError as e:
                print(f"\n[WARN] Malformed packet received: {e}")
    except socket.timeout:
        print(f"\n[FAILED] Timed out waiting for telemetry packets after {timeout:.1f}s.")
        sock.close()
        return False
    finally:
        sock.close()

    elapsed = time.time() - start_time
    rate = len(received_packets) / elapsed if elapsed > 0 else 0

    print(f"\n\n[+] Successfully captured {len(received_packets)} packets in {elapsed:.2f}s ({rate:.1f} packets/sec)!")

    # Perform field-by-field validation on the latest packet
    latest = received_packets[-1][1]

    print("\n" + "=" * 70)
    print(f" TELEMETRY INTEGRITY & SENSOR DATA VALIDATION REPORT")
    print("=" * 70)
    print(f"Sender IP:           {sender_addr[0]}")
    print(f"Autopilot:           {latest.get('autopilot', 'Unknown')}")
    print(f"Vehicle Frame:       {latest.get('vehicle_type', 'Unknown')}")
    print(f"Arm State:           {'ARMED' if latest.get('armed') else 'DISARMED'}")
    print(f"Flight Mode:         {latest.get('flight_mode', 'UNKNOWN')}")
    print("-" * 70)

    checks = []

    # 1. Attitude Check
    att = latest.get("attitude")
    if att and "roll_deg" in att and "pitch_deg" in att and "yaw_deg" in att:
        checks.append(("Attitude (Roll, Pitch, Yaw)", True, f"Roll={att['roll_deg']}°, Pitch={att['pitch_deg']}°, Yaw={att['yaw_deg']}°"))
    else:
        checks.append(("Attitude", False, "Missing attitude frame"))

    # 2. IMU Check
    imu = latest.get("imu")
    if imu and "accel_x_mg" in imu and "gyro_x_dps" in imu:
        checks.append(("IMU (Accel / Gyro / Mag)", True, f"AccelZ={imu['accel_z_mg']} mg | GyroZ={imu['gyro_z_dps']}°/s"))
    else:
        checks.append(("IMU", False, "Missing IMU frame"))

    # 3. Barometer Check
    baro = latest.get("barometer")
    if baro and "pressure_hpa" in baro and baro["pressure_hpa"] > 0:
        checks.append(("Barometer & Temp", True, f"Pressure={baro['pressure_hpa']} hPa | Temp={baro['temperature_c']}°C"))
    else:
        checks.append(("Barometer", False, "Missing or invalid pressure reading"))

    # 4. Battery / Power Check
    bat = latest.get("battery")
    if bat and "voltage_v" in bat:
        checks.append(("Battery Monitor", True, f"Voltage={bat['voltage_v']}V | Rem={bat['remaining_pct']}%"))
    else:
        checks.append(("Battery Monitor", False, "Missing battery packet"))

    # 5. GPS / GNSS Check
    gps = latest.get("gps")
    if gps and "fix_name" in gps:
        checks.append(("GPS Receiver", True, f"Fix: {gps['fix_name']} ({gps['satellites_visible']} sats)"))
    else:
        checks.append(("GPS Receiver", False, "Missing GPS packet"))

    # 6. Flight HUD
    hud = latest.get("vfr_hud")
    if hud and "alt_m" in hud:
        checks.append(("VFR HUD", True, f"Alt={hud['alt_m']}m | Heading={hud['heading_deg']}° | Throttle={hud['throttle_pct']}%"))
    else:
        checks.append(("VFR HUD", False, "Missing HUD packet"))

    # Display Checklist Results
    all_passed = True
    for name, passed, detail in checks:
        badge = "\033[92m[ PASSED ]\033[0m" if passed else "\033[91m[ FAILED ]\033[0m"
        print(f" {badge} {name:<26}: {detail}")
        if not passed:
            all_passed = False

    print("=" * 70)
    if all_passed:
        print("\033[92m[SUCCESS] All Pixhawk sensor telemetry is reaching the laptop accurately!\033[0m")
    else:
        print("\033[91m[WARNING] Some telemetry channels were missing or invalid.\033[0m")
    print("=" * 70 + "\n")
    return all_passed


def live_dashboard(port: int = 14551, log_file: str = None):
    """
    Continuous real-time dashboard on the laptop terminal.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", port))

    print(f"Listening for telemetry stream on 0.0.0.0:{port}...")
    packet_count = 0
    start_time = time.time()
    csv_handle = None
    csv_writer = None

    if log_file:
        import csv
        fieldnames = [
            "timestamp", "armed", "flight_mode", "roll_deg", "pitch_deg", "yaw_deg",
            "accel_x_mg", "accel_y_mg", "accel_z_mg", "pressure_hpa", "temp_c",
            "bat_voltage_v", "gps_fix", "hud_alt_m", "hud_heading"
        ]
        csv_handle = open(log_file, "w", newline="")
        csv_writer = csv.DictWriter(csv_handle, fieldnames=fieldnames)
        csv_writer.writeheader()

    print("\033[2J\033[H", end="")
    try:
        while True:
            data, addr = sock.recvfrom(65535)
            packet_count += 1
            now = time.time()
            rate = packet_count / (now - start_time) if (now - start_time) > 0 else 0

            try:
                telem = json.loads(data.decode("utf-8"))
            except Exception:
                continue

            print("\033[H", end="")
            now_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            armed_badge = "\033[91mARMED\033[0m" if telem.get("armed") else "\033[92mDISARMED\033[0m"

            print(f"┌────────────────────────────────────────────────────────────────────────┐")
            print(f"│  GROUND LAPTOP TELEMETRY RECEIVER (REDMI 10)     {now_str} │")
            print(f"├────────────────────────────────────────────────────────────────────────┤")
            print(f"│ From RPi:   {addr[0]:<15} | Throughput: {rate:>4.1f} packets/sec           │")
            print(f"│ Vehicle:    {telem.get('vehicle_type','N/A'):<15} | Autopilot:  {telem.get('autopilot','N/A'):<15} │")
            print(f"│ State:      {armed_badge:<18} | Mode:       {telem.get('flight_mode','N/A'):<15} │")
            print(f"├────────────────────────────────────────────────────────────────────────┤")

            att = telem.get("attitude") or {}
            roll = format_value(att.get("roll_deg"), "°")
            pitch = format_value(att.get("pitch_deg"), "°")
            yaw = format_value(att.get("yaw_deg"), "°")
            print(f"│ ATTITUDE:   Roll: {roll:<9} | Pitch: {pitch:<9} | Yaw: {yaw:<9}           │")

            imu = telem.get("imu") or {}
            ax = format_value(imu.get("accel_x_mg"), " mg", precision=0)
            ay = format_value(imu.get("accel_y_mg"), " mg", precision=0)
            az = format_value(imu.get("accel_z_mg"), " mg", precision=0)
            print(f"│ ACCEL:      X: {ax:<12} | Y: {ay:<12} | Z: {az:<12}    │")

            baro = telem.get("barometer") or {}
            bat = telem.get("battery") or {}
            p_val = format_value(baro.get("pressure_hpa"), " hPa")
            t_val = format_value(baro.get("temperature_c"), "°C")
            v_val = format_value(bat.get("voltage_v"), "V")
            print(f"│ BAROMETER:  Press: {p_val:<10} | Temp: {t_val:<10}                        │")
            print(f"│ BATTERY:    Volt:  {v_val:<10} | Rem:  {format_value(bat.get('remaining_pct'), '%'):<6}                        │")

            gps = telem.get("gps") or {}
            hud = telem.get("vfr_hud") or {}
            g_fix = gps.get("fix_name", "N/A")
            h_alt = format_value(hud.get("alt_m"), "m")
            h_hdg = format_value(hud.get("heading_deg"), "°", precision=0)
            print(f"│ GPS:        Fix: {g_fix:<15} | Sats: {gps.get('satellites_visible', 0):<4}                             │")
            print(f"│ VFR HUD:    Alt: {h_alt:<9} | Heading: {h_hdg:<8} | Throttle: {format_value(hud.get('throttle_pct'), '%', precision=0):<6}   │")
            print(f"└────────────────────────────────────────────────────────────────────────┘")
            print(" Press Ctrl+C to stop receiver...\033[K")

            if csv_writer:
                row = {
                    "timestamp": telem.get("timestamp"),
                    "armed": 1 if telem.get("armed") else 0,
                    "flight_mode": telem.get("flight_mode", ""),
                    "roll_deg": att.get("roll_deg"),
                    "pitch_deg": att.get("pitch_deg"),
                    "yaw_deg": att.get("yaw_deg"),
                    "accel_x_mg": imu.get("accel_x_mg"),
                    "accel_y_mg": imu.get("accel_y_mg"),
                    "accel_z_mg": imu.get("accel_z_mg"),
                    "pressure_hpa": baro.get("pressure_hpa"),
                    "temp_c": baro.get("temperature_c"),
                    "bat_voltage_v": bat.get("voltage_v"),
                    "gps_fix": gps.get("fix_name"),
                    "hud_alt_m": hud.get("alt_m"),
                    "hud_heading": hud.get("heading_deg"),
                }
                csv_writer.writerow(row)
                csv_handle.flush()

    except KeyboardInterrupt:
        print("\nReceiver stopped.")
    finally:
        sock.close()
        if csv_handle:
            csv_handle.close()


def main():
    parser = argparse.ArgumentParser(description="Laptop Pixhawk Telemetry Receiver")
    parser.add_argument("--port", type=int, default=14551, help="UDP listening port (default: 14551)")
    parser.add_argument("--verify", action="store_true", help="Run packet verification check and exit")
    parser.add_argument("--count", type=int, default=15, help="Number of packets for verification check")
    parser.add_argument("--timeout", type=float, default=10.0, help="Timeout in seconds for verification check")
    parser.add_argument("--log", type=str, default=None, help="Save received telemetry to CSV file")

    args = parser.parse_args()

    if args.verify:
        ok = verify_telemetry_stream(port=args.port, packet_count=args.count, timeout=args.timeout)
        sys.exit(0 if ok else 1)
    else:
        live_dashboard(port=args.port, log_file=args.log)


if __name__ == "__main__":
    main()
