#!/usr/bin/env python3
"""
sensor_telemetry.py - Pixhawk Real-Time Telemetry & Sensor Monitor
==================================================================
Streams and logs all sensor telemetry from Pixhawk to the Raspberry Pi:
  * Attitude (Roll, Pitch, Yaw, Angular rates)
  * IMU (Accelerometers, Gyroscopes, Magnetometers)
  * Barometer & Temperature (Air pressure, temp)
  * Battery & Power (Voltage, Current, Remaining %)
  * GPS Status & Raw Data (Fix type, Satellites, HDOP/VDOP)
  * Local Position NED (X, Y, Z, Vx, Vy, Vz - Vital for GPS-denied navigation!)
  * Rangefinder / Lidar distance (Vital for altitude hold in GPS-denied environments)
  * Optical Flow (Flow X/Y, quality, ground distance)
  * VFR HUD (Heading, Airspeed, Groundspeed, Throttle, Altitude)
  * EKF Health & Flight Mode / Arm Status

Usage Examples:
  python3 sensor_telemetry.py                  # Live interactive terminal dashboard
  python3 sensor_telemetry.py --snapshot       # Single formatted snapshot
  python3 sensor_telemetry.py --snapshot --json# Single JSON snapshot
  python3 sensor_telemetry.py --rate 10        # 10 Hz refresh rate
  python3 sensor_telemetry.py --log data.csv   # Log telemetry to CSV file
"""

import sys
import os
import time
import json
import csv
import argparse
from datetime import datetime
from pixhawk_controller import PixhawkVehicle


def format_value(val, unit="", precision=2):
    if val is None:
        return "N/A"
    if isinstance(val, float):
        return f"{val:.{precision}f}{unit}"
    return f"{val}{unit}"


def print_snapshot(telem: dict):
    print("\n" + "=" * 65)
    print(f" PIXHAWK SENSOR TELEMETRY SNAPSHOT | {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("=" * 65)

    print(f"Vehicle: {telem.get('vehicle_type', 'Unknown')} ({telem.get('autopilot', 'Unknown')})")
    print(f"Status:  Armed: [{'YES' if telem.get('armed') else 'NO'}] | Mode: [{telem.get('flight_mode', 'UNKNOWN')}]")
    print("-" * 65)

    # 1. Attitude
    att = telem.get("attitude")
    if att:
        print(f"ATTITUDE:      Roll: {format_value(att['roll_deg'], '°')} | Pitch: {format_value(att['pitch_deg'], '°')} | Yaw: {format_value(att['yaw_deg'], '°')}")
        print(f"               RollRate: {format_value(att['rollspeed_dps'], '°/s')} | PitchRate: {format_value(att['pitchspeed_dps'], '°/s')} | YawRate: {format_value(att['yawspeed_dps'], '°/s')}")
    else:
        print("ATTITUDE:      [No attitude packet]")

    # 2. IMU
    imu = telem.get("imu")
    if imu:
        print(f"IMU ACCEL:     X: {format_value(imu['accel_x_mg'], ' mg')} | Y: {format_value(imu['accel_y_mg'], ' mg')} | Z: {format_value(imu['accel_z_mg'], ' mg')}")
        print(f"IMU GYRO:      X: {format_value(imu['gyro_x_dps'], '°/s')} | Y: {format_value(imu['gyro_y_dps'], '°/s')} | Z: {format_value(imu['gyro_z_dps'], '°/s')}")
        print(f"IMU MAG:       X: {format_value(imu['mag_x_mgauss'], ' mG')} | Y: {format_value(imu['mag_y_mgauss'], ' mG')} | Z: {format_value(imu['mag_z_mgauss'], ' mG')}")
    else:
        print("IMU:           [No IMU packet]")

    # 3. Barometer & Battery
    baro = telem.get("barometer")
    bat = telem.get("battery")
    baro_str = f"Press: {format_value(baro['pressure_hpa'], ' hPa')} | Temp: {format_value(baro['temperature_c'], '°C')}" if baro else "N/A"
    bat_str = f"Volt: {format_value(bat['voltage_v'], 'V')} | Curr: {format_value(bat['current_a'], 'A')} | Rem: {format_value(bat['remaining_pct'], '%')}" if bat else "N/A"
    print(f"BAROMETER:     {baro_str}")
    print(f"BATTERY:       {bat_str}")

    # 4. GPS & Local Position
    gps = telem.get("gps")
    gps_str = f"{gps['fix_name']} | Sats: {gps['satellites_visible']} | HDOP: {format_value(gps['hdop'])}" if gps else "N/A"
    print(f"GPS:           {gps_str}")

    lpos = telem.get("local_position_ned")
    if lpos:
        print(f"LOCAL NED (m): X: {format_value(lpos['x_m'])} | Y: {format_value(lpos['y_m'])} | Z: {format_value(lpos['z_m'])}")
        print(f"VELOCITY (m/s):Vx: {format_value(lpos['vx_ms'])} | Vy: {format_value(lpos['vy_ms'])} | Vz: {format_value(lpos['vz_ms'])}")
    else:
        print("LOCAL NED:     [Awaiting EKF local estimate / Odometry]")

    # 5. GPS-Denied Sensors (Distance Sensor & Optical Flow)
    dist = telem.get("rangefinder")
    flow = telem.get("optical_flow")
    if dist:
        print(f"RANGEFINDER:   Dist: {format_value(dist['distance_m'], ' m')} (Range: {format_value(dist['min_distance_m'])} - {format_value(dist['max_distance_m'])}m)")
    if flow:
        print(f"OPTICAL FLOW:  FlowX: {flow['flow_x']} | FlowY: {flow['flow_y']} | Quality: {flow['quality']}% | GroundDist: {format_value(flow['ground_distance_m'], ' m')}")

    # 6. VFR HUD
    hud = telem.get("vfr_hud")
    if hud:
        print(f"VFR HUD:       Heading: {hud['heading_deg']}° | Alt: {format_value(hud['alt_m'], 'm')} | AirSpd: {format_value(hud['airspeed_ms'], ' m/s')} | Throttle: {hud['throttle_pct']}%")

    print("=" * 65 + "\n")


def live_dashboard(vehicle: PixhawkVehicle, rate_hz: float = 4.0, log_file: str = None):
    """
    Continuous interactive terminal dashboard.
    """
    interval = 1.0 / max(rate_hz, 0.5)
    csv_writer = None
    csv_handle = None

    if log_file:
        fieldnames = [
            "timestamp", "armed", "flight_mode", "roll_deg", "pitch_deg", "yaw_deg",
            "rollspeed_dps", "pitchspeed_dps", "yawspeed_dps",
            "accel_x_mg", "accel_y_mg", "accel_z_mg", "gyro_x_dps", "gyro_y_dps", "gyro_z_dps",
            "pressure_hpa", "temp_c", "bat_voltage_v", "bat_current_a", "bat_remaining_pct",
            "gps_fix", "gps_sats", "hud_alt_m", "hud_heading", "hud_throttle_pct",
            "local_x_m", "local_y_m", "local_z_m", "rangefinder_dist_m"
        ]
        csv_handle = open(log_file, "w", newline="")
        csv_writer = csv.DictWriter(csv_handle, fieldnames=fieldnames)
        csv_writer.writeheader()

    print("\033[2J\033[H", end="")  # Clear screen
    try:
        while True:
            telem = vehicle.get_telemetry()
            # Move cursor to top left
            print("\033[H", end="")

            now_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            armed_badge = "\033[91mARMED\033[0m" if telem.get("armed") else "\033[92mDISARMED\033[0m"

            print(f"┌────────────────────────────────────────────────────────────────────────┐")
            print(f"│  PIXHAWK REAL-TIME TELEMETRY DASHBOARD          {now_str} │")
            print(f"├────────────────────────────────────────────────────────────────────────┤")
            print(f"│ Autopilot: {telem.get('autopilot','N/A'):<12} | Vehicle: {telem.get('vehicle_type','N/A'):<15} | Port: {vehicle.connection_str:<12} │")
            print(f"│ Status:    {armed_badge:<18} | Mode: {telem.get('flight_mode','N/A'):<15}                  │")
            print(f"├────────────────────────────────────────────────────────────────────────┤")

            # Attitude
            att = telem.get("attitude") or {}
            roll = format_value(att.get("roll_deg"), "°")
            pitch = format_value(att.get("pitch_deg"), "°")
            yaw = format_value(att.get("yaw_deg"), "°")
            print(f"│ ATTITUDE:  Roll: {roll:<9} | Pitch: {pitch:<9} | Yaw: {yaw:<9}            │")

            # IMU
            imu = telem.get("imu") or {}
            ax = format_value(imu.get("accel_x_mg"), "mg", precision=0)
            ay = format_value(imu.get("accel_y_mg"), "mg", precision=0)
            az = format_value(imu.get("accel_z_mg"), "mg", precision=0)
            print(f"│ ACCEL:     X: {ax:<12} | Y: {ay:<12} | Z: {az:<12}     │")

            gx = format_value(imu.get("gyro_x_dps"), "°/s", precision=1)
            gy = format_value(imu.get("gyro_y_dps"), "°/s", precision=1)
            gz = format_value(imu.get("gyro_z_dps"), "°/s", precision=1)
            print(f"│ GYRO:      X: {gx:<12} | Y: {gy:<12} | Z: {gz:<12}     │")

            # Barometer & Battery
            baro = telem.get("barometer") or {}
            bat = telem.get("battery") or {}
            p_val = format_value(baro.get("pressure_hpa"), " hPa")
            t_val = format_value(baro.get("temperature_c"), "°C")
            v_val = format_value(bat.get("voltage_v"), "V")
            c_val = format_value(bat.get("current_a"), "A")
            rem_val = format_value(bat.get("remaining_pct"), "%")
            print(f"│ BAROMETER: Press: {p_val:<10} | Temp: {t_val:<10}                         │")
            print(f"│ BATTERY:   Volt:  {v_val:<10} | Curr: {c_val:<10} | Remaining: {rem_val:<6}  │")

            # GPS & Local Position
            gps = telem.get("gps") or {}
            g_fix = gps.get("fix_name", "N/A")
            g_sats = gps.get("satellites_visible", 0)
            print(f"│ GPS:       Fix: {g_fix:<15} | Sats: {g_sats:<4}                              │")

            lpos = telem.get("local_position_ned") or {}
            lx = format_value(lpos.get("x_m"), "m")
            ly = format_value(lpos.get("y_m"), "m")
            lz = format_value(lpos.get("z_m"), "m")
            print(f"│ LOCAL NED: X: {lx:<10} | Y: {ly:<10} | Z: {lz:<10}             │")

            # Rangefinder (Lidar)
            dist = telem.get("rangefinder") or {}
            rf_d = format_value(dist.get("distance_m"), "m")
            print(f"│ RANGEFIND: Dist: {rf_d:<10} (Lidar / Sonar rangefinder)                │")

            # HUD
            hud = telem.get("vfr_hud") or {}
            h_alt = format_value(hud.get("alt_m"), "m")
            h_hdg = format_value(hud.get("heading_deg"), "°", precision=0)
            h_thr = format_value(hud.get("throttle_pct"), "%", precision=0)
            print(f"│ VFR HUD:   Alt: {h_alt:<9} | Hdg: {h_hdg:<8} | Throttle: {h_thr:<6}               │")
            print(f"└────────────────────────────────────────────────────────────────────────┘")
            print(" Press Ctrl+C to stop dashboard...\033[K")

            if csv_writer:
                row = {
                    "timestamp": telem["timestamp"],
                    "armed": 1 if telem.get("armed") else 0,
                    "flight_mode": telem.get("flight_mode", ""),
                    "roll_deg": att.get("roll_deg"),
                    "pitch_deg": att.get("pitch_deg"),
                    "yaw_deg": att.get("yaw_deg"),
                    "rollspeed_dps": att.get("rollspeed_dps"),
                    "pitchspeed_dps": att.get("pitchspeed_dps"),
                    "yawspeed_dps": att.get("yawspeed_dps"),
                    "accel_x_mg": imu.get("accel_x_mg"),
                    "accel_y_mg": imu.get("accel_y_mg"),
                    "accel_z_mg": imu.get("accel_z_mg"),
                    "gyro_x_dps": imu.get("gyro_x_dps"),
                    "gyro_y_dps": imu.get("gyro_y_dps"),
                    "gyro_z_dps": imu.get("gyro_z_dps"),
                    "pressure_hpa": baro.get("pressure_hpa"),
                    "temp_c": baro.get("temperature_c"),
                    "bat_voltage_v": bat.get("voltage_v"),
                    "bat_current_a": bat.get("current_a"),
                    "bat_remaining_pct": bat.get("remaining_pct"),
                    "gps_fix": gps.get("fix_name"),
                    "gps_sats": gps.get("satellites_visible"),
                    "hud_alt_m": hud.get("alt_m"),
                    "hud_heading": hud.get("heading_deg"),
                    "hud_throttle_pct": hud.get("throttle_pct"),
                    "local_x_m": lpos.get("x_m"),
                    "local_y_m": lpos.get("y_m"),
                    "local_z_m": lpos.get("z_m"),
                    "rangefinder_dist_m": dist.get("distance_m"),
                }
                csv_writer.writerow(row)
                csv_handle.flush()

            time.sleep(interval)

    except KeyboardInterrupt:
        print("\nTelemetry dashboard stopped.")
    finally:
        if csv_handle:
            csv_handle.close()
            print(f"Telemetry log saved to: {log_file}")


def main():
    parser = argparse.ArgumentParser(description="Pixhawk Sensor Telemetry Monitor")
    parser.add_argument("--snapshot", action="store_true", help="Print a single telemetry snapshot and exit")
    parser.add_argument("--json", action="store_true", help="Format snapshot as JSON")
    parser.add_argument("--rate", type=float, default=4.0, help="Dashboard refresh rate in Hz (default: 4.0)")
    parser.add_argument("--log", type=str, default=None, help="Save live telemetry to CSV file")
    parser.add_argument("--port", type=str, default=None, help="Serial port (default: auto-detect)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")

    args = parser.parse_args()

    if not args.json:
        print(f"Connecting to Pixhawk (Port: {args.port or 'auto-detect'}, Baud: {args.baud})...")
    vehicle = PixhawkVehicle(connection_str=args.port, baud=args.baud)

    if not vehicle.is_connected:
        print("[ERROR] Could not connect to Pixhawk flight controller.", file=sys.stderr)
        sys.exit(1)

    # Allow telemetry stream to populate
    time.sleep(1.2)

    try:
        if args.snapshot:
            telem = vehicle.get_telemetry()
            if args.json:
                print(json.dumps(telem, indent=2))
            else:
                print_snapshot(telem)
        else:
            live_dashboard(vehicle, rate_hz=args.rate, log_file=args.log)
    finally:
        vehicle.disconnect()


if __name__ == "__main__":
    main()
