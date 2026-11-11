#!/usr/bin/env python3
"""
drone_cli.py - Unified Interactive Pixhawk Drone CLI
===================================================
A single unified command-line tool and interactive console for Pixhawk control:
  * Arm / Disarm / Force-Arm
  * Switch flight modes (ALT_HOLD, POSHOLD, LAND, STABILIZE, GUIDED, RTL, etc.)
  * Live sensor telemetry & status check
  * Takeoff command & emergency motor kill
  * Interactive control shell

Usage:
  # Interactive Console
  python3 drone_cli.py

  # Quick Direct Flags
  python3 drone_cli.py --arm
  python3 drone_cli.py --arm --force
  python3 drone_cli.py --disarm
  python3 drone_cli.py --mode ALT_HOLD
  python3 drone_cli.py --mode LAND
  python3 drone_cli.py --status
  python3 drone_cli.py --telemetry
"""

import sys
import time
import argparse
from pixhawk_controller import PixhawkVehicle


def interactive_shell(vehicle: PixhawkVehicle):
    """
    Launch interactive command console.
    """
    print("\n" + "=" * 65)
    print(" PIXHAWK INTERACTIVE COMPANION CONSOLE")
    print(" Type 'help' for available commands, 'exit' to quit.")
    print("=" * 65)

    while True:
        try:
            arm_str = "ARMED" if vehicle.is_armed else "DISARMED"
            mode_str = vehicle.flight_mode
            prompt = f"drone [{arm_str} | {mode_str}] > "

            line = input(prompt).strip()
            if not line:
                continue

            parts = line.split()
            cmd = parts[0].lower()
            args = parts[1:]

            if cmd in ["exit", "quit", "q"]:
                print("Exiting console...")
                break

            elif cmd in ["help", "?"]:
                print("\nAvailable Commands:")
                print("  arm [--force]       Arm vehicle motors (--force bypasses GPS pre-arm check)")
                print("  disarm              Disarm vehicle motors")
                print("  kill                Emergency immediate motor cutoff")
                print("  mode <MODE_NAME>    Switch flight mode (e.g. alt_hold, poshold, land, stabilize, guided)")
                print("  modes               List all supported flight modes")
                print("  status              Show current armed status and flight mode")
                print("  telemetry           Show full sensor telemetry snapshot")
                print("  takeoff <alt_m>     Command takeoff to altitude in meters (Guided mode only)")
                print("  clear               Clear console screen")
                print("  exit / quit         Exit interactive console\n")

            elif cmd == "status":
                print(f"Status: Armed=[{vehicle.is_armed}], Mode=[{vehicle.flight_mode}], Autopilot=[{vehicle.autopilot_type}]")

            elif cmd == "arm":
                force = "--force" in args or "-f" in args
                print(f"Commanding ARM (force={force})...")
                ok, msg = vehicle.arm(force=force)
                print(f"[{'SUCCESS' if ok else 'FAILED'}] {msg}")

            elif cmd == "disarm":
                print("Commanding DISARM...")
                ok, msg = vehicle.disarm()
                print(f"[{'SUCCESS' if ok else 'FAILED'}] {msg}")

            elif cmd == "kill":
                print("[ALERT] Triggering EMERGENCY MOTOR CUTOFF...")
                ok, msg = vehicle.emergency_kill()
                print(f"[{'SUCCESS' if ok else 'FAILED'}] {msg}")

            elif cmd == "mode":
                if not args:
                    print("Usage: mode <MODE_NAME> (e.g., mode alt_hold, mode land)")
                else:
                    target_mode = args[0]
                    ok, msg = vehicle.set_mode(target_mode)
                    print(f"[{'SUCCESS' if ok else 'FAILED'}] {msg}")

            elif cmd == "modes":
                modes = vehicle.get_available_modes()
                curr = vehicle.flight_mode
                print(f"Supported Modes: {', '.join(modes)}")
                print(f"Current Mode:    {curr}")

            elif cmd == "takeoff":
                if not args:
                    print("Usage: takeoff <altitude_in_meters> (e.g. takeoff 2.5)")
                else:
                    try:
                        alt = float(args[0])
                        ok, msg = vehicle.takeoff(alt)
                        print(f"[{'SUCCESS' if ok else 'FAILED'}] {msg}")
                    except ValueError:
                        print("Invalid altitude number.")

            elif cmd == "telemetry":
                telem = vehicle.get_telemetry()
                att = telem.get("attitude") or {}
                imu = telem.get("imu") or {}
                baro = telem.get("barometer") or {}
                bat = telem.get("battery") or {}
                gps = telem.get("gps") or {}
                hud = telem.get("vfr_hud") or {}
                print("\n--- Sensor Snapshot ---")
                print(f"  Attitude: Roll={att.get('roll_deg')}°, Pitch={att.get('pitch_deg')}°, Yaw={att.get('yaw_deg')}°")
                print(f"  Baro:     Press={baro.get('pressure_hpa')} hPa, Temp={baro.get('temperature_c')}°C")
                print(f"  Battery:  Volt={bat.get('voltage_v')}V, Rem={bat.get('remaining_pct')}%")
                print(f"  GPS:      {gps.get('fix_name')} ({gps.get('satellites_visible')} sats)")
                print(f"  HUD:      Alt={hud.get('alt_m')}m, Hdg={hud.get('heading_deg')}°, Thr={hud.get('throttle_pct')}%\n")

            elif cmd == "clear":
                print("\033[2J\033[H", end="")

            else:
                print(f"Unknown command: '{cmd}'. Type 'help' for available commands.")

        except (KeyboardInterrupt, EOFError):
            print("\nExiting console...")
            break


def main():
    parser = argparse.ArgumentParser(description="Unified Pixhawk Drone CLI")
    parser.add_argument("--arm", action="store_true", help="Arm vehicle motors")
    parser.add_argument("--disarm", action="store_true", help="Disarm vehicle motors")
    parser.add_argument("--force", action="store_true", help="Force arming (bypass pre-arm GPS check)")
    parser.add_argument("--kill", action="store_true", help="Emergency immediate motor cutoff")
    parser.add_argument("--mode", type=str, default=None, help="Change flight mode (e.g. ALT_HOLD, POSHOLD, LAND, etc.)")
    parser.add_argument("--status", action="store_true", help="Show armed status and mode")
    parser.add_argument("--telemetry", action="store_true", help="Show sensor snapshot")
    parser.add_argument("--port", type=str, default=None, help="Serial port (default: auto-detect)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")

    args = parser.parse_args()

    # If any action flag is given, run in non-interactive batch mode
    has_action = any([args.arm, args.disarm, args.kill, args.mode, args.status, args.telemetry])

    vehicle = PixhawkVehicle(connection_str=args.port, baud=args.baud)
    if not vehicle.is_connected:
        print("[ERROR] Could not connect to Pixhawk flight controller.", file=sys.stderr)
        sys.exit(1)

    try:
        if not has_action:
            interactive_shell(vehicle)
        else:
            if args.status:
                print(f"Vehicle: {vehicle.vehicle_type} ({vehicle.autopilot_type})")
                print(f"Armed:   [{'YES' if vehicle.is_armed else 'NO'}]")
                print(f"Mode:    [{vehicle.flight_mode}]")

            if args.mode:
                ok, msg = vehicle.set_mode(args.mode)
                print(f"[{'SUCCESS' if ok else 'FAILED'}] {msg}")

            if args.arm:
                ok, msg = vehicle.arm(force=args.force)
                print(f"[{'SUCCESS' if ok else 'FAILED'}] {msg}")

            if args.disarm:
                ok, msg = vehicle.disarm(force=args.force)
                print(f"[{'SUCCESS' if ok else 'FAILED'}] {msg}")

            if args.kill:
                ok, msg = vehicle.emergency_kill()
                print(f"[{'SUCCESS' if ok else 'FAILED'}] {msg}")

            if args.telemetry:
                time.sleep(1.0)
                telem = vehicle.get_telemetry()
                print("\n--- Sensor Snapshot ---")
                att = telem.get("attitude") or {}
                baro = telem.get("barometer") or {}
                bat = telem.get("battery") or {}
                gps = telem.get("gps") or {}
                hud = telem.get("vfr_hud") or {}
                print(f"  Attitude: Roll={att.get('roll_deg')}°, Pitch={att.get('pitch_deg')}°, Yaw={att.get('yaw_deg')}°")
                print(f"  Baro:     Press={baro.get('pressure_hpa')} hPa, Temp={baro.get('temperature_c')}°C")
                print(f"  Battery:  Volt={bat.get('voltage_v')}V, Rem={bat.get('remaining_pct')}%")
                print(f"  GPS:      {gps.get('fix_name')} ({gps.get('satellites_visible')} sats)")
                print(f"  HUD:      Alt={hud.get('alt_m')}m, Hdg={hud.get('heading_deg')}°, Thr={hud.get('throttle_pct')}%\n")

    finally:
        vehicle.disconnect()


if __name__ == "__main__":
    main()
