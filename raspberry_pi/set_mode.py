#!/usr/bin/env python3
"""
set_mode.py - Pixhawk Flight Mode Switching Utility
===================================================
Quickly switch flight modes from the Raspberry Pi companion computer.

Supported common modes:
  * ALT_HOLD (Altitude Mode)
  * POSHOLD / LOITER (Position Mode)
  * LAND (Land Mode)
  * STABILIZE (Manual / Stabilized Mode)
  * GUIDED / OFFBOARD (Autonomous / Companion Computer Guided Mode)
  * RTL (Return To Launch)
  * AUTO (Mission mode)

Usage Examples:
  python3 set_mode.py alt_hold
  python3 set_mode.py position
  python3 set_mode.py land
  python3 set_mode.py stabilize
  python3 set_mode.py guided
  python3 set_mode.py --list
  python3 set_mode.py --current
"""

import sys
import argparse
from pixhawk_controller import PixhawkVehicle


def main():
    parser = argparse.ArgumentParser(description="Pixhawk Flight Mode Switcher")
    parser.add_argument(
        "mode",
        nargs="?",
        type=str,
        default=None,
        help="Target flight mode (e.g. ALT_HOLD, POSHOLD, LAND, STABILIZE, GUIDED, RTL, etc.)"
    )
    parser.add_argument("--list", action="store_true", help="List all available flight modes for connected vehicle")
    parser.add_argument("--current", action="store_true", help="Show current active flight mode")
    parser.add_argument("--port", type=str, default=None, help="Serial port (default: auto-detect)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--timeout", type=float, default=5.0, help="Command timeout in seconds")

    args = parser.parse_args()

    if not args.mode and not args.list and not args.current:
        parser.print_help()
        sys.exit(1)

    print(f"Connecting to Pixhawk (Port: {args.port or 'auto-detect'}, Baud: {args.baud})...")
    vehicle = PixhawkVehicle(connection_str=args.port, baud=args.baud, timeout=args.timeout)

    if not vehicle.is_connected:
        print("[ERROR] Could not connect to Pixhawk flight controller.")
        sys.exit(1)

    try:
        current_mode = vehicle.flight_mode
        print(f"Connected! Autopilot: {vehicle.autopilot_type} | Current Mode: [{current_mode}]")

        if args.current:
            print(f"Current Flight Mode: {current_mode}")
            sys.exit(0)

        if args.list:
            modes = vehicle.get_available_modes()
            print("\nAvailable Flight Modes:")
            for m in modes:
                marker = " -> [CURRENT]" if m == current_mode else ""
                print(f"  * {m}{marker}")
            sys.exit(0)

        if args.mode:
            target = args.mode.strip()
            print(f"\nRequesting mode change to: {target.upper()}...")
            success, msg = vehicle.set_mode(target, timeout=args.timeout)
            if success:
                print(f"[SUCCESS] {msg}")
                print(f"Active Flight Mode is now: [{vehicle.flight_mode}]")
            else:
                print(f"[FAILED] {msg}")
                sys.exit(1)

    finally:
        vehicle.disconnect()


if __name__ == "__main__":
    main()
