#!/usr/bin/env python3
"""
arm_disarm.py - Pixhawk Arm / Disarm Command Utility
===================================================
Arm or disarm the drone with safety checks, feedback, and force options.

Usage Examples:
  python3 arm_disarm.py --status
  python3 arm_disarm.py --arm
  python3 arm_disarm.py --arm --force    # Useful for GPS-denied indoor bench testing
  python3 arm_disarm.py --disarm
  python3 arm_disarm.py --kill           # Emergency motor cutoff
"""

import sys
import argparse
from pixhawk_controller import PixhawkVehicle


def main():
    parser = argparse.ArgumentParser(description="Pixhawk Arm / Disarm Control Utility")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--arm", action="store_true", help="Arm vehicle motors")
    group.add_argument("--disarm", action="store_true", help="Disarm vehicle motors")
    group.add_argument("--kill", action="store_true", help="Emergency immediate motor cutoff")
    group.add_argument("--status", action="store_true", help="Display current arm state & mode")

    parser.add_argument("--force", action="store_true", help="Force arming (bypasses pre-arm checks like GPS fix)")
    parser.add_argument("--port", type=str, default=None, help="Serial port (default: auto-detect)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--timeout", type=float, default=6.0, help="Command timeout in seconds")

    args = parser.parse_args()

    print(f"Connecting to Pixhawk (Port: {args.port or 'auto-detect'}, Baud: {args.baud})...")
    vehicle = PixhawkVehicle(connection_str=args.port, baud=args.baud, timeout=args.timeout)

    if not vehicle.is_connected:
        print("[ERROR] Could not connect to Pixhawk flight controller.")
        sys.exit(1)

    print(f"Connected! Autopilot: {vehicle.autopilot_type} | Vehicle: {vehicle.vehicle_type}")
    print(f"Current State: [{'ARMED' if vehicle.is_armed else 'DISARMED'}] | Flight Mode: [{vehicle.flight_mode}]")

    try:
        if args.status:
            print("\nStatus summary:")
            print(f"  Motors Armed: {'YES' if vehicle.is_armed else 'NO'}")
            print(f"  Flight Mode:  {vehicle.flight_mode}")
            sys.exit(0)

        elif args.arm:
            print(f"\nCommanding ARM (Force={args.force})...")
            if not args.force:
                print("  Note: If arming fails indoors without GPS, use '--arm --force' to bypass pre-arm GPS check.")
            success, msg = vehicle.arm(force=args.force, timeout=args.timeout)
            if success:
                print(f"[SUCCESS] {msg}")
            else:
                print(f"[FAILED] {msg}")
                sys.exit(1)

        elif args.disarm:
            print(f"\nCommanding DISARM...")
            success, msg = vehicle.disarm(force=args.force, timeout=args.timeout)
            if success:
                print(f"[SUCCESS] {msg}")
            else:
                print(f"[FAILED] {msg}")
                sys.exit(1)

        elif args.kill:
            print(f"\n[ALERT] Triggering EMERGENCY MOTOR CUTOFF...")
            success, msg = vehicle.emergency_kill()
            if success:
                print(f"[SUCCESS] Motors cut off: {msg}")
            else:
                print(f"[FAILED] Kill command failed: {msg}")
                sys.exit(1)

    finally:
        vehicle.disconnect()


if __name__ == "__main__":
    main()
