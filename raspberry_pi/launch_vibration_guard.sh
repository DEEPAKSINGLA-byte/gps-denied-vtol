#!/usr/bin/env bash
# ==============================================================================
# launch_vibration_guard.sh - Launch Vibration Abnormality Guard & Motor Halt
# ==============================================================================
# Usage:
#   ./launch_vibration_guard.sh                 (Uses auto-detected /dev/pixhawk)
#   ./launch_vibration_guard.sh --threshold 30  (Custom vibration threshold)
#   ./launch_vibration_guard.sh --remote        (Run remotely on Raspberry Pi via SSH)
# ==============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PI_USER="deimos"
PI_IP="10.184.3.36"

# Check if user requested remote execution on the Raspberry Pi
if [[ "$1" == "--remote" ]] || [[ ! -e "/dev/pixhawk" && ! -e "/dev/ttyACM0" ]]; then
    # If not running on Pi and no local Pixhawk is plugged into laptop, run on Pi via SSH
    if [[ "$(hostname)" != "deimos" ]]; then
        echo "=================================================================="
        echo "  Forwarding Vibration Guard Execution to Raspberry Pi ($PI_IP)..."
        echo "=================================================================="
        ssh -t "$PI_USER@$PI_IP" "python3 /home/$PI_USER/raspberry_pi/vibration_motor_halt.py $@"
        exit $?
    fi
fi

# Run locally (on Pi or on Laptop if Pixhawk is connected to laptop)
python3 "$SCRIPT_DIR/vibration_motor_halt.py" "$@"
