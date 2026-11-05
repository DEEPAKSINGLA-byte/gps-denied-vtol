#!/usr/bin/env bash
# launch_plotjuggler.sh - 1-Click PlotJuggler Live Drone Telemetry Launcher
# ======================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LAYOUT_FILE="${SCRIPT_DIR}/plotjuggler_drone_layout.xml"

echo "======================================================================"
echo " LAUNCHING PLOTJUGGLER DRONE TELEMETRY VISUALIZATION"
echo "======================================================================"

# 1. Start the telemetry bridge in background
echo "[1/2] Starting PlotJuggler telemetry bridge (listening on UDP 14551)..."
python3 "${SCRIPT_DIR}/plotjuggler_bridge.py" --ros2 &
BRIDGE_PID=$!

# Ensure bridge terminates when PlotJuggler exits
trap "echo -e '\nClosing PlotJuggler bridge (PID: $BRIDGE_PID)...'; kill $BRIDGE_PID 2>/dev/null || true; exit 0" SIGINT SIGTERM EXIT

sleep 1.0

# 2. Launch PlotJuggler with pre-configured layout
echo "[2/2] Launching PlotJuggler GUI..."
echo "  * Streaming -> Click 'Start' on UDP Server (port 9870, JSON) or ROS2 Topic Subscriber!"
source /opt/ros/humble/setup.bash
ros2 run plotjuggler plotjuggler -l "${LAYOUT_FILE}" --nosplash
