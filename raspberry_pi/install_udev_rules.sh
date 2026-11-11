#!/usr/bin/env bash
# install_udev_rules.sh - Install persistent udev rules for Pixhawk & Camera
# =========================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PIXHAWK_RULE="${SCRIPT_DIR}/99-pixhawk.rules"
CAMERA_RULE="${SCRIPT_DIR}/98-camera.rules"

echo "======================================================================"
echo " INSTALLING DRONE UDEV RULES (/dev/pixhawk & /dev/camera)"
echo "======================================================================"

SUDO_CMD=""
if [ "$EUID" -ne 0 ]; then
    SUDO_CMD="sudo"
fi

if [ -f "${PIXHAWK_RULE}" ]; then
    $SUDO_CMD cp "${PIXHAWK_RULE}" "/etc/udev/rules.d/99-pixhawk.rules"
    echo "[+] Copied 99-pixhawk.rules -> /etc/udev/rules.d/"
fi

if [ -f "${CAMERA_RULE}" ]; then
    $SUDO_CMD cp "${CAMERA_RULE}" "/etc/udev/rules.d/98-camera.rules"
    echo "[+] Copied 98-camera.rules -> /etc/udev/rules.d/"
fi

$SUDO_CMD udevadm control --reload-rules
$SUDO_CMD udevadm trigger

sleep 0.5

echo "----------------------------------------------------------------------"
if [ -e "/dev/pixhawk" ]; then
    TARGET=$(readlink -f /dev/pixhawk)
    echo "[SUCCESS] /dev/pixhawk is active and points to ${TARGET}!"
else
    echo "[INFO] /dev/pixhawk will appear automatically when Pixhawk is plugged in."
fi

if [ -e "/dev/camera" ]; then
    TARGET=$(readlink -f /dev/camera)
    echo "[SUCCESS] /dev/camera is active and points to ${TARGET}!"
else
    echo "[INFO] /dev/camera will appear automatically when USB Camera is plugged in."
fi
echo "======================================================================"
