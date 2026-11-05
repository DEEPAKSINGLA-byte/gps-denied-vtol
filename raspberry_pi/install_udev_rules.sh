#!/usr/bin/env bash
# install_udev_rules.sh - Install persistent udev rules for Pixhawk
# ===============================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RULE_FILE="${SCRIPT_DIR}/99-pixhawk.rules"
DEST_FILE="/etc/udev/rules.d/99-pixhawk.rules"

echo "======================================================================"
echo " INSTALLING PIXHAWK UDEV RULES (/dev/pixhawk)"
echo "======================================================================"

if [ "$EUID" -ne 0 ]; then
    echo "Running with sudo..."
    sudo cp "${RULE_FILE}" "${DEST_FILE}"
    sudo udevadm control --reload-rules
    sudo udevadm trigger
else
    cp "${RULE_FILE}" "${DEST_FILE}"
    udevadm control --reload-rules
    udevadm trigger
fi

echo "[+] udev rules installed to ${DEST_FILE}!"
sleep 0.5

if [ -e "/dev/pixhawk" ]; then
    TARGET=$(readlink -f /dev/pixhawk)
    echo "[SUCCESS] /dev/pixhawk is active and points to ${TARGET}!"
else
    echo "[!] /dev/pixhawk will appear as soon as the Pixhawk USB cable is plugged in."
fi
