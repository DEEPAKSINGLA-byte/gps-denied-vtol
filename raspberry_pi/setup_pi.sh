#!/bin/bash
# One-time setup on the Raspberry Pi. Run from ~/raspberry_pi:   sudo ./setup_pi.sh
#
#  1. udev rules (/dev/pixhawk, /dev/camera) and serial access (dialout group)
#  2. chrony + iw installed (time sync to the laptop, Wi-Fi power saving)
#  3. Narrow passwordless sudo, ONLY for the commands start_streams.py runs:
#       iw dev <if> set power_save off
#       chronyc add server <laptop> ... / chronyc delete <laptop> / chronyc makestep
#  4. Wi-Fi power saving off for the current Wi-Fi connection (survives reboots)
set -e
[ "$(id -u)" -eq 0 ] || { echo "Run with sudo: sudo ./setup_pi.sh"; exit 1; }
U="${SUDO_USER:-deimos}"
cd "$(dirname "$0")"

echo "== 1. udev rules + serial access for $U"
[ -x ./install_udev_rules.sh ] && ./install_udev_rules.sh >/dev/null && echo "   udev rules installed"
usermod -aG dialout "$U"

echo "== 2. packages"
for pkg in chrony iw; do
  if dpkg -s "$pkg" >/dev/null 2>&1; then echo "   $pkg: installed"
  else apt-get install -y "$pkg" || { echo "   could not install $pkg (no internet? see docs/SETUP.md)"; exit 1; }; fi
done

echo "== 3. passwordless sudo for iw / chronyc (start_streams.py)"
RULE=/etc/sudoers.d/gps-denied-vtol
TMP=$(mktemp)
cat > "$TMP" <<EOF
# start_streams.py: Wi-Fi power saving off + sync the clock to the laptop. Nothing else.
$U ALL=(root) NOPASSWD: /usr/sbin/iw dev * set power_save off, /usr/bin/chronyc add server *, /usr/bin/chronyc delete *, /usr/bin/chronyc makestep
EOF
visudo -cf "$TMP" >/dev/null && install -m 440 -o root -g root "$TMP" "$RULE" && echo "   $RULE installed"
rm -f "$TMP"
rm -f /etc/sudoers.d/iw-powersave   # older single-purpose rule, now covered above

echo "== 4. Wi-Fi power saving off (persistent)"
if command -v nmcli >/dev/null; then
  nmcli -t -f NAME,TYPE connection show --active | awk -F: '$2=="802-11-wireless"{print $1}' |
  while read -r c; do nmcli connection modify "$c" 802-11-wireless.powersave 2 && echo "   '$c': power save off"; done
fi
for path in /sys/class/net/*/wireless; do
  [ -d "$path" ] || continue
  dev=$(basename "$(dirname "$path")")
  /usr/sbin/iw dev "$dev" set power_save off 2>/dev/null && echo "   $dev: off now"
done

echo "Done. Log out and back in once (dialout group). Then: python3 start_streams.py --nav-cmds"
