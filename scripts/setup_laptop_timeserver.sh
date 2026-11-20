#!/bin/bash
# One-time setup on the laptop: serve time (NTP) to the drone's Raspberry Pi.
#   sudo ./scripts/setup_laptop_timeserver.sh
#
# start_streams.py on the Pi then syncs the Pi clock to this laptop at every start, so
# Pi timestamps and laptop clock agree to ~1 ms and the /metrics/*/latency_ms values are real.
set -e
[ "$(id -u)" -eq 0 ] || { echo "Run with sudo: sudo $0"; exit 1; }

echo "== chrony"
if ! dpkg -s chrony >/dev/null 2>&1; then
  apt-get install -y chrony          # replaces systemd-timesyncd; keeps syncing to the internet
fi

CONF=/etc/chrony/conf.d/gps-denied-vtol.conf
mkdir -p /etc/chrony/conf.d
grep -q '^confdir /etc/chrony/conf.d' /etc/chrony/chrony.conf || \
  echo 'confdir /etc/chrony/conf.d' >> /etc/chrony/chrony.conf
cat > "$CONF" <<'EOF'
# Serve time to the drone's Raspberry Pi on the local network (private address ranges only).
allow 10.0.0.0/8
allow 172.16.0.0/12
allow 192.168.0.0/16
# Keep serving even when this laptop has no internet time source: the Pi only needs to
# agree with THIS clock, not with world time.
local stratum 10
EOF
echo "   $CONF written"

echo "== firewall"
if command -v ufw >/dev/null && ufw status | grep -q "Status: active"; then
  ufw allow 123/udp comment 'NTP for drone Pi' >/dev/null && echo "   ufw: 123/udp allowed"
else
  echo "   ufw not active, nothing to do"
fi

systemctl enable --now chrony >/dev/null 2>&1 || true
systemctl restart chrony
sleep 1
echo "== status"
chronyc -n tracking | grep -E "Reference ID|Stratum|System time"
echo "Done. The Pi syncs to this laptop automatically when start_streams.py starts."
