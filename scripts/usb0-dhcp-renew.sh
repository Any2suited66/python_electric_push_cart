#!/bin/bash
# Renew DHCP on usb0 after phone USB tethering comes up.
# Prevents keeping a stale lease (e.g. 10.141.179.2) while the phone
# is on a new tether subnet (e.g. 10.141.216.x).
#
# USB tether is ONLY for Cart Follow TCP (:9747). Do not let the phone
# become the default gateway — that steals Wi‑Fi / LAN connectivity.
set -euo pipefail

IFACE="${1:-usb0}"

if [[ ! -d "/sys/class/net/$IFACE" ]]; then
  echo "usb0-dhcp-renew: $IFACE not present"
  exit 0
fi

# Wait briefly for carrier / RNDIS bring-up
for _ in $(seq 1 20); do
  if [[ -e "/sys/class/net/$IFACE/carrier" ]] && [[ "$(cat "/sys/class/net/$IFACE/carrier")" == "1" ]]; then
    break
  fi
  sleep 0.25
done

ip link set "$IFACE" up || true
ip addr flush dev "$IFACE" || true

if command -v dhcpcd >/dev/null 2>&1; then
  dhcpcd -k "$IFACE" 2>/dev/null || true
  dhcpcd -n "$IFACE" || dhcpcd "$IFACE"
elif command -v dhclient >/dev/null 2>&1; then
  dhclient -r "$IFACE" 2>/dev/null || true
  dhclient -v "$IFACE"
else
  echo "usb0-dhcp-renew: need dhcpcd or dhclient" >&2
  exit 1
fi

# Drop default routes via USB so wlan0/eth0 keep internet / SSH.
while ip route del default dev "$IFACE" 2>/dev/null; do
  :
done

echo "usb0-dhcp-renew: $(ip -4 -o addr show dev "$IFACE" | awk '{print $4}')"
echo "usb0-dhcp-renew: routes: $(ip -4 route | tr '\n' '; ')"
