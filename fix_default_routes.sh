#!/usr/bin/env bash
# fix_default_routes.sh
# Prefer wlan0 for default route (lower metric), fall back to wwan0 (higher metric)

set -e

# Ensure script is run as root (re-exec with sudo if not)
if [[ $EUID -ne 0 ]]; then
    echo "Re-running as root via sudo…"
    exec sudo "$0" "$@"
fi

WWAN_DEV=wwan0

echo "Removing default route for $WWAN_DEV..."
# Get the current default gateway for wwan0
GATEWAY=$(ip route show dev $WWAN_DEV default | awk '{print $3}')

if [ -n "$GATEWAY" ]; then
    # Delete the current default route
    ip route del default via $GATEWAY dev $WWAN_DEV

    # Add it back with metric 100
    echo "Adding default route for $WWAN_DEV with metric 100..."
    ip route add default via $GATEWAY dev $WWAN_DEV metric 100
else
    echo "No default route found for $WWAN_DEV"
fi