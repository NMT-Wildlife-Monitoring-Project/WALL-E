#!/usr/bin/env bash
# fix_default_routes.sh
# Set wwan0 default route with metric 100 and wlan0 with metric 600

set -e

# Ensure script is run as root (re-exec with sudo if not)
if [[ $EUID -ne 0 ]]; then
    echo "Re-running as root via sudo…"
    exec sudo "$0" "$@"
fi

WWAN_DEV=wwan0
WLAN_DEV=wlan0
WWAN_METRIC=600
WLAN_METRIC=100

# Fix wwan0 default route
echo "Configuring default route for $WWAN_DEV..."
WWAN_GATEWAY=$(ip route show dev $WWAN_DEV default | awk '{print $3}')

if [ -n "$WWAN_GATEWAY" ]; then
    # Delete the current default route
    ip route del default via $WWAN_GATEWAY dev $WWAN_DEV
    
    # Add it back with metric 100
    echo "Adding default route for $WWAN_DEV with metric $WWAN_METRIC..."
    ip route add default via $WWAN_GATEWAY dev $WWAN_DEV metric $WWAN_METRIC
else
    echo "No default route found for $WWAN_DEV"
fi

# Fix wlan0 default route
echo "Configuring default route for $WLAN_DEV..."
WLAN_GATEWAY=$(ip route show dev $WLAN_DEV default | awk '{print $3}')

if [ -n "$WLAN_GATEWAY" ]; then
    # Delete the current default route
    ip route del default via $WLAN_GATEWAY dev $WLAN_DEV
    
    # Add it back with metric 600
    echo "Adding default route for $WLAN_DEV with metric $WLAN_METRIC..."
    ip route add default via $WLAN_GATEWAY dev $WLAN_DEV metric $WLAN_METRIC proto dhcp
else
    echo "No default route found for $WLAN_DEV"
fi

echo "Current default routes:"
ip route show default