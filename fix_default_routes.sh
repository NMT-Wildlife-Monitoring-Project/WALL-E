#!/bin/bash

# Find the current default route for wwan0
WWAN_ROUTE=$(ip route | grep '^default via' | grep 'dev wwan0' | awk '{print $3}')

# Set a lower metric (lower = higher priority)
METRIC=100

if [ -n "$WWAN_ROUTE" ]; then
    echo "Setting default route for wwan0 via $WWAN_ROUTE with metric $METRIC"
    sudo ip route replace default via "$WWAN_ROUTE" dev wwan0 metric $METRIC
else
    echo "No default route found for wwan0"
fi
