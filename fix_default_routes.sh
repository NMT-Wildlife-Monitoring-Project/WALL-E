#!/bin/bash

# Find current wwan0 default route
WWAN_ROUTE=$(ip route | grep '^default via' | grep 'dev wwan0' | awk '{print $3}')
METRIC=100

if [ -n "$WWAN_ROUTE" ]; then
    echo "Fixing default route for wwan0 via $WWAN_ROUTE with metric $METRIC"

    # Remove the old route (no metric)
    sudo ip route del default via "$WWAN_ROUTE" dev wwan0

    # Add the route back with preferred metric
    sudo ip route add default via "$WWAN_ROUTE" dev wwan0 metric $METRIC
else
    echo "No default route found for wwan0"
fi
