#!/bin/bash

# Wait a few seconds for USB devices to come online
sleep 5

# Set NDIS mode and APN (adjust APN as needed)
echo -e "AT+CNMP=38\r" > /dev/ttyUSB2
sleep 1
echo -e "AT+CMNB=1\r" > /dev/ttyUSB2
sleep 1

# Turn on GPS
echo -e "AT+CGPS=1\r" > /dev/ttyUSB3
sleep 1

# Try getting IP via DHCP (non-blocking)
dhclient wwan0

