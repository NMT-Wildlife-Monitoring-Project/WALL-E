#!/bin/bash
sudo timeout 2 gpsctl -x "AT+CGPS=1" /dev/ttyUSB3
