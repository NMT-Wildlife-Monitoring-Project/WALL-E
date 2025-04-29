#!/usr/bin/python

import serial
import time

ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)
ser.flushInput()

ser.write(b'AT+CGPS=1\r\n')
time.sleep(1)
print(ser.read(ser.inWaiting()).decode('utf-8'))
ser.close()
