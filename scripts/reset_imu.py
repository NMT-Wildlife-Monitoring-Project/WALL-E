#!/usr/bin/env python3
import Jetson.GPIO as GPIO
import time
import sys

# Pin Definitions (Physical Header Numbers)
RST_PIN = 7

def main():
    # Setup GPIO using Board numbering
    GPIO.setmode(GPIO.BOARD)
    
    # Configure Pin 7 as Output
    # BNO085 Reset is Active-Low, so start it HIGH
    GPIO.setup(RST_PIN, GPIO.OUT, initial=GPIO.HIGH)
    
    print(f"Starting hardware reset on Physical Pin {RST_PIN}...")
    
    # Trigger Reset: Pull LOW for 10-100ms
    GPIO.output(RST_PIN, GPIO.LOW)
    time.sleep(0.05) # 50ms pulse
    GPIO.output(RST_PIN, GPIO.HIGH)
    
    print("Reset complete. Waiting for IMU to boot...")
    time.sleep(0.5) # Allow time for internal IMU initialization
    
    # Clean up so other applications (like ROS nodes) can take control
    GPIO.cleanup(RST_PIN)
    print("Done.")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        GPIO.cleanup(RST_PIN)
        sys.exit(0)
