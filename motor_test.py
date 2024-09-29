import serial
import time

# Set the serial port and baud rate
SERIAL_PORT = '/dev/serial0'  # Update this if necessary
BAUD_RATE = 9600               # Update if your motor driver uses a different baud rate

# Create a serial connection
ser = serial.Serial(SERIAL_PORT, BAUD_RATE)

def convert_speed(m1, m2):
    # Convert motor speeds from -100 to 100 to Sabertooth values
    if m1 == 0 and m2 == 0:
        return [0, 0]

    # Convert m1 (motor 1) speed
    if m1 < 0:
        motor1_speed = max(1, min(127, 1 + (m1 + 100) * (126 / 100)))  # Full reverse is 1, stop is 64
    else:
        motor1_speed = max(1, min(127, 1 + (m1 * (126 / 100))))  # Full forward is 127, stop is 64

    # Convert m2 (motor 2) speed
    if m2 < 0:
        motor2_speed = max(128, min(255, 128 + (m2 + 100) * (126 / 100)))  # Full reverse is 128, stop is 192
    else:
        motor2_speed = max(128, min(255, 128 + (m2 * (126 / 100))))  # Full forward is 255, stop is 192

    return [int(motor1_speed), int(motor2_speed)]

def set_motor_speed(m1, m2):
    motor_speeds = convert_speed(m1, m2)
    ser.write(bytes(motor_speeds))

def run_motors():
    # Example: run motor speeds from full forward to full reverse for both motors
    set_motor_speed(-100, 0)
    time.sleep(1)

    # Shut down both motors
    set_motor_speed(0, 0)
    print("Motors shut down.")

try:
    run_motors()
finally:
    ser.close()  # Ensure the serial port is closed after operation
