import serial

# Setup serial communication
# Adjust the port if necessary (e.g., '/dev/serial0' or '/dev/ttyAMA0')
ser = serial.Serial('/dev/serial0', baudrate=9600, timeout=1)

def motor_speed_to_byte(speed, motor_num):
    """
    Convert motor speed (-100 to 100) to a byte value for Sabertooth.
    Motor 1 uses values from 1-127 and Motor 2 uses values from 128-255.
    """
    if speed < -100 or speed > 100:
        raise ValueError("Speed must be between -100 and 100")

    # Stop condition is 64 for motor 1, 192 for motor 2
    stop_offset = 64 if motor_num == 1 else 192

    # Map speed from -100 to 100 into 1 to 127 for motor 1 and 128 to 255 for motor 2
    if speed == 0:
        return stop_offset  # Stop byte
    elif motor_num == 1:
        return int(63 * (speed / 100.0)) + 64  # Motor 1 range: 1 to 127
    else:
        return int(63 * (speed / 100.0)) + 192  # Motor 2 range: 128 to 255

def send_motor_commands(left_speed, right_speed):
    """
    Send serial commands to Sabertooth for both motors.
    If both speeds are 0, send 0x00 to shut down motors.
    """
    if left_speed == 0 and right_speed == 0:
        # Send 0x00 to stop both motors
        ser.write(b'\x00')
    else:
        # Convert and send left motor speed (Motor 1)
        left_byte = motor_speed_to_byte(left_speed, 1)
        ser.write(bytes([left_byte]))

        # Convert and send right motor speed (Motor 2)
        right_byte = motor_speed_to_byte(right_speed, 2)
        ser.write(bytes([right_byte]))

left_speed = 0
right_speed = 0

send_motor_commands(left_speed, right_speed)
