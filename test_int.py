import time
import Jetson.GPIO as GPIO
from adafruit_extended_bus import ExtendedI2C
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR

# --- Configuration ---
INT_PIN = 15  # Physical Pin 15 (LCD_TE)

# --- Setup GPIO ---
GPIO.cleanup()
GPIO.setmode(GPIO.BOARD)
GPIO.setup(INT_PIN, GPIO.IN)

# --- Callback ---
def data_ready_handler(channel):
    global data_ready
    data_ready = True

# Add Event Detect (Falling Edge = Data Ready)
# BNO085 pulls line LOW when data is available
try:
    GPIO.add_event_detect(INT_PIN, GPIO.FALLING, callback=data_ready_handler)
except RuntimeError as e:
    print(f"GPIO Error: {e}")
    print("Tip: Ensure you are running as sudo or have gpio permissions.")

# --- Setup Sensor ---
try:
    i2c = ExtendedI2C(7)
    bno = BNO08X_I2C(i2c, address=0x4B)
    bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
    print(f"Sensor Connected. Waiting for INT on Pin {INT_PIN}...")
except ValueError:
    print("I2C Error: Sensor not found. Check bus 7 at address 0x4B.")

data_ready = False

try:
    while True:
        if data_ready:
            quat_i, quat_j, quat_k, quat_real = bno.quaternion
            print(f"INT Received! Quat I: {quat_i:.3f}")
            data_ready = False
        time.sleep(0.01)

except KeyboardInterrupt:
    GPIO.cleanup()
    print("\nCleaned up GPIO.")
