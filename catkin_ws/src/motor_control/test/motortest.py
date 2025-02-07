import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)

MOTOR1_PIN = 13
MOTOR2_PIN = 12
GPIO.setup(MOTOR1_PIN,GPIO.OUT)

m1 = GPIO.PWM(MOTOR1_PIN,5000)

m1.start(25)

time.sleep(1)

m1.stop
GPIO.cleanup()
