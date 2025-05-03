import RPi.GPIO as GPIO
import time

# Setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

MAX_SPEED = 480

_pin_M1FLT = 5
_pin_M2FLT = 6
_pin_M1PWM = 12
_pin_M2PWM = 13
_pin_M1EN = 22
_pin_M2EN = 23
_pin_M1DIR = 24
_pin_M2DIR = 25

GPIO.setup([_pin_M1FLT, _pin_M2FLT], GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup([_pin_M1EN, _pin_M2EN, _pin_M1DIR, _pin_M2DIR], GPIO.OUT)
GPIO.setup([_pin_M1PWM, _pin_M2PWM], GPIO.OUT)

_pwm_channels = {
    _pin_M1PWM: GPIO.PWM(_pin_M1PWM, 20000),  # 20kHz
    _pin_M2PWM: GPIO.PWM(_pin_M2PWM, 20000)
}
_pwm_channels[_pin_M1PWM].start(0)
_pwm_channels[_pin_M2PWM].start(0)

class Motor(object):
    MAX_SPEED = MAX_SPEED

    def __init__(self, pwm_pin, dir_pin, en_pin, flt_pin):
        self.pwm_pin = pwm_pin
        self.dir_pin = dir_pin
        self.en_pin = en_pin
        self.flt_pin = flt_pin
        GPIO.output(self.en_pin, 1)  # Enable driver

    def setSpeed(self, speed):
        if speed < 0:
            speed = -speed
            dir_value = 1
        else:
            dir_value = 0

        speed = min(speed, MAX_SPEED)
        duty_cycle = (speed / MAX_SPEED) * 100  # Convert to 0–100%

        GPIO.output(self.dir_pin, dir_value)
        _pwm_channels[self.pwm_pin].ChangeDutyCycle(duty_cycle)

    def enable(self):
        GPIO.output(self.en_pin, 1)

    def disable(self):
        GPIO.output(self.en_pin, 0)

    def getFault(self):
        return not GPIO.input(self.flt_pin)

class Motors(object):
    MAX_SPEED = MAX_SPEED

    def __init__(self):
        self.motor1 = Motor(_pin_M1PWM, _pin_M1DIR, _pin_M1EN, _pin_M1FLT)
        self.motor2 = Motor(_pin_M2PWM, _pin_M2DIR, _pin_M2EN, _pin_M2FLT)

    def setSpeeds(self, m1_speed, m2_speed):
        self.motor1.setSpeed(m1_speed)
        self.motor2.setSpeed(m2_speed)

    def enable(self):
        self.motor1.enable()
        self.motor2.enable()

    def disable(self):
        self.motor1.disable()
        self.motor2.disable()

    def getFaults(self):
        return self.motor1.getFault() or self.motor2.getFault()

    def forceStop(self):
        self.setSpeeds(0, 0)

motors = Motors()
