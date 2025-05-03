# motor.py — Pololu Dual G2 API using libgpiod instead of pigpio
import gpiod
import time

# Constants
_max_speed = 480
MAX_SPEED = _max_speed

# GPIO mapping
_pin_M1FLT = 5
_pin_M2FLT = 6
_pin_M1PWM = 12  # PWM functionality must be handled separately
_pin_M2PWM = 13
_pin_M1EN = 22
_pin_M2EN = 23
_pin_M1DIR = 24
_pin_M2DIR = 25

# Initialize GPIO chip (using first available gpiochip)
chip = gpiod.Chip(gpiod.gpiochip_find(0))

# Helper to request lines
def request_line(offset, direction, default=0, pull=None):
    config = gpiod.LineRequest()
    config.consumer = "motor_driver"
    config.request_type = (gpiod.LineRequest.DIRECTION_INPUT if direction == 'in'
                           else gpiod.LineRequest.DIRECTION_OUTPUT)
    if pull == 'up':
        config.flags = gpiod.LineRequest.FLAG_BIAS_PULL_UP
    elif pull == 'down':
        config.flags = gpiod.LineRequest.FLAG_BIAS_PULL_DOWN
    line = chip.get_line(offset)
    line.request(config, default)
    return line

# Stub for hardware PWM — users should implement using sysfs or appropriate PWM library
def hardware_PWM(pin, frequency, duty_cycle):
    raise NotImplementedError("PWM on pin {} not implemented; use linux-pwm or outro lib".format(pin))

class Motor(object):
    MAX_SPEED = _max_speed

    def __init__(self, pwm_pin, dir_pin, en_pin, flt_pin):
        self.pwm_pin = pwm_pin
        self.dir_pin = dir_pin
        self.en_pin = en_pin
        self.flt_pin = flt_pin
        
        # Request lines
        self.flt_line = request_line(flt_pin, 'in', pull='up')
        self.en_line = request_line(en_pin, 'out', default=1)
        self.dir_line = request_line(dir_pin, 'out', default=0)
        # PWM line requested as output (level), actual PWM must be handled
        self.pwm_line = request_line(pwm_pin, 'out', default=0)

    def setSpeed(self, speed):
        # Determine direction
        if speed < 0:
            dir_value = 1
            speed = -speed
        else:
            dir_value = 0
        # Cap speed
        if speed > MAX_SPEED:
            speed = MAX_SPEED
        # Set direction and enable
        self.dir_line.set_value(dir_value)
        # Invoke PWM stub (raise or handle externally)
        hardware_PWM(self.pwm_pin, 20000, int(speed * 6250 / 3))

    def enable(self):
        self.en_line.set_value(1)

    def disable(self):
        self.en_line.set_value(0)

    def getFault(self):
        # Fault line is active-low
        return not bool(self.flt_line.get_value())

class Motors(object):
    MAX_SPEED = _max_speed

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
        # Reset outputs to safe state
        self.disable()
        self.setSpeeds(0, 0)

# Instantiate module-level objects
motors = Motors()
