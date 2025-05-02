#!/usr/bin/env python3

from gpiozero import PWMOutputDevice, DigitalOutputDevice, DigitalInputDevice
from signal import pause

# Constants
MAX_SPEED = 480

# Pin assignments
_pin_M1FLT = 5
_pin_M2FLT = 6
_pin_M1PWM = 12
_pin_M2PWM = 13
_pin_M1EN = 22
_pin_M2EN = 23
_pin_M1DIR = 24
_pin_M2DIR = 25

class Motor:
    def __init__(self, pwm_pin, dir_pin, en_pin, flt_pin):
        self.pwm = PWMOutputDevice(pwm_pin, frequency=1000, initial_value=0)
        self.dir = DigitalOutputDevice(dir_pin, initial_value=False)
        self.en = DigitalOutputDevice(en_pin, initial_value=True)  # Enable by default
        self.flt = DigitalInputDevice(flt_pin, pull_up=True)

    def setSpeed(self, speed):
        if speed < 0:
            self.dir.on()
            speed = -speed
        else:
            self.dir.off()

        if speed > MAX_SPEED:
            speed = MAX_SPEED

        duty_cycle = speed / MAX_SPEED
        self.pwm.value = duty_cycle

    def enable(self):
        self.en.on()

    def disable(self):
        self.en.off()
        self.pwm.value = 0

    def getFault(self):
        return not self.flt.value  # FLT is active-low

    def cleanup(self):
        self.pwm.close()
        self.dir.close()
        self.en.close()
        self.flt.close()

class Motors:
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

    def cleanup(self):
        self.motor1.cleanup()
        self.motor2.cleanup()

# Example usage
if __name__ == "__main__":
    motors = Motors()
    try:
        print("Running motors at half speed for 2 seconds...")
        motors.setSpeeds(240, 240)
        pause()  # Or replace with `time.sleep(2)` for test
    except KeyboardInterrupt:
        print("Stopping motors.")
    finally:
        motors.forceStop()
        motors.cleanup()
