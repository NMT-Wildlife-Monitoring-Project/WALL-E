#!/usr/bin/env python3
import busio
from adafruit_bno08x.i2c import BNO08X_I2C
import numpy as np
import time
from tf_transformations import euler_from_quaternion
import warnings


from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    # BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR,
)

class BNO085:
    def __init__(self, i2c_addr):
        self.i2c = busio.I2C(3, 2)
        self.bno = BNO08X_I2C(self.i2c, address=i2c_addr)
        features = [
            BNO_REPORT_ACCELEROMETER,
            BNO_REPORT_GYROSCOPE,
            BNO_REPORT_MAGNETOMETER,
            # BNO_REPORT_ROTATION_VECTOR,
            BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR
        ]
        for feature in features:
            for attempt in range(1, 4):
                try:
                    self.bno.enable_feature(feature)
                    break
                except Exception as e2:
                    warnings.warn(f"Attempt {attempt} to enable feature {feature} failed: {e2}")
                    time.sleep(0.5)
            raise RuntimeError(f"Failed to enable feature {feature} after 3 attempts.")
        
        self.quat = np.zeroes(4)
        self.rpy = np.zeroes(3)
        self.rpy_covariance = np.array([
            1e-4, 0, 0,
            0, 1e-4, 0,
            0, 0, 1e-4
        ])
        self.accel = np.zeroes(3)
        self.accel_covariance = np.array([
            1e-4, 0, 0,
            0, 1e-4, 0,
            0, 0, 1e-4
        ])
        self.gyro = np.zeroes(3)
        self.gyro_covariance = np.array([
            1e-4, 0, 0,
            0, 1e-4, 0,
            0, 0, 1e-4
        ])
        self.mag = np.zeroes(3)
        self.mag_covariance = np.array([
            1e-4, 0, 0,
            0, 1e-4, 0,
            0, 0, 1e-4
        ])


    def calibrate(self, num_samples=100):
        quat_samples = np.array((len(self.quat), num_samples))
        rpy_samples = np.array((len(self.rpy), num_samples))
        gyro_samples = np.array((len(self.gyro), num_samples))
        accel_samples = np.array((len(self.accel), num_samples))
        mag_samples = np.array((len(self.mag), num_samples))
        
        # Get samples (1 second)
        for i in range(num_samples):
            self.get_data()
            quat_samples[:,i] = self.quat
            rpy_samples[:,i] = self.rpy
            gyro_samples[:,i] = self.gyro
            accel_samples[:,i] = self.accel
            mag_samples[:,i] = self.mag
            time.sleep(0.01)  # Adjust delay as needed based on sensor update rate
        
        # Calculate Gyro and Accel bias
        self.gryo_bias = np.mean(gyro_samples, axis=1)
        self.accel_bias = np.mean(accel_samples, axis=1)

        # Calculate variance
        rpy_var = np.var(rpy_samples, axis=1)
        gyro_var = np.var(gyro_samples, axis=1)
        accel_var = np.var(accel_samples, axis=1)
        mag_var = np.var(mag_samples, axis=1)

        self.rpy_covariance = np.diag(rpy_var)
        self.gyro_covariance = np.diag(gyro_var)
        self.accel_covariance = np.diag(accel_var)
        self.mag_covariance = np.diag(mag_var)

    def get_data(self):
        self.quat = np.array(self.bno.geomagnetic_quaternion)
        self.rpy = euler_from_quaternion(self.quat)
        self.accel = np.array(self.bno.acceleration)
        self.gyro = np.array(self.bno.gyro)
        self.mag = np.array(self.bno.magnetic)

        if self.accel is None or self.gyro is None or self.quat is None:
            raise RuntimeError("Received invalid data from BNO085 sensor.")        
    

