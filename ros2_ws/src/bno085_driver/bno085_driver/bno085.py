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
    BNO_REPORT_ROTATION_VECTOR,
)

class BNO085:
    """
    BNO085 sensor driver class for interfacing with the BNO085 IMU via I2C.
    This class provides methods to initialize the sensor, enable desired features,
    calibrate sensor biases and covariances, and update sensor readings.
    Attributes:
        i2c (busio.I2C): I2C bus instance used for communication.
        bno (BNO08X_I2C): BNO085 sensor object.
        quat (np.ndarray): Quaternion representing sensor orientation.
        rpy (np.ndarray): Roll, pitch, yaw angles derived from quaternion.
        rpy_covariance (np.ndarray): Covariance matrix for RPY angles.
        accel (np.ndarray): Accelerometer readings (m/s^2).
        accel_covariance (np.ndarray): Covariance matrix for accelerometer.
        gyro (np.ndarray): Gyroscope readings (rad/s).
        gyro_covariance (np.ndarray): Covariance matrix for gyroscope.
        mag (np.ndarray): Magnetometer readings (uT).
        mag_covariance (np.ndarray): Covariance matrix for magnetometer.
        gryo_bias (np.ndarray): Estimated gyroscope bias.
        accel_bias (np.ndarray): Estimated accelerometer bias.
    Methods:
        __init__(i2c_addr):
            Initializes the BNO085 sensor, enables features, and sets up data structures.
        calibrate(num_samples=100):
            Collects samples to estimate sensor biases and covariances for calibration.
            Expects imu to be stationary. Do not run this function to use default covariance
            and zero bias.
        update():
            Updates sensor readings for quaternion, RPY, accelerometer, gyroscope, and magnetometer
            and subtracts bias.
    """
    def __init__(self, i2c_addr):
        self.i2c = busio.I2C(3, 2)
        self.bno = BNO08X_I2C(self.i2c, address=i2c_addr)
        features = [
            BNO_REPORT_ACCELEROMETER,
            BNO_REPORT_GYROSCOPE,
            BNO_REPORT_MAGNETOMETER,
            # rotation vector (fused orientation)
            BNO_REPORT_ROTATION_VECTOR,
        ]
        for feature in features:
            for attempt in range(1, 4):
                try:
                    self.bno.enable_feature(feature)
                    break
                except Exception as e2:
                    warnings.warn(f"Attempt {attempt} to enable feature {feature} failed: {e2}")
                    time.sleep(0.5)
            else:
                raise RuntimeError(f"Failed to enable feature {feature} after 3 attempts.")
        
        self.quat = np.zeros(4)
        self.rpy = np.zeros(3)
        self.rpy_covariance = np.array([
            7.68e-6,    0,      0,
            0,          4.1e-6,  0,
            0,          0,      1.615e-5,
        ])
        self.accel = np.zeros(3)
        self.accel_covariance = np.array([
            0.0003155, 0, 0,
            0,  0.001256, 0,
            0, 0, 0.000849
        ])
        self.gyro = np.zeros(3)
        self.gyro_covariance = np.array([
            1e-4,  0,          0,
            0,         1e-4,   0,
            0,          0,      1e-4    
        ])
        self.mag = np.zeros(3)
        self.mag_covariance = np.array([
            1e-4, 0, 0,
            0, 1e-4, 0,
            0, 0, 1e-4
        ])

        self.gryo_bias = np.zeros(3)
        self.accel_bias = np.zeros(3)

    def calibrate(self, num_samples=100):
        quat_samples = np.zeros((len(self.quat), num_samples))
        rpy_samples = np.zeros((len(self.rpy), num_samples))
        gyro_samples = np.zeros((len(self.gyro), num_samples))
        accel_samples = np.zeros((len(self.accel), num_samples))
        mag_samples = np.zeros((len(self.mag), num_samples))
        
        # Get samples (1 second)
        for i in range(num_samples):
            self.update()
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
        rpy_var = np.var(rpy_samples, axis=1, ddof=1)
        gyro_var = np.var(gyro_samples, axis=1, ddof=1)
        accel_var = np.var(accel_samples, axis=1, ddof=1)
        mag_var = np.var(mag_samples, axis=1, ddof=1)

        self.rpy_covariance = np.diag(rpy_var).flatten()
        self.gyro_covariance = np.diag(gyro_var).flatten()
        self.accel_covariance = np.diag(accel_var).flatten()
        self.mag_covariance = np.diag(mag_var).flatten()

    def update(self):
        self.quat = np.array(self.bno.quaternion)
        self.rpy = np.array(euler_from_quaternion(self.quat))
        self.accel = np.array(self.bno.acceleration) - self.accel_bias
        self.gyro = np.array(self.bno.gyro) - self.gryo_bias
        self.mag = np.array(self.bno.magnetic)

        if self.accel is None or self.gyro is None or self.quat is None:
            raise RuntimeError("Received invalid data from BNO085 sensor.")


