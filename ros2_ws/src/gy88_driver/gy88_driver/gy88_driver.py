#!/usr/bin/env python3

import smbus2
import time
import math

class GY88:
    """
    GY88 sensor class to interface with the accelerometer, gyroscope and magnetometer.
    This class provides methods to read raw data and convert it to meaningful values.
    """
    # I2C addresses
    MPU6050_ADDR = 0x68  # MPU6050 address (accelerometer and gyroscope)
    HMC5883L_ADDR = 0x1E  # HMC5883L address (magnetometer)
    BMP085_ADDR = 0x77   # BMP085 address (barometer)
    
    # MPU6050 registers
    MPU6050_PWR_MGMT_1 = 0x6B
    MPU6050_ACCEL_XOUT_H = 0x3B
    MPU6050_GYRO_XOUT_H = 0x43
    MPU6050_TEMP_OUT_H = 0x41
    
    # HMC5883L registers
    HMC5883L_CONFIG_A = 0x00
    HMC5883L_CONFIG_B = 0x01
    HMC5883L_MODE = 0x02
    HMC5883L_DATA_X_H = 0x03
    
    # BMP085 registers
    BMP085_CONTROL = 0xF4
    BMP085_DATA = 0xF6
    BMP085_CAL_AC1 = 0xAA
    
    def __init__(self, bus_number=1, calibration=None):
        """Initialize the GY88 sensor interface."""

        self.calibration = calibration or {}
        self.bus = smbus2.SMBus(bus_number)
        self._initialize_sensors()
        
    def _initialize_sensors(self):
        """Initialize all sensors if bus is available."""
        if self.bus is None:
            return
            
        try:
            # Initialize MPU6050
            self.bus.write_byte_data(self.MPU6050_ADDR, self.MPU6050_PWR_MGMT_1, 0)
            
            # Initialize HMC5883L
            # Set to continuous measurement mode
            self.bus.write_byte_data(self.HMC5883L_ADDR, self.HMC5883L_CONFIG_A, 0x70)  # 8 samples, 15Hz
            self.bus.write_byte_data(self.HMC5883L_ADDR, self.HMC5883L_CONFIG_B, 0x20)  # Gain
            self.bus.write_byte_data(self.HMC5883L_ADDR, self.HMC5883L_MODE, 0x00)     # Continuous measurement
        except Exception as e:
            print(f"Warning: Could not initialize sensors: {e}")
        
        # Default calibration parameters
        self.accel_scale = 16384.0  # LSB/g for ±2g range
        self.gyro_scale = 131.0     # LSB/(°/s) for ±250°/s range
        self.accel_offset_x = 0.0
        self.accel_offset_y = 0.0
        self.accel_offset_z = 0.0
        self.gyro_offset_x = 0.0
        self.gyro_offset_y = 0.0
        self.gyro_offset_z = 0.0
        self.mag_scale_x = 1.0
        self.mag_scale_y = 1.0
        self.mag_scale_z = 1.0
        self.mag_offset_x = 0.0
        self.mag_offset_y = 0.0
        self.mag_offset_z = 0.0
        
        # Apply calibration if provided
        if self.calibration:
            if 'mpu6050' in self.calibration:
                mpu_cal = self.calibration['mpu6050']
                if 'accel_offset' in mpu_cal:
                    self.accel_offset_x = mpu_cal['accel_offset'].get('x', 0.0)
                    self.accel_offset_y = mpu_cal['accel_offset'].get('y', 0.0)
                    self.accel_offset_z = mpu_cal['accel_offset'].get('z', 0.0)
                if 'gyro_offset' in mpu_cal:
                    self.gyro_offset_x = mpu_cal['gyro_offset'].get('x', 0.0)
                    self.gyro_offset_y = mpu_cal['gyro_offset'].get('y', 0.0)
                    self.gyro_offset_z = mpu_cal['gyro_offset'].get('z', 0.0)
            
            if 'hmc5883l' in self.calibration:
                hmc_cal = self.calibration['hmc5883l']
                if 'offset' in hmc_cal:
                    self.mag_offset_x = hmc_cal['offset'].get('x', 0.0)
                    self.mag_offset_y = hmc_cal['offset'].get('y', 0.0)
                    self.mag_offset_z = hmc_cal['offset'].get('z', 0.0)
                if 'scale' in hmc_cal:
                    self.mag_scale_x = hmc_cal['scale'].get('x', 1.0)
                    self.mag_scale_y = hmc_cal['scale'].get('y', 1.0)
                    self.mag_scale_z = hmc_cal['scale'].get('z', 1.0)
        
        # Read BMP085 calibration data
        self._read_bmp085_calibration()
        
    def _read_word(self, addr, reg):
        """Read a word (2 bytes) from the specified device address and register."""
        high = self.bus.read_byte_data(addr, reg)
        low = self.bus.read_byte_data(addr, reg + 1)
        value = (high << 8) + low
        return value
    
    def _read_word_2c(self, addr, reg):
        """Read a word as a signed 2's complement value."""
        val = self._read_word(addr, reg)
        if val >= 0x8000:
            return -((65535 - val) + 1)
        else:
            return val
            
    def _read_bmp085_calibration(self):
        """Read calibration data from BMP085 sensor."""
        self.cal_AC1 = self._read_word_2c(self.BMP085_ADDR, self.BMP085_CAL_AC1)
        self.cal_AC2 = self._read_word_2c(self.BMP085_ADDR, self.BMP085_CAL_AC1 + 2)
        self.cal_AC3 = self._read_word_2c(self.BMP085_ADDR, self.BMP085_CAL_AC1 + 4)
        self.cal_AC4 = self._read_word(self.BMP085_ADDR, self.BMP085_CAL_AC1 + 6)
        self.cal_AC5 = self._read_word(self.BMP085_ADDR, self.BMP085_CAL_AC1 + 8)
        self.cal_AC6 = self._read_word(self.BMP085_ADDR, self.BMP085_CAL_AC1 + 10)
        self.cal_B1 = self._read_word_2c(self.BMP085_ADDR, self.BMP085_CAL_AC1 + 12)
        self.cal_B2 = self._read_word_2c(self.BMP085_ADDR, self.BMP085_CAL_AC1 + 14)
        self.cal_MB = self._read_word_2c(self.BMP085_ADDR, self.BMP085_CAL_AC1 + 16)
        self.cal_MC = self._read_word_2c(self.BMP085_ADDR, self.BMP085_CAL_AC1 + 18)
        self.cal_MD = self._read_word_2c(self.BMP085_ADDR, self.BMP085_CAL_AC1 + 20)
    
    def get_accel_data(self):
        """Read and return accelerometer data in g."""
        x = self._read_word_2c(self.MPU6050_ADDR, self.MPU6050_ACCEL_XOUT_H) / self.accel_scale
        y = self._read_word_2c(self.MPU6050_ADDR, self.MPU6050_ACCEL_XOUT_H + 2) / self.accel_scale
        z = self._read_word_2c(self.MPU6050_ADDR, self.MPU6050_ACCEL_XOUT_H + 4) / self.accel_scale
        return {'accel_x': x, 'accel_y': y, 'accel_z': z}
    
    def get_gyro_data(self):
        """Read and return gyroscope data in degrees per second."""
        x = self._read_word_2c(self.MPU6050_ADDR, self.MPU6050_GYRO_XOUT_H) / self.gyro_scale
        y = self._read_word_2c(self.MPU6050_ADDR, self.MPU6050_GYRO_XOUT_H + 2) / self.gyro_scale
        z = self._read_word_2c(self.MPU6050_ADDR, self.MPU6050_GYRO_XOUT_H + 4) / self.gyro_scale
        return {'gyro_x': x, 'gyro_y': y, 'gyro_z': z}
    
    def get_mag_data(self):
        """Read and return magnetometer data with calibration applied."""
        x = (self._read_word_2c(self.HMC5883L_ADDR, self.HMC5883L_DATA_X_H) * self.mag_scale_x) - self.mag_offset_x
        z = (self._read_word_2c(self.HMC5883L_ADDR, self.HMC5883L_DATA_X_H + 2) * self.mag_scale_z) - self.mag_offset_z
        y = (self._read_word_2c(self.HMC5883L_ADDR, self.HMC5883L_DATA_X_H + 4) * self.mag_scale_y) - self.mag_offset_y
        return {'mag_x': x, 'mag_y': y, 'mag_z': z}
    
    def get_temp_data(self):
        """Read and return temperature data from the MPU6050 in Celsius."""
        temp_raw = self._read_word_2c(self.MPU6050_ADDR, self.MPU6050_TEMP_OUT_H)
        temp_celsius = temp_raw / 340.0 + 36.53  # Formula from MPU6050 datasheet
        return temp_celsius
    
    def get_pressure_and_temp(self):
        """Read and return pressure (in Pa) and temperature (in C) from BMP085."""
        # Read uncompensated temperature
        self.bus.write_byte_data(self.BMP085_ADDR, self.BMP085_CONTROL, 0x2E)
        time.sleep(0.005)
        ut = self._read_word(self.BMP085_ADDR, self.BMP085_DATA)
        
        # Read uncompensated pressure
        self.bus.write_byte_data(self.BMP085_ADDR, self.BMP085_CONTROL, 0x34 + (3 << 6))
        time.sleep(0.026)  # Wait for conversion
        msb = self.bus.read_byte_data(self.BMP085_ADDR, self.BMP085_DATA)
        lsb = self.bus.read_byte_data(self.BMP085_ADDR, self.BMP085_DATA + 1)
        xlsb = self.bus.read_byte_data(self.BMP085_ADDR, self.BMP085_DATA + 2)
        up = ((msb << 16) + (lsb << 8) + xlsb) >> (8 - 3)
        
        # Calculate true temperature
        X1 = ((ut - self.cal_AC6) * self.cal_AC5) >> 15
        X2 = (self.cal_MC << 11) // (X1 + self.cal_MD)
        B5 = X1 + X2
        temp = ((B5 + 8) >> 4) / 10.0
        
        # Calculate true pressure
        B6 = B5 - 4000
        X1 = (self.cal_B2 * ((B6 * B6) >> 12)) >> 11
        X2 = (self.cal_AC2 * B6) >> 11
        X3 = X1 + X2
        B3 = (((self.cal_AC1 * 4 + X3) << 3) + 2) // 4
        X1 = (self.cal_AC3 * B6) >> 13
        X2 = (self.cal_B1 * ((B6 * B6) >> 12)) >> 16
        X3 = ((X1 + X2) + 2) >> 2
        B4 = (self.cal_AC4 * (X3 + 32768)) >> 15
        B7 = (up - B3) * (50000 >> 3)
        
        if B7 < 0x80000000:
            p = (B7 * 2) // B4
        else:
            p = (B7 // B4) * 2
            
        X1 = (p >> 8) * (p >> 8)
        X1 = (X1 * 3038) >> 16
        X2 = (-7357 * p) >> 16
        p = p + ((X1 + X2 + 3791) >> 4)
        
        return {'pressure': p, 'temperature': temp}
    
    def get_heading(self):
        """Calculate heading based on magnetometer data."""
        mag = self.get_mag_data()
        heading = math.atan2(mag['y'], mag['x'])
        
        # Adjust for declination
        declination_angle = 0.0  # Set your local declination angle here
        heading += declination_angle
        
        # Normalize to 0-360
        if heading < 0:
            heading += 2 * math.pi
        if heading > 2 * math.pi:
            heading -= 2 * math.pi
            
        heading_degrees = heading * 180 / math.pi
        return heading_degrees
    
    def set_mag_calibration(self, scale_x, scale_y, scale_z, offset_x, offset_y, offset_z):
        """Set magnetometer calibration parameters."""
        self.mag_scale_x = scale_x
        self.mag_scale_y = scale_y
        self.mag_scale_z = scale_z
        self.mag_offset_x = offset_x
        self.mag_offset_y = offset_y
        self.mag_offset_z = offset_z
    
    def set_accel_calibration(self, scale):
        """Set accelerometer scale factor."""
        self.accel_scale = scale
    
    def set_gyro_calibration(self, scale):
        """Set gyroscope scale factor."""
        self.gyro_scale = scale