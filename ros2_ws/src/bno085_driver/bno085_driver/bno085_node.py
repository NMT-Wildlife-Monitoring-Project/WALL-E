#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Header
import busio
from adafruit_bno08x.i2c import BNO08X_I2C
import time
import os
import numpy as np
from tf_transformations import euler_from_quaternion


from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    # BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR,
)

#TODO Refactor
#TODO fix high pass filter

class BNO085Node(Node):

    def __init__(self):
        super().__init__('bno085_node')
        # Declare parameter for I2C address
        self.declare_parameter('i2c_address', 0x4B)  # Default BNO085 address
        i2c_address = self.get_parameter('i2c_address').value
        
        # Declare parameter for frame ID
        self.declare_parameter('frame_id', 'imu_link')  # Default frame ID
        self.frame_id = self.get_parameter('frame_id').value
        
        # Publishers
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)
        
        # try:
        #     # Initialize I2C and BNO085 using board library
        #     self.i2c = busio.I2C(board.SCL, board.SDA)
        # except RuntimeError as e:
        #     self.get_logger().warning(f"Failed to initialize I2C using board library: {e}")
        #     self.get_logger().info("Falling back to direct I2C bus access.")
        #     # Fallback to direct I2C bus access
        self.i2c = busio.I2C(3, 2)  # Default I2C bus pins for Raspberry Pi
        
        self.initialize_bno085()
        self.get_logger().info(f'Using I2C address: 0x{i2c_address:02X}')
        
        # Create timer for publishing
        self.timer = self.create_timer(0.01, self.publish_imu_data)  # 100Hz

        self.rollingArray = np.zeros((9,200))

        self.get_logger().info('BNO085 Node initialized')
    
    def initialize_bno085(self):
        self.bno = BNO08X_I2C(self.i2c, address=self.get_parameter('i2c_address').value)
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
                    self.get_logger().warning(f"Attempt {attempt} to enable feature {feature} failed: {e2}")
                    time.sleep(0.5)

        #TODO collect bias
        #TODO test variance in init not rolling
        self.get_logger().info("BNO085 initialization complete.")

    def initialize_imu_msg(self):
        # Create IMU message
        self.imu_msg = Imu()
        # Header
        self.imu_msg.header = Header()
        self.imu_msg.header.stamp = self.get_clock().now().to_msg()
        self.imu_msg.header.frame_id = self.frame_id
        #TODO add bias
        self.rollingArray = np.roll(self.rollingArray,1,1)

    def get_imu_data(self):
        # Get sensor data
        self.accel = self.bno.acceleration
        self.gyro = self.bno.gyro
        self.quat = self.bno.geomagnetic_quaternion
        self.mag = self.bno.magnetic

        # Check if data is valid
        if self.accel is None or self.gyro is None or self.quat is None:
            self.get_logger().warning("Received invalid data from BNO085 sensor.")
            return
    
    def publish_mag(self):
        # Publish magnetometer data if available
        if self.mag is not None:
            mag_msg = MagneticField()
            mag_msg.header = Header()
            mag_msg.header.stamp = self.get_clock().now().to_msg()
            mag_msg.header.frame_id = 'imu_link'

            # Magnetic field (Tesla)
            mag_msg.magnetic_field.x = self.mag[0] * 1e-6  # Convert µT to T
            mag_msg.magnetic_field.y = self.mag[1] * 1e-6
            mag_msg.magnetic_field.z = self.mag[2] * 1e-6

            # Covariance (set to unknown)
            mag_msg.magnetic_field_covariance = [-1.0] * 9

            self.mag_pub.publish(mag_msg)

    def imu_msg_orientation(self): 
        # Orientation (quaternion)
        self.imu_msg.orientation.x = self.quat[0]
        self.imu_msg.orientation.y = self.quat[1]
        self.imu_msg.orientation.z = self.quat[2]
        self.imu_msg.orientation.w = self.quat[3]

    def orientation_variance(self):
        self.roll, self.pitch, self.yaw = euler_from_quaternion([self.quat[0], self.quat[1], self.quat[2], self.quat[3]])
        self.rollingArray[0][0] = self.roll
        self.rollingArray[1][0] = self.pitch
        self.rollingArray[2][0] = self.yaw
        self.imu_msg.orientation_covariance = [
            self.varianceArray[0], 0,       0,
            0,       self.varianceArray[1], 0,
            0,       0,       self.varianceArray[2]
        ]
    def log_orientation(self):
        self.get_logger().debug(f"Orientation: Roll={self.roll}, Pitch={self.pitch}, Yaw={self.yaw}")
            
    def imu_msg_angular(self): 
        # Angular velocity (rad/s)
        self.imu_msg.angular_velocity.x = self.gyro[0]
        self.imu_msg.angular_velocity.y = self.gyro[1]
        self.imu_msg.angular_velocity.z = self.gyro[2]

    def angular_variance(self):
        self.rollingArray[3][0] = self.gyro[0]            
        self.rollingArray[4][0] = self.gyro[1]            
        self.rollingArray[5][0] = self.gyro[2] 
        self.imu_msg.angular_velocity_covariance = [
            self.varianceArray[3], 0,      0,
            0,     self.varianceArray[4], 0,
            0,      0,      self.varianceArray[5]
        ]   

    def imu_msg_linear(self): 
        # Linear acceleration (m/s²)
        self.imu_msg.linear_acceleration.x = self.accel[0]
        self.imu_msg.linear_acceleration.y = self.accel[1]
        self.imu_msg.linear_acceleration.z = self.accel[2]
        self.imu_msg.linear_acceleration_covariance = [
            self.varianceArray[6], 0,       0,
            0,       self.varianceArray[7], 0,
            0,       0,       self.varianceArray[8]
        ]

    def linear_variance(self):
        self.rollingArray[6][0] = self.accel[0]            
        self.rollingArray[7][0] = self.accel[1]            
        self.rollingArray[8][0] = self.accel[2]

    def calculate_variance(self):
        self.varianceArray = np.var(self.rollingArray,axis=1)
        self.orientation_variance(self)
        self.angular_variance(self)
        self.linear_variance(self)

    def publish_imu_data(self):
        try:
            self.get_imu_data(self)

            self.initialize_imu_msg(self)

            self.imu_msg_orientation(self)
            self.log_orientation(self)
            self.imu_msg_angular(self)
            self.imu_msg_linear(self)

            self.calculate_variance(self)

            # Publish IMU message
            self.imu_pub.publish(self.imu_msg)

            self.publish_mag(self)

        except Exception as e:
            self.get_logger().error(f"Error reading BNO085 data: {e}")
            # Attempt to reinitialize the BNO085 sensor
            try:
                self.get_logger().info("Attempting to reinitialize BNO085 sensor...")
                self.initialize_bno085()
            except Exception as reinit_e:
                self.get_logger().error(f"Failed to reinitialize BNO085: {reinit_e}")
                self.get_logger().error("Critical failure: Unable to recover BNO085. Shutting down node.")
                rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = BNO085Node()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()