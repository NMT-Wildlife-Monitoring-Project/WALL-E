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
import bno085

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
        

        self.initialize_bno085()
        self.get_logger().info(f'Using I2C address: 0x{i2c_address:02X}')
        
        # Create timer for publishing
        self.timer = self.create_timer(0.01, self.publish_imu_data)  # 100Hz
        self.bno = bno085(i2c_address)
        self.bno.calibrate()

        self.get_logger().info('BNO085 Node initialized')
    

    def publish_mag(self):
        # Publish magnetometer data if available
        if self.bno.mag is not None:
            mag_msg = MagneticField()
            mag_msg.header = Header()
            mag_msg.header.stamp = self.get_clock().now().to_msg()
            mag_msg.header.frame_id = 'imu_link'

            # Magnetic field (Tesla)
            mag_msg.magnetic_field.x = self.bno.mag[0] * 1e-6  # Convert µT to T
            mag_msg.magnetic_field.y = self.bno.mag[1] * 1e-6
            mag_msg.magnetic_field.z = self.bno.mag[2] * 1e-6

            # Covariance (set to unknown)
            mag_msg.magnetic_field_covariance = self.bno.mag_covariance

            self.mag_pub.publish(mag_msg)

    def imu_msg_orientation(self): 
        # Orientation (quaternion)
        self.imu_msg.orientation.x = self.bno,quat[0]
        self.imu_msg.orientation.y = self.bno.quat[1]
        self.imu_msg.orientation.z = self.bno.quat[2]
        self.imu_msg.orientation.w = self.bno.quat[3]

    def imu_msg_angular(self): 
        # Angular velocity (rad/s)
        self.imu_msg.angular_velocity.x = self.bno.gyro[0]
        self.imu_msg.angular_velocity.y = self.bno.gyro[1]
        self.imu_msg.angular_velocity.z = self.bno.gyro[2]

    def imu_msg_linear(self): 
        # Linear acceleration (m/s²)
        self.imu_msg.linear_acceleration.x = self.bno.accel[0]
        self.imu_msg.linear_acceleration.y = self.bno.accel[1]
        self.imu_msg.linear_acceleration.z = self.bno.accel[2]

    def imu_msg_variance(self):
        self.imu_msg.orientation_covariance = self.bno.rpy_covariance
        self.imu_msg.angular_velocity_covariance = self.bno.gyro_covariance
        self.imu_msg.linear_acceleration_covariance = self.bno.accel_covariance

    def initialize_imu_msg(self):
        # Create IMU message
        self.imu_msg = Imu()
        # Header
        self.imu_msg.header = Header()
        self.imu_msg.header.stamp = self.get_clock().now().to_msg()
        self.imu_msg.header.frame_id = self.frame_id
        #TODO add bias
        self.imu_msg_orientation(self)
        self.imu_msg_angular(self)
        self.imu_msg_linear(self)
        self.imu_msg_variance(self)


    def publish_imu_data(self):
        try:
            self.bno.get_data()
        
            self.initialize_imu_msg(self)
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