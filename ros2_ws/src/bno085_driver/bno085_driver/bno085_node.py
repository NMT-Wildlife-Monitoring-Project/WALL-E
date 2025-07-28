#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Header
import busio
from adafruit_bno08x.i2c import BNO08X_I2C
import time
import os

from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR,
)

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
        
        self.bno = BNO08X_I2C(self.i2c, address=i2c_address)
        
        self.get_logger().info(f'Using I2C address: 0x{i2c_address:02X}')
        
        # Enable required reports with retry logic
        features = [
            BNO_REPORT_ACCELEROMETER,
            BNO_REPORT_GYROSCOPE,
            BNO_REPORT_MAGNETOMETER,
            BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR
        ]
        feature_names = [
            "ACCELEROMETER",
            "GYROSCOPE",
            "MAGNETOMETER",
            "ROTATION_VECTOR"
        ]
        for feature, name in zip(features, feature_names):
            for attempt in range(1, 4):
                try:
                    self.bno.enable_feature(feature)
                    break
                except Exception as e:
                    self.get_logger().warning(f"Attempt {attempt} to enable {name} failed: {e}")
                    time.sleep(0.5)
            else:
                self.get_logger().error(f"Failed to enable {name} after 3 attempts. Exiting.")
                raise RuntimeError(f"Failed to enable {name} after 3 attempts.")
        
        # Create timer for publishing
        self.timer = self.create_timer(0.01, self.publish_imu_data)  # 100Hz
        
        self.get_logger().info('BNO085 Node initialized')
    
    def publish_imu_data(self):
        try:
            # Get sensor data
            accel = self.bno.acceleration
            gyro = self.bno.gyro
            quat = self.bno.quaternion
            mag = self.bno.magnetic

            # Check if data is valid
            if accel is None or gyro is None or quat is None:
                self.get_logger().warning("Received invalid data from BNO085 sensor.")
                return

            # Create IMU message
            imu_msg = Imu()

            # Header
            imu_msg.header = Header()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = self.frame_id

            # Orientation (quaternion)
            imu_msg.orientation.x = quat[0]
            imu_msg.orientation.y = quat[1]
            imu_msg.orientation.z = quat[2]
            imu_msg.orientation.w = quat[3]

            # Angular velocity (rad/s)
            imu_msg.angular_velocity.x = gyro[0]
            imu_msg.angular_velocity.y = gyro[1]
            imu_msg.angular_velocity.z = gyro[2]

            # Linear acceleration (m/s²)
            imu_msg.linear_acceleration.x = accel[0]
            imu_msg.linear_acceleration.y = accel[1]
            imu_msg.linear_acceleration.z = accel[2]

            # Covariance matrices (set to unknown)
            imu_msg.orientation_covariance = [-1.0] * 9
            imu_msg.angular_velocity_covariance = [-1.0] * 9
            imu_msg.linear_acceleration_covariance = [-1.0] * 9

            # Publish IMU message
            self.imu_pub.publish(imu_msg)

            # Publish magnetometer data if available
            if mag is not None:
                mag_msg = MagneticField()
                mag_msg.header = Header()
                mag_msg.header.stamp = self.get_clock().now().to_msg()
                mag_msg.header.frame_id = 'imu_link'

                # Magnetic field (Tesla)
                mag_msg.magnetic_field.x = mag[0] * 1e-6  # Convert µT to T
                mag_msg.magnetic_field.y = mag[1] * 1e-6
                mag_msg.magnetic_field.z = mag[2] * 1e-6

                # Covariance (set to unknown)
                mag_msg.magnetic_field_covariance = [-1.0] * 9

                self.mag_pub.publish(mag_msg)

        except Exception as e:
            self.get_logger().error(f"Error reading BNO085 data: {e}")

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