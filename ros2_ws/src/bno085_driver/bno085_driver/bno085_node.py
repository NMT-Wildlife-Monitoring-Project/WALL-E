#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Header
import numpy as np
from bno085_driver.bno085 import BNO085


class BNO085Node(Node):
    def __init__(self):
        super().__init__('bno085_node')
        # Declare parameter for I2C address
        self.declare_parameter('i2c_address', 0x4B)  # Default BNO085 address
        self.i2c_address = self.get_parameter('i2c_address').value
        
        # Declare parameter for frame ID
        self.declare_parameter('frame_id', 'imu_link')  # Default frame ID
        self.frame_id = self.get_parameter('frame_id').value
        
        # Publishers
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)
        
        self.get_logger().info(f'Using I2C address: 0x{self.i2c_address:02X}')
        
        # Create timer for publishing
        self.timer = self.create_timer(0.01, self.publish)  # 100Hz

        # Create BNO085 and calibrate
        self.bno = None
        self.initialize()

        self.get_logger().info('BNO085 Node initialized')
    
    def initialize(self):
        if self.bno is not None:
            del self.bno
        self.bno = BNO085(self.i2c_address)
        self.bno.calibrate()
        
    
    def publish(self):
        try:
            self.bno.update()
        except Exception as e:
            self.get_logger().error(f"Error reading BNO085 data: {e}")
            # Attempt to reinitialize the BNO085 sensor
            try:
                self.get_logger().info("Attempting to reinitialize BNO085 sensor...")
                self.initialize()
            except Exception as reinit_e:
                self.get_logger().error(f"Failed to reinitialize BNO085: {reinit_e}")
                self.get_logger().error("Critical failure: Unable to recover BNO085. Shutting down node.")
                if rclpy.ok():
                    rclpy.shutdown()
        
        imu_msg = self.imu_msg()
        self.imu_pub.publish(imu_msg)

        mag_msg = self.mag_msg()
        if mag_msg:
            self.mag_pub.publish(mag_msg)
        
    
    def imu_msg(self):
        imu_msg = Imu()
        # Header
        imu_msg.header = Header()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.frame_id
        
        # Orientation (quaternion)
        imu_msg.orientation.x = self.bno.quat[0]
        imu_msg.orientation.y = self.bno.quat[1]
        imu_msg.orientation.z = self.bno.quat[2]
        imu_msg.orientation.w = self.bno.quat[3]

        # Angular velocity (rad/s)
        imu_msg.angular_velocity.x = self.bno.gyro[0]
        imu_msg.angular_velocity.y = self.bno.gyro[1]
        imu_msg.angular_velocity.z = self.bno.gyro[2]

        # Linear acceleration (m/s²)
        imu_msg.linear_acceleration.x = self.bno.accel[0]
        imu_msg.linear_acceleration.y = self.bno.accel[1]
        imu_msg.linear_acceleration.z = self.bno.accel[2]

        # Covariances
        imu_msg.orientation_covariance = self.bno.rpy_covariance.tolist()
        imu_msg.angular_velocity_covariance = self.bno.gyro_covariance.tolist()
        imu_msg.linear_acceleration_covariance = self.bno.accel_covariance.tolist()

        return imu_msg

    def mag_msg(self):
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

            return mag_msg
        return None

def main(args=None):
    rclpy.init(args=args)
    node = BNO085Node()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()