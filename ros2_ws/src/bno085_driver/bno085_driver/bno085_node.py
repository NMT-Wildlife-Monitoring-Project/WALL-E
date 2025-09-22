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
        self.timer = self.create_timer(0.05, self.publish_imu_data)  # 100Hz

        self.rollingArray = np.zeros((9,10))

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
        self.get_logger().info("BNO085 initialization complete.")

    def publish_imu_data(self):
        try:
            # Get sensor data
            accel = self.bno.acceleration
            gyro = self.bno.gyro
            quat = self.bno.geomagnetic_quaternion
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

            self.rollingArray = np.roll(self.rollingArray,1,1)

            # Orientation (quaternion)
            imu_msg.orientation.x = quat[0]
            imu_msg.orientation.y = quat[1]
            imu_msg.orientation.z = quat[2]
            imu_msg.orientation.w = quat[3]

            roll, pitch, yaw = euler_from_quaternion([quat[0], quat[1], quat[2], quat[3]])
            self.get_logger().debug(f"Orientation: Roll={roll}, Pitch={pitch}, Yaw={yaw}")
            
            self.rollingArray[0][0] = roll
            self.rollingArray[1][0] = pitch
            self.rollingArray[2][0] = yaw

            # Angular velocity (rad/s)
            imu_msg.angular_velocity.x = gyro[0]
            imu_msg.angular_velocity.y = gyro[1]
            imu_msg.angular_velocity.z = gyro[2]


            self.rollingArray[3][0] = gyro[0]            
            self.rollingArray[4][0] = gyro[1]            
            self.rollingArray[5][0] = gyro[2]            

            # Linear acceleration (m/s²)
            imu_msg.linear_acceleration.x = accel[0]
            imu_msg.linear_acceleration.y = accel[1]
            imu_msg.linear_acceleration.z = accel[2]

            self.rollingArray[6][0] = accel[0]            
            self.rollingArray[7][0] = accel[1]            
            self.rollingArray[8][0] = accel[2]            

            varianceArray = np.var(self.rollingArray,axis=1)


            # Covariance matrices
            imu_msg.orientation_covariance = [
                varianceArray[0], 0,       0,
                0,       varianceArray[1], 0,
                0,       0,       varianceArray[2]
            ]
            imu_msg.angular_velocity_covariance = [
                varianceArray[3], 0,      0,
                0,      varianceArray[4], 0,
                0,      0,      varianceArray[5]
            ]
            imu_msg.linear_acceleration_covariance = [
                varianceArray[6], 0,       0,
                0,       varianceArray[7], 0,
                0,       0,       varianceArray[8]
            ]

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