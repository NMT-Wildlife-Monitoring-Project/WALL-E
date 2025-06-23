#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, Temperature
from std_msgs.msg import Float32
import json
import os

from .gy88_driver import GY88  # Use relative import

class GY88Node(Node):
    def __init__(self):
        super().__init__('gy88_node')
        
        # Declare and get parameters
        self.declare_parameter('calibration_file', 'gy88_calibration.json')
        self.declare_parameter('publish_rate', 50.0)

        calibration_file = self.get_parameter('calibration_file').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        # Load calibration data
        config_path = os.path.join(os.path.dirname(__file__), '..', 'config', calibration_file)
        try:
            with open(config_path, 'r') as f:
                self.calibration_data = json.load(f)
        except FileNotFoundError:
            self.get_logger().error(f"Calibration file not found: {config_path}")
            self.calibration_data = {}

        # Initialize sensor
        try:
            self.gy88 = GY88(calibration=self.calibration_data)
        except Exception as e:
            self.get_logger().error(f"Failed to initialize GY88: {e}")
            return

        # Publishers
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)
        self.temp_pub = self.create_publisher(Temperature, 'temp', 10)
        self.pressure_pub = self.create_publisher(Float32, 'pressure', 10)

        # Timer
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)
        self.get_logger().info(f"GY88 node started at {publish_rate} Hz")

    def timer_callback(self):
        try:
            accel_data = self.gy88.get_accel_data()
            gyro_data = self.gy88.get_gyro_data()
            mag_data = self.gy88.get_mag_data()
            temp_press_data = self.gy88.get_pressure_and_temp()
            
            stamp = self.get_clock().now().to_msg()

            imu_msg = Imu()
            imu_msg.header.stamp = stamp
            imu_msg.header.frame_id = 'imu_link'
            imu_msg.linear_acceleration.x = accel_data['accel_x']
            imu_msg.linear_acceleration.y = accel_data['accel_y']
            imu_msg.linear_acceleration.z = accel_data['accel_z']
            imu_msg.angular_velocity.x = gyro_data['gyro_x']
            imu_msg.angular_velocity.y = gyro_data['gyro_y']
            imu_msg.angular_velocity.z = gyro_data['gyro_z']
            self.imu_pub.publish(imu_msg)

            mag_msg = MagneticField()
            mag_msg.header.stamp = stamp
            mag_msg.header.frame_id = 'imu_link'
            mag_msg.magnetic_field.x = mag_data['mag_x']
            mag_msg.magnetic_field.y = mag_data['mag_y']
            mag_msg.magnetic_field.z = mag_data['mag_z']
            self.mag_pub.publish(mag_msg)

            temp_msg = Temperature()
            temp_msg.header.stamp = stamp
            temp_msg.header.frame_id = 'imu_link'
            temp_msg.temperature = temp_press_data['temperature']
            self.temp_pub.publish(temp_msg)

            pressure_msg = Float32()
            pressure_msg.data = float(temp_press_data['pressure'])
            self.pressure_pub.publish(pressure_msg)

        except Exception as e:
            self.get_logger().error(f"Sensor read error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = GY88Node()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
