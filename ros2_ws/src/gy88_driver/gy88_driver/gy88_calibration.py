#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from .gy88_driver import GY88  # Relative import

class GY88CalibrationNode(Node):
    def __init__(self):
        super().__init__('gy88_calibration_node')
        self.gy88 = GY88()
        self.publisher = self.create_publisher(String, 'calibration_status', 10)
        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.velocity_callback, 10)
        self.timer = self.create_timer(1.0, self.calibration_timer_callback)
        self.calibration_data = []
        self.get_logger().info('GY88 Calibration Node started')

    def velocity_callback(self, msg):
        # Hook for dynamic calibration (optional to implement)
        pass

    def calibration_timer_callback(self):
        try:
            accel_data = self.gy88.read_accelerometer()
            gyro_data = self.gy88.read_gyroscope()
            mag_data = self.gy88.read_magnetometer()

            sample = {
                'timestamp': self.get_clock().now().to_msg(),
                'accelerometer': accel_data,
                'gyroscope': gyro_data,
                'magnetometer': mag_data
            }
            self.calibration_data.append(sample)

            status_msg = String()
            status_msg.data = f"Calibration samples collected: {len(self.calibration_data)}"
            self.publisher.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f"Calibration error: {str(e)}")

    def save_calibration(self):
        with open('/tmp/gy88_calibration.json', 'w') as f:
            json.dump(self.calibration_data, f, indent=2, default=str)
        self.get_logger().info('Calibration data saved')

def main(args=None):
    rclpy.init(args=args)
    node = GY88CalibrationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_calibration()
        node.get_logger().info('Calibration node shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
