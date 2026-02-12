#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
import numpy as np
import json
import os
import time
import threading
from collections import deque
from bno085_driver.bno085 import BNO085


class BNO085Node(Node):
    def __init__(self):
        super().__init__('bno085_node')
        # Declare parameter for I2C address
        self.declare_parameter('i2c_address', 0x4B)  # Default BNO085 address
        self.i2c_address = self.get_parameter('i2c_address').value

        # Declare parameter for I2C bus number
        self.declare_parameter('i2c_bus', 7)  # Default to bus 1 (Jetson/Pi standard)
        self.i2c_bus = self.get_parameter('i2c_bus').value

        # Declare parameter for frame ID
        self.declare_parameter('frame_id', 'imu_link')  # Default frame ID
        self.frame_id = self.get_parameter('frame_id').value
        
        # Publishers
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_teleop', 10)

        # Calibration state
        self.calibrating = False
        self.mag_samples = deque(maxlen=10000)  # Store up to 10000 samples
        self.cal_file = os.path.expanduser('~/.ros/bno085_mag_calibration.json')

        self.get_logger().info(f'Using I2C bus: {self.i2c_bus}, address: 0x{self.i2c_address:02X}')

        # Create timer for publishing
        self.timer = self.create_timer(0.01, self.publish)  # 100Hz

        # Create BNO085 and calibrate
        self.bno = None
        self.initialize()

        # Load saved magnetometer calibration if available
        self.load_mag_calibration()

        # Calibration action server and service
        # (Import done lazily to avoid dependency issues during build)
        try:
            from bno085_driver.action import CalibrateMagnetometer
            self._cal_action_server = ActionServer(
                self,
                CalibrateMagnetometer,
                'calibrate_magnetometer',
                self.execute_calibration
            )
            self.get_logger().info('Calibration action server ready')
        except ImportError as e:
            self.get_logger().warn(f'Calibration action not available: {e}')

        # Simple service wrapper for calibration
        self.cal_service = self.create_service(
            Trigger,
            'calibrate_mag',
            self.service_calibrate
        )

        self.get_logger().info('BNO085 Node initialized')
    
    def initialize(self):
        if self.bno is not None:
            del self.bno
        self.bno = BNO085(self.i2c_address, self.i2c_bus)
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

    def load_mag_calibration(self):
        """Load magnetometer calibration from file on startup."""
        if os.path.exists(self.cal_file):
            try:
                with open(self.cal_file, 'r') as f:
                    cal_data = json.load(f)
                offset = np.array(cal_data['hard_iron_offset'])
                self.bno.set_mag_offset(offset)
                self.get_logger().info(f'Loaded mag calibration: {offset}')
            except Exception as e:
                self.get_logger().warn(f'Failed to load mag calibration: {e}')

    def save_mag_calibration(self, offset, quality):
        """Save magnetometer calibration to file."""
        os.makedirs(os.path.dirname(self.cal_file), exist_ok=True)
        with open(self.cal_file, 'w') as f:
            json.dump({
                'hard_iron_offset': offset.tolist(),
                'calibration_quality': float(quality),
                'timestamp': time.time()
            }, f, indent=2)
        self.get_logger().info(f'Saved calibration to {self.cal_file}')

    def service_calibrate(self, request, response):
        """Simple service wrapper for calibration (non-blocking)."""
        self.get_logger().info('Calibration service called - starting in background thread')

        # Default calibration parameters
        duration = 20.0  # 20 seconds
        rotation_speed = 0.3  # rad/s

        # Run calibration in a background thread to avoid blocking the node
        cal_thread = threading.Thread(
            target=self.run_calibration,
            args=(duration, rotation_speed),
            daemon=False
        )
        cal_thread.start()

        response.success = True
        response.message = f'Calibration started in background (duration: {duration}s). Watch /calibrate_mag_result for updates.'
        return response

    def execute_calibration(self, goal_handle):
        """Action server callback for magnetometer calibration."""
        from bno085_driver.action import CalibrateMagnetometer

        self.get_logger().info('Executing magnetometer calibration action...')

        duration = goal_handle.request.duration
        rotation_speed = goal_handle.request.rotation_speed
        if rotation_speed == 0.0:
            rotation_speed = 0.3  # Default

        # Clear samples
        self.mag_samples.clear()
        self.calibrating = True

        # Start rotation
        self.get_logger().info(f'Rotating for {duration}s at {rotation_speed} rad/s')
        start_time = time.time()

        twist = Twist()
        twist.angular.z = rotation_speed

        # Collect samples while rotating
        rate = self.create_rate(20)  # 20 Hz command rate
        mag_min = np.full(3, np.inf)
        mag_max = np.full(3, -np.inf)

        while time.time() - start_time < duration:
            # Publish rotation command
            self.cmd_vel_pub.publish(twist)

            # Collect magnetometer samples
            if self.bno.mag is not None and not np.any(np.isnan(self.bno.mag)):
                self.mag_samples.append(self.bno.mag.copy())
                mag_min = np.minimum(mag_min, self.bno.mag)
                mag_max = np.maximum(mag_max, self.bno.mag)

            # Publish feedback
            feedback = CalibrateMagnetometer.Feedback()
            feedback.progress = (time.time() - start_time) / duration
            feedback.samples_collected = len(self.mag_samples)
            feedback.mag_min = mag_min.tolist()
            feedback.mag_max = mag_max.tolist()
            goal_handle.publish_feedback(feedback)

            rate.sleep()

        # Stop rotation
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.calibrating = False

        # Compute calibration
        result = CalibrateMagnetometer.Result()

        if len(self.mag_samples) < 100:
            goal_handle.abort()
            result.success = False
            result.message = f"Insufficient samples: {len(self.mag_samples)}"
            self.get_logger().error(result.message)
            return result

        # Calculate hard iron offset (simple min/max method)
        mag_array = np.array(list(self.mag_samples))
        mag_min = np.min(mag_array, axis=0)
        mag_max = np.max(mag_array, axis=0)
        hard_iron_offset = (mag_max + mag_min) / 2.0

        # Quality metric: how well we covered the full range
        mag_range = mag_max - mag_min
        expected_range = 60.0  # Expected ~60 µT range for full rotation
        quality = np.min(mag_range) / expected_range
        quality = min(1.0, max(0.0, quality))

        # Apply calibration
        self.bno.set_mag_offset(hard_iron_offset)
        self.save_mag_calibration(hard_iron_offset, quality)

        goal_handle.succeed()
        result.success = True
        result.hard_iron_offset = hard_iron_offset.tolist()
        result.calibration_quality = float(quality)
        result.message = f"Calibration complete! Quality: {quality:.1%}, Offset: {hard_iron_offset}"

        self.get_logger().info(result.message)
        return result

    def run_calibration(self, duration, rotation_speed):
        """Run calibration in background (can be called from thread)."""
        try:
            # Clear samples
            self.mag_samples.clear()
            self.calibrating = True

            # Start rotation
            self.get_logger().info(f'Rotating for {duration}s at {rotation_speed} rad/s')
            start_time = time.time()

            twist = Twist()
            twist.angular.z = rotation_speed

            # Collect samples
            rate = self.create_rate(20)
            while time.time() - start_time < duration:
                self.cmd_vel_pub.publish(twist)
                if self.bno.mag is not None and not np.any(np.isnan(self.bno.mag)):
                    self.mag_samples.append(self.bno.mag.copy())
                rate.sleep()

            # Stop
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            self.calibrating = False

            # Compute
            if len(self.mag_samples) < 100:
                self.get_logger().error(f'Insufficient samples: {len(self.mag_samples)}')
                return

            mag_array = np.array(list(self.mag_samples))
            mag_min = np.min(mag_array, axis=0)
            mag_max = np.max(mag_array, axis=0)
            hard_iron_offset = (mag_max + mag_min) / 2.0

            mag_range = mag_max - mag_min
            quality = np.min(mag_range) / 60.0
            quality = min(1.0, max(0.0, quality))

            # Apply and save
            self.bno.set_mag_offset(hard_iron_offset)
            self.save_mag_calibration(hard_iron_offset, quality)

            msg = f'Calibration complete! Quality: {quality:.1%}, Offset: {hard_iron_offset}'
            self.get_logger().info(msg)

        except Exception as e:
            self.get_logger().error(f'Calibration failed: {e}')
            self.calibrating = False
            # Ensure robot is stopped
            twist = Twist()
            self.cmd_vel_pub.publish(twist)

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