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

try:
    import Jetson.GPIO as GPIO
except Exception:
    GPIO = None


class BNO085Node(Node):
    def __init__(self):
        super().__init__('bno085_node')
        # Declare parameter for I2C address
        self.declare_parameter('i2c_address', 0x4B)  # Default BNO085 address
        self.i2c_address = self.get_parameter('i2c_address').value

        # Declare parameter for I2C bus number
        self.declare_parameter('i2c_bus', 7)  # Default to bus 7 on Jetson Orin Nano
        self.i2c_bus = self.get_parameter('i2c_bus').value

        # Interrupt-driven publishing
        self.declare_parameter('use_interrupt', True)
        self.declare_parameter('int_pin', 15)
        self.declare_parameter('int_pin_mode', 'BOARD')
        self.declare_parameter('int_edge', 'FALLING')
        self.declare_parameter('int_bouncetime_ms', 0)
        self.use_interrupt = self.get_parameter('use_interrupt').value
        self.int_pin = self.get_parameter('int_pin').value
        self.int_pin_mode = self.get_parameter('int_pin_mode').value
        self.int_edge = self.get_parameter('int_edge').value
        self.int_bouncetime_ms = self.get_parameter('int_bouncetime_ms').value

        # Publish rate control (Hz)
        self.declare_parameter('publish_rate_hz', 50.0)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.publish_period = 1.0 / self.publish_rate_hz if self.publish_rate_hz > 0.0 else 0.0
        self._last_publish_time = 0.0

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
        self.timer = None

        # Interrupt state
        self._shutdown = False
        self._int_event = threading.Event()
        self._int_thread = None

        # Create BNO085 and calibrate
        self.bno = None
        self.initialization_failed = False
        self.warning_timer = None
        if not self.initialize():
            self.initialization_failed = True
            self.get_logger().error('=' * 60)
            self.get_logger().error('BNO085 IMU NOT DETECTED ON I2C BUS')
            self.get_logger().error(f'Expected: Bus {self.i2c_bus}, Address 0x{self.i2c_address:02X}')
            self.get_logger().error('Check: Power, SDA/SCL wiring, PS0/PS1 pins')
            self.get_logger().error('Node will remain active but publish NO DATA')
            self.get_logger().error('=' * 60)
            # Create a timer to periodically warn about missing IMU
            self.warning_timer = self.create_timer(30.0, self.warn_no_imu)

        if self.use_interrupt:
            self.setup_interrupt()
        else:
            self.timer = self.create_timer(self.publish_period or 0.02, self.publish_once)

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

    def setup_interrupt(self):
        if GPIO is None:
            self.get_logger().error('Jetson.GPIO not available; cannot use interrupt mode')
            return

        mode = GPIO.BOARD if str(self.int_pin_mode).upper() != 'BCM' else GPIO.BCM
        edge_lookup = {
            'FALLING': GPIO.FALLING,
            'RISING': GPIO.RISING,
            'BOTH': GPIO.BOTH,
        }
        edge = edge_lookup.get(str(self.int_edge).upper(), GPIO.FALLING)

        try:
            GPIO.setmode(mode)
        except RuntimeError as e:
            self.get_logger().warn(f'GPIO mode already set: {e}. Continuing with existing mode.')
        
        GPIO.setup(self.int_pin, GPIO.IN)
        if self.int_bouncetime_ms and int(self.int_bouncetime_ms) > 0:
            GPIO.add_event_detect(self.int_pin, edge, callback=self._int_callback,
                                  bouncetime=int(self.int_bouncetime_ms))
        else:
            GPIO.add_event_detect(self.int_pin, edge, callback=self._int_callback)

        self._int_thread = threading.Thread(target=self._interrupt_loop, daemon=True)
        self._int_thread.start()
        self.get_logger().info(
            f'IMU interrupt enabled on pin {self.int_pin} ({self.int_pin_mode}, {self.int_edge})'
        )

    def _int_callback(self, channel):
        self._int_event.set()

    def _interrupt_loop(self):
        while not self._shutdown and rclpy.ok():
            self._int_event.wait()
            if self._shutdown:
                break
            self._int_event.clear()
            self.publish_once()
    
    def initialize(self):
        """Initialize BNO085 sensor. Returns True on success, False on failure."""
        if self.bno is not None:
            del self.bno
        try:
            self.bno = BNO085(self.i2c_address, self.i2c_bus)
            self.bno.calibrate()
            self.get_logger().info('✓ BNO085 IMU connected and initialized successfully')
            # Stop warning timer if it exists (IMU recovered)
            if self.warning_timer is not None:
                self.warning_timer.cancel()
                self.warning_timer = None
            return True
        except Exception as e:
            self.get_logger().error(f'✗ Failed to initialize BNO085: {e}')
            self.get_logger().error('IMU not detected on I2C bus. Node will not publish data.')
            self.bno = None
            return False
        
    def warn_no_imu(self):
        """Periodic warning when IMU is not connected."""
        if self.bno is None:
            self.get_logger().warn('BNO085 IMU still not connected - no IMU data available')

    
    def publish_once(self):
        # Skip publishing if sensor is not initialized
        if self.bno is None:
            # Silently skip - sensor was never initialized
            return

        if self.publish_period > 0.0:
            now = time.time()
            if (now - self._last_publish_time) < self.publish_period:
                return
            self._last_publish_time = now
        
        try:
            self.bno.update()
        except Exception as e:
            self.get_logger().error('=' * 60)
            self.get_logger().error(f"I2C CONNECTION LOST: {e}")
            self.get_logger().error('BNO085 IMU disconnected during operation')
            self.get_logger().error('Attempting recovery...')
            self.get_logger().error('=' * 60)
            
            # Attempt to reinitialize the BNO085 sensor once
            if self.initialize():
                self.get_logger().info("✓ Successfully recovered BNO085 connection")
                return
            else:
                # Failed to recover - set to None to stop further attempts
                self.get_logger().error('✗ BNO085 recovery failed. Stopping data publishing to prevent false readings.')
                self.bno = None
                # Start periodic warnings if not already running
                if self.warning_timer is None:
                    self.warning_timer = self.create_timer(30.0, self.warn_no_imu)
                return
        
        imu_msg = self.imu_msg()
        self.imu_pub.publish(imu_msg)

        mag_msg = self.mag_msg()
        if mag_msg:
            self.mag_pub.publish(mag_msg)

    def cleanup(self):
        self._shutdown = True
        self._int_event.set()
        if self._int_thread is not None:
            self._int_thread.join(timeout=1.0)
        if GPIO is not None:
            try:
                GPIO.cleanup()
            except Exception:
                pass
        
    
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
        # Only load if sensor is initialized
        if self.bno is None:
            self.get_logger().warn('Skipping mag calibration load - sensor not initialized')
            return
            
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
        # Check if sensor is initialized before starting calibration
        if self.bno is None:
            response.success = False
            response.message = 'BNO085 sensor not initialized. Check I2C connection and try again.'
            self.get_logger().error(response.message)
            return response
        
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
        node.cleanup()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()