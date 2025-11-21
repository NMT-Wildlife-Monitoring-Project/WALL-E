import serial.serialutil
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import math
from .roboclaw_3 import Roboclaw
from robot_messages.msg import RoboclawStatus
import serial
import os, glob, time, threading
import errno

class RoboclawNode(Node):
    ERROR_CODES = [
        (0x000001, "E-Stop"),
        (0x000002, "Temperature Error"),
        (0x000004, "Temperature 2 Error"),
        (0x000008, "Main Voltage High Error"),
        (0x000010, "Logic Voltage High Error"),
        (0x000020, "Logic Voltage Low Error"),
        (0x000040, "M1 Driver Fault Error"),
        (0x000080, "M2 Driver Fault Error"),
        (0x000100, "M1 Speed Error"),
        (0x000200, "M2 Speed Error"),
        (0x000400, "M1 Position Error"),
        (0x000800, "M2 Position Error"),
        (0x001000, "M1 Current Error"),
        (0x002000, "M2 Current Error"),
        (0x010000, "M1 Over Current Warning"),
        (0x020000, "M2 Over Current Warning"),
        (0x040000, "Main Voltage High Warning"),
        (0x080000, "Main Voltage Low Warning"),
        (0x100000, "Temperature Warning"),
        (0x200000, "Temperature 2 Warning"),
        (0x400000, "S4 Signal Triggered"),
        (0x800000, "S5 Signal Triggered"),
        (0x01000000, "Speed Error Limit Warning"),
        (0x02000000, "Position Error Limit Warning"),
    ]

    def __init__(self):
        super().__init__('roboclaw_node')
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/roboclaw')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('address', 128)
        self.declare_parameter('qppr', 6533)  # Quadrature pulses per revolution
        self.declare_parameter('accel', 1.5)    # m/s^2
        self.declare_parameter('max_speed', 1.0) # m/s
        self.declare_parameter('max_speed_qpps', 10560)  # Max speed in quadrature pulses per second, -1 means use max_speed
        self.declare_parameter('accel_qpps', -1)      # Max accel in quadrature pulses per second^2, -1 means use accel
        self.declare_parameter('wheel_separation', 0.24) # meters
        self.declare_parameter('wheel_diameter', 0.105)    # meters
        self.declare_parameter('m1_reverse', True)  # Reverse motor 1 direction
        self.declare_parameter('m2_reverse', False)  # Reverse motor 2 direction
        self.declare_parameter('odom_publish_rate', 20)  # Hz
        self.declare_parameter('status_publish_rate', 5)
        self.declare_parameter('status_topic', 'roboclaw_status')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('cmd_vel_timeout', 1.0)  # seconds
        self.declare_parameter('publish_tf', False)

        # Get parameters
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.address = self.get_parameter('address').get_parameter_value().integer_value
        self.qppr = self.get_parameter('qppr').get_parameter_value().integer_value
        self.accel = self.get_parameter('accel').get_parameter_value().double_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.wheel_separation = self.get_parameter('wheel_separation').get_parameter_value().double_value
        self.wheel_diameter = self.get_parameter('wheel_diameter').get_parameter_value().double_value
        self.m1_reverse = self.get_parameter('m1_reverse').get_parameter_value().bool_value
        self.m2_reverse = self.get_parameter('m2_reverse').get_parameter_value().bool_value
        self.odom_publish_rate = self.get_parameter('odom_publish_rate').get_parameter_value().integer_value
        self.status_publish_rate = self.get_parameter('status_publish_rate').get_parameter_value().integer_value
        self.status_topic = self.get_parameter('status_topic').get_parameter_value().string_value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.odom_frame_id = self.get_parameter('odom_frame_id').get_parameter_value().string_value
        self.base_frame_id = self.get_parameter('base_frame_id').get_parameter_value().string_value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').get_parameter_value().double_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value
        self.max_speed_qpps_param = self.get_parameter('max_speed_qpps').get_parameter_value().integer_value
        self.accel_qpps_param = self.get_parameter('accel_qpps').get_parameter_value().integer_value

        # Calculate max speed qpps
        if (self.max_speed_qpps_param is not None and self.max_speed_qpps_param >= 0):
            self.max_speed_qpps = self.max_speed_qpps_param
        else:
            self.max_speed_qpps = self.meters_to_pulses(self.max_speed) if self.max_speed is not None and self.max_speed >= 0 else 0

        # Calculate accel qpps
        if (self.accel_qpps_param is not None and self.accel_qpps_param >= 0):
            self.accel_qpps = self.accel_qpps_param
        else:
            self.accel_qpps = self.meters_to_pulses(self.accel) if self.accel is not None and self.accel >= 0 else 0

        self._port_lock = threading.Lock()
        self._reconnecting = False
        self.connected = False
        self._last_cmd = (0, 0)
        self._reconnect_backoff = 0.2

        # Roboclaw setup
        self.roboclaw = Roboclaw(self.serial_port, self.baudrate)
        self.connect()

        # ROS2 interfaces
        qos = QoSProfile(depth=10)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, qos)
        cmd_vel_qos = QoSProfile(depth=1)
        self.cmd_vel_sub = self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_vel_callback, cmd_vel_qos)
        self.status_pub = self.create_publisher(RoboclawStatus, self.status_topic, qos)

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_enc_left = None
        self.last_enc_right = None
        self.last_time = self.get_clock().now()

        # Track last cmd_vel time
        self.last_cmd_vel_time = self.get_clock().now()

        # Timers
        self.create_timer(1/self.odom_publish_rate, self.update_odom)
        self.create_timer(1/self.status_publish_rate, self.publish_status)
        self.create_timer(0.1, self.check_cmd_vel_timeout)
        self.create_timer(0.1, self._heartbeat)
    
    def _dev_exists(self):
        return os.path.exists(self.serial_port)

    def connect(self):
        """Try to open the port. Never raise. Set self.connected accordingly."""
        with self._port_lock:
            try:
                if not self._dev_exists():
                    self.connected = False
                    return
                if self.roboclaw.Open():
                    time.sleep(0.05)
                    try:
                        self.roboclaw._port.reset_input_buffer()
                        self.roboclaw._port.reset_output_buffer()
                    except Exception:
                        pass
                    self.connected = True
                    self._reconnecting = False
                    self._reconnect_backoff = 0.2
                    self.get_logger().info(f'Connected to Roboclaw on {self.serial_port} at {self.baudrate} baud')
                else:
                    self.connected = False
            except Exception as e:
                self.connected = False
                self.get_logger().warn(f'connect() failed: {e}')
        if not self.connected and not self._reconnecting:
            self._reconnecting = True
            self.create_timer(self._reconnect_backoff, self._reconnect_tick)

    def _reconnect_tick(self):
        if self.connected:
            return
        self._reconnect_backoff = min(self._reconnect_backoff * 1.6, 3.0)
        self.connect()

    def _handle_serial_exception(self, where: str, exc: Exception):
        self.get_logger().warning(f"Serial error in {where}: {exc}")
        with self._port_lock:
            try:
                self.roboclaw.Close()
            except Exception:
                pass
            self.connected = False
        if not self._reconnecting:
            self._reconnecting = True
            self.create_timer(self._reconnect_backoff, self._reconnect_tick)

    def meters_to_pulses(self, v):
        """Convert linear velocity in m/s to quadrature pulses per second."""
        return int(v * self.qppr / (math.pi * self.wheel_diameter))
    
    def pulses_to_meters(self, pulses):
        """Convert quadrature pulses to linear distance in meters."""
        return pulses * (math.pi * self.wheel_diameter) / self.qppr

    def cmd_vel_callback(self, msg):
        v = msg.linear.x
        omega = msg.angular.z
        v_left = v - omega * self.wheel_separation / 2.0
        v_right = v + omega * self.wheel_separation / 2.0
        qpps_left = self.meters_to_pulses(v_left)
        qpps_right = self.meters_to_pulses(v_right)
        qpps_left = max(-self.max_speed_qpps, min(self.max_speed_qpps, qpps_left))
        qpps_right = max(-self.max_speed_qpps, min(self.max_speed_qpps, qpps_right))
        if self.m1_reverse:
            qpps_left = -qpps_left
        if self.m2_reverse:
            qpps_right = -qpps_right
        self._last_cmd = (qpps_left, qpps_right)
        if not self.connected:
            return
        try:
            with self._port_lock:
                if not self.connected: return
                self.roboclaw.SpeedAccelM1M2(self.address, self.accel_qpps, qpps_left, qpps_right)
            self.last_cmd_vel_time = self.get_clock().now()
        except (serial.serialutil.SerialException, OSError) as e:
            self._handle_serial_exception("cmd_vel_callback", e)

    def check_cmd_vel_timeout(self):
        now = self.get_clock().now()
        dt = (now - self.last_cmd_vel_time).nanoseconds / 1e9
        if dt > self.cmd_vel_timeout:
            # Stop motors if timeout exceeded
            self.stop_motors()
    
    def stop_motors(self):
        if not self.connected:
            return
        try:
            with self._port_lock:
                if not self.connected: return
                self.roboclaw.SpeedAccelM1M2(self.address, self.accel_qpps, 0, 0)
        except (serial.serialutil.SerialException, OSError) as e:
            self.get_logger().warning(f"Serial error in stop_motors: {e}")

    def __del__(self):
        self.stop_motors()

    def _heartbeat(self):
        if not self.connected:
            return
        try:
            ql, qr = self._last_cmd
            with self._port_lock:
                if not self.connected: return
                self.roboclaw.SpeedAccelM1M2(self.address, self.accel_qpps, ql, qr)
        except (serial.serialutil.SerialException, OSError) as e:
            self._handle_serial_exception("heartbeat", e)

    def update_odom(self):
        if not self.connected:
            return
        try:
            with self._port_lock:
                if not self.connected: return
                enc_left = self.roboclaw.ReadEncM1(self.address)
                enc_right = self.roboclaw.ReadEncM2(self.address)
            if not (enc_left[0] and enc_right[0]):
                self.get_logger().warn('Failed to read encoders')
                return
            enc_left = enc_left[1]
            enc_right = enc_right[1]
            now = self.get_clock().now()
            dt = (now - self.last_time).nanoseconds / 1e9
            if self.last_enc_left is not None and self.last_enc_right is not None and dt > 0:
                # Account for motor direction reversal: encoder sign may be inverted
                left_sign = -1 if self.m1_reverse else 1
                right_sign = -1 if self.m2_reverse else 1
                d_left = self.pulses_to_meters(left_sign * (enc_left - self.last_enc_left))
                d_right = self.pulses_to_meters(right_sign * (enc_right - self.last_enc_right))
                d = (d_left + d_right) / 2.0
                dth = (d_right - d_left) / self.wheel_separation
                self.x += d * math.cos(self.th + dth / 2.0)
                self.y += d * math.sin(self.th + dth / 2.0)
                self.th += dth
                vx = d / dt
                vth = dth / dt
                odom = Odometry()
                odom.header.stamp = now.to_msg()
                odom.header.frame_id = self.odom_frame_id
                odom.child_frame_id = self.base_frame_id
                odom.pose.pose.position.x = self.x
                odom.pose.pose.position.y = self.y
                odom.pose.pose.position.z = 0.0
                q = quaternion_from_euler(0, 0, self.th)
                odom.pose.pose.orientation.x = q[0]
                odom.pose.pose.orientation.y = q[1]
                odom.pose.pose.orientation.z = q[2]
                odom.pose.pose.orientation.w = q[3]
                odom.twist.twist.linear.x = vx
                odom.twist.twist.linear.y = 0.0
                odom.twist.twist.angular.z = vth

                # Set pose covariance (example values, adjust as needed)
                odom.pose.covariance = [
                    1.0e-4,  0,       0,       0,       0,       0,
                    0,       1.0e-4,  0,       0,       0,       0,
                    0,       0,       1.0e-4,   0,       0,       0,
                    0,       0,       0,       1.0e-4,   0,       0,
                    0,       0,       0,       0,       1.0e-4,   0,
                    0,       0,       0,       0,       0,       1.0e-3
                ]
                # Set twist covariance (example values, adjust as needed)
                odom.twist.covariance = [
                    1.0e-4, 0,       0,       0,       0,       0,
                    0,       1.0e-4,   0,       0,       0,       0,
                    0,       0,       1.0e-4,   0,       0,       0,
                    0,       0,       0,       1.0e-4,   0,       0,
                    0,       0,       0,       0,       1.0e-4,   0,
                    0,       0,       0,       0,       0,       1e-3
                ]

                self.odom_pub.publish(odom)
            self.last_enc_left = enc_left
            self.last_enc_right = enc_right
            self.last_time = now
        except (serial.serialutil.SerialException, OSError) as e:
            self._handle_serial_exception("update_odom", e)

    def error_code_to_string(self, code):
        if code == 0:
            return "Normal 0x000000"
        msgs = []
        for val, name in self.ERROR_CODES:
            if code & val:
                msgs.append(name + f" 0x{val:06x}")
        if not msgs:
            return f"Unknown error 0x{code:06x}"
        return "; ".join(msgs)

    def publish_status(self):
        if not self.connected:
            return
        try:
            with self._port_lock:
                if not self.connected: return
                status = RoboclawStatus()
                m1_qpps_raw = self.roboclaw.ReadSpeedM1(self.address)
                m1_enc_raw = self.roboclaw.ReadEncM1(self.address)
                m1_current_raw = self.roboclaw.ReadCurrents(self.address)
                m1_qpps = int(m1_qpps_raw[1]) if m1_qpps_raw[0] else 0
                m1_speed = float(self.pulses_to_meters(m1_qpps))
                m1_motor_current = float(m1_current_raw[1])/100.0 if m1_current_raw[0] and len(m1_current_raw) > 1 else 0.0
                m1_encoder_value = int(m1_enc_raw[1]) if m1_enc_raw[0] else 0
                m1_encoder_status = int(m1_enc_raw[2]) if m1_enc_raw[0] and len(m1_enc_raw) > 2 else 0

                m2_qpps_raw = self.roboclaw.ReadSpeedM2(self.address)
                m2_enc_raw = self.roboclaw.ReadEncM2(self.address)
                m2_qpps = int(m2_qpps_raw[1]) if m2_qpps_raw[0] else 0
                m2_speed = float(self.pulses_to_meters(m2_qpps))
                m2_motor_current = float(m1_current_raw[2])/100.0 if m1_current_raw[0] and len(m1_current_raw) > 2 else 0.0
                m2_encoder_value = int(m2_enc_raw[1]) if m2_enc_raw[0] else 0
                m2_encoder_status = int(m2_enc_raw[2]) if m2_enc_raw[0] and len(m2_enc_raw) > 2 else 0

                m1_encoder_value = max(0, m1_encoder_value)
                m1_encoder_status = max(0, m1_encoder_status)
                m2_encoder_value = max(0, m2_encoder_value)
                m2_encoder_status = max(0, m2_encoder_status)
                qppr = max(0, self.qppr)
                max_speed = max(0, int(self.max_speed))
                max_speed_qpps = max(0, self.max_speed_qpps)
                accel = max(0, int(self.accel))
                accel_qpps = max(0, self.accel_qpps)

                status.qppr = qppr
                status.max_speed = max_speed
                status.max_speed_qpps = max_speed_qpps
                status.accel = accel
                status.accel_qpps = accel_qpps
                status.m1_qpps = m1_qpps
                status.m1_speed = m1_speed
                status.m1_motor_current = m1_motor_current
                status.m1_encoder_value = m1_encoder_value
                status.m1_encoder_status = m1_encoder_status
                status.m2_qpps = m2_qpps
                status.m2_speed = m2_speed
                status.m2_motor_current = m2_motor_current
                status.m2_encoder_value = m2_encoder_value
                status.m2_encoder_status = m2_encoder_status

                main_batt = self.roboclaw.ReadMainBatteryVoltage(self.address)
                logic_batt = self.roboclaw.ReadLogicBatteryVoltage(self.address)
                temp = self.roboclaw.ReadTemp(self.address)
                err = self.roboclaw.ReadError(self.address)
                err_code = err[1] if err[0] else 0

                status.main_battery_voltage = max(0.0, float(main_batt[1])/10.0 if main_batt[0] else 0.0)
                status.logic_battery_voltage = max(0.0, float(logic_batt[1])/10.0 if logic_batt[0] else 0.0)
                status.temperature = float(temp[1])/10.0 if temp[0] else 0.0
                status.error_string = self.error_code_to_string(err_code)
            self.status_pub.publish(status)
        except (serial.serialutil.SerialException, OSError) as e:
            self._handle_serial_exception("publish_status", e)

def main(args=None):
    rclpy.init(args=args)
    node = RoboclawNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.stop_motors()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
