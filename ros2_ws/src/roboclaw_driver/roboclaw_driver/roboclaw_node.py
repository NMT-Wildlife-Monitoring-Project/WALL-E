import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import math
from .roboclaw_3 import Roboclaw
from robot_messages.msg import RoboclawStatus

class RoboclawNode(Node):
    def __init__(self):
        super().__init__('roboclaw_node')
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/roboclaw')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('address', 128)
        self.declare_parameter('qppr', 6533)  # Quadrature pulses per revolution
        self.declare_parameter('accel', 1.0)    # m/s^2
        self.declare_parameter('max_speed', 1.0) # m/s
        self.declare_parameter('max_speed_qpps', 10560)  # Max speed in quadrature pulses per second
        self.declare_parameter('wheel_separation', 0.24) # meters
        self.declare_parameter('wheel_diameter', 0.095)    # meters
        self.declare_parameter('odom_publish_rate', 50)  # Hz
        self.declare_parameter('status_publish_rate', 5)
        self.declare_parameter('status_topic', 'roboclaw/status')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('cmd_vel_timeout', 1.0)  # seconds

        # Get parameters
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.address = self.get_parameter('address').get_parameter_value().integer_value
        self.qppr = self.get_parameter('qppr').get_parameter_value().integer_value
        self.accel = self.get_parameter('accel').get_parameter_value().double_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.wheel_separation = self.get_parameter('wheel_separation').get_parameter_value().double_value
        self.wheel_diameter = self.get_parameter('wheel_diameter').get_parameter_value().double_value
        self.odom_publish_rate = self.get_parameter('odom_publish_rate').get_parameter_value().integer_value
        self.status_publish_rate = self.get_parameter('status_publish_rate').get_parameter_value().integer_value
        self.status_topic = self.get_parameter('status_topic').get_parameter_value().string_value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.odom_frame_id = self.get_parameter('odom_frame_id').get_parameter_value().string_value
        self.base_frame_id = self.get_parameter('base_frame_id').get_parameter_value().string_value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').get_parameter_value().double_value

        self.max_speed_qpps = self.meters_to_pulses(self.max_speed)
        self.accel_qpps = self.meters_to_pulses(self.accel)                                                

        # Roboclaw setup
        self.roboclaw = Roboclaw(self.serial_port, self.baudrate)
        if not self.roboclaw.Open():
            self.get_logger().error(f'Failed to open Roboclaw on {self.serial_port}')
        else:
            self.get_logger().info('Roboclaw connected')

        # ROS2 interfaces
        qos = QoSProfile(depth=10)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, qos)
        self.cmd_vel_sub = self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_vel_callback, qos)
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

        # Timer for odometry
        self.create_timer(1/self.odom_publish_rate, self.update_odom)
        # Timer for status
        self.create_timer(1/self.status_publish_rate, self.publish_status)
        # Timer for cmd_vel timeout
        self.create_timer(0.1, self.check_cmd_vel_timeout)

    def meters_to_pulses(self, v):
        """Convert linear velocity in m/s to quadrature pulses per second."""
        return int(v * self.qppr / (math.pi * self.wheel_diameter))
    
    def pulses_to_meters(self, pulses):
        """Convert quadrature pulses to linear distance in meters."""
        return pulses * (math.pi * self.wheel_diameter) / self.qppr

    def cmd_vel_callback(self, msg):
        # Convert Twist to wheel speeds (m/s)
        v = msg.linear.x
        omega = msg.angular.z
        v_left = v - omega * self.wheel_separation / 2.0
        v_right = v + omega * self.wheel_separation / 2.0
        # Convert m/s to qpps
        qpps_left = self.meters_to_pulses(v_left)
        qpps_right = self.meters_to_pulses(v_right)        # Clamp speeds
        qpps_left = max(-self.max_speed_qpps, min(self.max_speed_qpps, qpps_left))
        qpps_right = max(-self.max_speed_qpps, min(self.max_speed_qpps, qpps_right))
        # Send to roboclaw
        self.roboclaw.SpeedAccelM1M2(self.address, self.accel_qpps, qpps_left, qpps_right)
        # Update last command time
        self.last_cmd_vel_time = self.get_clock().now()

    def check_cmd_vel_timeout(self):
        now = self.get_clock().now()
        dt = (now - self.last_cmd_vel_time).nanoseconds / 1e9
        if dt > self.cmd_vel_timeout:
            # Stop motors if timeout exceeded
            self.stop_motors()
    
    def stop_motors(self):
        try:
            self.roboclaw.SpeedAccelM1M2(self.address, self.accel_qpps, 0, 0)
        except Exception as e:
            self.get_logger().warn(f"Exception while stopping motors: {e}")

    def __del__(self):
        self.stop_motors()

    def update_odom(self):
        # Read encoders
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
            d_left = self.pulses_to_meters(enc_left - self.last_enc_left)
            d_right = self.pulses_to_meters(enc_right - self.last_enc_right)
            d = (d_left + d_right) / 2.0
            dth = (d_right - d_left) / self.wheel_separation
            self.x += d * math.cos(self.th + dth / 2.0)
            self.y += d * math.sin(self.th + dth / 2.0)
            self.th += dth
            vx = d / dt
            vth = dth / dt
            # Publish odometry
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
            self.odom_pub.publish(odom)
        self.last_enc_left = enc_left
        self.last_enc_right = enc_right
        self.last_time = now

    def publish_status(self):
        status = RoboclawStatus()
        # Read motor 1 data
        m1_qpps_raw = self.roboclaw.ReadSpeedM1(self.address)
        m1_enc_raw = self.roboclaw.ReadEncM1(self.address)
        m1_current_raw = self.roboclaw.ReadCurrents(self.address)
        m1_qpps = int(m1_qpps_raw[1]) if m1_qpps_raw[0] else 0
        m1_speed = float(self.pulses_to_meters(m1_qpps))
        m1_motor_current = float(m1_current_raw[1])/100.0 if m1_current_raw[0] and len(m1_current_raw) > 1 else 0.0
        m1_encoder_value = int(m1_enc_raw[1]) if m1_enc_raw[0] else 0
        m1_encoder_status = int(m1_enc_raw[2]) if m1_enc_raw[0] and len(m1_enc_raw) > 2 else 0

        # Read motor 2 data
        m2_qpps_raw = self.roboclaw.ReadSpeedM2(self.address)
        m2_enc_raw = self.roboclaw.ReadEncM2(self.address)
        m2_qpps = int(m2_qpps_raw[1]) if m2_qpps_raw[0] else 0
        m2_speed = float(self.pulses_to_meters(m2_qpps))
        m2_motor_current = float(m1_current_raw[2])/100.0 if m1_current_raw[0] and len(m1_current_raw) > 2 else 0.0
        m2_encoder_value = int(m2_enc_raw[1]) if m2_enc_raw[0] else 0
        m2_encoder_status = int(m2_enc_raw[2]) if m2_enc_raw[0] and len(m2_enc_raw) > 2 else 0

        # Clamp unsigned int fields to >= 0
        m1_encoder_value = max(0, m1_encoder_value)
        m1_encoder_status = max(0, m1_encoder_status)
        m2_encoder_value = max(0, m2_encoder_value)
        m2_encoder_status = max(0, m2_encoder_status)
        qppr = max(0, self.qppr)
        max_speed = max(0, int(self.max_speed))
        max_speed_qpps = max(0, self.max_speed_qpps)
        accel = max(0, int(self.accel))
        accel_qpps = max(0, self.accel_qpps)

        # Assign to message fields
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

        # Read voltages and temperature
        main_batt = self.roboclaw.ReadMainBatteryVoltage(self.address)
        logic_batt = self.roboclaw.ReadLogicBatteryVoltage(self.address)
        temp = self.roboclaw.ReadTemp(self.address)
        err = self.roboclaw.ReadError(self.address)
        err_code = err[1] if err[0] else 0

        status.main_battery_voltage = max(0.0, float(main_batt[1])/10.0 if main_batt[0] else 0.0)
        status.logic_battery_voltage = max(0.0, float(logic_batt[1])/10.0 if logic_batt[0] else 0.0)
        status.temperature = float(temp[1])/10.0 if temp[0] else 0.0
        status.error_string = f"{err_code}"
        self.status_pub.publish(status)

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
