#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
import serial


class MotorControlNode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('motor_control_node', anonymous=True)

        # Get parameters
        self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic', 'cmd_vel')
        self.wheel_base = rospy.get_param(
            '~wheel_base', 0.5)  # Default in meters
        self.wheel_diameter = rospy.get_param(
            '~wheel_diameter', 0.1)  # Default in meters
        # Maximum RPM value for motor control
        self.max_rpm = rospy.get_param('~max_rpm', 100)
        # Minimum RPM value for motor control
        self.min_rpm = rospy.get_param('~min_rpm', 10)
        self.motor_serial_device = rospy.get_param(
            '~motor_serial_device', '/dev/serial0')  # Default serial device

        self.wheel_radius = self.wheel_diameter / 2.0

        # Initialize serial connection
        self.serial = serial.Serial(
            self.motor_serial_device, baudrate=9600, timeout=1)

        # Subscriber
        self.cmd_vel_subscriber = rospy.Subscriber(
            self.cmd_vel_topic, Twist, self.cmd_vel_callback)


    def cmd_vel_callback(self, msg):
        # Extract linear and angular velocities
        v = msg.linear.x      # forward velocity (m/s)
        omega = msg.angular.z # angular velocity (rad/s)

        # Calculate wheel velocities (m/s)
        v_left = v - (omega * self.wheel_base / 2.0)
        v_right = v + (omega * self.wheel_base / 2.0)

        # Convert linear velocities to RPM:
        # RPM = (linear speed / wheel circumference) * 60,
        # where circumference = 2 * pi * wheel_radius.
        rpm_left = (v_left / (2 * 3.14159 * self.wheel_radius)) * 60
        rpm_right = (v_right / (2 * 3.14159 * self.wheel_radius)) * 60

        # Apply a deadband for small speeds.
        if abs(rpm_left) < self.min_rpm:
            rpm_left = 0
        if abs(rpm_right) < self.min_rpm:
            rpm_right = 0

        # Cap the RPM values to the maximum allowed.
        rpm_left = max(min(rpm_left, self.max_rpm), -self.max_rpm)
        rpm_right = max(min(rpm_right, self.max_rpm), -self.max_rpm)

        # Scale the computed RPM values to the command range expected by the driver.
        # This converts the RPM (in range [-max_rpm, max_rpm]) to a speed value in [-MAX_SPEED, MAX_SPEED].
        speed_left = int((rpm_left / self.max_rpm) * MAX_SPEED)
        speed_right = int((rpm_right / self.max_rpm) * MAX_SPEED)

        # Send the speed commands via the dual_g2_hpmd_rpi API.
        motors.setSpeeds(speed_left, speed_right)

        rospy.loginfo("Set speeds: motor1: %d, motor2: %d", speed_left, speed_right)


def main():
    try:
        node = MotorControlNode()
        rospy.spin()  # Keep the node running
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
