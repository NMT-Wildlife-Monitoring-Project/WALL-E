#!/usr/bin/env python

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
        v = msg.linear.x  # Linear velocity in m/s
        omega = msg.angular.z  # Angular velocity in rad/s

        # Calculate left and right wheel velocities (m/s)
        v_left = v - (omega * self.wheel_base / 2.0)
        v_right = v + (omega * self.wheel_base / 2.0)

        # Convert to RPM
        rpm_left = (v_left / self.wheel_radius) * \
            60 / (2 * 3.14159)  # m/s to RPM
        rpm_right = (v_right / self.wheel_radius) * 60 / (2 * 3.14159)

        # Apply limits to RPM
        if abs(rpm_left) < self.min_rpm:
            rpm_left = 0
        if abs(rpm_right) < self.min_rpm:
            rpm_right = 0

        # Cap RPM values
        if rpm_left > self.max_rpm:
            rpm_left = self.max_rpm
        elif rpm_left < -self.max_rpm:
            rpm_left = -self.max_rpm

        if rpm_right > self.max_rpm:
            rpm_right = self.max_rpm
        elif rpm_right < -self.max_rpm:
            rpm_right = -self.max_rpm

        # Prepare command bytes
        if rpm_left == 0 and rpm_right == 0:
            command = bytes([0])  # Shutdown both motors
        else:
            motor1_command = 1 + \
                int(126 * ((rpm_left / (2 * self.max_rpm)) + 0.5))
            motor2_command = 128 + \
                int(127 * ((rpm_right / (2 * self.max_rpm)) + 0.5))

            command = bytes([motor1_command, motor2_command])

        # Send command to motors
        self.serial.write(command)
        # rospy.loginfo(f'Sent command: Motor 1: {command[0]}, Motor 2: {command[1]}')


def main():
    try:
        node = MotorControlNode()
        rospy.spin()  # Keep the node running
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
