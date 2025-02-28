#!/usr/bin/env python2
import rospy
from geometry_msgs.msg import Twist
from dual_g2_hpmd_rpi import motors, MAX_SPEED


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
        self.max_velocity = rospy.get_param('~max_velocity', 5)
        self.min_velocity = rospy.get_param('~min_velocity', 0.1)
        self.motor_serial_device = rospy.get_param(
            '~motor_serial_device', '/dev/serial0')  # Default serial device

        self.wheel_radius = self.wheel_diameter / 2.0

        # Subscriber
        self.cmd_vel_subscriber = rospy.Subscriber(
            self.cmd_vel_topic, Twist, self.cmd_vel_callback)

    def cmd_vel_callback(self, msg):
        # Extract linear and angular velocities
        v = msg.linear.x      # forward velocity (m/s)
        omega = msg.angular.z  # angular velocity (rad/s)

        # Calculate wheel velocities (m/s)
        v_left = v - (omega * self.wheel_base / 2.0)
        v_right = v + (omega * self.wheel_base / 2.0)

        # Apply a deadband for small speeds.
        if abs(v_left) < self.min_velocity:
            v_left = 0
        if abs(v_right) < self.min_velocity:
            v_right = 0
        speed_left = int(v_left/self.max_velocity * MAX_SPEED)
        speed_right = int(v_right/self.max_velocity * MAX_SPEED)

        # Send the speed commands via the dual_g2_hpmd_rpi API.
        motors.setSpeeds(speed_left, speed_right)
        print("Set speeds: motor1: %d, motor2: %d",
                      speed_left, speed_right)
        rospy.loginfo("Set speeds: motor1: %d, motor2: %d",
                      speed_left, speed_right)


def main():
    try:
        node = MotorControlNode()
        rospy.spin()  # Keep the node running
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
