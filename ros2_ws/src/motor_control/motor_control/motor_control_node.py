import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

# GPIO Pin Definitions
S1_PIN = 22  # Linear velocity (x)
S2_PIN = 23  # Angular velocity (yaw)

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')

        # Set up GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(S1_PIN, GPIO.OUT)
        GPIO.setup(S2_PIN, GPIO.OUT)

        # Initialize PWM on both pins with 100 Hz frequency
        self.pwm_s1 = GPIO.PWM(S1_PIN, 100)
        self.pwm_s2 = GPIO.PWM(S2_PIN, 100)
        self.pwm_s1.start(0)
        self.pwm_s2.start(0)

        # Subscribe to the /cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)

        self.get_logger().info("Motor control node initialized.")

    def cmd_vel_callback(self, msg):
        # Extract linear x and angular z
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Map the linear x (range: -1 to 1) to PWM (0 to 100)
        pwm_s1_value = max(0, min(100, (linear_x + 1) * 50))
        pwm_s2_value = max(0, min(100, (angular_z + 1) * 50))

        # Set PWM duty cycle
        self.pwm_s1.ChangeDutyCycle(pwm_s1_value)
        self.pwm_s2.ChangeDutyCycle(pwm_s2_value)

        self.get_logger().info(f"Set PWM: S1 (linear x) = {pwm_s1_value}, S2 (angular z) = {pwm_s2_value}")

    def destroy_node(self):
        # Cleanup GPIO on exit
        self.pwm_s1.stop()
        self.pwm_s2.stop()
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    motor_control_node = MotorControlNode()

    try:
        rclpy.spin(motor_control_node)
    except KeyboardInterrupt:
        motor_control_node.get_logger().info('Shutting down motor control node.')

    motor_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
