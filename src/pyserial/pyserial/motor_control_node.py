import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import bobserial as bob  # Make sure bobserial is installed and correctly imported

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        self.arduino_serial = bob.ArduinoSerial('/dev/ttyUSB0', 115200, 1)  # Adjust port if necessary
        
        # Subscribe to the /cmd_vel topic for Twist messages
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.callback,
            10
        )
        self.get_logger().info("Motor Control Node started.")

    def callback(self, cmd_vel):
        linear_x = cmd_vel.linear.x
        pwm_speed = int(linear_x * 200)  # Convert linear velocity to PWM range (-255 to 255)

        # Send speed command to the Arduino
        self.arduino_serial.write(f"Speed:{pwm_speed}\n")
        self.get_logger().info(f"Sent Speed: {pwm_speed}")

def main(args=None):
    rclpy.init(args=args)
    motor_control_node = MotorControlNode()
    try:
        rclpy.spin(motor_control_node)
    except KeyboardInterrupt:
        pass
    finally:
        motor_control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
