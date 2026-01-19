import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

# Setup serial connections for the Arduinos
try:
    serial_arduino_1 = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    serial_arduino_2 = serial.Serial('/dev/ttyACM1', 9600, timeout=1)
    time.sleep(2)  # Wait for the serial connection to initialize
except serial.SerialException as e:
    print(f"Error connecting to Arduino: {e}")
    serial_arduino_1, serial_arduino_2 = None, None

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',  # Listening to the cmd_vel topic
            self.listener_callback,
            10
        )
        self.linear_pwm = 0   # Current linear speed in PWM
        self.angular_pwm = 0  # Current angular speed in PWM

    def listener_callback(self, msg):
        # Map linear.x to forward/backward speed (scaled to -255 to 255)
        linear_speed = max(-0.5, min(1.0, msg.linear.x))  # Clamp between -1 and 1
        self.linear_pwm = int(linear_speed * 255)         # Scale to PWM range

        # Map angular.z to turning speed (scaled to -255 to 255)
        angular_speed = max(-0.5, min(0.5, msg.angular.z))  # Clamp between -1 and 1
        self.angular_pwm = int(angular_speed * 255)         # Scale to PWM range

        # Send commands to Arduino based on linear and angular speeds
        if serial_arduino_1 and serial_arduino_2:
            try:
                if self.linear_pwm != 0 or self.angular_pwm != 0:
                    # Forward or reverse with turning
                    command_arduino_1 = f"Speed:{self.linear_pwm + self.angular_pwm}\n".encode()
                    command_arduino_2 = f"Speed:{self.linear_pwm - self.angular_pwm}\n".encode()
                else:
                    # Stop both motors
                    command_arduino_1 = b"Speed:0\n"
                    command_arduino_2 = b"Speed:0\n"

                serial_arduino_1.write(command_arduino_1)
                serial_arduino_2.write(command_arduino_2)
                print(f"Sent to Arduino 1: {command_arduino_1.decode().strip()}")
                print(f"Sent to Arduino 2: {command_arduino_2.decode().strip()}")
            except serial.SerialException as e:
                self.get_logger().error(f"Failed to send command: {e}")
        else:
            self.get_logger().error("No serial connection to Arduino.")

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
