import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time
import math

# Define the wheel radius (in meters)
WHEEL_RADIUS = 0.1  # Adjust this value based on your robot's wheel radius

# Setup serial connections for the Arduinos
try:
    serial_arduino_1 = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    serial_arduino_2 = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
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

    def rps_to_pwm(self, rps):
        """Convert RPS to PWM value scaled to -255 to 255."""
        pwm_value = rps * 255  # Scale RPS to PWM range
        return max(-255, min(255, int(pwm_value)))  # Clamp PWM value between -255 and 255

    def listener_callback(self, msg):
        # Map linear.x to forward/backward speed
        linear_speed = max(-1.0, min(1.0, msg.linear.x))  # Clamp between -1 and 1
        angular_speed = max(-1.0, min(1.0, msg.angular.z))  # Clamp between -1 and 1

        # Calculate velocities for left and right wheels
        v_left = linear_speed - angular_speed * (WHEEL_RADIUS / 2)
        v_right = linear_speed + angular_speed * (WHEEL_RADIUS / 2)

        # Calculate RPS for left and right wheels
        rps_left = v_left / (2 * math.pi * WHEEL_RADIUS)
        rps_right = v_right / (2 * math.pi * WHEEL_RADIUS)

        # Convert RPS to PWM
        pwm_left = self.rps_to_pwm(rps_left)
        pwm_right = self.rps_to_pwm(rps_right)

        # Send commands to Arduino based on PWM values
        if serial_arduino_1 and serial_arduino_2:
            try:
                if pwm_left != 0 or pwm_right != 0:
                    # Forward or reverse with turning
                    command_arduino_1 = f"Speed:{pwm_left}\n".encode()
                    command_arduino_2 = f"Speed:{pwm_right}\n".encode()
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