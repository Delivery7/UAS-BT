import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial  # Use pyserial

# Initialize serial connections for the Arduino boards
arduino_1 = serial.Serial('/dev/ttyUSB1', 9600, timeout=10)
arduino_2 = serial.Serial('/dev/ttyACM0', 9600, timeout=10)

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        # Forward / Backward
        # Extract linear and angular velocity from the Twist message
        linear_velocity = msg.linear.x  # Adjust if needed
        angular_velocity = msg.angular.z  # Adjust if needed
        
        # Send data to Arduino (customize the data format as needed for your Arduinos)
        arduino_1.write(f"{linear_velocity}\n".encode())  # Write to Arduino 1
        arduino_2.write(f"{angular_velocity}\n".encode())  # Write to Arduino 2

def main(args=None):
    rclpy.init(args=args)
    motor_control_node = MotorControlNode()
    rclpy.spin(motor_control_node)
    motor_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
