import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')

        # Declare parameters for serial port and baudrate
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 9600)

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        # Initialize serial connection
        try:
            self.serial_conn = serial.Serial(port, baudrate, timeout=1)
            self.get_logger().info(f"Connected to {port} at {baudrate} baud.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            return

        # Subscribe to /cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.get_logger().info("Subscribed to /cmd_vel.")

    def cmd_vel_callback(self, msg):
        # Extract linear and angular velocities
        linear_speed = msg.linear.x
        angular_speed = msg.angular.z

        # Create the serial command
        command = f"linear:{linear_speed:.2f} angular:{angular_speed:.2f}\n"

        # Send command over serial
        try:
            self.serial_conn.write(command.encode('utf-8'))
            self.get_logger().info(f"Sent command: {command.strip()}")
        except Exception as e:
            self.get_logger().error(f"Failed to send command: {e}")


def main(args=None):
    rclpy.init(args=args)
    serial_node = SerialNode()

    try:
        rclpy.spin(serial_node)
    except KeyboardInterrupt:
        serial_node.get_logger().info("Shutting down serial node.")
    finally:
        serial_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
