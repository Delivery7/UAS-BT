import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import bobserial as bob  # Assuming bobserial is your serial interface library
import time

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        self.arduino_serial = bob.ArduinoSerial('/dev/ttyUSB0', 115200, 1)
        
        # Subscribe to the /cmd_vel topic for Twist messages
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.callback,
            10
        )
        
        # Set a timer to read the encoder value periodically
        self.timer = self.create_timer(0.1, self.read_encoder)

    def callback(self, cmd_vel):
        # Convert linear x velocity to PWM range (-255 to 255)
        linear_x = cmd_vel.linear.x
        pwm_speed = int(linear_x * 255)  # Scale speed to PWM range
        
        # Send speed command to the Arduino
        self.arduino_serial.write(f"Speed:{pwm_speed}\n")
        self.get_logger().info(f"Sent Speed: {pwm_speed}")

    def read_encoder(self):
        # Send read command to Arduino and process the response
        self.arduino_serial.write("Read\n")
        
        try:
            value = self.arduino_serial.read().strip()  # Read and strip whitespace
            
            if value:  # Check for non-empty response
                if "Encoder:" in value:
                    encoder_value = value.split("Encoder:")[1].strip()
                    self.get_logger().info(f"Received Encoder: {encoder_value}")
                else:
                    self.get_logger().warn(f"Unexpected format: {value}")
        
        except UnicodeDecodeError:
            self.get_logger().error("UnicodeDecodeError: Received data cannot be decoded. Skipping this value.")

    def destroy_node(self):
        self.arduino_serial.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    motor_control_node = MotorControlNode()
    rclpy.spin(motor_control_node)
    motor_control_node.destroy_node()
    rclpy.shutdown()
    
    try:
        rclpy.spin(motor_control_node)
    except KeyboardInterrupt:
        motor_control_node.get_logger().info("Shutting down...")
    finally:
        motor_control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
