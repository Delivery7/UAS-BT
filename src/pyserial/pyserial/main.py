#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster

import math
import serial
import time

from pyserial.kinematics import DiffKinematic

# class DiffKinematic:
#     def __init__(self, wheel_rad, robot_rad):
#         self.r = wheel_rad
#         self.L = robot_rad

#     def twist_to_rpm(self, v, w):
#         v_r = (2 * v + w * self.L) / (2 * self.r)
#         v_l = (2 * v - w * self.L) / (2 * self.r)
#         return v_r, v_l

#     def rpm_to_twist(self, rpmL, rpmR):
#         v = (self.r / 2.0) * (rpmR + rpmL)
#         w = (self.r / (2.0 * self.L)) * (rpmR - rpmL)
#         return v, w

# Constants
wheel_rad = 0.1  # meter (sudah radius, tidak perlu dibagi 2)
robot_rad = 0.41  # meter (jarak sumbu roda / 2)
kinematic = DiffKinematic(robot_rad, wheel_rad)
class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_node')

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.br = TransformBroadcaster(self)
        print("init1")
        # Parameters
        self.declare_parameter('port_left', '/dev/ttyUSB0')
        self.declare_parameter('port_right', '/dev/ttyACM0')

        port_left = self.get_parameter('port_left').get_parameter_value().string_value
        port_right = self.get_parameter('port_right').get_parameter_value().string_value

        self.arduino_L = serial.Serial(port_left, 9600, timeout=0.5)
        self.arduino_R = serial.Serial(port_right, 9600, timeout=0.5)
        time.sleep(2)
        print("init2")

        # State
        self.prev_time = self.get_clock().now().nanoseconds / 1e9
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        # Main loop
        self.timer = self.create_timer(0.02, self.update_odometry)  # 50 Hz
        print("init_done")
    def parse_serial(self, serial_data):
        try:
            data = serial_data.decode().strip()
            if data.startswith("ENC:R="):
                return float(data.replace("ENC:R=", ""))
            elif data.startswith("ENC:L="):
                return float(data.replace("ENC:L=", ""))
        except:
            return None

    def update_odometry(self):
        print("stuck odom")
        now = self.get_clock().now().nanoseconds / 1e9
        dt = now - self.prev_time
        self.prev_time = now

        rpmL = 0.0
        rpmR = 0.0

        # Baca dari Serial Arduino
        if self.arduino_L.in_waiting:
            print("1")
            line = self.arduino_L.readline()
            val = self.parse_serial(line)
            if val is not None:
                print("not 1")
                rpmL = val

        if self.arduino_R.in_waiting:
            print("2")
            line = self.arduino_R.readline()
            val = self.parse_serial(line)
            if val is not None:
                rpmR = val
        print("stuck after")
        # Hitung Twist dari encoder
        v, w = kinematic.forward_kinematic(rpmL, rpmR)

        # Integrasi posisi (Euler)
        delta_x = v * math.cos(self.th) * dt
        delta_y = v * math.sin(self.th) * dt
        delta_th = w * dt

        self.x += delta_x * 10
        self.y += delta_y * 10
        self.th += delta_th
        print(f"x = {self.x}\ny = {self.y}\nyaw = {self.th}\n")
        # Buat pesan Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        qx, qy, qz, qw = self.euler_to_quaternion(0.0, 0.0, self.th)
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        odom_msg.twist.twist.linear.x = v
        odom_msg.twist.twist.angular.z = w

        self.odom_pub.publish(odom_msg)

        # TF Broadcast
        t = TransformStamped()
        t.header.stamp = odom_msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.br.sendTransform(t)

    def euler_to_quaternion(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qx = cy * cp * sr - sy * sp * cr
        qy = sy * cp * sr + cy * sp * cr
        qz = sy * cp * cr - cy * sp * sr
        qw = cy * cp * cr + sy * sp * sr
        return qx, qy, qz, qw
    
    

def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
