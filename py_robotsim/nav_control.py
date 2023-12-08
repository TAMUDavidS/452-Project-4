import rclpy

from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import math

class NavController(Node):
    def __init__(self):
        super().__init__("nav_control_node")

        # Create command publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_msg = Twist()

        # Create laser scan subscriber
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
    
    # Listen to scan and update commands
    def scan_callback(self, scan):
        self.update_movement(scan)
        self.cmd_pub.publish(self.cmd_msg)

    # Update movement
    def update_movement(self, scan):
        # Scaling factors for movement
        angular_scaling = 0.1
        angular_vel = 0.2
        movement_speed = 0.2

        # Get min and max range
        max = 0
        min = 0
        for i in range(len(scan.ranges)):
            if scan.ranges[i] > scan.ranges[max]:
                max = i
            if scan.ranges[i] < scan.ranges[min]:
                min = i
        
        angle_max = self.get_angle(scan, max)
        angle_min = self.get_angle(scan, min)

        # Keep moving forward
        self.cmd_msg.linear.x = movement_speed

        # Keep centered by moving away from nearest wall
        if angle_min == 0:
            self.cmd_msg.angular.z = angular_vel*(angle_max/abs(angle_max)) # gets sign of largest distance
        if angle_min > 0:
            self.cmd_msg.angular.z = -1*angular_vel
        if angle_min < 0:
            self.cmd_msg.angular.z = angular_vel
        

    
    # Convert index to angle
    def get_angle(self, scan, index):
        if index < len(scan.ranges):
            return scan.angle_increment*index + scan.angle_min
        else:
            return 0.0  


def main():
    rclpy.init()

    nc = NavController()

    try:
        rclpy.spin(nc)
    except KeyboardInterrupt:
        pass

    nc.destroy_node()
    rclpy.shutdown()