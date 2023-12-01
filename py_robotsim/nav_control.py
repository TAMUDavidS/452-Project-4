import rclpy

from rclpy import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class NavController(Node):
    def __init__(self):
        super().__init__("nav_control_node")

        # Create command publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_msg = Twist()

        # Create laser scan subscriber
        self.scan_sub = self.create_subscriber(LaserScan, '/scan', self.scan_callback)
    
    # Listen to scan and update commands
    def scan_callback(self, scan):
        self.updateMovement(scan)
        self.cmd_pub.publish(self.cmd_msg)

    # Update movement
    def update_movement(self, scan):
        # Move towards furthest point?
        # Avoid walls?
        return
    


def main():
    rclpy.init()

    nc = NavController()

    try:
        rclpy.spin(nc)
    except KeyboardInterrupt:
        pass

    nc.destroy_node()
    rclpy.shutdown()