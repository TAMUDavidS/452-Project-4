import rclpy

from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

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
        angle_error = 0.2
        movement_speed = 0.2

        # Move towards furthest point?
        # Get largest distance
        index = 0
        for i in range(len(scan.ranges)):
            if scan.ranges[i] > scan.ranges[index]:
                index = i
        
        angle = self.get_angle(scan, index)

        # Move to heading if angle is within bounds or rotate to heading
        if abs(angle) > angle_error:
            self.cmd_msg.angular.z = angle*angular_scaling
        else:
            self.cmd_msg.linear.x = movement_speed

        

    
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