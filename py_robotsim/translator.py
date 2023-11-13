import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

#velocity translator class
class translator(Node):

    def __init__(self):
        #initialize node
        super().__init__('translator') 
        #create subscription
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.callback, 10)
        #create publishers
        self.vr_publisher = self.create_publisher(Float64,'/vr', 10)
        self.vl_publisher = self.create_publisher(Float64, '/vl', 10)
    
    def callback(self, message):
        #collect linear and angular velocities
        linear_vel = message.linear.x
        angular_vel = message.angular.z

        #set wheel velocities
        vr_vel = linear_vel + (angular_vel / 2)
        vl_vel = linear_vel - (angular_vel / 2)

        #publish wheel velocities
        vr_message = Float64()
        vr_message.data = vr_vel
        self.vr_publisher.publish(vr_message)

        vl_message = Float64()
        vl_message.data = vl_vel
        self.vl_publisher.publish(vl_message)
              
def main():
    rclpy.init()
    translator_node = translator()
    rclpy.spin(translator_node)
    translator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()