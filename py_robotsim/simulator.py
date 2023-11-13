import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from math import cos, sin, sqrt
import numpy as np
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
import json

# Resource: https://docs.ros.org/en/foxy/Tutorials/Intermediate/URDF/Using-URDF-with-Robot-State-Publisher.html

# How often the timer will listen for updates
TIMESCALE = 0.01

# Node that simulates robot motion
class Simulator(Node):
    def __init__(self):
        super().__init__("simulator_node")

        # Require robot file to be loaded
        self.declare_parameter('robot', "")
        self.robot = json.loads(self.get_parameter('robot').get_parameter_value().string_value)
        #self.get_logger().info("{}".format(self.robot))

        # Subscribe to velocity topics
        self.left_vel_sub  = self.create_subscription(Float64, '/vl', self.update_left, 10)
        self.right_vel_sub = self.create_subscription(Float64, '/vr', self.update_right, 10)
        
        # Subscribe to velocity time out timer
        self.time_out = False
        self.time_out_counter  = 1/TIMESCALE
        self.timer = self.create_timer(TIMESCALE, self.update_timeout)

        # Execution timer
        self.update_timer = self.create_timer(TIMESCALE, self.update_position)

        # Create error update timer
        self.error_right = 0.0
        self.error_left  = 0.0
        self.error_time  = self.robot['wheels']['error_update_rate']
        self.error_right_base = self.robot['wheels']['error_variance_right']
        self.error_left_base  = self.robot['wheels']['error_variance_left']
        self.error_timer = self.create_timer(self.error_time, self.update_error)

        # Initialize velocity to 0
        self.left_vel  = 0.0
        self.right_vel = 0.0

        # Position and orientation
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.l = self.robot['wheels']['distance']

        # State broadcaster
        qos_profile = QoSProfile(depth=10)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        #self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)

    # Broadcaster
    def broadcast(self):
        odom_trans = TransformStamped()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'axis'
        joint_state = JointState()

        # update joint_state
        now = self.get_clock().now()
        joint_state.header.stamp = now.to_msg()
        joint_state.name = ['swivel', 'tilt', 'periscope']
        joint_state.position = [0., 0., 0.]

        # update transform
        # (moving in a circle with radius=2)
        odom_trans.header.stamp = now.to_msg()
        odom_trans.transform.translation.x = float(self.x)
        odom_trans.transform.translation.y = float(self.y)
        odom_trans.transform.translation.z = 0.0
        odom_trans.transform.rotation = \
            euler_to_quaternion(0, 0, self.theta) # roll,pitch,yaw

        # send the joint state and transform
        #self.joint_pub.publish(joint_state)
        self.broadcaster.sendTransform(odom_trans)

    # Update position
    def update_position(self):
        # Get input variables
        R = self.get_radius()
        w = self.get_angular_vel()
        c = self.get_ICC(R)
        dt = TIMESCALE

        # Calculate new position
        A = np.matrix([[cos(w*dt), -sin(w*dt), 0],
                       [sin(w*dt), cos(w*dt),  0],
                       [0        , 0,          1]])
        B = np.matrix([[self.x-c[0], self.y-c[1], self.theta]]).transpose()
        C = np.matrix([[c[0], c[1], w*dt]]).transpose()
        self.get_logger().info("B: {} C: {}".format(B.shape, C.shape))
        result_matrix = A*B
        #result_matrix = result_matrix+C

        # Update postion
        self.x = result_matrix.item(0)
        self.y = result_matrix.item(1)
        self.theta = result_matrix.item(2)

        self.broadcast()

    
    # Get turn radius
    def get_radius(self):
        return (self.l/2)*((self.right_vel+self.left_vel)/(self.right_vel-self.left_vel) ) if (self.right_vel-self.left_vel) > 0 else 0.0
    
    # Get angular velocity
    def get_angular_vel(self):
        return (self.right_vel-self.left_vel)/self.l
    
    # Get ICC
    def get_ICC(self, R):
        return (self.x-R*sin(self.theta),self.y+R*cos(self.theta))

    # Update left wheel velocity
    def update_left(self, velocity):
        self.left_vel = velocity.data*self.error_left
        self.reset_timer()
    
    # Update right wheel velocity
    def update_right(self, velocity):
        self.right_vel = velocity.data*self.error_right
        self.reset_timer()
    
    # Update velocity time out
    def update_timeout(self):
        self.time_out_counter = self.time_out_counter-1
        if self.time_out_counter <= 0:
            self.time_out_counter = True
    
    # Reset velocity time out
    def reset_timer(self):
        self.time_out = False
        self.time_out_counter = 1/TIMESCALE
    
    # Update velocity time out
    def update_timeout(self):
        self.time_out_counter = self.time_out_counter-1
        if self.time_out_counter <= 0:
            self.time_out_counter = True
    
    # Reset velocity time out
    def reset_timer(self):
        self.time_out = False
        self.time_out_counter = 1/TIMESCALE
    
    # Update error on gaussian distribution
    def update_error(self):
        self.error_left  = np.random.normal(1.0, sqrt(self.error_left_base), 1)
        self.error_right = np.random.normal(1.0, sqrt(self.error_right_base), 1)

# Helper function for broadcasting input
def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


# Start simulator
def main():
    rclpy.init()

    simulator = Simulator()

    try:
        rclpy.spin(simulator)
    except KeyboardInterrupt:
        pass

    simulator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
