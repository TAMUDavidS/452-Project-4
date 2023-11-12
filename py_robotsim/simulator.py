import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from math import cos, sin, sqrt
import numpy as np

# How often the timer will listen for updates
TIMESCALE = 0.01

# Node that simulates robot motion
class Simulator(Node):
    def __init__(self):
        super().__init__("simulator_node")

        # Subscribe to velocity topics
        self.left_vel_sub  = self.create_subscription(Float64, '/vl', self.update_left, 10)
        self.right_vel_sub = self.create_subscription(Float64, '/vr', self.update_right, 10)
        
        # Subscribe to velocity time out timer
        self.time_out = False
        self.time_out_counter  = 1/TIMESCALE
        self.timer = self.create_timer(TIMESCALE, self.update_timeout)

        # Create error update timer
        self.error_right = 0.0
        self.error_left  = 0.0
        self.error_time  = 0.5
        self.error_right_base = 0.0001
        self.error_left_base  = 0.0001
        self.error_timer = self.create_timer(self.error_time, self.update_error)

        # Initialize velocity to 0
        self.left_vel  = 0.0
        self.right_vel = 0.0

        # Position and orientation
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.l = 1.0 # TODO: get l from file

    # Update position
    def update_position(self):
        # Get input variables
        R = self.get_radius()
        w = self.get_angular_vel()
        c = self.get_ICC()
        dt = TIMESCALE

        # Calculate new position
        A = np.matrix([[cos(w*dt), -sin(w*dt), 0],
                       [sin(w*dt), cos(w*dt),  0],
                       [0        , 0,          1]])
        B = np.matrix([self.x-c[0], self.y-c[1], self.theta]).transpose()
        C = np.matrix([c[0], c[1], w*dt]).transpose()
        result_matrix = A*B+C

        # Update postion
        self.x = result_matrix.item(0)
        self.y = result_matrix.item(1)
        self.theta = result_matrix.item(2)

    
    # Get turn radius
    def get_radius(self):
        return (self.l/2)*((self.right_vel+self.left_vel)/(self.right_vel-self.left_vel))
    
    # Get angular velocity
    def get_angular_vel(self):
        return (self.right_vel-self.left_vel)/self.l
    
    # Get ICC
    def get_ICC(self, R):
        return (self.x-R*sin(self.theta),self.y+R*cos(self.theta))

    # Update left wheel velocity
    def update_left(self, velocity):
        self.left_vel = velocity*self.error_left
        self.reset_timer()
    
    # Update right wheel velocity
    def update_right(self, velocity):
        self.right_vel = velocity*self.error_right
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

        
# Start simulator
def main():
    rclpy.init()

    simulator = Simulator()

    rclpy.spin(simulator)

    simulator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
