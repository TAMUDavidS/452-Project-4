import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float64

from tf2_ros import TransformBroadcaster, TransformStamped

from math import cos, sin, sqrt, floor, nan
import numpy as np

import json

# Resources:
# https://docs.ros.org/en/foxy/Tutorials/Intermediate/URDF/Using-URDF-with-Robot-State-Publisher.html
# https://lodev.org/cgtutor/raycasting.html

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
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)

        # Map subscriber
        map_subscriber = self.create_subscribtion(OccupancyGrid, '/map', self.update_map, 10)
        self.map = OccupancyGrid()

    # Broadcaster
    # TODO: Lidar transform
    def broadcast(self):
        odom_trans = TransformStamped()
        odom_trans.header.frame_id = 'heading_box'
        odom_trans.child_frame_id = 'world'
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
        self.joint_pub.publish(joint_state)
        self.broadcaster.sendTransform(odom_trans)

    # Update position
    # TODO: Check for collision and stop
    def update_position(self):
        #self.get_logger().info("Time: {} {}".format(self.time_out_counter, self.time_out))
        if self.time_out:
            self.left_vel  = 0.0
            self.right_vel = 0.0

        # Get input variables
        R = self.get_radius()
        w = self.get_angular_vel()
        c = self.get_ICC(R)
        dt = TIMESCALE

        #self.get_logger().info("R: {} w: {} c: {} {}".format(R,w,c[0],c[1]))

        # Calculate new position
        A = np.matrix([[cos(w*dt), -sin(w*dt), 0],
                       [sin(w*dt), cos(w*dt),  0],
                       [0        , 0,          1]])
        B = np.matrix([[self.x-c[0], self.y-c[1], self.theta]]).transpose()
        C = np.matrix([[c[0], c[1], w*dt]]).transpose()
        #self.get_logger().info("B: {} C: {}".format(B.shape, C.shape))
        result_matrix = np.matmul(A, B)
        result_matrix = np.add(result_matrix,C)

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
        return float((self.right_vel-self.left_vel)/self.l)
    
    # Get ICC
    def get_ICC(self, R):
        x = float(self.x-R*sin(self.theta))
        y = float(self.y+R*cos(self.theta))
        return (x,y)

    # Update left wheel velocity
    def update_left(self, velocity):
        self.left_vel = velocity.data*self.error_left
        self.reset_timer()
    
    # Update right wheel velocity
    def update_right(self, velocity):
        self.right_vel = velocity.data*self.error_right
        self.reset_timer()
    
    # Get map
    def update_map(self, map):
        self.map = map

    # Update velocity time out
    def update_timeout(self):
        self.time_out_counter = self.time_out_counter-1
        if self.time_out_counter <= 0:
            self.time_out_counter = True
    
    # Update velocity time out
    def update_timeout(self):
        self.time_out_counter = self.time_out_counter+TIMESCALE
        if self.time_out_counter >= 1:
            self.time_out = True
    
    # Reset velocity time out
    def reset_timer(self):
        self.time_out = False
        self.time_out_counter = 0.0
    
    # Update error on gaussian distribution
    def update_error(self):
        self.error_left  = np.random.normal(1.0, sqrt(self.error_left_base), 1) if not self.error_left_base == 0.0 else 1.0
        self.error_right = np.random.normal(1.0, sqrt(self.error_right_base), 1) if not self.error_right_base == 0.0 else 1.0
    
    # Get grid location of robot on the map
    def get_grid_pos(self):
        dx = self.x/self.map.info.resolution
        dy = self.y/self.map.info.resolution
        xg = floor(dx)
        yg = floor(dy)
        xe = dx-xg
        ye = dy-yg
        return (xg,yg,xe,ye)

    # TODO: Check if the robot has encountered a wall
    def check_collision(self):
        return True
    
    # TODO: Simulate lidar scans
    def lidar_scan(self):
        # Calculate span of angles to conduct scans
        count = self.robot["laser"]["count"]
        angle_span = self.robot["laser"]["angle_max"]-self.robot["laser"]["angle_min"]
        angle = self.robot["laser"]["angle_min"]
        step = angle_span/count
        
        # Robot position
        pos = self.get_grid_pos()
        posX = pos[0]
        posY = pos[1]

        scans = []

        # Test ray collision at each step
        for i in range(0,count):
            # Check if scan fails
            fail = True if np.random() < self.robot["laser"]["fail_probability"] else False

            # Check for collision
            ray = self.raycast(self.robot["laser"]["range_min"],self.robot["laser"]["range_max"],
                         angle, posX, posY)
            
            # Check if there was a collision
            if ray[0] and not fail:
                dist = ray[3]
                error = np.random.normal(0.0, sqrt(self.robot["laser"]["error_variance"]), 1)
                dist += error
                scans.append(dist)
            else:
                scans.append(nan)
            
            angle += step

        return scans
    
    # TODO: Check for line collision
    def raycast(self, range_min, range_max, angle, posX, posY):
        # Robot position
        posX = 0
        posY = 0

        #which box of the map we're in
        mapX = int(posX)
        mapY = int(posY)

        # ray dir
        rayDirX = 0
        rayDirY = 0

        #length of ray from current position to next x or y-side
        sideDistX = 0
        sideDistY = 0

        #length of ray from one x or y-side to next x or y-side
        deltaDistX =  1e30 if rayDirX == 0 else abs(1 / rayDirX)
        deltaDistY =  1e30 if rayDirY == 0 else abs(1 / rayDirY)
        perpWallDist = 0

        #that direction to step in x or y-direction (either +1 or -1)
        stepX = 0
        stepY = 0

        hit = 0  #was there a wall hit?
        side = 0 #was a NS or a EW wall hit?

        #calculate step and initial sideDist
        if rayDirX < 0:
            stepX = -1
            sideDistX = (posX - mapX) * deltaDistX
        else:
            stepX = 1
            sideDistX = (mapX + 1.0 - posX) * deltaDistX
        
        if rayDirY < 0:
            stepY = -1
            sideDistY = (posY - mapY) * deltaDistY
        else:
            stepY = 1
            sideDistY = (mapY + 1.0 - posY) * deltaDistY

        #perform DDA
        while (hit == 0):
            #jump to next map square, either in x-direction, or in y-direction
            if sideDistX < sideDistY:
                sideDistX += deltaDistX
                mapX += stepX
                side = 0
            else:
                sideDistY += deltaDistY
                mapY += stepY
                side = 1
            #Check if ray has hit a wall
            if worldMap[mapX][mapY] > 0: hit = 1

        return (hit, mapX, mapY, dist)
        

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
