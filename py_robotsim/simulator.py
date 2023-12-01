import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState, LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float64, Header

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
# TODO: get robot initial position
class Simulator(Node):
    def __init__(self):
        super().__init__("simulator_node")

        # Require robot file to be loaded
        self.declare_parameter('robot', "")
        self.robot = json.loads(self.get_parameter('robot').get_parameter_value().string_value)
        self.declare_parameter('pos', "")
        start_pos = json.loads(self.get_parameter('pos').get_parameter_value().string_value)
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
        self.error_right = 1.0
        self.error_left  = 1.0
        self.error_time  = self.robot['wheels']['error_update_rate']
        self.error_right_base = self.robot['wheels']['error_variance_right']
        self.error_left_base  = self.robot['wheels']['error_variance_left']
        self.error_timer = self.create_timer(self.error_time, self.update_error)

        # Initialize velocity to 0
        self.left_vel  = 0.0
        self.right_vel = 0.0

        # Position and orientation
        self.x = start_pos["initial_pose"][0]
        self.y = start_pos["initial_pose"][1]
        self.theta = start_pos["initial_pose"][2]
        self.l = self.robot['wheels']['distance']
        self.rad1 = False
        self.rad2 = False
        self.rad3 = False
        self.rad4 = False
        self.rad5 = False
        self.rad6 = False
        self.rad7 = False
        self.rad8 = False
        # State broadcaster
        qos_profile = QoSProfile(depth=10)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)

        # Map subscriber
        self.map = OccupancyGrid()
        self.map_subscriber = self.create_subscription(OccupancyGrid, '/map', self.update_map, 10)

        # Lidar publisher
        self.lidar_seq = 0
        self.lidar_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.lidar_timer = self.create_timer(self.robot["laser"]["rate"], self.publish_scan)

    # Broadcaster
    # TODO: Fix transforms
    def broadcast(self):
        odom_trans = TransformStamped()
        odom_trans.header.frame_id = 'world'
        odom_trans.child_frame_id = 'base_link'
        joint_state = JointState()

        # update joint_state
        now = self.get_clock().now()
        joint_state.header.stamp = now.to_msg()
        joint_state.name = ['swivel', 'tilt', 'periscope']
        joint_state.position = [0., 0., 0.]

        # update transform
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
    def obstacle_contact(self, x, y):
        # Check if point is inside an obstacle
        if self.map.info.resolution == 0.0:
            return False  

        map_width = self.map.info.width
        map_height = self.map.info.height
        map_resolution = self.map.info.resolution
        map_origin_x = self.map.info.origin.position.x
        map_origin_y = self.map.info.origin.position.y

        # Convert world to map coordinates
        map_x = int((x - map_origin_x) / map_resolution)
        map_y = int((y - map_origin_y) / map_resolution)

        # Check for contact
        if (0 <= map_x < map_width and 0 <= map_y < map_height):
            index = map_x + map_y * map_width
            cell_value = self.map.data[index]
            return cell_value > 0  

        return False
    
    # Check if the robot's current position is inside an obstacle in the map
    def collision_check(self):
        # TODO: Check more than just the robots center point
        # Note: could be potentially fixed using several evenly placed points around the robots radius
        radius = self.robot["body"]["radius"]
        self.rad1 = False
        self.rad2 = False
        self.rad3 = False
        self.rad4 = False
        self.rad5 = False
        self.rad6 = False
        self.rad7 = False
        self.rad8 = False
        #check full radius of circle of robot for collision points
        #radian 0
        if self.obstacle_contact(self.x + radius, self.y):
            self.rad1 = True
        #radian 45
        elif self.obstacle_contact(self.x + (radius * sqrt(2) / 2) , self.y + (radius * sqrt(2) / 2)):
            self.rad2 = True
        #radian 90
        elif self.obstacle_contact(self.x , self.y + radius):
            self.rad3 = True
        #radian 135
        elif self.obstacle_contact(self.x - (radius * sqrt(2) / 2), self.y + (radius * sqrt(2) / 2)):
            self.rad4 = True
        #radian 180
        elif self.obstacle_contact(self.x - radius, self.y):
            self.rad5 = True
        #radian 225
        elif self.obstacle_contact(self.x - (radius * sqrt(2) / 2), self.y - (radius * sqrt(2) / 2)):
            self.rad6 = True
        #radian 270
        elif self.obstacle_contact(self.x , self.y - radius):
            self.rad7 = True
        #radian 315
        elif self.obstacle_contact(self.x + (radius * sqrt(2) / 2), self.y - (radius * sqrt(2) / 2)):
            self.rad8 = True

        if (self.rad1 or self.rad2 or self.rad3 or self.rad4 or self.rad5 or self.rad6 or self.rad7 or self.rad8) :
            #self.left_vel = 0.0
            #self.right_vel = 0.0
            return True



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
        result_matrix = np.add(result_matrix, C)

        self.collision_check()
        currentx = self.x
        currenty = self.y
        newx = result_matrix.item(0)
        newy = result_matrix.item(1)
        
        #flag detection
        if (currentx - result_matrix.item(0) < 0) :
            if (self.rad1 or self.rad2 or self.rad8) :
                newx = currentx
        if (currentx - result_matrix.item(0) > 0) :
            if (self.rad4 or self.rad5 or self.rad6) :
                newx = currentx
        if (currenty - result_matrix.item(1) < 0) :
            if (self.rad2 or self.rad3 or self.rad4) :
                newy = currenty
        if (currenty - result_matrix.item(1) > 0) :
            if (self.rad6 or self.rad7 or self.rad8) :
                newy = currenty  
        
        #self.get_logger().info("dX: {} dY: {}, new X: {} Y: {}".format(currentx - result_matrix.item(0), currenty - result_matrix.item(1), newx, newy))

        # Update postion
        self.x = newx
        self.y = newy
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
        self.get_logger().info("Map Loaded")
        #self.x = self.map.info.origin.position.x
        #self.y = self.map.info.origin.position.y
        # TODO: get angle from quaternion

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
        if not self.error_left_base == 0.0:
            self.error_left  = np.random.normal(1.0, sqrt(self.error_left_base), 1)
        else:
            self.error_left  = 1.0
        if not self.error_right_base == 0.0:
            self.error_right = np.random.normal(1.0, sqrt(self.error_right_base), 1)
        else:
            self.error_right  = 1.0
        
        #self.error_right = np.random.normal(1.0, sqrt(self.error_right_base), 1) if not self.error_right_base == 0.0 else 1.0
    
    # Get grid location of robot on the map
    def get_grid_pos(self):
        dx = self.x/self.map.info.resolution
        dy = self.y/self.map.info.resolution
        xg = floor(dx)
        yg = floor(dy)
        xe = dx-xg
        ye = dy-yg
        return (xg,yg,xe,ye)
    
    # Publish lidar scan
    def publish_scan(self):
        laserscan = LaserScan()
        
        # Header message
        header = Header()
        #header.seq = self.lidar_seq
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'laser'
        laserscan.header = header

        # Scan info
        laserscan.angle_min = self.robot["laser"]["angle_min"]
        laserscan.angle_max = self.robot["laser"]["angle_max"]
        laserscan.angle_increment = (self.robot["laser"]["angle_max"]-self.robot["laser"]["angle_min"])/self.robot["laser"]["count"]
        laserscan.scan_time = float(self.robot["laser"]["rate"])
        laserscan.range_min = self.robot["laser"]["range_min"]
        laserscan.range_max = self.robot["laser"]["range_max"]

        # Scan data
        if self.map.info.resolution == 0.0:
            #self.get_logger().info("{}".format(self.map.info))
            laserscan.ranges = [nan] * self.robot["laser"]["count"]
        else:
            #self.get_logger().info("Scan Start")
            ranges = self.lidar_scan()
            #self.get_logger().info("Scan Stop")
            #self.get_logger().info("{}".format(ranges))
            laserscan.ranges = ranges
        #laserscan.ranges = [1.0] * self.robot["laser"]["count"]

        self.lidar_pub.publish(laserscan)

    # Simulate lidar scans
    def lidar_scan(self):
        # Calculate span of angles to conduct scans
        count = self.robot["laser"]["count"]
        angle_span = self.robot["laser"]["angle_max"]-self.robot["laser"]["angle_min"]
        angle = self.robot["laser"]["angle_min"]+self.theta
        step = angle_span/count
        
        # Robot position
        pos = self.get_grid_pos()
        posX = pos[0]
        posY = pos[1]
        #self.get_logger().info("Map: {} {}".format(pos[0],pos[1]))
        #self.get_logger().info("Map Relative: {} {}".format(pos[0]*0.3,pos[1]*0.3))
        #self.get_logger().info("Actual: {} {}".format(self.x,self.y))

        scans = []

        # Test ray collision at each step
        for i in range(0,count):
            # Check if scan fails
            if np.random.random() < self.robot["laser"]["fail_probability"]:
                fail = True
            else: 
                fail = False

            # Check for collision
            #self.get_logger().info("Ray Start: {}".format(angle))
            ray = self.raycast(self.robot["laser"]["range_max"], angle, posX, posY)
            #self.get_logger().info("Ray Stop")
            
            # Check if there was a collision and apply error
            if ray[0] and not fail:
                dist = ray[3]*self.map.info.resolution
                if dist < self.robot["laser"]["range_min"]:
                    scans.append(float('nan'))
                else:
                    error = np.random.normal(0.0, sqrt(self.robot["laser"]["error_variance"]), 1)
                    dist += error
                    scans.append(float(dist))
            else:
                scans.append(float('nan'))
            
            angle += step

        return scans
    
    # TODO: Make sure collision is detected properly
    def raycast(self, range_max, angle, mapX, mapY):
        # Robot position
        posX = self.x+cos(self.theta)*self.robot["body"]["radius"]*0.5
        posY = self.y+sin(self.theta)*self.robot["body"]["radius"]*0.5

        # ray dir
        rayDirX = cos(angle)
        rayDirY = sin(angle)

        #length of ray from current position to next x or y-side
        sideDistX = 0
        sideDistY = 0

        #length of ray from one x or y-side to next x or y-side
        deltaDistX =  1e30 if rayDirX == 0 else sqrt(1 + (rayDirY / rayDirX) * (rayDirY / rayDirX))
        deltaDistY =  1e30 if rayDirY == 0 else sqrt(1 + (rayDirX / rayDirY) * (rayDirX / rayDirY))
        perpWallDist = 0

        #that direction to step in x or y-direction (either +1 or -1)
        stepX = 0
        stepY = 0

        hit = 0  #was there a wall hit?
        dist = 0

        #calculate step and initial sideDist
        if rayDirX < 0:
            stepX = -1
            sideDistX = (posX - mapX*self.map.info.resolution)/self.map.info.resolution * deltaDistX
        else:
            stepX = 1
            sideDistX = (mapX*self.map.info.resolution + self.map.info.resolution - posX)/self.map.info.resolution * deltaDistX
        
        if rayDirY < 0:
            stepY = -1
            sideDistY = (posY - mapY*self.map.info.resolution)/self.map.info.resolution * deltaDistY
        else:
            stepY = 1
            sideDistY = (mapY*self.map.info.resolution + self.map.info.resolution - posY)/self.map.info.resolution * deltaDistY

        #perform DDA
        while hit == 0 and dist < range_max:
            #jump to next map square, either in x-direction, or in y-direction
            if sideDistX < sideDistY:
                mapX += stepX
                dist = sideDistX
                sideDistX += deltaDistX
            else:
                mapY += stepY
                dist = sideDistY
                sideDistY += deltaDistY
            #Check if ray has hit a wall
            #self.get_logger().info("Check map: {} {}".format(mapX, mapY))
            if self.map.data[mapX + mapY * self.map.info.width] > 0: 
                hit = 1

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
