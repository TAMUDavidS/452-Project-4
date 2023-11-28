import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from nav_msgs.msg import MapMetaData, OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion

from math import sin, cos

import time

import yaml

# Get width and height of map file by counting linebreaks
# TODO: Check for extra spaces and blank lines
def get_width_height(map):
    height = 1
    stripped = map["map"].strip()
    for i in stripped:
        if i == '\n':
            height += 1
    map["map"] = stripped.replace('\n','')
    map["width"] = int(len(map["map"])/height)
    map["height"] = height
    return map

# Load map file as yaml
def load_map(file_name):
    with open(file_name) as f:
        map = yaml.safe_load(f)
    map = get_width_height(map)
    return map

# Load and publish a map
class MapLoader(Node):
    def __init__(self):
        super().__init__("map_loader_node")

        # Get map file
        self.declare_parameter('filename', "")
        filename = self.get_parameter('filename').get_parameter_value().string_value
        self.map = load_map(filename)
        #self.get_logger().info("{}".format(self.map))

        # Map message data
        self.seq = 0
        self.load_time = self.get_clock().now().to_msg()

        # Publish grid
        self.publisher = self.create_publisher(OccupancyGrid,'/map',10)
        time.sleep(5)
        self.publisher.publish(self.get_occupancy_grid())

    def get_occupancy_grid(self):
        grid = OccupancyGrid()
        grid.header = self.get_header()
        grid.info = self.get_meta_data()
        grid.data = self.get_data()
        return grid
    
    def get_header(self):
        header = Header()
        #header.seq = self.seq
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'world'
        self.seq += 1

        return header
    
    def get_meta_data(self):
        metadata = MapMetaData()
        metadata.map_load_time = self.load_time
        metadata.resolution = self.map["resolution"]
        metadata.width = self.map["width"]
        metadata.height = self.map["height"]

        origin = Pose()
        point = Point()
        #point.x = -self.map["width"]*self.map["resolution"]
        #point.y = -self.map["height"]*self.map["resolution"]
        origin.position = point
        metadata.origin = origin

        return metadata
    
    def get_data(self):
        data = []
        for i in self.map["map"]:
            if i == '#':
                data.append(100)
            else:
                data.append(0)
        
        return data

# Helper function for broadcasting input
def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main():
    rclpy.init()

    map_loader = MapLoader()

    try:
        rclpy.spin_once(map_loader)
    except KeyboardInterrupt:
        pass

    map_loader.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()