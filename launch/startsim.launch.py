import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import yaml
import json

from py_robotsim import disc_robot

# References: https://docs.ros.org/en/foxy/Tutorials/Intermediate/URDF/Using-URDF-with-Robot-State-Publisher.html

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    #robot_file   = LaunchConfiguration('robot_file', default='normal.robot')
    robot_file = 'robots/ideal.robot'
    world_file = 'worlds/brick.world'

    robot = disc_robot.load_disc_robot(robot_file)
    robots = json.dumps(robot)
    #print(robots)
    robot_desc  = disc_robot.disc_robot_urdf(robot)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        #DeclareLaunchArgument(
        #    'robot_file',
        #    default_value='normal.robot',
        #    description='Robot configuration file to load'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}]),
            #arguments=[urdf]),
        Node(
            package='py_robotsim',
            executable='simulator',
            parameters=[{'robot': robots}]),
        Node(
            package='py_robotsim',
            executable='translator',
            parameters=[]),
        Node(
            package='py_robotsim',
            executable='map',
            parameters=[{'filename': world_file}]),
    ])