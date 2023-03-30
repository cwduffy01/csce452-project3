import rclpy
from rclpy.node import Node

# rosdep install -i --from-path src --ignore-src -r -y --rosdistro humble
# . install/setup.bash
# colcon build --packages-select project3
# ros2 run project3 track
import numpy as np

from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32
from example_msgs.msg import Int64
from std_msgs.msg import *
from builtin_interfaces.msg import *

class person:
    center = []
    id = 0

    def __init__(self, points):
        self.points = points

class People(Node):

    boundary_size = 0.5 # distance away from a point to be considered in the same cluster
    person_threshold = 7 # number of points in cluster to be considered a person

    people_counter = 0
    people = []

    def __init__(self):
        super().__init__('people')
        self.points = self.create_subscription(LaserScan, '/people_points', self.callback, 10)
        self.person_locations = self.create_publisher(PointCloud, '/person_locations', 10)
        self.people_count_current = self.create_publisher(Int64, '/people_count_current', 10)
        self.people_count_total = self.create_publisher(Int64, '/people_count_total', 10)

    def callback(self, msg):
        # create bounding boxes around points and count how many other points lay within boundary (set boundary)
        # if number of points in box > threshold, consider cluster
        # loop through people and check their centers against cluster
        # pick closest cluster to be same person (within some boundary)
            # could also keep track of a movement vector to be more smart about it but could be overkill
            # if no cluster near person, remove them from list
            # if cluster far from all other people, create new person
    
