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
        # inputs
        self.points = self.create_subscription(PointCloud, '/people_points', self.callback, 10)
        # outputs
        self.person_locations = self.create_publisher(PointCloud, '/person_locations', 10)
        self.people_count_current = self.create_publisher(Int64, '/people_count_current', 10)
        self.people_count_total = self.create_publisher(Int64, '/people_count_total', 10)

    def callback(self, msg):
        clusters = self.jump_cluster(msg)
        print(clusters)
        # create bounding boxes around points and count how many other points lay within boundary (set boundary)
        # if number of points in box > threshold, consider cluster
        # loop through people and check their centers against cluster
        # pick closest cluster to be same person (within some boundary)
            # could also keep track of a movement vector to be more smart about it but could be overkill
            # if no cluster near person, remove them from list
            # if cluster far from all other people, create new person
    
    def jump_cluster(self, pc):
        blobs = []
        current_blob = []
        base_point = pc.points[0]
        max_delta = 0.25
        for i, point in enumerate(pc.points):
            x = point.x
            y = point.y
            distance = math.sqrt((x - base_point.x)**2 + (y - base_point.y)**2)
            
            if (distance > max_delta):
                print()
                print(f"({x}, {y})", end=",")
                if len(current_blob) > 0:
                    blobs.append(current_blob)
                    current_blob = []
                    current_blob.append(point)
                base_point = point
            else:
                print(f"({x}, {y})", end=",")
                base_point.x = (base_point.x + x)/2
                base_point.y = (base_point.y + y)/2
                current_blob.append(point)
        return blobs

def main(args=None):
    print("Hello from people.py")

    rclpy.init(args=args)
    people = People()

    rclpy.spin(people)

    people.destroy_node()
    rclpy.shutdown()

if __name__ == "__init__":
    main()