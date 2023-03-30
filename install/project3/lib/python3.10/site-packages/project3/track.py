import rclpy
from rclpy.node import Node

# rosdep install -i --from-path src --ignore-src -r -y --rosdistro humble
# . install/setup.bash
# colcon build --packages-select project3
# ros2 run project3 track
import numpy as np
import cv2
import matplotlib.pyplot as plt

from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import *
from builtin_interfaces.msg import *

np.set_printoptions(threshold=np.inf)


class Track(Node):

    background = np.zeros(512)
    frame = 0
    threshold = 0.5
    index_counts = {}

    def __init__(self):
        super().__init__('tracking')
        self.bag = self.create_subscription(LaserScan, '/scan', self.bag_callback, 10)
        self.publisher_ = self.create_publisher(PointCloud, '/person_locations', 10)
        self.people_points = self.create_publisher(PointCloud, '/people_points', 10)

    def bag_callback(self, msg):

        print('frame', self.frame)

        ranges = np.array(msg.ranges)   # numpy array of lidar ranges
        angles = (np.arange(0, len(msg.ranges)) * msg.angle_increment) + msg.angle_min  # angles corresponding to each range

        # trig to convert ranges and angles to x and y coords
        x_values = np.cos(angles) * ranges
        y_values = np.sin(angles) * ranges
        z_values = np.zeros(len(angles))

        # push transformed points to publisher_ point cloud (for testing)
        pc = PointCloud()
        pc.points = []
        # keep put points into an array
        pts = []

        for i in range(len(x_values)):
            p = Point32()
            p.x = x_values[i]
            p.y = y_values[i]
            p.z = z_values[i]
            pc.points.append(p)

            pts.append([x_values[i], y_values[i]])

        pc.header = msg.header
        self.publisher_.publish(pc)
        
        # convert points to matrix
        pts = np.matrix(pts)
        # replace inf with 100 so theres no funny business with math
        pts[pts == np.inf] = 100 
        pts[pts == -np.inf] = 100

        if(np.all(pts == 100) or np.all(np.isnan(pts))):
            print('bad read')
            return

        # replace nan with neighboring value
        if(np.any(np.isnan(pts[0]))):
            pts[0] = pts[1]
        for i in range(1, len(pts), 1):
            if np.any(np.isnan(pts[i])):
                pts[i,:] = pts[i-1,:]


        # if first frame, save as background
        if self.frame == 0:
            self.background = pts
        
        # backgound is the min of first 5 frames (might need to increase if the data is iffy)
        if self.frame < 5:
            # self.background = (self.background * self.frame + pts) / (self.frame + 1)
            for i, row in enumerate(self.background):
                if np.sum(self.background[i]) > np.sum(pts[i]):
                    self.background[i] = pts[i]
            

        # subtract background
        separated = pts - self.background
        separated = np.where(separated < -0.5, separated, 0) # replace everything over threshold with 0
        possible_people_points = []

        # if a row is not 0, add to possible people points
        for i, row in enumerate(separated):
            if np.any(row != 0):
                possible_people_points.append(pts[i])
                    
        print(possible_people_points)

        print('[')
        for row in possible_people_points:
            print(f'[{row[0,0]}, {row[0,1]}],')
        print(']')

        if self.frame == 20:
            exit()

        # convert to point could and publish to people_points
        people_pc = PointCloud()
        people_pc.points = []
        for i, pt in enumerate(possible_people_points):
            p = Point32()
            p.x = pt[0,0]
            p.y = pt[0,1]
            p.z = 0.0
            people_pc.points.append(p)

        people_pc.header = msg.header
        self.people_points.publish(people_pc)

        self.frame += 1
        

def main(args=None):
    print("Hello from track.py")

    rclpy.init(args=args)

    tracking = Track()

    rclpy.spin(tracking)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    tracking.destroy_node()
    rclpy.shutdown()

if __name__ == "__init__":
    main()
