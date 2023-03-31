import rclpy
from rclpy.node import Node

import numpy as np

from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import *
from builtin_interfaces.msg import *

# rosdep install -i --from-path src --ignore-src -r -y --rosdistro humble
# . install/setup.bash
# colcon build --packages-select project3
# ros2 run project3 track


class Track(Node):

    background = np.zeros(512) # background is initialized to 0
    frame = 0 # frame count

    def __init__(self):
        super().__init__('tracking')
        self.bag = self.create_subscription(LaserScan, '/scan', self.bag_callback, 10) # used for reading bag file
        self.people_points = self.create_publisher(PointCloud, '/cartesian_points', 10) # used to publish points for other node to read

    def bag_callback(self, msg):
        ranges = np.array(msg.ranges)   # numpy array of lidar ranges
        angles = (np.arange(0, len(msg.ranges)) * msg.angle_increment) + msg.angle_min  # angles corresponding to each range

        # trig to convert ranges and angles to x and y coords
        x_values = np.cos(angles) * ranges
        y_values = np.sin(angles) * ranges
        z_values = np.zeros(len(angles))

        # put points into an array
        pts = []
        for i in range(len(x_values)):
            pts.append([x_values[i], y_values[i]])

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
        
        # backgound is the min of first 5 frames (helps with sensor data being inconsistent)
        if self.frame < 5:
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

        # convert to point cloud and publish to people_points
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
    # print("Hello from track.py")

    rclpy.init(args=args)
    tracking = Track()

    rclpy.spin(tracking)

    tracking.destroy_node()
    rclpy.shutdown()

if __name__ == "__init__":
    main()