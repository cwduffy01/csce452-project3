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

# ros2 launch project3 launch.py bag_loc:=bags/example7

np.set_printoptions(threshold=np.inf)


class Track(Node):

    background = np.zeros(512)
    frame = 0
    threshold = 0.5
    index_counts = {}

    def __init__(self):
        super().__init__('tracking')
        self.bag = self.create_subscription(LaserScan, '/scan', self.bag_callback, 10)
        self.people_points = self.create_publisher(PointCloud, '/people_points', 10)
        self.boundary_topic = self.create_publisher(PointCloud, '/boundary', 10)

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
                    
        # print(possible_people_points)

        # print('[')
        # for row in possible_people_points:
        #     print(f'[{row[0,0]}, {row[0,1]}],')
        # print(']')

        # if self.frame == 20:
        #     exit()

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

        boundary_pc = PointCloud()
        boundary_pc.points = []
        for i in range(100):
            p_min = Point32()
            p_max = Point32()

            max_range = 5
            thresh = 0.2

            p_min.x = np.cos(msg.angle_min + thresh) * max_range * (i / 100)
            p_min.y = np.sin(msg.angle_min + thresh) * max_range * (i / 100)
            p_min.z = 0.0

            p_max.x = np.cos(msg.angle_max - thresh) * max_range * (i / 100)
            p_max.y = np.sin(msg.angle_max - thresh) * max_range * (i / 100)
            p_max.z = 0.0

            boundary_pc.points.append(p_min)
            boundary_pc.points.append(p_max)

        boundary_pc.header = msg.header
        self.boundary_topic.publish(boundary_pc)


        self.frame += 1
        

def main(args=None):
    print("Hello from track.py")

    rclpy.init(args=args)
    tracking = Track()

    rclpy.spin(tracking)

    tracking.destroy_node()
    rclpy.shutdown()

if __name__ == "__init__":
    main()
