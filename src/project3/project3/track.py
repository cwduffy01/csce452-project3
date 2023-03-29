import rclpy
from rclpy.node import Node

# rosdep install -i --from-path src --ignore-src -r -y --rosdistro humble
# colcon build --packages-select project3
# . install/setup.bash
# ros2 run project3 track
import numpy as np
import cv2
import matplotlib.pyplot as plt

from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import *
from builtin_interfaces.msg import *

class Track(Node):

    background = np.zeros(512)
    frame = 0
    threshold = 0.5

    def __init__(self):
        super().__init__('tracking')
        self.bag = self.create_subscription(LaserScan, '/scan', self.bag_callback, 10)
        self.publisher_ = self.create_publisher(PointCloud, '/person_locations', 10)

    def bag_callback(self, msg):

        ranges = np.array(msg.ranges)   # numpy array of lidar ranges
        angles = (np.arange(0, len(msg.ranges)) * msg.angle_increment) + msg.angle_min  # angles corresponding to each range

        # trig to convert ranges and angles to x and y coords
        x_values = np.cos(angles) * ranges
        y_values = np.sin(angles) * ranges
        z_values = np.zeros(len(angles))

        pc = PointCloud()
        pc.points = []
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
        
        print("message received")
        print(z_values)
        print(x_values)
        print(y_values)
        print(pts)
        exit()
        return

        data = np.array(msg.ranges)
        data[data == np.inf] = 100

        # if first frame, save as background
        if self.frame == 0:
            data[np.isnan(data)] = 100
            self.background = data
        else:
            data[np.isnan(data)] = self.background[np.isnan(data)] # replace faulty points with background

        

        
        # backgound is the average of first 5 frames (might need to increase if the data is iffy)
        if self.frame < 5:
            self.background = (self.background * self.frame + self.background) / (self.frame + 1)

        # subtract background
        separated = data - self.background

        if(len(separated[abs(separated) > self.threshold])):
            print('frame', self.frame)
            print('data\n', data)
            print('background\n', self.background)
            print('separated\n', separated)
            print()

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
