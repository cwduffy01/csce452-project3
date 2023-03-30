import rclpy
from rclpy.node import Node

# rosdep install -i --from-path src --ignore-src -r -y --rosdistro humble
import numpy as np
import math
import matplotlib.pyplot as plt

# import opencv as cv2

from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import *
from builtin_interfaces.msg import *


class Track(Node):

    def __init__(self):
        super().__init__('tracking')
        self.bag = self.create_subscription(LaserScan, '/scan', self.bag_callback, 10)
        self.publisher_ = self.create_publisher(PointCloud, '/person_locations', 10)

    def bag_callback(self, msg):
        ranges = np.array(msg.ranges)   # numpy array of lidar ranges
        print(ranges)
        angles = (np.arange(0, len(msg.ranges)) * msg.angle_increment) + msg.angle_min  # angles corresponding to each range

        # trig to convert ranges and angles to x and y coords
        x_values = np.cos(angles) * ranges
        y_values = np.sin(angles) * ranges
        z_values = np.zeros(len(angles))

        pc = PointCloud()
        pc.points = []

        for i in range(len(x_values)):
            p = Point32()
            p.x = x_values[i]
            p.y = y_values[i]
            p.z = z_values[i]
            pc.points.append(p)

        pc.header = msg.header

        self.publisher_.publish(pc)
        
        self.jump_cluster(pc)

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
    print("Hello from track.py")

    rclpy.init(args=args)

    tracking = Track()
    points = [
[3.992346523322011, 0.5422256633896342],
[2.8125931599341523, 2.5179248274552872],
[2.773380033311774, 2.513645245312507],
[2.7365366750943982, 2.5110083485900234],
[2.7210779025532177, 2.527752153408653],
[2.7055166829612642, 2.5444007897821006],
[2.6956475041413595, 2.5664698843586646],
[2.681289268009691, 2.584349761697295],
[2.668244407404073, 2.603546663251962],
[2.6564877389413297, 2.6240862506223057],
[2.6403366403162885, 2.640336755729076],
[2.6240861345035316, 2.656487853643785],
[2.581198628362864, 2.6453412585320586],
[2.545487247557406, 2.640969243779987],
[2.529234646383932, 2.6565383448390802],
[2.512886820950251, 2.672007428725406],
[2.4978056523138736, 2.6888412900850858],
[2.4900494240337343, 2.713695545220144],
[2.493498375168437, 2.7511516301569907],
[2.540602647846219, 2.8379251755460775],
[2.6437012562190194, 2.989803365684591],
[2.325214470266239, 2.942087297696058],
[2.281893963633499, 2.923977044943807],
[2.259026676310206, 2.931586480420173],
[2.226464052013183, 2.926292391701903],
[2.186844545854647, 2.911115279129993],
[2.0897130256759997, 2.817651991941932],
[2.0458008094731186, 2.7941131790745937],
[2.0274462907873145, 2.804992388639362],
[2.0101970144643406, 2.817379762366203],
[1.992872055355728, 2.829661063331477],
[1.9908831169754424, 2.8640055567496607],
[2.0281484834887564, 2.9561527366555427],
[2.2613101467006778, 4.4246396526329645],
[2.2224285947477016, 4.415207646004851],
[2.1952955525833153, 4.428761096841853],
[2.1680798587775603, 4.4421478073174265],
[2.1368847698673816, 4.447255291849408]
]

    blobs = []
    current_blob = []
    base_point = points[0]
    max_delta = 0.25
    for i, point in enumerate(points):
        x = point[0]
        y = point[1]

        if x == np.inf or x == -np.inf or np.isnan(x):
            continue
        if y == np.inf or y == -np.inf or np.isnan(y):
            continue

        distance = math.sqrt((x - base_point[0])**2 + (y - base_point[1])**2)
        
        if (distance > max_delta):
            print()
            print(f"({x}, {y})", end=",")
            if len(current_blob) > 0:
                blobs.append(current_blob)
                current_blob = []
                current_blob.append((x, y))
            base_point = point
        else:
            print(f"({x}, {y})", end=",")
            base_point[0] = (base_point[0] + x)/2
            base_point[1] = (base_point[1] + y)/2
            current_blob.append((x, y))
    print("\n\n\n")
    print(blobs)


    rclpy.spin(tracking)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    tracking.destroy_node()
    rclpy.shutdown()

if __name__ == "__init__":
    main()
