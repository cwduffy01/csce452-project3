import rclpy
from rclpy.node import Node

# rosdep install -i --from-path src --ignore-src -r -y --rosdistro humble
import numpy as np
# import opencv as cv2

from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32


class Track(Node):

    def __init__(self):
        super().__init__('tracking')
        self.bag = self.create_subscription(LaserScan, '/scan', self.bag_callback, 10)
        self.publisher_ = self.create_publisher(PointCloud, '/tracking', 10)

    def bag_callback(self, msg):
        ranges = np.array(msg.ranges)   # numpy array of lidar ranges
        angles = (np.arange(0, len(msg.ranges)) * msg.angle_increment) + msg.angle_min  # angles corresponding to each range

        # trig to convert ranges and angles to x and y coords
        # x_values = np.cos(angles) * ranges
        # y_values = np.sin(angles) * ranges
        x_values = np.ones(len(angles))
        y_values = np.ones(len(angles))
        z_values = np.zeros(len(angles))

        pc = PointCloud()
        pc.points = []

        for i in range(len(x_values)):
            p = Point32()
            p.x = x_values[i]
            p.y = y_values[i]
            p.z = z_values[i]
            pc.points.append(p)

        self.publisher_.publish(pc)
        
        print("message received")



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
