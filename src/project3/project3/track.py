import rclpy
from rclpy.node import Node

# rosdep install -i --from-path src --ignore-src -r -y --rosdistro humble
import numpy as np
import opencv as cv2

from sensor_msgs.msg import LaserScan 

class Track(Node):

    def __init__(self):
        super().__init__('tracking')
        self.bag = self.create_subscription(LaserScan, '/scan', self.bag_callback, 10)

    def bag_callback(self, msg):
        screen = msg.ranges
        cv2.imshow(screen)
        cv2.waitkey()

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
