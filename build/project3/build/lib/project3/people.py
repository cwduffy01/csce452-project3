import rclpy
from rclpy.node import Node

# rosdep install -i --from-path src --ignore-src -r -y --rosdistro humble
# . install/setup.bash
# colcon build --packages-select project3
# ros2 run project3 track
import numpy as np
import math

from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32
from example_interfaces.msg import Int64
# from std_msgs.msg import *
# from builtin_interfaces.msg import *

class Person:
    pos = []
    prev = []
    count = 0
    active = True
    count_absent = 0

    def __init__(self, pos):
        self.pos = pos
        self.prev = pos

    def check_points(self, points):
        threshold = 0.35
        min_index = None
        min_distance = 10000
        for i, point in enumerate(points):
            dist = math.sqrt((point[0] - self.pos[0])**2 + (point[1] - self.pos[1])**2)
            if dist > threshold:
                continue

            if dist < min_distance:
                min_index = i
                min_distance = dist

        if min_index is not None:
            self.prev = self.pos.copy()
            self.pos = points[min_index]
            points.pop(min_index)
            self.count += 1
            self.count_absent = 0
            return self.pos

        self.count_absent += 1
        prev_copy = self.prev.copy()
        self.prev = self.pos.copy()
        self.pos[0] += self.pos[0] - prev_copy[0]
        self.pos[1] += self.pos[1] - prev_copy[1]

        if self.count_absent > 7:
            self.active = False
        return self.pos

            


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

        # self.test_publish = self.create_publisher(PointCloud, '/test_publish', 10)


    def callback(self, msg):
        clusters = self.jump_cluster(msg)

        centers = []
        for cluster in clusters:
            if len(cluster) < 7: # change maybe (add to jump_cluster)
                continue
            sumx = 0
            sumy = 0
            for point in cluster:
                sumx += point.x
                sumy += point.y
            center = [sumx / len(cluster), sumy / len(cluster)]
            centers.append(center)

        people_pc = PointCloud()
        people_pc.points = []
        for center in centers:
            p = Point32()
            p.x = center[0]
            p.y = center[1]
            p.z = 0.0
            people_pc.points.append(p)

        people_pc.header = msg.header
        self.person_locations.publish(people_pc)

        current_people_int = Int64()
        current_people_int.data = len(centers)
        self.people_count_current.publish(current_people_int)

        # print('current:', len(centers))

        total_people = 0
        for person in self.people:
            if person.active:
                test = person.check_points(centers)

                # print(test)
                # test_pc = PointCloud()
                # test_pc.points = []

                # p = Point32()
                # p.x = test[0]
                # p.y = test[1]
                # p.z = 0.0
                # test_pc.points.append(p)

                # test_pc.header = msg.header
                # self.test_publish.publish(test_pc)

            if person.count > 10:
                total_people += 1

        for center in centers:
            p = Person(center)
            self.people.append(p)

        total_people_int = Int64()
        total_people_int.data = total_people
        self.people_count_total.publish(total_people_int)
        # print('total:',total_people)
        
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
        max_delta = 0.4

        if len(pc.points) == 0:
            return []

        base_point = pc.points[0]

        for i, point in enumerate(pc.points):
            x = point.x
            y = point.y
            distance = math.sqrt((x - base_point.x)**2 + (y - base_point.y)**2)
            
            if (distance > max_delta):
                # print()
                # print(f"({x}, {y})", end=",")
                if len(current_blob) > 0:
                    blobs.append(current_blob)
                    current_blob = []
                    current_blob.append(point)
                base_point = point
            else:
                # print(f"({x}, {y})", end=",")
                base_point.x = (base_point.x + x)/2
                base_point.y = (base_point.y + y)/2
                current_blob.append(point)
        blobs.append(current_blob)
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