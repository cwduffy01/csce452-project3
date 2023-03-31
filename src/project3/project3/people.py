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

class Person:
    pos = [] # current position of the person
    prev = [] # previous position of the person
    count = 0 # number of frames on screen
    active = True # if person is active
    count_absent = 0 # consecutive frames off screen

    def __init__(self, pos):
        self.pos = pos
        self.prev = pos

    def check_points(self, points):
        threshold = 0.35 # threshold that determines how close another cluster should be to be considered same person
        min_index = None # index of closest point
        min_distance = 10000 # distance to closest point
        for i, point in enumerate(points):
            # calculate distance to each point and continue if greater than threshold
            dist = math.sqrt((point[0] - self.pos[0])**2 + (point[1] - self.pos[1])**2)
            if dist > threshold:
                continue

            # find the minimum distance and closest point
            if dist < min_distance:
                min_index = i
                min_distance = dist

        # if there was a min index found (same person in new frame)
        if min_index is not None:
            self.prev = self.pos.copy() # previous frame = current frame
            self.pos = points[min_index] # current frame = new point
            points.pop(min_index) # remove point so it doesnt get considered for another person
            self.count += 1
            self.count_absent = 0
            return self.pos

        # if no point was found, increment absence and project point forward using previous point
        self.count_absent += 1
        prev_copy = self.prev.copy()
        self.prev = self.pos.copy()
        self.pos[0] += self.pos[0] - prev_copy[0]
        self.pos[1] += self.pos[1] - prev_copy[1]

        # if inactive for 7 consecutive frames, set inactive
        if self.count_absent > 7:
            self.active = False
        return self.pos

            


class People(Node):

    boundary_size = 0.5 # distance away from a point to be considered in the same cluster
    person_threshold = 7 # number of points in cluster to be considered a person

    people = []

    def __init__(self):
        super().__init__('people')
        # inputs
        self.points = self.create_subscription(PointCloud, '/cartesian_points', self.callback, 10)
        # outputs
        self.person_locations = self.create_publisher(PointCloud, '/person_locations', 10)
        self.people_count_current = self.create_publisher(Int64, '/people_count_current', 10)
        self.people_count_total = self.create_publisher(Int64, '/people_count_total', 10)

        # self.test_publish = self.create_publisher(PointCloud, '/test_publish', 10)


    def callback(self, msg):
        # find clusters of points
        clusters = self.jump_cluster(msg)

        # calculate centers of clusters
        centers = []
        for cluster in clusters:
            if len(cluster) < 7: # dont consider clusters smaller than 7 points
                continue
            sumx = 0
            sumy = 0
            for point in cluster:
                sumx += point.x
                sumy += point.y
            center = [sumx / len(cluster), sumy / len(cluster)]
            centers.append(center)

        # add centers to point cloud to be published to person_locations
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

        # publish current number of people with number of centers
        current_people_int = Int64()
        current_people_int.data = len(centers)
        self.people_count_current.publish(current_people_int)

        # loop through people and try to assign them to a cluster on the screen
        total_people = 0
        for person in self.people:
            if person.active:
                test = person.check_points(centers)
            # if person has been on screen for 10 frames, theyre considered in total people
            if person.count > 10:
                total_people += 1

        # assign the remainder of the clusters to a new person
        for center in centers:
            p = Person(center)
            self.people.append(p)

        # publish total number of people
        total_people_int = Int64()
        total_people_int.data = total_people
        self.people_count_total.publish(total_people_int)

    
    def jump_cluster(self, pc):
        
        # create the array to add the detected blobs
        blobs = []
        current_blob = []
        
        # max change in distance that points in a blob can have
        max_delta = 0.4
        
        if len(pc.points) == 0:
            return []
        
        # sets an original point to compare everything to
        base_point = pc.points[0]
        
        # loops through all points in order and if a jump in distance between a point and the average point of 
        # the blob being constructed occurs then it starts adding to a new blob
        for i, point in enumerate(pc.points):
            x = point.x
            y = point.y
            distance = math.sqrt((x - base_point.x)**2 + (y - base_point.y)**2)
            
            if (distance > max_delta):
                # jump detected
                if len(current_blob) > 0:
                    blobs.append(current_blob)
                    current_blob = []
                    current_blob.append(point)
                base_point = point
            else:
                # no jump detected
                base_point.x = (base_point.x + x)/2
                base_point.y = (base_point.y + y)/2
                current_blob.append(point)
        blobs.append(current_blob)
        return blobs

def main(args=None):
    # print("Hello from people.py")

    rclpy.init(args=args)
    people = People()

    rclpy.spin(people)

    people.destroy_node()
    rclpy.shutdown()

if __name__ == "__init__":
    main()
