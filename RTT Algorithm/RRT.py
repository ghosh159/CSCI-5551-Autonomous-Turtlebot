#!/usr/bin/env python3

import rospy
import random
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class RRT:
    class Node:
        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.parent = None

    def __init__(self, start, goal, map_data, expand_dis=3.0, path_resolution=0.5, max_iter=500):
        print("Initializing RRT...")
        self.start = self.Node(start[0], start[1])
        self.goal = self.Node(goal[0], goal[1])
        self.map_data = map_data
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.max_iter = max_iter
        self.node_list = [self.start]
        self.path_publisher = rospy.Publisher("path_topic", Path, queue_size=10)
        print("RRT initialized with start:", start, "goal:", goal)

    def update_start(self, start):
        # print("Updating start position to:", start)
        self.start = self.Node(start[0], start[1])
        self.node_list = [self.start]  # Reset the node list

    def update_map(self, map_data):
        print("Updating map data...")
        self.map_data = map_data

    def planning(self):
        print("Starting RRT planning...")
        for i in range(self.max_iter):
            rnd_node = self.get_random_node()
            nearest_node = self.get_nearest_node_index(rnd_node)
            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)

            if self.check_collision(new_node):
                self.node_list.append(new_node)
                if self.is_in_goal_region(new_node):
                    print("Goal reached. Generating path...")
                    return self.generate_final_course(len(self.node_list) - 1)
            if i % 10 == 0:
                print(f"Planning iteration {i}, no path found yet...")

        print("No path found after maximum iterations.")
        return None

   


    def get_random_node(self):
        rnd_x = random.uniform(self.start.x, self.goal.x)
        rnd_y = random.uniform(self.start.y, self.goal.y)
        rnd_node = self.Node(rnd_x, rnd_y)
        return rnd_node

    def steer(self, from_node, to_node, extend_length=float("inf")):
        d, theta = self.calc_distance_and_angle(from_node, to_node)
        extend_length = min(d, extend_length)
        new_node = self.Node(from_node.x, from_node.y)
        new_node.parent = from_node
        for _ in range(int(extend_length / self.path_resolution)):
            new_node.x += self.path_resolution * math.cos(theta)
            new_node.y += self.path_resolution * math.sin(theta)
        return new_node

    def is_in_goal_region(self, node):
        if self.calc_distance_and_angle(node, self.goal)[0] < self.expand_dis:
            return True
        return False

    def get_nearest_node_index(self, rnd_node):
        return min(self.node_list, key=lambda node: (node.x - rnd_node.x) ** 2 + (node.y - rnd_node.y) ** 2)

    def check_collision(self, node):
        map_width = self.map_data.info.width
        map_resolution = self.map_data.info.resolution
        map_origin = self.map_data.info.origin.position
        map_x = int((node.x - map_origin.x) / map_resolution)
        map_y = int((node.y - map_origin.y) / map_resolution)
        index = map_x + map_y * map_width
        if 0 <= index < len(self.map_data.data):
            # Check if the occupancy value at this index is greater than the threshold
            occupancy_value = self.map_data.data[index]
            if occupancy_value == -1:
                # Treat unknown cells as unoccupied or based on specific requirements
                return True
            else:
                # Use a threshold (e.g., 98) to determine if the cell is considered occupied
                return occupancy_value < 98
        else:
            # If the index is out of bounds, return False indicating a collision
            return False


    def calc_distance_and_angle(self, from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta

    def generate_final_course(self, goal_index):
        path = []
        node = self.node_list[goal_index]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([self.start.x, self.start.y])
        self.publish_path(path[::-1])

    def publish_path(self, path):
        ros_path = Path()
        ros_path.header.stamp = rospy.Time.now()
        ros_path.header.frame_id = "map"
        for point in path:
            pose = PoseStamped()
            pose.header.stamp = ros_path.header.stamp
            pose.header.frame_id = ros_path.header.frame_id
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.orientation.w = 1.0
            ros_path.poses.append(pose)
        self.path_publisher.publish(ros_path)

