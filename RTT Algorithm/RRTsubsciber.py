#!/usr/bin/env python3

import rospy
from RRT import RRT

from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Point

class RRTSubscriber:
    def __init__(self, rrt_algorithm):
        self.rrt_algorithm = rrt_algorithm
        self.map_data = None
        self.current_position = None

        # Initialize ROS subscribers
        self.map_subscriber = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.odom_subscriber = rospy.Subscriber("/odom", Odometry, self.odom_callback)

    def map_callback(self, data):
        # Process and update the occupancy grid data
        self.map_data = data
        self.rrt_algorithm.update_map(self.map_data)

    def odom_callback(self, data):
        # Update the robot's current position
        pose = data.pose.pose
        self.current_position = (pose.position.x, pose.position.y)
        self.rrt_algorithm.update_start(self.current_position)

    def plan_path(self):
        if self.map_data is not None and self.current_position is not None:
            return self.rrt_algorithm.planning()
        else:
            return None

def main():
    rospy.init_node('rrt_subscriber_node')

    # Retrieve goal coordinates from command-line arguments
    goal_x = rospy.get_param('~goal_x', 0.0)
    goal_y = rospy.get_param('~goal_y', 0.0)

    # Initialize the RRT algorithm with placeholder values
    rrt_algorithm = RRT(start=[0, 0], goal=[goal_x, goal_y], map_data=None)

    rrt_subscriber = RRTSubscriber(rrt_algorithm)

    rate = rospy.Rate(0.5)
    path = rrt_subscriber.plan_path()
    print("here")
    while not rospy.is_shutdown():

        path = rrt_subscriber.plan_path()
        print(path)
        # if path:
        #     # Process the path

        #     pass
        rate.sleep()

if __name__ == "__main__":
    main()
