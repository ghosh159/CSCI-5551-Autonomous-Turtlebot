#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
from tf.transformations import euler_from_quaternion

class PathFollower:
    def __init__(self):
        self.path_subscriber = rospy.Subscriber('path_topic', Path, self.path_callback)
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.current_path = []
        self.goal_index = 0
        self.current_pose = PoseStamped()
        rospy.on_shutdown(self.stop_robot)

    def odom_callback(self, data):
        self.current_pose = data.pose.pose

    def path_callback(self, msg):
        self.current_path = msg.poses
        self.goal_index = 0
    def stop_robot(self):
        stop_twist = Twist()
        self.cmd_vel_publisher.publish(stop_twist)

    def follow_path(self):
        if self.goal_index < len(self.current_path):
            goal_pose = self.current_path[self.goal_index].pose
            goal_position = goal_pose.position

            current_position = self.current_pose.position
            current_quaternion = (
                self.current_pose.orientation.x,
                self.current_pose.orientation.y,
                self.current_pose.orientation.z,
                self.current_pose.orientation.w
            )
            _, _, current_yaw = euler_from_quaternion(current_quaternion)

            angle_to_goal = math.atan2(
                goal_position.y - current_position.y,
                goal_position.x - current_position.x
            )
            angle_difference = self.normalize_angle(angle_to_goal - current_yaw)

            max_angular_speed = 1.0
            angular_speed = max(min(angle_difference * 2.0, max_angular_speed), -max_angular_speed)

            if abs(angle_difference) > math.radians(45):
                linear_speed = 0
            else:
                linear_speed = 0.2

            twist = Twist()
            twist.linear.x = linear_speed
            twist.angular.z = angular_speed
            self.cmd_vel_publisher.publish(twist)

            distance_to_goal = math.sqrt(
                (goal_position.x - current_position.x) ** 2 +
                (goal_position.y - current_position.y) ** 2
            )
            if distance_to_goal < 0.1:
                self.goal_index += 1

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.follow_path()
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('path_follower')
    follower = PathFollower()
    follower.run()
