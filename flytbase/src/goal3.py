#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion
from turtlesim.msg import Pose as TurtlePose
import math
import time
import random

class TurtlePatterns:
    def __init__(self):
        rospy.init_node('turtle_patterns', anonymous=True)
        rospy.Subscriber('/turtle1/pose', TurtlePose, self.pose_callback)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.real_pose_publisher = rospy.Publisher('/rt_real_pose', PoseStamped, queue_size=10)
        self.noisy_pose_publisher = rospy.Publisher('/rt_noisy_pose', PoseStamped, queue_size=10)

        self.pose = TurtlePose()
        self.rate = rospy.Rate(10)

        self.max_linear_speed = 1.0  
        self.angular_velocity = 1.0  

    def pose_callback(self, data):
        self.pose = data

    def move(self, linear_velocity, angular_velocity):
        twist_msg = Twist()
        twist_msg.linear.x = linear_velocity
        twist_msg.angular.z = angular_velocity
        self.velocity_publisher.publish(twist_msg)

    def circle_pattern(self):
        linear_velocity = self.max_linear_speed

        circle_time = 2 * math.pi / self.angular_velocity

        start_time = rospy.Time.now().to_sec()

        while (rospy.Time.now().to_sec() - start_time) < circle_time:
            self.move(linear_velocity, self.angular_velocity)

        self.move(0, 0)

    def add_noise(self, pose, std_dev):
        noisy_pose = PoseStamped()
        noisy_pose.header.stamp = rospy.Time.now()
        noisy_pose.header.frame_id = 'map'

        noisy_pose.pose.position.x = pose.x + random.gauss(0, std_dev)
        noisy_pose.pose.position.y = pose.y + random.gauss(0, std_dev)
        noisy_pose.pose.position.z = 0.0  

        noisy_pose.pose.orientation = Quaternion(0.0, 0.0, math.sin(pose.theta / 2), math.cos(pose.theta / 2))

        return noisy_pose

    def run(self):

        while not rospy.is_shutdown():
            self.circle_pattern()
            real_pose_msg = PoseStamped()
            real_pose_msg.header.stamp = rospy.Time.now()
            real_pose_msg.header.frame_id = 'map'
            real_pose_msg.pose.position = Point(self.pose.x, self.pose.y, 0.0)  
            real_pose_msg.pose.orientation = Quaternion(0.0, 0.0, math.sin(self.pose.theta / 2), math.cos(self.pose.theta / 2))

            self.real_pose_publisher.publish(real_pose_msg)
            print("Published to /rt_real_pose")

            noisy_pose_msg = self.add_noise(self.pose, std_dev=10)
            self.noisy_pose_publisher.publish(noisy_pose_msg)
            print("Published to /rt_noisy_pose")

            self.rate.sleep()

if __name__ == '__main__':
    try:
        turtle_patterns = TurtlePatterns()
        turtle_patterns.run()
    except rospy.ROSInterruptException:
        pass
