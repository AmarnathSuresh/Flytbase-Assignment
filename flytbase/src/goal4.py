#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Pose2D
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
import random
import math

robber_speed = 1.0
circle_radius = 3.0
noise_std_dev = 10.0
police_speed = 1.5
police_accel_limit = 0.1
police_decel_limit = 0.1

current_pose = Pose2D()
police_pose = Pose2D()

def pose_callback(msg):
    global current_pose
    current_pose.x = msg.x
    current_pose.y = msg.y
    current_pose.theta = msg.theta

def police_pose_callback(msg):
    global police_pose
    police_pose.x = msg.x
    police_pose.y = msg.y
    police_pose.theta = msg.theta

def move_turtle_in_circle(pub):
    cmd_vel = Twist()
    cmd_vel.linear.x = robber_speed
    cmd_vel.angular.z = robber_speed / circle_radius
    pub.publish(cmd_vel)

def move_police_turtle(pub, target_pose):
    cmd_vel = Twist()
    dx = target_pose.x - police_pose.x
    dy = target_pose.y - police_pose.y
    distance = math.sqrt(dx * dx + dy * dy)

    if distance <= 3.0:
        rospy.loginfo("Chase complete!")
        rospy.signal_shutdown("Chase complete!")
        return

    target_angle = math.atan2(dy, dx)
    angle_diff = target_angle - police_pose.theta

    cmd_vel.linear.x = min(police_speed, distance)
    cmd_vel.angular.z = angle_diff

    pub.publish(cmd_vel)

if __name__ == '__main__':
    rospy.init_node('turtle_chase_node')

    rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
    rospy.Subscriber('/turtle2/pose', Pose, police_pose_callback)
    cmd_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    police_cmd_vel_pub = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)

    spawn_client = rospy.ServiceProxy('/spawn', Spawn)

    rospy.wait_for_service('/spawn')

    x_pos = random.random() * 11
    y_pos = random.random() * 11
    theta_val = 0
    turtle_name = "turtle2"

    spawn_client(x_pos, y_pos, theta_val, turtle_name)

    rate = rospy.Rate(2)

    police_start_time = rospy.Time.now() + rospy.Duration(10)

    while not rospy.is_shutdown():
        move_turtle_in_circle(cmd_vel_pub)

        if rospy.Time.now() >= police_start_time:
            move_police_turtle(police_cmd_vel_pub, current_pose)

        rate.sleep()
