#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import sqrt, atan2, pi

velocity_publisher = None
pose_subscriber = None
turtlesim_pose = Pose()

x_min = 0.0
y_min = 0.0
x_max = 11.0
y_max = 11.0

def move(speed, distance, is_forward):
    vel_msg = Twist()
    vel_msg.linear.x = speed if is_forward else -speed
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    t0 = rospy.Time.now().to_sec()
    current_distance = 0.0
    loop_rate = rospy.Rate(100)
    while current_distance < distance:
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_distance = speed * (t1 - t0)
        loop_rate.sleep()

    vel_msg.linear.x = 0
    velocity_publisher.publish(vel_msg)

def rotate(angular_speed, angle, clockwise):
    vel_msg = Twist()
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = -angular_speed if clockwise else angular_speed

    t0 = rospy.Time.now().to_sec()
    current_angle = 0.0
    loop_rate = rospy.Rate(1000)
    while current_angle < angle:
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed * (t1 - t0)
        loop_rate.sleep()

    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)

def degrees2radians(angle_in_degrees):
    return angle_in_degrees * pi / 180.0

def set_desired_orientation(desired_angle_radians):
    relative_angle_radians = desired_angle_radians - turtlesim_pose.theta
    clockwise = relative_angle_radians < 0
    rotate(abs(relative_angle_radians), abs(relative_angle_radians), clockwise)
    return 0.0

def pose_callback(pose_message):
    global turtlesim_pose
    turtlesim_pose = pose_message

def move_goal(goal_pose, distance_tolerance):
    vel_msg = Twist()
    loop_rate = rospy.Rate(10)
    while get_distance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y) > distance_tolerance:
        vel_msg.linear.x = 1.5 * get_distance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y)
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 10 * (atan2(goal_pose.y - turtlesim_pose.y, goal_pose.x - turtlesim_pose.x) - turtlesim_pose.theta)

        velocity_publisher.publish(vel_msg)

        loop_rate.sleep()

    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)

def get_distance(x1, y1, x2, y2):
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def grid_clean():
    loop_rate = rospy.Rate(0.5)
    goal_pose = Pose()
    goal_pose.x = 1
    goal_pose.y = 1
    goal_pose.theta = 0
    move_goal(goal_pose, 0.01)
    loop_rate.sleep()
    set_desired_orientation(0)
    loop_rate.sleep()

    move(3, 9, True)
    loop_rate.sleep()
    rotate(degrees2radians(10), degrees2radians(90), False)
    loop_rate.sleep()
    move(3, 1, True)

    rotate(degrees2radians(10), degrees2radians(90), False)
    loop_rate.sleep()
    move(3, 9, True)
    rotate(degrees2radians(10), degrees2radians(270), False)
    loop_rate.sleep()
    move(3, 1, True)

    rotate(degrees2radians(30), degrees2radians(90), True)
    loop_rate.sleep()
    move(3, 9, True)
    rotate(degrees2radians(30), degrees2radians(270), True)
    loop_rate.sleep()
    move(3, 1, True)

if __name__ == '__main__':
    rospy.init_node('turtlesim_cleaner')
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1000)
    pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
    loop_rate = rospy.Rate(0.5)
    grid_clean()
    rospy.spin()
