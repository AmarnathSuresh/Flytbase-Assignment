#!/usr/bin/env python3

import rospy
import math
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class TurtleBotController:
    def __init__(self, node, Kp, Ki, Kd):
        self.goal_x = 8.0
        self.goal_y = 3.0
        self.turtle_x = 5.0
        self.turtle_y = 5.0
        self.turtle_theta = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0.0
        self.integral = 0.0

        self.time = []
        self.position_error = []
        self.linear_velocities = []
        self.angular_velocities = []

        self.node = node  
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)  

    def pose_callback(self, data):
        self.turtle_x = data.x
        self.turtle_y = data.y
        self.turtle_theta = data.theta

    def update_error(self):
        distance = math.sqrt((self.goal_x - self.turtle_x) ** 2 + (self.goal_y - self.turtle_y) ** 2)
        angle_to_goal = math.atan2(self.goal_y - self.turtle_y, self.goal_x - self.turtle_x)
        error = angle_to_goal - self.turtle_theta
        return distance, error

    def pid_control(self):
        distance, error = self.update_error()

        self.integral += error
        derivative = error - self.prev_error

        self.linear_velocity = self.Kp * distance
        self.angular_velocity = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        self.prev_error = error

    def run(self):
        while not rospy.is_shutdown():
            self.pid_control()

            self.time.append(rospy.Time.now().to_sec())
            self.position_error.append(self.linear_velocity)
            self.linear_velocities.append(self.linear_velocity)
            self.angular_velocities.append(self.angular_velocity)

            twist_msg = Twist()
            twist_msg.linear.x = self.linear_velocity
            twist_msg.angular.z = self.angular_velocity
            self.velocity_publisher.publish(twist_msg)

            self.rate.sleep()

def plot_performance(gains_list, time_list, position_error_list, linear_velocities_list, angular_velocities_list):
    plt.figure(figsize=(12, 6))

    plt.subplot(2, 1, 1)
    for i, gains in enumerate(gains_list):
        plt.plot(time_list[i], position_error_list[i], label='Kp={}, Ki={}, Kd={}'.format(*gains))
    plt.xlabel('Time (s)')
    plt.ylabel('Position Error')
    plt.grid(True)
    plt.legend()

    plt.subplot(2, 1, 2)
    for i, gains in enumerate(gains_list):
        plt.plot(time_list[i], linear_velocities_list[i], label='Kp={}, Ki={}, Kd={}'.format(*gains))
    plt.xlabel('Time (s)')
    plt.ylabel('Velocities')
    plt.grid(True)
    plt.legend()

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    try:
        rospy.init_node('turtle_pid_controller', anonymous=True)  
        gains_list = [(1.0, 0.1, 0.05), (1.5, 0.2, 0.1), (0.8, 0.05, 0.02)]  
        controller_list = []
        time_list = []
        position_error_list = []
        linear_velocities_list = []
        angular_velocities_list = []

        for gains in gains_list:
            controller = TurtleBotController(rospy, *gains) 
            controller_list.append(controller)

        for controller in controller_list:
            time_data = []
            position_error_data = []
            linear_velocities_data = []
            angular_velocities_data = []

            while not rospy.is_shutdown():
                controller.pid_control()

                time_data.append(rospy.Time.now().to_sec())
                position_error_data.append(controller.linear_velocity)
                linear_velocities_data.append(controller.linear_velocity)
                angular_velocities_data.append(controller.angular_velocity)

                twist_msg = Twist()
                twist_msg.linear.x = controller.linear_velocity
                twist_msg.angular.z = controller.angular_velocity
                controller.velocity_publisher.publish(twist_msg)

                controller.rate.sleep()

                if math.sqrt((controller.goal_x - controller.turtle_x) ** 2 + (controller.goal_y - controller.turtle_y) ** 2) < 0.1:
                    break

            time_list.append(time_data)
            position_error_list.append(position_error_data)
            linear_velocities_list.append(linear_velocities_data)
            angular_velocities_list.append(angular_velocities_data)

        plot_performance(gains_list, time_list, position_error_list, linear_velocities_list, angular_velocities_list)

    except rospy.ROSInterruptException:
        pass

