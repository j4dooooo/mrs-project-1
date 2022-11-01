#!/usr/bin/env python3

import behaviors

import rospy
import tf.transformations
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class RobotCommunicator:

    def __init__(self):

        # set general parameters
        self.number_of_robots = rospy.get_param("/num_of_robots")
        self.odom = np.zeros((self.number_of_robots, 3))
        self.vel = np.zeros((self.number_of_robots, 3))
        self.coeff_sep = 1
        self.coeff_coh = 3
        self.coeff_ali = 2

        # change dt at a later time
        self.dt = 0.01

        # set maximum values
        self.max_acc = 1
        self.max_vel = 2

        rospy.init_node("robot_communicator")

        self.pub = [None] * self.number_of_robots

        # create publishers and subscribers
        for robot_number in range(self.number_of_robots):
            rospy.Subscriber("/robot_{}/odom".format(robot_number), Odometry, self.callback, (robot_number))
            self.pub[robot_number] = rospy.Publisher("/robot_{}/cmd_vel".format(robot_number), Twist, queue_size=1)

        rospy.Timer(rospy.Duration(0.1), self.mainCode)

    def callback(self, data, args):
        self.odom[args,0] = data.pose.pose.position.x
        self.odom[args,1] = data.pose.pose.position.y
        self.odom[args,2] = tf.transformations.euler_from_quaternion((data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w))[2]
        self.vel[args,0] = data.twist.twist.linear.x
        self.vel[args,1] = data.twist.twist.linear.y
        self.vel[args,2] = data.twist.twist.angular.z

    def mainCode(self, event):
        print("Hello!")

        # start behaviors
        behaviors_code = behaviors.Behaviors(self.number_of_robots, self.max_acc, self.max_vel)

        # get acceleration
        combined_acc = behaviors_code.seperation(self.odom, self.coeff_sep) + behaviors_code.cohesion(self.odom, self.coeff_coh) + behaviors_code.alignment(self.vel, self.coeff_ali)
        self.robotController(combined_acc)

    # the robot controller (might change later)
    def robotController(self, combined_acc):
        self.vel = combined_acc * self.dt + self.vel
        # publish velocities
        for robot_number in range(self.number_of_robots):
            # create twist message
            msg_vel = Twist()
            msg_vel.linear.x = self.vel[robot_number,0]
            msg_vel.linear.y = self.vel[robot_number,1]
            msg_vel.angular.z = self.vel[robot_number,2]
            # publish twist message
            self.pub[robot_number].publish(msg_vel)


if __name__ == '__main__':
    robot_communicator = RobotCommunicator()
    rospy.spin()