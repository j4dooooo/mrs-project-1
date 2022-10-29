#!/usr/bin/env python3

from traceback import print_tb
import rospy
import tf.transformations
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class RobotCommunicator:

    def __init__(self):
        self.number_of_robots = rospy.get_param("/num_of_robots")
        self.odom = np.zeros((self.number_of_robots, 3))
        self.vel = np.zeros((self.number_of_robots, 3))

        # set maximum values
        self.max_acc = 1
        self.max_vel = 2

        rospy.init_node("robot_communicator")

        self.pub = [None] * self.number_of_robots

        for robot_number in range(self.number_of_robots):
            rospy.Subscriber("/robot_{}/odom".format(robot_number), Odometry, self.callback, (robot_number))
            self.pub[robot_number] = rospy.Publisher("/robot_{}/cmd_vel".format(robot_number), Twist, queue_size=1)

        rospy.Timer(rospy.Duration(0.1), self.mainCode)

    def callback(self, data, args):
        self.odom[args,0] = data.pose.pose.position.x
        self.odom[args,1] = data.pose.pose.position.y
        self.odom[args,2] = tf.transformations.euler_from_quaternion((data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w))[2]
        print("Yes!")

    def mainCode(self, event):
        print("Hello!")


if __name__ == '__main__':
    robot_communicator = RobotCommunicator()
    rospy.spin()