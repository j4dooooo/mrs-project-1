#!/usr/bin/env python3

import numpy as np

import rospy
import tf

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker

import utils.fov as fov
import utils.visualization_functions as utilsFunc

from utils.behaviors import Behaviors
from utils.obstacle_avoidance import ObstacleAvoidance
from utils.roaming import Roaming

class SwarmRobots:

    def __init__(self, gridmap_topic, odom_topic, cmd_vel_topic, dominion):

        # initialize attributes
        self.num_robots = rospy.get_param("/num_of_robots")
        self.ns_prefix = rospy.get_param("/robot_name")

        self.odom = np.zeros((self.num_robots, 3))
        self.cmd_vel = np.zeros((self.num_robots, 3))
        
        # set more parameters
        self.coeff_sep = rospy.get_param("~coeff_sep")
        self.coeff_coh = rospy.get_param("~coeff_coh")
        self.coeff_ali = rospy.get_param("~coeff_ali")
        self.roaming_coeff = rospy.get_param("~roaming_coeff")
        self.avoidance_coeff = rospy.get_param("~avoidance_coeff")
        self.neighbors_radius = rospy.get_param("~neighbors_radius")

        self.sub_coeff_sep = rospy.Subscriber("/coeff_sep", Float32, self.callback_coeff_sep)
        self.sub_coeff_coh = rospy.Subscriber("/coeff_coh", Float32, self.callback_coeff_coh)
        self.sub_coeff_ali = rospy.Subscriber("/coeff_ali", Float32, self.callback_coeff_ali)
        self.sub_roaming_coeff = rospy.Subscriber("/roaming_coeff", Float32, self.callback_roaming_coeff)
        self.sub_avoidance_coeff = rospy.Subscriber("/avoidance_coeff", Float32, self.callback_avoidance_coeff)
        self.sub_neighbors_radius = rospy.Subscriber("/neighbors_radius", Float32, self.callback_neighbors_radius)

        # publishers to visualize results
        self.publish_neighbors = rospy.Publisher('pose_neighbors', Marker, queue_size=2)
        self.publish_flock = rospy.Publisher('pose_flock', Marker, queue_size=2)
        self.publish_agent = rospy.Publisher('pose_agent', Marker, queue_size=2)
        self.publish_goal = rospy.Publisher('pose_goal', Marker, queue_size=2)

        # roaming
        self.slowing_distance = rospy.get_param("~slowing_distance")
        self.slowing_speed = rospy.get_param("~slowing_speed")

        # set maximum values
        self.max_acc = rospy.get_param("~max_acc")
        self.max_vel = rospy.get_param("~max_vel")
        self.fov = rospy.get_param("~fov")
        self.max_see_ahead = rospy.get_param("~max_see_ahead")

        self.cmd_pub = [None] * self.num_robots
        self.odom_sub =  [None] * self.num_robots

        self.pose = np.zeros((self.num_robots, 3))
        self.vel = np.zeros((self.num_robots, 3))

        # THIS VALUE NEEDS TO BE CHANGED
        self.dt = 0.03

        self.oa = ObstacleAvoidance(r=self.num_robots, fov=self.fov, max_see_ahead=self.max_see_ahead)
        self.behaviors = Behaviors(self.num_robots, self.max_acc, self.max_vel)
        self.roaming = Roaming(self.max_vel, self.slowing_speed, self.slowing_distance)

        # roaming goal, set to None if not chosen                                                                 
        self.goal = None

        # avoid updating the map too often                                               
        self.last_map_time = rospy.Time.now()                       
        self.dominion = dominion                                        

        # controller parameters              
        self.v_max = 0.15             
        self.w_max = 0.3                

        # create subscribers and publishers
        for r in range(self.num_robots):
            self.cmd_pub[r] = rospy.Publisher(self.ns_prefix+"_{}".format(str(r)) + cmd_vel_topic, Twist, queue_size=10)
            self.odom_sub[r] = rospy.Subscriber(self.ns_prefix+"_{}".format(str(r))+odom_topic, Odometry, self.odom_callback, (r))

        self.gridmap_sub = rospy.Subscriber(gridmap_topic, OccupancyGrid, self.get_gridmap)
        self.move_goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.get_goal)
        self.roaming_acc = np.zeros((self.num_robots, 3))
        
        # velocity controller timer
        rospy.Timer(rospy.Duration(0.05), self.controller)

    def callback_coeff_sep(self, data):
        self.coeff_sep = data.data

    def callback_coeff_coh(self, data):
        self.coeff_coh = data.data

    def callback_coeff_ali(self, data):
        self.coeff_ali = data.data

    def callback_roaming_coeff(self, data):
        self.roaming_coeff = data.data

    def callback_avoidance_coeff(self, data):
        self.avoidance_coeff = data.data

    def callback_neighbors_radius(self, data):
        self.neighbors_radius = data.data
    
    # odometry callback gets the current robot pose and stores it in self.pose
    def odom_callback(self, odom, r):
        _, _, yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w])
        self.pose[r,0] = odom.pose.pose.position.x
        self.pose[r,1] = odom.pose.pose.position.y
        self.pose[r,2] = yaw
        
    # goal callback
    def get_goal(self, goal):
        if self.oa.there_is_map:
            print("New goal received: ({}, {})".format(goal.pose.position.x, goal.pose.position.y))
            self.goal = np.array([goal.pose.position.x, goal.pose.position.y])
            utilsFunc.publish_goal(self.goal, self.publish_goal)
             
    def get_gridmap(self, gridmap):
        if (gridmap.header.stamp - self.last_map_time).to_sec() > 0.5:
            self.last_map_time = gridmap.header.stamp
            
            # update map
            env = np.array(gridmap.data).reshape(gridmap.info.height, gridmap.info.width).T
            origin = [gridmap.info.origin.position.x, gridmap.info.origin.position.y]
            self.oa.set_map(env, gridmap.info.resolution, origin)

    # main velocity controller
    def controller(self, event):

        # get roaming velocities, NEED TO ADD PARAMETERS
        if self.goal is not None:
            self.roaming_acc = self.roaming.arrival(self.pose[:,0:2], self.goal, self.vel[:,0:2], self.slowing_speed, self.slowing_distance)
            self.roaming_acc = np.concatenate((self.roaming_acc, np.zeros((self.num_robots, 1))), axis=1)

        # get neighbors list for each robot, NEED TO ADD RADIUS PARAMETERS
        neighbors_list, vel_list, no_neighbors_list = fov.circle_fov(self.pose, self.vel, self.num_robots, self.neighbors_radius)

        # get the acceleration of the three main behaviors
        self.combined_acc = self.roaming_coeff*self.roaming_acc + self.behaviors.seperation(self.pose, neighbors_list, self.coeff_sep) + self.behaviors.cohesion(self.pose, neighbors_list, self.coeff_coh) + self.behaviors.alignment(self.vel, vel_list, self.coeff_ali)
        self.vel = self.combined_acc*self.dt + self.vel

        # obstacle avoidance
        self.vel_norm = self.dt*self.vel
        avoidance_force = self.oa.look_ahead(self.pose[:,0:2], self.vel_norm)
        self.vel[:,0:2] = self.vel[:,0:2] + self.avoidance_coeff*avoidance_force

        utilsFunc.publish_corners(no_neighbors_list[1], self.publish_flock, frame='map', color=(0.23, 0.33, 0.33, 1), scale=0.1)
        utilsFunc.publish_corners(neighbors_list[1], self.publish_neighbors, frame='map', color=(0, 1, 0, 1))
        utilsFunc.publish_agent(self.pose[1,:2], self.publish_agent, scale=self.neighbors_radius)

        for r in range(self.num_robots):
            v = self.vel[r,0]
            u = self.vel[r,1]
            w = self.vel[r,2]
            
            # publish velocity command
            self.__send_commnd__(v, u, w, r)
    
    def __send_commnd__(self, v, u, w, r):
        cmd = Twist()
        cmd.linear.x = np.clip(v, -self.v_max, self.v_max)
        cmd.linear.y = np.clip(u, -self.v_max, self.v_max)
        cmd.linear.z = 0
        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = np.clip(w, -self.w_max, self.w_max)
        self.cmd_pub[r].publish(cmd)

if __name__ == '__main__':
    rospy.init_node('swarm_robots')   
    node = SwarmRobots('/map', '/odom', '/cmd_vel', np.array([-15.0, 15.0]))
    rospy.spin()