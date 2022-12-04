import numpy as np

class Behaviors:

    def __init__(self, number_of_robots, max_acc, max_vel):
        
        # set maximum values
        self.max_acc = max_acc
        self.max_vel = max_vel
        self.number_of_robots = number_of_robots

    def seperation(self, odom, neighbors, coeff_sep):
        # initialize seperation acceleration
        seperation_acc = np.zeros((self.number_of_robots, 3))
        for robot in range(self.number_of_robots):
            # get number of neighbors
            num_of_neighbors = np.shape(neighbors[robot])[0]
            # get position difference
            pos_odom = odom[:,:2]
            diff_neighbor_robot = neighbors[robot][:,:2] - pos_odom[robot,:].reshape((1,2))
            # get norm
            norm_neighbor_robot = np.linalg.norm(diff_neighbor_robot, axis=1).reshape((num_of_neighbors, 1))
            norm_neighbor_robot = norm_neighbor_robot**2
            norm_neighbor_robot[norm_neighbor_robot == 0.] = 1
            # get final result
            fin_neighbor_robot = diff_neighbor_robot/norm_neighbor_robot
            # get acceleration
            seperation_acc[robot,:2] = (-coeff_sep/num_of_neighbors)*np.sum(fin_neighbor_robot, axis=0)

        return seperation_acc

    def cohesion(self, odom, neighbors, coeff_coh):
        # initialize cohesion acceleration
        cohesion_acc = np.zeros((self.number_of_robots, 3))
        for robot in range(self.number_of_robots):
            # get number of neighbors
            num_of_neighbors = np.shape(neighbors[robot])[0]
            # get position difference
            pos_odom = odom[:,:2]
            diff_neighbor_robot = neighbors[robot][:,:2] - pos_odom[robot,:].reshape((1,2))
            # get final result
            fin_neighbor_robot = diff_neighbor_robot
            # get acceleration
            cohesion_acc[robot,:2] = (coeff_coh/num_of_neighbors)*np.sum(fin_neighbor_robot, axis=0)

        return cohesion_acc

    def alignment(self, vel, neighbors_vel, coeff_ali):
        # initialize alignment acceleration
        alignment_acc = np.zeros((self.number_of_robots, 3))
        for robot in range(self.number_of_robots):
            # get number of neighbors
            num_of_neighbors = np.shape(neighbors_vel[robot])[0]
            # get position difference
            pos_vel = vel[:,:2]
            diff_neighbor_robot = neighbors_vel[robot][:,:2] - pos_vel[robot,:].reshape((1,2))
            # get final result
            fin_neighbor_robot = diff_neighbor_robot
            # get acceleration
            alignment_acc[robot,:2] = (coeff_ali/num_of_neighbors)*np.sum(fin_neighbor_robot, axis=0)

        return alignment_acc