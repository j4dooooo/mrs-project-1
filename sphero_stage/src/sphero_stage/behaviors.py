import numpy as np

class Behaviors:

    def __init__(self, number_of_robots, max_acc, max_vel):
        
        # set maximum values
        self.max_acc = max_acc
        self.max_vel = max_vel
        self.number_of_robots = number_of_robots

    def seperation(self, odom, coeff_sep):
        # take advantage of array broadcasting for fast optimization
        i_points = j_points = odom
        # broadcast arrays
        i_points = i_points[np.newaxis,:,:]
        j_points = j_points[:,np.newaxis,:]
        # get position difference
        diff_i_j = i_points - j_points
        # remove 0 values
        # diff_i_j = diff_i_j[diff_i_j != 0.].reshape((self.number_of_robots,self.number_of_robots-1,2))
        # get norm
        norm_i_j = np.linalg.norm(diff_i_j, axis=2).reshape((self.number_of_robots,self.number_of_robots,1))
        norm_i_j = norm_i_j**2
        norm_i_j[norm_i_j == 0.] = 1
        # get final result
        fin_i_j = diff_i_j/norm_i_j
        seperation_acc = (-coeff_sep/self.number_of_robots)*np.sum(fin_i_j, axis=1)
        # temporary measure to preserve array dimensions
        # seperation_acc = np.concatenate((seperation_acc, np.zeros((self.number_of_robots,1))), axis=1)
        return seperation_acc

    def cohesion(self, odom, coeff_coh):
        # take advantage of array broadcasting for fast optimization
        i_points = j_points = odom
        # broadcast arrays
        i_points = i_points[np.newaxis,:,:]
        j_points = j_points[:,np.newaxis,:]
        # get position difference
        diff_i_j = i_points - j_points
        # get final result
        cohesion_acc = (coeff_coh/self.number_of_robots)*np.sum(diff_i_j, axis=1)
        # temporary measure to preserve array dimensions
        # cohesion_acc = np.concatenate((cohesion_acc, np.zeros((self.number_of_robots,1))), axis=1)
        return cohesion_acc

    def alignment(self, vel, coeff_ali):
        # take advantage of array broadcasting for fast optimization
        i_points = j_points = vel
        # broadcast arrays
        i_points = i_points[np.newaxis,:,:]
        j_points = j_points[:,np.newaxis,:]
        # get position difference
        diff_i_j = i_points - j_points
        # get final result
        alignment_acc = (coeff_ali/self.number_of_robots)*np.sum(diff_i_j, axis=1)
        # temporary measure to preserve array dimensions
        # alignment_acc = np.concatenate((alignment_acc, np.zeros((self.number_of_robots,1))), axis=1)
        return alignment_acc


# hypothetical number of robots
hyp_number_of_robots = 4

# the following is an example of good code optimization using array broadcasting
i_points = np.array([[1.,2.], [2.,3.], [4.,5.], [5.,6.]])
j_points = np.array([[1.,2.], [2.,3.], [4.,5.], [5.,6.]])

# broadcast arrays
i_points = i_points[np.newaxis,:,:]
j_points = j_points[:,np.newaxis,:]
diff_i_j = i_points - j_points
print(diff_i_j)

# remove 0 values
# diff_i_j = diff_i_j[diff_i_j != 0.].reshape((hyp_number_of_robots,hyp_number_of_robots-1,2))
# print(diff_i_j)

# get norm
norm_i_j = np.linalg.norm(diff_i_j, axis=2).reshape((hyp_number_of_robots,hyp_number_of_robots,1))
norm_i_j = norm_i_j**2
norm_i_j[norm_i_j == 0.] = 1
print(norm_i_j)

# final result
fin_i_j = diff_i_j/norm_i_j
print(fin_i_j)

# sum of coordinates
sum_i_j = np.sum(fin_i_j, axis=1)
print(sum_i_j)