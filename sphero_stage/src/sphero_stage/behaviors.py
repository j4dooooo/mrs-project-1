import numpy as np

class Behaviors:

    def __init__(self, max_acc, max_vel):
        
        # set maximum values
        self.max_acc = max_acc
        self.max_vel = max_vel

    def seperation(self):
        pass


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
diff_i_j = diff_i_j[diff_i_j != 0.].reshape((hyp_number_of_robots,hyp_number_of_robots-1,2))
print(diff_i_j)

# get norm
norm_i_j = np.linalg.norm(diff_i_j, axis=2).reshape((hyp_number_of_robots,hyp_number_of_robots-1,1))
norm_i_j = norm_i_j**2
print(norm_i_j)

# final result
fin_i_j = diff_i_j/norm_i_j
print(fin_i_j)

# sum of coordinates
sum_i_j = np.sum(fin_i_j, axis=1)
print(sum_i_j)