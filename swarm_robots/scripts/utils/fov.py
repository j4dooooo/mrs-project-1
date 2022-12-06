import numpy as np

# use a circular field of view for the three main behaviors
def circle_fov(pose, vel, num_robots, radius):

    main_pose = secondary_pose = pose
    main_pose = main_pose[np.newaxis,:,:]
    secondary_pose = secondary_pose[:,np.newaxis,:]

    aux = main_pose - secondary_pose + secondary_pose

    main_vel = secondary_vel = vel
    main_vel = secondary_vel[np.newaxis,:,:]
    secondary_vel = secondary_vel[:,np.newaxis,:]

    aux_vel = main_vel - secondary_vel + secondary_vel

    distance = main_pose - secondary_pose
    vector_l = np.linalg.norm(distance, axis=2).reshape((num_robots, num_robots, 1))

    n_index = []
    no_n_index = []
    for agent in range(num_robots):
        n_ind = np.where(np.any(vector_l[agent] < radius, axis=1))
        no_n_ind = np.where(np.any(vector_l[agent] >= radius, axis=1))
        n_index.append(n_ind)
        no_n_index.append(no_n_ind)
    
    neighbors_list = []
    no_n_neighbors_list = []
    vel_list = []
    for agent in range(num_robots):
        neighbors = np.array(aux[agent])[n_index[agent]]
        no_neighbors = np.array(aux[agent])[no_n_index[agent]]
        vel = np.array(aux_vel[agent])[n_index[agent]]
        neighbors_list.append(neighbors)
        vel_list.append(vel)
        no_n_neighbors_list.append(no_neighbors)

    return neighbors_list, vel_list, no_n_neighbors_list